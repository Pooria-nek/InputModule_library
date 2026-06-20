// Desktop compile/logic test for the 4Z library.
// Builds against test_stubs/Arduino.h instead of the real Arduino core.
// Validates debounce timing, per-channel polarity, push-on-change
// behavior, and status-request/response -- NOT real HDL wire compatibility
// (frame format is still a placeholder, shared with 4R via BusproCore).

#include <cassert>
#include <cstdio>
#include "Input4Z.h"

static const uint8_t kPins[INPUT4Z_CHANNEL_COUNT] = {20, 21, 22, 23};

static void testDebounceIgnoresShortGlitch() {
    const ContactPolarity polarity[INPUT4Z_CHANNEL_COUNT] = {
        ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_OPEN,
        ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_OPEN
    };
    Input4Z device(1, 20, -1, kPins, polarity, /*debounceMs=*/30);
    HardwareSerial fakeSerial;

    // All idle (open, HIGH) at boot.
    for (auto p : kPins) testSetPinLevel(p, HIGH);
    testMillisRef() = 0;
    device.begin(&fakeSerial, 9600);
    assert(device.getInputState(0) == false);

    // Brief glitch: contact closes for 10ms (less than 30ms debounce), then reopens.
    testSetPinLevel(kPins[0], LOW); // closed = active for NORMALLY_OPEN
    testAdvanceMillis(10);
    device.poll();
    assert(device.getInputState(0) == false); // not yet accepted

    testSetPinLevel(kPins[0], HIGH); // back to idle before debounce window elapses
    testAdvanceMillis(10);
    device.poll();
    assert(device.getInputState(0) == false); // glitch correctly ignored

    fakeSerial.takeTxLog(); // drain, nothing meaningful expected
    std::printf("[PASS] debounce ignores short glitch\n");
}

static void testDebounceAcceptsStableChangeAndPushes() {
    const ContactPolarity polarity[INPUT4Z_CHANNEL_COUNT] = {
        ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_OPEN,
        ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_OPEN
    };
    Input4Z device(1, 20, -1, kPins, polarity, /*debounceMs=*/30);
    HardwareSerial fakeSerial;
    for (auto p : kPins) testSetPinLevel(p, HIGH);
    testMillisRef() = 0;
    device.begin(&fakeSerial, 9600);

    testSetPinLevel(kPins[2], LOW); // channel 2 closes (active)
    testAdvanceMillis(10);
    device.poll(); // candidate timer starts now (t=10)
    assert(device.getInputState(2) == false); // still within debounce window

    testAdvanceMillis(35); // now t=45, i.e. 35ms since candidate started at t=10 -- past the 30ms window
    device.poll();
    assert(device.getInputState(2) == true); // accepted

    auto txLog = fakeSerial.takeTxLog();
    assert(!txLog.empty()); // a push frame should have been sent

    BusproFrameDecoder decoder;
    BusproFrame decoded;
    BusproDecodeResult result = BusproDecodeResult::NO_FRAME;
    for (auto b : txLog) result = decoder.feed(b, decoded);
    assert(result == BusproDecodeResult::FRAME_READY);
    assert(decoded.opCode == BusproOp::INPUT_STATUS_PUSH);
    assert(decoded.payload[2] == 0x01); // channel 2 active
    assert(decoded.payload[0] == 0x00 && decoded.payload[1] == 0x00 && decoded.payload[3] == 0x00);
    std::printf("[PASS] stable change accepted after debounce window and pushed\n");
}

static void testNormallyClosedPolarity() {
    const ContactPolarity polarity[INPUT4Z_CHANNEL_COUNT] = {
        ContactPolarity::NORMALLY_CLOSED, ContactPolarity::NORMALLY_OPEN,
        ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_OPEN
    };
    Input4Z device(1, 20, -1, kPins, polarity, /*debounceMs=*/10);
    HardwareSerial fakeSerial;

    // Channel 0 is normally-closed: idle = physically closed = LOW.
    testSetPinLevel(kPins[0], LOW);
    for (uint8_t i = 1; i < INPUT4Z_CHANNEL_COUNT; ++i) testSetPinLevel(kPins[i], HIGH);
    testMillisRef() = 0;
    device.begin(&fakeSerial, 9600);
    assert(device.getInputState(0) == false); // idle, not active

    // Contact opens (door opens, e.g.) -> physically open = HIGH -> active for NC.
    testSetPinLevel(kPins[0], HIGH);
    device.poll(); // candidate timer starts
    assert(device.getInputState(0) == false);
    testAdvanceMillis(15);
    device.poll(); // 15ms later, past the 10ms debounce window
    assert(device.getInputState(0) == true);
    std::printf("[PASS] normally-closed polarity correctly inverted\n");
}

static void testStatusRequestResponse() {
    const ContactPolarity polarity[INPUT4Z_CHANNEL_COUNT] = {
        ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_OPEN,
        ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_OPEN
    };
    Input4Z device(/*subnet=*/1, /*deviceId=*/20, -1, kPins, polarity, 10);
    HardwareSerial fakeSerial;
    for (auto p : kPins) testSetPinLevel(p, HIGH); // idle at boot
    testMillisRef() = 0;
    device.begin(&fakeSerial, 9600);

    for (auto p : kPins) testSetPinLevel(p, LOW); // all become active after boot
    device.poll(); // candidate timer starts
    testAdvanceMillis(15);
    device.poll(); // past debounce window, accepted + pushed
    fakeSerial.takeTxLog(); // discard the push frame, not what we're testing here

    BusproFrame req;
    req.srcSubnetId = 1;
    req.srcDeviceId = 200; // master
    req.dstSubnetId = 1;
    req.dstDeviceId = 20; // this 4Z device
    req.opCode = BusproOp::INPUT_STATUS_REQUEST;
    req.payloadLen = 0;

    uint8_t buf[64];
    uint16_t len = busproEncodeFrame(req, buf, sizeof(buf));
    fakeSerial.injectBytes(buf, len);
    device.poll();

    auto txLog = fakeSerial.takeTxLog();
    assert(!txLog.empty());

    BusproFrameDecoder decoder;
    BusproFrame resp;
    BusproDecodeResult result = BusproDecodeResult::NO_FRAME;
    for (auto b : txLog) result = decoder.feed(b, resp);
    assert(result == BusproDecodeResult::FRAME_READY);
    assert(resp.opCode == BusproOp::INPUT_STATUS_RESPONSE);
    assert(resp.dstSubnetId == 1 && resp.dstDeviceId == 200); // replied to requester
    for (uint8_t i = 0; i < INPUT4Z_CHANNEL_COUNT; ++i) {
        assert(resp.payload[i] == 0x01);
    }
    std::printf("[PASS] status request/response roundtrip\n");
}

static void testAddressFilteringIgnoresOtherDevice() {
    const ContactPolarity polarity[INPUT4Z_CHANNEL_COUNT] = {
        ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_OPEN,
        ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_OPEN
    };
    Input4Z device(1, 20, -1, kPins, polarity, 10);
    HardwareSerial fakeSerial;
    for (auto p : kPins) testSetPinLevel(p, HIGH);
    testMillisRef() = 0;
    device.begin(&fakeSerial, 9600);

    BusproFrame req;
    req.srcSubnetId = 1;
    req.srcDeviceId = 200;
    req.dstSubnetId = 1;
    req.dstDeviceId = 99; // NOT this device (which is 20)
    req.opCode = BusproOp::INPUT_STATUS_REQUEST;
    req.payloadLen = 0;

    uint8_t buf[64];
    uint16_t len = busproEncodeFrame(req, buf, sizeof(buf));
    fakeSerial.injectBytes(buf, len);
    device.poll();

    auto txLog = fakeSerial.takeTxLog();
    assert(txLog.empty()); // correctly ignored, no response sent
    std::printf("[PASS] status request to a different device address ignored\n");
}

int main() {
    testDebounceIgnoresShortGlitch();
    testDebounceAcceptsStableChangeAndPushes();
    testNormallyClosedPolarity();
    testStatusRequestResponse();
    testAddressFilteringIgnoresOtherDevice();
    std::printf("\nAll desktop logic tests passed.\n");
    std::printf("REMINDER: this validates library LOGIC only.\n");
    std::printf("Wire format (BusproCore/src/BusproFrame.cpp) is still UNVERIFIED against real HDL hardware.\n");
    return 0;
}
