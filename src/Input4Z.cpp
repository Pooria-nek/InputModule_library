#include "Input4Z.h"

namespace {
    void onStatusRequest(void* ctx, const BusproFrame& f, BusproTransport& t) {
        static_cast<Input4Z*>(ctx)->handleStatusRequest(f, t);
    }

    const BusproOpBinding kBindings[] = {
        { BusproOp::INPUT_STATUS_REQUEST, onStatusRequest },
    };
    constexpr uint8_t kBindingCount = sizeof(kBindings) / sizeof(kBindings[0]);
}

Input4Z::Input4Z(uint8_t mySubnetId, uint8_t myDeviceId, int8_t dePin,
                  const uint8_t inputPins[INPUT4Z_CHANNEL_COUNT],
                  const ContactPolarity polarity[INPUT4Z_CHANNEL_COUNT],
                  uint16_t debounceMs)
    : debounceMs_(debounceMs),
      transport_(mySubnetId, myDeviceId, dePin),
      device_(transport_) {
    for (uint8_t i = 0; i < INPUT4Z_CHANNEL_COUNT; ++i) {
        inputPins_[i] = inputPins[i];
        polarity_[i] = polarity[i];
    }
}

void Input4Z::begin(HardwareSerial* serial, uint32_t baud) {
    for (uint8_t i = 0; i < INPUT4Z_CHANNEL_COUNT; ++i) {
        // Dry contacts: use internal pull-up so an open (floating)
        // contact reads a defined HIGH level rather than noise.
        pinMode(inputPins_[i], INPUT_PULLUP);
        debouncedState_[i] = readRawActive(i);
        candidateState_[i] = debouncedState_[i];
        candidatePending_[i] = false;
    }
    transport_.begin(serial, baud);
    device_.configure(kBindings, kBindingCount, this);
}

bool Input4Z::readRawActive(uint8_t channel) const {
    // With INPUT_PULLUP: idle/open reads HIGH, closed-to-ground reads LOW.
    const int level = digitalRead(inputPins_[channel]);
    const bool physicallyClosed = (level == LOW);

    switch (polarity_[channel]) {
        case ContactPolarity::NORMALLY_OPEN:
            // Idle = open (not closed). Active = closed.
            return physicallyClosed;
        case ContactPolarity::NORMALLY_CLOSED:
            // Idle = closed. Active = open (not closed).
            return !physicallyClosed;
    }
    return false; // unreachable, silences compiler warning
}

void Input4Z::pollInputsAndDebounce() {
    const uint32_t now = millis();
    bool anyChanged = false;

    for (uint8_t i = 0; i < INPUT4Z_CHANNEL_COUNT; ++i) {
        const bool raw = readRawActive(i);

        if (raw != debouncedState_[i]) {
            if (!candidatePending_[i] || candidateState_[i] != raw) {
                // New (or newly-different) candidate -- restart the debounce timer.
                candidateState_[i] = raw;
                candidateSinceMs_[i] = now;
                candidatePending_[i] = true;
            } else if (now - candidateSinceMs_[i] >= debounceMs_) {
                // Candidate has been stable long enough -- accept it.
                debouncedState_[i] = raw;
                candidatePending_[i] = false;
                anyChanged = true;
            }
        } else {
            // Raw matches the already-accepted debounced state -- no
            // pending change for this channel.
            candidatePending_[i] = false;
        }
    }

    if (anyChanged) {
        sendStatusPush();
    }
}

bool Input4Z::getInputState(uint8_t channel) const {
    if (channel >= INPUT4Z_CHANNEL_COUNT) return false;
    return debouncedState_[channel];
}

void Input4Z::sendStatusPush() {
    // TODO_VERIFY_HDL: real HDL push destination convention. Placeholder
    // here broadcasts the push (dst = broadcast) so any master listening
    // on the bus picks it up, since dry-contact push events are typically
    // meant for whichever controller(s) care, not one fixed master
    // address. If your real master needs frames unicast to it specifically,
    // change this to a configured master subnet/device ID instead.
    sendStatusTo(BusproAddr::BROADCAST_SUBNET, BusproAddr::BROADCAST_DEVICE);
}

void Input4Z::sendStatusTo(uint8_t dstSubnet, uint8_t dstDevice) {
    BusproFrame frame;
    frame.srcSubnetId = transport_.subnetId();
    frame.srcDeviceId = transport_.deviceId();
    frame.dstSubnetId = dstSubnet;
    frame.dstDeviceId = dstDevice;
    frame.opCode = BusproOp::INPUT_STATUS_PUSH;
    for (uint8_t i = 0; i < INPUT4Z_CHANNEL_COUNT; ++i) {
        frame.payload[i] = debouncedState_[i] ? 0x01 : 0x00;
    }
    frame.payloadLen = INPUT4Z_CHANNEL_COUNT;
    transport_.send(frame);
}

void Input4Z::handleStatusRequest(const BusproFrame& frame, BusproTransport& transport) {
    BusproFrame resp;
    resp.srcSubnetId = transport.subnetId();
    resp.srcDeviceId = transport.deviceId();
    resp.dstSubnetId = frame.srcSubnetId;
    resp.dstDeviceId = frame.srcDeviceId;
    resp.opCode = BusproOp::INPUT_STATUS_RESPONSE;
    for (uint8_t i = 0; i < INPUT4Z_CHANNEL_COUNT; ++i) {
        resp.payload[i] = debouncedState_[i] ? 0x01 : 0x00;
    }
    resp.payloadLen = INPUT4Z_CHANNEL_COUNT;
    transport.send(resp);
}

void Input4Z::poll() {
    pollInputsAndDebounce();
    device_.poll(); // handles transport_.poll() + dispatch to handleStatusRequest
}
