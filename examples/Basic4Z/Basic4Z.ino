/*
 * Basic4Z.ino
 *
 * Example sketch for the 4Z dry-contact input subdevice library.
 *
 * Target: any Arduino-framework board, including STM32 via STM32duino.
 * Designed to share the same RS485 bus as a 4R relay board (or other
 * Buspro-style subdevices) -- each device just needs a unique
 * Subnet ID + Device ID pair.
 *
 * Wiring assumed below (CHANGE TO MATCH YOUR BOARD):
 *   - Serial1 wired to the same RS485 transceiver/bus as other subdevices.
 *   - PA8 drives the transceiver's combined DE/RE pin.
 *   - PB0..PB3 read 4 dry-contact inputs (switch to GND, using the
 *     STM32's internal pull-up -- no external pull-up resistor needed).
 *
 * !!! REMINDER !!!
 * The wire frame format lives in BusproCore and is a PLACEHOLDER pending
 * verification against real captured HDL Buspro traffic. See BusproCore's
 * README and /docs/CAPTURE_TEMPLATE.md before using this on a bus shared
 * with genuine HDL devices. The INPUT_STATUS_PUSH/REQUEST/RESPONSE
 * op-codes used here are also unverified guesses, not researched values.
 */

#include <Input4Z.h>

// This device's own bus address. Must be different from 4R (and any
// other subdevice) on the same bus -- e.g. 4R used subnet 1 / device 12
// in its example, so this uses device 20 to avoid a collision.
constexpr uint8_t MY_SUBNET_ID = 1;
constexpr uint8_t MY_DEVICE_ID = 20;

constexpr int8_t DE_PIN = PA8;

// 4 input pins, in channel order [1,2,3,4].
const uint8_t kInputPins[INPUT4Z_CHANNEL_COUNT] = {PB0, PB1, PB2, PB3};

// Per-channel polarity -- set per your actual wiring. Example: channels
// 1-3 are normally-open push buttons, channel 4 is a normally-closed
// door/tamper switch.
const ContactPolarity kPolarity[INPUT4Z_CHANNEL_COUNT] = {
    ContactPolarity::NORMALLY_OPEN,
    ContactPolarity::NORMALLY_OPEN,
    ContactPolarity::NORMALLY_OPEN,
    ContactPolarity::NORMALLY_CLOSED,
};

Input4Z input4z(MY_SUBNET_ID, MY_DEVICE_ID, DE_PIN, kInputPins, kPolarity, /*debounceMs=*/30);

void setup() {
    // Must match the baud rate every other subdevice and the master use
    // on this shared bus.
    input4z.begin(&Serial1, 9600);
}

void loop() {
    input4z.poll(); // non-blocking; handles debounce, push-on-change, and bus polling
}
