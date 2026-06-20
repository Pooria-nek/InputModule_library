#pragma once
/*
 * Input4Z.h
 *
 * The "4Z" device: 4 dry-contact (switch) input channels, exposed as a
 * subdevice on a shared HDL-Buspro-style RS485 bus (same bus as 4R).
 *
 * Behavior:
 *  - Each channel is debounced with a simple time-based debounce (state
 *    must be stable for a configurable window before being accepted).
 *  - Each channel has a configurable idle/active polarity (since contacts
 *    can be wired normally-open or normally-closed per channel).
 *  - On a debounced state change, 4Z immediately sends an unsolicited
 *    INPUT_STATUS_PUSH frame on the bus (per your requirement: push on
 *    change, not poll-only).
 *  - Also responds to INPUT_STATUS_REQUEST with the current state of all
 *    4 channels, in case the master wants to poll/resync (e.g. after a
 *    master reboot, to avoid relying solely on missed push events).
 */

#include <Arduino.h>
#include "BusproFrame.h"
#include "BusproTransport.h"
#include "BusproDevice.h"

constexpr uint8_t INPUT4Z_CHANNEL_COUNT = 4;

// Per-channel idle polarity: whether the contact is normally-open (idle =
// open/HIGH with a pull-up) or normally-closed (idle = closed/LOW).
// "Active" means the contact is in its non-idle state (e.g. someone
// pressed a button wired normally-open).
enum class ContactPolarity : uint8_t {
    NORMALLY_OPEN,   // idle = HIGH (open), active = LOW (closed) w/ pull-up wiring
    NORMALLY_CLOSED  // idle = LOW (closed... wiring-dependent), active = HIGH
};

class Input4Z {
public:
    // mySubnetId/myDeviceId: this device's own bus address (1-254 each).
    // dePin: RS485 transceiver DE/RE control pin (-1 if not used).
    // inputPins: array of exactly INPUT4Z_CHANNEL_COUNT GPIO pins.
    // polarity: array of exactly INPUT4Z_CHANNEL_COUNT polarity settings
    //           (configurable per channel, per your requirement).
    // debounceMs: how long a raw input must be stable before its
    //             debounced state is updated and (if changed) pushed.
    Input4Z(uint8_t mySubnetId, uint8_t myDeviceId, int8_t dePin,
            const uint8_t inputPins[INPUT4Z_CHANNEL_COUNT],
            const ContactPolarity polarity[INPUT4Z_CHANNEL_COUNT],
            uint16_t debounceMs = 30);

    void begin(HardwareSerial* serial, uint32_t baud = 9600);

    // Call frequently from loop(); non-blocking. Reads GPIOs, debounces,
    // sends push frames on change, and handles incoming bus traffic
    // (status requests from the master).
    void poll();

    // Current debounced "active" state of a channel (true = active,
    // i.e. contact is in its non-idle state per configured polarity).
    bool getInputState(uint8_t channel /*0-3*/) const;

    // --- Internal handlers (public so the C-style dispatch table in
    //     BusproDevice can call them; not intended for direct use) ---
    void handleStatusRequest(const BusproFrame& frame, BusproTransport& transport);

private:
    void pollInputsAndDebounce();
    bool readRawActive(uint8_t channel) const;
    void sendStatusPush();
    void sendStatusTo(uint8_t dstSubnet, uint8_t dstDevice);

    uint8_t inputPins_[INPUT4Z_CHANNEL_COUNT];
    ContactPolarity polarity_[INPUT4Z_CHANNEL_COUNT];
    uint16_t debounceMs_;

    bool debouncedState_[INPUT4Z_CHANNEL_COUNT] = {false, false, false, false};
    bool candidateState_[INPUT4Z_CHANNEL_COUNT] = {false, false, false, false};
    uint32_t candidateSinceMs_[INPUT4Z_CHANNEL_COUNT] = {0, 0, 0, 0};
    bool candidatePending_[INPUT4Z_CHANNEL_COUNT] = {false, false, false, false};

    BusproTransport transport_;
    BusproDevice device_;
};
