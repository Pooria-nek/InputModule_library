#pragma once
/*
 * InputModule.h
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
#include <Wire.h>
#include "BusproFrame.h"
#include "BusproTransport.h"
#include "BusproDevice.h"
#include "MemoryCore.h"

#include "InputStorage.h"
#include "InputFunctionStorage.h"

constexpr uint8_t INPUT_CHANNEL_COUNT = 4;

constexpr uint16_t INPUT_TYPE = 121; //

// Per-channel idle polarity: whether the contact is normally-open (idle =
// open/HIGH with a pull-up) or normally-closed (idle = closed/LOW).
// "Active" means the contact is in its non-idle state (e.g. someone
// pressed a button wired normally-open).
enum class ContactPolarity : uint8_t
{
    NORMALLY_OPEN,  // idle = HIGH (open), active = LOW (closed) w/ pull-up wiring
    NORMALLY_CLOSED // idle = LOW (closed... wiring-dependent), active = HIGH
};

class InputModule
{
public:
    // mySubnetId/myDeviceId: this device's own bus address (1-254 each).
    // dePin: RS485 transceiver DE/RE control pin (-1 if not used).
    // inputPins: array of exactly INPUT_CHANNEL_COUNT GPIO pins.
    // polarity: array of exactly INPUT_CHANNEL_COUNT polarity settings
    //           (configurable per channel, per your requirement).
    // debounceMs: how long a raw input must be stable before its
    //             debounced state is updated and (if changed) pushed.
    InputModule(
        BusproTransport &bus,
        MemoryCore &flash,
        const uint8_t inputPins[INPUT_CHANNEL_COUNT]);

    bool begin();
    void update();
    bool firstime();

    void refactor();

    bool reMatch();

    bool init();

    bool syncValues();

    void process(const BusproFrame &frame);

    void sendSuccess(uint16_t opcode, uint16_t dst);
    void sendResponse(uint16_t opcode, uint16_t dst, const uint8_t *payload, uint8_t payloadLen);

    // Current debounced "active" state of a channel (true = active,
    // i.e. contact is in its non-idle state per configured polarity).
    bool getInputState(uint8_t channel /*0-3*/) const;

    bool getPreviousInputState(uint8_t channel) const;
    bool hasInputChanged(uint8_t channel) const;
    bool isRisingEdge(uint8_t channel) const;
    bool isFallingEdge(uint8_t channel) const;

    void handleInputEdge(uint8_t channel, bool state);

    // --- Internal handlers (public so the C-style dispatch table in
    //     BusproDevice can call them; not intended for direct use) ---
    void handleStatusRequest(const BusproFrame &frame, BusproTransport &transport);

    enum class SwitchType : uint8_t
    {
        Mechanical = 0x00, // 1-99 opration ON + 1-99 opration OFF + delay ON OFF

        SingleON = 0x01,    // 1 opration just turn on
        SingleOFF = 0x02,   // 1 opration just turn off
        SingleONOFF = 0x03, // 1 opration do both on/off

        CombinationON = 0x04,    // 1-99 opration just turn on
        CombinationOFF = 0x05,   // 1-99 opration just turn off
        CombinationONOFF = 0x06, // 1-99 opration do both on/off

        MultiFunction = 0x07, // 1-99 opration

        ParallelSwitch = 0x08 // 1 opration
    };

    enum class OperationType : uint8_t
    {
        Scene = 0x55,                // param 1 -> Zone no        | param 2 -> Scene no      | param 3 -> ---          | param 4 -> ---
        Sequence = 0x56,             // param 1 -> Zone no        | param 2 -> Sequence      | param 3 -> ---          | param 4 -> ---
        UniversalSwitch = 0x58,      // param 1 -> Switch no      | param 2 -> Switch Statue | param 3 -> ---          | param 4 -> ---
        SingleChannelControl = 0x59, // param 1 -> Channel no     | param 2 -> Intensity     | param 3 -> Running time | param 4 -> ---
        CurtainSwitch = 0x60,        // param 1 -> Curtain no     | param 2 -> Switch Status | param 3 -> ---          | param 4 -> ---
        GPRSControl = 0x61,          // param 1 -> Message        | param 2 -> no            | param 3 -> ---          | param 4 -> ---
        PanelControl = 0x62,         // param 1 -> Function       | param 2 -> par1          | param 3 -> par2         | param 4 -> ---
        BroadcastScene = 0x63,       // param 1 -> All Zone       | param 2 -> Scene no      | param 3 -> ---          | param 4 -> ---
        BroadcastChannel = 0x64,     // param 1 -> All Channel    | param 2 -> Channel no    | param 3 -> Running time | param 4 -> ---
        SecurityModule = 0x65,       // param 1 -> Zone no        | param 2 -> Mode          | param 3 -> ---          | param 4 -> ---
        MusicControl = 0x67,         // param 1 -> par1           | param 2 -> par2          | param 3 -> par3         | param 4 -> ---
    };

private:
    static constexpr const char *FACTORY_NAME = "Zeller 4Z";
    static constexpr const char *HARDWARE_VERSION = "";
    static constexpr const char *SOFTWARE_VERSION = "v0.6.0b " __DATE__;

    static constexpr uint32_t flashAddress = 60; // its the refrens address of data on memoryflash

    using InfoStorageLayout = InputStorage<
        flashAddress, // Main sector index / reference address
        INPUT_CHANNEL_COUNT>;

    using InputFunctionLayout = InputFunctionStorage<
        flashAddress + 2,
        INPUT_CHANNEL_COUNT>;

    BusproTransport &bus_;
    MemoryCore &flash_;

    uint16_t deviceAddress_ = 0x010B;
    uint8_t fuid_[12];
    uint8_t uid_[12];
    uint16_t devType_ = INPUT_TYPE; // 4Z device type per HDL spec

    bool inputEnable_[INPUT_CHANNEL_COUNT] = {false};
    uint8_t inputOnDelay_[INPUT_CHANNEL_COUNT] = {0};
    uint8_t inputOffDelay_[INPUT_CHANNEL_COUNT] = {0};
    uint8_t inputType_[INPUT_CHANNEL_COUNT] = {0};

    uint32_t statusReportIntervalMs_ = 1000;
    uint32_t lastStatusReportMs_ = 0;

    bool debouncedState_[INPUT_CHANNEL_COUNT] = {false};
    bool previousState_[INPUT_CHANNEL_COUNT] = {false};
    bool candidateState_[INPUT_CHANNEL_COUNT] = {false};
    uint32_t candidateSinceMs_[INPUT_CHANNEL_COUNT] = {0};
    bool candidatePending_[INPUT_CHANNEL_COUNT] = {false};

    void pollInputsAndDebounce();
    bool readRawActive(uint8_t channel) const;
    void sendStatusPush();
    void sendStatusTo(uint8_t dstSubnet, uint8_t dstDevice);

    uint8_t inputPins_[INPUT_CHANNEL_COUNT];
    ContactPolarity polarity_[INPUT_CHANNEL_COUNT];
    uint16_t debounceMs_;

    /////////////////////////////// UNIVERSAL REQUEST ///////////////////////////////

    void handleReadFirmware(const BusproFrame &frame);
    void handleReadHardware(const BusproFrame &frame);
    void handleFindDevice(const BusproFrame &frame);

    void handleSearchDevice(const BusproFrame &frame);

    void handleReadMacaddress(const BusproFrame &frame);
    void handleModifyMacaddress(const BusproFrame &frame);

    void handleReadDeviceRemark(const BusproFrame &frame);
    void handleModifyDeviceRemark(const BusproFrame &frame);

    ////////////////////////////////// DRY CONTACT //////////////////////////////////

    void handleReadChannelEnable(const BusproFrame &frame);
    void handleModifyChannelEnable(const BusproFrame &frame);

    void handleReadInfoEnable(const BusproFrame &frame);
    void handleModifyInfoEnable(const BusproFrame &frame);

    void handleReadChannelType(const BusproFrame &frame);
    void handleModifyChannelType(const BusproFrame &frame);

    void handleReadChannelDim(const BusproFrame &frame);
    void handleModifyChannelDim(const BusproFrame &frame);

    void handleReadChannelLock(const BusproFrame &frame);
    void handleModifyChannelLock(const BusproFrame &frame);

    void handleReadMinDimmer(const BusproFrame &frame);
    void handleModifyMinDimmer(const BusproFrame &frame);

    void handleReadChannelRemark(const BusproFrame &frame);
    void handleModifyChannelRemark(const BusproFrame &frame);

    void handleReadChannelDelay(const BusproFrame &frame);
    void handleModifyChannelDelay(const BusproFrame &frame);

    void handleReadChannelFunctions(const BusproFrame &frame);
    void handleModifyChannelFunctions(const BusproFrame &frame);
};
