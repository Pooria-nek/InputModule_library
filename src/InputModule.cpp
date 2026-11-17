#include "InputModule.h"

InputModule::InputModule(
    BusproTransport &bus,
    MemoryCore &flash,
    const uint8_t inputPins[INPUT_CHANNEL_COUNT])
    : bus_(bus), flash_(flash), debounceMs_(100)
{
    for (uint8_t i = 0; i < INPUT_CHANNEL_COUNT; ++i)
    {
        inputPins_[i] = inputPins[i];
        polarity_[i] = ContactPolarity::NORMALLY_OPEN;
    }
}

bool InputModule::begin()
{
    for (uint8_t i = 0; i < INPUT_CHANNEL_COUNT; ++i)
    {
        pinMode(inputPins_[i], INPUT_PULLUP);

        const bool initialState = readRawActive(i);

        debouncedState_[i] = initialState;
        previousState_[i] = initialState;

        candidateState_[i] = initialState;
        candidateSinceMs_[i] = 0;
        candidatePending_[i] = false;
    }

    if (firstime())
    {
        init();
    }
    reMatch();

    return syncValues();
}

void InputModule::update()
{
    pollInputsAndDebounce();
}

bool InputModule::firstime()
{
    uint8_t payload[16];
    flash_.read(InfoStorageLayout::deviceAddress(), payload, sizeof(payload));

    for (uint8_t i = 0; i < sizeof(payload); i++)
    {
        if (payload[i] != 0xFF)
            return false;
    }

    return true;
}

void InputModule::refactor()
{
    uint8_t payload[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    flash_.update(InfoStorageLayout::deviceAddress(), payload, sizeof(payload));

    while (true)
    {
        /* code */
    }
}

bool InputModule::reMatch()
{
    char storedVersion[20] = {};

    auto address = InfoStorageLayout::softwareVersionAddress();

    flash_.read(address, storedVersion, sizeof(storedVersion));

    if (strncmp(storedVersion, SOFTWARE_VERSION, sizeof(storedVersion)) == 0)
        return false;

    char newVersion[20] = {};
    strncpy(newVersion, SOFTWARE_VERSION, sizeof(newVersion) - 1);

    flash_.update(address, SOFTWARE_VERSION, sizeof(storedVersion));

    return true;
}

bool InputModule::init()
{
    uint8_t payload[2] = {static_cast<uint8_t>(deviceAddress_ >> 8), static_cast<uint8_t>(deviceAddress_ & 0xFF)};
    flash_.update(InfoStorageLayout::deviceAddress(), payload, sizeof(payload));

    uint8_t payload1[2] = {static_cast<uint8_t>(devType_ >> 8), static_cast<uint8_t>(devType_ & 0xFF)};
    flash_.update(InfoStorageLayout::deviceTypeAddress(), payload1, sizeof(payload1));

    // // MCU UID
    // const uint8_t *uid = mcu::getMcuUID();
    // flash_.update(InfoStorageLayout::deviceMacAddress(), uid, sizeof(uid));

    flash_.update(InfoStorageLayout::deviceRemarkAddress(), FACTORY_NAME, 20);
    flash_.update(InfoStorageLayout::hardwareVersionAddress(), HARDWARE_VERSION, 30);
    flash_.update(InfoStorageLayout::softwareVersionAddress(), SOFTWARE_VERSION, 20);

    char remark[20];
    for (size_t i = 1; i <= INPUT_CHANNEL_COUNT; i++)
    {
        snprintf(remark, sizeof(remark), "INPUT %d", static_cast<unsigned>(i));
        flash_.update(InfoStorageLayout::channelRemarkAddress(i), remark, sizeof(remark));

        uint8_t enable = 0x01;

        flash_.update(InfoStorageLayout::channelEnableAddress(i), &enable, 1);
        flash_.update(InfoStorageLayout::channelOnDelayAddress(i), 0, 1);
        flash_.update(InfoStorageLayout::channelOffDelayAddress(i), 0, 1);

        uint8_t deftype = static_cast<uint8_t>(SwitchType::Mechanical);
        flash_.update(InfoStorageLayout::channelTypeAddress(i), &deftype, 1);
    }

    return true;
}

bool InputModule::syncValues()
{
    uint8_t payload[2] = {};
    flash_.read(InfoStorageLayout::deviceAddress(), payload, sizeof(payload));
    deviceAddress_ = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];

    // flash_.read(InfoStorageLayout::deviceMacAddress(), uid_);
    flash_.read(InfoStorageLayout::channelEnableAddress(), inputEnable_, sizeof(inputEnable_));
    flash_.read(InfoStorageLayout::channelOnDelayAddress(), inputOnDelay_, sizeof(inputOnDelay_));
    flash_.read(InfoStorageLayout::channelOffDelayAddress(), inputOffDelay_, sizeof(inputOffDelay_));
    flash_.read(InfoStorageLayout::channelTypeAddress(), inputType_, sizeof(inputType_));

    return true;
}

void InputModule::process(const BusproFrame &frame)
{
    // if (frame.devType != devType_)
    //     return;

    if (frame.srcAddress == deviceAddress_)
    {
        return;
    }

    if (frame.dstAddress == 0xFFFF) // universal comands
    {
        switch (frame.opCode)
        {
        case BusproOp::DEVICE_SEARCH_HDL.req():
            handleSearchDevice(frame);
            break;

        case BusproOp::DEVICE_REMARK.readReq():
            handleReadDeviceRemark(frame);
            break;
        }
    }
    else if (frame.dstAddress == deviceAddress_) // my comands
    {
        switch (frame.opCode)
        {
            /////////////////////////////// UNIVERSAL REQUEST ///////////////////////////////

        case BusproOp::DEVICE_FIRMWARE.req():
            handleReadFirmware(frame);
            break;

        case BusproOp::DEVICE_HARDWARE.req():
            handleReadHardware(frame);
            break;

        case BusproOp::DEVICE_FINDIT.req():
            handleFindDevice(frame);
            break;

        case BusproOp::DEVICE_SEARCH_HDL.req():
            handleSearchDevice(frame);
            break;

        case BusproOp::DEVICE_MAC_ADDRESS.readReq():
            handleReadMacaddress(frame);
            break;

        case BusproOp::DEVICE_MAC_ADDRESS.writeReq():
            handleModifyMacaddress(frame);
            break;

        case BusproOp::DEVICE_REMARK.readReq():
            handleReadDeviceRemark(frame);
            break;

        case BusproOp::DEVICE_REMARK.writeReq():
            handleModifyDeviceRemark(frame);
            break;

            ////////////////////////////////// DRY CONTACT //////////////////////////////////

        case BusproOp::Input::CHANNEL_ENABLE.readReq():
            handleReadChannelEnable(frame);
            break;

        case BusproOp::Input::CHANNEL_ENABLE.writeReq():
            handleModifyChannelEnable(frame);
            break;

        case BusproOp::Input::READ_INFO.readReq():
            handleReadInfoEnable(frame);
            break;

        case BusproOp::Input::READ_INFO.writeReq():
            handleModifyInfoEnable(frame);
            break;

        case BusproOp::Input::CHANNEL_TYPE.readReq():
            handleReadChannelType(frame);
            break;

        case BusproOp::Input::CHANNEL_TYPE.writeReq():
            handleModifyChannelType(frame);
            break;

        case BusproOp::Input::CHANNEL_DIM_MODE.readReq():
            handleReadChannelDim(frame);
            break;

        case BusproOp::Input::CHANNEL_DIM_MODE.writeReq():
            handleModifyChannelDim(frame);
            break;

        case BusproOp::Input::CHANNEL_ENABLE_LOCK.readReq():
            handleReadChannelLock(frame);
            break;

        case BusproOp::Input::CHANNEL_ENABLE_LOCK.writeReq():
            handleModifyChannelLock(frame);
            break;

        case BusproOp::Input::MIN_DIM.readReq():
            handleReadMinDimmer(frame);
            break;

        case BusproOp::Input::MIN_DIM.writeReq():
            handleModifyMinDimmer(frame);
            break;

        case BusproOp::Input::CHANNEL_REMARK_R.req():
            handleReadChannelRemark(frame);
            break;

        case BusproOp::Input::CHANNEL_REMARK_W.req():
            handleModifyChannelRemark(frame);
            break;

        case BusproOp::Input::CHANNEL_DELAY_R.req():
            handleReadChannelDelay(frame);
            break;

        case BusproOp::Input::CHANNEL_DELAY_W.req():
            handleModifyChannelDelay(frame);
            break;

        case BusproOp::Input::CHANNEL_FUNCTION.readReq():
            handleReadChannelFunctions(frame);
            break;

        case BusproOp::Input::CHANNEL_FUNCTION.writeReq():
            handleModifyChannelFunctions(frame);
            break;

            ///////////////////////////////// INPUT CONTROLL ////////////////////////////////

        default:
            break;
        }

        return;
    }
}

// for sending 1 byte success
void InputModule::sendSuccess(uint16_t opcode, uint16_t dst)
{
    uint8_t payload[1] = {BusproOp::SUCCESS};
    sendResponse(opcode, dst, payload, sizeof(payload));
}

void InputModule::sendResponse(uint16_t opcode, uint16_t dst, const uint8_t *payload, uint8_t payloadLen)
{
    bus_.send(deviceAddress_, devType_, opcode, dst, payload, payloadLen);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// DEVICE HANDLERS ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void InputModule::handleReadChannelEnable(const BusproFrame &frame)
{
    if (frame.payloadLen != 0)
        return;

    uint8_t payload[INPUT_CHANNEL_COUNT] = {};

    flash_.read(InfoStorageLayout::channelEnableAddress(), payload, INPUT_CHANNEL_COUNT);

    sendResponse(BusproOp::Input::CHANNEL_ENABLE.readResp(), frame.srcAddress, payload, sizeof(payload));
}
void InputModule::handleModifyChannelEnable(const BusproFrame &frame)
{
    if (frame.payloadLen != INPUT_CHANNEL_COUNT)
        return;

    flash_.update(InfoStorageLayout::channelEnableAddress(), frame.payload, INPUT_CHANNEL_COUNT);

    syncValues();

    sendSuccess(BusproOp::Input::CHANNEL_ENABLE.writeResp(), frame.srcAddress);
}

void InputModule::handleReadInfoEnable(const BusproFrame &frame)
{
    if (frame.payloadLen != 0)
        return;

    const uint8_t payload[1] = {BusproOp::SUCCESS};

    sendResponse(BusproOp::Input::READ_INFO.readResp(), frame.srcAddress, payload, sizeof(payload));
}
void InputModule::handleModifyInfoEnable(const BusproFrame &frame)
{
    if (frame.payloadLen != 4)
        return;

    sendSuccess(BusproOp::Input::READ_INFO.writeResp(), frame.srcAddress);
}

void InputModule::handleReadChannelType(const BusproFrame &frame)
{
    if (frame.payloadLen != 0)
        return;

    uint8_t payload[INPUT_CHANNEL_COUNT + 1] = {BusproOp::SUCCESS};

    flash_.read(InfoStorageLayout::channelTypeAddress(), payload + 1, INPUT_CHANNEL_COUNT);

    sendResponse(BusproOp::Input::CHANNEL_TYPE.readResp(), frame.srcAddress, payload, sizeof(payload));
}
void InputModule::handleModifyChannelType(const BusproFrame &frame)
{
    if (frame.payloadLen != INPUT_CHANNEL_COUNT)
        return;

    flash_.update(InfoStorageLayout::channelTypeAddress(), frame.payload, INPUT_CHANNEL_COUNT);

    syncValues();

    sendSuccess(BusproOp::Input::CHANNEL_TYPE.writeResp(), frame.srcAddress);
}

void InputModule::handleReadChannelDim(const BusproFrame &frame)
{
    if (frame.payloadLen != 0)
        return;

    uint8_t payload[INPUT_CHANNEL_COUNT + 1] = {INPUT_CHANNEL_COUNT};

    flash_.read(InfoStorageLayout::channelDimmingAddress(), payload + 1, INPUT_CHANNEL_COUNT);

    sendResponse(BusproOp::Input::CHANNEL_DIM_MODE.readResp(), frame.srcAddress, payload, sizeof(payload));
}
void InputModule::handleModifyChannelDim(const BusproFrame &frame)
{
    if (frame.payloadLen != INPUT_CHANNEL_COUNT)
        return;

    flash_.update(InfoStorageLayout::channelDimmingAddress(), frame.payload, INPUT_CHANNEL_COUNT);

    sendSuccess(BusproOp::Input::CHANNEL_DIM_MODE.writeResp(), frame.srcAddress);
}

void InputModule::handleReadChannelLock(const BusproFrame &frame)
{
    if (frame.payloadLen != 0)
        return;

    const uint8_t payload[INPUT_CHANNEL_COUNT + 1] = {BusproOp::SUCCESS};

    sendResponse(BusproOp::Input::CHANNEL_ENABLE_LOCK.readResp(), frame.srcAddress, payload, sizeof(payload));
}
void InputModule::handleModifyChannelLock(const BusproFrame &frame)
{
    if (frame.payloadLen != 4)
        return;

    sendSuccess(BusproOp::Input::CHANNEL_ENABLE_LOCK.writeResp(), frame.srcAddress);
}

void InputModule::handleReadMinDimmer(const BusproFrame &frame)
{
    if (frame.payloadLen != 0)
        return;

    const uint8_t payload[2] = {BusproOp::SUCCESS, 0x00};

    sendResponse(BusproOp::Input::MIN_DIM.readResp(), frame.srcAddress, payload, sizeof(payload));
}
void InputModule::handleModifyMinDimmer(const BusproFrame &frame)
{
    if (frame.payloadLen != 1)
        return;

    sendSuccess(BusproOp::Input::MIN_DIM.writeResp(), frame.srcAddress);
}

void InputModule::handleReadChannelRemark(const BusproFrame &frame)
{
    if (frame.payloadLen != 2)
        return;

    uint8_t payload[22];

    payload[0] = BusproOp::SUCCESS;
    payload[1] = frame.payload[0];
    payload[2] = frame.payload[1];

    flash_.read(InfoStorageLayout::channelRemarkAddress(frame.payload[0] + 1), payload + 3, 20);

    sendResponse(BusproOp::Input::CHANNEL_REMARK_R.resp(), frame.srcAddress, payload, sizeof(payload));
}
void InputModule::handleModifyChannelRemark(const BusproFrame &frame)
{
    if (frame.payloadLen != 22)
        return;

    flash_.update(InfoStorageLayout::channelRemarkAddress(frame.payload[0] + 1), frame.payload + 2, 20);

    const uint8_t payload[3] = {BusproOp::SUCCESS, 0, 0};

    sendResponse(BusproOp::Input::CHANNEL_REMARK_W.resp(), frame.srcAddress, payload, sizeof(payload));
}

void InputModule::handleReadChannelDelay(const BusproFrame &frame)
{
    if (frame.payloadLen != 2)
        return;

    // uint d = frame.payload[1];

    uint8_t payload[5] = {INPUT_CHANNEL_COUNT};

    // if (d == 1)
    // {
    //     flash_.read(InfoStorageLayout::channelOnDelayAddress(), payload + 1, INPUT_CHANNEL_COUNT);
    // }
    // else
    // {
    //     flash_.read(InfoStorageLayout::channelOffDelayAddress(), payload + 1, INPUT_CHANNEL_COUNT);
    // }

    sendResponse(BusproOp::Input::CHANNEL_DELAY_R.resp(), frame.srcAddress, payload, sizeof(payload));
}
void InputModule::handleModifyChannelDelay(const BusproFrame &frame)
{
    if (frame.payloadLen != 4)
        return;

    // uint d = frame.payload[1];

    // if (d == 1)
    // {
    //     flash_.update(InfoStorageLayout::channelOnDelayAddress(), frame.payload + 4, INPUT_CHANNEL_COUNT);
    // }
    // else
    // {
    //     flash_.update(InfoStorageLayout::channelOffDelayAddress(), frame.payload + 4, INPUT_CHANNEL_COUNT);
    // }

    const uint8_t payload[1] = {BusproOp::SUCCESS};

    sendResponse(BusproOp::Input::CHANNEL_DELAY_W.resp(), frame.srcAddress, payload, sizeof(payload));
}


void InputModule::handleReadChannelFunctions(const BusproFrame &frame)
{
    if (frame.payloadLen != 3)
        return;

    const uint8_t input = frame.payload[0];
    const uint8_t rangeValue = frame.payload[1];
    const uint8_t function = frame.payload[2];

    if (input >= INPUT_CHANNEL_COUNT || rangeValue >= InputFunctionLayout::RANGE_COUNT ||
        function >= InputFunctionLayout::FUNCTIONS_PER_RANGE)
        return;

    const auto range =
        static_cast<InputFunctionLayout::Range>(rangeValue);

    const uint32_t address = InputFunctionLayout::functionAddress(
        input + 1, range, function + 1);

    if (address == UINT32_MAX)
        return;

    uint8_t payload[12] = {
        BusproOp::SUCCESS,
        input,
        rangeValue,
        function
    };

    flash_.read(address, payload + 4, InputFunctionLayout::FUNCTION_SIZE);

    sendResponse(
        BusproOp::Input::CHANNEL_FUNCTION.readResp(),
        frame.srcAddress,
        payload,
        sizeof(payload));
}
void InputModule::handleModifyChannelFunctions(const BusproFrame &frame)
{
    if (frame.payloadLen != 10)
        return;

    const uint8_t input = frame.payload[0];
    const uint8_t rangeValue = frame.payload[1];
    const uint8_t function = frame.payload[2];

    if (input >= INPUT_CHANNEL_COUNT || rangeValue >= InputFunctionLayout::RANGE_COUNT ||
        function >= InputFunctionLayout::FUNCTIONS_PER_RANGE)
        return;

    const auto range =
        static_cast<InputFunctionLayout::Range>(rangeValue);

    const uint32_t address = InputFunctionLayout::functionAddress(
        input + 1, range, function + 1);

    if (address == UINT32_MAX)
        return;

    flash_.update(
        address,
        frame.payload + 3,
        InputFunctionLayout::FUNCTION_SIZE);

    const uint8_t payload[4] = {
        BusproOp::SUCCESS,
        input,
        rangeValue,
        function
    };

    sendResponse(
        BusproOp::Input::CHANNEL_FUNCTION.writeResp(),
        frame.srcAddress,
        payload,
        sizeof(payload));
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// UNIVERSAL REQUEST ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void InputModule::handleReadFirmware(const BusproFrame &frame)
{
    if (frame.payloadLen != 0)
        return;

    uint8_t payload[20] = {0x00};

    flash_.read(InfoStorageLayout::softwareVersionAddress(), payload, sizeof(payload));

    sendResponse(BusproOp::DEVICE_FIRMWARE.resp(), frame.srcAddress, payload, sizeof(payload));
}

void InputModule::handleReadHardware(const BusproFrame &frame)
{
    if (frame.payloadLen != 0)
        return;

    uint8_t payload[30] = {};

    flash_.read(InfoStorageLayout::hardwareVersionAddress(), payload, sizeof(payload));

    sendResponse(BusproOp::DEVICE_HARDWARE.resp(), frame.srcAddress, payload, sizeof(payload));
}

void InputModule::handleFindDevice(const BusproFrame &frame)
{
    if (frame.payloadLen != 0)
        return;

    uint8_t payload[8] = {0x00};

    sendResponse(BusproOp::DEVICE_FINDIT.resp(), frame.srcAddress, payload, sizeof(payload));
}

void InputModule::handleSearchDevice(const BusproFrame &frame)
{
    if (frame.payloadLen != 4 || frame.payloadLen != 2)
        return;

    uint8_t payload[30 + frame.payloadLen] = {0x00};

    payload[0] = frame.payload[0];
    payload[1] = frame.payload[1];

    if (frame.payloadLen == 4)
    {
        payload[2] = frame.payload[2];
        payload[3] = frame.payload[3];
    }

    flash_.read(InfoStorageLayout::deviceRemarkAddress(), payload + frame.payloadLen, 20);

    sendResponse(BusproOp::DEVICE_SEARCH_HDL.resp(), frame.srcAddress, payload, sizeof(payload));
}

void InputModule::handleReadMacaddress(const BusproFrame &frame)
{
    if (frame.payloadLen != 0)
        return;

    uint8_t payload[12] = {0x00};

    // mcu::copyArray(mcu::getMcuUID(), payload, 8);

    sendResponse(BusproOp::DEVICE_MAC_ADDRESS.readResp(), frame.srcAddress, payload, sizeof(payload));
}
void InputModule::handleModifyMacaddress(const BusproFrame &frame)
{
    if (frame.payloadLen != 10)
        return;

    // if (!mcu::bufferEquals(frame.payload, mcu::getMcuUID()))
    //     return;

    uint8_t address[2] = {frame.payload[8], frame.payload[9]};

    flash_.update(InfoStorageLayout::deviceAddress(), address, 2);
    syncValues();

    sendSuccess(BusproOp::DEVICE_MAC_ADDRESS.writeResp(), frame.srcAddress);
}

void InputModule::handleReadDeviceRemark(const BusproFrame &frame)
{
    static uint32_t lastResponseTime = 0;
    static bool hasResponded = false;

    if (frame.payloadLen != 0)
        return;

    uint32_t now = millis();

    // 10-second cooldown after the previous response.
    if (hasResponded &&
        (uint32_t)(now - lastResponseTime) < 10000)
        return;

    // Random delay to reduce simultaneous responses.
    delay(random(100, 2001));

    // Check cooldown again in case time passed during the delay.
    now = millis();

    if (hasResponded &&
        (uint32_t)(now - lastResponseTime) < 10000)
        return;

    uint8_t payload[20];

    flash_.read(InfoStorageLayout::deviceRemarkAddress(), payload, sizeof(payload));

    sendResponse(BusproOp::DEVICE_REMARK.readResp(), 0xFFFF, payload, sizeof(payload));
}

void InputModule::handleModifyDeviceRemark(const BusproFrame &frame)
{
    if (frame.payloadLen != 20)
        return;

    if (strncmp(reinterpret_cast<const char *>(frame.payload), "RESET", 5) == 0)
    {
        refactor();
    }
    else
    {
        flash_.update(InfoStorageLayout::deviceRemarkAddress(), frame.payload, frame.payloadLen);
    }

    sendSuccess(BusproOp::DEVICE_REMARK.writeResp(), frame.srcAddress);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool InputModule::readRawActive(uint8_t channel) const
{
    if (channel >= INPUT_CHANNEL_COUNT || !inputEnable_[channel])
        return false;

    // With INPUT_PULLUP: idle/open reads HIGH, closed-to-ground reads LOW.
    const int level = digitalRead(inputPins_[channel]);
    const bool physicallyClosed = (level == LOW);

    switch (polarity_[channel])
    {
    case ContactPolarity::NORMALLY_OPEN:
        return physicallyClosed;

    case ContactPolarity::NORMALLY_CLOSED:
        return !physicallyClosed;
    }

    return false;
}

void InputModule::pollInputsAndDebounce()
{
    const uint32_t now = millis();
    bool anyChanged = false;

    for (uint8_t i = 0; i < INPUT_CHANNEL_COUNT; ++i)
    {
        const bool raw = readRawActive(i);

        if (raw != debouncedState_[i])
        {
            if (!candidatePending_[i] || candidateState_[i] != raw)
            {
                // New (or newly-different) candidate -- restart the debounce timer.
                candidateState_[i] = raw;
                candidateSinceMs_[i] = now;
                candidatePending_[i] = true;
            }
            else if (now - candidateSinceMs_[i] >= debounceMs_)
            {
                previousState_[i] = debouncedState_[i];
                debouncedState_[i] = candidateState_[i];

                candidatePending_[i] = false;
                anyChanged = true;

                // handleInputEdge(i, debouncedState_[i]);
            }
        }
        else
        {
            // Raw matches the already-accepted debounced state -- no
            // pending change for this channel.
            candidatePending_[i] = false;
        }
    }

    if (anyChanged)
    {
        sendStatusPush();
    }
}

bool InputModule::getPreviousInputState(uint8_t channel) const
{
    if (channel >= INPUT_CHANNEL_COUNT)
        return false;

    return previousState_[channel];
}

bool InputModule::hasInputChanged(uint8_t channel) const
{
    if (channel >= INPUT_CHANNEL_COUNT)
        return false;

    return debouncedState_[channel] != previousState_[channel];
}

bool InputModule::getInputState(uint8_t channel) const
{
    if (channel >= INPUT_CHANNEL_COUNT)
        return false;
    return debouncedState_[channel];
}

bool InputModule::isRisingEdge(uint8_t channel) const
{
    if (channel >= INPUT_CHANNEL_COUNT)
        return false;

    return !previousState_[channel] && debouncedState_[channel];
}

bool InputModule::isFallingEdge(uint8_t channel) const
{
    if (channel >= INPUT_CHANNEL_COUNT)
        return false;

    return previousState_[channel] && !debouncedState_[channel];
}

void InputModule::sendStatusPush()
{
    uint8_t payload[INPUT_CHANNEL_COUNT];

    for (uint8_t i = 0; i < INPUT_CHANNEL_COUNT; ++i)
    {
        payload[i] = debouncedState_[i] ? 1 : 0;
    }

    sendResponse(0xDDDD, 0xFFFF, payload, sizeof(payload));
}

void InputModule::handleInputEdge(uint8_t channel, bool state)
{
    const bool risingEdge = !previousState_[channel] && state;
    const bool fallingEdge = previousState_[channel] && !state;

    if (!risingEdge && !fallingEdge)
        return;

    // const SwitchType type = getSwitchType(channel);

    // if (type == SwitchType::Mechanical)
    // {
    //     if (risingEdge)
    //     {
    //         // Execute configured ON functions.
    //          // inputOnDelay_
    //     }
    //     else
    //     {2307

    //         // Execute configured OFF functions.
    //          // inputOffDelay_
    //     }

    //     return;
    // }

    // // All other switch types trigger only on 0 -> 1.
    // if (risingEdge)
    // {
    //     switch (type)
    //     {
    //     case SwitchType::SingleON:
    //     case SwitchType::SingleOFF:
    //     case SwitchType::SingleONOFF:
    //     case SwitchType::CombinationON:
    //     case SwitchType::CombinationOFF:
    //     case SwitchType::CombinationONOFF:
    //     case SwitchType::MultiFunction:
    //     case SwitchType::ParallelSwitch:
    //         // Execute the function(s) configured for this mode.
    //         break;

    //     case SwitchType::Mechanical:
    //         break;
    //     }
    // }
}