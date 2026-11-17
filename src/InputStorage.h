#pragma once

#include <cstdint>
#include <climits>

template <
    uint32_t BaseAddress,
    uint8_t ChannelCount>
class InputStorage
{
public:
    // Device address helpers
    static constexpr uint32_t deviceAddress()
    {
        return BASE_ADDRESS + DEVICE_ADDRESS;
    }

    static constexpr uint32_t deviceTypeAddress()
    {
        return BASE_ADDRESS + DEVICE_TYPE;
    }

    static constexpr uint32_t deviceMacAddress()
    {
        return BASE_ADDRESS + DEVICE_MAC_ADDRESS;
    }

    static constexpr uint32_t deviceRemarkAddress()
    {
        return BASE_ADDRESS + DEVICE_REMARK;
    }

    static constexpr uint32_t hardwareVersionAddress()
    {
        return BASE_ADDRESS + DEVICE_HARDWARE_VER;
    }

    static constexpr uint32_t softwareVersionAddress()
    {
        return BASE_ADDRESS + DEVICE_SOFTWARE_VER;
    }

    // Channel remark address
    static constexpr uint32_t channelRemarkAddress(uint8_t channel)
    {
        if (!validChannel(channel))
            return UINT32_MAX;

        return BASE_ADDRESS + CHANNEL_REMARK + uint32_t(channel) * CHANNEL_REMARK_SIZE;
    }

    // Address of a one-byte-per-channel setting
    static constexpr uint32_t channelAddress(uint32_t offset, uint8_t channel)
    {
        if (!validChannel(channel))
            return UINT32_MAX;

        return BASE_ADDRESS + offset + uint32_t(channel);
    }

    static constexpr bool validChannel(uint8_t channel)
    {
        return channel >= 1 && channel <= CHANNEL_COUNT;
    }

    // Convenience helpers
    static constexpr uint32_t channelEnableAddress(uint8_t channel)
    {
        return channelAddress(CHANNEL_ENABLE, channel);
    }
    static constexpr uint32_t channelEnableAddress()
    {
        return BASE_ADDRESS + CHANNEL_ENABLE;
    }

    static constexpr uint32_t channelOnDelayAddress(uint8_t channel)
    {
        return channelAddress(CHANNEL_ONDELAY, channel);
    }
    static constexpr uint32_t channelOnDelayAddress()
    {
        return BASE_ADDRESS + CHANNEL_ONDELAY;
    }

    static constexpr uint32_t channelOffDelayAddress(uint8_t channel)
    {
        return channelAddress(CHANNEL_OFFDELAY, channel);
    }
    static constexpr uint32_t channelOffDelayAddress()
    {
        return BASE_ADDRESS + CHANNEL_OFFDELAY;
    }

    static constexpr uint32_t channelTypeAddress(uint8_t channel)
    {
        return channelAddress(CHANNEL_TYPE, channel);
    }
    static constexpr uint32_t channelTypeAddress()
    {
        return BASE_ADDRESS + CHANNEL_TYPE;
    }

    static constexpr uint32_t channelDimmingAddress(uint8_t channel)
    {
        return channelAddress(CHANNEL_DIMMING, channel);
    }
    static constexpr uint32_t channelDimmingAddress()
    {
        return BASE_ADDRESS + CHANNEL_DIMMING;
    }

private:
    static constexpr uint32_t SECTOR_SIZE = 0x1000;
    static constexpr uint32_t MAIN_SECTOR = BaseAddress;

    static constexpr uint32_t BASE_ADDRESS = MAIN_SECTOR * SECTOR_SIZE;
    static constexpr uint8_t CHANNEL_COUNT = ChannelCount;

    // Device information
    static constexpr uint32_t DEVICE_ADDRESS = 0x000010;
    static constexpr uint32_t DEVICE_TYPE = 0x000012;
    static constexpr uint32_t DEVICE_MAC_ADDRESS = 0x000014;

    static constexpr uint32_t DEVICE_REMARK = 0x000020;
    static constexpr uint32_t DEVICE_HARDWARE_VER = 0x000040;
    static constexpr uint32_t DEVICE_SOFTWARE_VER = 0x000060;

    // Channel remarks
    static constexpr uint32_t CHANNEL_REMARK = 0x000080;
    static constexpr uint32_t CHANNEL_REMARK_SIZE = 0x20;

    // Channel configuration
    static constexpr uint32_t CHANNEL_ENABLE = 0x000380;
    static constexpr uint32_t CHANNEL_ONDELAY = 0x0003A0;
    static constexpr uint32_t CHANNEL_OFFDELAY = 0x0003C0;
    static constexpr uint32_t CHANNEL_TYPE = 0x0003E0;
    static constexpr uint32_t CHANNEL_DIMMING = 0x000400;
};