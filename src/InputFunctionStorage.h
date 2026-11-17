
#pragma once

#include <cstdint>
#include <climits>


// at mosta it use 13 sector for 32 inpput
template <
uint32_t BaseAddress, 
uint8_t InputCount>
class InputFunctionStorage
{
public:
    static constexpr uint32_t SECTOR_SIZE = 0x1000;
    static constexpr uint32_t MAIN_SECTOR = BaseAddress;
    static constexpr uint32_t BASE_ADDRESS = MAIN_SECTOR * SECTOR_SIZE;

    static constexpr uint8_t INPUT_COUNT = InputCount;
    static constexpr uint8_t FUNCTIONS_PER_RANGE = 100;
    static constexpr uint8_t RANGE_COUNT = 2;

    static constexpr uint32_t FUNCTION_SIZE = 8;
    static constexpr uint32_t RANGE_SIZE =
        uint32_t(FUNCTIONS_PER_RANGE) * FUNCTION_SIZE;

    static constexpr uint32_t INPUT_SIZE =
        uint32_t(RANGE_COUNT) * RANGE_SIZE;

    static constexpr uint32_t TOTAL_SIZE =
        uint32_t(INPUT_COUNT) * INPUT_SIZE;

    static constexpr uint32_t SECTORS_REQUIRED =
        (TOTAL_SIZE + SECTOR_SIZE - 1) / SECTOR_SIZE;

    static_assert(INPUT_COUNT > 0, "InputCount must be nonzero");

    enum class Range : uint8_t
    {
        First = 0, // Functions 1–99
        Second = 1 // Functions 1–99
    };

    static constexpr bool validInput(uint8_t input)
    {
        return input >= 1 && input <= INPUT_COUNT;
    }

    static constexpr bool validFunction(uint8_t function)
    {
        return function >= 1 && function <= FUNCTIONS_PER_RANGE;
    }

    static constexpr bool validRange(Range range)
    {
        return range == Range::First || range == Range::Second;
    }

    static constexpr uint32_t inputAddress(uint8_t input)
    {
        if (!validInput(input))
            return UINT32_MAX;

        return BASE_ADDRESS + uint32_t(input - 1) * INPUT_SIZE;
    }

    static constexpr uint32_t rangeAddress(uint8_t input, Range range)
    {
        if (!validInput(input) || !validRange(range))
            return UINT32_MAX;

        return inputAddress(input) + uint32_t(range) * RANGE_SIZE;
    }

    static constexpr uint32_t functionAddress(
        uint8_t input,
        Range range,
        uint8_t function)
    {
        if (!validFunction(function))
            return UINT32_MAX;

        const uint32_t address = rangeAddress(input, range);

        if (address == UINT32_MAX)
            return UINT32_MAX;

        return address + uint32_t(function - 1) * FUNCTION_SIZE;
    }
};
