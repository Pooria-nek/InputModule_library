#pragma once
// Minimal stand-in for Arduino.h, used ONLY to compile-test the library
// logic on a desktop g++ before deploying to real hardware. Not shipped
// as part of the library; STM32duino/Arduino provides the real one.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <queue>
#include <vector>

#define HIGH 1
#define LOW  0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int level) {
    std::printf("[pin %d] -> %s\n", pin, level ? "HIGH" : "LOW");
}

// Test harness can override per-pin digital input levels via this map;
// defaults to HIGH (idle/open with a pull-up) if not explicitly set.
#include <map>
inline std::map<int, int>& testPinLevels() {
    static std::map<int, int> levels;
    return levels;
}
inline int digitalRead(int pin) {
    auto& levels = testPinLevels();
    auto it = levels.find(pin);
    return (it != levels.end()) ? it->second : HIGH;
}
inline void testSetPinLevel(int pin, int level) {
    testPinLevels()[pin] = level;
}

// Separate map for analog readings (0-4095 style), since ADC pins and
// digital pins are conceptually different even though both use "int pin"
// in this stub.
inline std::map<int, int>& testAnalogLevels() {
    static std::map<int, int> levels;
    return levels;
}
inline int analogRead(int pin) {
    auto& levels = testAnalogLevels();
    auto it = levels.find(pin);
    return (it != levels.end()) ? it->second : 0;
}
inline void testSetAnalogLevel(int pin, int value) {
    testAnalogLevels()[pin] = value;
}

// Fake millis(): test harness advances this manually to exercise debounce
// timing without real delays.
inline uint32_t& testMillisRef() {
    static uint32_t t = 0;
    return t;
}
inline uint32_t millis() { return testMillisRef(); }
inline void testAdvanceMillis(uint32_t deltaMs) { testMillisRef() += deltaMs; }

// Fake serial port: lets the test harness push bytes in and inspect bytes out.
class HardwareSerial {
public:
    void begin(uint32_t) {}
    int available() { return static_cast<int>(rxQueue_.size()); }
    int read() {
        if (rxQueue_.empty()) return -1;
        int b = rxQueue_.front();
        rxQueue_.pop();
        return b;
    }
    size_t write(const uint8_t* buf, size_t len) {
        for (size_t i = 0; i < len; ++i) txLog_.push_back(buf[i]);
        return len;
    }
    void flush() {}

    // test helpers
    void injectByte(uint8_t b) { rxQueue_.push(b); }
    void injectBytes(const uint8_t* buf, size_t len) {
        for (size_t i = 0; i < len; ++i) rxQueue_.push(buf[i]);
    }
    std::vector<uint8_t> takeTxLog() {
        auto out = txLog_;
        txLog_.clear();
        return out;
    }

private:
    std::queue<uint8_t> rxQueue_;
    std::vector<uint8_t> txLog_;
};
