# 4Z — 4-Channel Dry-Contact Input Subdevice Library (HDL Buspro-style)

Arduino-framework library (works with STM32duino / "STM32 Cores" board package)
for a 4-channel dry-contact (switch) input board that acts as a **subdevice**
on a shared RS485 bus, using HDL Buspro-style addressing (Subnet ID / Device ID)
and operation codes -- designed to share the same bus as the `4R` relay library.

## Depends on BusproCore

This library depends on a separate `BusproCore` library (frame codec,
transport, RS485 DE/RE control, address filtering, op-code dispatch) shared
with `4R` and any future subdevice on the same bus. Install `BusproCore`
alongside this library.

## ⚠️ Frame format and op-codes are placeholders -- see BusproCore

Same caveat as `4R`: the wire-level frame format lives in
`BusproCore/src/BusproFrame.cpp` and is **not yet verified** against real
HDL Buspro hardware. On top of that, this library's own op-codes
(`INPUT_STATUS_PUSH` / `_REQUEST` / `_RESPONSE`) are placeholder numbers I
chose to avoid colliding with 4R's op-codes -- they are **not** researched
HDL values (unlike 4R's `SCENE_CONTROL`, which you confirmed from a real
spec). Treat these as provisional until you have real captures of an
actual HDL input/sensor module's traffic, or have decided you don't need
byte-exact compatibility for this particular device type.

Do not deploy on a bus with real HDL Buspro devices until verified. See
`BusproCore/README.md` and the 4R library's `docs/CAPTURE_TEMPLATE.md` for
the capture procedure (applies equally here).

## What this library does

- Reads 4 dry-contact (switch) inputs via GPIO with internal pull-ups.
- Debounces each channel independently with a simple time-based debounce
  (default 30ms, configurable) -- a raw level must be stable for the full
  window before it's accepted as the new state.
- Supports **per-channel** configurable polarity (normally-open or
  normally-closed), since different contacts on the same board might be
  wired either way.
- On any debounced state change, immediately sends an unsolicited
  `INPUT_STATUS_PUSH` frame on the bus (broadcast by default -- see note
  below if you need it unicast to a specific master).
- Responds to `INPUT_STATUS_REQUEST` with the current state of all 4
  channels, so a master can poll/resync (e.g. after its own reboot,
  rather than relying solely on push events it may have missed).

## Push destination

`sendStatusPush()` currently sends to the broadcast address
(`BusproAddr::BROADCAST_SUBNET` / `BROADCAST_DEVICE`), since dry-contact
push events are often meant for whichever controller(s) care, not one
fixed master. If your setup has a single known master that should
receive these unicast instead, change `sendStatusPush()` in `Input4Z.cpp`
to call `sendStatusTo(masterSubnet, masterDevice)` with a configured
address -- this is a one-line change, intentionally factored out.

## Hardware assumptions

- RS485 transceiver (e.g. MAX485) with a single GPIO controlling combined DE/RE
- 4 GPIO inputs, each configured `INPUT_PULLUP`, with contacts wired
  switch-to-GND (so a closed contact reads LOW)
- One shared HardwareSerial port also used by other subdevices on the same
  bus (e.g. a 4R board) -- this device filters by Subnet ID + Device ID

## Quick start

```cpp
#include <Input4Z.h>

const uint8_t pins[INPUT4Z_CHANNEL_COUNT] = {PB0, PB1, PB2, PB3};
const ContactPolarity polarity[INPUT4Z_CHANNEL_COUNT] = {
    ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_OPEN,
    ContactPolarity::NORMALLY_OPEN, ContactPolarity::NORMALLY_CLOSED,
};

Input4Z input4z(/*subnet=*/1, /*device=*/20, /*dePin=*/PA8, pins, polarity, /*debounceMs=*/30);

void setup() {
    input4z.begin(&Serial1, 9600);
}

void loop() {
    input4z.poll(); // non-blocking; call frequently
}
```

## Running the desktop logic tests

```sh
g++ -std=c++17 -Wall -Wextra \
  -I test_stubs -I src -I ../BusproCore/src \
  ../BusproCore/src/BusproFrame.cpp ../BusproCore/src/BusproTransport.cpp \
  src/Input4Z.cpp test_stubs/test_main.cpp -o test_4z
./test_4z
```

Covers: debounce correctly ignoring short glitches, a stable change being
accepted and pushed after the debounce window, per-channel polarity
inversion (normally-open vs normally-closed), status request/response,
and address filtering (ignoring frames meant for a different device on
the bus). This validates library logic only, not real HDL wire
compatibility.
