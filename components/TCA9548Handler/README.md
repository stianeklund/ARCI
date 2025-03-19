# TCA9548Handler

Driver for TCA9548A I2C multiplexer (1-to-8 bus switch).

## Purpose

Manages the TCA9548A I2C multiplexer to enable communication with multiple devices sharing the same I2C address or to isolate bus segments. Essential for systems with multiple I2C peripherals.

## Features

- **Channel selection**: Switch between 8 independent I2C channels
- **Multi-channel enable**: Enable multiple channels simultaneously
- **Status reading**: Query currently active channels
- **RAII helpers**: Auto-restore previous channel state

## Hardware

- **Device**: TCA9548A 8-channel I2C multiplexer
- **Address**: 0x70-0x77 (configurable via A0-A2 pins)
- **Channels**: SC0-SC7 (independent I2C buses)
- **Voltage**: 1.65V to 5.5V

## Usage

```cpp
#include "TCA9548Handler.h"

TCA9548Handler mux(I2C_NUM_0, 0x70);

// Select single channel
mux.selectChannel(3);  // Enable SC3

// Select multiple channels
mux.selectChannels(0b00000101);  // Enable SC0 and SC2

// Disable all channels
mux.disableAll();

// Read current state
uint8_t channels = mux.getActiveChannels();
```

## Common Applications

- Multiple displays with same I2C address
- Isolated sensor networks
- Antenna switch controller arrays
- GPIO expander banks

## Dependencies

- ESP-IDF I2C driver

## Configuration

Configure I2C bus and multiplexer address in component init.

## Design Notes

- No channel state caching (always reads/writes hardware)
- Thread-safe with I2C bus locking
- Compatible with ESP-IDF I2C master driver
