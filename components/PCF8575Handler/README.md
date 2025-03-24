# PCF8575Handler

Driver for PCF8575 I2C 16-bit GPIO expander with interrupt support.

## Purpose

Provides 16 additional GPIO pins over I2C for button inputs, LED outputs, or general-purpose I/O. Used to expand the ESP32's limited GPIO count without additional wiring complexity.

## Features

- **16-bit I/O**: 16 bidirectional GPIO pins (P00-P17)
- **Read/write**: Individual bit or full port operations
- **Interrupt support**: INT pin triggers on any input change
- **Quasi-bidirectional**: No direction register, pins are input by default
- **I2C interface**: Up to 400 kHz, address 0x20-0x27

## Hardware

- **Device**: PCF8575 or PCF8575C
- **Pins**: 16 quasi-bidirectional I/O
- **Address**: 0x20-0x27 (configurable via A0-A2)
- **Voltage**: 2.5V to 5.5V
- **Current**: 25mA per pin

## Usage

```cpp
#include "PCF8575Handler.h"

PCF8575Handler gpio(I2C_NUM_0, 0x20, GPIO_NUM_36);

// Initialize
gpio.initialize();

// Write outputs (LEDs)
gpio.setPin(0, true);   // Set P00 high
gpio.setPin(1, false);  // Set P01 low
gpio.writePort(0x00FF); // Set lower 8 bits high

// Read inputs (buttons)
bool button = gpio.readPin(8);  // Read P10
uint16_t inputs = gpio.readPort();

// Interrupt handling
gpio.setInterruptCallback([]() {
    // Input changed, read port to determine which pin
});
```

## Common Applications

- **Button matrices**: Row/column scanning
- **LED arrays**: Indicator lights
- **Relay control**: Switch external devices
- **Encoder inputs**: Additional rotary encoders

## Dependencies

- ESP-IDF I2C driver
- ESP-IDF GPIO (for interrupt pin)

## Configuration

Set I2C bus number, device address, and interrupt GPIO in constructor.

## Design Notes

- Quasi-bidirectional: outputs must be written high before reading
- Interrupt pin goes low on any input change (requires readPort() to clear)
- No internal pull-ups: external resistors required for button inputs
- Thread-safe with I2C bus locking
