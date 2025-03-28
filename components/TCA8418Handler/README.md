# TCA8418Handler

Driver for TCA8418 I2C keypad matrix controller with interrupt support.

## Purpose

Interfaces with TCA8418 keypad controller to scan button matrices and handle key events. Provides hardware-based debouncing and interrupt-driven event notification, offloading keypad scanning from the CPU.

## Features

- **Matrix scanning**: Up to 8x10 key matrix (80 keys)
- **Hardware debouncing**: Configurable debounce time
- **Interrupt support**: Event notification via INT pin
- **FIFO buffering**: 10-event hardware FIFO
- **GPIO mode**: Can use pins as GPIOs when not used for keypad

## Hardware

- **Device**: TCA8418 I2C keypad scan IC
- **Address**: 0x34 (fixed)
- **Matrix**: 8 rows × 10 columns maximum
- **Interface**: I2C (400 kHz max)
- **Interrupt**: Active-low open-drain INT pin

## Usage

```cpp
#include "TCA8418Handler.h"

// Initialize with 4x4 matrix
TCA8418Handler keypad(I2C_NUM_0, 4, 4, GPIO_NUM_35);

// Configure
keypad.initialize();
keypad.setDebounceTime(20);  // 20ms debounce

// Poll for events
if (keypad.hasEvents()) {
    uint8_t row, col;
    bool pressed;
    while (keypad.getEvent(row, col, pressed)) {
        ESP_LOGI("KEY", "Row %d Col %d %s", row, col,
                 pressed ? "pressed" : "released");
    }
}

// Or use interrupt callback
keypad.setEventCallback([](uint8_t row, uint8_t col, bool pressed) {
    // Handle key event
});
```

## Applications

- Button matrix scanning for radio control panels
- Numeric keypads
- Custom keyboard interfaces

## Dependencies

- ESP-IDF I2C driver
- ESP-IDF GPIO (for interrupt pin)

## Design Notes

- Hardware FIFO prevents event loss during CPU busy periods
- Interrupt pin reduces polling overhead
- Compatible with ButtonHandler for higher-level button management
