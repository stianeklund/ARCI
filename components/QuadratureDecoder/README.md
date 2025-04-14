# QuadratureDecoder

Abstract interface for quadrature encoder decoding (rotary encoder inputs).

## Purpose

Defines a common interface for rotary encoder handling, supporting multiple decoder implementations with different performance and feature characteristics.

## Interface

```cpp
class IQuadratureDecoder {
    virtual esp_err_t initialize() = 0;
    virtual int32_t getCount() = 0;
    virtual void resetCount() = 0;
    virtual void setCount(int32_t value) = 0;
    virtual Direction getDirection() = 0;
};
```

## Implementations

- **SimpleQuadratureEncoder**: Software-based GPIO polling
- **EncoderHandler**: Advanced implementation with acceleration and filtering

## Usage

Used by `MultiEncoderHandler` and UI components for VFO tuning, menu navigation, and other rotary control functions.

## Features

- Direction detection
- Configurable resolution (detents per rotation)
- Count accumulation for precise positioning

## Dependencies

- ESP-IDF GPIO

## Design Notes

- Interface allows swapping between software and hardware (PCNT) implementations
- Designed for high-frequency sampling (10kHz+) in ISR context
