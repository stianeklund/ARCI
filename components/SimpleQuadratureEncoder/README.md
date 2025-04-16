# SimpleQuadratureEncoder

Lightweight quadrature encoder driver using GPIO interrupts (no PCNT).

## Overview

SimpleQuadratureEncoder provides basic rotary encoder support without consuming PCNT hardware units. Ideal for secondary encoders that don't need acceleration logic.

## Features

- GPIO interrupt-based quadrature decoding
- Hardware glitch filtering (configurable, default 2000ns)
- Optional switch input with debouncing
- EMI noise rejection (burst suppression, rate limiting)
- Detent accumulator for mechanical bounce filtering

## Usage

```cpp
SimpleQuadratureEncoder encoder(GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6);
encoder.initialize();

encoder.setRotationCallback([](int32_t delta) {
    // delta > 0 = clockwise, delta < 0 = counter-clockwise
});

encoder.setSwitchCallback([](bool pressed) {
    // Handle switch press/release
});
```

## Noise Filtering

Multiple stages of noise rejection:
1. Hardware glitch filter on GPIO pins
2. Rate limiting (reject transitions < 500us apart)
3. Burst suppression (reject alternating +1/-1 patterns)
4. Detent accumulator (require 2 transitions per reported click)
5. Switch press lockout (ignore rotation for 50ms after switch)

## Performance

- ISR latency: ~1-2us
- No PCNT unit consumption
- Suitable for encoders up to ~1000 RPM

## Dependencies

- ESP-IDF GPIO driver
- FreeRTOS
