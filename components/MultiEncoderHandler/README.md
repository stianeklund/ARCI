# MultiEncoderHandler

Coordinates multiple rotary encoders from both direct GPIO and I2C sources.

## Purpose

Manages five encoders: RF gain, AF gain, multi-knob, and two additional EC11E encoders. Provides unified callback interface for rotation and switch events.

## Supported Encoders

1. **RF_GAIN**: EC11E #1 Axis 1 (I2C via PCF8575)
2. **AF_GAIN**: EC11E #1 Axis 2 (I2C via PCF8575)
3. **MULTI_KNOB**: Multi-purpose knob (direct GPIO)
4. **EC11E2_ENC1**: EC11E #2 Axis 1 (I2C via PCF8575) - IF Shift or High Cut
5. **EC11E2_ENC2**: EC11E #2 Axis 2 (I2C via PCF8575) - Low Cut or Filter Width

## Features

- Mixed GPIO and I2C encoder support
- Default callbacks for RF/AF gain control
- Mode-aware filter controls (IF shift for CW, cut filters for SSB)
- Switch press callbacks for all encoders
- RadioManager integration for direct state updates

## Usage

```cpp
radio::RadioManager radioManager(radioSerial, usbSerial);
MultiEncoderHandler encoders(&radioManager);

// Initialize
encoders.initializeDirectEncoders();
encoders.initializeI2CEncoders(&pcf8575);

// Use default callbacks (RF/AF gain, filter controls)
encoders.configureDefaultCallbacks();

// Or custom callbacks
encoders.setEncoderCallback([](EncoderId id, int32_t delta) {
    // Handle rotation
});

encoders.startAll();
```

## Default Behavior

- **RF_GAIN**: Adjusts RF gain (0-255)
- **AF_GAIN**: Adjusts AF gain, switch toggles mute
- **MULTI_KNOB**: Sends CH0/CH1 commands
- **EC11E2_ENC1**: IF Shift (CW/CW-R) or High Cut (SSB/AM)
- **EC11E2_ENC2**: Low Cut (SSB/AM) or Filter Width (other modes)

## Dependencies

- SimpleQuadratureEncoder (GPIO encoders)
- PCF8575Handler (I2C encoders)
- RadioManager (state access)
