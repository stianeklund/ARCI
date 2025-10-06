# AntennaSwitch

ESP32 Band Decoder antenna switch API client with HTTP and WebSocket support.

## Purpose

Provides automatic antenna switching based on frequency compatibility. Integrates with ESP32 Band Decoder hardware to control antenna selection via network API.

## Features

- HTTP and WebSocket transport modes with auto-fallback
- Async operation with connection pooling
- Cached status (configurable expiration)
- Fallback to radio's built-in antenna ports
- Real-time WebSocket event notifications

## Configuration

```cpp
antenna::AntennaConfig config;
config.enabled = true;
config.baseUrl = "http://192.168.1.100";
config.timeoutMs = 1000;
config.transportType = antenna::TransportType::Auto;  // Try WS, fallback to HTTP
config.enableWebSocketEvents = true;
```

## Usage

```cpp
antenna::AntennaSwitch antennaSwitch;
antennaSwitch.initialize(config);

// Switch to best antenna for frequency
antennaSwitch.selectAntennaForFrequency(14150000, 1);  // 20m, TX ANT 1

// Get status
auto status = antennaSwitch.getStatus();
```

## Documentation

- `API_REFERENCE.md`: Full API documentation
- `WEBSOCKET_INTEGRATION.md`: WebSocket protocol details

## Dependencies

- ESP HTTP Client
- cJSON
- AntennaWebSocketClient (included)
