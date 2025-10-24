# WiFiManager

WiFi connection management with station mode support and persistent configuration.

## Purpose

Handles WiFi initialization, connection, and monitoring for network-based features (TCP CAT, WebSocket, remote control). Manages credentials via NVS and provides reconnection logic.

## Features

- **Station mode**: Connect to existing WiFi network
- **Persistent credentials**: Store SSID/password in NVS
- **Auto-reconnect**: Automatic reconnection on disconnect
- **Event monitoring**: Connection state callbacks
- **RSSI tracking**: Signal strength monitoring
- **IP address management**: DHCP client

## Usage

```cpp
#include "WiFiManager.h"

WiFiManager wifi;

// Configure credentials
wifi.setCredentials("MySSID", "MyPassword");

// Initialize and connect
wifi.initialize();
wifi.connect();

// Check status
if (wifi.isConnected()) {
    std::string ip = wifi.getIpAddress();
    int8_t rssi = wifi.getRSSI();
    ESP_LOGI("WiFi", "Connected: %s, RSSI: %d", ip.c_str(), rssi);
}

// Register event callback
wifi.setEventCallback([](wifi_event_t event) {
    if (event == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI("WiFi", "Connected to AP");
    }
});
```

## Configuration

Kconfig options:
- `WIFI_SSID`: Default SSID
- `WIFI_PASSWORD`: Default password
- `WIFI_MAXIMUM_RETRY`: Max reconnection attempts
- `WIFI_CONNECT_TIMEOUT`: Connection timeout (ms)

## Dependencies

- ESP-IDF WiFi driver
- ESP-IDF event loop
- NvsManager (for credential storage)

## Design Notes

- Uses ESP-IDF WiFi provisioning APIs
- Thread-safe event handling
- Supports both compile-time and runtime credential configuration
- Does not implement AP mode or provisioning (station mode only)
