# WebSocket Antenna Switch Integration

This document describes how to use the new WebSocket-based antenna switch API client that has been added alongside the existing HTTP implementation.
Note: This also isn't used very much as I have implemented the same websocket integration on the display (see the RemoteRadioDisplay repo).

## Overview

The WebSocket integration provides:
- **Real-time bidirectional communication** with sub-10ms latency
- **Automatic fallback** to HTTP when WebSocket is unavailable
- **Event-driven updates** for real-time antenna status changes
- **Connection resilience** with automatic reconnection
- **Full backward compatibility** with existing HTTP API

## Configuration

The `AntennaConfig` structure has been extended with WebSocket options:

```cpp
antenna::AntennaConfig config;
config.enabled = true;

// Transport selection
config.transportType = antenna::TransportType::Auto;  // Try WebSocket first, fallback to HTTP
// config.transportType = antenna::TransportType::WebSocket;  // WebSocket only
// config.transportType = antenna::TransportType::Http;       // HTTP only (original behavior)

// HTTP configuration (unchanged)
config.baseUrl = "http://192.168.1.100";
config.statusEndpoint = "/status";
config.controlEndpoint = "/api/antenna/switch";
config.timeoutMs = 1000;

// WebSocket configuration (new)
config.wsUrl = "ws://192.168.1.100/ws";
config.enableWebSocketEvents = true;    // Subscribe to real-time events
config.wsReconnectTimeoutMs = 5000;     // Reconnection delay
config.wsKeepAliveMs = 30000;           // Keep-alive ping interval
```

## Transport Types

### Auto Transport (Recommended)
- Attempts WebSocket connection first
- Falls back to HTTP if WebSocket unavailable
- Switches back to WebSocket when connection restored
- Best performance with maximum compatibility

```cpp
config.transportType = antenna::TransportType::Auto;
```

### WebSocket Only
- Uses only WebSocket communication
- Initialization fails if WebSocket unavailable
- Best for applications requiring real-time updates

```cpp
config.transportType = antenna::TransportType::WebSocket;
```

### HTTP Only
- Original behavior, no WebSocket client created
- Most compatible with existing setups
- No real-time event capabilities

```cpp
config.transportType = antenna::TransportType::Http;
```

## Usage Examples

### Basic Setup
```cpp
#include "RadioManager.h"

// In RadioManager initialization
antenna::AntennaConfig antennaConfig;
antennaConfig.enabled = true;
antennaConfig.transportType = antenna::TransportType::Auto;
antennaConfig.baseUrl = "http://192.168.1.100";
antennaConfig.wsUrl = "ws://192.168.1.100/ws";
antennaConfig.enableWebSocketEvents = true;

radioManager.getAntennaSwitch().initialize(antennaConfig);
```

### Antenna Switching
```cpp
// Switch to next antenna - automatically uses best available transport
uint64_t frequency = 14205000; // 20m band
bool success = radioManager.getAntennaSwitch().switchToNextAntenna(frequency);

if (success) {
    ESP_LOGI(TAG, "Antenna switch command sent successfully");
} else {
    ESP_LOGW(TAG, "Failed to switch antenna: %s", 
             radioManager.getAntennaSwitch().getLastError().c_str());
}
```

### Getting Antenna Information
```cpp
// Get compatible antennas for current frequency
auto antennas = radioManager.getAntennaSwitch().getCompatibleAntennas(frequency);
ESP_LOGI(TAG, "Found %zu compatible antennas", antennas.size());

for (const auto& antenna : antennas) {
    ESP_LOGI(TAG, "Antenna %d: %s (%s)", 
             antenna.id, antenna.name.c_str(), 
             antenna.isActive ? "active" : "inactive");
}

// Get current active antenna
antenna::AntennaInfo current = radioManager.getAntennaSwitch().getCurrentAntenna();
if (current.id != -1) {
    ESP_LOGI(TAG, "Current antenna: %s", current.name.c_str());
}
```

## WebSocket API Messages

The client automatically handles JSON message formatting according to the API specification:

### Status Query
```json
{
  "id": "req-1234567890-1",
  "type": "request",
  "action": "status"
}
```

### Antenna Switch
```json
{
  "id": "req-1234567890-2", 
  "type": "request",
  "action": "antenna_switch",
  "data": {
    "radio": "A",
    "action": "next"
  }
}
```

### Event Subscription
```json
{
  "id": "req-1234567890-3",
  "type": "request", 
  "action": "subscribe",
  "data": {
    "events": ["status_updates", "relay_state_changes", "frequency_changes"]
  }
}
```

## Real-Time Events

When `enableWebSocketEvents = true`, the client automatically subscribes to:
- **status_updates**: Full antenna status changes
- **relay_state_changes**: Individual relay state modifications  
- **frequency_changes**: Operating frequency updates

Events update the internal cache in real-time, providing instant response to queries.

## Performance Characteristics

### WebSocket Transport
- **Connection Setup**: < 100ms typical
- **Message Latency**: < 10ms typical  
- **Memory Overhead**: ~12-20KB additional RAM
- **Real-time Updates**: Automatic via events

### HTTP Transport (Fallback)
- **Request Latency**: 50-200ms typical
- **Memory Overhead**: ~8KB
- **Real-time Updates**: Not available

## Error Handling and Resilience

### Automatic Reconnection
- WebSocket connections automatically reconnect on failure
- Configurable retry count and timeout intervals
- Switches to HTTP during WebSocket outages
- Returns to WebSocket when connection restored

### Graceful Degradation
```cpp
// Transport switching is transparent to application code
bool result = antennaSwitch.switchToNextAntenna(frequency);
// Uses WebSocket if available, HTTP if not - same API call
```

### Error Reporting
```cpp
if (!antennaSwitch.switchToNextAntenna(frequency)) {
    std::string error = antennaSwitch.getLastError();
    ESP_LOGW(TAG, "Antenna switch failed: %s", error.c_str());
    
    // Check transport availability
    if (!antennaSwitch.isApiAvailable()) {
        ESP_LOGW(TAG, "Antenna switch API not available");
    }
}
```

## Integration with Existing Code

### No Changes Required
Existing code using `AntennaSwitch` continues to work unchanged:
- `switchToNextAntenna()` - same signature and behavior
- `getCompatibleAntennas()` - same signature and behavior  
- `getCurrentAntenna()` - same signature and behavior
- `isApiAvailable()` - same signature and behavior

### Performance Improvements
- Cached responses are now updated in real-time via WebSocket events
- Query responses are significantly faster when cache is fresh
- Network traffic reduced through event-driven updates

## Testing

Basic integration tests are available in `components/AntennaSwitch/test/`:

```bash
# Run antenna switch WebSocket tests (requires ESP-IDF environment)
idf.py -C components/AntennaSwitch test
```

Tests verify:
- Configuration validation
- Transport switching logic  
- Message format compliance
- Error handling behavior
- Integration with caching system

## Troubleshooting

### WebSocket Connection Issues
```cpp
// Check if WebSocket is available
if (!antennaSwitch.isApiAvailable()) {
    ESP_LOGW(TAG, "WebSocket unavailable, check network and server");
    // Will automatically use HTTP fallback
}
```

### High Memory Usage
- Reduce `messageQueueSize` in WebSocket configuration
- Disable `enableWebSocketEvents` if real-time updates not needed
- Use HTTP-only transport for memory-constrained applications

### Connection Instability
- Increase `wsReconnectTimeoutMs` for slower networks
- Reduce `wsKeepAliveMs` for faster failure detection
- Check network infrastructure for WebSocket support

## Configuration Examples

### Low Latency (Real-time Applications)
```cpp
config.transportType = antenna::TransportType::WebSocket;
config.wsKeepAliveMs = 10000;     // Fast keepalive
config.wsReconnectTimeoutMs = 1000; // Quick reconnection
config.cacheExpirationMs = 5000;  // Short cache TTL
config.enableWebSocketEvents = true;
```

### Memory Optimized
```cpp
config.transportType = antenna::TransportType::Auto;
config.enableWebSocketEvents = false; // No event subscription
config.cacheExpirationMs = 300000;   // Long cache TTL  
config.useAsyncMode = false;          // No async worker task
```

### Maximum Compatibility
```cpp
config.transportType = antenna::TransportType::Http;
// WebSocket client not created - original behavior
```