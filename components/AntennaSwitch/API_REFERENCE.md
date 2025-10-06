# AntennaSwitch WebSocket API Reference

API reference for the WebSocket-enabled AntennaSwitch component, covering client-side C++ API and WebSocket protocol specification.

## Table of Contents

- [Client C++ API](#client-c-api)
  - [AntennaSwitch Class](#antennaswitch-class)
  - [AntennaWebSocketClient Class](#antennawebsocketclient-class)
  - [Configuration Structures](#configuration-structures)
  - [Data Types](#data-types)
- [WebSocket Protocol API](#websocket-protocol-api)
  - [Connection Management](#connection-management)
  - [Message Format](#message-format)
  - [Request Types](#request-types)
  - [Response Types](#response-types)
  - [Event Types](#event-types)
  - [Error Handling](#error-handling)
- [Examples](#examples)
- [Error Codes](#error-codes)

---

## Client C++ API

### AntennaSwitch Class

The main interface for antenna switching operations, supporting both HTTP and WebSocket transports.

#### Constructor

```cpp
AntennaSwitch();
```

Creates a new AntennaSwitch instance. Does not initialize any transport mechanisms.

#### Destructor

```cpp
~AntennaSwitch();
```

Automatically cleans up all resources including HTTP client and WebSocket client.

---

#### Core Methods

##### initialize()

```cpp
bool initialize(const AntennaConfig& config);
```

Initializes the antenna switch with the specified configuration.

**Parameters:**
- `config`: Configuration structure containing transport settings

**Returns:**
- `true` if initialization successful
- `false` if initialization failed (check `getLastError()`)

**Example:**
```cpp
antenna::AntennaConfig config;
config.enabled = true;
config.transportType = antenna::TransportType::Auto;
config.baseUrl = "http://192.168.1.100";
config.wsUrl = "ws://192.168.1.100/ws";

if (!antennaSwitch.initialize(config)) {
    ESP_LOGE(TAG, "Failed to initialize: %s", antennaSwitch.getLastError().c_str());
}
```

##### switchToNextAntenna()

```cpp
bool switchToNextAntenna(uint64_t frequencyHz);
```

Switches to the next compatible antenna for the given frequency.

**Parameters:**
- `frequencyHz`: Operating frequency in Hz

**Returns:**
- `true` if switch command sent successfully
- `false` if operation failed

**Behavior:**
- Uses WebSocket transport if available and connected
- Falls back to HTTP if WebSocket unavailable
- Caches result for performance optimization

**Example:**
```cpp
uint64_t freq = 14205000; // 20m band
if (antennaSwitch.switchToNextAntenna(freq)) {
    ESP_LOGI(TAG, "Antenna switch command sent successfully");
}
```

##### switchToAntenna()

```cpp
bool switchToAntenna(int antennaId);
```

Switches to a specific antenna by ID. Note: The underlying API uses "next" action cycling.

**Parameters:**
- `antennaId`: Antenna ID (parameter is preserved for compatibility but ignored)

**Returns:**
- `true` if switch command sent successfully
- `false` if operation failed

**Note:** The antenna switch API only supports "next" and "previous" actions, not direct antenna selection.

##### getCompatibleAntennas()

```cpp
std::vector<AntennaInfo> getCompatibleAntennas(uint64_t frequencyHz);
```

Retrieves list of antennas compatible with the given frequency.

**Parameters:**
- `frequencyHz`: Frequency in Hz to check compatibility

**Returns:**
- Vector of `AntennaInfo` structures containing compatible antennas
- Empty vector if no antennas available or on error

**Caching:**
- Results are cached based on frequency band
- Cache TTL configurable via `cacheExpirationMs`
- WebSocket events automatically update cache

**Example:**
```cpp
auto antennas = antennaSwitch.getCompatibleAntennas(14205000);
for (const auto& antenna : antennas) {
    ESP_LOGI(TAG, "Antenna %d: %s (%s)", 
             antenna.id, antenna.name.c_str(),
             antenna.isActive ? "active" : "inactive");
}
```

##### getCurrentAntenna()

```cpp
AntennaInfo getCurrentAntenna();
```

Gets information about the currently active antenna.

**Returns:**
- `AntennaInfo` structure with current antenna details
- Empty structure (id = -1) if no antenna active or on error

**Example:**
```cpp
antenna::AntennaInfo current = antennaSwitch.getCurrentAntenna();
if (current.id != -1) {
    ESP_LOGI(TAG, "Current antenna: %s", current.name.c_str());
}
```

---

#### Status and Configuration Methods

##### isApiAvailable()

```cpp
bool isApiAvailable();
```

Checks if the antenna switch API is reachable and responding.

**Returns:**
- `true` if API is available (HTTP or WebSocket)
- `false` if API is unreachable

##### isNetworkReady()

```cpp
bool isNetworkReady();
```

Checks if network connectivity is available.

**Returns:**
- `true` if network interface has IP address
- `false` if network is not ready

##### getConfig()

```cpp
const AntennaConfig& getConfig() const;
```

Gets the current configuration.

**Returns:**
- Reference to current `AntennaConfig` structure

##### updateConfig()

```cpp
void updateConfig(const AntennaConfig& config);
```

Updates the configuration and reinitializes transport if needed.

**Parameters:**
- `config`: New configuration structure

##### getLastError()

```cpp
std::string getLastError() const;
```

Gets the last error message from any operation.

**Returns:**
- String containing last error message
- Empty string if no error occurred

---

### AntennaWebSocketClient Class

Low-level WebSocket client for direct protocol access.

#### Constructor

```cpp
AntennaWebSocketClient();
```

Creates WebSocket client instance. Must call `initialize()` before use.

#### Destructor

```cpp
~AntennaWebSocketClient();
```

Automatically disconnects and cleans up all resources.

---

#### Connection Management

##### initialize()

```cpp
bool initialize(const WSAntennaConfig& config);
```

Initializes WebSocket client with configuration.

**Parameters:**
- `config`: WebSocket-specific configuration structure

**Returns:**
- `true` if initialization successful
- `false` on failure

##### connect()

```cpp
bool connect();
```

Establishes connection to WebSocket server.

**Returns:**
- `true` if connection successful
- `false` on connection failure

**Timeout:** Configurable via `responseTimeoutMs` in config

##### disconnect()

```cpp
void disconnect();
```

Closes WebSocket connection gracefully.

##### isConnected()

```cpp
bool isConnected() const;
```

Checks connection status.

**Returns:**
- `true` if connected to server
- `false` if disconnected

---

#### Message Sending Methods

##### sendStatusQuery()

```cpp
bool sendStatusQuery(const std::string& requestId = "");
```

Sends status query request to server.

**Parameters:**
- `requestId`: Optional request ID for correlation (auto-generated if empty)

**Returns:**
- `true` if message sent successfully
- `false` on send failure

**Protocol Message:**
```json
{
  "id": "req-1234567890-1",
  "type": "request",
  "action": "status"
}
```

##### sendRelayControl()

```cpp
bool sendRelayControl(int relay, bool state, const std::string& requestId = "");
```

Sends relay control command.

**Parameters:**
- `relay`: Relay number (1-16)
- `state`: Relay state (true = ON, false = OFF)
- `requestId`: Optional request ID for correlation

**Returns:**
- `true` if message sent successfully
- `false` on send failure or invalid parameters

**Protocol Message:**
```json
{
  "id": "req-1234567890-2",
  "type": "request",
  "action": "relay_control",
  "data": {
    "relay": 3,
    "state": true
  }
}
```

##### sendAntennaSwitch()

```cpp
bool sendAntennaSwitch(const std::string& radio, const std::string& action, 
                       const std::string& requestId = "");
```

Sends antenna switch command.

**Parameters:**
- `radio`: Radio identifier ("A" or "B")
- `action`: Switch action ("next" or "previous")
- `requestId`: Optional request ID for correlation

**Returns:**
- `true` if message sent successfully
- `false` on send failure or invalid parameters

**Protocol Message:**
```json
{
  "id": "req-1234567890-3",
  "type": "request",
  "action": "antenna_switch",
  "data": {
    "radio": "A",
    "action": "next"
  }
}
```

##### sendConfigQuery()

```cpp
bool sendConfigQuery(const std::string& requestId = "");
```

Requests basic system configuration.

**Returns:**
- `true` if message sent successfully
- `false` on send failure

##### sendRelayNamesQuery()

```cpp
bool sendRelayNamesQuery(const std::string& requestId = "");
```

Requests custom relay names/labels.

**Returns:**
- `true` if message sent successfully
- `false` on send failure

---

#### Event Subscription

##### subscribeToEvents()

```cpp
bool subscribeToEvents(const std::vector<std::string>& events, 
                       const std::string& requestId = "");
```

Subscribes to real-time event notifications.

**Parameters:**
- `events`: List of event types to subscribe to
- `requestId`: Optional request ID for correlation

**Available Events:**
- `"status_updates"`: Full status updates
- `"relay_state_changes"`: Individual relay state changes
- `"frequency_changes"`: Frequency updates
- `"transmit_state_changes"`: TX state changes
- `"config_changes"`: Configuration updates

**Returns:**
- `true` if subscription request sent successfully
- `false` on send failure

**Example:**
```cpp
std::vector<std::string> events = {
    "status_updates", 
    "relay_state_changes", 
    "frequency_changes"
};
wsClient.subscribeToEvents(events);
```

##### unsubscribeFromEvents()

```cpp
bool unsubscribeFromEvents(const std::vector<std::string>& events, 
                           const std::string& requestId = "");
```

Unsubscribes from event notifications.

**Parameters:**
- `events`: List of event types to unsubscribe from
- `requestId`: Optional request ID for correlation

**Returns:**
- `true` if unsubscription request sent successfully
- `false` on send failure

---

#### Callback Registration

##### setResponseCallback()

```cpp
void setResponseCallback(WSEventCallback callback);
```

Sets callback function for handling server responses.

**Parameters:**
- `callback`: Function to call when response received

**Callback Signature:**
```cpp
using WSEventCallback = std::function<void(const WSMessage&)>;
```

##### setEventCallback()

```cpp
void setEventCallback(WSEventCallback callback);
```

Sets callback function for handling server events.

**Parameters:**
- `callback`: Function to call when event received

##### setErrorCallback()

```cpp
void setErrorCallback(WSEventCallback callback);
```

Sets callback function for handling error messages.

**Parameters:**
- `callback`: Function to call when error received

---

#### Statistics

##### getStats()

```cpp
Stats getStats() const;
```

Gets connection and message statistics.

**Returns:**
- `Stats` structure containing performance metrics

**Stats Structure:**
```cpp
struct Stats {
    uint32_t messagesSent = 0;
    uint32_t messagesReceived = 0;
    uint32_t reconnectCount = 0;
    std::chrono::steady_clock::time_point lastConnectTime;
    std::chrono::steady_clock::time_point lastMessageTime;
};
```

---

### Configuration Structures

#### AntennaConfig

Main configuration structure for AntennaSwitch.

```cpp
struct AntennaConfig {
    // Basic settings
    bool enabled = false;
    
    // HTTP configuration
    std::string baseUrl = "http://192.168.1.100";
    int timeoutMs = 1000;
    bool fallbackToRadio = true;
    int maxRetries = 2;
    std::string statusEndpoint = "/status";
    std::string controlEndpoint = "/api/antenna/switch";
    
    // Performance optimization
    bool useAsyncMode = true;
    int cacheExpirationMs = 60000;
    int connectionPoolSize = 1;
    bool enableCompression = false;
    
    // WebSocket configuration
    TransportType transportType = TransportType::Auto;
    std::string wsUrl = "ws://192.168.1.100/ws";
    bool enableWebSocketEvents = true;
    int wsReconnectTimeoutMs = 5000;
    int wsKeepAliveMs = 30000;
};
```

#### WSAntennaConfig

WebSocket-specific configuration structure.

```cpp
struct WSAntennaConfig {
    std::string wsUrl = "ws://192.168.1.100/ws";
    int reconnectTimeoutMs = 5000;
    int keepaliveIntervalMs = 30000;
    int responseTimeoutMs = 3000;
    bool autoReconnect = true;
    int maxReconnectAttempts = 5;
    int messageQueueSize = 20;
};
```

---

### Data Types

#### TransportType

Enumeration for transport selection.

```cpp
enum class TransportType {
    Http,       // HTTP only (original behavior)
    WebSocket,  // WebSocket only
    Auto        // Try WebSocket first, fallback to HTTP
};
```

#### AntennaInfo

Structure containing antenna information.

```cpp
struct AntennaInfo {
    int id = -1;
    std::string name;
    std::vector<std::string> compatibleBands;
    bool isActive = false;
    
    AntennaInfo();
    AntennaInfo(int antennaId, const std::string& antennaName);
};
```

#### WSMessage

Structure for WebSocket message data.

```cpp
struct WSMessage {
    std::string id;
    WSMessageType type;
    std::string action;
    std::string data;
    std::string event;
};
```

#### WSMessageType

Enumeration for WebSocket message types.

```cpp
enum class WSMessageType {
    Request,
    Response,
    Event,
    Error
};
```

---

## WebSocket Protocol API

### Connection Management

#### Connection Endpoint

```
ws://device-ip/ws
```

#### Connection Limits

- **Maximum Concurrent Connections**: 5 clients
- **Connection Timeout**: 60 seconds
- **Keepalive Interval**: 30 seconds

#### Connection Lifecycle

1. **HTTP Upgrade**: Client sends WebSocket upgrade request
2. **Connection Accepted**: Server accepts and tracks connection
3. **Active Communication**: Bidirectional message exchange
4. **Keepalive**: Automatic ping/pong every 30 seconds
5. **Cleanup**: Inactive connections removed after 60 seconds
6. **Graceful Closure**: Client or server initiated close

---

### Message Format

All messages use JSON format with the following structure:

```json
{
  "id": "unique_request_id",
  "type": "request|response|event|error",
  "action": "action_name",
  "data": { /* action-specific data */ }
}
```

#### Message Fields

- **id** (string, optional): Unique request ID for correlating requests with responses
- **type** (string, required): Message type indicator
- **action** (string, required for requests): Action to perform
- **data** (object, optional): Action-specific data payload
- **event** (string, required for events): Event type name

---

### Request Types

#### Status Query

Request current system status including frequency, antenna, and available antennas.

**Request:**
```json
{
  "id": "req-001",
  "type": "request",
  "action": "status"
}
```

**Response:**
```json
{
  "id": "req-001",
  "type": "response",
  "data": {
    "frequency": 14205000,
    "frequency_mhz": 14.205,
    "antenna": "Antenna 3",
    "transmitting": false,
    "data_source": "Serial",
    "available_antennas": [1, 2, 3, 4],
    "antenna_b": "None",
    "frequency_b": 0,
    "transmitting_b": false,
    "data_source_b": "Serial",
    "available_antennas_b": []
  }
}
```

#### Relay Control

Control individual relays directly.

**Request:**
```json
{
  "id": "req-002",
  "type": "request",
  "action": "relay_control",
  "data": {
    "relay": 3,
    "state": true
  }
}
```

**Parameters:**
- `relay`: Relay number (1-16)
- `state`: Boolean (true = ON, false = OFF)

**Response:**
```json
{
  "id": "req-002",
  "type": "response",
  "data": {
    "state": true
  }
}
```

#### Antenna Switching

High-level antenna switching for next/previous antenna in current band.

**Request:**
```json
{
  "id": "req-003",
  "type": "request",
  "action": "antenna_switch",
  "data": {
    "radio": "A",
    "action": "next"
  }
}
```

**Parameters:**
- `radio`: "A" or "B"
- `action`: "next" or "previous"

**Response:**
```json
{
  "id": "req-003",
  "type": "response",
  "data": {
    "status": "success",
    "radio": "A",
    "frequency": 14205000,
    "frequency_mhz": 14.205,
    "band": "20M",
    "previous_antenna": 2,
    "new_antenna": 3,
    "available_antennas": [1, 2, 3, 4]
  }
}
```

#### Configuration Query

Get basic system configuration information.

**Request:**
```json
{
  "id": "req-004",
  "type": "request",
  "action": "config_basic"
}
```

**Response:**
```json
{
  "id": "req-004",
  "type": "response",
  "data": {
    "auto_mode": true,
    "num_bands": 8,
    "num_antenna_ports": 8,
    "mqtt_enabled": false,
    "radio_operation_mode": "SINGLE_A"
  }
}
```

#### Relay Names

Get custom relay names/labels.

**Request:**
```json
{
  "id": "req-005",
  "type": "request",
  "action": "relay_names"
}
```

**Response:**
```json
{
  "id": "req-005",
  "type": "response",
  "data": {
    "1": "40M Yagi",
    "2": "40M Dipole",
    "3": "20M Beam",
    "4": "20M Vertical",
    "5": "Relay 5",
    "6": "Relay 6",
    "7": "Relay 7",
    "8": "Relay 8",
    "9": "Relay 9",
    "10": "Relay 10",
    "11": "Relay 11",
    "12": "Relay 12",
    "13": "Relay 13",
    "14": "Relay 14",
    "15": "Relay 15",
    "16": "Relay 16"
  }
}
```

#### Event Subscription

Subscribe/unsubscribe from real-time events.

**Subscribe Request:**
```json
{
  "id": "req-006",
  "type": "request",
  "action": "subscribe",
  "data": {
    "events": ["status_updates", "relay_state_changes", "frequency_changes"]
  }
}
```

**Unsubscribe Request:**
```json
{
  "id": "req-007",
  "type": "request",
  "action": "unsubscribe",
  "data": {
    "events": ["status_updates"]
  }
}
```

**Response:**
```json
{
  "id": "req-006",
  "type": "response",
  "data": {
    "status": "success",
    "message": "Subscribed to events"
  }
}
```

**Available Events:**
- `status_updates`: Full status updates
- `relay_state_changes`: Individual relay state changes
- `frequency_changes`: Frequency updates
- `transmit_state_changes`: TX state changes
- `config_changes`: Configuration updates

---

### Response Types

All responses include the original request ID for correlation and follow the standard message format.

#### Success Response

```json
{
  "id": "original-request-id",
  "type": "response",
  "data": {
    /* action-specific response data */
  }
}
```

#### Error Response

```json
{
  "id": "original-request-id",
  "type": "error",
  "data": {
    "status": "error",
    "message": "Error description"
  }
}
```

---

### Event Types

Server automatically sends events to subscribed clients when system state changes.

#### Status Update Event

```json
{
  "type": "event",
  "event": "status_update",
  "data": {
    "frequency": 14205000,
    "frequency_mhz": 14.205,
    "antenna": "Antenna 3",
    "transmitting": false,
    "data_source": "Serial",
    "available_antennas": [1, 2, 3, 4]
  }
}
```

#### Relay State Change Event

```json
{
  "type": "event",
  "event": "relay_state_changed",
  "data": {
    "relay": 3,
    "state": true
  }
}
```

#### Frequency Change Event

```json
{
  "type": "event",
  "event": "frequency_changed",
  "data": {
    "frequency": 14205000,
    "frequency_mhz": 14.205
  }
}
```

#### Transmit State Change Event

```json
{
  "type": "event",
  "event": "transmit_state_changed",
  "data": {
    "transmitting": true
  }
}
```

#### Configuration Change Event

```json
{
  "type": "event",
  "event": "config_changed",
  "data": {}
}
```

---

### Error Handling

#### Error Message Format

```json
{
  "id": "original-request-id",
  "type": "error",
  "data": {
    "status": "error",
    "message": "Descriptive error message"
  }
}
```

#### Common Error Messages

- `"Missing data for relay control"`
- `"Invalid relay control data"`
- `"Failed to set relay"`
- `"Invalid relay ID: 17"`
- `"Invalid radio (must be 'A' or 'B')"`
- `"Invalid action (must be 'next' or 'previous')"`
- `"No frequency available for specified radio"`
- `"No antennas available for current frequency"`
- `"Config updates not supported via WebSocket. Use HTTP API."`
- `"Unknown action"`

---

## Examples

### Complete C++ Integration Example

```cpp
#include "AntennaSwitch.h"

class AntennaController {
private:
    antenna::AntennaSwitch antennaSwitch_;
    
public:
    bool initialize() {
        antenna::AntennaConfig config;
        config.enabled = true;
        config.transportType = antenna::TransportType::Auto;
        config.baseUrl = "http://192.168.1.100";
        config.wsUrl = "ws://192.168.1.100/ws";
        config.enableWebSocketEvents = true;
        config.timeoutMs = 2000;
        config.wsReconnectTimeoutMs = 3000;
        config.cacheExpirationMs = 30000;
        
        return antennaSwitch_.initialize(config);
    }
    
    void switchAntenna(uint64_t frequency) {
        if (!antennaSwitch_.isApiAvailable()) {
            ESP_LOGW(TAG, "Antenna switch API not available");
            return;
        }
        
        // Get current antennas
        auto antennas = antennaSwitch_.getCompatibleAntennas(frequency);
        ESP_LOGI(TAG, "Found %zu compatible antennas", antennas.size());
        
        // Switch to next antenna
        if (antennaSwitch_.switchToNextAntenna(frequency)) {
            ESP_LOGI(TAG, "Antenna switch successful");
            
            // Get new current antenna
            auto current = antennaSwitch_.getCurrentAntenna();
            if (current.id != -1) {
                ESP_LOGI(TAG, "Now using: %s", current.name.c_str());
            }
        } else {
            ESP_LOGW(TAG, "Antenna switch failed: %s", 
                     antennaSwitch_.getLastError().c_str());
        }
    }
};
```

### Direct WebSocket Client Example

```cpp
#include "AntennaWebSocketClient.h"

class DirectWebSocketController {
private:
    antenna::AntennaWebSocketClient wsClient_;
    
public:
    bool initialize() {
        antenna::WSAntennaConfig config;
        config.wsUrl = "ws://192.168.1.100/ws";
        config.autoReconnect = true;
        config.enableWebSocketEvents = true;
        
        if (!wsClient_.initialize(config)) {
            return false;
        }
        
        // Set up event callbacks
        wsClient_.setResponseCallback([this](const antenna::WSMessage& msg) {
            handleResponse(msg);
        });
        
        wsClient_.setEventCallback([this](const antenna::WSMessage& msg) {
            handleEvent(msg);
        });
        
        wsClient_.setErrorCallback([this](const antenna::WSMessage& msg) {
            handleError(msg);
        });
        
        // Connect and subscribe to events
        if (wsClient_.connect()) {
            std::vector<std::string> events = {
                "status_updates", "relay_state_changes"
            };
            wsClient_.subscribeToEvents(events);
            return true;
        }
        
        return false;
    }
    
    void switchAntenna() {
        if (wsClient_.isConnected()) {
            wsClient_.sendAntennaSwitch("A", "next", "antenna-switch-001");
        }
    }
    
    void controlRelay(int relay, bool state) {
        if (wsClient_.isConnected()) {
            wsClient_.sendRelayControl(relay, state, "relay-control-001");
        }
    }
    
private:
    void handleResponse(const antenna::WSMessage& msg) {
        ESP_LOGI(TAG, "Response to %s: %s", msg.id.c_str(), msg.data.c_str());
    }
    
    void handleEvent(const antenna::WSMessage& msg) {
        ESP_LOGI(TAG, "Event %s: %s", msg.event.c_str(), msg.data.c_str());
    }
    
    void handleError(const antenna::WSMessage& msg) {
        ESP_LOGW(TAG, "Error: %s", msg.data.c_str());
    }
};
```

---

## Error Codes

### HTTP Transport Errors

| Error Code | Description |
|------------|-------------|
| `ESP_FAIL` | General HTTP operation failure |
| `ESP_ERR_TIMEOUT` | HTTP request timeout |
| `ESP_ERR_NOT_FOUND` | DNS resolution failed |
| `ESP_ERR_INVALID_ARG` | Invalid HTTP parameters |
| `ESP_ERR_NO_MEM` | Insufficient memory |

### WebSocket Transport Errors

| Error Type | Description |
|------------|-------------|
| Connection Failed | Unable to establish WebSocket connection |
| Send Failed | Failed to send message to server |
| Parse Error | Invalid JSON message format |
| Protocol Error | WebSocket protocol violation |
| Authentication Failed | Server rejected connection |

### Application Errors

| Error Message | Cause | Resolution |
|---------------|--------|-----------|
| "HTTP client not initialized" | `initialize()` not called or failed | Call `initialize()` with valid config |
| "WebSocket not connected" | WebSocket connection lost | Check network, server availability |
| "Invalid relay number: X" | Relay number outside 1-16 range | Use valid relay number |
| "Invalid radio identifier: X" | Radio not "A" or "B" | Use "A" or "B" for radio parameter |
| "Invalid action: X" | Action not "next" or "previous" | Use valid action string |
| "Failed to create JSON object" | Memory allocation failure | Check available heap memory |
| "Message queue full" | Too many pending messages | Increase `messageQueueSize` or process messages faster |

---

## Performance Characteristics

### Latency Comparison

| Transport | Typical Latency | Best Case | Worst Case |
|-----------|-----------------|-----------|------------|
| WebSocket | < 10ms | 2-5ms | 50ms |
| HTTP | 50-200ms | 30ms | 500ms |

### Memory Usage

| Component | RAM Usage | Notes |
|-----------|-----------|--------|
| HTTP Client | ~8KB | ESP-IDF HTTP client |
| WebSocket Client | ~12-20KB | Additional for real-time features |
| Message Buffers | ~8KB | Per-message processing |
| Total Overhead | ~12-20KB | When WebSocket enabled |

### Throughput

| Metric | WebSocket | HTTP |
|--------|-----------|------|
| Messages/second | 100+ | 10-20 |
| Concurrent operations | Unlimited | Limited by connection pool |
| Real-time events | Yes | No |
| Connection reuse | Persistent | Keep-alive |

---

This API reference documents WebSocket-based antenna switching integration for ESP32 applications. The dual-transport design ensures maximum compatibility while providing optimal performance when WebSocket connectivity is available.