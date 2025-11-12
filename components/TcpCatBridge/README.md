# TCP CAT Bridge Component

## Overview

The TCP CAT Bridge provides network access to the radio's CAT (Computer Aided Transceiver) command interface over TCP/IP. It enables remote CAT control applications to send commands and receive responses over WiFi.

## Features

- **Lightweight Design**: Minimal CPU and memory overhead
- **Single Client Enforcement**: One active client per bridge instance (new connections replace existing)
- **Event-driven I/O**: Uses `select()` for efficient multiplexing, ~0% CPU when idle
- **Dual Bridge Support**: Two independent TCP ports for separate CAT sessions
- **Callback-based Routing**: Incoming CAT frames dispatched via registered callback
- **Graceful Handling**: Proper client connection/disconnection management

## Architecture

```
┌─────────────┐         ┌──────────────┐         ┌──────────────┐
│  TCP Client │ <-----> │ TcpCatBridge │ <-----> │ RadioManager │
│  (Network)  │         │   (Bridge)   │         │ (CAT Router) │
└─────────────┘         └──────────────┘         └──────────────┘
     ^                         ^                         ^
     │                         │                         │
  socket I/O              select() mux            frame callback
```

### Data Flow

**TCP → Radio (Inbound):**
1. TCP client sends CAT command → `recv()` on client socket
2. Bridge parses semicolon-delimited frames
3. Each complete frame invokes `incomingFrameCallback`
4. RadioManager dispatches to appropriate CATHandler

**Radio → TCP (Outbound):**
1. RadioManager calls `sendToActiveClient()` with response
2. Bridge sends response to connected TCP client via `send()`

## Configuration

All settings are configurable via `menuconfig`:

```
Component config → TCP CAT Bridge Configuration
```

### Available Options

| Option                             | Default | Description                     |
|------------------------------------|---------|---------------------------------|
| `TCP_CAT_BRIDGE_ENABLE`            | Yes     | Enable/disable the bridge       |
| `TCP_CAT_BRIDGE_PORT_0`            | 7373    | TCP port for bridge 0           |
| `TCP_CAT_BRIDGE_PORT_1`            | 7374    | TCP port for bridge 1           |
| `TCP_CAT_BRIDGE_ENABLE_DUAL`       | Yes     | Enable second bridge instance   |
| `TCP_CAT_BRIDGE_MAX_CLIENTS`       | 2       | Maximum client slots per bridge |
| `TCP_CAT_BRIDGE_BUFFER_SIZE`       | 256     | RX buffer size per client       |
| `TCP_CAT_BRIDGE_TASK_STACK_SIZE`   | 4096    | Bridge task stack (bytes)       |
| `TCP_CAT_BRIDGE_TASK_PRIORITY`     | 5       | FreeRTOS task priority          |
| `TCP_CAT_BRIDGE_SELECT_TIMEOUT_MS` | 100     | Select timeout (ms)             |

## Resource Usage

**CPU:**
- Idle: ~0% (blocked in `select()`)
- Active: 2-4% (data forwarding)

**Memory:**
- Per client: ~7-9 KB heap
- Task stack: 4 KB (default)
- Total per bridge: ~20 KB

## Usage

### Initialization

```cpp
#ifdef CONFIG_TCP_CAT_BRIDGE_ENABLE
    // Create bridge instance
    tcpCatBridge0 = std::make_unique<tcp_cat_bridge::TcpCatBridge>(
        CONFIG_TCP_CAT_BRIDGE_PORT_0,  // TCP port
        0                               // Bridge ID
    );

    // Create dedicated CAT handler for this bridge
    tcp0CatHandler = std::make_unique<radio::CATHandler>(
        radioManager.getCommandDispatcher(),
        radioManager,
        radioSerial,
        usbSerial,
        radio::CommandSource::Tcp0
    );

    // Wire incoming frames to CAT handler
    tcpCatBridge0->setIncomingFrameCallback(
        [&](std::string_view frame) {
            radioManager.dispatchMessage(*tcp0CatHandler, frame);
        });

    // Register bridge for outbound responses
    radioManager.setTcp0Bridge(tcpCatBridge0.get());

    // Start the bridge
    tcpCatBridge0->start();
#endif
```

## Diagnostics

### Logging

```
I (12345) TcpCatBridge: TCP-CAT bridge started for bridge 0 on port 7373
I (12346) TcpCatBridge: Client 0 connected to bridge 0 from 192.168.1.50:54321
I (12347) TcpCatBridge: Client 0 disconnected from bridge 0 (RX=1234, TX=5678)
```

### Statistics API

```cpp
ESP_LOGI(TAG, "Active clients: %d", tcpCatBridge->getClientCount());
ESP_LOGI(TAG, "Total RX: %llu bytes", tcpCatBridge->getBytesReceived());
ESP_LOGI(TAG, "Total TX: %llu bytes", tcpCatBridge->getBytesSent());
```

## Implementation Details

### Thread Safety

- Single-threaded design (one bridge task handles all I/O)
- Callback invocation protected by mutex
- Statistics use atomic operations

### Frame Parsing

- CAT commands are semicolon-delimited (`;`)
- Control characters (0x00-0x1F except `;`) are stripped
- Buffer overflow handled by dropping oldest data

### Single Client Policy

When a new client connects, any existing client is disconnected. This prevents command interleaving from multiple sources on the same bridge.

## Limitations

- **No authentication**: Plain TCP, no encryption or access control
- **Single client per bridge**: New connections replace existing
- **Best-effort delivery**: No flow control or guaranteed delivery