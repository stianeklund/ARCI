# Diagnostics

System diagnostics and health monitoring for ESP32 firmware.

## Purpose

Collects and reports system health metrics including heap usage, task statistics, WiFi status, and component states. Used for debugging, performance analysis, and runtime monitoring.

## Features

- **Heap monitoring**: Free/used memory tracking per capability
- **Task statistics**: CPU usage, stack high water marks
- **WiFi diagnostics**: Connection state, RSSI, IP address
- **Uptime tracking**: System runtime and reboot counters
- **Formatted reports**: Human-readable diagnostic output
- **Performance profiling**: Execution time measurements

## Usage

```cpp
#include "Diagnostics.h"

Diagnostics diag;

// Print full system report
diag.printSystemReport();

// Get specific metrics
size_t freeHeap = diag.getFreeHeap();
float cpuUsage = diag.getCpuUsage();

// Log to console periodically
diag.logHeapStatus();
diag.logTaskStatistics();
```

## Reported Metrics

- Heap: free/min/largest block (DRAM, SPIRAM, DMA-capable)
- Tasks: CPU %, stack usage, state
- WiFi: SSID, RSSI, IP, connection time
- Uptime: runtime, boot count
- Component health: radio link, USB status

## Dependencies

- ESP-IDF (FreeRTOS, heap caps, WiFi)

## Design Notes

- Minimal overhead when not actively profiling
- Safe to call from any task context
- Output formatted for serial console viewing
