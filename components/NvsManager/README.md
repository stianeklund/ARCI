# NvsManager

Non-volatile storage manager for persistent radio settings using ESP-IDF NVS.

## Purpose

Provides type-safe persistence for radio configuration, user preferences, and state that must survive reboots. Wraps ESP-IDF NVS API with error handling and namespace management.

## Features

- **Type-safe operations**: Templated get/set for primitives
- **Namespace management**: Isolated storage areas per component
- **Error handling**: esp_err_t return codes with logging
- **Default values**: Automatic fallback when keys don't exist
- **Kconfig integration**: Configure NVS partition via menuconfig

## Usage

```cpp
#include "NvsManager.h"

NvsManager nvs("radio_config");

// Store settings
nvs.setUInt32("vfo_a_freq", 14150000);
nvs.setUInt8("power_level", 100);

// Load settings with defaults
uint32_t freq = nvs.getUInt32("vfo_a_freq", 14000000);
uint8_t power = nvs.getUInt8("power_level", 50);

// Commit changes
nvs.commit();
```

## Stored Data Examples

- VFO frequencies (A/B)
- Operating mode per band
- Filter settings
- User preferences (backlight, contrast)
- Calibration data

## Dependencies

- ESP-IDF NVS

## Configuration

Kconfig options in `Kconfig`:
- `NVS_NAMESPACE`: Default namespace prefix
- Partition table must include NVS partition

## Design Notes

- Lazy write: changes buffered until `commit()`
- Read-through cache for frequently accessed keys (future enhancement)
- Does not use exceptions (ESP-IDF compatible)
