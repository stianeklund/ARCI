# CommonConstants

Shared radio constants and type definitions used across all components.

## Purpose

Provides common enumerations, constants, and type definitions for radio operations to ensure consistency and type safety across the codebase.

## Key Contents

- **radio_constants.h**: Shared constants and enums
  - Frequency ranges and band definitions
  - Common radio parameters
  - Type definitions for consistent usage

## Usage

```cpp
#include "radio_constants.h"

// Access shared constants
uint32_t minFreq = RADIO_MIN_FREQUENCY;
```

## Dependencies

None - foundation component.

## Build

```cmake
idf_component_register(
    SRCS "radio_constants.cpp"
    INCLUDE_DIRS "include"
)
```
