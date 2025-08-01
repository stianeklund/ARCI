# FrequencyFormatter

Utility functions for formatting radio frequencies in various display formats.

## Purpose

Provides consistent frequency formatting and parsing for display, logging, and CAT command generation. Handles frequency representation with proper decimal placement and unit conversion.

## Key Features

- Convert Hz to formatted strings (e.g., "14.150.000")
- Parse frequency strings to Hz
- Support for different display formats (MHz, kHz with/without separators)
- Validation of frequency ranges

## Usage

```cpp
#include "FrequencyFormatter.h"

// Format frequency for display
uint32_t freq = 14150000;  // Hz
std::string display = FrequencyFormatter::format(freq);  // "14.150.000"

// Parse frequency from user input
auto freqOpt = FrequencyFormatter::parse("14.150");
if (freqOpt) {
    uint32_t hz = *freqOpt;  // 14150000
}
```

## Dependencies

- CommonConstants (for frequency ranges)

## Design Notes

- Header-only implementation with constexpr functions where possible
- Zero heap allocations for common formatting operations
- Locale-independent formatting for CAT protocol compatibility
