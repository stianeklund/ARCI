# unit_test

Unity test framework infrastructure for ESP-IDF component testing.

## Purpose

Provides common test infrastructure, utilities, and configuration for all component unit tests. Enables running tests via `idf.py test` with consistent setup and teardown.

## Contents

- **tests_main.cpp**: Unity test runner entry point
- **test_tracker.h**: Test execution tracking and statistics
- **test_hooks.h**: Setup/teardown hooks for test suites
- **unity_config.h**: Unity framework configuration
- **test_output_capture.cpp**: Utilities for capturing stdout/stderr in tests

## Usage

Component test structure:
```
components/MyComponent/
  ├── CMakeLists.txt
  ├── MyComponent.cpp
  ├── include/MyComponent.h
  └── test/
      ├── CMakeLists.txt
      └── test_my_component.cpp
```

Test CMakeLists.txt:
```cmake
idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "." "../include"
    REQUIRES unity unit_test MyComponent
)
```

Test file:
```cpp
#include "unity.h"
#include "test_hooks.h"
#include "MyComponent.h"

TEST_CASE("MyComponent initializes", "[mycomponent]") {
    MyComponent comp;
    TEST_ASSERT_EQUAL(ESP_OK, comp.initialize());
}
```

## Running Tests

```bash
# Run all tests
idf.py test

# Run specific component tests
idf.py test -t MyComponent

# Run tests matching pattern
idf.py test -n "test_parse*"
```

## Kconfig

Options in `Kconfig`:
- `UNIT_TEST_VERBOSE`: Enable verbose test output
- Test-specific settings per component

## Dependencies

- ESP-IDF Unity framework

## Design Notes

- Minimal test infrastructure to avoid coupling
- Shared utilities reduce boilerplate in component tests
- FreeRTOS-aware: handles task cleanup between tests
