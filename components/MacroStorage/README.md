# MacroStorage

Persistent storage for user-defined CAT command macros using ESP-IDF NVS.

## Overview

MacroStorage is a C++ singleton providing CRUD operations for user-defined macros and F-button slot assignments. It handles NVS persistence with an in-memory cache for fast access.

## Features

- Up to 50 user-defined macros
- 12 F-button slots (6 buttons × short/long press)
- Automatic NVS persistence on changes
- Default macros loaded on first boot
- Factory reset capability

## API

### Initialization

```cpp
#include "MacroStorage.h"

// Initialize once at startup after nvs_flash_init()
esp_err_t err = storage::MacroStorage::instance().init();
```

### Macro Operations

```cpp
using namespace storage;

// Get macro by ID (1-50)
MacroDefinition macro{};
esp_err_t err = MacroStorage::instance().getMacro(macroId, macro);

// Set/update macro
MacroDefinition newMacro{};
std::strncpy(newMacro.name, "20M FT8", kMacroNameMaxLength - 1);
std::strncpy(newMacro.command, "FA00014074000|MD2|DA1", kMacroCommandMaxLength - 1);
err = MacroStorage::instance().setMacro(macroId, newMacro);

// Delete macro (also removes from any slot assignments)
err = MacroStorage::instance().deleteMacro(macroId);
```

Note: `setMacro()` automatically sets `enabled = true` and persists to NVS.

### Slot Assignments

```cpp
// Get all F-button assignments
std::array<uint8_t, kMacroSlotCount> assignments{};
esp_err_t err = MacroStorage::instance().getSlotAssignments(assignments);

// Assign macro to slot (slot: 0-11, macroId: 0=empty, 1-50=macro)
err = MacroStorage::instance().setSlotAssignment(slot, macroId);
```

### Utility

```cpp
bool ready = MacroStorage::instance().isInitialized();   // Check init status
uint8_t count = MacroStorage::instance().getCount();     // Number of enabled macros
uint8_t max = MacroStorage::getMaxCount();               // Always 50 (constexpr)
esp_err_t err = MacroStorage::instance().factoryReset(); // Reset to defaults
```

## Data Structures

```cpp
namespace storage {

constexpr size_t kMacroNameMaxLength = 32;
constexpr size_t kMacroCommandMaxLength = 64;
constexpr size_t kMacroCountMax = 50;
constexpr size_t kMacroSlotCount = 12;

struct MacroDefinition {
    char name[kMacroNameMaxLength]{};
    char command[kMacroCommandMaxLength]{};
    bool enabled = false;

    void clear();  // Reset all fields
};

}  // namespace storage
```

## Command Format

Commands are stored with `|` separator:
```
FA00014074000|MD2|DA1
```

RadioMacroManager splits on `|` and appends `;` terminator if missing.

## Default Macros

On first boot, these macros are loaded:
1. "20M FT8": `FA00014074000|MD2|DA1`
2. "10M FT8": `FA00028074000|MD2|DA1`
3. "30M CW": `FA00010105000|MD3`
4. "CQ Call": `KY CQ CQ DE LB1TI LB1TI K`
5. "Send Call": `KY LB1TI`

F1-F5 short press (slots 0-4) are assigned to macros 1-5. F6 and all long press slots (6-11) are unassigned.

## Error Handling

Common return values:
- `ESP_OK` - Success
- `ESP_ERR_NOT_FOUND` - Macro doesn't exist or not enabled
- `ESP_ERR_INVALID_ARG` - Invalid macro ID or slot index
  - Macro operations: ID must be 1-50
  - Slot assignment: ID 0-50 (0 = clear), slot 0-11
- `ESP_ERR_INVALID_STATE` - Storage not initialized

## Advanced API

```cpp
// Bulk load/save (for backup/restore)
MacroStorageData data{};
err = MacroStorage::instance().load(data);
err = MacroStorage::instance().save(data);  // Also updates internal cache
```

## Dependencies

- ESP-IDF NVS Flash
