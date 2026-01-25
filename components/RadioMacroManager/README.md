# RadioMacroManager

Unified macro execution engine for all macro types - both semantic (hardcoded) and user-defined (stored).

## Overview

RadioMacroManager provides a single execution engine for:

1. **Semantic Macros**: Complex, stateful operations with conditional logic
   - Transverter mode (EX056, antenna, DRV out configuration)
   - Split operation (VFO copy, FR/FT commands)
   - Band change (planned)
   - Contest mode (planned)

2. **User-Defined Macros**: Stored CAT command sequences
   - Up to 50 macros stored in NVS via MacroStorage
   - F-button slot assignments (F1-F6 short/long press = 12 slots)
   - Pipe-separated commands executed with proper delays

## API

### Semantic Macros

```cpp
RadioMacroManager macros(radioManager);

// Enable transverter mode
macros.executeTransverterMacro(true);

// Enable split with VFO A copied to B
macros.executeSplitMacro(true, true);
```

### User-Defined Macros

```cpp
// Execute macro by ID (1-50)
esp_err_t result = macros.executeUserMacro(5);

// Execute macro from F-button slot (0-11)
esp_err_t result = macros.executeSlot(0);  // F1 short press
```

## Features

- Commands dispatched through CATHandler for proper state tracking
- Configurable inter-command delay (50ms for user macros, 20ms for semantic)
- State validation before execution
- Error reporting via `getLastError()` and `getLastStatus()`

## Command Format

User macros store commands with `|` separator:
```
FA00014074000|MD2|DA1
```

At execution, each segment is dispatched with `;` terminator and 50ms inter-command delay.

## Dependencies

- RadioCore (RadioManager, CATHandler)
- MacroStorage (NVS persistence for user macros)
