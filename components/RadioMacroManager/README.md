# RadioMacroManager

Executes complex sequences of CAT commands as atomic macro operations.
This technically overlaps a bit with the new macro functionality..
TODO: Fix that :P

## Overview

RadioMacroManager provides high-level semantic operations that coordinate multiple CAT commands. This abstraction layer allows complex radio configurations without exposing CAT protocol details to UI code.

## Available Macros

| Macro                       | Description                                          |
|-----------------------------|------------------------------------------------------|
| `executeTransverterMacro()` | Configure transverter mode (EX056, antenna, DRV out) |
| `executeSplitMacro()`       | Enable/disable split with optional VFO copy          |
| `executeBandChangeMacro()`  | Band change with filter/power/antenna (planned)      |
| `executeContestModeMacro()` | Contest preset configuration (planned)               |

## Usage

```cpp
RadioMacroManager macros(radioManager);

// Enable transverter mode
macros.executeTransverterMacro(true);

// Enable split with VFO A copied to B
macros.executeSplitMacro(true, true);
```

## Features

- Commands dispatched through CATHandler for proper state tracking
- Configurable inter-command delay (default 20ms)
- State validation before execution
- Error reporting via `getLastError()`

## Dependencies

- RadioCore (RadioManager, CATHandler)
