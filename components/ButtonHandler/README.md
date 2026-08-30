# ButtonHandler

Handles button inputs and interfaces with RadioManager for radio control operations.

## Overview

Manages multiple button inputs (mode, band, split, etc.) and triggers appropriate radio operations via RadioManager and RadioMacroManager.

## Attributes

- `m_radioManager`: Pointer to RadioManager for radio operations
- `m_macroManager`: Pointer to RadioMacroManager for complex command sequences
- Button instances: `m_modeButton`, `m_bandButton`, `m_tfSetButton`, `m_splitButton`, `m_onOffButton`, `m_functionButton1-4`, `m_transverterMacroButton`

## Methods

### Constructor
```cpp
ButtonHandler::ButtonHandler(RadioManager* radioManager, RadioMacroManager* macroManager)
```
- **Parameters**: `radioManager`, `macroManager` - pointers to respective managers
- **Behavior**: Sets up Button instances and initializes GPIO pins

### handleAllButtons
```cpp
void ButtonHandler::handleAllButtons()
```
- **Description**: Calls individual methods to handle each button's events.
- **Behavior**:
    - Calls `handleModeButton`, `handleBandButton`, `handleTfSetButton`, `handleSplitButton`, `handleOnOffButton`, and `handleTransverterMacroButton`.

### updateButtonStates
```cpp
void ButtonHandler::updateButtonStates()
```
- **Description**: Updates the state of all buttons by calling their respective `update` methods.
- **Behavior**:
    - Calls `update` on each button instance to read the current state, apply debouncing, and check for changes.

### handleModeButton
```cpp
void ButtonHandler::handleModeButton()
```
- **Description**: Handles events related to the mode button
- **Behavior**:
    - Short press: Toggles data mode, turns off processor if in data mode
    - Long press: Triggers rotary button click action

### handleBandButton
- Short press: Requests VFO A frequency and decodes band
- Long press: Changes current band

### handleTfSetButton
- Toggles TF setting via RadioManager

### handleSplitButton
- Toggles split mode via RadioManager helpers (`enableSplit`/`disableSplit`)

### handleOnOffButton
- Toggles radio ON/OFF state via RadioManager

### handleFunctionButton1-4
- Placeholder functions for user-configurable actions

### handleTransverterMacroButton
- Toggles transverter state via RadioMacroManager

## Notes

- Uses Button class instances for debounced press/release/long-press detection
- Actions performed via RadioManager helpers and RadioMacroManager
- **Band stacking memory belongs to the radio.** A normal BND+ or BND- press selects the target radio band memory, then sends `MD;` only to read its selected mode. ARCI neither persists a per-band mode preference nor sends an `MDx;` setting that could overwrite the selected radio slot. Long press cycles the current band's radio slots.
- **Boolean toggles are atomic.** Buttons that flip an on/off radio setting (RF attenuator, preamp, RIT, XIT, VOX, antenna, TX-ATU) call `RadioManager::dispatchToggle(handler, ToggleTarget)` rather than reading state and inverting locally. `dispatchToggle` resolves the read+invert+dispatch as a single critical section under the dispatch lock, so a concurrent CAT client (or radio answer) cannot make the press a no-op or flip the wrong way.
- **PROC is a three-state atomic cycle.** A normal PROC press selects `OFF → PROC → DATA → OFF` through `RadioManager::cycleProcessorDataMode`; every transition disables the active mode before enabling the other, so PROC and DATA cannot coexist. A long press retains the processor-level popup, which now contains only input and output levels.
- **Other multi-state cycles are atomic too.** Buttons that step through several states — noise reduction (`NR`) and noise blanker (`NB`), including the NB popup-open active cycle — call `RadioManager::dispatchCycleLocked(handler, CycleTarget)`, the cycle analogue of `dispatchToggle` (read current → advance → dispatch, under the lock). Long-press/absolute actions (e.g. `AC111;` start-tuning) keep their own logic here. There is no AGC cycle button; the former unwired `triggerFunctionButton1/2/4` were removed.
