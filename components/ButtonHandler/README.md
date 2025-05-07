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