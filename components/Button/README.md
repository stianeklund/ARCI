# Button Class Documentation

## Overview
The `Button` class is designed to handle button input. It provides methods for initializing the GPIO pin, 
reading button state, detecting changes in button press & release events, and determining if a long press has occurred.

## Class Members

### Attributes
- `m_pin`: The GPIO pin number corresponding to the button.
- `m_debounceTime`: The debounce time in milliseconds used to filter out noise from the button signal.
- `m_longPressTime`: The time threshold in milliseconds to determine if a button press is a long press.
- `m_state`, `m_lastState`: Booleans representing the current and previous states of the button.
- `m_changed`: A boolean flag indicating whether the button state has changed since the last update.
- `m_lastDebounceTime`: The timestamp in milliseconds when the last debounce event occurred.
- `m_pressedTime`: The timestamp in milliseconds when the button was first pressed.
- `m_longPressed`: A boolean flag indicating if the button is currently in a long press state.

## Methods

### Constructor
```cpp
Button::Button(gpio_num_t pin, uint32_t debounceTime, uint32_t longPressTime)
```
- **Description**: Initializes the `Button` object.
- **Parameters**:
    - `pin`: The GPIO pin number where the button is connected.
    - `debounceTime`: The debounce time in milliseconds to filter out noise from the button signal.
    - `longPressTime`: The time threshold in milliseconds to determine if a button press is a long press.
- **Behavior**: Configures the specified GPIO pin as an input with pull-up resistor.

### update
```cpp
void Button::update()
```
- **Description**: Reads the current state of the button, applies debouncing, and updates internal flags for button presses, releases, and long presses.
- **Behavior**:
    - Reads the current state of the button (inverted because of pull-up resistor).
    - Updates the `m_lastDebounceTime` if the button state has changed.
    - Debounces the button state by checking if the time since the last debounce event exceeds `m_debounceTime`.
    - Sets the `m_changed` flag and updates the `m_pressedTime` or `m_longPressed` status based on the new button state.

### isPressed
```cpp
bool Button::isPressed() const
```
- **Description**: Checks if the button is currently pressed.
- **Returns**:
    - `true` if the button is currently pressed, `false` otherwise.

### wasPressed
```cpp
bool Button::wasPressed()
```
- **Description**: Determines if the button was just pressed (edge detection).
- **Returns**:
    - `true` if the button state changed from not pressed to pressed since the last update, `false` otherwise.
- **Behavior**: Resets the `m_changed` flag after detecting a press.

### wasReleased
```cpp
bool Button::wasReleased()
```
- **Description**: Determines if the button was just released (edge detection).
- **Returns**:
    - `true` if the button state changed from pressed to not pressed since the last update, `false` otherwise.
- **Behavior**: Resets the `m_changed` flag after detecting a release.

### isLongPressed
```cpp
bool Button::isLongPressed() const
```
- **Description**: Checks if the button is currently in a long press state.
- **Returns**:
    - `true` if the button has been pressed for longer than `m_longPressTime`, `false` otherwise.

### pressedTime
```cpp
int64_t Button::pressedTime() const
```
- **Description**: Retrieves the timestamp in milliseconds when the button was first pressed.
- **Returns**:
    - The timestamp of the initial press event.

## Notes
- The class uses ESP-IDF GPIO driver for reading button state.
- Debouncing is applied to filter out noise from mechanical switches.
- Long press detection is based on a defined time threshold.