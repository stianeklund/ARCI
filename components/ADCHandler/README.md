# ADCHandler

Handles ADC operations from volume potentiometers and similar analog inputs.

Note: This is currently not actively used by the ARCI project as I use encoders instead.
Since we're operating a remote radio anyway, having pots locally makes little sense because the host ultimately "decides" the value.
Encoders are much more convenient in this case.

## Overview

Provides methods for initializing the ADC, reading and mapping ADC values from specific GPIO pins, detecting changes, and updating radio state via RadioManager.

## Attributes

- `m_radioManager`: Pointer to RadioManager for state updates
- `m_afPin`, `m_rfPin`, `m_ifPin`, `m_ritXitPin`: GPIO pins for ADC channels
- `m_hasChanged`: Flag indicating significant value changes
- `m_adc1_handle`: Handle to ADC unit for one-shot readings
- `m_previousValues`, `m_currentValues`: Stored ADC values

## Constants

- `CHANGE_THRESHOLD`: Minimum change required to consider significant

## Methods

### Constructor
```cpp
ADCHandler::ADCHandler(radio::RadioManager* radioManager)
```
- **Parameters**: `radioManager` - Pointer to RadioManager instance
- **Behavior**: Sets up ADC for one-shot readings on specified GPIO pins

### Destructor
```cpp
ADCHandler::~ADCHandler()
```
- **Description**: Cleans up resources when the `ADCHandler` object is destroyed.
- **Behavior**: Deletes the ADC unit handle to free resources.

### update
```cpp
void ADCHandler::update()
```
- **Description**: Reads current ADC values from the configured pins, maps them to a specified range, and checks for significant changes.
- **Behavior**:
    - Stores previous ADC values.
    - Reads and maps ADC values from `m_afPin`, `m_rfPin`, `m_ifPin`, and `m_ritXitPin`.
    - Updates `m_hasChanged` based on significant changes in any of the ADC values.

### hasChanged
```cpp
bool ADCHandler::hasChanged() const
```
- **Description**: Returns whether any ADC values have changed significantly since the last update.
- **Returns**:
    - `true` if any value has changed significantly, `false` otherwise.

### afHasChanged
```cpp
bool ADCHandler::afHasChanged() const
```
- **Description**: Checks if the AF gain (analog front-end gain) value has changed significantly.
- **Returns**:
    - `true` if the AF gain has changed significantly, `false` otherwise.

### rfHasChanged
```cpp
bool ADCHandler::rfHasChanged() const
```
- **Description**: Checks if the RF gain (radio frequency gain) value has changed significantly.
- **Returns**:
    - `true` if the RF gain has changed significantly, `false` otherwise.

### ifShiftHasChanged
```cpp
bool ADCHandler::ifShiftHasChanged() const
```
- **Description**: Checks if the IF shift (intermediate frequency shift) value has changed significantly.
- **Returns**:
    - `true` if the IF shift has changed significantly, `false` otherwise.

### ritXitHasChanged
```cpp
bool ADCHandler::ritXitHasChanged() const
```
- **Description**: Checks if the RIT/XIT (receive incremental tuning/xmit incremental tuning) value has changed significantly.
- **Returns**:
    - `true` if the RIT/XIT value has changed significantly, `false` otherwise.

### getValues
```cpp
ADCHandler::ADCValues ADCHandler::getValues() const
```
- **Description**: Retrieves the current ADC values.
- **Returns**:
    - A structure containing the current AF gain, RF gain, IF shift, and RIT/XIT values.

### readAndMap
```cpp
int ADCHandler::readAndMap(gpio_num_t pin, int minVal, int maxVal) const
```
- **Description**: Reads an ADC value from a specified GPIO pin and maps it to a given range.
- **Parameters**:
    - `pin`: The GPIO pin number to read from.
    - `minVal`, `maxVal`: The minimum and maximum values of the mapped range.
- **Returns**:
    - The mapped ADC value.

### gpioToAdcChannel
```cpp
adc_channel_t ADCHandler::gpioToAdcChannel(gpio_num_t gpio)
```
- **Description**: Converts a GPIO pin number to its corresponding ADC channel.
- **Parameters**:
    - `gpio`: The GPIO pin number.
- **Returns**:
    - The ADC channel corresponding to the given GPIO pin.

### isSignificantChange
```cpp
bool ADCHandler::isSignificantChange(int current, int previous) const
```
- **Description**: Determines if a change between two values is significant.
- **Parameters**:
    - `current`, `previous`: The current and previous values to compare.
- **Returns**:
    - `true` if the absolute difference between the values is greater than or equal to `CHANGE_THRESHOLD`, `false` otherwise.

### handleChange
```cpp
void ADCHandler::handleChange()
```
- **Description**: Handles changes in ADC values by updating RadioManager state
- **Behavior**:
    - Checks if significant changes occurred since last update
    - Updates corresponding settings via RadioManager

## Notes

- Uses ESP-IDF ADC driver for one-shot readings
- Significant changes determined by `CHANGE_THRESHOLD`
- Includes debounce mechanism to prevent rapid updates