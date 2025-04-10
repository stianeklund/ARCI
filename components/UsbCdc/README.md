# UsbCdc Class Documentation

## Overview

The `UsbCdc` class provides a static interface for managing the USB CDC (Communications Device Class) functionality on the ESP32. It is designed to be used as a singleton, and all its methods are static.

## Features

- Initializes the USB CDC driver with custom descriptors.
- Redirects ESP_LOG output to the USB CDC interface, freeing up UART0 for other uses.
- Provides methods for writing data to the USB CDC interface.
- Manages the DTR (Data Terminal Ready), RTS (Request To Send), and CTS (Clear To Send) control pins.

## Class Members

### Public Methods

- **`init()`**: Initializes the USB CDC driver.
- **`enableUsbLogs()`**: Redirects `ESP_LOG` output to the USB CDC interface.
- **`disableUsbLogs()`**: Redirects `ESP_LOG` output back to UART0.
- **`writeData(const uint8_t* data, size_t length)`**: Writes a block of data to the USB CDC interface.
- **`writeString(const char* str)`**: Writes a null-terminated string to the USB CDC interface.
- **`initControlPins()`**: Initializes the DTR, RTS, and CTS GPIO pins.
- **`setDTR(bool state)`**: Sets the DTR signal state.
- **`setRTS(bool state)`**: Sets the RTS signal state.
- **`getCTS()`**: Gets the CTS signal state.
- **`getDTR()`**: Gets the current DTR state.
- **`getRTS()`**: Gets the current RTS state.

### Private Members

- **`m_initialized`**: A static boolean that is true if the USB CDC driver has been initialized.
- **`m_control_pins_initialized`**: A static boolean that is true if the control pins have been initialized.
- **`m_dtr_state`**: The current state of the DTR signal.
- **`m_rts_state`**: The current state of the RTS signal.
- **`m_last_cdc_connected`**: The last known connection status of the CDC interface.
- **`m_last_usb_mounted`**: The last known mount status of the USB device.

## Usage

1.  **Initialize the USB CDC driver** by calling `UsbCdc::init()` at the beginning of your application.
2.  **Enable USB logging** by calling `UsbCdc::enableUsbLogs()`.
3.  **Write data** to the USB CDC interface using `UsbCdc::writeData()` or `UsbCdc::writeString()`.
4.  **Manage control pins** using `UsbCdc::initControlPins()`, `UsbCdc::setDTR()`, `UsbCdc::setRTS()`, and `UsbCdc::getCTS()`.
