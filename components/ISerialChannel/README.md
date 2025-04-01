# ISerialChannel

Abstract interface for serial communication channels used by CAT handlers.

## Purpose

Defines a common interface for serial I/O operations, enabling polymorphic use of different serial backends (UART, USB CDC, TCP) by the CAT command system.

## Interface

```cpp
class ISerialChannel {
    virtual esp_err_t write(const std::string& data) = 0;
    virtual esp_err_t write(std::string_view data) = 0;
    virtual bool available() = 0;
    virtual size_t getBytesAvailable() = 0;
    virtual std::string read(size_t maxLen = 0) = 0;
    virtual std::string_view getMessageView() = 0;
};
```

## Implementations

- **SerialHandler**: UART-based implementation
- **UsbCdc**: USB CDC-ACM implementation
- **TcpCatBridge**: TCP socket implementation (internal)

## Usage

Used by `CATHandler` and `RadioManager` for source-agnostic serial communication.

## Dependencies

- ESP-IDF (esp_err_t types)

## Design Notes

- Zero-copy `getMessageView()` for high-performance parsing
- Supports both string and string_view for flexibility
