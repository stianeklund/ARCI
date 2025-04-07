# SerialHandler Class Documentation

## Overview

The `SerialHandler` class provides an interface for managing UART (Universal Asynchronous Receiver/Transmitter) communication in embedded systems. It includes functionalities to initialize and configure UART, handle incoming data events, process received messages, and send messages.

## Member Functions

### Constructor and Destructor

- **Constructor (`SerialHandler(uart_port_t uart_num)`)**: Initializes the `SerialHandler` with a specific UART port.
  - `uart_port_t uart_num`: The UART port number to be used.

- **Destructor (`~SerialHandler()`)**: Cleans up resources such as task handles and queues, and deletes the UART driver.

### Setup Functions

- **setupUart(int baud_rate, int rx_pin = UART_PIN_NO_CHANGE, int tx_pin = UART_PIN_NO_CHANGE)**: Configures and initializes the UART with given baud rate and optional RX/TX pins.
  - `int baud_rate`: The desired baud rate for UART communication.
  - `int rx_pin` (optional): The GPIO pin number for receiving data. Defaults to `UART_PIN_NO_CHANGE`.
  - `int tx_pin` (optional): The GPIO pin number for transmitting data. Defaults to `UART_PIN_NO_CHANGE`.
  - Returns: An `esp_err_t` error code indicating the success or failure of the setup.

### Send Functions

- **sendMessage(const std::string_view message)**: Sends a single string message over UART.
  - `const std::string_view message`: The message to be sent.
  - Returns: An `esp_err_t` error code indicating the success or failure of sending.

- **sendMessage(std::string_view message1, std::string_view message2)**: Combines two messages and sends them as a single message.
  - `std::string_view message1`: The first part of the message.
  - `std::string_view message2`: The second part of the message.
  - Returns: An `esp_err_t` error code indicating the success or failure of sending.

### Receive Functions

- **hasMessage() const**: Checks if there is a complete message ready to be processed.
  - Returns: A boolean value indicating whether a message is available.

- **getMessage()**: Retrieves and clears a complete message from the internal buffer.
  - Returns: A `std::pair<esp_err_t, std::string_view>` containing an error code and the message. If no message is available, returns `ESP_FAIL` with an empty string.

### Internal Functions

- **uartEventTask(void* pvParameters)**: The task function that handles UART events such as data reception, buffer overflow, etc.
  - This function continuously monitors the UART queue for events and processes them accordingly.

- **processReceivedData(const uint8_t* data, size_t len)**: Processes incoming data bytes by appending them to an internal buffer and checking for message termination.
  - `const uint8_t* data`: The received data bytes.
  - `size_t len`: The number of bytes received.

- **processMessage(std::string_view message)**: Parses and processes a complete message. Currently, it checks if the message is a frequency setting command (`SET_FREQ:`) and responds accordingly.
  - `std::string_view message`: The message to be processed.

## Usage

1. **Create an instance of `SerialHandler`**:
   ```cpp
   SerialHandler serialHandler(UART_NUM_1);
   ```

2. **Set up the UART** with a desired baud rate and optional RX/TX pins:
   ```cpp
   esp_err_t ret = serialHandler.setupUart(9600, 34, 35);
   if (ret != ESP_OK) {
       // Handle error
   }
   ```

3. **Send messages**:
   ```cpp
   ret = serialHandler.sendMessage("Hello UART");
   if (ret != ESP_OK) {
       // Handle error
   }
   ```

4. **Check for and receive messages**:
   ```cpp
   while (serialHandler.hasMessage()) {
       auto [err, message] = serialHandler.getMessage();
       if (err == ESP_OK) {
           // Process the received message
       } else {
           // Handle error
       }
   }
   ```

This class simplifies UART communication by abstracting away much of the low-level details and providing a 
high-level interface for sending and receiving messages. 