#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * @brief USB CDC handler for debug output redirection
 * 
 * This class initializes and manages the USB CDC interface for debug logging,
 * redirecting logs from ESP_LOG to USB CDC instead of UART0, which allows
 * UART0 to be used exclusively for application communication.
 */
class UsbCdc {
public:
    /**
     * @brief Initialize the USB CDC driver
     * 
     * @return ESP_OK on success, otherwise an error code
     */
    static esp_err_t init();
    
    /**
     * @brief Redirect logs to USB CDC
     * 
     * @return ESP_OK on success, otherwise an error code
     */
    static esp_err_t enableUsbLogs();
    
    /**
     * @brief Redirect logs back to UART
     * 
     * @return ESP_OK on success, otherwise an error code
     */
    static esp_err_t disableUsbLogs();
    
    /**
     * @brief Write data to USB CDC interface
     * 
     * @param data Data to write
     * @param length Length of data to write
     * @param instance CDC instance (0 or 1, default 0)
     * @return ESP_OK on success, otherwise an error code
     */
    static esp_err_t writeData(const uint8_t* data, size_t length, uint8_t instance = 0);
    
    /**
     * @brief Write string to USB CDC interface
     * 
     * @param str String to write
     * @param instance CDC instance (0 or 1, default 0)
     * @return ESP_OK on success, otherwise an error code
     */
    static esp_err_t writeString(const char* str, uint8_t instance = 0);

    /**
     * @brief Explicitly flush pending TX data for a CDC instance
     *
     * Non-blocking (timeout 0). Safe to call frequently.
     */
    static void flush(uint8_t instance = 0);
    
    /**
     * @brief Initialize DTR, RTS, CTS GPIO pins
     * 
     * @return ESP_OK on success, otherwise an error code
     */
    static esp_err_t initControlPins();
    
    /**
     * @brief Set DTR (Data Terminal Ready) signal state
     * 
     * @param state true to assert DTR, false to deassert
     * @return ESP_OK on success, otherwise an error code
     */
    static esp_err_t setDTR(bool state);
    
    /**
     * @brief Set RTS (Request To Send) signal state
     * 
     * @param state true to assert RTS, false to deassert
     * @return ESP_OK on success, otherwise an error code
     */
    static esp_err_t setRTS(bool state);
    
    /**
     * @brief Get CTS (Clear To Send) signal state
     * 
     * @return true if CTS is asserted, false otherwise
     */
    static bool getCTS();
    
    /**
     * @brief Get current DTR state
     * 
     * @return true if DTR is asserted, false otherwise
     */
    static bool getDTR();
    
    /**
     * @brief Get current RTS state
     * 
     * @return true if RTS is asserted, false otherwise
     */
    static bool getRTS();

    /**
     * @brief Control whether CDC writes require DTR (host-open) state
     *
     * Many CAT apps for TS-590SG do not assert DTR. When disabled, data will be
     * accepted and buffered even if DTR is not asserted; it will be flushed when
     * the host opens the port or as the USB stack allows.
     */
    static void setRequireDtr(bool require) { m_require_dtr = require; }
    static bool getRequireDtr() { return m_require_dtr; }
    
    /**
     * @brief Check if USB CDC is connected and ready
     * 
     * @return true if connected and ready for communication
     */
    static bool isConnected();
    static bool isConnected(uint8_t instance);

    /**
     * @brief Attempt to wake up or reconnect to the host
     * 
     * @return ESP_OK if wakeup was attempted or not needed
     */
    static esp_err_t attemptReconnect();

    /**
     * @brief Get total immediate write headroom for a CDC endpoint.
     *
     * Adds the TinyUSB device FIFO space and our pending ring-buffer space so
     * callers can decide whether to enqueue without risking a drop.
     */
    static size_t availableForWrite(uint8_t instance = 0);

    // Register a callback invoked when TinyUSB signals RX data is available.
    // The callback should be lightweight and typically signal a semaphore/queue.
    using RxCallback = void (*)(uint8_t instance);
    static void setRxCallback(RxCallback cb);
    // Called by TinyUSB RX callback to notify upper layers
    static void notifyRxAvailable(uint8_t instance);

    // Optional TX mirror callback invoked whenever data is written to CDC.
    // Allows observers (e.g., TCP bridge) to mirror outbound frames.
    using TxCallback = void (*)(uint8_t instance, const uint8_t* data, size_t length);
    static TxCallback setTxCallback(TxCallback cb);
    static void notifyTxMirrored(uint8_t instance, const uint8_t* data, size_t length);

    // Backpressure helpers
    static bool shouldThrottleWrite(uint8_t instance);
    static void notifyWriteSuccess(uint8_t instance);
    static void notifyWriteBackpressure(uint8_t instance);
    static void resetBackpressure(uint8_t instance);
    
private:
    // Lightweight TX ring buffer per CDC instance to absorb short writes
    static constexpr size_t TX_BUFFER_SIZE = 1024;
    static uint8_t m_tx_buf[2][TX_BUFFER_SIZE];
    static size_t m_tx_head[2];
    static size_t m_tx_tail[2];
    static SemaphoreHandle_t m_tx_mutex[2];

    static size_t txFreeSpace(uint8_t instance);
    static size_t txBuffered(uint8_t instance);
    static size_t txContigAvailable(uint8_t instance);
    static void   txEnqueue(uint8_t instance, const uint8_t* data, size_t len);
    static size_t txDequeueToUsb(uint8_t instance);
    static bool m_initialized;
    static bool m_control_pins_initialized;
    static bool m_dtr_state;
    static bool m_rts_state;
    static uint64_t m_writeBlockUntilUs[2];
    static uint8_t m_backpressureLevel[2];
    static bool m_last_cdc_connected[2];
    static bool m_last_usb_mounted[2];
    static RxCallback m_rx_callback;
    static TxCallback m_tx_callback;
    static bool m_require_dtr;  // if false, don't drop writes when DTR is not asserted
};
