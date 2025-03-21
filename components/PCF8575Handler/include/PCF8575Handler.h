#pragma once

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include <cstdint>
#include <functional>

// Forward declaration
class TCA9548Handler;

/**
 * @brief PCF8575 16-bit I2C GPIO Expander Handler with Interrupt Support
 *
 * Manages PCF8575 I/O expander for EC11E dual-axis encoder #2.
 * Uses interrupt-driven change detection for minimal latency.
 * Supports quadrature encoder decoding and switch debouncing.
 */
class PCF8575Handler {
public:
    /**
     * @brief Pin change callback
     * @param pinNumber Pin that changed (0-15)
     * @param newState New pin state (true=high, false=low)
     */
    using PinChangeCallback = std::function<void(uint8_t pinNumber, bool newState)>;

    /**
     * @brief Construct PCF8575 handler
     * @param i2cAddr I2C address (default 0x20)
     * @param interruptPin ESP32 GPIO for interrupt line (default GPIO_NUM_NC for polling)
     */
    explicit PCF8575Handler(uint8_t i2cAddr = 0x20, gpio_num_t interruptPin = GPIO_NUM_NC);
    ~PCF8575Handler();

    /**
     * @brief Initialize PCF8575 on given I2C bus
     * @param i2cBusHandle Handle to initialized I2C master bus
     * @return ESP_OK on success
     */
    esp_err_t initialize(i2c_master_bus_handle_t i2cBusHandle);

    /**
     * @brief Set pin mode (input=true, output=false)
     * @param pinNumber Pin number (0-15)
     * @param isInput true for input, false for output
     * @return ESP_OK on success
     */
    esp_err_t setPinMode(uint8_t pinNumber, bool isInput);

    /**
     * @brief Write to output pin
     * @param pinNumber Pin number (0-15)
     * @param state Pin state (true=high, false=low)
     * @return ESP_OK on success
     */
    esp_err_t writePin(uint8_t pinNumber, bool state);

    /**
     * @brief Read input pin
     * @param pinNumber Pin number (0-15)
     * @param state Output: current pin state
     * @return ESP_OK on success
     */
    esp_err_t readPin(uint8_t pinNumber, bool& state);

    /**
     * @brief Read all 16 pins at once
     * @param pins Output: 16-bit pin state (bit 0 = P0, bit 15 = P15)
     * @return ESP_OK on success
     */
    esp_err_t readAllPins(uint16_t& pins);

    /**
     * @brief Register callback for pin changes (interrupt-driven)
     * @param callback Callback function
     */
    void setChangeCallback(PinChangeCallback callback);

    /**
     * @brief Get last read pin states
     * @return 16-bit pin state
     */
    uint16_t getLastPinStates() const { return m_lastPinState; }

    /**
     * @brief Set I2C multiplexer channel for this device
     * @param muxHandler Pointer to TCA9548 handler
     * @param channel Channel number (0-7)
     */
    void setMuxChannel(TCA9548Handler* muxHandler, uint8_t channel);

private:
    static constexpr uint16_t MAX_PINS = 16;
    // CRITICAL: Minimal debounce for quadrature encoders (4 transitions per detent at ~1-2ms each)
    // Encoder state machine in MultiEncoderHandler handles proper debouncing via accumulator
    static constexpr uint32_t DEBOUNCE_TIME_MS = 0; // No debounce - rely on hardware glitch filter and state machine

    uint8_t m_i2cAddr;
    gpio_num_t m_interruptPin;
    i2c_master_bus_handle_t m_i2cBusHandle;
    i2c_master_dev_handle_t m_i2cDevHandle;

    bool m_initialized;
    bool m_useInterrupt;

    // Pin state tracking
    uint16_t m_outputMask;      // 1 = output (drive), 0 = input (high-Z)
    uint16_t m_outputState;     // Output pin states
    uint16_t m_lastPinState;    // Last read input state

    // Interrupt handling
    TaskHandle_t m_taskHandle;
    SemaphoreHandle_t m_interruptSemaphore;
    PinChangeCallback m_changeCallback;

    // Debouncing
    int64_t m_lastReadTime;

    // I2C multiplexer support
    TCA9548Handler* m_muxHandler;
    uint8_t m_muxChannel;

    esp_err_t writePort(uint16_t value);
    esp_err_t readPort(uint16_t& value);
    void processChanges(uint16_t newState, uint16_t oldState);
    esp_err_t selectMuxChannel();

    static void IRAM_ATTR interruptHandler(void* arg);
    static void interruptTask(void* param);
};