#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdint>

/**
 * @brief TCA9548 I2C Multiplexer Handler
 *
 * Manages channel selection for TCA9548A 8-channel I2C multiplexer.
 * Allows multiple I2C devices with same address on different channels.
 * Thread-safe channel switching with mutex protection.
 */
class TCA9548Handler {
public:
    /**
     * @brief Construct TCA9548 handler
     * @param i2cAddr I2C address of TCA9548 (default 0x70)
     */
    explicit TCA9548Handler(uint8_t i2cAddr = 0x70);
    ~TCA9548Handler();

    /**
     * @brief Initialize TCA9548 on given I2C bus
     * @param i2cBusHandle Handle to initialized I2C master bus
     * @return ESP_OK on success
     */
    esp_err_t initialize(i2c_master_bus_handle_t i2cBusHandle);

    /**
     * @brief Select a specific channel (0-7)
     * @param channel Channel number (0-7), or 0xFF to disable all
     * @return ESP_OK on success
     */
    esp_err_t selectChannel(uint8_t channel);

    /**
     * @brief Disable all channels
     * @return ESP_OK on success
     */
    esp_err_t disableAllChannels();

    /**
     * @brief Get currently selected channel
     * @return Current channel (0-7) or 0xFF if none selected
     */
    uint8_t getCurrentChannel() const { return m_currentChannel; }

    /**
     * @brief Check if TCA9548 is responding
     * @return true if device responds correctly
     */
    bool isDevicePresent();

private:
    static constexpr uint8_t MAX_CHANNELS = 8;
    static constexpr TickType_t MUTEX_TIMEOUT_MS = 100;  // 100ms timeout for mutex acquisition

    uint8_t m_i2cAddr;
    i2c_master_bus_handle_t m_i2cBusHandle;
    i2c_master_dev_handle_t m_i2cDevHandle;
    uint8_t m_currentChannel;
    bool m_initialized;
    SemaphoreHandle_t m_mutex;  // Mutex for thread-safe channel switching

    esp_err_t writeChannelRegister(uint8_t channelMask);
    esp_err_t readChannelRegister(uint8_t& channelMask);
};