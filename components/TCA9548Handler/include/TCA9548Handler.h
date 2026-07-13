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
     * @brief Select a specific channel (0-7) and release the mux immediately
     * @param channel Channel number (0-7), or 0xFF to disable all
     * @return ESP_OK on success
     *
     * @warning This selects the channel and drops the mutex before returning, so
     * it does NOT protect a subsequent device transaction against a concurrent
     * channel switch. It is only safe for single-threaded init/diagnostic use
     * (bus scans, boot-time setup). Any code that selects a channel and then
     * transacts with a device behind the mux MUST use lockChannel()/ChannelGuard
     * so the channel stays fixed for the whole transaction.
     */
    esp_err_t selectChannel(uint8_t channel);

    /**
     * @brief RAII guard that holds the mux on one channel across a transaction
     *
     * Acquiring the guard takes the mux mutex and selects the requested channel,
     * then keeps the mutex held until the guard is destroyed. This makes the
     * channel-select and the caller's device I2C transaction atomic: no other
     * task can switch the mux to a different channel (or a same-address sibling
     * device on another channel) in between.
     *
     * A default-constructed guard is a valid no-op (used when a consumer has no
     * mux configured). Construct a real guard via TCA9548Handler::lockChannel().
     * Check valid() before transacting; if false, the channel could not be
     * secured (mutex timeout or channel-select I2C failure) and the caller must
     * bail out instead of transacting on an unknown channel.
     */
    class ChannelGuard {
    public:
        // Default: no mux held, valid no-op. Safe to transact directly.
        ChannelGuard() noexcept : m_owner(nullptr), m_valid(true), m_locked(false) {}
        ~ChannelGuard();

        // Non-copyable
        ChannelGuard(const ChannelGuard&) = delete;
        ChannelGuard& operator=(const ChannelGuard&) = delete;

        // Movable: transfers mutex ownership; source becomes a no-op guard
        ChannelGuard(ChannelGuard&& other) noexcept
            : m_owner(other.m_owner), m_valid(other.m_valid), m_locked(other.m_locked) {
            other.m_owner = nullptr;
            other.m_valid = true;
            other.m_locked = false;
        }
        ChannelGuard& operator=(ChannelGuard&& other) noexcept {
            if (this != &other) {
                release();
                m_owner = other.m_owner;
                m_valid = other.m_valid;
                m_locked = other.m_locked;
                other.m_owner = nullptr;
                other.m_valid = true;
                other.m_locked = false;
            }
            return *this;
        }

        /// @return true if the channel is secured (or no mux was needed)
        bool valid() const { return m_valid; }
        explicit operator bool() const { return m_valid; }

    private:
        friend class TCA9548Handler;
        ChannelGuard(TCA9548Handler* owner, uint8_t channel);
        void release();

        TCA9548Handler* m_owner;
        bool m_valid;
        bool m_locked;  // true while this guard holds the mux mutex
    };

    /**
     * @brief Acquire the mux, select @p channel, and hold it until the guard dies
     * @param channel Channel number (0-7), or 0xFF to disable all
     * @return A ChannelGuard; call valid() before transacting
     */
    ChannelGuard lockChannel(uint8_t channel);

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