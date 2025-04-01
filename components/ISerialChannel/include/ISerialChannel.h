#pragma once

#include "esp_err.h"
#include <functional>
#include <string>
#include <string_view>
#include <utility>

/**
 * @file ISerialChannel.h
 * @brief Abstract interface for serial communication channels
 *
 * This interface decouples transport implementations (UART, USB CDC, TCP, etc.)
 * from protocol logic. All serial channel types (SerialHandler, CdcSerialHandler,
 * MockSerialHandler) implement this interface to enable:
 * - Polymorphic usage in RadioManager and command handlers
 * - Easy mocking in tests without UART dependencies
 * - Future transport types (Bluetooth, network, etc.) without coupling
 *
 * Design rationale:
 * - CdcSerialHandler previously inherited from SerialHandler (LSP violation)
 * - This interface pattern is architecturally correct for polymorphism
 * - Zero runtime overhead (pure virtual, no RTTI needed)
 * - ESP-IDF compatible (no exceptions, no std::optional in interface)
 *
 * Methods:
 * - Message querying: hasMessage(), getMessage(), getMessageView()
 * - Message sending: sendMessage() (single and dual variants)
 * - Diagnostics: getSendFailureCount()
 * - Events: setOnFrameCallback()
 *
 * Thread safety: Implementation-dependent. Check concrete class documentation.
 */
class ISerialChannel {
public:
    virtual ~ISerialChannel() = default;

    /**
     * @brief Check if a complete message (frame) is available
     * @return true if at least one ';'-terminated message is queued
     * @note Should be const and thread-safe for polling from multiple contexts
     */
    virtual bool hasMessage() const = 0;

    /**
     * @brief Retrieve and remove the oldest queued message
     * @return pair<error_code, message_string>
     *         - ESP_OK + message if available
     *         - ESP_ERR_NOT_FOUND + empty string if no message
     * @note Allocates a std::string. Use getMessageView() for zero-copy.
     */
    virtual std::pair<esp_err_t, std::string> getMessage() = 0;

    /**
     * @brief Retrieve and remove the oldest queued message (zero-copy)
     * @return pair<error_code, message_view>
     *         - ESP_OK + string_view if available
     *         - ESP_ERR_NOT_FOUND + empty view if no message
     * @note View is valid until next getMessage/getMessageView call
     * @note Preferred over getMessage() in hot paths (no allocation)
     */
    virtual std::pair<esp_err_t, std::string_view> getMessageView() = 0;

    /**
     * @brief Send a single message
     * @param message Message to send (typically ';'-terminated CAT command)
     * @return ESP_OK on success, error code on failure
     * @note Thread-safe in most implementations
     */
    virtual esp_err_t sendMessage(std::string_view message) = 0;

    /**
     * @brief Send two messages atomically (concatenated)
     * @param message1 First message
     * @param message2 Second message
     * @return ESP_OK if both sent successfully, error code otherwise
     * @note Useful for multi-command sequences that must not be interleaved
     * @note Implementations may concatenate or send sequentially
     */
    virtual esp_err_t sendMessage(std::string_view message1, std::string_view message2) = 0;

    /**
     * @brief Get count of send failures since initialization
     * @return Number of failed send attempts
     * @note Used for diagnostics and monitoring channel health
     */
    virtual uint32_t getSendFailureCount() const = 0;

    /**
     * @brief Register callback invoked when a frame is queued
     * @param cb Callback function (called from ISR or task context)
     * @note Used to wake up processing tasks when new data arrives
     * @note Callback should be fast (no blocking operations)
     */
    virtual void setOnFrameCallback(std::function<void()> cb) = 0;
};
