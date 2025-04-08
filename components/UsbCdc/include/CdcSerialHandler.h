#pragma once

#include "ISerialChannel.h"
#include "esp_err.h"
#include <string>
#include <string_view>
#include <functional>
#include <atomic>

/**
 * @brief CDC-based serial handler implementing ISerialChannel
 *
 * This class provides serial communication over USB CDC instead of UART.
 * It implements the ISerialChannel interface for polymorphic usage with
 * RadioManager and command handlers.
 */
class CdcSerialHandler : public ISerialChannel {
public:
    explicit CdcSerialHandler(uint8_t instance = 0);
    ~CdcSerialHandler() override = default;

    // ISerialChannel interface implementation
    esp_err_t sendMessage(std::string_view message) override;
    esp_err_t sendMessage(std::string_view message1, std::string_view message2) override;
    std::pair<esp_err_t, std::string> getMessage() override;
    std::pair<esp_err_t, std::string_view> getMessageView() override;
    [[nodiscard]] bool hasMessage() const override;
    uint32_t getSendFailureCount() const override { return sendFailures_.load(); }
    void setOnFrameCallback(std::function<void()> cb) override { onFrameCallback_ = std::move(cb); }

protected:
    uint8_t getInstance() const { return m_instance; }

private:
    static constexpr size_t RX_ACCUM_SIZE = 512;
    static constexpr size_t FRAME_BUFFER_SIZE = 256;
    uint8_t m_rxAccum[RX_ACCUM_SIZE] = {};
    size_t m_rxLen = 0; // bytes currently in accumulator
    uint8_t m_instance;
    mutable uint8_t m_frameBuffer[FRAME_BUFFER_SIZE] = {};
    std::atomic<uint32_t> sendFailures_{0};
    std::function<void()> onFrameCallback_{};
};
