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
    std::pair<esp_err_t, std::string> getMessage() override;
    std::pair<esp_err_t, std::string_view> getMessageView() override;
    [[nodiscard]] bool hasMessage() const override;
    uint32_t getSendFailureCount() const override { return sendFailures_.load(); }
    // CDC RX wake is delivered via UsbCdc::setRxCallback, not this ISerialChannel hook,
    // so the frame callback is intentionally unused for CDC channels.
    void setOnFrameCallback(std::function<void()>) override {}

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

    // Extracts the first complete ';'-terminated frame from m_rxAccum into
    // m_frameBuffer and returns a sanitized view. Consumes the frame from the
    // accumulator. An oversized frame (length > FRAME_BUFFER_SIZE) is DISCARDED
    // (consumed, not emitted) so a truncated terminator-less frame is never
    // injected. Returns an empty view when no complete usable frame is present.
    std::string_view extractFrameFromAccum();
};
