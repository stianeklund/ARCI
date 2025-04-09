#include "CdcSerialHandler.h"
#include "UsbCdc.h"
#include "esp_log.h"
#include "tusb_cdc_acm.h"
#include "tusb.h"

#include <algorithm>
#include <cstring>
#include <string_view>

namespace {

tinyusb_cdcacm_itf_t toTinyUsbInterface(uint8_t instance) {
    return instance == 0 ? TINYUSB_CDC_ACM_0 : TINYUSB_CDC_ACM_1;
}

const char* instanceTag(uint8_t instance) {
    return instance == 0 ? "CDC0" : "CDC1";
}

bool isTrimChar(unsigned char c) {
    return c <= 0x1F && c != ';';
}

std::string_view sanitizeFrameBuffer(uint8_t* buffer, size_t length) {
    if (buffer == nullptr || length == 0) {
        return {};
    }

    size_t start = 0;
    while (start < length) {
        const unsigned char c = buffer[start];
        if (!isTrimChar(c)) {
            break;
        }
        ++start;
    }

    size_t end = length;
    while (end > start && isTrimChar(buffer[end - 1])) {
        --end;
    }

    if (start >= end) {
        return {};
    }

    return std::string_view{reinterpret_cast<char*>(buffer + start), end - start};
}

} // namespace

static const char* TAG = "CdcSerialHandler";

CdcSerialHandler::CdcSerialHandler(const uint8_t instance)
    : m_instance(instance <= 1 ? instance : 0) {
    if (instance > 1) {
        ESP_LOGW(TAG, "Invalid CDC instance %u requested; defaulting to CDC0", instance);
    }
}

esp_err_t CdcSerialHandler::sendMessage(const std::string_view message) {
    if (message.empty()) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGV(TAG, "%s send: %.*s", instanceTag(m_instance), static_cast<int>(message.length()), message.data());

    const size_t required = message.length();
    if (UsbCdc::shouldThrottleWrite(m_instance)) {
        ESP_LOGV(TAG, "%s write throttled due to prior backpressure", instanceTag(m_instance));
        return ESP_ERR_NO_MEM;
    }
    size_t available = UsbCdc::availableForWrite(m_instance);
    if (available < required) {
        UsbCdc::flush(m_instance);
        available = UsbCdc::availableForWrite(m_instance);
    }
    if (available < required) {
        ESP_LOGW(TAG, "%s backpressure: dropping %zu-byte frame (avail=%zu)",
                 instanceTag(m_instance), required, available);
        UsbCdc::notifyWriteBackpressure(m_instance);
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t result = UsbCdc::writeData(reinterpret_cast<const uint8_t*>(message.data()), message.length(), m_instance);
    // Flush on CAT frame boundary or non-CAT payloads to minimize latency
    if (!message.empty() && (message.back() == ';' || message.back() == '\n' || message.back() == '\r')) {
        UsbCdc::flush(m_instance);
    }
    ESP_LOGV(TAG, "%s write result: %s", instanceTag(m_instance), esp_err_to_name(result));
    if (result == ESP_OK) {
        UsbCdc::notifyWriteSuccess(m_instance);
        if (!message.empty()) {
            UsbCdc::notifyTxMirrored(m_instance, reinterpret_cast<const uint8_t*>(message.data()), message.length());
        }
    } else {
        UsbCdc::notifyWriteBackpressure(m_instance);
    }
    return result;
}

esp_err_t CdcSerialHandler::sendMessage(const std::string_view message1, const std::string_view message2) {
    if (message1.empty() && message2.empty()) {
        return ESP_ERR_INVALID_ARG;
    }

    // Calculate total length
    const size_t totalLength = message1.length() + message2.length();

    // Create a temporary buffer to combine messages
    std::string combined;
    combined.reserve(totalLength);
    combined.append(message1);
    combined.append(message2);

    ESP_LOGV(TAG, "%s send: %.*s", instanceTag(m_instance), static_cast<int>(combined.length()), combined.data());

    const size_t required = combined.length();
    if (UsbCdc::shouldThrottleWrite(m_instance)) {
        ESP_LOGV(TAG, "%s write throttled due to prior backpressure", instanceTag(m_instance));
        return ESP_ERR_NO_MEM;
    }
    size_t available = UsbCdc::availableForWrite(m_instance);
    if (available < required) {
        UsbCdc::flush(m_instance);
        available = UsbCdc::availableForWrite(m_instance);
    }
    if (available < required) {
        ESP_LOGW(TAG, "%s backpressure: dropping %zu-byte frame (avail=%zu)",
                 instanceTag(m_instance), required, available);
        UsbCdc::notifyWriteBackpressure(m_instance);
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t result = UsbCdc::writeData(reinterpret_cast<const uint8_t*>(combined.data()), combined.length(), m_instance);
    if (!combined.empty() && (combined.back() == ';' || combined.back() == '\n' || combined.back() == '\r')) {
        UsbCdc::flush(m_instance);
    }
    if (result == ESP_OK) {
        UsbCdc::notifyWriteSuccess(m_instance);
        if (!combined.empty()) {
            UsbCdc::notifyTxMirrored(m_instance, reinterpret_cast<const uint8_t*>(combined.data()), combined.length());
        }
    } else {
        UsbCdc::notifyWriteBackpressure(m_instance);
    }
    return result;
}

std::pair<esp_err_t, std::string> CdcSerialHandler::getMessage() {
    uint8_t readBuffer[FRAME_BUFFER_SIZE];

    size_t rx_len = 0;
    const auto itf = toTinyUsbInterface(m_instance);
    const esp_err_t err = tinyusb_cdcacm_read(itf, readBuffer, sizeof(readBuffer), &rx_len);
    if (err != ESP_OK || rx_len == 0) {
        ESP_LOGD(TAG, "%s no data (err=%s, rx_len=%zu)", instanceTag(m_instance), esp_err_to_name(err), rx_len);
        return {ESP_FAIL, ""};
    }

    ESP_LOGD(TAG, "%s read %zu bytes", instanceTag(m_instance), rx_len);

    std::string message(reinterpret_cast<char*>(readBuffer), rx_len);
    ESP_LOGD(TAG, "%s RX (%zu bytes): %.*s", instanceTag(m_instance), rx_len,
             static_cast<int>(message.length()), message.data());

    return {ESP_OK, message};
}

std::pair<esp_err_t, std::string_view> CdcSerialHandler::getMessageView() {
    // 1) If accumulator already has a full frame, extract it
    for (size_t i = 0; i < m_rxLen; ++i) {
        if (m_rxAccum[i] == ';') {
            const size_t frameLen = i + 1;
            const size_t copyLen = std::min(frameLen, static_cast<size_t>(FRAME_BUFFER_SIZE));
            memcpy(m_frameBuffer, m_rxAccum, copyLen);
            const size_t remain = m_rxLen - frameLen;
            memmove(m_rxAccum, m_rxAccum + frameLen, remain);
            m_rxLen = remain;
            if (copyLen == 0) {
                i = static_cast<size_t>(-1);
                continue;
            }

            const std::string_view frameView = sanitizeFrameBuffer(m_frameBuffer, copyLen);
            if (frameView.empty()) {
                i = static_cast<size_t>(-1);
                continue;
            }
            return {ESP_OK, frameView};
        }
    }

    // 2) Read new bytes and append to accumulator; return first full frame found
    while (true) {
        uint8_t tmp[FRAME_BUFFER_SIZE / 2];
        size_t rx_len = 0;
        const auto itf = toTinyUsbInterface(m_instance);
        const esp_err_t err = tinyusb_cdcacm_read(itf, tmp, sizeof(tmp), &rx_len);
        if (err != ESP_OK || rx_len == 0) {
            break; // nothing more available now
        }
        const size_t space = (RX_ACCUM_SIZE > m_rxLen) ? (RX_ACCUM_SIZE - m_rxLen) : 0;
        const size_t to_copy = rx_len > space ? space : rx_len;
        if (to_copy > 0) {
            memcpy(m_rxAccum + m_rxLen, tmp, to_copy);
            m_rxLen += to_copy;
        }
        // If overflow, drop oldest data to keep most recent (unlikely with CAT)
        if (to_copy < rx_len) {
            const size_t overflow = rx_len - to_copy;
            if (overflow < RX_ACCUM_SIZE) {
                const size_t keep = RX_ACCUM_SIZE - overflow;
                memmove(m_rxAccum, m_rxAccum + overflow, keep);
                memcpy(m_rxAccum + keep, tmp + to_copy, overflow);
                m_rxLen = RX_ACCUM_SIZE;
            } else {
                // Overflow larger than buffer: keep last bytes only
                memcpy(m_rxAccum, tmp + (rx_len - RX_ACCUM_SIZE), RX_ACCUM_SIZE);
                m_rxLen = RX_ACCUM_SIZE;
            }
            ESP_LOGW(TAG, "%s RX accumulator overflow: dropped %zu bytes", instanceTag(m_instance), overflow);
        }

        // Scan for a frame terminator now
        for (size_t i = 0; i < m_rxLen; ++i) {
            if (m_rxAccum[i] == ';') {
                const size_t frameLen = i + 1;
                const size_t copyLen = std::min(frameLen, static_cast<size_t>(FRAME_BUFFER_SIZE));
                memcpy(m_frameBuffer, m_rxAccum, copyLen);
                const size_t remain = m_rxLen - frameLen;
                memmove(m_rxAccum, m_rxAccum + frameLen, remain);
                m_rxLen = remain;
                if (copyLen == 0) {
                    i = static_cast<size_t>(-1);
                    continue;
                }

                const std::string_view frameView = sanitizeFrameBuffer(m_frameBuffer, copyLen);
                if (frameView.empty()) {
                    i = static_cast<size_t>(-1);
                    continue;
                }
                return {ESP_OK, frameView};
            }
        }
        // If no terminator yet, continue reading until CDC ring drains
    }

    // No full frame yet
    return {ESP_FAIL, std::string_view{}};
}

bool CdcSerialHandler::hasMessage() const {
    // Event-driven design: hasMessage() should not be used for USB CDC
    // Callback triggers semaphore -> task reads directly
    ESP_LOGW(TAG, "%s hasMessage() called - use event-driven RX", instanceTag(m_instance));
    return false;
}
