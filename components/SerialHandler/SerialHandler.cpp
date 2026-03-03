#include "SerialHandler.h"
#include <algorithm>
#include <cctype>
#include <cstring>
#include <string_view>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "../../include/pin_definitions.h"
#include "driver/gpio.h"

static const char *TAG = "SerialHandler";

SerialHandler::SerialHandler(const uart_port_t uart_num)
    : m_uart_num(uart_num), m_uartTaskHandle(nullptr), m_uartQueue(nullptr) {
    m_queueSpinlock = portMUX_INITIALIZER_UNLOCKED;
}

SerialHandler::~SerialHandler() {
    if (m_uartTaskHandle != nullptr) {
        vTaskDelete(m_uartTaskHandle);
    }
    if (m_uartQueue != nullptr) {
        vQueueDelete(m_uartQueue);
    }
    uart_driver_delete(m_uart_num);
}

esp_err_t SerialHandler::setupUart(const int baud_rate) {
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
        .flags = {0}
    };


    esp_err_t ret = uart_driver_install(m_uart_num, RX_BUF_SIZE, TX_BUF_SIZE, 20, &m_uartQueue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return ret;
    }

    ret = uart_param_config(m_uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed");
        return ret;
    }

    ret = uart_set_pin(m_uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return ret;
    }

    xTaskCreate(uartEventTask, "uart_event_task", 4096, this, 12, &m_uartTaskHandle);
    if (m_uartTaskHandle == nullptr) {
        ESP_LOGE(TAG, "Failed to create UART event task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t SerialHandler::setupUart(const int baud_rate, const int tx_pin, const int rx_pin) {
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
        .flags = {0}
    };


    esp_err_t ret = uart_driver_install(m_uart_num, RX_BUF_SIZE, TX_BUF_SIZE, 20, &m_uartQueue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return ret;
    }

    ret = uart_param_config(m_uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed");
        return ret;
    }

    ESP_LOGI(TAG, "Setting UART %d pins: TX=%d, RX=%d", m_uart_num, tx_pin, rx_pin);
    ret = uart_set_pin(m_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return ret;
    }

    xTaskCreate(uartEventTask, "uart_event_task", 4096, this, 12, &m_uartTaskHandle);
    if (m_uartTaskHandle == nullptr) {
        ESP_LOGE(TAG, "Failed to create UART event task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t SerialHandler::setupUart(const int baud_rate, const int tx_pin, const int rx_pin, int rts_pin, int cts_pin) {
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
        .flags = {0}
    };


    esp_err_t ret = uart_driver_install(m_uart_num, RX_BUF_SIZE, TX_BUF_SIZE, 20, &m_uartQueue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return ret;
    }

    ret = uart_param_config(m_uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed");
        return ret;
    }

    ESP_LOGI(TAG, "Setting UART %d pins: TX=%d, RX=%d (flow control disabled)", m_uart_num, tx_pin, rx_pin);
    ret = uart_set_pin(m_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return ret;
    }

    xTaskCreate(uartEventTask, "uart_event_task", 4096, this, 12, &m_uartTaskHandle);
    if (m_uartTaskHandle == nullptr) {
        ESP_LOGE(TAG, "Failed to create UART event task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

bool SerialHandler::hasMessage() const {
    portENTER_CRITICAL(&m_queueSpinlock);
    const bool hasMsg = m_count > 0;
    portEXIT_CRITICAL(&m_queueSpinlock);
    return hasMsg;
}

std::pair<esp_err_t, std::string> SerialHandler::getMessage() {
    clearExpiredMessages();  // Remove stale messages before processing

    // Take snapshot of queue state under lock
    portENTER_CRITICAL(&m_queueSpinlock);
    if (m_count == 0) {
        portEXIT_CRITICAL(&m_queueSpinlock);
        return {ESP_FAIL, ""};
    }

    const MsgSlot &slot = m_queue[m_head];
    std::string message(slot.data, slot.len);

    m_head = (m_head + 1) % QUEUE_CAPACITY;
    m_count--;
    const size_t remainingCount = m_count;
    portEXIT_CRITICAL(&m_queueSpinlock);

    // CAT protocol sanitization moved to CatParser layer (see ParserUtils::sanitizeFrame)
    // SerialHandler now remains protocol-agnostic - it only handles framing (';' delimiter)

    ESP_LOGV(TAG, "UART %d: Returning queued message (len=%zu, q=%zu)", m_uart_num, message.length(), remainingCount);
    return {ESP_OK, message};
}

std::pair<esp_err_t, std::string_view> SerialHandler::getMessageView() {
    clearExpiredMessages();  // Remove stale messages before processing

    // All queue access and buffer modification under lock
    portENTER_CRITICAL(&m_queueSpinlock);
    if (m_count == 0) {
        portEXIT_CRITICAL(&m_queueSpinlock);
        return {ESP_FAIL, std::string_view{}};
    }

    auto &[data, len, timestamp] = m_queue[m_head];
    (void)timestamp;  // Unused in this function
    std::string_view view{data, len};

    // CAT protocol sanitization moved to CatParser layer (see ParserUtils::sanitizeFrame)
    // SerialHandler now remains protocol-agnostic - it only handles framing (';' delimiter)

    m_head = (m_head + 1) % QUEUE_CAPACITY;
    m_count--;
    const size_t remainingCount = m_count;
    portEXIT_CRITICAL(&m_queueSpinlock);

    ESP_LOGV(TAG, "UART %d: Returning queued message view (len=%zu, q=%zu)", m_uart_num, view.length(), remainingCount);

    return {ESP_OK, view};
}

void SerialHandler::recordSendFailure(esp_err_t error,
                                      const char* reason,
                                      size_t attemptedLen,
                                      esp_log_level_t level) {
    const uint32_t count = sendFailures_.fetch_add(1) + 1;
    const char* errName = esp_err_to_name(error);

    if (level == ESP_LOG_WARN) {
        ESP_LOGW(TAG,
                 "UART%d TX drop #%u (%s, bytes=%zu): %s",
                 m_uart_num,
                 count,
                 reason,
                 attemptedLen,
                 errName);
    } else {
        ESP_LOGE(TAG,
                 "UART%d TX drop #%u (%s, bytes=%zu): %s",
                 m_uart_num,
                 count,
                 reason,
                 attemptedLen,
                 errName);
    }
}

esp_err_t SerialHandler::sendMessage(const std::string_view message) {
    const bool needs_terminator = message.empty() || message.back() != END_MARKER;
    const size_t total_len = message.length() + (needs_terminator ? 1 : 0);

    if (total_len > TX_BUF_SIZE) {
        recordSendFailure(ESP_ERR_INVALID_SIZE,
                          "payload too large",
                          total_len,
                          ESP_LOG_ERROR);
        return ESP_ERR_INVALID_SIZE;
    }

    if (!message.empty()) {
        // Log radio UART TX at INFO, skip periodic SM/PS polling noise
        if (m_uart_num == UART_NUM_1) {
            const bool periodic = (message.length() >= 2 &&
                ((message[0] == 'S' && message[1] == 'M') ||
                 (message[0] == 'P' && message[1] == 'S')));
            if (!periodic) {
                ESP_LOGI(TAG, "UART1 TX: '%.*s%s' (%zu bytes)",
                         static_cast<int>(message.length()), message.data(),
                         needs_terminator ? ";" : "",
                         total_len);
            }
        } else {
            ESP_LOGV(TAG, "UART%d TX: '%.*s%s' (%zu bytes)", m_uart_num,
                     static_cast<int>(message.length()), message.data(),
                     needs_terminator ? ";" : "",
                     total_len);
        }

        if (m_uart_num == UART_NUM_1) {
            const unsigned char first = static_cast<unsigned char>(message.front());
            if (!(first >= 'A' && first <= 'Z')) {
                ESP_LOGE(TAG, "UART%d TX anomaly: leading byte 0x%02X for frame '%.*s'",
                         m_uart_num, first, static_cast<int>(message.length()), message.data());
            }
        }

        // Check for control characters
        for (size_t i = 0; i < message.length(); i++) {
            if (message[i] < 32 && message[i] != ';') {
                ESP_LOGW(TAG, "UART%d TX WARNING: Control char at pos %zu: 0x%02X",
                         m_uart_num, i, static_cast<uint8_t>(message[i]));
            }
        }
    }

    // Check if TX buffer has space before writing to prevent blocking
    size_t tx_buffer_free = 0;
    esp_err_t space_check = uart_get_tx_buffer_free_size(m_uart_num, &tx_buffer_free);
    if (space_check != ESP_OK) {
        recordSendFailure(space_check,
                          "tx buffer query failed",
                          total_len,
                          ESP_LOG_ERROR);
        return space_check;
    }

    const size_t required_space = total_len;
    if (tx_buffer_free < required_space) {
        recordSendFailure(ESP_ERR_NO_MEM,
                          "tx buffer full",
                          required_space,
                          ESP_LOG_WARN);
        return ESP_ERR_NO_MEM;
    }

    int written = 0;
    if (!message.empty()) {
        const int bw = uart_write_bytes(m_uart_num, message.data(), message.length());
        if (bw != static_cast<int>(message.length())) {
            recordSendFailure(ESP_FAIL,
                              "uart write failed (body)",
                              message.length(),
                              ESP_LOG_ERROR);
            return ESP_FAIL;
        }
        written += bw;
    }

    if (needs_terminator) {
        constexpr char term = END_MARKER;
        const int bw = uart_write_bytes(m_uart_num, &term, 1);

        if (bw != 1) {
            recordSendFailure(ESP_FAIL,
                              "uart write failed (terminator)",
                              1,
                              ESP_LOG_ERROR);
            return ESP_FAIL;
        }
        written += bw;
    }

    ESP_LOGV(TAG, "uart_write_bytes successful. Wrote %d bytes.", written);
    if (m_uart_num == UART_NUM_1) {
        lastSentFrame_.assign(message.begin(), message.end());
        if (needs_terminator) {
            lastSentFrame_.push_back(END_MARKER);
        }
        lastSentTimestampUs_.store(esp_timer_get_time(), std::memory_order_relaxed);
    }
    return ESP_OK;
}

esp_err_t SerialHandler::sendMessage(const std::string_view message1, const std::string_view message2) {
    const bool ends_with_term = (
        (!message2.empty() && message2.back() == END_MARKER) ||
        (message2.empty() && !message1.empty() && message1.back() == END_MARKER)
    );

    const bool needs_terminator = !ends_with_term;

    const size_t total_len = message1.length() + message2.length() + (needs_terminator ? 1 : 0);

    if (total_len > TX_BUF_SIZE) {
        recordSendFailure(ESP_ERR_INVALID_SIZE,
                          "payload too large",
                          total_len,
                          ESP_LOG_ERROR);
        return ESP_ERR_INVALID_SIZE;
    }

    // Check if TX buffer has space before writing to prevent blocking
    size_t tx_buffer_free = 0;
    esp_err_t space_check = uart_get_tx_buffer_free_size(m_uart_num, &tx_buffer_free);
    if (space_check != ESP_OK) {
        recordSendFailure(space_check,
                          "tx buffer query failed",
                          total_len,
                          ESP_LOG_ERROR);
        return space_check;
    }

    const size_t required_space = total_len;
    if (tx_buffer_free < required_space) {
        recordSendFailure(ESP_ERR_NO_MEM,
                          "tx buffer full",
                          required_space,
                          ESP_LOG_WARN);
        return ESP_ERR_NO_MEM;
    }

    if (m_uart_num == UART_NUM_1) {
        unsigned char first = 0;
        if (!message1.empty()) {
            first = static_cast<unsigned char>(message1.front());
        } else if (!message2.empty()) {
            first = static_cast<unsigned char>(message2.front());
        }
        if (first != 0 && !(first >= 'A' && first <= 'Z')) {
            ESP_LOGE(TAG, "UART%d TX anomaly: leading byte 0x%02X for frame parts '%.*s' + '%.*s'",
                     m_uart_num,
                     first,
                     static_cast<int>(message1.length()), message1.data(),
                     static_cast<int>(message2.length()), message2.data());
        }
    }

    int written = 0;
    if (!message1.empty()) {
        const int bw = uart_write_bytes(m_uart_num, message1.data(), message1.length());
        if (bw != static_cast<int>(message1.length())) {
            recordSendFailure(ESP_FAIL,
                              "uart write failed (part1)",
                              message1.length(),
                              ESP_LOG_ERROR);
            return ESP_FAIL;
        }
        written += bw;
    }
    if (!message2.empty()) {
        const int bw = uart_write_bytes(m_uart_num, message2.data(), message2.length());
        if (bw != static_cast<int>(message2.length())) {
            recordSendFailure(ESP_FAIL,
                              "uart write failed (part2)",
                              message2.length(),
                              ESP_LOG_ERROR);
            return ESP_FAIL;
        }
        written += bw;
    }

    if (needs_terminator) {
        constexpr char term = END_MARKER;
        const int bw = uart_write_bytes(m_uart_num, &term, 1);
        if (bw != 1) {
            recordSendFailure(ESP_FAIL,
                              "uart write failed (terminator)",
                              1,
                              ESP_LOG_ERROR);
            return ESP_FAIL;
        }
        written += bw;
    }

    ESP_LOGV(TAG, "uart_write_bytes successful. Wrote %d bytes.", written);
    if (m_uart_num == UART_NUM_1) {
        lastSentFrame_.assign(message1.data(), message1.size());
        lastSentFrame_.append(message2.data(), message2.size());
        if (needs_terminator) {
            lastSentFrame_.push_back(END_MARKER);
        }
        lastSentTimestampUs_.store(esp_timer_get_time(), std::memory_order_relaxed);
    }
    return ESP_OK;
}

void SerialHandler::processReceivedData(const uint8_t *data, const size_t len) {
    // Enhanced RX logging for error tracking
    if (len > 0) {
        // First pass: scan for error patterns WITHOUT building string (cheap)
        bool hasEOError = false;       // 'E;' or 'O;' in this chunk
        bool hasQuestionError = false; // '?;' in this chunk

        for (size_t i = 1; i < len; i++) {
            if (data[i] == ';') {
                if (data[i-1] == 'E' || data[i-1] == 'O') hasEOError = true;
                if (data[i-1] == '?') hasQuestionError = true;
            }
        }

        // Only build string if we need to log (error detected or debug enabled)
        const bool needsLogging = hasEOError || hasQuestionError ||
                                  esp_log_level_get(TAG) >= ESP_LOG_DEBUG;

        std::string rxStr;
        if (needsLogging) {
            rxStr.assign(reinterpret_cast<const char*>(data), len);
        }

        if (hasEOError) {
            ESP_LOGI(TAG, "UART%d RX ERROR: '%s'", m_uart_num, rxStr.c_str());
            if (m_uart_num == UART_NUM_1) {
                const uint64_t lastTs = lastSentTimestampUs_.load(std::memory_order_relaxed);
                const double deltaMs = lastTs ? (esp_timer_get_time() - lastTs) / 1000.0 : -1.0;
                ESP_LOGI(TAG, "UART%d last TX before error: '%s' (%.1f ms ago)",
                         m_uart_num, lastSentFrame_.c_str(), deltaMs);
            }
            if (esp_log_level_get(TAG) >= ESP_LOG_VERBOSE) {
                printf("UART%d RX HEX:", m_uart_num);
                for (size_t i = 0; i < len; i++) {
                    printf(" %02X", data[i]);
                }
                printf("\n");
            }
        } else if (hasQuestionError) {
            // Find the command that preceded ?; to help identify which query failed
            // Response format: "CMD1...;CMD2...;?;CMD3...;" - the ?; follows the failed query
            std::string precedingCmd;
            std::string followingCmd;
            const size_t qPos = rxStr.find("?;");
            if (qPos != std::string::npos) {
                // Find preceding response (scan backward from ?; to find previous ;)
                if (qPos >= 2) {
                    size_t prevEnd = rxStr.rfind(';', qPos - 1);
                    if (prevEnd != std::string::npos && prevEnd > 0) {
                        size_t prevStart = rxStr.rfind(';', prevEnd - 1);
                        prevStart = (prevStart == std::string::npos) ? 0 : prevStart + 1;
                        precedingCmd = rxStr.substr(prevStart, prevEnd - prevStart);
                    } else if (prevEnd == std::string::npos && qPos > 0) {
                        // ?; is at start or no preceding ;
                        precedingCmd = rxStr.substr(0, qPos);
                    }
                }
                // Find following response (scan forward from ?; to find next command)
                const size_t afterQ = qPos + 2;
                if (afterQ < rxStr.size()) {
                    size_t nextEnd = rxStr.find(';', afterQ);
                    if (nextEnd != std::string::npos) {
                        followingCmd = rxStr.substr(afterQ, nextEnd - afterQ);
                    }
                }
            }

            // Build informative message
            if (!precedingCmd.empty() || !followingCmd.empty()) {
                ESP_LOGW(TAG, "UART%d '?;' error - preceding response: '%s', following: '%s'",
                         m_uart_num,
                         precedingCmd.empty() ? "(none)" : precedingCmd.c_str(),
                         followingCmd.empty() ? "(none)" : followingCmd.c_str());
            }
            ESP_LOGW(TAG, "UART%d RX: '%s'", m_uart_num, rxStr.c_str());
            if (m_uart_num == UART_NUM_1 && !lastSentFrame_.empty()) {
                const uint64_t lastTs = lastSentTimestampUs_.load(std::memory_order_relaxed);
                const double deltaMs = lastTs ? (esp_timer_get_time() - lastTs) / 1000.0 : -1.0;
                ESP_LOGI(TAG, "UART%d last TX: '%s' (%.1f ms ago)",
                         m_uart_num, lastSentFrame_.c_str(), deltaMs);
            }
        } else if (esp_log_level_get(TAG) >= ESP_LOG_DEBUG) {
            ESP_LOGD(TAG, "UART%d RX: '%s'", m_uart_num, rxStr.c_str());
        }
    }

    for (size_t i = 0; i < len; i++) {
        const char inChar = static_cast<char>(data[i]);

        // Skip non-printable control characters except ';' which is our frame marker
        if (inChar != ';' && (inChar < 32 || inChar > 126)) {
            if (inChar == '\r' || inChar == '\n') {
                // Carriage return / newline often appear on WAN/TCP bridges; ignore quietly
                ESP_LOGD(TAG, "UART %d: Ignoring CR/LF at pos %zu", m_uart_num, i);
            } else {
                const char* device_name = (m_uart_num == UART_NUM_1) ? "Radio" : "Display";
                ESP_LOGV(TAG, "%s UART %d: Skipping invalid char 0x%02X at pos %zu", device_name, m_uart_num,
                         (uint8_t)inChar, i);
            }
            continue;
        }

        // Append to accumulator if space remains
        if (m_accum_len < MAX_MESSAGE_LENGTH) {
            m_accum[m_accum_len++] = inChar;
        }

        // If end marker, attempt to process accumulated data
        if (inChar == END_MARKER) {
            if (m_accum_len <= 1) {
                // Only ';' in buffer - this is likely a wakeup command or framing artifact
                ESP_LOGD(TAG, "UART %d: Standalone semicolon (wakeup/framing)", m_uart_num);
                m_accum_len = 0; // Reset and continue
                continue;
            }

            // Check if we have a complete valid CAT command
            const std::string_view msg_view(m_accum, m_accum_len);

            // Try to find the start of a valid CAT command in the buffer
            // This handles cases where we have fragments like "006;SM00007;"
            size_t cmd_start = 0;
            bool found_valid_cmd = false;

            // Scan backwards from the semicolon to find a valid command start
            for (size_t search_pos = 0; search_pos < m_accum_len - 1; search_pos++) {
                // Check if this position starts a valid CAT command
                size_t remaining = m_accum_len - search_pos;
                std::string_view test_view(m_accum + search_pos, remaining);

                // Check for known short responses
                if (test_view == "?;" || test_view == "O;" || test_view == "E;") {
                    cmd_start = search_pos;
                    found_valid_cmd = true;
                    break;
                }

                // Check for standard CAT command (2+ letters followed by data and ';')
                if (remaining >= 3 &&
                    std::isalpha(static_cast<unsigned char>(m_accum[search_pos])) &&
                    std::isalpha(static_cast<unsigned char>(m_accum[search_pos + 1]))) {
                    cmd_start = search_pos;
                    found_valid_cmd = true;
                    break;
                }
            }

            if (!found_valid_cmd) {
                // No valid command found, this is likely a fragment
                // Don't clear accumulator - keep collecting data
                ESP_LOGD(TAG, "UART %d: Fragment detected, keeping in buffer: '%.*s' (len=%zu)",
                         m_uart_num, static_cast<int>(m_accum_len), m_accum, m_accum_len);

                // But if accumulator is getting too full with fragments, clear oldest data
                if (m_accum_len > MAX_MESSAGE_LENGTH / 2) {
                    // Shift buffer to keep newer data
                    size_t shift_amount = m_accum_len / 3;
                    memmove(m_accum, m_accum + shift_amount, m_accum_len - shift_amount);
                    m_accum_len -= shift_amount;
                    ESP_LOGD(TAG, "UART %d: Shifted buffer by %zu bytes to prevent overflow",
                            m_uart_num, shift_amount);
                }
                continue;
            }

            // Extract the valid command
            size_t cmd_len = m_accum_len - cmd_start;

            // '?;' frames are queued like any other valid response.
            // Routing/suppression is handled by the CommandDispatcher which
            // has query context to decide whether to forward or drop.

            // Capture data needed for queue operation
            const size_t copyLen = cmd_len > MAX_MESSAGE_LENGTH ? MAX_MESSAGE_LENGTH : cmd_len;
            const int64_t timestamp = esp_timer_get_time();
            bool droppedOldest = false;

            // Queue operation under spinlock
            portENTER_CRITICAL(&m_queueSpinlock);

            // If queue is full, drop oldest to make room
            if (m_count == QUEUE_CAPACITY) {
                m_head = (m_head + 1) % QUEUE_CAPACITY;
                m_count--;
                droppedOldest = true;
            }

            // Copy the valid command into tail slot
            auto &slot = m_queue[m_tail];
            memcpy(slot.data, m_accum + cmd_start, copyLen);
            slot.len = copyLen;
            slot.timestamp = timestamp;

            m_tail = (m_tail + 1) % QUEUE_CAPACITY;
            m_count++;
            const size_t queueCount = m_count;

            portEXIT_CRITICAL(&m_queueSpinlock);

            // Logging outside critical section
            if (droppedOldest) {
                ESP_LOGV(TAG, "UART %d: Message queue full, dropping oldest", m_uart_num);
            }
            // Log radio UART (UART1) non-periodic messages at INFO; skip SM/PS noise
            if (m_uart_num == UART_NUM_1) {
                const bool periodic = (copyLen >= 2 &&
                    ((slot.data[0] == 'S' && slot.data[1] == 'M') ||
                     (slot.data[0] == 'P' && slot.data[1] == 'S')));
                if (!periodic) {
                    ESP_LOGI(TAG, "UART1 RX queued: '%.*s' (len=%zu)",
                             static_cast<int>(copyLen), slot.data, copyLen);
                }
            } else {
                ESP_LOGD(TAG, "UART %d: Queued valid message: '%.*s' (len=%zu, q=%zu)",
                         m_uart_num, static_cast<int>(copyLen), slot.data, copyLen, queueCount);
            }

            // Remove processed command from accumulator
            // If there's data before the command (fragments), discard it
            if (cmd_start > 0) {
                ESP_LOGD(TAG, "UART %d: Discarding %zu bytes of fragments before valid command",
                         m_uart_num, cmd_start);
            }

            // Clear entire accumulator since we've processed the command ending with ';'
            m_accum_len = 0;

            // Notify higher level (if registered) that a complete frame is available
            if (onFrameCallback_) {
                onFrameCallback_();
            }
        }
    }

    if (m_accum_len >= MAX_MESSAGE_LENGTH) {
        // Show first 32 bytes of buffer content for debugging
        const size_t showLen = std::min(m_accum_len, size_t{32});
        ESP_LOGW(TAG, "UART %d: Message accumulator overflow (%zu bytes), clearing. Data (first %zu): '%.*s'",
                 m_uart_num, m_accum_len, showLen, static_cast<int>(showLen), m_accum);
        m_accum_len = 0;
    }
}

void SerialHandler::clearExpiredMessages() {
    portENTER_CRITICAL(&m_queueSpinlock);
    if (m_count == 0) {
        portEXIT_CRITICAL(&m_queueSpinlock);
        return;  // Nothing to clear
    }

    const int64_t currentTime = esp_timer_get_time();
    size_t cleared = 0;

    // Remove expired messages from head of queue
    while (m_count > 0) {
        const auto &headSlot = m_queue[m_head];
        const int64_t age = currentTime - headSlot.timestamp;

        if (age < QUEUE_TIMEOUT_US) {
            break;  // First non-expired message found, stop clearing
        }

        // Message is expired, remove it
        m_head = (m_head + 1) % QUEUE_CAPACITY;
        m_count--;
        cleared++;
    }

    const size_t remainingCount = m_count;
    portEXIT_CRITICAL(&m_queueSpinlock);

    if (cleared > 0) {
        ESP_LOGD(TAG, "UART %d: Cleared %zu expired messages (queue now: %zu)",
                 m_uart_num, cleared, remainingCount);
    }
}

void SerialHandler::uartEventTask(void *pvParameters) {
    const auto handler = static_cast<SerialHandler *>(pvParameters);
    uart_event_t event;
    auto dtmp = static_cast<uint8_t *>(heap_caps_malloc(RX_BUF_SIZE, MALLOC_CAP_8BIT));

    if (dtmp == nullptr) {
        ESP_LOGE(TAG, "UART %d: Failed to allocate RX buffer (%zu bytes)", handler->m_uart_num, RX_BUF_SIZE);
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGD(TAG, "UART event task started for UART %d", handler->m_uart_num);

    for (;;) {
        if (xQueueReceive(handler->m_uartQueue, &event, portMAX_DELAY)) {
            ESP_LOGV(TAG, "UART %d event received: type=%d, size=%d", handler->m_uart_num, event.type, event.size);
            switch (event.type) {
                case UART_DATA: {
                    const int bytesRead = uart_read_bytes(handler->m_uart_num, dtmp, event.size, portMAX_DELAY);
                    if (bytesRead > 0) {
#ifdef SERIAL_HANDLER_HEXDUMP_DEBUG
                        ESP_LOG_BUFFER_HEXDUMP(TAG, dtmp, bytesRead, ESP_LOG_VERBOSE);
#endif
                        handler->processReceivedData(dtmp, static_cast<size_t>(bytesRead));
                    } else if (bytesRead < 0) {
                        ESP_LOGW(TAG, "UART %d: uart_read_bytes failed", handler->m_uart_num);
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART %d hw fifo overflow", handler->m_uart_num);
                    uart_flush_input(handler->m_uart_num);
                    xQueueReset(handler->m_uartQueue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART %d ring buffer full", handler->m_uart_num);
                    uart_flush_input(handler->m_uart_num);
                    xQueueReset(handler->m_uartQueue);
                    break;
                case UART_BREAK: {
                    const char* device_name = (handler->m_uart_num == UART_NUM_1) ? "Radio" : "Display";
                    ESP_LOGW(TAG, "%s UART %d rx break", device_name, handler->m_uart_num);
                    break;
                }
                case UART_PARITY_ERR:
                    ESP_LOGW(TAG, "UART %d parity error", handler->m_uart_num);
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "UART %d frame error", handler->m_uart_num);
                    break;
                default:
                    ESP_LOGW(TAG, "UART %d unknown event type: %d", handler->m_uart_num, event.type);
                    break;
            }
        }
    }

    free(dtmp);
    dtmp = nullptr;
    vTaskDelete(nullptr);
}
