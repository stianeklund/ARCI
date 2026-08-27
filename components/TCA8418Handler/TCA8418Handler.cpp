#include "TCA8418Handler.h"
#include "../TCA9548Handler/include/TCA9548Handler.h"
#include "esp_log.h"
#include "../../include/pin_definitions.h"

#include "esp_check.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

static const char *TAG = "TCA8418";

TCA8418Handler::TCA8418Handler(gpio_num_t interruptPin, uint8_t numRows, uint8_t numCols)
    : m_i2cBusHandle(nullptr)
      , m_i2cDevHandle(nullptr)
      , m_interruptPin(interruptPin)
      , m_keySemaphore(nullptr)
      , m_keyTaskHandle(nullptr)
      , m_initialized(false)
      , m_usePolling(false)
      , m_muxHandler(nullptr)
      , m_muxChannel(0xFF)
      , m_lastHealthCheck(0)
      , m_consecutiveFailures(0)
      , m_numRows(numRows > 0 && numRows <= 8 ? numRows : 5)
      , m_numCols(numCols > 0 && numCols <= 10 ? numCols : 10)
      , m_lastKeyEventTime(0)
      , m_lastIntTime(0)
      , m_intPrevLow(false)
      , m_consecutiveTimeouts(0) {
    ESP_LOGD(TAG, "TCA8418Handler created with %dx%d matrix configuration", m_numRows, m_numCols);
}

TCA8418Handler::~TCA8418Handler() {
    if (m_keyTaskHandle != nullptr) {
        vTaskDelete(m_keyTaskHandle);
    }
    if (m_keySemaphore != nullptr) {
        vSemaphoreDelete(m_keySemaphore);
    }
    if (m_interruptPin != GPIO_NUM_NC) {
        gpio_isr_handler_remove(m_interruptPin);
    }
}

bool TCA8418Handler::initialize(i2c_master_bus_handle_t i2cBusHandle) {
    m_i2cBusHandle = i2cBusHandle;
    ESP_LOGD(TAG, "Starting TCA8418 initialization");

    // Select TCA9548 mux channel FIRST if configured
    if (m_muxHandler != nullptr && m_muxChannel != 0xFF) {
        ESP_LOGD(TAG, "Selecting TCA9548 channel %d for TCA8418", m_muxChannel);
        esp_err_t mux_ret = m_muxHandler->selectChannel(m_muxChannel);
        if (mux_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to select TCA9548 channel %d: %s", m_muxChannel, esp_err_to_name(mux_ret));
            return false;
        }
        // Add small delay to let mux channel stabilize
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Create device handle for TCA8418
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = TCA8418_I2C_ADDR;
    dev_cfg.scl_speed_hz = 100000;

    esp_err_t ret = i2c_master_bus_add_device(m_i2cBusHandle, &dev_cfg, &m_i2cDevHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add TCA8418 device to I2C bus: %s", esp_err_to_name(ret));
        return false;
    }

    // Test basic I2C communication first
    if (!testI2CCommunication()) {
        ESP_LOGE(TAG, "TCA8418 not responding on I2C - check wiring and address");
        scanI2CBus();
        return false;
    }
    ESP_LOGD(TAG, "TCA8418 I2C communication test passed");

    if (!configureKeypadEngine()) {
        ESP_LOGE(TAG, "Failed to configure keypad GPIOs");
        return false;
    }

    if (m_interruptPin != GPIO_NUM_NC) {
        // Reset GPIO to reclaim from any peripheral (flash, JTAG, etc.)
        if (gpio_reset_pin(m_interruptPin) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to reset GPIO%d (might be in use by system)", m_interruptPin);
        }

        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << m_interruptPin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE; // Disable HW interrupt before configuring
        if (gpio_config(&io_conf) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to configure interrupt pin GPIO%d, using polling mode", m_interruptPin);
            m_usePolling = true;
        } else {
            ESP_LOGD(TAG, "GPIO%d configured as INPUT with PULLUP", m_interruptPin);
        }
    } else {
        ESP_LOGW(TAG, "No interrupt pin specified - using polling mode");
        m_usePolling = true;
    }

    m_keySemaphore = xSemaphoreCreateBinary();
    if (m_keySemaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create key semaphore");
        return false;
    }

    if (xTaskCreate(keyTask, "tca8418_task", KEY_TASK_STACK_BYTES, this, 12, &m_keyTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create key task");
        vSemaphoreDelete(m_keySemaphore);
        m_keySemaphore = nullptr;
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Give the task a moment to start.

    if (!m_usePolling) {
        clearPendingInterruptsOnTCA();

        // Verify INT line went HIGH after clearing
        vTaskDelay(pdMS_TO_TICKS(10)); // Let INT line settle
        int int_level = gpio_get_level(m_interruptPin);

        // If still LOW, aggressively drain FIFO until INT releases
        int drain_attempts = 0;
        while (int_level == 0 && drain_attempts < 5) {
            ESP_LOGW(TAG, "INT still LOW after clear attempt %d, draining again...", drain_attempts + 1);
            clearPendingInterruptsOnTCA();
            vTaskDelay(pdMS_TO_TICKS(10));
            int_level = gpio_get_level(m_interruptPin);
            drain_attempts++;
        }

        if (int_level == 0) {
            ESP_LOGE(TAG, "INT line stuck LOW after %d drain attempts - possible hardware issue", drain_attempts);
            ESP_LOGE(TAG, "Check for: shorted buttons, floating matrix inputs, missing pull-up resistor");
        }
    }

    // The TCA8418 interrupt is not yet enabled, so the line should be high.
    if (!m_usePolling) {
        if (gpio_isr_handler_add(m_interruptPin, keyInterruptHandler, this) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to add ISR handler, falling back to polling mode");
            m_usePolling = true;
        } else {
            // Use ANYEDGE for reliable interrupt detection
            gpio_set_intr_type(m_interruptPin, GPIO_INTR_ANYEDGE);

            if (gpio_intr_enable(m_interruptPin) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to enable interrupt for GPIO%d", m_interruptPin);
                m_usePolling = true;
            }
        }
    }

    // The ESP32 is now listening, so any new event will be caught.
    if (!m_usePolling) {
        // Configure for active-low interrupts with key lock interrupt enabled
        uint8_t cfg_value_final = CFG_AI | CFG_KE_IEN | CFG_OVR_FLOW_IEN | CFG_K_LCK_IEN;
        if (!writeRegister(REG_CFG, cfg_value_final)) {
            ESP_LOGW(TAG, "Failed to enable TCA8418 interrupts in CFG - using polling mode");
            m_usePolling = true;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Allow CFG to take effect.

    // If INT is still LOW after everything, manually trigger event processing
    if (!m_usePolling) {
        int final_level = gpio_get_level(m_interruptPin);
        if (final_level == 0) {
            ESP_LOGD(TAG, "INT line LOW after ISR setup - triggering event drain");
            xSemaphoreGive(m_keySemaphore); // Wake up task to process events
        }
    }

    m_initialized = true;
    ESP_LOGI(TAG, "TCA8418 initialized (GPIO%d, %dx%d, %s mode)",
             m_interruptPin, m_numRows, m_numCols,
             m_usePolling ? "polling" : "interrupt");
    return true;
}

bool TCA8418Handler::testI2CCommunication() {
    // Try to read the CFG register as a basic connectivity test
    uint8_t testValue;
    if (!readRegister(REG_CFG, testValue)) {
        ESP_LOGE(TAG, "Failed to read CFG register - I2C communication failed");
        return false;
    }

    // Test write/read cycle to verify bidirectional communication
    uint8_t originalValue = testValue;
    uint8_t testWriteValue = 0x00; // Safe test value

    if (!writeRegister(REG_CFG, testWriteValue)) {
        ESP_LOGE(TAG, "Failed to write test value to CFG register");
        return false;
    }

    uint8_t readbackValue;
    if (!readRegister(REG_CFG, readbackValue)) {
        ESP_LOGE(TAG, "Failed to read back test value from CFG register");
        return false;
    }

    if (readbackValue != testWriteValue) {
        ESP_LOGW(TAG, "Write/read test mismatch: wrote 0x%02X, read 0x%02X", testWriteValue, readbackValue);
    }

    // Restore original value
    writeRegister(REG_CFG, originalValue);

    return true;
}

void TCA8418Handler::scanI2CBus() const {
    ESP_LOGD(TAG, "Scanning I2C bus for diagnostic...");

    bool found = false;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        esp_err_t ret = i2c_master_probe(m_i2cBusHandle, addr, 50);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02X%s", addr,
                     addr == TCA8418_I2C_ADDR ? " (TCA8418)" : "");
            found = true;
        }
    }

    if (!found) {
        ESP_LOGW(TAG, "No I2C devices found - check SDA=%d, SCL=%d", PIN_I2C_SDA, PIN_I2C_SCL);
    }
}

void TCA8418Handler::setMuxChannel(TCA9548Handler* muxHandler, uint8_t channel) {
    m_muxHandler = muxHandler;
    m_muxChannel = channel;
}

bool TCA8418Handler::configureKeypadEngine() {
    // Software reset sequence
    if (!writeRegister(REG_CFG, 0x00)) {
        ESP_LOGE(TAG, "Failed to reset CFG register");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Clear all GPIO configurations
    writeRegister(REG_KP_GPIO1, 0x00);
    writeRegister(REG_KP_GPIO2, 0x00);
    writeRegister(REG_KP_GPIO3, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure rows: K0-K7 based on instance configuration
    uint8_t row_mask = (1 << m_numRows) - 1;
    if (!writeRegister(REG_KP_GPIO1, row_mask)) {
        ESP_LOGE(TAG, "Failed to configure KP_GPIO1 for %d rows", m_numRows);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Configure columns: K8-K17 (REG_KP_GPIO2 = K8-K15, REG_KP_GPIO3 = K16-K17)
    uint8_t col_mask_1 = 0;
    uint8_t col_mask_2 = 0;

    if (m_numCols <= 8) {
        col_mask_1 = (1 << m_numCols) - 1;
    } else {
        col_mask_1 = 0xFF;
        col_mask_2 = (1 << (m_numCols - 8)) - 1;
    }

    if (!writeRegister(REG_KP_GPIO2, col_mask_1)) {
        ESP_LOGE(TAG, "Failed to configure KP_GPIO2 for columns");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    if (!writeRegister(REG_KP_GPIO3, col_mask_2)) {
        ESP_LOGE(TAG, "Failed to configure KP_GPIO3 for columns");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Verify configuration by reading back
    uint8_t kp_gpio1, kp_gpio2, kp_gpio3;
    if (readRegister(REG_KP_GPIO1, kp_gpio1) &&
        readRegister(REG_KP_GPIO2, kp_gpio2) &&
        readRegister(REG_KP_GPIO3, kp_gpio3)) {
        ESP_LOGD(TAG, "GPIO config: KP_GPIO1=0x%02X, KP_GPIO2=0x%02X, KP_GPIO3=0x%02X",
                 kp_gpio1, kp_gpio2, kp_gpio3);
    }

    return true;
}

void TCA8418Handler::clearPendingInterruptsOnTCA() {
    // First, drain any pending events from the FIFO
    uint8_t keyLockEventCounter;
    if (readRegister(REG_KEY_LCK_EC, keyLockEventCounter)) {
        uint8_t eventsInFifo = keyLockEventCounter & 0x0F;
        if (eventsInFifo > 0) {
            ESP_LOGV(TAG, "Draining %d events from FIFO", eventsInFifo);
        }

        // Read all events to clear the FIFO - INCREASED to 16 to handle full FIFO
        for (uint8_t i = 0; i < eventsInFifo && i < 16; i++) {
            uint8_t eventByte;
            if (!readRegister(REG_KEY_EVENT_A, eventByte)) {
                ESP_LOGW(TAG, "Failed to read event %d/%d", i + 1, eventsInFifo);
                break;
            }
            // Decode the event to help diagnose hardware issues
            bool isPress = (eventByte & 0x80) == 0;
            uint8_t keyCode = eventByte & 0x7F;
            uint8_t row = 0, col = 0;
            if (keyCode > 0) {
                row = ((keyCode - 1) % m_numRows) + 1;
                col = ((keyCode - 1) / m_numRows) + 1;
            }
            ESP_LOGV(TAG, "Event %d: 0x%02X = Key 0x%02X (R%d C%d) %s",
                     i + 1, eventByte, keyCode, row, col, isPress ? "PRESS" : "RELEASE");
        }
    }

    // Read INT_STAT to see what interrupts are pending
    uint8_t intStat;
    if (readRegister(REG_INT_STAT, intStat)) {
        if (intStat != 0) {
            ESP_LOGV(TAG, "INT_STAT before clear: 0x%02X", intStat);
        }
    }

    // Then clear the interrupt status register to release the INT pin
    writeRegister(REG_INT_STAT, 0xFF);
}

bool TCA8418Handler::writeRegister(uint8_t reg, uint8_t value) {
    // CRITICAL: Hold the mux on this device's channel across the whole transaction.
    // Multiple TCA8418 instances share the same address (0x34) on different channels;
    // a preempting task (or another keypad's interrupt handler) must not be able to
    // switch the mux between the channel-select and this write, or the write would
    // land on the WRONG keypad. The guard keeps the channel fixed until it destructs.
    TCA9548Handler::ChannelGuard guard;
    if (m_muxHandler != nullptr && m_muxChannel != 0xFF) {
        guard = m_muxHandler->lockChannel(m_muxChannel);
        if (!guard.valid()) {
            ESP_LOGE(TAG, "Failed to lock mux channel %d before write", m_muxChannel);
            return false;
        }
    }

    uint8_t write_buf[2] = {reg, value};

    esp_err_t ret = i2c_master_transmit(m_i2cDevHandle, write_buf, sizeof(write_buf), 1000);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write to reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
        return false;
    }
    ESP_LOGV(TAG, "I2C write successful: reg=0x%02X, value=0x%02X", reg, value);
    return true;
}

bool TCA8418Handler::readRegister(uint8_t reg, uint8_t &value) {
    // CRITICAL: Hold the mux on this device's channel across the whole transaction.
    // Multiple TCA8418 instances share the same address (0x34) on different channels;
    // a preempting task must not be able to switch the mux between the channel-select
    // and this read, or the read would return data from the WRONG keypad. The guard
    // keeps the channel fixed until it destructs.
    TCA9548Handler::ChannelGuard guard;
    if (m_muxHandler != nullptr && m_muxChannel != 0xFF) {
        guard = m_muxHandler->lockChannel(m_muxChannel);
        if (!guard.valid()) {
            ESP_LOGE(TAG, "Failed to lock mux channel %d before read", m_muxChannel);
            return false;
        }
    }

    // Write register address then read data
    esp_err_t ret = i2c_master_transmit_receive(m_i2cDevHandle, &reg, 1, &value, 1, 1000);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read from reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
        return false;
    }
    ESP_LOGV(TAG, "I2C read successful: reg=0x%02X, value=0x%02X", reg, value);
    return true;
}

void IRAM_ATTR TCA8418Handler::keyInterruptHandler(void *arg) {
    auto *handler = static_cast<TCA8418Handler *>(arg);

    // For ANYEDGE interrupts, no need to disable - edge-triggered won't storm
    BaseType_t hpw = pdFALSE;
    if (handler->m_keySemaphore) {
        xSemaphoreGiveFromISR(handler->m_keySemaphore, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

void TCA8418Handler::keyTask(void *param) {
    TCA8418Handler *handler = static_cast<TCA8418Handler *>(param);

    uint32_t healthCheckCounter = 0;
    const uint32_t HEALTH_CHECK_INTERVAL = handler->m_usePolling ? 200 : 600; // Every 10s in polling, 30s in interrupt mode
    uint32_t stackLogCounter = 0;

    while (true) {
        if (++stackLogCounter >= 200) { // roughly every 10 seconds in polling mode
            stackLogCounter = 0;
            UBaseType_t watermark = uxTaskGetStackHighWaterMark(nullptr);
            ESP_LOGD(TAG, "tca8418_task stack watermark: %lu words", static_cast<unsigned long>(watermark));
        }

        if (handler->m_usePolling) {
            // Polling mode: check for events every 50ms
            handler->handleKeyEvents();

            // Periodic health check
            healthCheckCounter++;
            if (healthCheckCounter >= HEALTH_CHECK_INTERVAL) {
                healthCheckCounter = 0;
                if (!handler->checkDeviceHealth()) {
                    ESP_LOGW(TAG, "Device health check failed, attempting recovery");
                    // CRITICAL FIX: Try recovery before full reset
                    handler->recoverFromStuckState();
                    if (handler->m_consecutiveFailures >= 3) {
                        ESP_LOGE(TAG, "Multiple consecutive failures, resetting device");
                        handler->resetDevice();
                    }
                }
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            // MACRO DIAGNOSTIC: Track consecutive timeouts to detect task starvation.
            // Per-instance member (was a function-local static shared across both
            // keypad instances, which corrupted the diagnostic when two exist).

            // Interrupt mode: wait for semaphore from ISR with timeout for health checks
            if (xSemaphoreTake(handler->m_keySemaphore, pdMS_TO_TICKS(500)) == pdTRUE) {
                // Reset timeout counter when we successfully process events
                handler->m_consecutiveTimeouts = 0;
                handler->handleKeyEvents();
            } else {
                // Timeout fallback: if INT is asserted but edge was missed, process events
                if (gpio_get_level(handler->m_interruptPin) == 0) {
                    handler->handleKeyEvents();
                    handler->m_consecutiveTimeouts = 0;
                    continue;
                }

                handler->m_consecutiveTimeouts++;
                if (handler->m_consecutiveTimeouts >= 3) {
                    ESP_LOGV(TAG, "TCA8418 task timeout %lu times - possible CPU starvation during macro execution", handler->m_consecutiveTimeouts);
                    handler->m_consecutiveTimeouts = 0;
                }

                // Timeout occurred - perform health check
                healthCheckCounter++;
                if (healthCheckCounter >= HEALTH_CHECK_INTERVAL) {
                    healthCheckCounter = 0;
                    if (!handler->checkDeviceHealth()) {
                        ESP_LOGW(TAG, "Device health check failed, attempting recovery");
                        // CRITICAL FIX: Try recovery before full reset
                        handler->recoverFromStuckState();
                        if (handler->m_consecutiveFailures >= 3) {
                            ESP_LOGE(TAG, "Multiple consecutive failures, resetting device");
                            handler->resetDevice();
                        }
                    }
                }
            }
        }
    }
}

void TCA8418Handler::handleKeyEvents() {
    // Check for a stuck INT line BEFORE processing. Deliberately do NOT prime the
    // watchdog timestamp here: the stuck detector needs the INT-low age to
    // accumulate across calls, otherwise it always measures ~0 and never fires.
    // m_lastIntTime is refreshed only when INT is observed HIGH (in
    // checkStuckInterruptPin) or after events are actually processed (below).
    if (checkStuckInterruptPin()) {
        ESP_LOGW(TAG, "Stuck interrupt pin detected, forcing recovery");
        return;
    }

    // Drain all events to empty in one pass - prevents interrupt storms
    EventBatch batch = drainEventsToEmpty();

    // Handle critical conditions first
    if (batch.hasOverflow) {
        handleOverflowRecovery();
    }

    if (batch.hasKeyLock) {
        handleKeyLockRecovery();
    }

    // Process all batched events
    if (batch.count > 0) {
        processBatchedEvents(batch);
        ESP_LOGD(TAG, "Processed batch of %d events (overflow: %s, keylock: %s)",
                 batch.count, batch.hasOverflow ? "YES" : "NO", batch.hasKeyLock ? "YES" : "NO");
        // Successful event processing means the INT line is being serviced: reset
        // the stuck-watchdog baseline so a busy key burst never looks "stuck".
        m_lastIntTime = esp_timer_get_time() / 1000;
    }

    // Single INT_STAT clear after all processing - critical for proper interrupt handling
    writeRegister(REG_INT_STAT, 0xFF);

    if (!m_usePolling) {
        if (gpio_get_level(m_interruptPin) == 0) {
            // Race condition guard: a new event arrived during processing (or the line
            // is stuck low). The edge-triggered ISR won't fire without a transition, so
            // re-give the semaphore to re-enter and drain / let the watchdog age it.
            xSemaphoreGive(m_keySemaphore);
        } else {
            // INT deasserted (HIGH) => line healthy. Refresh the watchdog baseline.
            m_lastIntTime = esp_timer_get_time() / 1000;
            m_intPrevLow = false;
        }
    }
}

void TCA8418Handler::processKeyEvent(uint8_t eventByte) {
    // Improved rate limiting - allow rapid events through but log them
    uint32_t currentTime = esp_timer_get_time() / 1000; // Convert to milliseconds
    
    // Reduce aggressive rate limiting from 10ms to 5ms and don't drop events
    static constexpr uint32_t IMPROVED_MIN_INTERVAL_MS = 5;
    if (currentTime - m_lastKeyEventTime < IMPROVED_MIN_INTERVAL_MS) {
        ESP_LOGV(TAG, "⏱️  Fast event: %lu ms since last (processing anyway)", 
                 currentTime - m_lastKeyEventTime);
        // Continue processing instead of dropping the event
    }
    m_lastKeyEventTime = currentTime;

    // FIXED: Use decode helper functions
    bool isPress = isKeyPress(eventByte);
    MatrixKey key = extractKey(eventByte);

    // Get row and column for debug display
    uint8_t row, column;
    getRowColumn(key, row, column);

    ESP_LOGV(TAG, "🔘 Key event: 0x%02X, Key: 0x%02X, Row: %d, Col: %d, Press: %s (CH%d)",
             eventByte, static_cast<uint8_t>(key), row, column, isPress ? "true" : "false", m_muxChannel);
    ESP_LOGV(TAG, "RAW event 0x%02X (CH%d)", eventByte & 0x7F, m_muxChannel);   // mask off the press/release bit

    // Filter out truly invalid keys (TCA8418 supports up to 8x10 = 80 keys max)
    uint8_t keyCode = static_cast<uint8_t>(key);
    if (keyCode == 0 || keyCode > 80) {
        ESP_LOGW(TAG, "Ignoring invalid key 0x%02X (outside TCA8418 range)", keyCode);
        return;
    }

    // Call registered callback if exists
    auto callback = m_keyCallbacks.find(key);
    if (callback != m_keyCallbacks.end()) {
        callback->second(key, isPress);
    } else {
        ESP_LOGW(TAG, "No callback registered for key 0x%02X (Row %d, Col %d)", keyCode, row, column);
    }
}

TCA8418Handler::MatrixKey TCA8418Handler::decodeKeyEvent(uint8_t eventByte) {
    // TCA8418 uses linear key codes: (column * num_rows) + row + 1
    // The eventByte directly corresponds to our MatrixKey enum values
    return static_cast<MatrixKey>(eventByte);
}

void TCA8418Handler::getRowColumn(MatrixKey key, uint8_t& row, uint8_t& column) {
    uint8_t keyCode = static_cast<uint8_t>(key);

    // Calculate row and column from linear key code using instance configuration
    // TCA8418 uses: key = (col * num_rows) + row + 1
    uint8_t maxKeys = m_numRows * m_numCols;

    if (keyCode > 0 && keyCode <= maxKeys) {
        row = ((keyCode - 1) % m_numRows) + 1;
        column = ((keyCode - 1) / m_numRows) + 1;

        ESP_LOGD(TAG, "Key 0x%02X decoded: R%d C%d (matrix %dx%d)",
                 keyCode, row, column, m_numRows, m_numCols);
    } else {
        row = 0xFF;
        column = 0xFF;
        ESP_LOGW(TAG, "Invalid key code: 0x%02X (max: 0x%02X for %dx%d matrix)",
                 keyCode, maxKeys, m_numRows, m_numCols);
    }
}

void TCA8418Handler::setKeyCallback(MatrixKey key, KeyCallback callback) {
    m_keyCallbacks[key] = callback;
}


bool TCA8418Handler::resetDevice() {
    ESP_LOGI(TAG, "Performing device reset");

    // Reset all configuration to defaults
    bool success = true;
    success &= writeRegister(REG_CFG, 0x00);
    success &= writeRegister(REG_KP_GPIO1, 0x00);
    success &= writeRegister(REG_KP_GPIO2, 0x00);
    success &= writeRegister(REG_KP_GPIO3, 0x00);

    if (!success) {
        ESP_LOGE(TAG, "Failed to reset device registers");
        return false;
    }

    // Allow time for reset to complete
    vTaskDelay(pdMS_TO_TICKS(20));

    // Reconfigure the device
    if (!configureKeypadEngine()) {
        ESP_LOGE(TAG, "Failed to reconfigure device after reset");
        return false;
    }

    // CRITICAL: Re-enable interrupts if not in polling mode
    // configureKeypadEngine() only sets up matrix GPIO, it doesn't restore REG_CFG
    if (!m_usePolling) {
        uint8_t cfg_value = CFG_AI | CFG_KE_IEN | CFG_OVR_FLOW_IEN | CFG_K_LCK_IEN;
        ESP_LOGI(TAG, "Re-enabling TCA8418 interrupts after reset (CFG = 0x%02X)", cfg_value);
        if (!writeRegister(REG_CFG, cfg_value)) {
            ESP_LOGW(TAG, "Failed to re-enable interrupts after reset");
            return false;
        }
    }

    m_consecutiveFailures = 0;
    ESP_LOGI(TAG, "Device reset completed successfully");
    return true;
}

bool TCA8418Handler::checkDeviceHealth() {
    // Simple health check - try to read CFG register
    uint8_t cfgValue;
    if (!readRegister(REG_CFG, cfgValue)) {
        m_consecutiveFailures++;
        ESP_LOGW(TAG, "Health check failed - consecutive failures: %lu", m_consecutiveFailures);
        return false;
    }

    // Check if CFG register has expected values (INT_CFG bit = 0 for active-low)
    // CRITICAL FIX: Include CFG_K_LCK_IEN in expected configuration
    uint8_t expectedCfg = CFG_AI | CFG_KE_IEN | CFG_OVR_FLOW_IEN | CFG_K_LCK_IEN;
    if (!m_usePolling && cfgValue != expectedCfg) {
        ESP_LOGW(TAG, "CFG register mismatch - expected: 0x%02X (active-low INT), actual: 0x%02X", expectedCfg, cfgValue);
        m_consecutiveFailures++;
        return false;
    }

    m_consecutiveFailures = 0;
    return true;
}

bool TCA8418Handler::recoverFromStuckState() {
    ESP_LOGW(TAG, "🔧 Attempting recovery from stuck device state");
    
    uint8_t intStat, keyLockEventCounter;
    bool recoverySuccessful = false;
    
    // Step 1: Read current interrupt status
    if (readRegister(REG_INT_STAT, intStat)) {
        ESP_LOGI(TAG, "INT_STAT during recovery: 0x%02X", intStat);
        
        // Check for specific stuck conditions
        if (intStat & INT_STAT_K_LCK_INT) {
            ESP_LOGW(TAG, "Key lock interrupt detected - clearing");
            recoverySuccessful |= handleKeyLockRecovery();
        }
        
        if (intStat & INT_STAT_OVR_FLOW_INT) {
            ESP_LOGW(TAG, "Overflow interrupt detected - clearing");
            recoverySuccessful |= handleOverflowRecovery();
        }
    }
    
    // Step 2: Check and clear event counter/FIFO
    if (readRegister(REG_KEY_LCK_EC, keyLockEventCounter)) {
        uint8_t eventsInFifo = keyLockEventCounter & 0x0F;
        if (eventsInFifo > 0) {
            ESP_LOGW(TAG, "Found %d events in FIFO during recovery - draining", eventsInFifo);
            EventBatch emergencyBatch = drainEventsToEmpty();
            processBatchedEvents(emergencyBatch);
            recoverySuccessful = true;
        }
    }
    
    // Step 3: Force clear all interrupt status bits
    writeRegister(REG_INT_STAT, 0xFF);
    
    // Step 4: Verify and restore CFG register if corrupted
    uint8_t cfgValue;
    uint8_t expectedCfg = CFG_AI | CFG_KE_IEN | CFG_OVR_FLOW_IEN | CFG_K_LCK_IEN;
    if (readRegister(REG_CFG, cfgValue) && cfgValue != expectedCfg) {
        ESP_LOGW(TAG, "CFG register corrupted (0x%02X), restoring to 0x%02X", cfgValue, expectedCfg);
        writeRegister(REG_CFG, expectedCfg);
        recoverySuccessful = true;
    }
    
    // Step 5: Reset consecutive failures if recovery actions were taken
    if (recoverySuccessful) {
        m_consecutiveFailures = 0;
        ESP_LOGI(TAG, "✅ Device recovery completed");
    } else {
        ESP_LOGW(TAG, "⚠️  No specific recovery actions were needed");
    }
    
    return recoverySuccessful;
}

TCA8418Handler::EventBatch TCA8418Handler::drainEventsToEmpty() {
    EventBatch batch = {};
    uint8_t intStat;
    
    // Read interrupt status first
    if (!readRegister(REG_INT_STAT, intStat)) {
        ESP_LOGE(TAG, "Failed to read interrupt status in drain");
        return batch;
    }
    
    // Check for critical conditions
    batch.hasOverflow = (intStat & INT_STAT_OVR_FLOW_INT) != 0;
    batch.hasKeyLock = (intStat & INT_STAT_K_LCK_INT) != 0;
    
    if (batch.hasOverflow) {
        ESP_LOGW(TAG, "⚠️  FIFO overflow detected (INT_STAT: 0x%02X) - draining to recover", intStat);
    }
    
    if (batch.hasKeyLock) {
        ESP_LOGW(TAG, "⚠️  Key lock detected (INT_STAT: 0x%02X) - clearing lock state", intStat);
    }
    
    // Drain FIFO completely in one pass
    bool fifoEmpty = false;
    uint32_t loopCount = 0;
    const uint32_t MAX_DRAIN_LOOPS = 20; // Safety net to prevent infinite loop
    
    while (!fifoEmpty && loopCount < MAX_DRAIN_LOOPS) {
        uint8_t keyLockEventCounter;
        if (!readRegister(REG_KEY_LCK_EC, keyLockEventCounter)) {
            ESP_LOGE(TAG, "Failed to read event counter during drain");
            break;
        }
        
        uint8_t eventsInFifo = keyLockEventCounter & 0x0F;
        
        if (eventsInFifo == 0) {
            fifoEmpty = true;
            break;
        }
        
        // Read all available events in this iteration
        for (uint8_t i = 0; i < eventsInFifo && batch.count < MAX_EVENT_BATCH_SIZE; i++) {
            uint8_t eventByte;
            if (!readRegister(REG_KEY_EVENT_A, eventByte)) {
                ESP_LOGE(TAG, "Failed to read event %d during drain", i + 1);
                break;
            }
            
            batch.events[batch.count] = eventByte;
            batch.count++;
            
            ESP_LOGV(TAG, "Drained event %d: 0x%02X", batch.count, eventByte);
        }
        
        loopCount++;
    }
    
    if (loopCount >= MAX_DRAIN_LOOPS) {
        ESP_LOGW(TAG, "Drain loop safety limit reached - FIFO may not be empty");
    }
    
    // Log performance metrics for rapid button scenarios
    if (batch.count > 1) {
        ESP_LOGD(TAG, "Drained batch of %d events in %lu loops (rapid presses)",
                 batch.count, loopCount);
    }
    
    return batch;
}

void TCA8418Handler::processBatchedEvents(const EventBatch& batch) {
    for (uint8_t i = 0; i < batch.count; i++) {
        processKeyEvent(batch.events[i]);
    }
}

bool TCA8418Handler::handleOverflowRecovery() {
    ESP_LOGW(TAG, "🔧 Recovering from FIFO overflow");
    
    // Per datasheet: overflow condition is cleared by reading INT_STAT
    // FIFO should already be drained by drainEventsToEmpty()
    
    // Reset consecutive failures since we're recovering
    m_consecutiveFailures = 0;
    
    ESP_LOGI(TAG, "✅ Overflow recovery completed");
    return true;
}

bool TCA8418Handler::handleKeyLockRecovery() {
    ESP_LOGW(TAG, "🔧 Recovering from key lock condition");
    
    // Read current key lock event counter
    uint8_t keyLockEventCounter;
    if (!readRegister(REG_KEY_LCK_EC, keyLockEventCounter)) {
        ESP_LOGE(TAG, "Failed to read key lock counter during recovery");
        return false;
    }
    
    // Clear key lock by writing 0 to event counter (per TCA8418 datasheet)
    if (!writeRegister(REG_KEY_LCK_EC, 0x00)) {
        ESP_LOGE(TAG, "Failed to clear key lock counter");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ Key lock recovery completed (was: 0x%02X)", keyLockEventCounter);
    return true;
}

bool TCA8418Handler::checkStuckInterruptPin() {
    if (m_usePolling) {
        return false; // Not applicable in polling mode
    }

    const int pinLevel = gpio_get_level(m_interruptPin);
    const uint32_t currentTime = esp_timer_get_time() / 1000; // Convert to milliseconds

    if (pinLevel != 0) {
        // INT deasserted (HIGH) => line healthy. Reset the low-duration baseline so
        // the next assertion measures how long INT has been CONTINUOUSLY low.
        m_lastIntTime = currentTime;
        m_intPrevLow = false;
        return false;
    }

    // INT is asserted (LOW). Start the clock on the HIGH->LOW transition (or first
    // observation) so the watchdog measures continuous low time, not the idle gap
    // since the last event. Without this, a normal keypress after a long idle would
    // read a huge age and trip a false "stuck" recovery that discards the press.
    if (!m_intPrevLow) {
        m_intPrevLow = true;
        m_lastIntTime = currentTime;
        return false;
    }

    if (currentTime - m_lastIntTime > INT_STUCK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "⚠️  INT pin stuck low for %lu ms - forcing recovery",
                 currentTime - m_lastIntTime);

        // Force a complete drain and clear
        EventBatch emergencyBatch = drainEventsToEmpty();
        if (emergencyBatch.hasOverflow) handleOverflowRecovery();
        if (emergencyBatch.hasKeyLock) handleKeyLockRecovery();

        // Clear interrupt status aggressively
        writeRegister(REG_INT_STAT, 0xFF);

        // Restart the clock after recovery so we don't immediately re-trigger.
        m_lastIntTime = currentTime;

        ESP_LOGI(TAG, "🔧 Forced recovery completed, drained %d events", emergencyBatch.count);
        return true;
    }

    return false;
}
