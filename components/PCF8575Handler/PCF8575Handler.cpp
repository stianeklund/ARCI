#include "include/PCF8575Handler.h"
#include "TCA9548Handler.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>

static const char* TAG = "PCF8575Handler";

PCF8575Handler::PCF8575Handler(uint8_t i2cAddr, gpio_num_t interruptPin)
    : m_i2cAddr(i2cAddr)
    , m_interruptPin(interruptPin)
    , m_i2cBusHandle(nullptr)
    , m_i2cDevHandle(nullptr)
    , m_initialized(false)
    , m_useInterrupt(interruptPin != GPIO_NUM_NC)
    , m_outputMask(0x0000)    // All inputs initially
    , m_outputState(0xFFFF)   // Outputs high
    , m_lastPinState(0xFFFF)
    , m_taskHandle(nullptr)
    , m_interruptSemaphore(nullptr)
    , m_changeCallback(nullptr)
    , m_lastReadTime(0)
    , m_muxHandler(nullptr)
    , m_muxChannel(0)
{
}

PCF8575Handler::~PCF8575Handler()
{
    if (m_taskHandle != nullptr) {
        vTaskDelete(m_taskHandle);
        m_taskHandle = nullptr;
    }

    if (m_interruptSemaphore != nullptr) {
        vSemaphoreDelete(m_interruptSemaphore);
        m_interruptSemaphore = nullptr;
    }

    if (m_useInterrupt && m_interruptPin != GPIO_NUM_NC) {
        gpio_isr_handler_remove(m_interruptPin);
    }

    if (m_i2cDevHandle != nullptr) {
        i2c_master_bus_rm_device(m_i2cDevHandle);
        m_i2cDevHandle = nullptr;
    }
}

esp_err_t PCF8575Handler::initialize(i2c_master_bus_handle_t i2cBusHandle)
{
    if (i2cBusHandle == nullptr) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return ESP_ERR_INVALID_ARG;
    }

    m_i2cBusHandle = i2cBusHandle;

    // Configure PCF8575 device on I2C bus
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = m_i2cAddr;
    dev_cfg.scl_speed_hz = 400000; // 400 kHz

    esp_err_t ret = i2c_master_bus_add_device(m_i2cBusHandle, &dev_cfg, &m_i2cDevHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PCF8575 to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize all pins as inputs (write 0xFFFF)
    ret = writePort(0xFFFF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCF8575 pins");
        return ret;
    }

    // Read initial state
    uint16_t initialState = 0;
    ret = readPort(initialState);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read initial state");
        return ret;
    }
    m_lastPinState = initialState;

    // Setup interrupt if enabled
    if (m_useInterrupt) {
        // Create semaphore for interrupt signaling
        m_interruptSemaphore = xSemaphoreCreateBinary();
        if (m_interruptSemaphore == nullptr) {
            ESP_LOGE(TAG, "Failed to create interrupt semaphore");
            return ESP_ERR_NO_MEM;
        }

        // Configure interrupt pin (open-drain, pull-up, edge-triggered)
        // CRITICAL: Use ANYEDGE (both edges) for reliable change detection
        // PCF8575 INT is active-LOW: goes LOW on any pin change, HIGH after read
        // ANYEDGE catches both transitions, less sensitive than LOW_LEVEL (avoids noise)
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_ANYEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << m_interruptPin);
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure interrupt pin");
            return ret;
        }

        // Install ISR handler
        ret = gpio_isr_handler_add(m_interruptPin, interruptHandler, this);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add ISR handler");
            return ret;
        }

        // Enable the interrupt (required after gpio_config)
        ret = gpio_intr_enable(m_interruptPin);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable interrupt for GPIO%d", m_interruptPin);
            return ret;
        }

        // Create interrupt task
        BaseType_t taskRet = xTaskCreate(
            interruptTask,
            "pcf8575_isr",
            5120,  // Balanced stack size for I2C ops + logging + callbacks
            this,
            12, // High priority for encoder responsiveness
            &m_taskHandle
        );
        if (taskRet != pdPASS) {
            ESP_LOGE(TAG, "Failed to create interrupt task");
            return ESP_ERR_NO_MEM;
        }

    }

    m_initialized = true;
    ESP_LOGI(TAG, "PCF8575 initialized at 0x%02X (%s mode)",
             m_i2cAddr, (m_interruptPin != GPIO_NUM_NC) ? "interrupt" : "polling");
    return ESP_OK;
}

esp_err_t PCF8575Handler::setPinMode(uint8_t pinNumber, bool isInput)
{
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (pinNumber >= MAX_PINS) {
        return ESP_ERR_INVALID_ARG;
    }

    // Update output mask
    if (isInput) {
        m_outputMask &= ~(1 << pinNumber);  // Clear bit = input
    } else {
        m_outputMask |= (1 << pinNumber);   // Set bit = output
    }

    // Recalculate port value (outputs use m_outputState, inputs are high)
    uint16_t portValue = m_outputState | ~m_outputMask;
    return writePort(portValue);
}

esp_err_t PCF8575Handler::writePin(uint8_t pinNumber, bool state)
{
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (pinNumber >= MAX_PINS) {
        return ESP_ERR_INVALID_ARG;
    }

    // Update output state
    if (state) {
        m_outputState |= (1 << pinNumber);
    } else {
        m_outputState &= ~(1 << pinNumber);
    }

    // Write to port (only affects output pins)
    uint16_t portValue = m_outputState | ~m_outputMask;
    return writePort(portValue);
}

esp_err_t PCF8575Handler::readPin(uint8_t pinNumber, bool& state)
{
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (pinNumber >= MAX_PINS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t pins = 0;
    esp_err_t ret = readPort(pins);
    if (ret == ESP_OK) {
        state = (pins & (1 << pinNumber)) != 0;
    }
    return ret;
}

esp_err_t PCF8575Handler::readAllPins(uint16_t& pins)
{
    return readPort(pins);
}

void PCF8575Handler::setChangeCallback(PinChangeCallback callback)
{
    m_changeCallback = std::move(callback);
    // Publish AFTER the assignment so the interrupt task (already running) never
    // observes a half-assigned std::function. Pairs with the acquire load in
    // processChanges().
    m_callbackReady.store(true, std::memory_order_release);
}

void PCF8575Handler::setMuxChannel(TCA9548Handler* muxHandler, uint8_t channel)
{
    m_muxHandler = muxHandler;
    m_muxChannel = channel;
}

esp_err_t PCF8575Handler::writePort(uint16_t value)
{
    // PCF8575: Write 2 bytes (LSB first)
    uint8_t buf[2];
    buf[0] = value & 0xFF;         // P0-P7
    buf[1] = (value >> 8) & 0xFF;  // P10-P17

    // Hold the mux on this device's channel across the whole transaction so a
    // preempting task cannot switch the mux between channel-select and this write.
    TCA9548Handler::ChannelGuard guard;
    if (m_muxHandler != nullptr) {
        guard = m_muxHandler->lockChannel(m_muxChannel);
        if (!guard.valid()) {
            ESP_LOGE(TAG, "Failed to lock mux channel %d", m_muxChannel);
            return ESP_FAIL;
        }
    }

    esp_err_t ret = i2c_master_transmit(m_i2cDevHandle, buf, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t PCF8575Handler::readPort(uint16_t& value)
{
    // Debounce: Don't read too frequently (only if DEBOUNCE_TIME_MS > 0)
    int64_t now = esp_timer_get_time();
    if (DEBOUNCE_TIME_MS > 0 && (now - m_lastReadTime) < (DEBOUNCE_TIME_MS * 1000)) {
        value = m_lastPinState;
        return ESP_OK;
    }

    // PCF8575: Read 2 bytes (LSB first)
    // Hold the mux on this device's channel only across the channel-select + read
    // so the two are atomic against a concurrent channel switch. The guard is
    // released at the end of this block, BEFORE processChanges() runs its callback:
    // the callback must not execute while the (non-recursive) mux mutex is held, or
    // any I2C access it triggers would self-deadlock.
    uint8_t buf[2];
    {
        TCA9548Handler::ChannelGuard guard;
        if (m_muxHandler != nullptr) {
            guard = m_muxHandler->lockChannel(m_muxChannel);
            if (!guard.valid()) {
                ESP_LOGE(TAG, "Failed to lock mux channel %d", m_muxChannel);
                return ESP_FAIL;
            }
        }

        esp_err_t ret = i2c_master_receive(m_i2cDevHandle, buf, 2, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    value = buf[0] | (buf[1] << 8);

    // CRITICAL: Only update timestamp AFTER successful I2C read
    // This prevents blocking future reads if the I2C transaction failed
    m_lastReadTime = now;

    // Process changes if different from last state
    // CRITICAL: Update m_lastPinState BEFORE calling processChanges so that
    // callbacks see the CURRENT state, not the stale previous state.
    // This fixes encoder direction detection bugs where getLastPinStates()
    // was returning old data during the callback.
    if (value != m_lastPinState) {
        const uint16_t oldState = m_lastPinState;
        m_lastPinState = value;  // Update FIRST
        processChanges(value, oldState);  // Then notify with both states
    }

    return ESP_OK;
}

void PCF8575Handler::processChanges(uint16_t newState, uint16_t oldState)
{
    // Acquire pairs with the release in setChangeCallback(): if the callback has
    // not been published yet, the running interrupt task must not read the
    // std::function while it is being assigned on another task (data race / UB).
    if (!m_callbackReady.load(std::memory_order_acquire)) {
        return;
    }

    uint16_t changed = newState ^ oldState;

    // Call callback for each changed pin
    for (uint8_t pin = 0; pin < MAX_PINS; ++pin) {
        if (changed & (1 << pin)) {
            bool newPinState = (newState & (1 << pin)) != 0;
            m_changeCallback(pin, newPinState);
        }
    }
}

void IRAM_ATTR PCF8575Handler::interruptHandler(void* arg)
{
    PCF8575Handler* handler = static_cast<PCF8575Handler*>(arg);
    if (handler != nullptr && handler->m_interruptSemaphore != nullptr) {
        // For ANYEDGE interrupts, no need to disable - edge-triggered won't storm
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(handler->m_interruptSemaphore, &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

void PCF8575Handler::interruptTask(void* param)
{
    PCF8575Handler* handler = static_cast<PCF8575Handler*>(param);
    if (handler == nullptr) {
        vTaskDelete(nullptr);
        return;
    }

    uint32_t interruptCount = 0;
    uint32_t failedReads = 0;

    while (true) {
        // Bounded wait: a single missed edge (ISR lost under load, or INT already
        // asserted before the handler was installed) must not freeze the panel
        // encoders until reboot. On timeout, poll the INT line and force a recovery
        // read if it is still asserted (PCF8575 INT is active-LOW). Mirrors the
        // TCA8418 semaphore-timeout fallback.
        if (xSemaphoreTake(handler->m_interruptSemaphore, pdMS_TO_TICKS(250)) == pdTRUE) {
            interruptCount++;

            // Read all pins (clears interrupt on PCF8575 side)
            uint16_t pins = 0;
            esp_err_t ret = handler->readPort(pins);
            if (ret != ESP_OK) {
                failedReads++;
                ESP_LOGE(TAG, "Failed to read port on interrupt (count: %lu/%lu failed, err: %s)",
                         failedReads, interruptCount, esp_err_to_name(ret));

                // Log diagnostic info every 10 failures
                if (failedReads % 10 == 0) {
                    ESP_LOGE(TAG, "PCF8575 failure rate: %lu/%lu (%.1f%%)",
                             failedReads, interruptCount,
                             (failedReads * 100.0f) / interruptCount);
                }
            }
            // processChanges() called within readPort()
            // No need to re-enable interrupt with ANYEDGE (edge-triggered)
        } else if (handler->m_interruptPin != GPIO_NUM_NC &&
                   gpio_get_level(handler->m_interruptPin) == 0) {
            // Timed out with INT still asserted => an edge was missed. Read the port
            // to service and clear it (readPort() logs its own I2C errors and invokes
            // processChanges()); this releases the stuck INT line.
            uint16_t pins = 0;
            (void)handler->readPort(pins);
        }
    }
}