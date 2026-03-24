#include "include/SimpleQuadratureEncoder.h"
#include "QuadratureDecoder.h"  // Shared quadrature decoding logic
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include <cstring>

static const char* TAG = "SimpleQuadratureEncoder";

SimpleQuadratureEncoder::SimpleQuadratureEncoder(gpio_num_t pinA, gpio_num_t pinB, gpio_num_t pinSW, uint32_t glitchFilterNs)
    : m_pinA(pinA)
    , m_pinB(pinB)
    , m_pinSW(pinSW)
    , m_glitchFilterNs(glitchFilterNs)
    , m_count(0)
    , m_lastState(0)
    , m_hasChange(false)
    , m_lastSwitchState(false)
    , m_rotationCallback(nullptr)
    , m_switchCallback(nullptr)
    , m_taskHandle(nullptr)
    , m_filterA(nullptr)
    , m_filterB(nullptr)
    , m_filterSW(nullptr)
    , m_lastSwitchTime(0)
{
}

SimpleQuadratureEncoder::~SimpleQuadratureEncoder()
{
    if (m_taskHandle != nullptr) {
        vTaskDelete(m_taskHandle);
        m_taskHandle = nullptr;
    }

    // Remove ISR handlers
    gpio_isr_handler_remove(m_pinA);
    gpio_isr_handler_remove(m_pinB);
    if (m_pinSW != GPIO_NUM_NC) {
        gpio_isr_handler_remove(m_pinSW);
    }

    // Delete glitch filters
    if (m_filterA != nullptr) {
        gpio_del_glitch_filter(m_filterA);
    }
    if (m_filterB != nullptr) {
        gpio_del_glitch_filter(m_filterB);
    }
    if (m_filterSW != nullptr) {
        gpio_del_glitch_filter(m_filterSW);
    }
}

esp_err_t SimpleQuadratureEncoder::initialize()
{
    // Configure encoder A/B pins as inputs (external + internal pull-ups for noise immunity)
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << m_pinA) | (1ULL << m_pinB);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;    // Enable internal pull-ups (KY-040 also has external 10k)
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure encoder pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create and enable hardware glitch filter to suppress EMI noise (especially during TX/RX transitions)
    // Note: Uses pin glitch filter (hardware-dependent window, typically 1-2 clock cycles)
    // The glitchFilterNs parameter is stored but not used with pin filter API (for future ESP-IDF versions)
    gpio_pin_glitch_filter_config_t filter_config = {};
    filter_config.clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT;
    filter_config.gpio_num = m_pinA;

    // Create filter for pin A
    ret = gpio_new_pin_glitch_filter(&filter_config, &m_filterA);
    if (ret == ESP_OK) {
        ret = gpio_glitch_filter_enable(m_filterA);
        if (ret != ESP_OK) {
            gpio_del_glitch_filter(m_filterA);
            m_filterA = nullptr;
        }
    } else {
        m_filterA = nullptr;
    }

    // Create filter for pin B
    filter_config.gpio_num = m_pinB;
    ret = gpio_new_pin_glitch_filter(&filter_config, &m_filterB);
    if (ret == ESP_OK) {
        ret = gpio_glitch_filter_enable(m_filterB);
        if (ret != ESP_OK) {
            gpio_del_glitch_filter(m_filterB);
            m_filterB = nullptr;
        }
    } else {
        m_filterB = nullptr;
    }

    // Read initial state
    bool initialA = gpio_get_level(m_pinA);
    bool initialB = gpio_get_level(m_pinB);
    m_lastState.store((initialA ? 0x02 : 0x00) | (initialB ? 0x01 : 0x00));

    // Configure switch pin if present
    if (m_pinSW != GPIO_NUM_NC) {
        gpio_config_t sw_conf = {};
        sw_conf.pin_bit_mask = (1ULL << m_pinSW);
        sw_conf.mode = GPIO_MODE_INPUT;
        sw_conf.pull_up_en = GPIO_PULLUP_ENABLE;    // Enable internal pull-up (KY-040 also has external 10k)
        sw_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        sw_conf.intr_type = GPIO_INTR_ANYEDGE;

        ret = gpio_config(&sw_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure switch pin: %s", esp_err_to_name(ret));
            return ret;
        }

        // Create and enable hardware glitch filter on switch pin
        filter_config.gpio_num = m_pinSW;
        ret = gpio_new_pin_glitch_filter(&filter_config, &m_filterSW);
        if (ret == ESP_OK) {
            ret = gpio_glitch_filter_enable(m_filterSW);
            if (ret != ESP_OK) {
                gpio_del_glitch_filter(m_filterSW);
                m_filterSW = nullptr;
            }
        } else {
            m_filterSW = nullptr;
        }

        // Read initial switch state (active low)
        int initialLevel = gpio_get_level(m_pinSW);
        m_lastSwitchState.store(initialLevel == 0);
        m_lastSwitchTime = esp_timer_get_time();

        // Install switch ISR handler
        ret = gpio_isr_handler_add(m_pinSW, switchISR, this);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add switch ISR handler: %s", esp_err_to_name(ret));
            return ret;
        }

        // Enable switch interrupt
        ret = gpio_intr_enable(m_pinSW);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable interrupt for GPIO%d: %s", m_pinSW, esp_err_to_name(ret));
            gpio_isr_handler_remove(m_pinSW);
            return ret;
        }
    }

    // Install encoder ISR handlers
    ret = gpio_isr_handler_add(m_pinA, encoderISR, this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add encoder A ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(m_pinB, encoderISR, this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add encoder B ISR handler: %s", esp_err_to_name(ret));
        gpio_isr_handler_remove(m_pinA);
        return ret;
    }

    // CRITICAL: Enable interrupts (they're disabled by default after gpio_config)
    ret = gpio_intr_enable(m_pinA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable interrupt for GPIO%d: %s", m_pinA, esp_err_to_name(ret));
        gpio_isr_handler_remove(m_pinA);
        gpio_isr_handler_remove(m_pinB);
        return ret;
    }

    ret = gpio_intr_enable(m_pinB);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable interrupt for GPIO%d: %s", m_pinB, esp_err_to_name(ret));
        gpio_intr_disable(m_pinA);
        gpio_isr_handler_remove(m_pinA);
        gpio_isr_handler_remove(m_pinB);
        return ret;
    }

    // Create processing task
    BaseType_t task_ret = xTaskCreate(
        encoderTask,
        "simple_enc_task",
        4096,  // Increased stack size for callback execution
        this,
        10,   // High priority for encoder responsiveness
        &m_taskHandle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create encoder task");
        gpio_isr_handler_remove(m_pinA);
        gpio_isr_handler_remove(m_pinB);
        if (m_pinSW != GPIO_NUM_NC) {
            gpio_isr_handler_remove(m_pinSW);
        }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "SimpleQuadratureEncoder initialized on GPIO%d/%d/%d", m_pinA, m_pinB, m_pinSW);
    return ESP_OK;
}

void SimpleQuadratureEncoder::setRotationCallback(RotationCallback callback)
{
    m_rotationCallback = callback;
}

void SimpleQuadratureEncoder::setSwitchCallback(SwitchCallback callback)
{
    m_switchCallback = callback;
}

int32_t SimpleQuadratureEncoder::getCount() const
{
    return m_count.load();
}

void SimpleQuadratureEncoder::resetCount()
{
    m_count.store(0);
}

void IRAM_ATTR SimpleQuadratureEncoder::encoderISR(void* arg)
{
    auto* encoder = static_cast<SimpleQuadratureEncoder*>(arg);

    // Track ISR fires for diagnostics
    encoder->m_isrFireCount.fetch_add(1, std::memory_order_relaxed);

    // Rotation lockout: ignore rotation for SWITCH_ROTATION_LOCKOUT_MS after switch press
    const uint64_t now = esp_timer_get_time();
    const uint64_t lastSwitchPress = encoder->m_lastSwitchPressTime.load(std::memory_order_relaxed);
    if (lastSwitchPress > 0 && (now - lastSwitchPress) < (SWITCH_ROTATION_LOCKOUT_MS * 1000ULL)) {
        // Too soon after switch press - ignore rotation to prevent accidental triggers
        encoder->m_rejectedTransitionCount.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Read current pin states
    bool pinA = gpio_get_level(encoder->m_pinA);
    bool pinB = gpio_get_level(encoder->m_pinB);

    // Build new 2-bit state
    uint8_t newState = (pinA ? 0x02 : 0x00) | (pinB ? 0x01 : 0x00);
    uint8_t oldState = encoder->m_lastState.load();

    // Decode quadrature transition using shared decoder
    const int8_t delta = quadrature::decode(oldState, newState);

    if (delta != 0) {
        // Rate limiting: reject transitions faster than physically possible
        const uint64_t lastTime = encoder->m_lastTransitionTime.load(std::memory_order_relaxed);

        if (lastTime > 0 && (now - lastTime) < MIN_TRANSITION_US) {
            // Transition too fast - likely EMI, reject it
            encoder->m_rejectedTransitionCount.fetch_add(1, std::memory_order_relaxed);
            encoder->m_lastState.store(newState);
            return;
        }

        // Burst suppression: detect alternating +1/-1 pattern (EMI noise)
        const int8_t lastDelta = encoder->m_lastDelta.load();

        if (lastDelta != 0 && delta == -lastDelta) {
            // Alternating direction detected - increment burst counter
            uint8_t burstCount = encoder->m_burstCount.fetch_add(1, std::memory_order_relaxed) + 1;

            if (burstCount >= BURST_THRESHOLD) {
                // EMI burst detected - reject this transition
                encoder->m_rejectedTransitionCount.fetch_add(1, std::memory_order_relaxed);
                encoder->m_lastState.store(newState);
                return;
            }
        } else {
            // Same direction or reset - clear burst counter
            encoder->m_burstCount.store(0, std::memory_order_relaxed);
        }

        encoder->m_lastDelta.store(delta, std::memory_order_relaxed);
        encoder->m_lastTransitionTime.store(now, std::memory_order_relaxed);

        // Detent accumulator: require DETENT_THRESHOLD transitions (1 detent = 4 transitions) before reporting
        // This filters out partial detents caused by mechanical bounce
        int8_t accumulator = encoder->m_detentAccumulator.fetch_add(delta, std::memory_order_relaxed) + delta;

        // Check if we've accumulated enough transitions for a complete detent
        if (accumulator >= DETENT_THRESHOLD) {
            // Positive detent complete - report CW rotation
            encoder->m_count.fetch_add(1);
            encoder->m_detentAccumulator.fetch_sub(DETENT_THRESHOLD, std::memory_order_relaxed);
            encoder->m_hasChange.store(true);
            encoder->m_validTransitionCount.fetch_add(1, std::memory_order_relaxed);
        } else if (accumulator <= -DETENT_THRESHOLD) {
            // Negative detent complete - report CCW rotation
            encoder->m_count.fetch_add(-1);
            encoder->m_detentAccumulator.fetch_add(DETENT_THRESHOLD, std::memory_order_relaxed);
            encoder->m_hasChange.store(true);
            encoder->m_validTransitionCount.fetch_add(1, std::memory_order_relaxed);
        }
        // Otherwise: accumulator is between -DETENT_THRESHOLD and +DETENT_THRESHOLD, keep accumulating
    }

    encoder->m_lastState.store(newState);
}

void IRAM_ATTR SimpleQuadratureEncoder::switchISR(void* arg)
{
    auto* encoder = static_cast<SimpleQuadratureEncoder*>(arg);

    // Read switch state (active low)
    const bool pressed = gpio_get_level(encoder->m_pinSW) == 0;

    // Only process if state actually changed
    if (const bool lastState = encoder->m_lastSwitchState.load(); pressed == lastState) {
        return; // No change, ignore
    }

    // Debounce check
    uint64_t now = esp_timer_get_time();
    uint64_t lastTime = encoder->m_lastSwitchTime;

    if ((now - lastTime) < (SWITCH_DEBOUNCE_MS * 1000ULL)) {
        return; // Too soon, ignore bounce
    }

    // Valid state change
    encoder->m_lastSwitchTime = now;
    encoder->m_lastSwitchState.store(pressed);
    encoder->m_hasChange.store(true);

    // Set rotation lockout timestamp on switch press to prevent accidental rotation triggers
    if (pressed) {
        encoder->m_lastSwitchPressTime.store(now, std::memory_order_relaxed);
    }
}

void SimpleQuadratureEncoder::encoderTask(void* param)
{
    auto* encoder = static_cast<SimpleQuadratureEncoder*>(param);

    int32_t lastCount = 0;
    bool lastSwitchState = encoder->m_lastSwitchState.load();
    uint32_t loopCount = 0;

    while (true) {
        // Check if there's been any change
        if (encoder->m_hasChange.load()) {
            encoder->m_hasChange.store(false);

            // Check rotation
            int32_t currentCount = encoder->m_count.load();
            if (currentCount != lastCount) {
                int32_t delta = currentCount - lastCount;
                lastCount = currentCount;

                ESP_LOGV(TAG, "🎛️ Rotation detected: delta=%ld, total=%ld", delta, currentCount);

                if (encoder->m_rotationCallback) {
                    encoder->m_rotationCallback(delta);
                } else {
                    ESP_LOGW(TAG, "Rotation callback is NULL!");
                }
            }

            // Check switch
            bool currentSwitchState = encoder->m_lastSwitchState.load();
            if (currentSwitchState != lastSwitchState) {
                lastSwitchState = currentSwitchState;

                ESP_LOGV(TAG, "🔘 Switch state changed: %s", currentSwitchState ? "PRESSED" : "RELEASED");

                if (encoder->m_switchCallback) {
                    encoder->m_switchCallback(currentSwitchState);
                } else {
                    ESP_LOGW(TAG, "Switch callback is NULL!");
                }
            }
        }

        // Periodic heartbeat log (every 10 seconds) with ISR diagnostics
        if (++loopCount % 1000 == 0) {
            const uint32_t isrFires = encoder->m_isrFireCount.load();
            const uint32_t validTrans = encoder->m_validTransitionCount.load();
            const uint32_t rejectedTrans = encoder->m_rejectedTransitionCount.load();

            // Manual GPIO reads to verify hardware (bypass ISR)
            const int levelA = gpio_get_level(encoder->m_pinA);
            const int levelB = gpio_get_level(encoder->m_pinB);
            const int levelSW = encoder->m_pinSW != GPIO_NUM_NC ? gpio_get_level(encoder->m_pinSW) : -1;

            ESP_LOGD(TAG, "Heartbeat: count=%ld, ISR=%lu, valid=%lu, reject=%lu | GPIO: A=%d B=%d SW=%d",
                     encoder->m_count.load(), isrFires, validTrans, rejectedTrans,
                     levelA, levelB, levelSW);

            // Reset counters for next interval
            encoder->m_isrFireCount.store(0);
            encoder->m_validTransitionCount.store(0);
            encoder->m_rejectedTransitionCount.store(0);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10ms
    }
}
