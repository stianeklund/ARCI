#pragma once

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <functional>
#include <atomic>

/**
 * @brief Lightweight quadrature encoder using GPIO ISR (no PCNT)
 *
 * This class provides simple quadrature decoding using GPIO interrupts.
 * It's designed for encoders that don't need the sophisticated acceleration
 * logic of EncoderHandler, freeing up PCNT units for critical encoders.
 *
 * Features:
 * - GPIO interrupt-based quadrature decoding
 * - State table for CW/CCW detection
 * - Optional switch support with debouncing
 * - Callbacks for rotation and switch events
 * - No PCNT unit usage
 *
 * Typical latency: ~1-2μs for direct GPIO
 */
class SimpleQuadratureEncoder {
public:
    using RotationCallback = std::function<void(int32_t delta)>;
    using SwitchCallback = std::function<void(bool pressed)>;

    /**
     * @brief Construct a new Simple Quadrature Encoder
     *
     * @param pinA Encoder A pin (quadrature input)
     * @param pinB Encoder B pin (quadrature input)
     * @param pinSW Optional switch pin (GPIO_NUM_NC if no switch)
     * @param glitchFilterNs Glitch filter window in nanoseconds (0 = default, recommended: 2000ns for KY-040)
     */
    SimpleQuadratureEncoder(gpio_num_t pinA, gpio_num_t pinB, gpio_num_t pinSW = GPIO_NUM_NC, uint32_t glitchFilterNs = 2000);

    ~SimpleQuadratureEncoder();

    /**
     * @brief Initialize the encoder (configure GPIOs, start task)
     *
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t initialize();

    /**
     * @brief Set callback for rotation events
     *
     * @param callback Function called with delta count (positive = CW, negative = CCW)
     */
    void setRotationCallback(RotationCallback callback);

    /**
     * @brief Set callback for switch events
     *
     * @param callback Function called with switch state (true = pressed)
     */
    void setSwitchCallback(SwitchCallback callback);

    /**
     * @brief Get current accumulated count
     *
     * @return int32_t Current count value
     */
    int32_t getCount() const;

    /**
     * @brief Reset accumulated count to zero
     */
    void resetCount();

private:
    // GPIO pins
    gpio_num_t m_pinA;
    gpio_num_t m_pinB;
    gpio_num_t m_pinSW;

    // Configuration
    uint32_t m_glitchFilterNs;  // Glitch filter window in nanoseconds

    // State tracking
    std::atomic<int32_t> m_count;
    std::atomic<uint8_t> m_lastState;
    std::atomic<bool> m_hasChange;
    std::atomic<bool> m_lastSwitchState;

    // Callbacks
    RotationCallback m_rotationCallback;
    SwitchCallback m_switchCallback;

    // FreeRTOS
    TaskHandle_t m_taskHandle;

    // Glitch filter handles
    gpio_glitch_filter_handle_t m_filterA;
    gpio_glitch_filter_handle_t m_filterB;
    gpio_glitch_filter_handle_t m_filterSW;

    // Burst suppression - reject alternating +1/-1 EMI noise
    std::atomic<int8_t> m_lastDelta{0};
    std::atomic<uint8_t> m_burstCount{0};

    // Rate limiting - reject impossibly fast transitions (EMI)
    std::atomic<uint64_t> m_lastTransitionTime{0};

    // Detent accumulator - require multiple transitions before reporting (filters mechanical bounce)
    std::atomic<int8_t> m_detentAccumulator{0};
    std::atomic<uint64_t> m_lastSwitchPressTime{0};  // For rotation lockout after switch press

    // Diagnostics - track ISR activity
    std::atomic<uint32_t> m_isrFireCount{0};
    std::atomic<uint32_t> m_validTransitionCount{0};
    std::atomic<uint32_t> m_rejectedTransitionCount{0};

    // ISR handlers (must be static)
    static void IRAM_ATTR encoderISR(void *arg);
    static void IRAM_ATTR switchISR(void *arg);
    static void encoderTask(void *param);

    // Debounce constants optimized for KY-040 mechanical encoder
    static constexpr uint32_t SWITCH_DEBOUNCE_MS = 20;  // Increased from 5ms for mechanical switch
    static constexpr uint8_t BURST_THRESHOLD = 3;  // Reject if alternating direction 3+ times
    static constexpr uint32_t MIN_TRANSITION_US = 500;  // Increased from 200μs for mechanical encoder
    static constexpr int8_t DETENT_THRESHOLD = 2;  // Require 2 transitions (1 detent) - encoder has 2 transitions per physical click
    static constexpr uint32_t SWITCH_ROTATION_LOCKOUT_MS = 50;  // Ignore rotation for 50ms after switch press
    uint64_t m_lastSwitchTime;
};
