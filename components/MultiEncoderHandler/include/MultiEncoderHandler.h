#pragma once

#include "../../SimpleQuadratureEncoder/include/SimpleQuadratureEncoder.h"
#include "../../PCF8575Handler/include/PCF8575Handler.h"
#include "../../RadioCore/include/RadioManager.h"
#include "esp_err.h"
#include <atomic>
#include <cstdlib>
#include <functional>
#include <memory>

/**
 * @brief Multi-Encoder Manager
 *
 * Coordinates multiple rotary encoders (direct GPIO + I2C-based).
 * Manages RF gain, AF gain, and multi-purpose encoders.
 */
class MultiEncoderHandler {
public:
    enum class EncoderId : uint8_t {
        RF_GAIN = 0,          // EC11E #1 Axis 1 (via PCF8575)
        AF_GAIN = 1,          // EC11E #1 Axis 2 (via PCF8575)
        MULTI_KNOB = 2,       // Multi-knob (direct GPIO)
        EC11E2_ENC1 = 3,      // EC11E #2 Axis 1 (via PCF8575)
        EC11E2_ENC2 = 4,      // EC11E #2 Axis 2 (via PCF8575)
        COUNT = 5
    };

    /**
     * @brief Encoder event callback
     * @param encoderId Which encoder generated the event
     * @param delta Rotation delta (+ or -)
     */
    using EncoderCallback = std::function<void(EncoderId encoderId, int32_t delta)>;

    /**
     * @brief Switch event callback
     * @param encoderId Which encoder's switch was pressed
     * @param isPressed true=pressed, false=released
     */
    using SwitchCallback = std::function<void(EncoderId encoderId, bool isPressed)>;

    explicit MultiEncoderHandler(radio::RadioManager* radioManager);
    ~MultiEncoderHandler();

    /**
     * @brief Initialize all direct GPIO encoders
     * @return ESP_OK on success
     */
    esp_err_t initializeDirectEncoders();

    /**
     * @brief Initialize I2C-based encoders via PCF8575
     * @param pcf8575Handler Initialized PCF8575 handler
     * @return ESP_OK on success
     */
    esp_err_t initializeI2CEncoders(PCF8575Handler* pcf8575Handler);

    /**
     * @brief Register encoder rotation callback
     * @param callback Callback function
     */
    void setEncoderCallback(EncoderCallback callback);

    /**
     * @brief Register switch press callback
     * @param callback Callback function
     */
    void setSwitchCallback(SwitchCallback callback);

    /**
     * @brief Configure default callbacks for RF/AF gain, multi-knob, and filter controls
     *
     * Sets up standard encoder behavior:
     * - RF_GAIN: Controls RF gain (0-255)
     * - AF_GAIN: Controls AF gain (0-255), switch toggles mute
     * - MULTI_KNOB: Sends CH0/CH1 commands
     * - EC11E2_ENC1: IF Shift (CW/CW-R) or High Cut (SSB/AM)
     * - EC11E2_ENC2: Low Cut (SSB/AM) or Filter Width (other modes)
     */
    void configureDefaultCallbacks();

    /**
     * @brief Start all encoder handlers
     */
    void startAll();

    /**
     * @brief Stop all encoder handlers
     */
    void stopAll();

private:
    radio::RadioManager* m_radioManager;

    // Direct GPIO encoders (multi-knob only)
    std::unique_ptr<SimpleQuadratureEncoder> m_multiKnobEncoder;

    // I2C encoder state (via PCF8575)
    PCF8575Handler* m_pcf8575;
    struct I2CEncoderState {
        uint8_t lastState;  // 2-bit quadrature state (A:B)
        int32_t position;
        bool switchPressed;
        int64_t lastSwitchPressTime;  // Timestamp of last switch press (microseconds)
        uint8_t invalidTransitionCount;  // Count consecutive invalid transitions for jitter filtering
        int8_t accumulator;  // Accumulate transitions before firing callback (for detent encoders)
        // Speed tracking for acceleration
        int64_t lastDetentTime;  // Timestamp of last detent completion (microseconds)
        uint8_t accelBand;  // Current acceleration band (0-4)
    };
    I2CEncoderState m_rfGainState{};     // EC11E #1 Axis 1 (outer shaft)
    I2CEncoderState m_afGainState{};     // EC11E #1 Axis 2 (inner shaft)
    I2CEncoderState m_ec11e2Enc1State{}; // EC11E #2 Axis 1
    I2CEncoderState m_ec11e2Enc2State{}; // EC11E #2 Axis 2
    I2CEncoderState m_multiKnobState{};  // MULTI_KNOB acceleration tracking

    // Debounce settings
    static constexpr int64_t SWITCH_ROTATION_DEBOUNCE_US = 50000;  // 50ms ignore rotation after switch press
    static constexpr uint8_t MAX_INVALID_TRANSITIONS = 2;  // Ignore rotation if too many invalid transitions
    static constexpr int8_t ACCUMULATOR_THRESHOLD = 2;  // Require 2 transitions (1 detent) for EC11E encoders

    // Acceleration settings - speed thresholds in detents per second
    // More aggressive thresholds than main encoder (gain controls benefit from faster acceleration)
    static constexpr float ACCEL_BAND1_DETENTS_PER_SEC = 2.0f;   // Slow → Medium (2 detents/sec)
    static constexpr float ACCEL_BAND2_DETENTS_PER_SEC = 5.0f;   // Medium → Fast (5 detents/sec)
    static constexpr float ACCEL_BAND3_DETENTS_PER_SEC = 10.0f;  // Fast → Very Fast (10 detents/sec)
    static constexpr float ACCEL_BAND4_DETENTS_PER_SEC = 20.0f;  // Very Fast → Max (20 detents/sec)
    static constexpr float ACCEL_HYSTERESIS = 0.20f;  // 20% hysteresis for downshift stability

    // Acceleration multipliers per band (for AF/RF gain encoders: 0-255 range)
    // EC11E #2 (filter/IF shift) uses lighter acceleration: 1x/1x/2x/3x/4x (see getAccelMultiplier)
    static constexpr int32_t ACCEL_MULT_BAND0 = 1;    // Slow: 1x (fine control)
    static constexpr int32_t ACCEL_MULT_BAND1 = 2;    // Medium: 2x (reduced for better resolution)
    static constexpr int32_t ACCEL_MULT_BAND2 = 4;    // Fast: 4x (reduced for better resolution)
    static constexpr int32_t ACCEL_MULT_BAND3 = 8;    // Very Fast: 8x (reduced for better resolution)
    static constexpr int32_t ACCEL_MULT_BAND4 = 12;   // Max: 12x (reduced for better resolution)

    // Callbacks
    EncoderCallback m_encoderCallback;
    SwitchCallback m_switchCallback;

    // AF mute state (for default callbacks)
    bool m_afMuted{false};
    int m_savedAfGainValue{128};
    std::atomic<uint64_t> m_afGainSwitchPressTime{0};

    // PCF8575 pin mapping for EC11E #1 (AF/RF Gain)
    // PCB layout: P0=A1, P1=A2, P2=B1, P3=B2, P4=D1 (interleaved A signals, then B signals)
    // Note: A1/B1 = outer shaft (AF Gain), A2/B2 = inner shaft (RF Gain)
    static constexpr uint8_t EC11E1_ENC1_A = 1;   // Inner shaft A (RF Gain) - P1 (A2)
    static constexpr uint8_t EC11E1_ENC1_B = 3;   // Inner shaft B (RF Gain) - P3 (B2)
    static constexpr uint8_t EC11E1_ENC2_A = 0;   // Outer shaft A (AF Gain) - P0 (A1)
    static constexpr uint8_t EC11E1_ENC2_B = 2;   // Outer shaft B (AF Gain) - P2 (B1)
    static constexpr uint8_t EC11E1_SW = 4;       // Outer shaft switch (AF mute, active low) - P4 (D1) ⚠️ Non-standard wiring!

    // PCF8575 pin mapping for EC11E #2 (High Cut/Low Cut control)
    // PCB layout: P10=A1, P11=A2, P12=B1, P13=B2, P14=D1 (interleaved A signals, then B signals)
    // Note: A1/B1 = outer shaft, A2/B2 = inner shaft
    // CRITICAL: PCF8575 uses P10-P17 naming but they map to bits 8-15, NOT bits 10-17!
    //           P10→bit8, P11→bit9, P12→bit10, P13→bit11, P14→bit12
    static constexpr uint8_t EC11E2_ENC1_A = 8;   // Outer shaft A (High Cut in SSB/AM, IF Shift in CW) - P10 (A1) → bit 8
    static constexpr uint8_t EC11E2_ENC1_B = 10;  // Outer shaft B (High Cut in SSB/AM, IF Shift in CW) - P12 (B1) → bit 10
    static constexpr uint8_t EC11E2_ENC2_A = 9;   // Inner shaft A (Low Cut in SSB/AM, Filter Width in CW/FSK/FM) - P11 (A2) → bit 9
    static constexpr uint8_t EC11E2_ENC2_B = 11;  // Inner shaft B (Low Cut in SSB/AM, Filter Width in CW/FSK/FM) - P13 (B2) → bit 11
    static constexpr uint8_t EC11E2_SW = 12;      // Outer shaft switch (IF Shift/High Cut reset, active low) - P14 (D1) → bit 12

    void handlePCF8575Change(uint8_t pinNumber, bool newState);
    void processI2CEncoder(I2CEncoderState& state, bool pinA, bool pinB, bool pinSw,
                          EncoderId encoderId);
    int8_t decodeQuadrature(uint8_t oldState, uint8_t newState);
    uint8_t updateAccelBand(I2CEncoderState& state, int64_t now);
    int32_t getAccelMultiplier(uint8_t band, EncoderId encoderId) const;
};
