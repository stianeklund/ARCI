#include "include/MultiEncoderHandler.h"
#include "../../include/pin_definitions.h"
#include "QuadratureDecoder.h"  // Shared quadrature decoding logic
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MultiEncoderHandler";

MultiEncoderHandler::MultiEncoderHandler(radio::RadioManager *radioManager)
    : m_radioManager(radioManager)
      , m_multiKnobEncoder(nullptr)
      , m_pcf8575(nullptr)
      , m_encoderCallback(nullptr)
      , m_switchCallback(nullptr) {
    // Initialize I2C encoder states (with acceleration tracking)
    m_rfGainState = {0, 0, false, 0, 0, 0, 0, 0};
    m_afGainState = {0, 0, false, 0, 0, 0, 0, 0};
    m_ec11e2Enc1State = {0, 0, false, 0, 0, 0, 0, 0};
    m_ec11e2Enc2State = {0, 0, false, 0, 0, 0, 0, 0};
    m_multiKnobState = {0, 0, false, 0, 0, 0, 0, 0};
}

MultiEncoderHandler::~MultiEncoderHandler() {
    stopAll();
}

esp_err_t MultiEncoderHandler::initializeDirectEncoders() {

    // Multi-knob encoder (GPIO13/14 + switch on GPIO15)
    m_multiKnobEncoder = std::make_unique<SimpleQuadratureEncoder>(
        PIN_MULTIKNOB_A,
        PIN_MULTIKNOB_B,
        PIN_MULTIKNOB_SW
    );

    // CRITICAL: Set callbacks BEFORE initialize() since the task starts immediately
    m_multiKnobEncoder->setRotationCallback([this](int32_t delta) {
        if (m_encoderCallback) {
            // Apply acceleration based on rotation speed (same as I2C encoders)
            const int64_t now = esp_timer_get_time();
            const uint8_t accelBand = updateAccelBand(m_multiKnobState, now);
            const int32_t accelMult = getAccelMultiplier(accelBand, EncoderId::MULTI_KNOB);
            const int32_t acceleratedDelta = delta * accelMult;

            m_encoderCallback(EncoderId::MULTI_KNOB, acceleratedDelta);
        }
    });
    m_multiKnobEncoder->setSwitchCallback([this](bool pressed) {
        if (m_switchCallback) {
            m_switchCallback(EncoderId::MULTI_KNOB, pressed);
        }
    });

    esp_err_t ret = m_multiKnobEncoder->initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize multi-knob encoder: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t MultiEncoderHandler::initializeI2CEncoders(PCF8575Handler *pcf8575Handler) {
    if (pcf8575Handler == nullptr) {
        ESP_LOGE(TAG, "PCF8575Handler is null");
        return ESP_ERR_INVALID_ARG;
    }

    m_pcf8575 = pcf8575Handler;

    // Configure PCF8575 pins as inputs for EC11E #2 (High Cut / Low Cut control)
    // PCB layout: A1, A2, B1, B2, D1 on P10-P14 (A1/B1=outer, A2/B2=inner)
    esp_err_t ret = m_pcf8575->setPinMode(EC11E2_ENC1_A, true);   // P10 (A1): Outer shaft A (High Cut/IF Shift)
    ret |= m_pcf8575->setPinMode(EC11E2_ENC1_B, true);            // P12 (B1): Outer shaft B (High Cut/IF Shift)
    ret |= m_pcf8575->setPinMode(EC11E2_ENC2_A, true);            // P11 (A2): Inner shaft A (Low Cut/Filter Width)
    ret |= m_pcf8575->setPinMode(EC11E2_ENC2_B, true);            // P13 (B2): Inner shaft B (Low Cut/Filter Width)
    ret |= m_pcf8575->setPinMode(EC11E2_SW, true);                // P14 (D1): Outer shaft switch

    // Configure PCF8575 pins as inputs for EC11E #1 (AF/RF Gain)
    // PCB layout: A1, A2, B1, B2, D1 on P0-P4 (A1/B1=outer, A2/B2=inner)
    ret |= m_pcf8575->setPinMode(EC11E1_ENC1_A, true);            // P1 (A2): Inner shaft A (RF)
    ret |= m_pcf8575->setPinMode(EC11E1_ENC1_B, true);            // P3 (B2): Inner shaft B (RF)
    ret |= m_pcf8575->setPinMode(EC11E1_ENC2_A, true);            // P0 (A1): Outer shaft A (AF)
    ret |= m_pcf8575->setPinMode(EC11E1_ENC2_B, true);            // P2 (B1): Outer shaft B (AF)
    ret |= m_pcf8575->setPinMode(EC11E1_SW, true);                // P4 (D1): Outer shaft switch (AF mute)

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PCF8575 pins");
        return ret;
    }

    // Read initial states
    uint16_t pins = 0;
    ret = m_pcf8575->readAllPins(pins);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read initial PCF8575 state");
        return ret;
    }

    // Initialize EC11E #2 encoder states from current pin states
    const bool enc2_outerA = (pins & (1 << EC11E2_ENC1_A)) != 0;  // Outer shaft A
    const bool enc2_outerB = (pins & (1 << EC11E2_ENC1_B)) != 0;  // Outer shaft B
    const bool enc2_innerA = (pins & (1 << EC11E2_ENC2_A)) != 0;  // Inner shaft A
    const bool enc2_innerB = (pins & (1 << EC11E2_ENC2_B)) != 0;  // Inner shaft B
    const bool enc2_switchPressed = (pins & (1 << EC11E2_SW)) == 0; // Inner shaft switch (active low)

    m_ec11e2Enc1State.lastState = (enc2_outerA ? 0x02 : 0x00) | (enc2_outerB ? 0x01 : 0x00);
    m_ec11e2Enc2State.lastState = (enc2_innerA ? 0x02 : 0x00) | (enc2_innerB ? 0x01 : 0x00);
    m_ec11e2Enc1State.switchPressed = enc2_switchPressed;  // Outer shaft has switch (non-standard PCB wiring)
    m_ec11e2Enc2State.switchPressed = false;               // Inner shaft has no switch (non-standard PCB wiring)

    // Initialize EC11E #1 encoder states from current pin states
    const bool enc1_innerA = (pins & (1 << EC11E1_ENC1_A)) != 0;  // Inner shaft A (RF)
    const bool enc1_innerB = (pins & (1 << EC11E1_ENC1_B)) != 0;  // Inner shaft B (RF)
    const bool enc1_outerA = (pins & (1 << EC11E1_ENC2_A)) != 0;  // Outer shaft A (AF)
    const bool enc1_outerB = (pins & (1 << EC11E1_ENC2_B)) != 0;  // Outer shaft B (AF)
    const bool enc1_sw = (pins & (1 << EC11E1_SW)) == 0;          // Inner shaft switch (active low)

    m_rfGainState.lastState = (enc1_innerA ? 0x02 : 0x00) | (enc1_innerB ? 0x01 : 0x00);
    m_afGainState.lastState = (enc1_outerA ? 0x02 : 0x00) | (enc1_outerB ? 0x01 : 0x00);
    m_rfGainState.switchPressed = false;    // Inner shaft has no switch (non-standard PCB wiring)
    m_afGainState.switchPressed = enc1_sw;  // Outer shaft has switch (non-standard PCB wiring)

    // Register pin change callback
    m_pcf8575->setChangeCallback([this](uint8_t pin, bool state) {
        handlePCF8575Change(pin, state);
    });

    return ESP_OK;
}

void MultiEncoderHandler::setEncoderCallback(EncoderCallback callback) {
    m_encoderCallback = callback;
}

void MultiEncoderHandler::setSwitchCallback(SwitchCallback callback) {
    m_switchCallback = callback;
}

void MultiEncoderHandler::startAll() {
    // SimpleQuadratureEncoder starts automatically after initialize()
}

void MultiEncoderHandler::stopAll() {
    // Encoders will be stopped when destroyed (in destructor)
}

void MultiEncoderHandler::handlePCF8575Change(uint8_t pinNumber, bool newState) {
    // Read all pins to get consistent state
    const uint16_t pins = m_pcf8575->getLastPinStates();

    // Extract EC11E #2 (High Cut/Low Cut control) states
    const bool enc2_outerA = (pins & (1 << EC11E2_ENC1_A)) != 0;  // Outer shaft A
    const bool enc2_outerB = (pins & (1 << EC11E2_ENC1_B)) != 0;  // Outer shaft B
    const bool enc2_innerA = (pins & (1 << EC11E2_ENC2_A)) != 0;  // Inner shaft A
    const bool enc2_innerB = (pins & (1 << EC11E2_ENC2_B)) != 0;  // Inner shaft B
    const bool enc2_sw = (pins & (1 << EC11E2_SW)) == 0;          // Inner shaft switch (active low)

    // Extract EC11E #1 (AF/RF Gain) states
    const bool enc1_innerA = (pins & (1 << EC11E1_ENC1_A)) != 0;  // Inner shaft A (RF)
    const bool enc1_innerB = (pins & (1 << EC11E1_ENC1_B)) != 0;  // Inner shaft B (RF)
    const bool enc1_outerA = (pins & (1 << EC11E1_ENC2_A)) != 0;  // Outer shaft A (AF)
    const bool enc1_outerB = (pins & (1 << EC11E1_ENC2_B)) != 0;  // Outer shaft B (AF)
    const bool enc1_sw = (pins & (1 << EC11E1_SW)) == 0;          // Inner shaft switch (active low)

    // Process EC11E #2 encoders (High Cut/Low Cut control)
    processI2CEncoder(m_ec11e2Enc1State, enc2_outerA, enc2_outerB, enc2_sw, EncoderId::EC11E2_ENC1);   // Outer shaft (with switch - non-std wiring)
    processI2CEncoder(m_ec11e2Enc2State, enc2_innerA, enc2_innerB, false, EncoderId::EC11E2_ENC2);     // Inner shaft (no switch - non-std wiring)

    // Process EC11E #1 encoders (AF/RF Gain)
    processI2CEncoder(m_rfGainState, enc1_innerA, enc1_innerB, false, EncoderId::RF_GAIN);    // Inner shaft (no switch - non-std wiring)
    processI2CEncoder(m_afGainState, enc1_outerA, enc1_outerB, enc1_sw, EncoderId::AF_GAIN);  // Outer shaft (with switch - non-std wiring)
}

void MultiEncoderHandler::processI2CEncoder(I2CEncoderState &state, bool pinA, bool pinB,
                                            bool pinSw, EncoderId encoderId) {
    // Build new 2-bit state
    const uint8_t newState = (pinA ? 0x02 : 0x00) | (pinB ? 0x01 : 0x00);

    // Decode quadrature transition
    const int8_t delta = decodeQuadrature(state.lastState, newState);

    if (delta != 0) {
        // Check if we're within debounce window after switch press
        const int64_t now = esp_timer_get_time();
        const int64_t timeSincePress = now - state.lastSwitchPressTime;

        if (timeSincePress < SWITCH_ROTATION_DEBOUNCE_US) {
            // Ignore rotation during switch debounce window
            state.invalidTransitionCount = 0;  // Reset jitter counter
            state.accumulator = 0;  // Reset accumulator
        } else {
            // Accumulate transitions (2 transitions = 1 detent for EC11E encoders)
            state.accumulator += delta;
            state.invalidTransitionCount = 0;  // Reset jitter counter on valid transition

            ESP_LOGD(TAG, "Encoder %d: delta=%d, accumulator=%d (threshold=%d)",
                     static_cast<int>(encoderId), delta, state.accumulator, ACCUMULATOR_THRESHOLD);

            // Only fire callback when accumulator reaches threshold
            if (abs(state.accumulator) >= ACCUMULATOR_THRESHOLD) {
                const int8_t steps = state.accumulator / ACCUMULATOR_THRESHOLD;
                state.position += steps;
                state.accumulator = state.accumulator % ACCUMULATOR_THRESHOLD;  // Keep remainder

                ESP_LOGV(TAG, "Encoder %d: FIRED steps=%d, remainder=%d",
                         static_cast<int>(encoderId), steps, state.accumulator);

                // Apply acceleration based on rotation speed (encoder-specific)
                const uint8_t accelBand = updateAccelBand(state, now);
                const int32_t accelMult = getAccelMultiplier(accelBand, encoderId);
                const int32_t acceleratedSteps = steps * accelMult;

                if (m_encoderCallback) {
                    m_encoderCallback(encoderId, acceleratedSteps);
                }
            }
        }
    } else if (state.lastState != newState) {
        // Invalid transition detected (quadrature error)
        state.invalidTransitionCount++;
        if (state.invalidTransitionCount > MAX_INVALID_TRANSITIONS) {
            state.invalidTransitionCount = MAX_INVALID_TRANSITIONS;
        }
    } else {
        // No state change, reset jitter counter
        state.invalidTransitionCount = 0;
    }

    state.lastState = newState;

    // Check switch state change
    if (pinSw != state.switchPressed) {
        state.switchPressed = pinSw;

        // Record timestamp when switch is pressed
        if (pinSw) {
            state.lastSwitchPressTime = esp_timer_get_time();
        }

        if (m_switchCallback) {
            m_switchCallback(encoderId, pinSw);
        }
    }
}

int8_t MultiEncoderHandler::decodeQuadrature(uint8_t oldState, uint8_t newState) {
    return quadrature::decode(oldState, newState);
}

uint8_t MultiEncoderHandler::updateAccelBand(I2CEncoderState& state, int64_t now) {
    // Calculate detents per second based on time since last detent
    float detentsPerSec = 0.0f;
    if (state.lastDetentTime > 0) {
        const int64_t timeDelta = now - state.lastDetentTime;
        if (timeDelta > 0) {
            detentsPerSec = 1000000.0f / static_cast<float>(timeDelta);  // Convert μs to detents/sec
        }
    }

    // Determine new acceleration band based on speed
    uint8_t newBand = 0;
    if (detentsPerSec >= ACCEL_BAND4_DETENTS_PER_SEC) {
        newBand = 4;
    } else if (detentsPerSec >= ACCEL_BAND3_DETENTS_PER_SEC) {
        newBand = 3;
    } else if (detentsPerSec >= ACCEL_BAND2_DETENTS_PER_SEC) {
        newBand = 2;
    } else if (detentsPerSec >= ACCEL_BAND1_DETENTS_PER_SEC) {
        newBand = 1;
    }

    // Apply hysteresis for downshift to prevent oscillation
    if (newBand < state.accelBand && state.accelBand > 0) {
        // Calculate downshift threshold with hysteresis
        float downThreshold = 0.0f;

        switch (state.accelBand) {
            case 1: downThreshold = ACCEL_BAND1_DETENTS_PER_SEC * (1.0f - ACCEL_HYSTERESIS); break;
            case 2: downThreshold = ACCEL_BAND2_DETENTS_PER_SEC * (1.0f - ACCEL_HYSTERESIS); break;
            case 3: downThreshold = ACCEL_BAND3_DETENTS_PER_SEC * (1.0f - ACCEL_HYSTERESIS); break;
            case 4: downThreshold = ACCEL_BAND4_DETENTS_PER_SEC * (1.0f - ACCEL_HYSTERESIS); break;
        }

        if (detentsPerSec > downThreshold) {
            newBand = state.accelBand;  // Stay in current band (hysteresis)
        }
    }

    state.accelBand = newBand;
    state.lastDetentTime = now;
    return newBand;
}

int32_t MultiEncoderHandler::getAccelMultiplier(uint8_t band, EncoderId encoderId) const {
    // EC11E #2 encoders (filter/IF shift): no acceleration - 1:1 control
    // These control discrete filter values where precise adjustment is needed
    if (encoderId == EncoderId::EC11E2_ENC1 || encoderId == EncoderId::EC11E2_ENC2) {
        return 1;
    }

    // MULTI_KNOB: no acceleration - needs precise 1:1 control for UI popups
    // The step size is already configured per-control (e.g., step=5 for power)
    if (encoderId == EncoderId::MULTI_KNOB) {
        return 1;  // Always 1x - no acceleration
    }

    // AF/RF gain encoders: aggressive acceleration for 0-255 range
    switch (band) {
        case 0: return ACCEL_MULT_BAND0;
        case 1: return ACCEL_MULT_BAND1;
        case 2: return ACCEL_MULT_BAND2;
        case 3: return ACCEL_MULT_BAND3;
        case 4: return ACCEL_MULT_BAND4;
        default: return ACCEL_MULT_BAND0;
    }
}

void MultiEncoderHandler::configureDefaultCallbacks() {
    // Guard window for AF gain switch press (ignore rotation within 150ms)
    static constexpr uint64_t AF_SWITCH_GUARD_US = 150000;

    setEncoderCallback([this](EncoderId encoderId, int32_t delta) {
        ESP_LOGV(TAG, "Encoder %d rotated: delta=%ld", static_cast<int>(encoderId), delta);

        // Check if radio interface is on before processing encoder changes
        if (m_radioManager->getOnOffState() != 1) {
            ESP_LOGV(TAG, "Radio interface OFF, ignoring encoder input");
            return;
        }

        // Signal user activity to wake display (debounced UIPS1)
        m_radioManager->signalUserActivity();

        auto &state = m_radioManager->getState();

        switch (encoderId) {
        case EncoderId::RF_GAIN: {
            // RF Gain encoder: 0-255 range, accumulate delta
            int rfGainValue = state.rfGain.load();
            rfGainValue += static_cast<int>(delta);
            if (rfGainValue < 0) rfGainValue = 0;
            if (rfGainValue > 255) rfGainValue = 255;

            // Set flag to prevent race condition with polling
            state.isAdjustingRfGain.store(true);
            state.rfGainAdjustTime.store(esp_timer_get_time());
            m_radioManager->setRfGain(rfGainValue);
            break;
        }

        case EncoderId::AF_GAIN: {
            // AF Gain encoder: 0-255 range, accumulate delta
            // Ignore rotation events within 150ms of switch press
            const uint64_t now = esp_timer_get_time();
            if ((now - m_afGainSwitchPressTime.load()) < AF_SWITCH_GUARD_US) {
                ESP_LOGV(TAG, "AF_GAIN rotation ignored (within 150ms of switch press)");
                return;
            }

            int afGainValue = state.afGain.load();
            afGainValue += static_cast<int>(delta);
            if (afGainValue < 0) afGainValue = 0;
            if (afGainValue > 255) afGainValue = 255;

            // Set flag to prevent race condition with polling
            state.isAdjustingAfGain.store(true);
            state.afGainAdjustTime.store(esp_timer_get_time());
            m_radioManager->setAfGain(afGainValue);
            break;
        }

        case EncoderId::MULTI_KNOB: {
            // Multi-knob encoder: UI adjustment mode or normal CH command mode
            // When UI mode is active, adjust the UI value instead of sending CH commands

            if (m_radioManager->isUIModeActive())
            {
                // UI mode: adjust the active control value
                m_radioManager->adjustUIValue(static_cast<int>(delta));
                m_radioManager->recordEncoderActivity();
                ESP_LOGV(TAG, "🎛️ MULTI_KNOB: UI value adjusted (delta=%ld)", delta);
            }
            else
            {
                // Normal mode: Send CH (MULTI/CH encoder) CAT commands to radio
                // CH0; = Up one step (CW rotation), CH1; = Down one step (CCW rotation)
                const int32_t absDelta = std::abs(delta);
                const char *command = (delta > 0) ? "CH0;" : "CH1;";

                // Send one CH command per encoder step
                for (int32_t i = 0; i < absDelta; i++) {
                    m_radioManager->dispatchMessage(m_radioManager->getPanelCATHandler(), command);
                }
            }
            break;
        }

        case EncoderId::EC11E2_ENC1: {
            // EC11E #2 Encoder 1 (Outer shaft): High Cut (LSB/USB/AM) or IF Shift (CW/CW-R)
            const int currentMode = state.mode.load();
            if (currentMode == 3 || currentMode == 7) {  // CW or CW-R
                // IF Shift: increment by 100 Hz per click
                const int currentShift = state.ifShiftValue.load();
                int newShift = currentShift + (static_cast<int>(delta) * 100);
                if (newShift < 0) newShift = 0;
                if (newShift > 9999) newShift = 9999;

                m_radioManager->setIfShift(newShift);
                ESP_LOGI(TAG, "🎛️ EC11E2_ENC1: IF Shift %d → %d Hz", currentShift, newShift);
            } else if (currentMode == 1 || currentMode == 2 || currentMode == 5) {  // LSB/USB/AM
                // High Cut: adjust by delta
                m_radioManager->setHighCut(static_cast<int>(delta));
                ESP_LOGV(TAG, "🎛️ EC11E2_ENC1: HIGH CUT adjusted (delta=%ld)", delta);
            } else {
                ESP_LOGV(TAG, "EC11E2_ENC1: No function in current mode (%d)", currentMode);
            }
            break;
        }

        case EncoderId::EC11E2_ENC2: {
            // EC11E #2 Encoder 2 (Inner shaft with switch): Low Cut (LSB/USB/AM) or Filter Width (CW/FM/FSK)
            const int currentMode = state.mode.load();

            if (currentMode == 1 || currentMode == 2 || currentMode == 5) {  // LSB/USB/AM
                // Low Cut: adjust by delta
                m_radioManager->setLowCut(static_cast<int>(delta));
                ESP_LOGV(TAG, "🎛️ EC11E2_ENC2: LOW CUT adjusted (delta=%ld)", delta);
            } else {
                // Other modes: use setFilterWidth for CW/FM/FSK bandwidth control
                m_radioManager->setFilterWidth(static_cast<int>(delta));
                ESP_LOGV(TAG, "🎛️ EC11E2_ENC2: Filter width adjusted (delta=%ld)", delta);
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown encoder ID: %d", static_cast<int>(encoderId));
            break;
        }
    });

    setSwitchCallback([this](EncoderId encoderId, bool isPressed) {
        // Only act on button press, not release
        if (!isPressed) {
            return;
        }

        // Signal user activity to wake display (debounced UIPS1)
        m_radioManager->signalUserActivity();

        auto &state = m_radioManager->getState();

        switch (encoderId) {
        case EncoderId::AF_GAIN: {
            // AF Gain switch: mute/unmute audio (PCB has switch on outer/AF shaft, non-standard wiring)
            // Record timestamp of switch press to guard against accidental rotation
            m_afGainSwitchPressTime.store(esp_timer_get_time());

            m_afMuted = !m_afMuted;  // Toggle mute state

            if (m_afMuted) {
                // Mute: save current gain and set to 0
                m_savedAfGainValue = state.afGain.load();
                m_radioManager->setAfGain(0);
                ESP_LOGI(TAG, "🔇 AF MUTED (saved gain: %d)", m_savedAfGainValue);
            } else {
                // Unmute: restore saved gain
                m_radioManager->setAfGain(m_savedAfGainValue);
                ESP_LOGI(TAG, "🔊 AF UNMUTED (restored gain: %d)", m_savedAfGainValue);
            }
            break;
        }

        case EncoderId::RF_GAIN:
            ESP_LOGI(TAG, "🔘 RF_GAIN switch pressed");
            // RF Gain encoder has no switch on this PCB (non-standard wiring)
            break;

        case EncoderId::MULTI_KNOB:
            // Multi-knob switch: Confirm UI mode selection or toggle VFO
            if (m_radioManager->isUIModeActive())
            {
                // UI mode active: confirm and apply the value, then exit UI mode
                ESP_LOGI(TAG, "🔘 MULTI_KNOB: Confirming UI selection");
                m_radioManager->exitUIMode(true);  // Apply the value
            }
            else
            {
                // Normal mode: Toggle VFO A/B (FR command)
                const int currentVfo = state.currentRxVfo.load();
                const char *command = (currentVfo == 0) ? "FR1;" : "FR0;";
                m_radioManager->dispatchMessage(m_radioManager->getPanelCATHandler(), command);
                ESP_LOGI(TAG, "🔘 MULTI_KNOB: VFO toggle %s -> %s",
                         currentVfo == 0 ? "A" : "B",
                         currentVfo == 0 ? "B" : "A");
            }
            break;

        case EncoderId::EC11E2_ENC1: {
            // EC11E #2 Encoder 1 switch (outer shaft - non-standard wiring): Reset IF Shift or High Cut
            const int currentMode = state.mode.load();

            if (currentMode == 3 || currentMode == 7) {  // CW or CW-R
                // Reset IF Shift to center
                m_radioManager->resetIfShift();
                ESP_LOGI(TAG, "🔘 EC11E2_ENC1: IF Shift reset to 0 Hz");
            } else if (currentMode == 1 || currentMode == 2 || currentMode == 5) {  // LSB/USB/AM
                // Reset High Cut to default
                ESP_LOGI(TAG, "🔘 EC11E2_ENC1: High cut reset");
                // TODO: Implement high cut reset to default value
            } else {
                ESP_LOGV(TAG, "EC11E2_ENC1 switch: No function in current mode %d", currentMode);
            }
            break;
        }

        case EncoderId::EC11E2_ENC2:
            // EC11E #2 Encoder 2: No switch (inner shaft - non-standard wiring)
            ESP_LOGW(TAG, "EC11E2_ENC2 switch event (unexpected - inner shaft has no switch on this PCB)");
            break;

        default:
            break;
        }
    });
}
