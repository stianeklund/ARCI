#include "EncoderHandler.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits.h>
#include <limits>
#include "esp_log.h"

#include "DisplayLatencyProfiler.h"
#include "ForwardingPolicy.h"
#include "FrequencyFormatter.h"
#include "esp_timer.h"
#include "sdkconfig.h"

// Fallback defaults if Kconfig not available
#ifndef CONFIG_TS590_DEFAULT_PPR
#define CONFIG_TS590_DEFAULT_PPR 250
#endif

#ifndef CONFIG_ENCODER_ACCEL_ENABLE
#define CONFIG_ENCODER_ACCEL_ENABLE 1
#endif
// Discrete acceleration thresholds (rps x100)
#ifndef CONFIG_ENCODER_BAND1_RPS_X100
#define CONFIG_ENCODER_BAND1_RPS_X100 100 // 1.00 rps → medium
#endif
#ifndef CONFIG_ENCODER_BAND2_RPS_X100
#define CONFIG_ENCODER_BAND2_RPS_X100 300 // 3.00 rps → fast
#endif
// Hysteresis percent (e.g., 20)
#ifndef CONFIG_ENCODER_BAND_HYSTERESIS_PERCENT
#define CONFIG_ENCODER_BAND_HYSTERESIS_PERCENT 20
#endif
// PCNT usage and filter
#ifndef CONFIG_ENCODER_USE_PCNT
#define CONFIG_ENCODER_USE_PCNT 1
#endif
#ifndef CONFIG_ENCODER_PCNT_GLITCH_US
#define CONFIG_ENCODER_PCNT_GLITCH_US 6
#endif
// GPIO software filter fallback
#ifndef CONFIG_ENCODER_GPIO_SW_FILTER_US
#define CONFIG_ENCODER_GPIO_SW_FILTER_US 20
#endif
// Verbose logging control
#ifndef CONFIG_ENCODER_VERBOSE_LOGS
#define CONFIG_ENCODER_VERBOSE_LOGS 0
#endif
// Optional: send UP/DN increments to the radio instead of absolute FA/FB
#ifndef CONFIG_ENCODER_RADIO_USE_RELATIVE_CMDS
#define CONFIG_ENCODER_RADIO_USE_RELATIVE_CMDS 0
#endif

static const char *TAG = "EncoderHandler";

// Verbose logging macro - compiles out in production builds
#if CONFIG_ENCODER_VERBOSE_LOGS
#define ENCODER_LOGV(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define ENCODER_LOGD(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#else
#define ENCODER_LOGV(format, ...)                                                                                      \
    do                                                                                                                 \
    {                                                                                                                  \
    }                                                                                                                  \
    while (0)
#define ENCODER_LOGD(format, ...)                                                                                      \
    do                                                                                                                 \
    {                                                                                                                  \
    }                                                                                                                  \
    while (0)
#endif

// Base radio frequency limits (TS-590SG baseband)
static constexpr uint64_t BASE_MIN_FREQUENCY_HZ = 30000; // 30 kHz minimum
static constexpr uint64_t BASE_MAX_FREQUENCY_HZ = 60000000; // 60 MHz maximum

// Time to wait after the knob stops turning before sending the final frequency (configurable via Kconfig).
static constexpr int64_t FINAL_UPDATE_DELAY_US =
    CONFIG_ENCODER_FINAL_UPDATE_DELAY_MS * 1000; // Convert ms to microseconds
// Minimum interval between frequency updates while the knob is actively turning (configurable via Kconfig).
static constexpr int64_t LIVE_UPDATE_INTERVAL_US =
    CONFIG_ENCODER_LIVE_UPDATE_INTERVAL_MS * 1000; // Convert ms to microseconds

// Quadrature encoder state transition table.
static constexpr int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

EncoderHandler::EncoderHandler(const gpio_num_t pinA, const gpio_num_t pinB, radio::RadioManager *radioManager) :
    m_pinA(pinA), m_pinB(pinB), m_radioManager(radioManager), m_taskHandle(nullptr), m_movementSemaphore(nullptr),
    m_position(0), m_lastMovementTime(0), m_state(0), m_lastReportedPosition(0), m_lastUpdateTime(0),
    m_isTuning(false), m_cachedTuneVfo(-1), m_pulsesPerRev(CONFIG_TS590_DEFAULT_PPR), m_edgeRemainder(0),
    m_accelBand(0), m_lastSampleTimeUs(0), m_lastLogicalDelta(0), m_stepSizeCache(0)
{
#if CONFIG_ENCODER_VERBOSE_LOGS
    // Elevate EncoderHandler logs to DEBUG when verbose mode is enabled without touching the global level
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    m_spinlock = portMUX_INITIALIZER_UNLOCKED;
    // Create binary semaphore for event-driven processing
    m_movementSemaphore = xSemaphoreCreateBinary();
    if (m_movementSemaphore == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create movement semaphore");
    }
    // Create semaphore for clean shutdown synchronization
    m_stopSemaphore = xSemaphoreCreateBinary();
    if (m_stopSemaphore == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create stop semaphore");
    }
}

EncoderHandler::~EncoderHandler()
{
    stop(); // Ensure task is stopped
    if (m_movementSemaphore != nullptr)
    {
        vSemaphoreDelete(m_movementSemaphore);
        m_movementSemaphore = nullptr;
    }
    if (m_stopSemaphore != nullptr)
    {
        vSemaphoreDelete(m_stopSemaphore);
        m_stopSemaphore = nullptr;
    }
}

void EncoderHandler::setup()
{

#if CONFIG_ENCODER_USE_PCNT // Re-enable PCNT mode
    // Configure GPIOs for PCNT input with pull-ups - NO interrupts for PCNT mode
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE; // NO interrupts when using PCNT
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << m_pinA) | (1ULL << m_pinB);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    m_state = (gpio_get_level(m_pinA) << 1) | gpio_get_level(m_pinB);

    // Configure new PCNT unit
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.high_limit = INT16_MAX;
    unit_cfg.low_limit = INT16_MIN;
    esp_err_t unit_err = pcnt_new_unit(&unit_cfg, &m_pcntUnit);
    if (unit_err != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT unit creation failed: %s", esp_err_to_name(unit_err));
        return;
    }

    // Glitch filter - start conservative (higher value = less filtering)
    pcnt_glitch_filter_config_t filt = {};
    filt.max_glitch_ns = 10000U; // 10 microseconds - conservative setting
    esp_err_t filt_err = pcnt_unit_set_glitch_filter(m_pcntUnit, &filt);
    if (filt_err != ESP_OK)
    {
        ESP_LOGW(TAG, "PCNT glitch filter setup failed: %s", esp_err_to_name(filt_err));
    }

    // Full x4 quadrature decoding with two channels for 1000 edges/rev
    // Channel 0: A signal as pulse, B signal as level control
    pcnt_channel_handle_t ch0;
    pcnt_chan_config_t ch0_cfg = {};
    ch0_cfg.edge_gpio_num = m_pinA; // A as pulse (edge source)
    ch0_cfg.level_gpio_num = m_pinB; // B as level (direction control)
    esp_err_t ch0_err = pcnt_new_channel(m_pcntUnit, &ch0_cfg, &ch0);
    if (ch0_err != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT channel 0 creation failed: %s", esp_err_to_name(ch0_err));
        return;
    }
    // Channel 0: A edges controlled by B level - CW rotation increments
    // When B=0: A rising decrements, A falling increments
    // When B=1: A rising increments, A falling decrements
    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(ch0, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(ch0, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Channel 1: B signal as pulse, A signal as level control (enables x4 decoding)
    pcnt_channel_handle_t ch1;
    pcnt_chan_config_t ch1_cfg = {};
    ch1_cfg.edge_gpio_num = m_pinB; // B as pulse (edge source)
    ch1_cfg.level_gpio_num = m_pinA; // A as level (direction control)
    esp_err_t ch1_err = pcnt_new_channel(m_pcntUnit, &ch1_cfg, &ch1);
    if (ch1_err != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT channel 1 creation failed: %s", esp_err_to_name(ch1_err));
        return;
    }
    // Channel 1: B edges controlled by A level - matches Ch0 for proper quadrature
    // When A=0: B rising increments, B falling decrements
    // When A=1: B rising decrements, B falling increments
    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(ch1, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(ch1, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Check if GPIOs are actually routed to PCNT - try toggling GPIO direction temporarily

    esp_err_t enable_err = pcnt_unit_enable(m_pcntUnit);
    if (enable_err != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT unit enable failed: %s", esp_err_to_name(enable_err));
        return;
    }

    esp_err_t clear_err = pcnt_unit_clear_count(m_pcntUnit);
    if (clear_err != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT clear failed: %s", esp_err_to_name(clear_err));
    }

    esp_err_t start_err = pcnt_unit_start(m_pcntUnit);
    if (start_err != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT unit start failed: %s", esp_err_to_name(start_err));
        return;
    }

    // Verify PCNT is operational and clear initial state
    int initial_count = 0;
    esp_err_t rc = pcnt_unit_get_count(m_pcntUnit, &initial_count);
    if (rc != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT initial count read failed: %s", esp_err_to_name(rc));
        m_lastPcntCount = 0;
    }
    else
    {
        m_lastPcntCount = initial_count;
    }

    m_usePcnt = true;

    // Add wake ISRs to trigger task processing when encoder moves
    ESP_ERROR_CHECK(gpio_isr_handler_add(m_pinA, wakeIsrHandler, this));
    ESP_ERROR_CHECK(gpio_isr_handler_add(m_pinB, wakeIsrHandler, this));
#else
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << m_pinA) | (1ULL << m_pinB);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    m_state = (gpio_get_level(m_pinA) << 1) | gpio_get_level(m_pinB);
    ESP_ERROR_CHECK(gpio_isr_handler_add(m_pinA, isrHandler, this));
    ESP_ERROR_CHECK(gpio_isr_handler_add(m_pinB, isrHandler, this));
    m_usePcnt = false;
#endif

    ESP_LOGD(TAG, "Encoder setup complete - %s mode", m_usePcnt ? "PCNT" : "GPIO ISR");
}

void EncoderHandler::start()
{
    if (m_taskHandle != nullptr)
    {
        ESP_LOGW(TAG, "EncoderHandler task already running");
        return;
    }

    m_running.store(true, std::memory_order_release);
    xTaskCreate(encoderTask, "EncoderTask", 4096, this, 12, &m_taskHandle); // High priority for tuning responsiveness
}

void EncoderHandler::stop()
{
    if (m_taskHandle == nullptr)
    {
        return;
    }

    // Signal the task to stop
    m_running.store(false, std::memory_order_release);

    // Give the movement semaphore to wake the task if it's waiting
    if (m_movementSemaphore != nullptr)
    {
        xSemaphoreGive(m_movementSemaphore);
    }

    // Wait for task to signal completion via semaphore (with timeout)
    if (m_stopSemaphore != nullptr)
    {
        constexpr TickType_t stopTimeoutTicks = pdMS_TO_TICKS(500);
        if (xSemaphoreTake(m_stopSemaphore, stopTimeoutTicks) != pdTRUE)
        {
            // Timeout - task didn't respond, force delete
            ESP_LOGW(TAG, "EncoderHandler task did not exit cleanly, forcing deletion");
            vTaskDelete(m_taskHandle);
        }
    }
    else
    {
        // No semaphore, fall back to delay
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    m_taskHandle = nullptr;
    ESP_LOGI(TAG, "EncoderHandler task stopped");
}

void EncoderHandler::encoderTask(void *arg)
{
    const auto handler = static_cast<EncoderHandler *>(arg);

    while (handler->m_running.load(std::memory_order_acquire))
    {
        // Wait for encoder movement signal (or timeout for periodic cleanup)
        const bool movementDetected = xSemaphoreTake(handler->m_movementSemaphore, pdMS_TO_TICKS(50)) == pdTRUE;
        handler->task(movementDetected);
    }

    // Signal stop() that we're exiting cleanly
    if (handler->m_stopSemaphore != nullptr)
    {
        xSemaphoreGive(handler->m_stopSemaphore);
    }

    vTaskDelete(nullptr);
}

void EncoderHandler::task(const bool movementDetected)
{
    if (!m_radioManager)
        return;

    // Check panel lock state - block encoder input when locked
    if (m_radioManager->getState().panelLock)
    {
        // Panel is locked - ignore all encoder input
        // Clear any pending movement to reset state properly
        if (m_isTuning)
        {
            m_isTuning = false;
            m_radioManager->getState().isTuning.store(false);
            m_cachedTuneVfo = -1;
        }
        return;
    }

    // Obtain raw edge delta since last call
    int32_t edgeDelta = 0;
    int64_t lastMovementTime = m_lastMovementTime.load();
    if (m_usePcnt)
    {
        int cnt = 0;
        const esp_err_t rc = pcnt_unit_get_count(m_pcntUnit, &cnt);
        if (rc != ESP_OK)
        {
            ESP_LOGE(TAG, "PCNT read failed: %s", esp_err_to_name(rc));
        }
        else
        {
            edgeDelta = cnt - m_lastPcntCount; // Delta since last read
            m_lastPcntCount = cnt;

            if (edgeDelta != 0)
            {
                lastMovementTime = esp_timer_get_time();
#if CONFIG_ENCODER_VERBOSE_LOGS
                m_edgeCountForValidation += std::abs(edgeDelta);
                if (m_lastValidationResetTime == 0)
                {
                    m_lastValidationResetTime = lastMovementTime;
                }
                else if ((lastMovementTime - m_lastValidationResetTime) > 5000000)
                { // Every 5 seconds
                    const uint32_t totalEdges = m_edgeCountForValidation;
                    const float timeSeconds = (lastMovementTime - m_lastValidationResetTime) / 1000000.0f;
                    const float edgesPerSec = totalEdges / timeSeconds;
                    ESP_LOGI(TAG, "PCNT validation: %u edges in %.1fs (%.0f edges/sec)", totalEdges, timeSeconds,
                             edgesPerSec);
                    m_edgeCountForValidation = 0;
                    m_lastValidationResetTime = lastMovementTime;
                }
#endif
            }
            else if (movementDetected)
            {
                // Wake triggered but no delta yet; preserve ISR timestamp
                lastMovementTime = esp_timer_get_time();
            }

            // Clear PCNT when it gets close to limits to prevent overflow
            if (std::abs(cnt) > 10000)
            {
                pcnt_unit_clear_count(m_pcntUnit);
                m_lastPcntCount = 0;
            }
        }
    }
    else
    {
        portENTER_CRITICAL(&m_spinlock);
        const int32_t currentPosition = m_position;
        portEXIT_CRITICAL(&m_spinlock);
        lastMovementTime = m_lastMovementTime.load();
        edgeDelta = currentPosition - m_lastReportedPosition;
        m_lastReportedPosition = currentPosition;
    }
    int ppr = m_pulsesPerRev.load();
    // Convert raw x4 edges to logical pulses per revolution (TS-590SG Menu 13)
    // 1000 raw edges → 250 pulses (edgeDiv=4), 500 pulses (edgeDiv=2), 1000 pulses (edgeDiv=1)
    int edgeDiv = std::max(1, RAW_EDGES_PER_REV / ppr);

    // Accumulate fractional edges across calls
    m_edgeRemainder += edgeDelta;
    int32_t delta = 0;
    while (m_edgeRemainder >= edgeDiv)
    {
        ++delta;
        m_edgeRemainder -= edgeDiv;
    }
    while (m_edgeRemainder <= -edgeDiv)
    {
        --delta;
        m_edgeRemainder += edgeDiv;
    }


    const int64_t currentTime = esp_timer_get_time();

    // Check if encoder is idle (final snap delay) - widened for stability
    static constexpr int64_t IDLE_TIMEOUT_US = 100000; // 100ms
    if (delta == 0 || (currentTime - lastMovementTime) > IDLE_TIMEOUT_US)
    {
        // If there's no change or encoder is idle, ensure the local tuning flag is false
        if (m_isTuning)
        {
            m_isTuning = false;
            m_radioManager->getState().isTuning.store(false);
            m_cachedTuneVfo = -1;
            ESP_LOGD(TAG, "Local tuning stopped (idle/no delta)");
        }
        return;
    }

    // Require minimum encoder steps before sending frequency update (configurable)
    if (std::abs(delta) < CONFIG_ENCODER_MIN_STEPS_THRESHOLD)
    {
        ESP_LOGI(TAG, "Encoder delta too small (%ld < %d), waiting for more steps", delta,
                 CONFIG_ENCODER_MIN_STEPS_THRESHOLD);
        return;
    }

    ESP_LOGD(TAG, "Encoder delta: %ld (PASSED threshold)", (long)delta);

    // Signal user activity to wake display (debounced UIPS1)
    m_radioManager->signalUserActivity();

    const int64_t timeSinceLastMove = currentTime - lastMovementTime;
    const int64_t timeSinceLastUpdate = currentTime - m_lastUpdateTime;

    bool shouldUpdate = false;
    constexpr int64_t liveGateUs = LIVE_UPDATE_INTERVAL_US;

    // Live updates on a steady cadence while turning
    ENCODER_LOGV("Timing check: timeSinceLastUpdate=%lld, liveGateUs=%lld", timeSinceLastUpdate, liveGateUs);
    if (timeSinceLastUpdate >= liveGateUs)
    {
        shouldUpdate = true;
        ENCODER_LOGV("shouldUpdate=true (live gate)");
        if (!m_isTuning)
        {
            m_isTuning = true; // Set local tuning flag
            // Notify RadioManager that local tuning has started
            auto &state = m_radioManager->getState();
            state.isTuning.store(true);
            state.tuningStartTime.store(currentTime);
            // Latch active VFO at start of turn; remain constant during this turn
            // Active VFO = TX VFO when transmitting, RX VFO when receiving
            const bool isTxActive = state.isTx.load();
            m_cachedTuneVfo = isTxActive ? (int8_t)state.currentTxVfo.load() : (int8_t)state.currentRxVfo.load();
            ENCODER_LOGD("Local tuning started (VFO %c, %s mode)", m_cachedTuneVfo == 1 ? 'B' : 'A',
                         isTxActive ? "TX" : "RX");
        }
    }
    // Condition 2: Send a final update if the knob has stopped turning.
    const int64_t finalDelayUs = FINAL_UPDATE_DELAY_US;
    ENCODER_LOGV("Final timing check: timeSinceLastMove=%lld, finalDelayUs=%lld", timeSinceLastMove, finalDelayUs);
    if (timeSinceLastMove > finalDelayUs)
    {
        shouldUpdate = true;
        ENCODER_LOGV("shouldUpdate=true (final delay)");
        if (m_isTuning)
        {
            m_isTuning = false; // Clear local tuning flag - final update
            // Notify RadioManager that local tuning has stopped
            m_radioManager->getState().isTuning.store(false);
            m_radioManager->getState().tuningStopTime.store(currentTime);
            m_cachedTuneVfo = -1;
            ENCODER_LOGD("Local tuning stopped");
        }
    }

    if (shouldUpdate)
    {
        if (!m_radioManager)
        {
            ESP_LOGE(TAG, "ERROR: m_radioManager is null! ABORTING update");
            return;
        }
        const auto &state = m_radioManager->getState();

        // Snapshot frequently-read radio state once to reduce redundant atomic loads
        const bool isTxActive = state.isTx.load();
        const uint8_t currentRxVfo = state.currentRxVfo.load();
        const uint8_t currentTxVfo = state.currentTxVfo.load();
        const uint64_t vfoAFreq = state.vfoAFrequency.load();
        const uint64_t vfoBFreq = state.vfoBFrequency.load();

        // Determine which VFO is currently active for tuning
        // Active VFO = TX VFO when transmitting, RX VFO when receiving
        const uint8_t fallbackVfo = isTxActive ? currentTxVfo : currentRxVfo;
        const uint8_t latchedVfo =
            (m_cachedTuneVfo >= 0 && m_cachedTuneVfo <= 1) ? static_cast<uint8_t>(m_cachedTuneVfo) : fallbackVfo;
        const bool tuneVfoB = (latchedVfo == 1);
        ENCODER_LOGV(
            "VFO selection: cachedTuneVfo=%d, currentRxVfo=%d, currentTxVfo=%d, isTx=%s, latchedVfo=%d, tuneVfoB=%s",
            static_cast<int>(m_cachedTuneVfo), static_cast<int>(currentRxVfo), static_cast<int>(currentTxVfo),
            isTxActive ? "true" : "false", static_cast<int>(latchedVfo), tuneVfoB ? "true" : "false");

        // If we don't have valid VFO state, skip this tick (no query here)
        if (latchedVfo > 1)
        {
            ESP_LOGE(TAG, "Invalid VFO state: latchedVfo=%d, SKIPPING update", (int)latchedVfo);
            m_lastUpdateTime = currentTime;
            return;
        }
        ENCODER_LOGV("VFO state valid, continuing...");

        const uint64_t currentFrequency = tuneVfoB ? vfoBFreq : vfoAFreq;
        ENCODER_LOGV("Current frequency: %llu Hz (VFO %s)", currentFrequency, tuneVfoB ? "B" : "A");

        [[maybe_unused]] const bool splitActive = (currentRxVfo != currentTxVfo);
        ENCODER_LOGD("Split: %s, RX VFO: %s, TX VFO: %s, Active: %s, Tuning VFO: %s", splitActive ? "ON" : "OFF",
                     currentRxVfo == 0 ? "A" : "B", currentTxVfo == 0 ? "A" : "B", isTxActive ? "TX" : "RX",
                     tuneVfoB ? "B" : "A");

        ENCODER_LOGV("Current base frequency: %llu", currentFrequency);

        // If frequency looks invalid, do not query here; skip this tick
        if (currentFrequency == 0 || currentFrequency < BASE_MIN_FREQUENCY_HZ)
        {
            ESP_LOGE(TAG, "Invalid frequency: %llu Hz (min=%llu), SKIPPING update", currentFrequency,
                     BASE_MIN_FREQUENCY_HZ);
            m_lastUpdateTime = currentTime;
            return;
        }
        ENCODER_LOGV("Frequency valid, continuing...");

        // Convert to display frequency if transverter mode is enabled
        const uint64_t workingFrequency = m_radioManager->baseToDisplayFrequency(currentFrequency);

        ENCODER_LOGV("Working frequency: %llu", workingFrequency);

        // TS-590: fixed step by mode + [FINE], with optional discrete acceleration bands
        const bool fine = state.fineTune;
        const Mode mode = static_cast<Mode>(state.mode.load());
        const int32_t radioStepHz = pickKenwoodStepHz(fine, mode); // hardware step size (no acceleration)
        int32_t cappedStepSize = radioStepHz; // default TS-590
        ENCODER_LOGV("Step calculation: fine=%s, mode=%d, baseStepSize=%ld Hz", fine ? "true" : "false",
                     static_cast<int>(mode), (long)cappedStepSize);
#if CONFIG_ENCODER_ACCEL_ENABLE
        // Calculate windowed speed to choose band
        if (m_lastSampleTimeUs == 0)
            m_lastSampleTimeUs = currentTime - liveGateUs;
        const int64_t dt = currentTime - m_lastSampleTimeUs;
        const float logicalPerSec = (dt > 0) ? (std::abs(delta) * (1000000.0f / (float)dt)) : 0.0f;
        const float rps = (ppr > 0) ? (logicalPerSec / (float)ppr) : 0.0f;
        updateAccelBandDiscrete(rps, currentTime);
        cappedStepSize = selectStepDiscrete(fine, mode);
        m_lastSampleTimeUs = currentTime;
#else
        (void)timeSinceLastMove;
#endif

        const int64_t wholeHz = static_cast<int64_t>(delta) * cappedStepSize;
        ENCODER_LOGV("Frequency change calc: delta=%ld * stepSize=%ld = wholeHz=%lld", static_cast<long>(delta),
                     static_cast<long>(cappedStepSize), wholeHz);

        const int64_t frequencyChange = wholeHz;

        // Prevent underflow when subtracting from frequency
        uint64_t requestedFrequency;
        if (frequencyChange < 0 && static_cast<uint64_t>(-frequencyChange) > workingFrequency)
        {
            // Would underflow - clamp to minimum frequency
            requestedFrequency = BASE_MIN_FREQUENCY_HZ;
            ESP_LOGW(TAG, "Frequency underflow prevented: %llu + %lld clamped to %llu", workingFrequency,
                     frequencyChange, requestedFrequency);
        }
        else
        {
            requestedFrequency = workingFrequency + frequencyChange;
        }

        // Snap to current step grid for clean digit behavior
        uint64_t newFrequency = snapToStepGrid(requestedFrequency, cappedStepSize);

        ENCODER_LOGV("Frequency calculation: working=%llu + change=%lld = requested=%llu, final=%llu", workingFrequency,
                     frequencyChange, requestedFrequency, newFrequency);

        // If using transverter display, bound the display frequency so that the mapped base stays within limits
        if (state.transverter && state.transverterOffsetHz > 0)
        {
            const uint64_t off = state.transverterOffsetHz;
            const bool plus = state.transverterOffsetPlus;
            if (off > 0)
            {
                uint64_t minDisp = 0;
                uint64_t maxDisp = UINT64_MAX;
                if (plus)
                {
                    // base = display - off => display >= off + BASE_MIN, <= off + BASE_MAX
                    minDisp = off + BASE_MIN_FREQUENCY_HZ;
                    maxDisp = off + BASE_MAX_FREQUENCY_HZ;
                }
                else
                {
                    // base = display + off => display >= max(BASE_MIN - off, 0), <= BASE_MAX - off (if off <= BASE_MAX)
                    minDisp = (BASE_MIN_FREQUENCY_HZ > off) ? (BASE_MIN_FREQUENCY_HZ - off) : 0ULL;
                    maxDisp = (BASE_MAX_FREQUENCY_HZ > off) ? (BASE_MAX_FREQUENCY_HZ - off) : 0ULL;
                }

                if (newFrequency < minDisp)
                {
                    ENCODER_LOGV("Encoder: clamping display freq up %llu -> %llu (XO mapping)", newFrequency, minDisp);
                    newFrequency = minDisp;
                }
                else if (maxDisp != 0ULL && newFrequency > maxDisp)
                {
                    ENCODER_LOGV("Encoder: clamping display freq down %llu -> %llu (XO mapping)", newFrequency,
                                 maxDisp);
                    newFrequency = maxDisp;
                }
            }
        }

        // Validate and clamp frequency to safe (display) limits
        if (!isValidFrequency(newFrequency, currentFrequency))
        {
            const uint64_t oldFreq = newFrequency;
            newFrequency = clampToValidFrequency(newFrequency, currentFrequency);
            ESP_LOGW(TAG, "Encoder: Clamped frequency %llu -> %llu Hz", oldFreq, newFrequency);
        }

        ENCODER_LOGD("TS-590: delta=%ld, step=%ld Hz, freq=%llu Hz", (long)delta, (long)cappedStepSize, newFrequency);

        ENCODER_LOGV("New frequency: %llu", newFrequency);

        // For non-transverter mode, validate base frequency range
        uint64_t frequencyToSend = newFrequency;
        if (!(state.transverter && state.transverterOffsetHz > 0))
        {
            // Only validate base frequency limits when not using transverter
            if (frequencyToSend < BASE_MIN_FREQUENCY_HZ || frequencyToSend > BASE_MAX_FREQUENCY_HZ)
            {
                ESP_LOGW(TAG, "Encoder: base frequency out of range (%llu). Skipping update.", frequencyToSend);
                m_lastUpdateTime = currentTime;
                return;
            }
        }

        // Send CAT command to set frequency on the appropriate VFO
        if (m_radioManager)
        {
            // Always send base-space frequency to radio - convert from display if needed
            uint64_t freqToSend = m_radioManager->displayToBaseFrequency(frequencyToSend);

#if CONFIG_ENCODER_RADIO_USE_RELATIVE_CMDS
            const uint64_t previousBaseFreq = tuneVfoB ? vfoBFreq : vfoAFreq;
            const bool allowRelative = (!tuneVfoB) && !state.transverter;
            bool usedRelativeCommand = false;
            int64_t totalStepsSent = 0; // Track signed steps sent for cache update
            const int64_t freqDiffHz = static_cast<int64_t>(freqToSend) - static_cast<int64_t>(previousBaseFreq);
            if (allowRelative && delta != 0)
            {
                if (radioStepHz > 0)
                {
                    const int64_t stepsNeeded = freqDiffHz / radioStepHz;
                    const int64_t remainderHz = freqDiffHz % radioStepHz;
                    if (stepsNeeded != 0 && remainderHz == 0)
                    {
                        int64_t stepsRemaining = std::llabs(stepsNeeded);
                        const int64_t stepSign = (stepsNeeded > 0) ? 1 : -1;
                        const char *prefix = (stepsNeeded > 0) ? "UP" : "DN";
                        while (stepsRemaining > 0)
                        {
                            const int32_t chunk = static_cast<int32_t>(std::min<int64_t>(stepsRemaining, 99));
                            char stepCmd[8];
                            snprintf(stepCmd, sizeof(stepCmd), "%s%02d;", prefix, chunk);
                            ENCODER_LOGD("Encoder: sending incremental CAT command %s", stepCmd);
                            m_radioManager->dispatchMessage(m_radioManager->getPanelCATHandler(), stepCmd);
                            stepsRemaining -= chunk;
                            totalStepsSent += stepSign * static_cast<int64_t>(chunk);
                        }
                        if (totalStepsSent != 0)
                        {
                            usedRelativeCommand = true;
                            ENCODER_LOGD("Encoder: used relative commands for %lld radio steps (step=%ld Hz)",
                                         static_cast<long long>(std::llabs(totalStepsSent)),
                                         static_cast<long>(radioStepHz));
                        }
                    }
                    else if (stepsNeeded == 0)
                    {
                        ENCODER_LOGD("Encoder: skipping relative update, computed step count is zero");
                    }
                    else
                    {
                        ENCODER_LOGD("Encoder: relative delta %lld Hz not aligned to %ld Hz step, using absolute",
                                     static_cast<long long>(freqDiffHz), static_cast<long>(radioStepHz));
                    }
                }
                else
                {
                    ENCODER_LOGD("Encoder: invalid radio step (%ld), falling back to absolute",
                                 static_cast<long>(radioStepHz));
                }
            }
            else if (allowRelative && delta == 0)
            {
                ENCODER_LOGD("Encoder: skipping update, delta is zero");
            }

            if (!usedRelativeCommand)
            {
                static thread_local FrequencyFormatter formatter;
                const char* cmd = tuneVfoB ? formatter.formatFB(freqToSend) : formatter.formatFA(freqToSend);
                ENCODER_LOGD("Encoder: sending CAT command %s (freq=%llu Hz)", cmd, freqToSend);
                m_radioManager->dispatchMessage(m_radioManager->getPanelCATHandler(), cmd);
            }
#else
            static thread_local FrequencyFormatter formatter;
            const char* cmd = tuneVfoB ? formatter.formatFB(freqToSend) : formatter.formatFA(freqToSend);
            ENCODER_LOGD("Encoder: sending CAT command %s (freq=%llu Hz)", cmd, freqToSend);
            m_radioManager->dispatchMessage(m_radioManager->getPanelCATHandler(), cmd);
#endif

            // Update local state immediately
            // For relative commands: send UP/DN to radio (efficient) but update cache/display
            // with calculated absolute frequency (accurate). The grace period prevents
            // stale FA answers from overwriting our calculated value.
            // For absolute commands: use the exact frequency we commanded
            uint64_t actualFreqToUpdate = freqToSend;
#if CONFIG_ENCODER_RADIO_USE_RELATIVE_CMDS
            if (usedRelativeCommand)
            {
                const int64_t appliedHz = totalStepsSent * static_cast<int64_t>(radioStepHz);
                actualFreqToUpdate = static_cast<uint64_t>(static_cast<int64_t>(previousBaseFreq) + appliedHz);
                ENCODER_LOGD("Encoder: relative cmd applied %lld Hz (step=%ld), cache freq=%llu",
                             static_cast<long long>(appliedHz), static_cast<long>(radioStepHz), actualFreqToUpdate);
            }
#endif
            if (tuneVfoB)
            {
                m_radioManager->updateVfoBFrequency(actualFreqToUpdate);
            }
            else
            {
                m_radioManager->updateVfoAFrequency(actualFreqToUpdate);
            }

            // Route frequency updates to AI-enabled interfaces (TS-590SG AI behavior)
            // If transverter display mix is enabled, report mixed (display) to USB; else report base
            uint64_t ifaceFreq =
                actualFreqToUpdate; // Use actual frequency (steps-based for relative, exact for absolute)
            // Apply transverter offset for display/USB if enabled via UIXD1
            if (m_radioManager->isTransverterOffsetActive())
            {
                ifaceFreq = m_radioManager->baseToDisplayFrequency(actualFreqToUpdate); // Convert base to display
            }

            // Format frequency command using shared formatter (declared earlier in the function)
            const char* ifaceCmd = tuneVfoB ? formatter.formatFB(ifaceFreq) : formatter.formatFA(ifaceFreq);

            // Always send to display
            m_radioManager->sendToDisplay(ifaceCmd);
            radio::DisplayLatencyProfiler::instance().markEncoderDirectSend();
            // Route to AI-enabled USB interfaces using ForwardingPolicy
            const std::string_view response(ifaceCmd);

            // Check and route to USB CDC0 if AI mode is enabled
            if (radio::ForwardingPolicy::shouldForwardToUsbCdc0(response, state, static_cast<uint64_t>(currentTime)))
            {
                ENCODER_LOGD("Encoder: routing frequency update to USB0: %s", ifaceCmd);
                m_radioManager->sendToSource(radio::CommandSource::UsbCdc0, response);
            }

            // Check and route to USB CDC1 if AI mode is enabled
            if (radio::ForwardingPolicy::shouldForwardToUsbCdc1(response, state, static_cast<uint64_t>(currentTime)))
            {
                ENCODER_LOGD("Encoder: routing frequency update to USB1: %s", ifaceCmd);
                m_radioManager->sendToSource(radio::CommandSource::UsbCdc1, response);
            }
        }

        // Update last reported position for GPIO-ISR mode
        if (!m_usePcnt)
        {
            // currentPosition is valid only in ISR mode
            // m_lastReportedPosition was advanced earlier in ISR mode input path
        }
        m_lastUpdateTime = currentTime;
    }
}


bool EncoderHandler::isValidFrequency(const uint64_t frequency, uint64_t currentVfoBaseFreq) const
{
    // Determine display-space valid range considering transverter offset and Kconfig
    uint64_t minHz = BASE_MIN_FREQUENCY_HZ;
    uint64_t maxHz = BASE_MAX_FREQUENCY_HZ;

#ifdef CONFIG_RADIOCORE_EMULATE_TS2000
    // TS-2000 typical 2m band limits for tuning range when emulating
    minHz = 30000;
    maxHz = 440000000ULL;
#endif

    const auto &state = m_radioManager->getState();
    if (state.transverter)
    {
        const uint64_t off = state.transverterOffsetHz;
        const bool plus = state.transverterOffsetPlus;
        if (off > 0)
        {
            // Auto-derive range from current display frequency using the active VFO
            uint64_t currentBase = currentVfoBaseFreq;
            if (currentBase > 0)
            {
                uint64_t currentDisplayFreq =
                    plus ? (currentBase + off) : (currentBase > off ? currentBase - off : 0ULL);
                if (currentDisplayFreq > 0)
                {
                    // Use current display frequency as center, with ±2 MHz range
                    const uint64_t rangeHz = 2000000; // 2 MHz
                    minHz = (currentDisplayFreq > rangeHz) ? (currentDisplayFreq - rangeHz) : 1000000;
                    maxHz = currentDisplayFreq + rangeHz;
                }
            }
        }
    }
    return frequency >= minHz && frequency <= maxHz;
}

uint64_t EncoderHandler::clampToValidFrequency(const uint64_t frequency, uint64_t currentVfoBaseFreq) const
{
    uint64_t minHz = BASE_MIN_FREQUENCY_HZ;
    uint64_t maxHz = BASE_MAX_FREQUENCY_HZ;

#ifdef CONFIG_RADIOCORE_EMULATE_TS2000
    minHz = 30000;
    maxHz = 440000000ULL;
#endif

    const auto &state = m_radioManager->getState();
    if (state.transverter)
    {
        const uint64_t off = state.transverterOffsetHz;
        const bool plus = state.transverterOffsetPlus;
        if (off > 0)
        {
            // Auto-derive range from current display frequency using the active VFO
            uint64_t currentBase = currentVfoBaseFreq;
            if (currentBase > 0)
            {
                uint64_t currentDisplayFreq =
                    plus ? (currentBase + off) : (currentBase > off ? currentBase - off : 0ULL);
                if (currentDisplayFreq > 0)
                {
                    // Use current display frequency as center, with ±2 MHz range
                    const uint64_t rangeHz = 2000000; // 2 MHz
                    minHz = (currentDisplayFreq > rangeHz) ? (currentDisplayFreq - rangeHz) : 1000000;
                    maxHz = currentDisplayFreq + rangeHz;
                }
            }
        }
    }

    if (frequency < minHz)
        return minHz;
    if (frequency > maxHz)
        return maxHz;
    return frequency;
}

// NOTE: Transverter offset helpers have been moved to RadioManager as the
// centralized, authoritative implementation. Use m_radioManager->baseToDisplayFrequency()
// and m_radioManager->displayToBaseFrequency() instead.

void EncoderHandler::setTs590PulsesPerRev(int ppr)
{
    if (ppr != 250 && ppr != 500 && ppr != 1000)
        ppr = 1000;
    m_pulsesPerRev.store(ppr);
    const int edgeDiv = std::max(1, RAW_EDGES_PER_REV / ppr);
    ESP_LOGI(TAG, "TS-590SG Menu 13: %d pulses/rev (edge divisor: %d, %s mode)", ppr, edgeDiv,
             m_usePcnt ? "x4 PCNT" : "x2 GPIO");
}

int EncoderHandler::pickKenwoodStepHz(bool fine, Mode mode) const
{
    // TS-590SG authentic step sizes per manual:
    // SSB/CW/FSK: 10 Hz normal, 1 Hz with [FINE]
    // AM/FM: 100 Hz normal, 10 Hz with [FINE]
    const bool isAM = (mode == Mode::AM);
    const bool isFM = (mode == Mode::FM);
    if (fine)
    {
        return (isAM || isFM) ? 10 : 1; // [FINE] mode: AM/FM=10Hz, others=1Hz
    }
    else
    {
        return (isAM || isFM) ? 100 : 10; // Normal mode: AM/FM=100Hz, others=10Hz
    }
}

int EncoderHandler::updateAccelBandDiscrete(float rps, int64_t /*nowUs*/)
{
    // Five-band thresholds: 0.3, 0.8, 1.5, 2.5 RPS
    static const float thresholds[4] = {0.30f, 0.80f, 1.50f, 2.50f};
    const float hyst = CONFIG_ENCODER_BAND_HYSTERESIS_PERCENT / 100.0f;

    uint8_t newBand = 0;
    // Determine new band based on speed
    for (int i = 3; i >= 0; --i)
    {
        if (rps >= thresholds[i])
        {
            newBand = i + 1;
            break;
        }
    }

    // Apply hysteresis for downshift to prevent oscillation
    if (newBand < m_accelBand)
    {
        const float downThreshold = thresholds[m_accelBand - 1] * (1.0f - hyst);
        if (rps > downThreshold)
        {
            newBand = m_accelBand; // Stay in current band
        }
    }

    m_accelBand = newBand;
    return m_accelBand;
}

int32_t EncoderHandler::selectStepDiscrete(bool fine, Mode mode) const
{
    const bool isAM = (mode == Mode::AM);
    const bool isFM = (mode == Mode::FM);
    if (fine)
        return (isAM || isFM) ? 10 : 1;

    if (isAM || isFM)
    {
        static const int amfmStep[5] = {100, 250, 500, 1000, 2500};
        return amfmStep[std::min<int>(m_accelBand, 4)];
    }
    else
    {
        static const int ssbStep[5] = {10, 25, 50, 100, 250};
        return ssbStep[std::min<int>(m_accelBand, 4)];
    }
}

uint64_t EncoderHandler::snapToStepGrid(uint64_t frequency, int32_t stepSize) const
{
    if (stepSize <= 1)
        return frequency;
    const uint64_t s = (uint64_t)stepSize;
    const uint64_t rem = frequency % s;
    const uint64_t half = s / 2;
    return (rem >= half) ? (frequency + (s - rem)) : (frequency - rem);
}

// ISR Handler - always compiled for fallback mode
void IRAM_ATTR EncoderHandler::isrHandler(void *arg)
{
    auto *encoder = static_cast<EncoderHandler *>(arg);
    const int64_t currentTime = esp_timer_get_time();

// Optional software glitch filter for GPIO mode
#if CONFIG_ENCODER_GPIO_SW_FILTER_US > 0
    gpio_num_t triggerPin = GPIO_NUM_MAX;
    // Determine which pin triggered the interrupt
    const uint8_t pinA_level = gpio_get_level(encoder->m_pinA);
    const uint8_t pinB_level = gpio_get_level(encoder->m_pinB);
    const uint8_t new_state = (pinA_level << 1) | pinB_level;
    const uint8_t old_state = encoder->m_state;

    // Check if Pin A changed
    if ((old_state ^ new_state) & 0x02)
        triggerPin = encoder->m_pinA;
    // Check if Pin B changed
    else if ((old_state ^ new_state) & 0x01)
        triggerPin = encoder->m_pinB;

    // Apply software debounce filter
    if (triggerPin != GPIO_NUM_MAX)
    {
        std::atomic<int64_t> *lastToggleTime =
            (triggerPin == encoder->m_pinA) ? &encoder->m_lastGpioToggleTimeA : &encoder->m_lastGpioToggleTimeB;

        const int64_t lastTime = lastToggleTime->load();
        if (currentTime - lastTime < CONFIG_ENCODER_GPIO_SW_FILTER_US)
        {
            return; // Filter out glitch
        }
        lastToggleTime->store(currentTime);
    }
#else
    const uint8_t new_state = (gpio_get_level(encoder->m_pinA) << 1) | gpio_get_level(encoder->m_pinB);
#endif

    const uint8_t transition = (encoder->m_state << 2) | new_state;

    if (const int8_t direction = enc_states[transition]; direction != 0)
    {
        // Simple counter for debugging (checked from task)
        encoder->m_wakeIsrCount.fetch_add(1, std::memory_order_relaxed);
        portENTER_CRITICAL_ISR(&encoder->m_spinlock);
        encoder->m_position += direction;
        encoder->m_state = new_state; // Move state update inside critical section
        portEXIT_CRITICAL_ISR(&encoder->m_spinlock);
        encoder->m_lastMovementTime.store(currentTime);

        // Record encoder activity for burst polling (ISR-safe atomic operation)
        encoder->m_radioManager->recordEncoderActivity();

        // Signal the encoder task that movement occurred (ISR-safe)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(encoder->m_movementSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        encoder->m_state = new_state; // Update state even if no direction change
    }
}

void IRAM_ATTR EncoderHandler::wakeIsrHandler(void *arg)
{
    auto *encoder = static_cast<EncoderHandler *>(arg);
    const int64_t currentTime = esp_timer_get_time();
    // Just record last movement time and wake task; PCNT counts are read in task()
    encoder->m_lastMovementTime.store(currentTime);
    encoder->m_wakeIsrCount.fetch_add(1, std::memory_order_relaxed);

    // Record encoder activity for panel->display wake signaling (UIPS)
    encoder->m_radioManager->recordEncoderActivity();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(encoder->m_movementSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
