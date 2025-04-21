#pragma once

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
// ReSharper disable once CppUnusedIncludeDirective
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h" // For portMUX_TYPE
#include "../../RadioCore/include/RadioManager.h" // Include RadioManager header
#include <atomic>

// Mode enumeration for type safety and readability
enum class Mode : uint8_t {
    LSB = 1,
    USB = 2, 
    CW  = 3,
    FM  = 4,
    AM  = 5,
    FSK = 6
};

class EncoderHandler {
public:
    EncoderHandler(gpio_num_t pinA, gpio_num_t pinB, radio::RadioManager *radioManager);
    ~EncoderHandler();

    void setup();
    void start();
    void stop();
    void task(bool movementDetected);
    
    // Check if encoder is currently being turned (for feedback loop prevention)
    bool isTuning() const { return m_isTuning; }
    
    // TS-590SG configuration methods
    void setTs590PulsesPerRev(int ppr);  // Menu 13: 250/500/1000 pulses per revolution
    int getTs590PulsesPerRev() const { return m_pulsesPerRev.load(); }

private:
    const gpio_num_t m_pinA;
    const gpio_num_t m_pinB;
    radio::RadioManager *m_radioManager; // Pointer to the RadioManager object
    TaskHandle_t m_taskHandle;
    SemaphoreHandle_t m_movementSemaphore; // Semaphore for event-driven processing
    SemaphoreHandle_t m_stopSemaphore{nullptr}; // Semaphore for clean shutdown synchronization
    std::atomic<bool> m_running{false};

    volatile int32_t m_position;
    std::atomic<int64_t> m_lastMovementTime;
    volatile uint8_t m_state;
    portMUX_TYPE m_spinlock{};

    int32_t m_lastReportedPosition;
    int64_t m_lastUpdateTime; // Tracks time of the last sent update for throttling
    bool m_isTuning; // Usb flag to track when encoder is actively being turned
    int8_t m_cachedTuneVfo; // VFO latched at start of turn (0=A,1=B,-1=unset)
    
    // TS-590SG emulation fields - RMS20-250-201 encoder
    // x4 quadrature decoding: 250 P/R × 4 edges (A rise/fall, B rise/fall) = 1000 total edges/rev
    static constexpr int RAW_EDGES_PER_REV = 1000;
    std::atomic<int> m_pulsesPerRev;  // Menu 13: 250/500/1000 logical pulses per rev
    int32_t m_edgeRemainder;          // Fractional edge accumulator for pulse conversion

    // Simplified acceleration (discrete bands with hysteresis)
    uint8_t m_accelBand;              // 0..2 discrete bands
    int64_t m_lastSampleTimeUs;       // Time of last window sample
    int32_t m_lastLogicalDelta;       // Logical pulses observed in last window
    int32_t m_stepSizeCache;          // Cached step size for current band/mode
    std::atomic<uint32_t> m_wakeIsrCount{0}; // ISR wake counter (PCNT mode)
    
    // GPIO software filtering (fallback mode only)
    std::atomic<int64_t> m_lastGpioToggleTimeA{0};
    std::atomic<int64_t> m_lastGpioToggleTimeB{0};
    
    // Edge counting validation (debug only)
    volatile uint32_t m_edgeCountForValidation{0};
    volatile int64_t m_lastValidationResetTime{0};
    
    // Frequency validation helpers
    bool isValidFrequency(uint64_t frequency, uint64_t currentVfoBaseFreq) const;
    uint64_t clampToValidFrequency(uint64_t frequency, uint64_t currentVfoBaseFreq) const;
    // NOTE: Transverter offset helpers moved to RadioManager (baseToDisplayFrequency/displayToBaseFrequency)

    // TS-590SG authentic step calculation
    int pickKenwoodStepHz(bool fine, Mode mode) const;
    
    // Simplified acceleration helpers
    int updateAccelBandDiscrete(float rps, int64_t nowUs);
    int32_t selectStepDiscrete(bool fine, Mode mode) const;
    uint64_t snapToStepGrid(uint64_t frequency, int32_t stepSize) const;

    static void IRAM_ATTR isrHandler(void *arg);
    static void encoderTask(void* arg);
    // Lightweight GPIO ISR used in PCNT mode to wake the task on any edge
    static void IRAM_ATTR wakeIsrHandler(void* arg);

    // PCNT (pulse counter) based quadrature decoding (optional, new driver)
    pcnt_unit_handle_t m_pcntUnit = nullptr;
    bool m_usePcnt = false;
    int m_lastPcntCount = 0;
};
