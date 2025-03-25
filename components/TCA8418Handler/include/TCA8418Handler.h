#pragma once

#include <functional>
#include <map>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

class TCA8418Handler {
public:
    enum class MatrixKey : uint8_t {
        // TCA8418 key codes: key = (col * 5) + row + 1 (5-row mode)
        // Physical layout verified against ARCI-SMT PCB

        // Left PCB Matrix - verified by PCB testing
        // Note: Key codes don't follow simple row/col pattern due to PCB wiring
        KEY_0x01 = 0x01,   // SEND/MOX - bottom right of left PCB
        KEY_0x02 = 0x02,   // VOX
        KEY_0x03 = 0x03,   // TUNE
        KEY_0x04 = 0x04,   // PRE (preamp)
        KEY_0x05 = 0x05,   // PWR (power output) - bottom left of left PCB
        KEY_0x06 = 0x06,   // (unused or alternate TUNE)
        KEY_0x07 = 0x07,   // XVTR (transverter)
        KEY_0x08 = 0x08,   // PROC (speech processor)
        KEY_0x09 = 0x09,   // ATT (attenuator)

        // Keys used on TCA8418 #1 (main left PCB column 2) and #2 (F-buttons)
        KEY_0x0B = 0x0B,   // TCA#2: F2 macro
        KEY_0x0C = 0x0C,   // TCA#1: POWER on/off, TCA#2: F4 macro
        KEY_0x0D = 0x0D,   // TCA#1: XVTR, TCA#2: F6 macro
        KEY_0x0E = 0x0E,   // TCA#1: PROC (speech processor)
        KEY_0x0F = 0x0F,   // TCA#1: ATT (attenuator)

        // Right PCB Matrix (C5,C7,C9 × R0-R3) - 12 buttons
        // Note: Physical columns skip C6, C8
        KEY_0x1A = 0x1A,   // C5,R0 - NTCH (notch)
        KEY_0x1B = 0x1B,   // C5,R1 - BND+ (band up)
        KEY_0x1C = 0x1C,   // C5,R2 - BND- (band down)
        KEY_0x1D = 0x1D,   // C5,R3 - CLR (clear)
        KEY_0x24 = 0x24,   // C7,R0 - NR (noise reduction)
        KEY_0x25 = 0x25,   // C7,R1 - RIT
        KEY_0x26 = 0x26,   // C7,R2 - A/B (VFO swap)
        KEY_0x27 = 0x27,   // C7,R3 - MODE
        KEY_0x2E = 0x2E,   // C9,R0 - NB (noise blanker)
        KEY_0x2F = 0x2F,   // C9,R1 - XIT
        KEY_0x30 = 0x30,   // C9,R2 - SPLIT
        KEY_0x31 = 0x31,   // C9,R3 - ENT (enter)
    };

    struct KeyEvent {
        MatrixKey key;
        bool isPressed;  // true for press, false for release
    };

    using KeyCallback = std::function<void(MatrixKey key, bool isPressed)>;

    /**
     * @brief Construct TCA8418 handler with matrix configuration
     * @param interruptPin GPIO pin for interrupt (GPIO_NUM_NC for polling)
     * @param numRows Number of rows in the matrix (1-8, default 5)
     * @param numCols Number of columns in the matrix (1-10, default 10)
     */
    explicit TCA8418Handler(gpio_num_t interruptPin = GPIO_NUM_NC,
                           uint8_t numRows = 5,
                           uint8_t numCols = 10);
    ~TCA8418Handler();

    bool initialize(i2c_master_bus_handle_t i2cBusHandle);

    /**
     * @brief Set TCA9548 mux channel for this TCA8418 instance
     * @param muxHandler Pointer to TCA9548Handler (nullptr if no mux)
     * @param channel Channel number (0-7)
     */
    void setMuxChannel(class TCA9548Handler* muxHandler, uint8_t channel);

    void handleKeyEvents();
    void setKeyCallback(MatrixKey key, KeyCallback callback);

    // Reset functionality
    bool resetDevice();
    bool checkDeviceHealth();
    bool recoverFromStuckState(); // Recovery method for stuck device state

private:
    // TCA8418 I2C Keypad Controller Register definitions
    static constexpr uint8_t TCA8418_I2C_ADDR = 0x34;
    static constexpr uint8_t REG_CFG = 0x01;
    static constexpr uint8_t REG_INT_STAT = 0x02;
    static constexpr uint8_t REG_KEY_LCK_EC = 0x03;
    static constexpr uint8_t REG_KEY_EVENT_A = 0x04;
    static constexpr uint8_t REG_GPIO_INT_STAT1 = 0x11;
    static constexpr uint8_t REG_GPIO_INT_STAT2 = 0x12;
    static constexpr uint8_t REG_GPIO_INT_STAT3 = 0x13;
    static constexpr uint8_t REG_GPIO_DAT_STAT1 = 0x14;
    static constexpr uint8_t REG_GPIO_DAT_STAT2 = 0x15;
    static constexpr uint8_t REG_GPIO_DAT_STAT3 = 0x16;
    static constexpr uint8_t REG_GPIO_DAT_OUT1 = 0x17;
    static constexpr uint8_t REG_GPIO_DAT_OUT2 = 0x18;
    static constexpr uint8_t REG_GPIO_DAT_OUT3 = 0x19;
    static constexpr uint8_t REG_GPIO_INT_EN1 = 0x1A;
    static constexpr uint8_t REG_GPIO_INT_EN2 = 0x1B;
    static constexpr uint8_t REG_GPIO_INT_EN3 = 0x1C;
    static constexpr uint8_t REG_KP_GPIO1 = 0x1D;
    static constexpr uint8_t REG_KP_GPIO2 = 0x1E;
    static constexpr uint8_t REG_KP_GPIO3 = 0x1F;
    static constexpr uint8_t REG_GPI_EM1 = 0x20;
    static constexpr uint8_t REG_GPI_EM2 = 0x21;
    static constexpr uint8_t REG_GPI_EM3 = 0x22;

    // TCA8418 Configuration register bit definitions
    static constexpr uint8_t CFG_AI = 0x80;        // Auto-increment
    static constexpr uint8_t CFG_GPI_E_CFG = 0x40; // GPI event mode
    static constexpr uint8_t CFG_OVR_FLOW_M = 0x20; // Overflow mode
    static constexpr uint8_t CFG_INT_CFG = 0x10;   // Interrupt config
    static constexpr uint8_t CFG_OVR_FLOW_IEN = 0x08; // Overflow interrupt enable
    static constexpr uint8_t CFG_K_LCK_IEN = 0x04; // Key lock interrupt enable
    static constexpr uint8_t CFG_GPI_IEN = 0x02;   // GPI interrupt enable
    static constexpr uint8_t CFG_KE_IEN = 0x01;    // Key event interrupt enable

    // TCA8418 Interrupt Status register (REG_INT_STAT) bit definitions
    static constexpr uint8_t INT_STAT_K_INT = 0x01;        // Key Event Interrupt
    static constexpr uint8_t INT_STAT_GPI_INT = 0x02;      // GPI Interrupt
    static constexpr uint8_t INT_STAT_K_LCK_INT = 0x04;    // Key Lock Interrupt
    static constexpr uint8_t INT_STAT_OVR_FLOW_INT = 0x08; // Overflow Interrupt
    static constexpr uint8_t INT_STAT_CAD_INT = 0x10;      // CAD Interrupt
    static constexpr uint8_t INT_STAT_PWR_ON_INT = 0x20;   // Power-On Reset Interrupt
    
    i2c_master_bus_handle_t m_i2cBusHandle;
    i2c_master_dev_handle_t m_i2cDevHandle;
    gpio_num_t m_interruptPin;
    SemaphoreHandle_t m_keySemaphore;
    TaskHandle_t m_keyTaskHandle;
    std::map<MatrixKey, KeyCallback> m_keyCallbacks;
    bool m_initialized;
    bool m_usePolling;

    // TCA9548 mux support
    class TCA9548Handler* m_muxHandler;
    uint8_t m_muxChannel;
    uint32_t m_lastHealthCheck;
    uint32_t m_consecutiveFailures;

    // Matrix configuration (set at construction, immutable)
    const uint8_t m_numRows;
    const uint8_t m_numCols;
    
    // Improved event processing to prevent FIFO overflow
    uint32_t m_lastKeyEventTime;
    uint32_t m_lastIntTime;  // Track INT pin timing for watchdog
    static constexpr uint32_t MIN_KEY_INTERVAL_MS = 10; // Minimum 10ms between key events
    static constexpr uint32_t MAX_EVENT_BATCH_SIZE = 32; // Maximum events to process in one batch
    static constexpr uint32_t KEY_TASK_STACK_WORDS = 6144; // ~24 KB stack to accommodate logging/path handling
    static constexpr uint32_t INT_STUCK_TIMEOUT_MS = 50; // Max time INT can stay low without events
    
    // Event batch buffer for drain-to-empty processing
    struct EventBatch {
        uint8_t events[MAX_EVENT_BATCH_SIZE];
        uint8_t count;
        bool hasOverflow;
        bool hasKeyLock;
    };

    bool testI2CCommunication();
    void scanI2CBus() const;
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    bool readMultipleRegisters(uint8_t startReg, uint8_t* buffer, uint8_t count); // Optimized batch I2C
    bool configureKeypadEngine();
    esp_err_t selectMuxChannel(); // Select TCA9548 channel if configured
    void clearPendingInterruptsOnTCA(); // Helper to clear INT_STAT and FIFO
    MatrixKey decodeKeyEvent(uint8_t eventByte);
    
    // Improved batch processing methods
    EventBatch drainEventsToEmpty();
    void processBatchedEvents(const EventBatch& batch);
    bool handleOverflowRecovery();
    bool handleKeyLockRecovery();
    bool checkStuckInterruptPin();
    
    static void IRAM_ATTR keyInterruptHandler(void* arg);
    static void keyTask(void* param);
    void processKeyEvent(uint8_t eventByte);

    // Helper functions for key event decoding
    static inline bool isKeyPress(uint8_t eventByte) { return (eventByte & 0x80) == 0; }
    static inline MatrixKey extractKey(uint8_t eventByte) { return static_cast<MatrixKey>(eventByte & 0x7F); }
    void getRowColumn(MatrixKey key, uint8_t& row, uint8_t& column);
};
