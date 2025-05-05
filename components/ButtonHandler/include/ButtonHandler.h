#pragma once

#include <atomic>
#include <map>
#include "MatrixButton.h"
#include "RadioManager.h"
#include "TCA8418Handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Forward declarations
class TCA8418Handler;
class NvsManager;

namespace radio {
    class RadioManager;
    class RadioMacroManager;
}

/**
 * @brief Button handler for TCA8418 I2C key matrix controller
 * 
 * This class manages button input via the TCA8418 I2C key matrix controller.
 * GPIO-based buttons have been removed in favor of the matrix approach.
 * 
 * Features:
 * - TCA8418 matrix button support with long/short press detection
 * - Integration with RadioManager for CAT command generation
 * - Support for complex button sequences via RadioMacroManager
 * - Power state control and split frequency functionality
 */
class ButtonHandler {
#ifdef CONFIG_RUN_UNIT_TESTS
    friend class TestButtonHandler;
#endif

public:
    explicit ButtonHandler(radio::RadioManager *radioManager, radio::RadioMacroManager *macroManager, NvsManager *nvsManager);

    ~ButtonHandler();

    void start();
    void stop();
    void updateButtonStates();

    // Public trigger methods used by TCA8418 key mappings
    void trigger_A_equals_B_button();
    void triggerFunctionButton1();
    void triggerFunctionButton2();
    void triggerFunctionButton3();
    void triggerFunctionButton4();
    void triggerFunctionButton5();
    void triggerFunctionButton6();
    void triggerTransverterMacroButton();
    void triggerBandUpButton();
    void triggerModeUpButton();
    void triggerModeDownButton();
    void triggerVfoToggleButton();
    void triggerTfSetButton();
    void triggerSplitButton();
    void triggerSpeechProcessorButton();

    // Setup TCA8418 key matrix mappings
    void setupTCA8418KeyMappings(TCA8418Handler *tca8418);
    void setupF1F6KeyMappings(TCA8418Handler *tca8418Handler2);

    // Diagnostic test mode - logs all button presses without executing functions
    void setupDiagnosticMode(TCA8418Handler *tca8418);

    // NVS persistence for mode memory
    void loadModeMemoryFromNvs();
    void saveModeMemoryToNvs() const;

    // Template button accessors removed - using TCA8418 matrix buttons only
    // Test helpers available via TestButtonHandler friend class if needed

private:
    radio::RadioManager &m_radioManager;
    radio::RadioMacroManager &m_macroManager;
    NvsManager &m_nvsManager;
    TaskHandle_t m_taskHandle;
    SemaphoreHandle_t m_stopSemaphore{nullptr};
    std::atomic<bool> m_running{false};

    static void buttonTask(void *arg);

    // All GPIO buttons removed - using TCA8418 matrix buttons only
    // Kept for potential future ADC-based or special purpose buttons if needed

    // Power state control
    void togglePowerState();
    
    // Split frequency functionality
    void setSplitFrequency(int splitValue);
    bool m_splitWaitingForNumeric = false;

    // Display backlight state tracking
    bool m_displayBacklightOn = true;

    // TCA8418 Matrix button management
    std::map<TCA8418Handler::MatrixKey, MatrixButton> m_matrixButtons;
    void initializeMatrixButtons();
    void handleMatrixButtonEvent(TCA8418Handler::MatrixKey key, bool pressed);

    // TCA8418 Matrix button handlers
    void handleBandUpButton();
    void handleBandDownButton();
    void handleNotchButton(MatrixButton &button);
    void handleLockButton();
    void handleVfoToggleButton();
    void handleNoiseReductionButton(MatrixButton &button);
    void handleRitButton(MatrixButton &button);
    void handleXitButton(MatrixButton &button);
    void handleNoiseBlankerButton(MatrixButton &button);
    void handleRfAttenuatorButton();
    void handlePreampButton();
    void handleModeMatrixButton(MatrixButton &button);
    void handleMoxButton(MatrixButton &button);
    void handleVoxButton(MatrixButton &button);
    void handleAntennaTunerButton(MatrixButton &button);
    void handleSpeechProcessorButton(MatrixButton &button);
    void handleLeftPcbButton8(MatrixButton &button);
    void handleAEqualsBMatrixButton(MatrixButton &button);
    void handlePowerButton(MatrixButton &button);
    void handleClearButton();
    void handleSplitButton();

    // Helper methods
    void fallbackToRadioAntennaSwitch();
    
    // Mode memory per band (11 bands: 0-10)
    // Default modes per band (0..10):
    // Low bands prefer LSB: 0=160m LSB, 1=80m LSB, 2=40m LSB
    // High bands prefer USB: 3=30m USB, 4=20m USB, 5=17m USB,
    // 6=15m USB, 7=12m USB, 8=10m USB, 9=6m USB, 10=GENE LSB
    // Note: Mode values per TS-590SG spec: 1=LSB, 2=USB, 5=AM
    std::array<int8_t, 11> m_bandModeMemory{1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1};
    void saveModeToMemory(int bandIndex, int8_t mode);
    int8_t getModeFromMemory(int bandIndex) const;
    
    // Mode cycling helper methods
    template <size_t N>
    int8_t getNextModeInCycle(int8_t currentMode, const std::array<int8_t, N>& validModes) const;
    int8_t getDefaultModeForFrequency(uint64_t frequency) const;
};
