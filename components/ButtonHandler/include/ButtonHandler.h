#pragma once

#include <array>
#include <atomic>
#include <map>
#include "MatrixButton.h"
#include "RadioManager.h"
#include "TCA8418Handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Forward declarations
class TCA8418Handler;

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
 * - Power state control via the matrix POWER key
 */
class ButtonHandler {
#ifdef CONFIG_RUN_UNIT_TESTS
    friend class TestButtonHandler;
#endif

public:
    explicit ButtonHandler(radio::RadioManager *radioManager, radio::RadioMacroManager *macroManager);

    ~ButtonHandler();

    void start();
    void stop();
    void updateButtonStates();

    // Public trigger methods used by TCA8418 key mappings
    void trigger_A_equals_B_button();
    void triggerFunctionButton3();
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

    // Template button accessors removed - using TCA8418 matrix buttons only
    // Test helpers available via TestButtonHandler friend class if needed

private:
    radio::RadioManager &m_radioManager;
    radio::RadioMacroManager &m_macroManager;
    TaskHandle_t m_taskHandle;
    SemaphoreHandle_t m_stopSemaphore{nullptr};
    std::atomic<bool> m_running{false};

    static void buttonTask(void *arg);

    // All GPIO buttons removed - using TCA8418 matrix buttons only
    // Kept for potential future ADC-based or special purpose buttons if needed

    // Display backlight state tracking
    bool m_displayBacklightOn = true;

    // TCA8418 Matrix button management
    //
    // OWNERSHIP: every MatrixButton/Button object in these two maps is mutated
    // EXCLUSIVELY by buttonTask. The TCA8418 keypad callbacks run on the keypad
    // task and only enqueue events onto m_matrixEventQueue; buttonTask drains the
    // queue and performs all state mutation and action dispatch. Because there is
    // a single owning task, the plain (non-atomic) Button fields — including the
    // 64-bit m_pressedTime — are never read or written concurrently.
    std::map<TCA8418Handler::MatrixKey, MatrixButton> m_matrixButtons;      // TCA #1 (left/right PCB)
    std::map<TCA8418Handler::MatrixKey, MatrixButton> m_matrixButtons2;     // TCA #2 (F-buttons)

    // Internal event queue: keypad callbacks (keypad task) -> buttonTask.
    QueueHandle_t m_matrixEventQueue{nullptr};

    void initializeMatrixButtons();
    void initializeMatrixButtons2();  // F1-F6 buttons with long press support

    // Keypad-task entry points (registered from main.cpp). These now ONLY enqueue
    // an event; they must not touch any MatrixButton/Button state.
    void handleMatrixButtonEvent(TCA8418Handler::MatrixKey key, bool pressed);
    void handleMatrixButton2Event(TCA8418Handler::MatrixKey key, bool pressed);  // F-button handler

    // buttonTask-side event processing: buttonTask drains m_matrixEventQueue and
    // calls these to apply the mutations + action dispatch that previously ran
    // inline in the keypad callbacks.
    void applyMatrixButtonEvent(TCA8418Handler::MatrixKey key, bool pressed, int64_t pressTimeUs);
    void applyMatrixButton2Event(TCA8418Handler::MatrixKey key, bool pressed, int64_t pressTimeUs);

    // TCA8418 Matrix button handlers
    void handleBandUpButton(MatrixButton &button);
    void handleBandDownButton(MatrixButton &button);

    // Long-press companion to the BND+/BND- band change: steps through the
    // TS-590SG band stacking register slots (3 per band) without leaving the band.
    void cycleBandMemorySlot(bool up);
    void handleNotchButton(MatrixButton &button);
    void handleLockButton(MatrixButton &button);
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
    
    // Mode cycling helper methods
    template <size_t N>
    int8_t getNextModeInCycle(int8_t currentMode, const std::array<int8_t, N>& validModes) const;
    int8_t getDefaultModeForFrequency(uint64_t frequency) const;
};
