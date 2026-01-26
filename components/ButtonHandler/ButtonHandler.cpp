#include "ButtonHandler.h"

#include <ranges>

#include "../../include/pin_definitions.h"
#include "../RadioMacroManager/include/RadioMacroManager.h"
#include "FrequencyFormatter.h"
#include "NvsManager.h"
#include "RadioManager.h"
#include "ISerialChannel.h"
#include "TCA8418Handler.h"
#include "esp_log.h"
#include "esp_timer.h"

using radio::RadioManager;

static const char *TAG = "BUTTONHANDLER";
const char *bandNames[] = {"1.8MHz", "3.5MHz", "7MHz",  "10MHz", "14MHz", "18MHz",
                           "21MHz",  "24MHz",  "28MHz", "50MHz", "GENE"};

ButtonHandler::ButtonHandler(RadioManager *radioManager, radio::RadioMacroManager *macroManager, NvsManager *nvsManager) :
    m_radioManager(*radioManager), m_macroManager(*macroManager), m_nvsManager(*nvsManager), m_taskHandle(nullptr)
{
    // Create synchronization semaphore for clean task shutdown
    m_stopSemaphore = xSemaphoreCreateBinary();
    if (m_stopSemaphore == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create stop semaphore");
    }

    // Note: loadModeMemoryFromNvs() must be called explicitly after nvs_flash_init()
}

ButtonHandler::~ButtonHandler()
{
    stop();  // Ensure task is stopped before cleanup

    if (m_stopSemaphore != nullptr)
    {
        vSemaphoreDelete(m_stopSemaphore);
        m_stopSemaphore = nullptr;
    }
}

void ButtonHandler::start()
{
    if (m_taskHandle != nullptr)
    {
        ESP_LOGW(TAG, "ButtonHandler task already running");
        return;
    }

    m_running = true;
    xTaskCreate(buttonTask, "ButtonTask", 4096, this, 5, &m_taskHandle);
    ESP_LOGI(TAG, "ButtonHandler task started");
}

void ButtonHandler::stop()
{
    if (m_taskHandle == nullptr)
    {
        return;
    }

    // Signal the task to stop
    m_running.store(false, std::memory_order_release);

    // Wait for task to signal completion via semaphore (with timeout)
    if (m_stopSemaphore != nullptr)
    {
        constexpr TickType_t stopTimeoutTicks = pdMS_TO_TICKS(500);
        if (xSemaphoreTake(m_stopSemaphore, stopTimeoutTicks) != pdTRUE)
        {
            // Timeout - task didn't respond, force delete
            ESP_LOGW(TAG, "ButtonHandler task did not exit cleanly, forcing deletion");
            vTaskDelete(m_taskHandle);
        }
    }
    else
    {
        // No semaphore, fall back to delay
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    m_taskHandle = nullptr;
    ESP_LOGI(TAG, "ButtonHandler task stopped");
}

void ButtonHandler::buttonTask(void *arg)
{
    ButtonHandler *handler = static_cast<ButtonHandler *>(arg);

    while (handler->m_running.load(std::memory_order_acquire))
    {
        // Update matrix buttons for long press timing - no GPIO polling needed
        for (auto &button : handler->m_matrixButtons | std::views::values)
        {
            button.update(); // This handles long press timing
        }

        // Call handlers for currently pressed matrix buttons to check for long press events
        for (auto &[key, button] : handler->m_matrixButtons)
        {
            if (button.isPressed())
            {
                // Call the appropriate handler for pressed buttons to detect long press events
                switch (key)
                {
                // Left PCB buttons with long-press handlers
                case TCA8418Handler::MatrixKey::KEY_0x01:  // SEND (MOX)
                    handler->handleMoxButton(button);
                    break;
                case TCA8418Handler::MatrixKey::KEY_0x02:  // VOX
                    handler->handleVoxButton(button);
                    break;
                case TCA8418Handler::MatrixKey::KEY_0x03:  // TUNE (verified: physical TUNE→0x03)
                    handler->handleAntennaTunerButton(button);
                    break;
                case TCA8418Handler::MatrixKey::KEY_0x05:  // PWR output
                    handler->handleLeftPcbButton8(button);
                    break;
                case TCA8418Handler::MatrixKey::KEY_0x06:  // TUNE fallback
                    handler->handleAntennaTunerButton(button);
                    break;
                case TCA8418Handler::MatrixKey::KEY_0x08:  // PROC (speech processor)
                    handler->handleSpeechProcessorButton(button);
                    break;
                // Right PCB buttons with long-press handlers
                case TCA8418Handler::MatrixKey::KEY_0x26:  // A/B (VFO swap)
                    handler->handleAEqualsBMatrixButton(button);
                    break;
                case TCA8418Handler::MatrixKey::KEY_0x27:  // MODE
                    handler->handleModeMatrixButton(button);
                    break;
                default:
                    break;
                }
            }
        }

        // Handle any remaining GPIO-based buttons (if any)
        handler->updateButtonStates();

        // Check for UI mode timeout (auto-dismiss popups)
        handler->m_radioManager.checkUITimeout();

        vTaskDelay(pdMS_TO_TICKS(50)); // Reduced frequency since no GPIO polling
    }

    // Signal stop() that we're exiting cleanly
    if (handler->m_stopSemaphore != nullptr)
    {
        xSemaphoreGive(handler->m_stopSemaphore);
    }

    vTaskDelete(nullptr);
}


void ButtonHandler::updateButtonStates()
{
    // Update matrix buttons for timing (long press detection)
    for (auto &button : m_matrixButtons | std::views::values)
    {
        button.update();
    }

    // Update F-button matrix for long press timing
    for (auto &button : m_matrixButtons2 | std::views::values)
    {
        button.update();
    }
}

void ButtonHandler::togglePowerState()
{
    // Get current state before making any changes
    uint8_t currentState = m_radioManager.getOnOffState();
    bool keepAliveState = m_radioManager.getState().keepAlive.load();
    ESP_LOGI(TAG, "Power toggle requested - Current onOffState: %d, keepAlive: %d", currentState, keepAliveState);

    if (currentState == 0)
    {
        ESP_LOGI(TAG, "Turning interface ON - sending PS1 command");
        // Request power on via CAT
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "PS1;");
        ESP_LOGI(TAG, "PS1 command sent to LocalCATHandler");
    }
    else
    {
        ESP_LOGI(TAG, "Turning interface OFF - sending PS0 command");
        // Request power off via CAT
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "PS0;");
        ESP_LOGI(TAG, "PS0 command sent to LocalCATHandler");
    }

    // Note: State update happens asynchronously in PS command handler
    // LED will be updated automatically in the main loop
    vTaskDelay(pdMS_TO_TICKS(100));

    // Log the new state after processing
    uint8_t newState = m_radioManager.getOnOffState();
    bool newKeepAlive = m_radioManager.getState().keepAlive.load();
    ESP_LOGI(TAG, "Power toggle completed - New onOffState: %d, keepAlive: %d", newState, newKeepAlive);
}

void ButtonHandler::handlePowerButton(MatrixButton &button)
{
    // Get current state
    uint8_t currentState = m_radioManager.getOnOffState();
    bool keepAliveState = m_radioManager.getState().keepAlive.load();
    bool radioIsOff = (currentState == 0 || !keepAliveState);

    // Sync backlight tracking with radio power state
    // When radio is OFF, backlight should also be OFF
    if (radioIsOff && m_displayBacklightOn)
    {
        ESP_LOGD(TAG, "Syncing display backlight state: radio OFF -> backlight tracking set to OFF");
        m_displayBacklightOn = false;
    }

    // When radio is OFF, any press (short or long) turns it ON
    if (radioIsOff)
    {
        if (button.wasShortPressed() || button.wasLongPressed())
        {
            ESP_LOGI(TAG, "Power button pressed while OFF - turning interface ON (PS1)");
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "PS1;");
            m_displayBacklightOn = true; // RadioManager will send UIBL255 when power state changes
            ESP_LOGI(TAG, "PS1 command sent - interface turning on, backlight will restore");
        }
        return;
    }

    // Radio is ON - handle short vs long press
    if (button.wasLongPressed())
    {
        ESP_LOGI(TAG, "Power button LONG press - shutting down radio (PS0)");
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "PS0;");
        m_displayBacklightOn = false; // Display will be off when radio powers down
        ESP_LOGI(TAG, "PS0 command sent - radio powering off");
        return;
    }

    if (button.wasShortPressed())
    {
        // Interface is ON - toggle display backlight
        ISerialChannel *displaySerial = m_radioManager.getDisplaySerial();
        if (!displaySerial)
        {
            ESP_LOGW(TAG, "Display serial not available - cannot control backlight");
            return;
        }

        if (m_displayBacklightOn)
        {
            // Turn OFF display backlight
            ESP_LOGI(TAG, "Power button SHORT press - turning OFF display backlight (UIBL000)");
            displaySerial->sendMessage("UIBL000;");
            m_displayBacklightOn = false;
            ESP_LOGI(TAG, "UIBL000 sent - display backlight off, radio remains powered");
        }
        else
        {
            // Turn ON display backlight
            ESP_LOGI(TAG, "Power button SHORT press - turning ON display backlight (UIBL255)");
            displaySerial->sendMessage("UIBL255;");
            m_displayBacklightOn = true;
            ESP_LOGI(TAG, "UIBL255 sent - display backlight restored");
        }
    }
}

void ButtonHandler::setSplitFrequency(const int splitValue)
{
    if (!m_splitWaitingForNumeric)
    {
        return; // Not in split frequency entry mode
    }

    // Clear the waiting state since we're handling a number press
    m_splitWaitingForNumeric = false;

    // Get VFO A frequency from the radio
    const uint64_t vfoAFreq = m_radioManager.getVfoAFrequency();

    // Calculate new VFO B frequency by adding the splitValue kHz
    const uint64_t newVfoB = vfoAFreq + (splitValue * 1000); // Convert kHz to Hz

    // Send commands to set VFO B frequency using shared formatter
    static thread_local FrequencyFormatter formatter;
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), formatter.formatFB(newVfoB));

    // Small delay to let the command be processed
    vTaskDelay(pdMS_TO_TICKS(20));
}

// TCA8418 trigger methods - used by key mappings
void ButtonHandler::triggerTfSetButton()
{
    // This handles the TF Set button via TCA8418
    // Note: TF Set is a press/release type button, handled in the TCA8418 key mappings
    ESP_LOGI(TAG, "TF Set button triggered from TCA8418 matrix");
}

void ButtonHandler::triggerSplitButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Split button blocked - panel is LOCKED");
        return;
    }

    m_radioManager.recordButtonActivity();

    ESP_LOGI(TAG, "Split button triggered from TCA8418 matrix");
    // Toggle split via RadioManager helpers (keeps ButtonHandler CAT-agnostic)
    m_radioManager.toggleSplit(/*copyVfoBeforeEnable*/ false);
}

void ButtonHandler::triggerSpeechProcessorButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Speech Processor button blocked - panel is LOCKED");
        return;
    }

    m_radioManager.recordButtonActivity();

    ESP_LOGI(TAG, "Speech Processor button triggered - toggling DATA mode");
    // Simulate long press behavior: toggle data mode
    m_radioManager.toggleDataMode();

    bool newDataMode = m_radioManager.getState().dataMode.load();
    ESP_LOGI(TAG, "Speech Processor trigger: DATA mode -> %s", newDataMode ? "ON" : "OFF");

    // Also simulate short press behavior: toggle Speech Processor (PR)
    bool currentPR = m_radioManager.getState().processor;
    int newPR = currentPR ? 0 : 1; // Toggle between 0 (OFF) and 1 (ON)

    char prCommand[8];
    std::snprintf(prCommand, sizeof(prCommand), "PR%d;", newPR);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), prCommand);

    ESP_LOGI(TAG, "Speech Processor trigger: PR %s -> %s", currentPR ? "ON" : "OFF", newPR ? "ON" : "OFF");
}

void ButtonHandler::trigger_A_equals_B_button()
{
    if (!m_radioManager.getState().keepAlive.load())
    {
        ESP_LOGW(TAG, "Interface is off, ignoring A=B button");
        return;
    }
    m_radioManager.recordButtonActivity();

    ESP_LOGI(TAG, "A=B button pressed - copying current RX VFO to other VFO");
    std::string vvCommand = "VV;";
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), vvCommand);
    ESP_LOGI(TAG, "Sent VV command for VFO copy");
}

// Function button triggers - used by TCA8418 key mappings
void ButtonHandler::triggerFunctionButton1()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Function Button 1 blocked - panel is LOCKED");
        return;
    }

    m_radioManager.recordButtonActivity();

    // Toggle AGC - query current state first, then toggle
    int currentAgc = m_radioManager.getState().agcMode;
    int newAgc = (currentAgc == 1) ? 2 : 1; // Toggle between FAST (1) and SLOW (2)

    char gtCommand[8];
    std::snprintf(gtCommand, sizeof(gtCommand), "GT%d;", newAgc);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), gtCommand);

    const char *agcNames[] = {"OFF", "FAST", "SLOW", "AUTO"};
    ESP_LOGI(TAG, "Function Button 1 pressed: AGC from %s to %s", agcNames[currentAgc], agcNames[newAgc]);
}

void ButtonHandler::triggerFunctionButton2()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Toggle Noise Blanker - query current state first, then toggle
    int currentNB = m_radioManager.getState().noiseBlanker;
    int newNB = (currentNB + 1) % 3; // Cycle through 0, 1, 2

    char nlCommand[8];
    std::snprintf(nlCommand, sizeof(nlCommand), "NL%d;", newNB);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), nlCommand);

    const char *nbNames[] = {"OFF", "NB1", "NB2", "NB3"};
    ESP_LOGI(TAG, "Function Button 2 pressed: Noise Blanker from %s to %s", nbNames[currentNB], nbNames[newNB]);
}

void ButtonHandler::triggerFunctionButton3()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Cycle down through bands using BD command with shared band index
    int currentBandIndex = m_radioManager.getState().bandDownSlotIndex;
    int nextBandIndex = (currentBandIndex == 0) ? 10 : currentBandIndex - 1; // Wrap from band 0 to band 10 (GENE)

    // Use BD command for band down
    char bdCommand[8];
    std::snprintf(bdCommand, sizeof(bdCommand), "BD%d;", nextBandIndex);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), bdCommand);

    // Apply last-used mode for the target band
    const int8_t targetMode = getModeFromMemory(nextBandIndex);
    if (targetMode >= 1)
    {
        char mdCommand[8];
        std::snprintf(mdCommand, sizeof(mdCommand), "MD%d;", targetMode);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), mdCommand);
    }

    ESP_LOGI(TAG, "Function Button 3 pressed: Band Down from %s to %s", bandNames[currentBandIndex],
             bandNames[nextBandIndex]);

    // Invalidate frequency cache and request fresh FA/FB from radio
    m_radioManager.requestFrequencyUpdate();
}

void ButtonHandler::triggerFunctionButton4()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Toggle between noise blanker variants: OFF (0) -> NB1 (1) -> NB2 (2) -> NB3 (3) -> OFF (0)
    int currentNB = m_radioManager.getState().noiseBlanker;
    int nextNB = (currentNB + 1) % 3; // Cycle through 0, 1, 2

    // Use NL command for noise blanker
    char nlCommand[8];
    std::snprintf(nlCommand, sizeof(nlCommand), "NL%d;", nextNB);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), nlCommand);

    const char *nbNames[] = {"OFF", "NB1", "NB2", "NB3"};
    ESP_LOGI(TAG, "Function Button 4 pressed: Toggling Noise Blanker from %s to %s", nbNames[currentNB],
             nbNames[nextNB]);
}

void ButtonHandler::triggerFunctionButton5()
{
    // Function button 5 doesn't have a dedicated handler yet
    ESP_LOGI("BUTTONHANDLER", "Function Button 5 triggered from matrix");
}

void ButtonHandler::triggerFunctionButton6()
{
    // Function button 6 doesn't have a dedicated handler yet
    ESP_LOGI("BUTTONHANDLER", "Function Button 6 triggered from matrix");
}

void ButtonHandler::triggerTransverterMacroButton()
{
    if (!m_radioManager.getState().keepAlive.load())
    {
        ESP_LOGD(TAG, "Keep alive not active; ignoring transverter macro trigger");
        return;
    }

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Transverter Macro button blocked - panel is LOCKED");
        return;
    }

    ESP_LOGI(TAG, "Transverter macro button pressed, toggling state");
    const bool currentState = m_radioManager.getState().transverter;
    const bool newState = !currentState;

    bool result = m_macroManager.executeTransverterMacro(newState);
    if (result)
    {
        ESP_LOGI(TAG, "Transverter macro executed successfully");
        ESP_LOGI(TAG, "Transverter state toggled to: %s", newState ? "ENABLED" : "DISABLED");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to execute transverter macro: %s", m_macroManager.getLastError().c_str());
    }

    ESP_LOGI(TAG, "Final states - Transverter: %d, DRV Out: %d, RX ANT: %d",
             m_radioManager.getState().transverter.load(std::memory_order_relaxed),
             m_radioManager.getState().drvOut.load(std::memory_order_relaxed),
             m_radioManager.getState().rxAnt.load(std::memory_order_relaxed));
}

void ButtonHandler::triggerBandUpButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Band Up button blocked - panel is LOCKED");
        return;
    }

    m_radioManager.recordButtonActivity();

    // Cycle up through bands using BU command with shared band index
    int currentBandIndex = m_radioManager.getState().bandUpSlotIndex;
    int nextBandIndex = (currentBandIndex + 1) % 11; // Cycle through 0-10, wrap from band 10 (GENE) to band 0 (1.8MHz)

    // Use BU command for band up
    char buCommand[8];
    std::snprintf(buCommand, sizeof(buCommand), "BU%d;", nextBandIndex);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), buCommand);

    // Apply last-used mode for the target band
    const int8_t targetMode = getModeFromMemory(nextBandIndex);
    if (targetMode >= 1)
    {
        char mdCommand[8];
        std::snprintf(mdCommand, sizeof(mdCommand), "MD%d;", targetMode);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), mdCommand);
    }

    // Update our internal state to reflect the change
    m_radioManager.getState().bandUpSlotIndex = nextBandIndex;

    ESP_LOGI(TAG, "Matrix Mode key pressed: Band Up from %s to %s", bandNames[currentBandIndex],
             bandNames[nextBandIndex]);

    // Invalidate frequency cache and request fresh FA/FB from radio
    m_radioManager.requestFrequencyUpdate();
}

void ButtonHandler::triggerModeUpButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Get current mode and increment it
    int8_t currentMode = m_radioManager.getState().mode.load();

    // Valid modes: 1=LSB, 2=USB, 3=CW, 4=FM, 5=AM, 6=FSK, 7=CW-R, 9=FSK-R (skip 0,8)
    const int8_t validModes[] = {1, 2, 3, 4, 5, 6, 7, 9};
    const char *modeNames[] = {"Invalid", "LSB", "USB", "CW", "FM", "AM", "FSK", "CW-R", "Invalid", "FSK-R"};

    // Find current mode in valid modes array
    int currentIndex = -1;
    for (int i = 0; i < 8; i++)
    {
        if (validModes[i] == currentMode)
        {
            currentIndex = i;
            break;
        }
    }

    // If current mode is invalid, start from LSB (1)
    int8_t nextMode;
    if (currentIndex == -1)
    {
        nextMode = 1; // LSB
    }
    else
    {
        // Increment with wraparound
        int nextIndex = (currentIndex + 1) % 8;
        nextMode = validModes[nextIndex];
    }

    // Use MD command for mode change
    char mdCommand[8];
    std::snprintf(mdCommand, sizeof(mdCommand), "MD%d;", nextMode);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), mdCommand);

    ESP_LOGI(TAG, "Matrix TF Set key pressed: Mode Up from %s to %s",
             (currentMode >= 0 && currentMode <= 9) ? modeNames[currentMode] : "Unknown", modeNames[nextMode]);
}

void ButtonHandler::triggerModeDownButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Get current mode and decrement it
    int8_t currentMode = m_radioManager.getState().mode.load();

    // Valid modes: 1=LSB, 2=USB, 3=CW, 4=FM, 5=AM, 6=FSK, 7=CW-R, 9=FSK-R (skip 0,8)
    const int8_t validModes[] = {1, 2, 3, 4, 5, 6, 7, 9};
    const char *modeNames[] = {"Invalid", "LSB", "USB", "CW", "FM", "AM", "FSK", "CW-R", "Invalid", "FSK-R"};

    // Find current mode in valid modes array
    int currentIndex = -1;
    for (int i = 0; i < 8; i++)
    {
        if (validModes[i] == currentMode)
        {
            currentIndex = i;
            break;
        }
    }

    // If current mode is invalid, start from FSK-R (9)
    int8_t nextMode;
    if (currentIndex == -1)
    {
        nextMode = 9; // FSK-R
    }
    else
    {
        // Decrement with wraparound
        int nextIndex = (currentIndex == 0) ? 7 : currentIndex - 1;
        nextMode = validModes[nextIndex];
    }

    // Use MD command for mode change
    char mdCommand[8];
    std::snprintf(mdCommand, sizeof(mdCommand), "MD%d;", nextMode);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), mdCommand);

    ESP_LOGI(TAG, "Matrix Band key pressed: Mode Down from %s to %s",
             (currentMode >= 0 && currentMode <= 9) ? modeNames[currentMode] : "Unknown", modeNames[nextMode]);
}

void ButtonHandler::triggerVfoToggleButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;
    m_radioManager.recordButtonActivity();

    // Get current RX VFO state
    int currentRxVfo = m_radioManager.getRxVfo();
    bool splitEnabled = m_radioManager.isSplitEnabled();

    if (splitEnabled)
    {
        // In split mode: switch RX VFO while keeping TX on opposite VFO to maintain split
        if (currentRxVfo == 1)
        {
            // Currently RX=B, switch to RX=A with TX=B (to maintain split)
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "FR0;FT1;");
            ESP_LOGI(TAG, "VFO A/B toggle (split mode): RX=B -> RX=A, TX=B (split maintained)");
        }
        else
        {
            // Currently RX=A, switch to RX=B with TX=A (to maintain split)
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "FR1;FT0;");
            ESP_LOGI(TAG, "VFO A/B toggle (split mode): RX=A -> RX=B, TX=A (split maintained)");
        }
    }
    else
    {
        // In simplex mode: normal toggle using FR command only
        int newRxVfo = (currentRxVfo == 0) ? 1 : 0;
        char frCommand[8];
        std::snprintf(frCommand, sizeof(frCommand), "FR%d;", newRxVfo);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), frCommand);

        const char *vfoNames[] = {"VFO A", "VFO B", "Memory"};
        ESP_LOGI(TAG, "VFO A/B toggle (simplex): RX VFO changed from %s to %s",
                 (currentRxVfo >= 0 && currentRxVfo <= 2) ? vfoNames[currentRxVfo] : "Unknown",
                 (newRxVfo >= 0 && newRxVfo <= 2) ? vfoNames[newRxVfo] : "Unknown");
    }
}

void ButtonHandler::setupDiagnosticMode(TCA8418Handler *tca8418)
{
    if (!tca8418)
    {
        ESP_LOGE(TAG, "TCA8418Handler is null, cannot setup diagnostic mode");
        return;
    }

    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║          BUTTON DIAGNOSTIC MODE ENABLED                      ║");
    ESP_LOGI(TAG, "║  All buttons will log their key codes without executing      ║");
    ESP_LOGI(TAG, "║  Press each button and record the key code shown below       ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");

    // Define all possible key codes we want to monitor
    const uint8_t testKeys[] = {
        // Left PCB expected keys
        0x24, 0x25, 0x26, 0x27, 0x2D, 0x2E, 0x2F, 0x30, 0x31,
        // Right PCB expected keys
        0x01, 0x02, 0x03, 0x04, 0x06, 0x07, 0x08, 0x09,
        0x0B, 0x0C, 0x0D, 0x0E, 0x15, 0x16, 0x17,
        // Additional keys that might be present
        0x05, 0x0A, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
        0x20, 0x21, 0x22, 0x23, 0x28, 0x29, 0x2A, 0x2B, 0x2C
    };

    for (uint8_t keyCode : testKeys)
    {
        ESP_LOGD(TAG, "Registering callback for key 0x%02X", keyCode);
        tca8418->setKeyCallback(
            static_cast<TCA8418Handler::MatrixKey>(keyCode),
            [keyCode](TCA8418Handler::MatrixKey key, bool isPressed)
            {
                // Force log at INFO level to ensure it's visible
                printf("\n");
                printf("═══════════════════════════════════════════════\n");
                printf("🔍 BUTTON TEST - KEY CODE: 0x%02X (decimal %d)\n", keyCode, keyCode);
                printf("🔍 STATUS: %s\n", isPressed ? "PRESSED" : "RELEASED");
                printf("🔍 Which physical button did you press?\n");
                printf("═══════════════════════════════════════════════\n");

                ESP_LOGI("BUTTON_TEST", "═══════════════════════════════════════════════");
                ESP_LOGI("BUTTON_TEST", "KEY CODE: 0x%02X (decimal %d)", keyCode, keyCode);
                ESP_LOGI("BUTTON_TEST", "STATUS: %s", isPressed ? "PRESSED" : "RELEASED");
                ESP_LOGI("BUTTON_TEST", "Which physical button did you press?");
                ESP_LOGI("BUTTON_TEST", "═══════════════════════════════════════════════");
            });
    }

    ESP_LOGI(TAG, "✅ Diagnostic mode setup complete - monitoring %d key codes", sizeof(testKeys));
    ESP_LOGI(TAG, "📝 Record the key code for each physical button you press");
    ESP_LOGI(TAG, "⚠️  If you don't see BUTTON_TEST messages, callbacks aren't firing");
}

void ButtonHandler::setupTCA8418KeyMappings(TCA8418Handler *tca8418)
{
    if (!tca8418)
    {
        ESP_LOGE(TAG, "TCA8418Handler is null, cannot setup key mappings");
        return;
    }

    // Initialize matrix buttons first
    initializeMatrixButtons();

    // ========================================================================
    // LEFT PCB Matrix (C0-C1 × R0-R4) - 9 buttons
    // Physical layout verified against ARCI-SMT PCB
    // ========================================================================

    // 0x01 - SEND/MOX (C0,R0) - verified by PCB testing
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x01,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // SEND/MOX
                            });

    // 0x0C - POWER on/off (C2,R1) - verified by PCB testing
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x0C,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // POWER with short/long press support
                            });

    // 0x02 - VOX (verified by PCB testing)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x02,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // VOX with long-press support
                            });

    // 0x03 - TUNE (verified by PCB testing - physical TUNE button generates 0x03)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x03,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // TUNE/Antenna tuner
                            });

    // 0x04 - PRE/Preamp (verified by PCB testing - physical PRE button generates 0x04)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x04,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // PRE
                            });

    // 0x06 - (alternate/unused - physical TUNE uses 0x03)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x06,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // Fallback TUNE
                            });

    // 0x07 - XVTR/Transverter (C1,R1)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x07,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // XVTR
                            });

    // 0x08 - PROC/Speech Processor (C1,R2)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x08,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // PROC
                            });

    // 0x09 - ATT/Attenuator (C1,R3)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x09,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // ATT
                            });

    // 0x05 - PWR output (verified by PCB testing - bottom left button)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x05,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // PWR output
                            });

    // Left PCB Column 2 (physical right column) - verified by PCB testing
    // 0x0D - XVTR/Transverter
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x0D,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // XVTR (alt)
                            });

    // 0x0E - PROC/Speech Processor
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x0E,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // PROC
                            });

    // 0x0F - ATT/Attenuator
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x0F,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // ATT (alt)
                            });

    // ========================================================================
    // RIGHT PCB Matrix (C5,C7,C9 × R0-R3) - 12 buttons
    // Physical columns skip C6, C8
    // ========================================================================

    // Column 1 (C5): NTCH, BND+, BND-, CLR
    // 0x1A - NTCH/Notch (C5,R0)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x1A,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // Notch filter
                            });

    // 0x1B - BND+/Band Up (C5,R1)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x1B,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // BND+
                            });

    // 0x1C - BND-/Band Down (C5,R2)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x1C,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // BND-
                            });

    // 0x1D - CLR/Clear (C5,R3)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x1D,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // CLR
                            });

    // Column 2 (C7): NR, RIT, A/B, MODE
    // 0x24 - NR/Noise Reduction (C7,R0)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x24,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // NR
                            });

    // 0x25 - RIT (C7,R1)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x25,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // RIT
                            });

    // 0x26 - A/B VFO swap (C7,R2)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x26,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // A/B
                            });

    // 0x27 - MODE (C7,R3)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x27,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // MODE
                            });

    // Column 3 (C9): NB, XIT, SPLIT, ENT
    // 0x2E - NB/Noise Blanker (C9,R0)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x2E,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // NB
                            });

    // 0x2F - XIT (C9,R1)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x2F,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // XIT
                            });

    // 0x30 - SPLIT (C9,R2)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x30,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // SPLIT
                            });

    // 0x31 - ENT/Enter (C9,R3)
    tca8418->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x31,
                            [this](TCA8418Handler::MatrixKey key, const bool pressed)
                            {
                                handleMatrixButtonEvent(key, pressed); // ENT
                            });

    ESP_LOGD(TAG, "TCA8418 key mappings configured");
}

void ButtonHandler::setupF1F6KeyMappings(TCA8418Handler *tca8418Handler2)
{
    if (!tca8418Handler2)
    {
        ESP_LOGE(TAG, "TCA8418Handler #2 is null, cannot setup F1-F6 key mappings");
        return;
    }

    // Initialize F-button matrix with long press support
    initializeMatrixButtons2();

    // F1 - Actual key code 0x01 (Row 0, Col 0)
    tca8418Handler2->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x01,
        [this](TCA8418Handler::MatrixKey key, bool isPressed) {
            handleMatrixButton2Event(key, isPressed);  // Route through timing system
        });

    // F2 - Actual key code 0x0B (Row 0, Col 2)
    tca8418Handler2->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x0B,
        [this](TCA8418Handler::MatrixKey key, bool isPressed) {
            handleMatrixButton2Event(key, isPressed);  // Route through timing system
        });

    // F3 - Confirmed code 0x02 via short test (PCB Row 2, Col 1 = TCA C1+R0)
    tca8418Handler2->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x02,
        [this](TCA8418Handler::MatrixKey key, bool isPressed) {
            handleMatrixButton2Event(key, isPressed);  // Route through timing system
        });

    // F4 - Confirmed code 0x0C via short test (PCB Row 2, Col 2 = TCA C1+R1)
    tca8418Handler2->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x0C,
        [this](TCA8418Handler::MatrixKey key, bool isPressed) {
            handleMatrixButton2Event(key, isPressed);  // Route through timing system
        });

    // F5 - Actual key code 0x03 (Row 2, Col 0)
    tca8418Handler2->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x03,
        [this](TCA8418Handler::MatrixKey key, bool isPressed) {
            handleMatrixButton2Event(key, isPressed);  // Route through timing system
        });

    // F6 - Actual key code 0x0D (Row 2, Col 2)
    tca8418Handler2->setKeyCallback(TCA8418Handler::MatrixKey::KEY_0x0D,
        [this](TCA8418Handler::MatrixKey key, bool isPressed) {
            handleMatrixButton2Event(key, isPressed);  // Route through timing system
        });

    ESP_LOGD(TAG, "F1-F6 macro key mappings configured");
}

void ButtonHandler::initializeMatrixButtons()
{

    // Left PCB buttons that need button-like behavior (verified by PCB testing)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x01),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x01, 50, 500)); // SEND/MOX
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x02),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x02, 50, 500)); // VOX
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x03),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x03, 50, 500)); // TUNE (verified: physical TUNE→0x03)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x04),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x04, 50, 500)); // PRE (preamp)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x05),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x05, 50, 500)); // PWR output
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x06),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x06, 50, 500)); // TUNE fallback
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x07),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x07, 50, 500)); // XVTR (transverter)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x08),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x08, 50, 500)); // PROC (alt)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x09),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x09, 50, 500)); // ATT (attenuator)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0C),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0C, 50, 500)); // POWER
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0D),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0D, 50, 500)); // XVTR (alt)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0E),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0E, 50, 500)); // PROC
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0F),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0F, 50, 500)); // ATT (alt)

    // Right PCB buttons that need button-like behavior
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x1A),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x1A, 50, 500)); // NTCH (Notch)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x1B),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x1B, 50, 500)); // BND+ (Band Up)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x1C),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x1C, 50, 500)); // BND- (Band Down)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x1D),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x1D, 50, 500)); // CLR (Clear RIT/XIT)
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x24),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x24, 50, 500)); // NR
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x25),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x25, 50, 500)); // RIT
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x26),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x26, 50, 500)); // A/B
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x27),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x27, 50, 500)); // MODE
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x2E),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x2E, 50, 500)); // NB
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x2F),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x2F, 50, 500)); // XIT
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x30),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x30, 50, 500)); // SPLIT
    m_matrixButtons.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x31),
                            std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x31, 50, 500)); // ENT
}

void ButtonHandler::initializeMatrixButtons2()
{
    // F-buttons (TCA8418 #2) - support short/long press for 12 macro slots
    // Short press: slots 0-5, Long press: slots 6-11
    m_matrixButtons2.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x01),
                             std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x01, 50, 500)); // F1
    m_matrixButtons2.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0B),
                             std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0B, 50, 500)); // F2
    m_matrixButtons2.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x02),
                             std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x02, 50, 500)); // F3
    m_matrixButtons2.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0C),
                             std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0C, 50, 500)); // F4
    m_matrixButtons2.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x03),
                             std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x03, 50, 500)); // F5
    m_matrixButtons2.emplace(std::piecewise_construct, std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0D),
                             std::forward_as_tuple(TCA8418Handler::MatrixKey::KEY_0x0D, 50, 500)); // F6

    ESP_LOGD(TAG, "F1-F6 buttons initialized with long press support (12 macro slots)");
}

void ButtonHandler::handleMatrixButton2Event(const TCA8418Handler::MatrixKey key, const bool pressed)
{
    // F-buttons handler (TCA8418 #2) with short/long press detection
    // Routes to macro slots 0-5 (short press) or 6-11 (long press)

    auto it = m_matrixButtons2.find(key);
    if (it == m_matrixButtons2.end())
    {
        ESP_LOGW(TAG, "Unknown F-button key: 0x%02X", static_cast<uint8_t>(key));
        return;
    }

    MatrixButton &button = it->second;
    bool previousTcaState = button.getLastTcaState();
    button.updateState(pressed);

    // Only process if state changed
    if (pressed == previousTcaState)
    {
        return;
    }

    // Process short vs long press after timing logic runs
    if (!button.hasStateChanged())
    {
        return;
    }

    // Handle short and long press events
    switch (key)
    {
    case TCA8418Handler::MatrixKey::KEY_0x01: // F1
        if (button.wasLongPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F1 long press - executing macro slot 6");
            m_macroManager.executeSlot(6);
        }
        else if (button.wasShortPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F1 short press - executing macro slot 0");
            m_macroManager.executeSlot(0);
        }
        break;

    case TCA8418Handler::MatrixKey::KEY_0x0B: // F2
        if (button.wasLongPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F2 long press - executing macro slot 7");
            m_macroManager.executeSlot(7);
        }
        else if (button.wasShortPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F2 short press - executing macro slot 1");
            m_macroManager.executeSlot(1);
        }
        break;

    case TCA8418Handler::MatrixKey::KEY_0x02: // F3
        if (button.wasLongPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F3 long press - executing macro slot 8");
            m_macroManager.executeSlot(8);
        }
        else if (button.wasShortPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F3 short press - executing macro slot 2");
            m_macroManager.executeSlot(2);
        }
        break;

    case TCA8418Handler::MatrixKey::KEY_0x0C: // F4
        if (button.wasLongPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F4 long press - executing macro slot 9");
            m_macroManager.executeSlot(9);
        }
        else if (button.wasShortPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F4 short press - executing macro slot 3");
            m_macroManager.executeSlot(3);
        }
        break;

    case TCA8418Handler::MatrixKey::KEY_0x03: // F5
        if (button.wasLongPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F5 long press - executing macro slot 10");
            m_macroManager.executeSlot(10);
        }
        else if (button.wasShortPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F5 short press - executing macro slot 4");
            m_macroManager.executeSlot(4);
        }
        break;

    case TCA8418Handler::MatrixKey::KEY_0x0D: // F6
        if (button.wasLongPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F6 long press - executing macro slot 11");
            m_macroManager.executeSlot(11);
        }
        else if (button.wasShortPressed())
        {
            m_radioManager.signalUserActivity();
            ESP_LOGI(TAG, "F6 short press - executing macro slot 5");
            m_macroManager.executeSlot(5);
        }
        break;

    default:
        ESP_LOGW(TAG, "Unhandled F-button key: 0x%02X", static_cast<uint8_t>(key));
        break;
    }
}

void ButtonHandler::handleMatrixButtonEvent(const TCA8418Handler::MatrixKey key, const bool pressed)
{
    // POWER button (0x0C) needs special handling - it should work even when interface is OFF
    if (key == TCA8418Handler::MatrixKey::KEY_0x0C)
    {
        auto it = m_matrixButtons.find(key);
        if (it != m_matrixButtons.end())
        {
            bool previousTcaState = it->second.getLastTcaState();
            it->second.updateState(pressed);

            if (pressed != previousTcaState)
            {
                handlePowerButton(it->second);
            }
        }
        return; // Handle power button separately, bypass normal checks
    }

    // Check if interface is on - most button functions require this
    if (!m_radioManager.getState().keepAlive.load())
    {
        if (pressed)
        { // Only log on button press to avoid spam
            ESP_LOGW(TAG, "Button 0x%02X pressed but RRC interface is OFF - turn on interface first",
                     static_cast<uint8_t>(key));
        }
        return;
    }

    // Check panel lock state - allow only LOCK and Power buttons when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        // Allow MODE button (0x27, used as LOCK) and POWER button (0x0C) when panel is locked
        if (key != TCA8418Handler::MatrixKey::KEY_0x27 && key != TCA8418Handler::MatrixKey::KEY_0x0C)
        {
            if (pressed)
            { // Only log on button press to avoid spam
                ESP_LOGD(TAG, "Button 0x%02X blocked - panel is LOCKED (press MODE button to unlock)",
                         static_cast<uint8_t>(key));
            }
            return;
        }
    }

    // Update the matrix button state
    auto it = m_matrixButtons.find(key);
    if (it != m_matrixButtons.end())
    {
        // Store the previous TCA state before updating
        bool previousTcaState = it->second.getLastTcaState();

        it->second.updateState(pressed);

        // Only handle button logic if there was an actual TCA8418 state change
        if (pressed != previousTcaState)
        {
            // Signal user activity to wake display (debounced UIPS1)
            m_radioManager.signalUserActivity();

            switch (key)
            {
            // Left PCB buttons - mappings verified by PCB testing
            case TCA8418Handler::MatrixKey::KEY_0x01:
                handleMoxButton(it->second);             // SEND/MOX (0x01)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x02:
                handleVoxButton(it->second);             // VOX (0x02)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x03:
                handleAntennaTunerButton(it->second);    // TUNE (0x03) - verified physical TUNE→0x03
                break;
            case TCA8418Handler::MatrixKey::KEY_0x05:
                handleLeftPcbButton8(it->second);        // PWR output (0x05)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x06:
                handleAntennaTunerButton(it->second);    // TUNE fallback (0x06)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x08:
                handleSpeechProcessorButton(it->second); // PROC (0x08)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x0E:
                handleSpeechProcessorButton(it->second); // PROC (0x0E)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x04:
                if (pressed) handlePreampButton();       // PRE (0x04)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x07:
                if (pressed) triggerTransverterMacroButton(); // XVTR (0x07)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x09:
                if (pressed) handleRfAttenuatorButton(); // ATT (0x09)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x0D:
                if (pressed) triggerTransverterMacroButton(); // XVTR alt (0x0D)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x0F:
                if (pressed) handleRfAttenuatorButton(); // ATT alt (0x0F)
                break;

            // Right PCB buttons
            case TCA8418Handler::MatrixKey::KEY_0x1A:
                handleNotchButton(it->second);           // NTCH (0x1A)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x24:
                handleNoiseReductionButton(it->second);  // NR (0x24)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x25:
                handleRitButton(it->second);             // RIT (0x25)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x26:
                handleAEqualsBMatrixButton(it->second);  // A/B (0x26)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x27:
                handleModeMatrixButton(it->second);      // MODE (0x27)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x2E:
                handleNoiseBlankerButton(it->second);    // NB (0x2E)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x2F:
                handleXitButton(it->second);             // XIT (0x2F)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x1B:
                if (pressed) handleBandUpButton();       // BND+ (0x1B)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x1C:
                if (pressed) handleBandDownButton();     // BND- (0x1C)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x1D:
                if (pressed) handleClearButton();        // CLR (0x1D)
                break;
            case TCA8418Handler::MatrixKey::KEY_0x30:
                if (!pressed) handleSplitButton();       // SPLIT (0x30) - triggers on release
                break;
            case TCA8418Handler::MatrixKey::KEY_0x31:
                // ENT button - no specific handler yet
                break;
            default:
                break;
            }
        }
    }
}

// TCA8418 Matrix button handler implementations
void ButtonHandler::handleBandDownButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Band Down button blocked - panel is LOCKED");
        return;
    }

    m_radioManager.recordButtonActivity();

    // Non-blocking approach: Use cached frequency values instead of querying radio
    const uint8_t rxVfo = m_radioManager.getState().currentRxVfo.load();
    const uint64_t currentFreq = (rxVfo == 1) ? m_radioManager.getVfoBFrequency() : m_radioManager.getVfoAFrequency();

    ESP_LOGI(TAG, "Matrix Band Down: Current RX VFO %s frequency: %llu Hz (cached)", rxVfo == 1 ? "B" : "A",
             currentFreq);
    m_radioManager.decodeBandFromFreq(currentFreq);

    const int currentBandIndex = m_radioManager.getState().bandNumber.load(std::memory_order_relaxed);
    int nextBandIndex;
    if (currentBandIndex <= 0 || currentBandIndex > 9)
    {
        nextBandIndex = 9; // Wrap from 160m or invalid/GENE directly to 6m
    }
    else
    {
        nextBandIndex = currentBandIndex - 1;
        if (nextBandIndex == 10)
        {
            nextBandIndex = 9; // Skip GENE band
        }
    }

    char bdCommand[8];
    snprintf(bdCommand, sizeof(bdCommand), "BD%02d;", nextBandIndex);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), bdCommand);

    const int8_t targetMode = getModeFromMemory(nextBandIndex);
    if (targetMode >= 1)
    {
        char mdCommand[8];
        std::snprintf(mdCommand, sizeof(mdCommand), "MD%d;", targetMode);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), mdCommand);
    }

    const char *currentName =
        (currentBandIndex >= 0 && currentBandIndex < 11) ? bandNames[currentBandIndex] : "Unknown";
    ESP_LOGI(TAG, "🎛️ Band Down button (0x03): %s -> %s (non-blocking)", currentName, bandNames[nextBandIndex]);

    // Invalidate frequency cache and request fresh FA/FB from radio
    // This ensures display updates quickly after band change
    m_radioManager.requestFrequencyUpdate();
}

void ButtonHandler::handleBandUpButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Band Up button blocked - panel is LOCKED");
        return;
    }

    m_radioManager.recordButtonActivity();

    // Non-blocking approach: Use cached frequency values instead of querying radio
    const uint8_t rxVfo = m_radioManager.getState().currentRxVfo.load();
    const uint64_t currentFreq = (rxVfo == 1) ? m_radioManager.getVfoBFrequency() : m_radioManager.getVfoAFrequency();

    // Update band number based on current cached frequency
    ESP_LOGI(TAG, "Matrix Band Up: Current RX VFO %s frequency: %llu Hz (cached)", rxVfo == 1 ? "B" : "A", currentFreq);
    m_radioManager.decodeBandFromFreq(currentFreq);

    // Get the frequency-based band number (0-10) and increment
    const int currentBandIndex = m_radioManager.getState().bandNumber.load(std::memory_order_relaxed);
    // Skip band 10 (GENE) in progression - wrap from band 9 directly to band 0
    int nextBandIndex;
    if (currentBandIndex >= 9)
    {
        nextBandIndex = 0; // Wrap from 50MHz (band 9) to 160m (band 0)
    }
    else
    {
        nextBandIndex = currentBandIndex + 1;
    }
    ESP_LOGI(TAG, "Matrix Band Up: Mapped to band index: %d, next: %d", currentBandIndex, nextBandIndex);

    // Send commands without blocking delays
    char buCommand[8];
    snprintf(buCommand, sizeof(buCommand), "BU%02d;", nextBandIndex);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), buCommand);

    // Apply last-used mode for the target band
    const int8_t targetMode = getModeFromMemory(nextBandIndex);
    if (targetMode >= 1)
    {
        char mdCommand[8];
        std::snprintf(mdCommand, sizeof(mdCommand), "MD%d;", targetMode);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), mdCommand);
    }

    ESP_LOGI(TAG, "🎛️ Band Up button (0x02): %s -> %s (non-blocking)", bandNames[currentBandIndex],
             bandNames[nextBandIndex]);

    // Invalidate frequency cache and request fresh FA/FB from radio
    // This ensures display updates quickly after band change
    m_radioManager.requestFrequencyUpdate();
}

void ButtonHandler::handleNotchButton(MatrixButton &button)
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Notch button blocked - panel is LOCKED");
        return;
    }

    const auto &uiState = m_radioManager.getState().uiState;
    const bool notchPopupOpen = uiState.isActive() && uiState.getActiveControl() == radio::UIControl::NotchFrequency;

    // Handle long press - toggle notch frequency popup
    if (button.wasLongPressed())
    {
        if (notchPopupOpen)
        {
            // Close popup without applying
            ESP_LOGI(TAG, "Notch button long press - closing notch frequency popup");
            m_radioManager.exitUIMode(false);
            return;
        }

        // Open notch frequency popup
        // Must be in Manual Notch mode - enable it if not already
        int currentNotch = m_radioManager.getState().notchFilterMode;
        if (currentNotch != 2)
        {
            // Enable Manual Notch (mode=2, keep current bandwidth)
            int bandwidth = m_radioManager.getState().notchFilterBandwidth;
            ESP_LOGI(TAG, "Notch button long press - enabling Manual Notch first (was mode %d)", currentNotch);
            char ntCommand[8];
            std::snprintf(ntCommand, sizeof(ntCommand), "NT2%d;", bandwidth);
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), ntCommand);
        }

        // Get current notch frequency value (0-127)
        int currentFreq = m_radioManager.getState().manualNotchFrequency;
        if (currentFreq < 0) currentFreq = 0;
        if (currentFreq > 127) currentFreq = 127;

        ESP_LOGI(TAG, "Notch button long press - entering notch frequency UI mode (0-127)");
        m_radioManager.enterUIMode(radio::UIControl::NotchFrequency, currentFreq, 0, 127, 1);
        return;
    }

    // Handle short press
    if (button.wasShortReleased())
    {
        if (notchPopupOpen)
        {
            // Toggle between Normal and Wide bandwidth while popup is open
            int currentBandwidth = m_radioManager.getState().notchFilterBandwidth;
            int newBandwidth = (currentBandwidth == 0) ? 1 : 0;

            ESP_LOGI(TAG, "Notch button short press - toggling bandwidth: %s -> %s",
                     currentBandwidth == 0 ? "Normal" : "Wide",
                     newBandwidth == 0 ? "Normal" : "Wide");

            // Send NT command to change bandwidth (mode stays at 2=Manual)
            char ntCommand[8];
            std::snprintf(ntCommand, sizeof(ntCommand), "NT2%d;", newBandwidth);
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), ntCommand);

            // Re-enter UI mode to refresh popup
            int currentFreq = m_radioManager.getState().manualNotchFrequency;
            if (currentFreq < 0) currentFreq = 0;
            if (currentFreq > 127) currentFreq = 127;
            m_radioManager.enterUIMode(radio::UIControl::NotchFrequency, currentFreq, 0, 127, 1);
            return;
        }

        // Normal short press when popup closed - cycle notch modes
        int currentMode = m_radioManager.getState().mode.load();
        bool isCwMode = (currentMode == 3 || currentMode == 7); // CW or CW-R

        int currentNotch = m_radioManager.getState().notchFilterMode;
        int currentBandwidth = m_radioManager.getState().notchFilterBandwidth;

        int newNotch, newBandwidth;
        std::string modeDescription;

        if (isCwMode)
        {
            // CW mode: cycle through OFF (NT00;) -> Manual Normal (NT20;) -> Manual Wide (NT21;) -> OFF
            if (currentNotch == 0)
            {
                newNotch = 2;
                newBandwidth = 0;
                modeDescription = "Manual Normal";
            }
            else if (currentNotch == 2 && currentBandwidth == 0)
            {
                newNotch = 2;
                newBandwidth = 1;
                modeDescription = "Manual Wide";
            }
            else
            {
                newNotch = 0;
                newBandwidth = 0;
                modeDescription = "OFF";
            }

            const char *currentDesc = (currentNotch == 0)      ? "OFF"
                : (currentNotch == 2 && currentBandwidth == 0) ? "Manual Normal"
                                                               : "Manual Wide";
            ESP_LOGI(TAG, "Notch button (CW mode): %s -> %s", currentDesc, modeDescription.c_str());
        }
        else
        {
            // Non-CW modes: cycle through OFF -> Auto -> Manual Normal -> Manual Wide -> OFF
            if (currentNotch == 0)
            {
                // OFF -> Auto
                newNotch = 1;
                newBandwidth = 0;
                modeDescription = "Auto";
            }
            else if (currentNotch == 1)
            {
                // Auto -> Manual Normal
                newNotch = 2;
                newBandwidth = 0;
                modeDescription = "Manual Normal";
            }
            else if (currentNotch == 2 && currentBandwidth == 0)
            {
                // Manual Normal -> Manual Wide
                newNotch = 2;
                newBandwidth = 1;
                modeDescription = "Manual Wide";
            }
            else
            {
                // Manual Wide -> OFF
                newNotch = 0;
                newBandwidth = 0;
                modeDescription = "OFF";
            }

            const char *currentDesc = (currentNotch == 0)      ? "OFF"
                : (currentNotch == 1)                          ? "Auto"
                : (currentNotch == 2 && currentBandwidth == 0) ? "Manual Normal"
                                                               : "Manual Wide";
            ESP_LOGI(TAG, "Notch button: %s -> %s", currentDesc, modeDescription.c_str());
        }

        char notchCommand[8];
        std::snprintf(notchCommand, sizeof(notchCommand), "NT%d%d;", newNotch, newBandwidth);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), notchCommand);

        // Update local state immediately to ensure consistent cycling
        m_radioManager.getState().notchFilterMode = newNotch;
        m_radioManager.getState().notchFilterBandwidth = newBandwidth;
    }
}

void ButtonHandler::handleLockButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Toggle panel lock state
    bool currentLock = m_radioManager.getState().panelLock;
    bool newLock = !currentLock;

    // Format: LKP1P2; where P1=0/1 (OFF/ON), P2=0 (always 0) per TS-590SG specification
    char lockCommand[8];
    std::snprintf(lockCommand, sizeof(lockCommand), "LK%d0;", newLock ? 1 : 0);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), lockCommand);

    ESP_LOGI(TAG, "Panel Lock button pressed (0x04): %s -> %s", currentLock ? "ON" : "OFF", newLock ? "ON" : "OFF");
}

void ButtonHandler::handleAEqualsBMatrixButton(MatrixButton &button)
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "A=B button blocked - panel is LOCKED");
        return;
    }

    if (button.wasLongPressed())
    {
        ESP_LOGI(TAG, "A=B button long press detected - copying RX to opposite VFO");
        trigger_A_equals_B_button();
    }

    if (button.wasShortReleased())
    {
        ESP_LOGI(TAG, "A=B button short press detected - toggling VFO A/B");
        m_radioManager.recordButtonActivity();
        handleVfoToggleButton();
    }
}

void ButtonHandler::handleVfoToggleButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "VFO Toggle button blocked - panel is LOCKED");
        return;
    }

    bool rxVfo = m_radioManager.getRxVfo(); // false=VFO A, true=VFO B
    bool splitEnabled = m_radioManager.isSplitEnabled();

    ESP_LOGI(TAG, "🎛️ VFO A/B: Button pressed - Current RX=%s, Split=%s", rxVfo ? "VFO B" : "VFO A",
             splitEnabled ? "ON" : "OFF");

    if (splitEnabled)
    {
        // In split mode: switch RX VFO while maintaining split (TX on opposite VFO)
        // FT parameter meaning: 0=TX same as RX (simplex), 1=TX opposite from RX (split)
        if (rxVfo)
        {
            // Currently RX=B, switch to RX=A, maintain split (TX=B)
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "FR0;FT1;"); // RX=A, TX=opposite(A)=B
            ESP_LOGI(TAG, "VFO A/B button (split mode): RX=B -> RX=A, TX=B (split maintained)");
        }
        else
        {
            // Currently RX=A, switch to RX=B, maintain split (TX=A)
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "FR1;FT0;"); // RX=B, TX=opposite(B)=A
            ESP_LOGI(TAG, "VFO A/B button (split mode): RX=A -> RX=B, TX=A (split maintained)");
        }
    }
    else
    {
        // In simplex mode: toggle RX VFO only - TX will follow automatically
        // Do NOT send FT commands as they activate split mode on TS-590SG
        if (rxVfo)
        {
            // Currently on VFO B, switch to VFO A
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "FR0;");
            ESP_LOGI(TAG, "VFO A/B button (simplex): VFO B -> VFO A");
        }
        else
        {
            // Currently on VFO A, switch to VFO B
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "FR1;");
            ESP_LOGI(TAG, "VFO A/B button (simplex): VFO A -> VFO B");
        }
    }
}

void ButtonHandler::handleNoiseReductionButton(MatrixButton &button)
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Noise Reduction button blocked - panel is LOCKED");
        return;
    }

    const auto &uiState = m_radioManager.getState().uiState;
    const auto activeCtrl = uiState.getActiveControl();
    const bool nrPopupOpen = uiState.isActive() &&
        (activeCtrl == radio::UIControl::NrLevel || activeCtrl == radio::UIControl::Nr2Speed);

    // Handle long press - toggle NR level/speed popup
    if (button.wasLongPressed())
    {
        if (nrPopupOpen)
        {
            // Close popup without applying
            ESP_LOGI(TAG, "NR button long press - closing NR popup");
            m_radioManager.exitUIMode(false);
            return;
        }

        // NR level can only be set when NR is ON (NR1 or NR2)
        // If NR is OFF, first enable NR1 before entering level adjustment
        int nrMode = m_radioManager.getState().noiseReductionMode;
        if (nrMode == 0)
        {
            ESP_LOGI(TAG, "NR button long press - enabling NR1 first (was OFF)");
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "NR1;");
            nrMode = 1; // Will be NR1 after command executes
        }

        // Use different UI control types for NR1 vs NR2:
        // NR1: level 1-10, NR2: SPAC speed 0-9 (2-20ms)
        const bool isNr2 = (nrMode == 2);
        const radio::UIControl controlType = isNr2 ? radio::UIControl::Nr2Speed : radio::UIControl::NrLevel;
        const int minVal = isNr2 ? 0 : 1;
        const int maxVal = isNr2 ? 9 : 10;

        // Use the appropriate stored value for each mode
        int currentLevel = isNr2 ? m_radioManager.getState().nr2Speed
                                 : m_radioManager.getState().nr1Level;
        // Clamp to valid range for current mode
        if (currentLevel < minVal) currentLevel = minVal;
        if (currentLevel > maxVal) currentLevel = maxVal;

        ESP_LOGI(TAG, "NR button long press - entering %s UI mode (range %d-%d)",
                 isNr2 ? "NR2 SPAC speed" : "NR1 level", minVal, maxVal);
        m_radioManager.enterUIMode(controlType, currentLevel, minVal, maxVal, 1);
        return;
    }

    // Handle short press
    if (button.wasShortReleased())
    {
        if (nrPopupOpen)
        {
            // Switch between NR1 level and NR2 speed while popup is open
            if (activeCtrl == radio::UIControl::NrLevel)
            {
                // Switch to NR2 speed - change mode and show NR2 popup
                ESP_LOGI(TAG, "NR button short press - switching to NR2 speed");
                m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "NR2;");

                int currentSpeed = m_radioManager.getState().nr2Speed;
                if (currentSpeed < 0) currentSpeed = 0;
                if (currentSpeed > 9) currentSpeed = 9;

                m_radioManager.enterUIMode(radio::UIControl::Nr2Speed, currentSpeed, 0, 9, 1);
            }
            else
            {
                // Switch to NR1 level - change mode and show NR1 popup
                ESP_LOGI(TAG, "NR button short press - switching to NR1 level");
                m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "NR1;");

                int currentLevel = m_radioManager.getState().nr1Level;
                if (currentLevel < 1) currentLevel = 1;
                if (currentLevel > 10) currentLevel = 10;

                m_radioManager.enterUIMode(radio::UIControl::NrLevel, currentLevel, 1, 10, 1);
            }
            return;
        }

        // Normal short press when popup closed - cycle NR mode
        int currentNR = m_radioManager.getState().noiseReductionMode;
        int newNR = (currentNR + 1) % 3; // Cycle through 0 (OFF), 1 (NR1), 2 (NR2)

        char nrCommand[8];
        std::snprintf(nrCommand, sizeof(nrCommand), "NR%d;", newNR);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), nrCommand);

        const char *nrNames[] = {"OFF", "NR1", "NR2"};
        ESP_LOGI(TAG, "NR button short press: %s -> %s", nrNames[currentNR], nrNames[newNR]);
    }
}

void ButtonHandler::handleRitButton(MatrixButton &button)
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "RIT button blocked - panel is LOCKED");
        return;
    }

    const auto &uiState = m_radioManager.getState().uiState;
    const bool ritXitPopupOpen = uiState.isActive() && uiState.getActiveControl() == radio::UIControl::RitXitOffset;

    // Handle long press - toggle RIT/XIT offset popup
    if (button.wasLongPressed())
    {
        if (ritXitPopupOpen)
        {
            // Close popup without applying
            ESP_LOGI(TAG, "RIT button long press - closing RIT/XIT offset popup");
            m_radioManager.exitUIMode(false);
            return;
        }

        // Open RIT/XIT offset popup
        // Enable RIT if not already on
        bool currentRIT = m_radioManager.isRitEnabled();
        if (!currentRIT)
        {
            ESP_LOGI(TAG, "RIT button long press - enabling RIT first (was OFF)");
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "RT1;");
        }

        // Query RT; to confirm RIT state from radio
        ESP_LOGI(TAG, "RIT button long press - querying RT; for current state");
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "RT;");

        // Get current RIT/XIT offset value (-9999 to +9999)
        int currentOffset = m_radioManager.getState().ritXitOffset.load();
        ESP_LOGI(TAG, "RIT button long press - ritXitOffset from state: %d Hz", currentOffset);
        if (currentOffset < -9999) currentOffset = -9999;
        if (currentOffset > 9999) currentOffset = 9999;

        ESP_LOGI(TAG, "RIT button long press - entering RIT/XIT offset UI mode (value=%d, range=-9999 to +9999)", currentOffset);
        m_radioManager.enterUIMode(radio::UIControl::RitXitOffset, currentOffset, -9999, 9999, 10);
        return;
    }

    // Handle short press - always toggle RIT on/off
    if (button.wasShortReleased())
    {
        bool currentRIT = m_radioManager.isRitEnabled();
        int newRIT = currentRIT ? 0 : 1;

        char rtCommand[8];
        std::snprintf(rtCommand, sizeof(rtCommand), "RT%d;", newRIT);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), rtCommand);

        // Query RT; to confirm RIT state from radio
        ESP_LOGI(TAG, "RIT button short press - querying RT; for current state");
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "RT;");

        ESP_LOGI(TAG, "RIT button short press: %s -> %s", currentRIT ? "ON" : "OFF", newRIT ? "ON" : "OFF");
    }
}


void ButtonHandler::handleXitButton(MatrixButton &button)
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "XIT button blocked - panel is LOCKED");
        return;
    }

    const auto &uiState = m_radioManager.getState().uiState;
    const bool ritXitPopupOpen = uiState.isActive() && uiState.getActiveControl() == radio::UIControl::RitXitOffset;

    // Handle long press - toggle RIT/XIT offset popup
    if (button.wasLongPressed())
    {
        if (ritXitPopupOpen)
        {
            // Close popup without applying
            ESP_LOGI(TAG, "XIT button long press - closing RIT/XIT offset popup");
            m_radioManager.exitUIMode(false);
            return;
        }

        // Open RIT/XIT offset popup
        // Enable XIT if not already on
        bool currentXIT = m_radioManager.isXitEnabled();
        if (!currentXIT)
        {
            ESP_LOGI(TAG, "XIT button long press - enabling XIT first (was OFF)");
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "XT1;");
        }

        // Query XT; to confirm XIT state from radio
        ESP_LOGI(TAG, "XIT button long press - querying XT; for current state");
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "XT;");

        // Get current RIT/XIT offset value (-9999 to +9999)
        int currentOffset = m_radioManager.getState().ritXitOffset.load();
        ESP_LOGI(TAG, "XIT button long press - ritXitOffset from state: %d Hz", currentOffset);
        if (currentOffset < -9999) currentOffset = -9999;
        if (currentOffset > 9999) currentOffset = 9999;

        ESP_LOGI(TAG, "XIT button long press - entering RIT/XIT offset UI mode (value=%d, range=-9999 to +9999)", currentOffset);
        m_radioManager.enterUIMode(radio::UIControl::RitXitOffset, currentOffset, -9999, 9999, 10);
        return;
    }

    // Handle short press - always toggle XIT on/off
    if (button.wasShortReleased())
    {
        bool currentXIT = m_radioManager.isXitEnabled();
        int newXIT = currentXIT ? 0 : 1;

        char xtCommand[8];
        std::snprintf(xtCommand, sizeof(xtCommand), "XT%d;", newXIT);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), xtCommand);

        // Query XT; to confirm XIT state from radio
        ESP_LOGI(TAG, "XIT button short press - querying XT; for current state");
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "XT;");

        ESP_LOGI(TAG, "XIT button short press: %s -> %s", currentXIT ? "ON" : "OFF", newXIT ? "ON" : "OFF");
    }
}

void ButtonHandler::handleNoiseBlankerButton(MatrixButton &button)
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Noise Blanker button blocked - panel is LOCKED");
        return;
    }

    const auto &uiState = m_radioManager.getState().uiState;
    const bool nbPopupOpen = uiState.isActive() && uiState.getActiveControl() == radio::UIControl::NbLevel;

    // Handle long press - toggle NB level popup
    if (button.wasLongPressed())
    {
        if (nbPopupOpen)
        {
            // Close popup without applying
            ESP_LOGI(TAG, "NB button long press - closing NB level popup");
            m_radioManager.exitUIMode(false);
            return;
        }

        // NB level can only be set when NB is ON (NB1, NB2, or NB3)
        // If NB is OFF, first enable NB1 before entering level adjustment
        const int currentNBMode = m_radioManager.getState().noiseBlanker;
        if (currentNBMode == 0)
        {
            ESP_LOGI(TAG, "NB button long press - enabling NB1 first (was OFF)");
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "NB1;");
        }

        // NL command: range 001-010 (1-10) per spec
        int currentLevel = m_radioManager.getState().noiseBlankerLevel;
        if (currentLevel < 1) currentLevel = 1;
        if (currentLevel > 10) currentLevel = 10;

        ESP_LOGI(TAG, "NB button long press - entering NB level UI mode (range 1-10)");
        m_radioManager.enterUIMode(radio::UIControl::NbLevel, currentLevel, 1, 10, 1);
        return;
    }

    // Handle short press
    if (button.wasShortReleased())
    {
        if (nbPopupOpen)
        {
            // Cycle NB modes (NB1→NB2→NB3→NB1) while popup is open
            int currentNB = m_radioManager.getState().noiseBlanker;
            // Cycle 1→2→3→1 (staying within active modes, not going to OFF)
            int newNB = (currentNB % 3) + 1;

            const char *nbNames[] = {"OFF", "NB1", "NB2", "NB3"};
            ESP_LOGI(TAG, "NB button short press - cycling mode: %s -> %s (popup stays open)",
                     nbNames[currentNB], nbNames[newNB]);

            char nbCommand[8];
            std::snprintf(nbCommand, sizeof(nbCommand), "NB%d;", newNB);
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), nbCommand);

            // Re-enter UI mode to refresh popup (level stays the same)
            int currentLevel = m_radioManager.getState().noiseBlankerLevel;
            if (currentLevel < 1) currentLevel = 1;
            if (currentLevel > 10) currentLevel = 10;
            m_radioManager.enterUIMode(radio::UIControl::NbLevel, currentLevel, 1, 10, 1);
            return;
        }

        // Normal short press when popup closed - cycle NB mode
        int currentNB = m_radioManager.getState().noiseBlanker;
        // Bounds check to prevent array out-of-bounds crash
        if (currentNB < 0 || currentNB > 3)
        {
            ESP_LOGW(TAG, "NB button: Invalid state %d, treating as OFF", currentNB);
            currentNB = 0;
        }
        int newNB = (currentNB + 1) % 4; // Cycle through 0 (OFF), 1 (NB1), 2 (NB2), 3 (NB3)

        char nbCommand[8];
        std::snprintf(nbCommand, sizeof(nbCommand), "NB%d;", newNB);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), nbCommand);

        const char *nbNames[] = {"OFF", "NB1", "NB2", "NB3"};
        ESP_LOGI(TAG, "NB button short press: %s -> %s", nbNames[currentNB], nbNames[newNB]);
    }
}


void ButtonHandler::handleSpeechProcessorButton(MatrixButton &button)
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Speech Processor button blocked - panel is LOCKED");
        return;
    }

    const auto &uiState = m_radioManager.getState().uiState;
    const auto activeCtrl = uiState.getActiveControl();
    const bool procPopupOpen = uiState.isActive() &&
        (activeCtrl == radio::UIControl::ProcInputLevel ||
         activeCtrl == radio::UIControl::ProcOutputLevel ||
         activeCtrl == radio::UIControl::DataMode);

    // Handle long press - open processor level popup
    if (button.wasLongPressed())
    {
        if (procPopupOpen)
        {
            // Close popup without applying
            ESP_LOGI(TAG, "PROC button long press - closing processor level popup");
            m_radioManager.exitUIMode(false);
            return;
        }

        // Query current PROC levels from radio to ensure fresh values
        ESP_LOGD(TAG, "PROC button long press - querying current PL levels");
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "PL;");

        // Auto-enable processor if it's OFF
        bool currentPR = m_radioManager.getState().processor;
        if (!currentPR)
        {
            ESP_LOGI(TAG, "PROC button long press - enabling processor first (was OFF)");
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "PR1;");
        }

        // Open processor input level popup (use cached value, will update when PL response arrives)
        int currentLevel = m_radioManager.getState().speechProcessorInLevel;
        // Use default of 50 if value appears uninitialized (will be updated when PL response arrives)
        if (currentLevel == 0) currentLevel = 50;
        if (currentLevel < 0) currentLevel = 0;
        if (currentLevel > 100) currentLevel = 100;

        ESP_LOGI(TAG, "PROC button long press - entering processor input level UI mode (level=%d)", currentLevel);
        m_radioManager.enterUIMode(radio::UIControl::ProcInputLevel, currentLevel, 0, 100, 1);
        return;
    }

    // Handle short press
    if (button.wasShortReleased())
    {
        if (procPopupOpen)
        {
            // Cycle through: Input → Output → Data Mode → Input...
            if (activeCtrl == radio::UIControl::ProcInputLevel)
            {
                // Switch to output level
                int currentLevel = m_radioManager.getState().speechProcessorOutLevel;
                // Use default of 50 if value appears uninitialized
                if (currentLevel == 0) currentLevel = 50;
                if (currentLevel < 0) currentLevel = 0;
                if (currentLevel > 100) currentLevel = 100;

                ESP_LOGI(TAG, "PROC button short press - switching to output level (level=%d)", currentLevel);
                m_radioManager.enterUIMode(radio::UIControl::ProcOutputLevel, currentLevel, 0, 100, 1);
            }
            else if (activeCtrl == radio::UIControl::ProcOutputLevel)
            {
                // Switch to data mode toggle
                int8_t currentDataMode = m_radioManager.getState().dataMode.load();
                ESP_LOGI(TAG, "PROC button short press - switching to data mode (current=%d)", currentDataMode);
                m_radioManager.enterUIMode(radio::UIControl::DataMode, currentDataMode, 0, 1, 1);
            }
            else // DataMode
            {
                // Cycle back to input level
                int currentLevel = m_radioManager.getState().speechProcessorInLevel;
                // Use default of 50 if value appears uninitialized
                if (currentLevel == 0) currentLevel = 50;
                if (currentLevel < 0) currentLevel = 0;
                if (currentLevel > 100) currentLevel = 100;

                ESP_LOGI(TAG, "PROC button short press - cycling back to input level (level=%d)", currentLevel);
                m_radioManager.enterUIMode(radio::UIControl::ProcInputLevel, currentLevel, 0, 100, 1);
            }
            return;
        }

        // Normal short press - toggle Speech Processor (PR command)
        ESP_LOGI(TAG, "Speech Processor button short press - toggling PR");

        bool currentPR = m_radioManager.getState().processor;
        int newPR = currentPR ? 0 : 1; // Toggle between 0 (OFF) and 1 (ON)

        char prCommand[8];
        std::snprintf(prCommand, sizeof(prCommand), "PR%d;", newPR);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), prCommand);

        ESP_LOGI(TAG, "Speech Processor: %s -> %s", currentPR ? "ON" : "OFF", newPR ? "ON" : "OFF");
    }
}

void ButtonHandler::handleRfAttenuatorButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "RF Attenuator button blocked - panel is LOCKED");
        return;
    }

    bool currentRA = m_radioManager.getState().attenuator;
    int newRA = currentRA ? 0 : 1; // Toggle between 0 (OFF) and 1 (ON)

    // RA command format: RA<P1><P1>; where P1P1 is 00 (OFF) or 01 (ON)
    char raCommand[8];
    std::snprintf(raCommand, sizeof(raCommand), "RA0%d;", newRA);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), raCommand);

    ESP_LOGI(TAG, "RF Attenuator button pressed (0x27): %s -> %s", currentRA ? "ON" : "OFF", newRA ? "ON" : "OFF");
}


void ButtonHandler::handlePreampButton()
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Preamp button blocked - panel is LOCKED");
        return;
    }

    const bool currentPreamp = m_radioManager.getState().preAmplifier;
    const bool newPreamp = !currentPreamp;

    char paCommand[8];
    std::snprintf(paCommand, sizeof(paCommand), "PA%d;", newPreamp ? 1 : 0);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), paCommand);

    ESP_LOGI(TAG, "Preamp button (0x04): %s -> %s", currentPreamp ? "ON" : "OFF", newPreamp ? "ON" : "OFF");
}

void ButtonHandler::handleClearButton()
{
    // CLR button - Clears RIT/XIT offset
    // Note: keepAlive and panelLock checks already done in handleMatrixButtonEvent()

    int prevOffset = m_radioManager.getState().ritXitOffset.load();
    ESP_LOGI(TAG, "CLR button pressed (0x1D) - clearing RIT/XIT offset (was %d Hz)", prevOffset);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "RC;");

    m_radioManager.getState().ritXitOffset.store(0);
    ESP_LOGI(TAG, "CLR: ritXitOffset set to 0");

    const auto &uiState = m_radioManager.getState().uiState;
    if (uiState.isActive() && uiState.getActiveControl() == radio::UIControl::RitXitOffset)
    {
        ESP_LOGI(TAG, "CLR: Popup open, re-entering UI mode with value 0");
        m_radioManager.enterUIMode(radio::UIControl::RitXitOffset, 0, -9999, 9999, 10);
    }
}

void ButtonHandler::handleSplitButton()
{
    // SPLIT button - Toggle split mode
    // Note: keepAlive and panelLock checks already done in handleMatrixButtonEvent()

    ESP_LOGI(TAG, "SPLIT button pressed (0x30)");
    m_radioManager.toggleSplit(/*copyVfoBeforeEnable*/ false);
}

// Mode memory management methods
void ButtonHandler::saveModeToMemory(const int bandIndex, const int8_t mode)
{
    if (bandIndex >= 0 && bandIndex < 11)
    {
        m_bandModeMemory[bandIndex] = mode;
        ESP_LOGD(TAG, "Saved mode %d to band %d memory", mode, bandIndex);

        // Persist to NVS immediately for reliability
        saveModeMemoryToNvs();
    }
}

int8_t ButtonHandler::getModeFromMemory(const int bandIndex) const
{
    if (bandIndex >= 0 && bandIndex < 11)
    {
        return m_bandModeMemory[bandIndex];
    }
    return 2; // Default to USB if invalid band
}

template <size_t N>
int8_t ButtonHandler::getNextModeInCycle(const int8_t currentMode, const std::array<int8_t, N> &validModes) const
{
    // Find current mode in valid modes array
    int currentIndex = -1;
    for (size_t i = 0; i < N; i++)
    {
        if (validModes[i] == currentMode)
        {
            currentIndex = i;
            break;
        }
    }

    // If current mode is invalid, start from first mode
    if (currentIndex == -1)
    {
        return validModes[0];
    }

    // Get next mode with wraparound
    const size_t nextIndex = (static_cast<size_t>(currentIndex) + 1) % N;
    return validModes[nextIndex];
}

int8_t ButtonHandler::getDefaultModeForFrequency(const uint64_t frequency) const
{
    // LSB for ≤10MHz, USB for >10MHz (standard amateur practice)
    return (frequency <= 10000000) ? 1 : 2;
}

// NVS persistence methods
void ButtonHandler::loadModeMemoryFromNvs()
{
    esp_err_t err = m_nvsManager.loadButtonModeMemory(
        reinterpret_cast<uint8_t*>(m_bandModeMemory.data()),
        m_bandModeMemory.size()
    );

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Loaded band mode memory from NVS via NvsManager");
    }
    else if (err == ESP_ERR_NOT_FOUND)
    {
        ESP_LOGD(TAG, "No mode memory in NVS, using defaults");
    }
    else if (err == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "Invalid mode data in NVS, resetting to defaults");
        // Reset to defaults per user preference:
        // 160/80/40m = LSB, 30/20/17/15/12/10/6m = USB, GENE = LSB
        m_bandModeMemory = {1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1};
        saveModeMemoryToNvs(); // Save corrected defaults
    }
    else
    {
        ESP_LOGE(TAG, "Failed to load mode memory: %s, using defaults", esp_err_to_name(err));
    }
}

void ButtonHandler::saveModeMemoryToNvs() const
{
    esp_err_t err = m_nvsManager.saveButtonModeMemory(
        reinterpret_cast<const uint8_t*>(m_bandModeMemory.data()),
        m_bandModeMemory.size()
    );

    if (err == ESP_OK)
    {
        ESP_LOGD(TAG, "Band mode memory saved to NVS via NvsManager");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to save mode memory to NVS: %s", esp_err_to_name(err));
    }
}

// New MatrixButton-based handlers that use Button class logic
void ButtonHandler::handleModeMatrixButton(MatrixButton &button)
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Mode button blocked - panel is LOCKED");
        return;
    }

    const int8_t currentMode = m_radioManager.getMode();
    const int bandIndex = m_radioManager.getState().bandNumber.load(std::memory_order_relaxed);
    const char *modeNames[] = {"Invalid", "LSB", "USB", "CW", "FM", "AM", "FSK", "CW-R", "Invalid", "FSK-R"};

    // Handle long press using Button class logic
    if (button.wasLongPressed())
    {
        // Long press action - toggle between alternate modes and save to memory
        ESP_LOGI(TAG, "Mode button long press detected");

        int8_t nextMode = currentMode;

        switch (currentMode)
        {
        case 1:
            nextMode = 2;
            break; // LSB -> USB
        case 2:
            nextMode = 1;
            break; // USB -> LSB
        case 3:
            nextMode = 7;
            break; // CW -> CW-R
        case 7:
            nextMode = 3;
            break; // CW-R -> CW
        case 4:
            nextMode = 5;
            break; // FM -> AM
        case 5:
            nextMode = 4;
            break; // AM -> FM
        case 6:
            nextMode = 9;
            break; // FSK -> FSK-R
        case 9:
            nextMode = 6;
            break; // FSK-R -> FSK
        default:
            nextMode = getDefaultModeForFrequency(m_radioManager.getVfoAFrequency());
            break;
        }

        if (nextMode != currentMode)
        {
            char mdCommand[8];
            std::snprintf(mdCommand, sizeof(mdCommand), "MD%d;", nextMode);
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), mdCommand);

            // Save the new mode to memory for this band
            saveModeToMemory(bandIndex, nextMode);
        }

        ESP_LOGI(TAG, "Mode button long press: %s -> %s (alternate, saved to band %d)", modeNames[currentMode],
                 modeNames[nextMode], bandIndex);
    }

    // Handle short press using Button class logic
    if (button.wasShortPressed())
    {
        // Short press action - band-dependent cycle
        ESP_LOGI(TAG, "Mode button short press detected");

        // Determine current RX frequency to pick cycle set
        const uint8_t rxVfo = m_radioManager.getState().currentRxVfo.load();
        const uint64_t freq = (rxVfo == 1) ? m_radioManager.getVfoBFrequency() : m_radioManager.getVfoAFrequency();

        // Build preferred cycle for this band/frequency
        // Low bands (160/80/40): LSB, CW, AM, FSK
        // 60m: USB, CW, AM, FSK
        // High bands (30m/20m/17m/15m/12m/10m/6m/up): USB, CW, AM, FSK
        std::array<int8_t, 4> cycle{};
        if ((freq >= 1800000 && freq <= 2000000) || (freq >= 3500000 && freq <= 4000000) ||
            (freq >= 7000000 && freq <= 7300000))
        {
            cycle = {1, 3, 5, 6};
        }
        else if (freq >= 5000000 && freq <= 5600000)
        { // 60m region (approx)
            cycle = {2, 3, 5, 6};
        }
        else
        {
            cycle = {2, 3, 5, 6};
        }

        // Normalize disallowed/alternate modes before cycling
        int8_t normalized = currentMode;
        if (normalized == 7)
            normalized = 3; // CW-R -> CW
        if (normalized == 9)
            normalized = 6; // FSK-R -> FSK
        if (normalized == 4)
            normalized = 5; // FM not in cycle; treat as AM

        // Cycle to next preferred mode
        const int8_t nextMode = getNextModeInCycle(normalized, cycle);

        if (nextMode != currentMode)
        {
            char mdCommand[8];
            std::snprintf(mdCommand, sizeof(mdCommand), "MD%d;", nextMode);
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), mdCommand);
            // Persist last used mode per band on short press
            saveModeToMemory(bandIndex, nextMode);
        }

        ESP_LOGI(TAG, "Mode button short press: %s -> %s (cycle)", modeNames[currentMode], modeNames[nextMode]);
    }
}

void ButtonHandler::handleMoxButton(MatrixButton &button)
{
    static bool isTx0On = false; // Application state only - track TX0 switch state

    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "MOX button blocked - panel is LOCKED");
        return;
    }

    // Handle long press detection during button hold
    if (button.wasLongPressed())
    {
        // Long press detected while button is held down - start tune mode (TX2)
        ESP_LOGI(TAG, "MOX button long press detected - starting tune (TX2)");
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "TX2;");
    }

    // Handle short press completion (released after short press)
    if (button.wasShortReleased())
    {
        // Short press completed - toggle TX0 switch on/off
        ESP_LOGI(TAG, "MOX button short press completed - toggling TX0");

        if (isTx0On)
        {
            // Turn TX0 switch OFF
            ESP_LOGI(TAG, "🎛️ MOX: Sending RX; command to localCATHandler");
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "RX;");
            ESP_LOGI(TAG, "MOX switch: OFF (RX)");
            isTx0On = false;
        }
        else
        {
            // Turn TX0 switch ON
            ESP_LOGI(TAG, "🎛️ MOX: Sending TX0; command to localCATHandler");
            m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "TX0;");
            ESP_LOGI(TAG, "MOX switch: ON (TX0)");
            isTx0On = true;
        }
    }

    // Handle long press release (released after long press)
    if (button.wasLongReleased())
    {
        // Released after long press - return to RX (tune off)
        ESP_LOGI(TAG, "MOX button released after long press - returning to RX");
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), "RX;");
        // Note: Don't change isTx0On state - maintain switch position
    }
}

void ButtonHandler::handleVoxButton(MatrixButton &button)
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "VOX button blocked - panel is LOCKED");
        return;
    }

    // Handle long press using Button class logic
    if (button.wasLongPressed())
    {
        // Long press - Set VOX delay (VD command)
        ESP_LOGI(TAG, "VOX button long press detected - adjusting VOX delay");

        // Query current VOX delay and increment it (cycling through reasonable values)
        // For simplicity, we'll cycle through common delay values: 150ms, 300ms, 500ms, 1000ms
        static int voxDelayIndex = 0;
        const int voxDelays[] = {150, 300, 500, 1000}; // milliseconds
        const int numDelays = sizeof(voxDelays) / sizeof(voxDelays[0]);

        voxDelayIndex = (voxDelayIndex + 1) % numDelays;
        int newDelay = voxDelays[voxDelayIndex];

        // Send VD command to set VOX delay
        char vdCommand[12];
        std::snprintf(vdCommand, sizeof(vdCommand), "VD%d;", newDelay);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), vdCommand);

        ESP_LOGI(TAG, "VOX button long press: Set VOX delay to %d ms", newDelay);
    }

    // Handle short press using Button class logic
    if (button.wasShortPressed())
    {
        // Short press - toggle VOX on/off
        ESP_LOGI(TAG, "VOX button short press detected - toggling VOX");

        // Note: We don't have VOX state tracked in RadioState, so we'll need to query first
        // or maintain our own state. For now, we'll toggle based on a simple assumption
        // and let the radio handle the actual state

        static bool voxState = false; // Track VOX state locally
        voxState = !voxState;

        int voxMode = voxState ? 1 : 0; // 0=OFF, 1=ON

        // Send VX command to toggle VOX
        char vxCommand[8];
        std::snprintf(vxCommand, sizeof(vxCommand), "VX%d;", voxMode);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), vxCommand);

        ESP_LOGI(TAG, "VOX button short press: VOX %s", voxState ? "ON" : "OFF");
    }
}

void ButtonHandler::fallbackToRadioAntennaSwitch()
{
    // Fallback to radio's built-in antenna switching (ANT1/ANT2)
    int currentAnt = m_radioManager.getState().mainAntenna;
    int newAnt = (currentAnt == 0) ? 1 : 0; // Toggle between ANT1 (0) and ANT2 (1)

    // Use AN command: AN<main_ant><rx_ant><drv_out>;
    // Keep RX antenna and DRV unchanged (9), only change main antenna
    char anCommand[8];
    std::snprintf(anCommand, sizeof(anCommand), "AN%d99;", newAnt);
    m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), anCommand);

    const char *antNames[] = {"ANT1", "ANT2"};
    ESP_LOGI(TAG, "Radio antenna fallback: %s -> %s", antNames[currentAnt], antNames[newAnt]);
}

void ButtonHandler::handleAntennaTunerButton(MatrixButton &button)
{
    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "Antenna Tuner button blocked - panel is LOCKED");
        return;
    }

    // Handle long press using Button class logic
    if (button.wasLongPressed())
    {
        // Long press - start antenna tuning cycle
        ESP_LOGI(TAG, "Antenna Tuner button long press detected");

        // AC111; enables RX-AT IN, TX-AT IN, and starts tuning
        std::string acCommand = "AC111;";
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), acCommand);

        ESP_LOGI(TAG, "Antenna Tuner button long press: Starting tuning cycle");
    }

    // Handle short press using Button class logic
    if (button.wasShortPressed())
    {
        // Short press - toggle antenna tuner on/off
        ESP_LOGI(TAG, "Antenna Tuner button short press detected");

        bool currentTxAT = m_radioManager.getState().txAtIn;
        int newTxAT = currentTxAT ? 0 : 1; // Toggle TX-AT between 0 (THRU) and 1 (IN)

        // AC<RX><TX>0; - RX path follows TX path, no tuning
        char acCommand[8];
        std::snprintf(acCommand, sizeof(acCommand), "AC%d%d0;", newTxAT, newTxAT);
        m_radioManager.dispatchMessage(m_radioManager.getPanelCATHandler(), acCommand);

        ESP_LOGI(TAG, "Antenna Tuner button short press: %s -> %s", currentTxAT ? "ON" : "OFF", newTxAT ? "ON" : "OFF");
    }
}

void ButtonHandler::handleLeftPcbButton8(MatrixButton &button)
{
    // PWR button (0x31): Controls RF output power and carrier level via UI mode
    // - Short press: Enter power (PC) adjustment UI mode
    // - Long press: Enter carrier/monitor level (ML) adjustment UI mode

    if (!m_radioManager.getState().keepAlive.load())
        return;

    // Check panel lock state - block button when locked
    if (m_radioManager.getState().panelLock.load(std::memory_order_relaxed))
    {
        ESP_LOGD(TAG, "PWR button blocked - panel is LOCKED");
        return;
    }

    m_radioManager.recordButtonActivity();
    auto &state = m_radioManager.getState();

    // Handle long press: Enter carrier/monitor level (ML) adjustment mode
    if (button.wasLongPressed())
    {
        // Only enter UI mode if not already active
        if (!m_radioManager.isUIModeActive())
        {
            const int currentLevel = state.txMonitorLevel;
            // ML command range: 000-020 for TS-590SG (000=OFF, 001-020 level)
            m_radioManager.enterUIMode(radio::UIControl::CarrierLevel, currentLevel, 0, 20, 1);
            ESP_LOGI(TAG, "PWR button long press - entering carrier level (ML) UI mode, current=%d", currentLevel);
        }
    }

    // Handle long press release (nothing special needed, UI mode continues)
    if (button.wasLongReleased())
    {
        ESP_LOGD(TAG, "PWR button hold ended");
    }

    // Handle short press: Enter power (PC) adjustment mode
    if (button.wasShortPressed())
    {
        // If UI mode is already active, exit it (toggle behavior)
        if (m_radioManager.isUIModeActive())
        {
            ESP_LOGI(TAG, "PWR button short press - exiting UI mode (apply=false)");
            m_radioManager.exitUIMode(false);
        }
        else
        {
            const int currentPower = state.transmitPower;
            // PC command range: 005-100W for TS-590SG (5W min, 100W max)
            // Use step size of 5 for faster adjustment
            m_radioManager.enterUIMode(radio::UIControl::Power, currentPower, 5, 100, 5);
            ESP_LOGI(TAG, "PWR button short press - entering power (PC) UI mode, current=%dW", currentPower);
        }
    }
}
