#pragma once

#include <array>
#include <atomic>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string_view>
#include <unordered_map>
#include "rtos_mutex.h"
#include "../../AntennaSwitch/include/AntennaSwitch.h"
#include "../../CommonConstants/include/radio_constants.h"
#include "CATHandler.h"
#include "CommandDispatcher.h"
#include "RadioState.h"
#include "ISerialChannel.h"

namespace tcp_cat_bridge
{
    class TcpCatBridge;
}

namespace radio
{

    // Forward declarations
    class CommandDispatcher;
    class RadioMacroManager;

    // Power state enum (moved from Cat)
    enum PowerState
    {
        Off = 0,
        On = 1
    };

    /**
     * @brief Central manager for all radio state and command processing
     *
     * - Maintains the single authoritative RadioState
     * - Processes events from both local (USB) and remote (radio) sources
     * - Prevents feedback loops by tracking command sources
     * - Provides immediate responses to local queries from cached state
     * - Manages synchronization with the physical radio
     */
    class RadioManager
    {
    public:
        /**
         * @brief Constructor
         * @param radioSerial Reference to radio serial handler
         * @param usbSerial Reference to USB serial handler
         */
        RadioManager(ISerialChannel &radioSerial, ISerialChannel &usbSerial);
        ~RadioManager();

        // Non-copyable, non-movable for safety
        RadioManager(const RadioManager &) = delete;
        RadioManager &operator=(const RadioManager &) = delete;
        RadioManager(RadioManager &&) = delete;
        RadioManager &operator=(RadioManager &&) = delete;

        /**
         * @brief Get read-only access to the radio state
         */
        const RadioState &getState() const { return state_; }

        /**
         * @brief Get read-write access to the radio state
         */
        RadioState &getState() { return state_; }

        /**
         * @brief Clear the command cache timestamps
         * Used primarily for testing to ensure clean cache state
         */
        void clearCommandCache() const { state_.commandCache.clear(); }

        /**
         * @brief Check if radio appears to be connected and responding
         */
        bool isRadioConnected() const { return state_.radioConnected.load(); }

        /**
         * @brief Manually mark radio as connected/disconnected
         */
        void setRadioConnected(const bool connected) { state_.radioConnected.store(connected); }

        /**
         * @brief Get statistics about command processing
         */
        struct Statistics
        {
            size_t totalCommandsProcessed{0};
            size_t localCommandsProcessed{0};
            size_t remoteCommandsProcessed{0};
            size_t readCommandsProcessed{0};
            size_t setCommandsProcessed{0};
            size_t feedbackLoopsPrevented{0};
        };

        Statistics getStatistics() const { return stats_; }
        void resetStatistics() { stats_ = Statistics{}; }

        // Business logic methods moved from Cat facade

        /**
         * @brief Static initialization method (for compatibility)
         */
        static esp_err_t init();

        /**
         * @brief Start FreeRTOS tasks (TX timeout monitor, keepalive)
         *
         * MUST be called after constructor and FreeRTOS scheduler is running.
         * Separated from constructor to avoid init-order hazards.
         *
         * @return ESP_OK on success, ESP_FAIL if tasks cannot be created
         */
        esp_err_t startTasks();

        /**
         * @brief Perform boot sequence initialization
         */
        void performBootSequence() const;

        /**
         * @brief Perform periodic synchronization with radio
         */
        void performPeriodicSync();

        /**
         * @brief Synchronize transverter-related menu settings with the radio on startup
         */
        void syncTransverterMenuSettings() const;

        /**
         * @brief Update band from VFO A frequency
         */
        void updateBandFromVfoA();

        /**
         * @brief Update band from VFO B frequency
         */
        void updateBandFromVfoB();

        /**
         * @brief Decode band from frequency in Hz
         * @param frequencyHz Frequency in Hz
         */
        void decodeBandFromFreq(uint64_t frequencyHz);

        /**
         * @brief Decode band from frequency string (convenience overload)
         * @param frequency Frequency string to decode
         */
        void decodeBandFromFreq(std::string_view frequency);

        /**
         * @brief Change to next band
         */
        void changeBand();

        /**
         * @brief Send VFO updates to USB
         */
        void sendVfoUpdates() const;

        // Runtime toggle for transverter offset behavior (affects display and USB CDC)
        bool isTransverterOffsetEnabled() const { return state_.transverterOffsetEnabled.load(std::memory_order_relaxed); }
        void setTransverterOffsetEnabled(bool enabled) { state_.transverterOffsetEnabled.store(enabled, std::memory_order_relaxed); }

        // === TRANSVERTER FREQUENCY CONVERSION (Centralized - THE authoritative implementation) ===

        /**
         * @brief Check if transverter offset is currently active
         * @return true if transverterOffsetEnabled && transverter && transverterOffsetHz > 0
         */
        bool isTransverterOffsetActive() const;

        /**
         * @brief Convert base (radio) frequency to display frequency
         * @param baseFreq Frequency in Hz as seen by the radio
         * @return Frequency in Hz for display/USB (with transverter offset applied if active)
         *
         * Uses atomic snapshot of transverter state to prevent race conditions.
         * Returns baseFreq unchanged if transverter offset is not active.
         */
        uint64_t baseToDisplayFrequency(uint64_t baseFreq) const;

        /**
         * @brief Convert display frequency to base (radio) frequency
         * @param displayFreq Frequency in Hz from display/USB
         * @return Frequency in Hz for the radio (with transverter offset removed if active)
         *
         * Uses atomic snapshot of transverter state to prevent race conditions.
         * Returns displayFreq unchanged if transverter offset is not active.
         * Handles underflow protection for offset-minus case.
         */
        uint64_t displayToBaseFrequency(uint64_t displayFreq) const;

        /**
         * @brief Refresh display with current VFO frequencies (applying transverter offset)
         *
         * Sends FA and FB commands to display with the appropriate frequency representation.
         * Call this when UIXD toggles to immediately update the display.
         */
        void refreshDisplayFrequencies() const;

        /**
         * @brief Check if a radio response should be forwarded to USB
         * @param response
         * @return true if response should be forwarded to USB
         */
        bool shouldForwardToUSB(const std::string_view &response) const;

        bool shouldForwardToDisplay(const std::string_view &response) const;

        /**
         * @brief Check if there has been recent external CAT activity from USB
         * @return true if external CAT commands detected within configured timeout
         */
        bool hasRecentExternalCatActivity() const;

        /**
         * @brief Record button activity for burst polling
         */
        void recordButtonActivity();

        /**
         * @brief Record encoder activity for burst polling
         */
        void recordEncoderActivity();

        /**
         * @brief Signal user activity to wake display and verify state
         * Call this from input handler entry points when physical input detected.
         * If display is asleep, sends UIPS1 wake immediately. If awake, sends UIPS; query every 15s to verify state.
         */
        void signalUserActivity();

        /**
         * @brief Update display awake state (called when UIPS response received)
         * @param awake true if display is awake, false if screensaver active
         */
        void setDisplayAwake(bool awake);

        /**
         * @brief Update display screensaver timeout (called when UIPT response received)
         * @param minutes Screensaver timeout in minutes (0 = disabled)
         */
        void setDisplayScreensaverTimeout(uint8_t minutes);

        /**
         * @brief Check if there has been recent user activity (button or encoder)
         * @return true if button or encoder activity detected within burst timeout
         */
        bool hasRecentUserActivity() const;

        /**
         * @brief Set callback function to signal immediate polling needs
         * @param callback Function to call when immediate polling is needed
         */
        void setImmediatePollingCallback(void (*callback)());

        /**
         * @brief Set callback function to handle power state changes
         * @param callback Function to call when power state changes (powerOn, oldState)
         */
        void setPowerStateChangeCallback(void (*callback)(bool powerOn, bool oldState));

        // Legacy: clearPendingQuery(EventType) removed with RadioEvent pipeline

        /**
         * @brief Send direct response to USB (for error responses, etc.)
         * @param response The response string to send
         */
        void sendDirectResponse(std::string_view response) const;

        // Optional display serial for mirroring local SET commands
        void setDisplaySerial(ISerialChannel *display);
        ISerialChannel *getDisplaySerial() const;
        void sendToDisplay(std::string_view frames) const;

        // Optional TCP CAT bridges for network clients
        void setTcp0Bridge(tcp_cat_bridge::TcpCatBridge *bridge);
        void setTcp1Bridge(tcp_cat_bridge::TcpCatBridge *bridge);

        // Optional macro manager for user-defined macro execution
        void setMacroManager(RadioMacroManager *macroManager) { macroManager_ = macroManager; }
        RadioMacroManager *getMacroManager() const { return macroManager_; }

        // Send frames to a specific local interface (CDC0/CDC1/Display/Panel)
        void sendToSource(CommandSource src, std::string_view frames) const;

        /**
         * @brief Send a raw CAT command directly to the radio (outside dispatcher)
         *
         * This bypasses the CAT parser and should only be used for maintenance tasks
         * that need to align the physical radio with local policy.
         */
        void sendRawRadioCommand(std::string_view command) const;

        /**
         * @brief Request immediate frequency update from radio
         *
         * Invalidates the FA/FB cache entries and sends FA; and FB; queries
         * to the radio to get fresh frequency values. Use after band changes
         * or other operations that modify VFO frequencies.
         */
        void requestFrequencyUpdate();

        // Optional second USB CDC serial (CDC1) for targeted routing
        void setUsbCdc1Serial(ISerialChannel *cdc1) { usbCdc1Serial_ = cdc1; }

        // USB serial access for encoder frequency updates in AI2/AI4 modes
        ISerialChannel &getUsbSerial() { return usbSerial_; }

        // Frequency access methods for command handlers
        /**
     * @brief Get current VFO A frequency
     * @return Frequency in Hz
         * Hot path: Called frequently during CAT processing
         */
        uint64_t getVfoAFrequency() const __attribute__((hot))
        {
            return state_.vfoAFrequency.load(std::memory_order_relaxed);
        }

        /**
         * @brief Get current VFO B frequency
         * @return Frequency in Hz
         * Hot path: Called frequently during CAT processing
         */
        uint64_t getVfoBFrequency() const __attribute__((hot))
        {
            return state_.vfoBFrequency.load(std::memory_order_relaxed);
        }

        /**
         * @brief Update VFO A frequency
         * @param frequency New frequency in Hz
         * @return true if frequency was changed
         */
        bool updateVfoAFrequency(uint64_t frequency);

        /**
         * @brief Update VFO B frequency
         * @param frequency New frequency in Hz
         * @return true if frequency was changed
         */
        bool updateVfoBFrequency(uint64_t frequency);


        // Mode access methods for command handlers
        /**
         * @brief Get current operating mode
         * @return Mode value (1-9 for TS-590SG)
         */
        int getCurrentMode() const { return state_.mode.load(); }

        /**
         * @brief Get current data mode status
         * @return Data mode (0=off, 1=on)
         */
        int getDataMode() const { return state_.dataMode.load(); }

        /**
         * @brief Update operating mode
         * @param mode New mode value
         * @return true if mode was changed
         */
        bool updateMode(int mode);

        /**
         * @brief Update data mode
         * @param dataMode New data mode (0=off, 1=on)
         * @return true if data mode was changed
         */
        bool updateDataMode(int dataMode);

        // VFO selection methods for command handlers
        /**
         * @brief Get current RX VFO
         * @return RX VFO selection (0=VFO A, 1=VFO B, 2=Memory)
         */
        int getRxVfo() const { return state_.currentRxVfo.load(); }

        /**
         * @brief Get current TX VFO
         * @return TX VFO selection (0=VFO A, 1=VFO B, 2=Memory)
         */
        int getTxVfo() const { return state_.currentTxVfo.load(); }

        /**
         * @brief Update RX VFO selection
         * @param vfo New RX VFO selection (0=VFO A, 1=VFO B, 2=Memory)
         * @return true if VFO selection was changed
         */
        bool updateRxVfo(int vfo);

        /**
     * @brief Update TX VFO selection
     * @param vfo New TX VFO selection (0=VFO A, 1=VFO B, 2=Memory)

         * * @return true if VFO selection was changed
     */
        bool updateTxVfo(int vfo);

        // Primary control lease helpers for multi-client arbitration
        bool acquirePrimaryControl(CommandSource source);
        bool refreshPrimaryControl(CommandSource source);
        void releasePrimaryControl(CommandSource source);
        bool hasPrimaryControl(CommandSource source) const;
        int currentPrimaryControlOwner() const;
        void forceReleasePrimaryControl();

        // Thread-safe CAT dispatch entry point used by all tasks/macros
        bool dispatchMessage(CATHandler &handler, std::string_view message) const;

        // Mode access methods for command handlers
        /**
     * @brief Get current operating mode
     * @return Mode value (1-9 for TS-590SG)
     */
        int getMode() const { return state_.mode.load(); }

        // Split control methods
        /**
         * @brief Check if split is enabled
         * @return true if split is enabled
         */
        bool isSplitEnabled() const { return state_.split.load(); }

        /**
         * @brief Update split enabled state
         * @param enabled New split enabled state
         * @return true if state was changed
         */
        bool updateSplitEnabled(bool enabled);

        // RIT/XIT control methods
        /**
         * @brief Check if RIT is enabled
         * @return true if RIT is enabled
         */
        bool isRitEnabled() const { return state_.ritOn.load(); }

        /**
         * @brief Check if XIT is enabled
         * @return true if XIT is enabled
         */
        bool isXitEnabled() const { return state_.xitOn.load(); }

        /**
         * @brief Update RIT enabled state
         * @param enabled New RIT enabled state
         * @return true if state was changed
         */
        bool updateRitEnabled(bool enabled);

        /**
         * @brief Update XIT enabled state
         * @param enabled New XIT enabled state
         * @return true if state was changed
         */
        bool updateXitEnabled(bool enabled);

        /**
         * @brief Get current RIT offset
         * @return RIT offset in Hz
         */
        int getRitOffset() const { return state_.ritXitOffset.load(); }

        /**
         * @brief Get current XIT offset (same as RIT offset in TS-590SG)
         * @return XIT offset in Hz
         */
        int getXitOffset() const { return state_.ritXitOffset.load(); }

        /**
         * @brief Update RIT offset
         * @param offset New RIT offset in Hz
         * @return true if offset was changed
         */
        bool updateRitOffset(int offset);

        /**
         * @brief Update XIT offset (same as RIT offset in TS-590SG)
         * @param offset New XIT offset in Hz
         * @return true if offset was changed
         */
        bool updateXitOffset(int offset);

        /**
         * @brief Update power state
         * @param powerOn New power state (true=on, false=off)
         * @return true if power state was changed
         */
        bool updatePowerState(bool powerOn);

        // === CAT HANDLER ACCESS ===

        /**
         * @brief Get access to the local CAT handler (for USB commands)
         */
        CATHandler &getLocalCATHandler() const { return *localHandler_; }

        /**
         * @brief Get access to the remote CAT handler (for radio responses)
         */
        CATHandler &getRemoteCATHandler() const { return *remoteHandler_; }

        /**
         * @brief Get access to the panel CAT handler (for on-device controls)
         */
        CATHandler &getPanelCATHandler() const { return *panelHandler_; }

        /**
         * @brief Get access to the macro CAT handler (for internal macros)
         */
        CATHandler &getMacroCATHandler() const { return *macroHandler_; }

        /**
         * @brief Get access to the antenna switch
         */
        antenna::AntennaSwitch &getAntennaSwitch() { return *antennaSwitch_; }
        const antenna::AntennaSwitch &getAntennaSwitch() const { return *antennaSwitch_; }

        /**
         * @brief Get access to the command dispatcher
         */
        CommandDispatcher &getCommandDispatcher() { return *commandDispatcher_; }
        const CommandDispatcher &getCommandDispatcher() const { return *commandDispatcher_; }

        // === HIGH-LEVEL CONVENIENCE METHODS ===

        /**
         * @brief Send one or more local CAT frames through the handler
         */
        void sendLocal(const std::string_view frames) const { localHandler_->parseMessage(frames); }
        void sendLocal(std::initializer_list<std::string_view> frames) const;


        /**
         * @brief Enable split mode
         * @param copyVfoBeforeEnable Whether to copy VFO A to B before enabling
         */
        void enableSplit(bool copyVfoBeforeEnable = true) const;

        /**
         * @brief Disable split mode
         */
        void disableSplit() const;

        /**
         * @brief Toggle split mode
         * @param copyVfoBeforeEnable Whether to copy VFO A to B when enabling
         */
        void toggleSplit(bool copyVfoBeforeEnable = true) const;

        /**
         * @brief Set AF gain level
         */
        void setAfGain(int gain) const;

        /**
         * @brief Set RF gain level
         */
        void setRfGain(int gain) const;

        /**
         * @brief Set RIT/XIT offset value
         */
        void setRitXitValue(int value) const;

        /**
         * @brief Adjust filter width (mode-aware: FW for CW/FSK/FM, SH/SL for SSB/AM)
         * @param delta Positive=wider, negative=narrower
         */
        void setFilterWidth(int delta) const;

        /**
         * @brief Set IF shift value (CW/CW-R only)
         * @param valueHz IF shift in Hz (0-9999)
         */
        void setIfShift(int valueHz);

        /**
         * @brief Reset IF shift to center (0 Hz)
         */
        void resetIfShift() const;

        /**
         * @brief Adjust high cut frequency (SH command)
         * @param delta Positive=increase, negative=decrease
         */
        void setHighCut(int delta) const;

        /**
         * @brief Adjust low cut frequency (SL command)
         * @param delta Positive=increase, negative=decrease
         */
        void setLowCut(int delta) const;

        /**
         * @brief Set operating mode
         */
        void setMode(int mode) const;

        /**
         * @brief Set data mode
         */
        void setDataMode(int8_t mode) const;

        /**
         * @brief Toggle data mode
         */
        void toggleDataMode() const;

        /**
         * @brief Set processor state
         */
        void setProcessorState(int proc) const;

        // === VFO CONVENIENCE METHODS ===

        /**
         * @brief Copy VFO A to VFO B
         */
        void copyVfoAToB() const { sendLocal("VV;"); }

        /**
         * @brief Set RX on VFO A
         */
        void setRxOnA() const { sendLocal("FR0;"); }

        /**
         * @brief Set RX on VFO B
         */
        void setRxOnB() const { sendLocal("FR1;"); }

        /**
         * @brief Set TX on VFO A
         */
        void setTxOnA() const { sendLocal("FT0;"); }

        /**
         * @brief Set TX on VFO B
         */
        void setTxOnB() const { sendLocal("FT1;"); }

        // === LEGACY ACCESSORS ===

        /**
         * @brief Get power state as on/off flag
         */
        uint8_t getOnOffState() const { return state_.powerOn.load() ? 1 : 0; }

        /**
         * @brief Get power state enum
         */
        PowerState getPowerState() const { return state_.powerOn.load() ? On : Off; }

        /**
         * @brief Get power state enum
         */
        bool getPreamp() const { return state_.preAmplifier; }

        /**
         * @brief Get radio AI mode (from radio/display)
         */
        AIMode getAiMode() const { return static_cast<AIMode>(state_.aiMode.load()); }

        // Per-CDC AI mode getters
        AIMode getUsbCdc0AiMode() const { return static_cast<AIMode>(state_.usbCdc0AiMode.load()); }
        AIMode getUsbCdc1AiMode() const { return static_cast<AIMode>(state_.usbCdc1AiMode.load()); }

        /**
         * @brief Get display AI mode (independent from radio and USB)
         */
        AIMode getDisplayAiMode() const { return static_cast<AIMode>(state_.displayAiMode.load()); }

        /**
         * @brief Get split status
         */
        int getSplitStatus() const { return state_.split.load() ? 1 : 0; }

        // === UI MODE METHODS (Panel-Display Interaction) ===

        /**
         * @brief Enter UI adjustment mode for a specific control
         * @param control The UI control type to adjust (Power, CarrierLevel, etc.)
         * @param initialValue Starting value for the control
         * @param minValue Minimum allowed value
         * @param maxValue Maximum allowed value
         * @param stepSize Step size for encoder adjustments
         * @param timeoutUs Timeout in microseconds (0 = use default)
         */
        void enterUIMode(UIControl control, int initialValue, int minValue, int maxValue, int stepSize = 1,
                         uint64_t timeoutUs = 0);

        /**
         * @brief Exit UI mode and optionally apply the value
         * @param applyValue If true, send the actual CAT command to the radio
         */
        void exitUIMode(bool applyValue = true);

        /**
         * @brief Check if UI mode is currently active
         */
        bool isUIModeActive() const { return state_.uiState.isActive(); }

        /**
         * @brief Get the active UI control type
         */
        UIControl getActiveUIControl() const { return state_.uiState.getActiveControl(); }

        /**
         * @brief Adjust UI value by delta (for encoder turns)
         * @param delta Amount to change (+/- based on encoder direction)
         * @return The new clamped value
         */
        int adjustUIValue(int delta);

        /**
         * @brief Send a UI command to the display (UIPC, UIML, UIMN, etc.)
         * @param command The formatted UI command string
         */
        void sendUICommand(std::string_view command) const;

        /**
         * @brief Check and auto-dismiss UI if timed out
         * Should be called periodically (e.g., from button/encoder poll task)
         */
        void checkUITimeout();

    private:
        // Core components
        RadioState state_;
        ISerialChannel &radioSerial_;
        ISerialChannel &usbSerial_;
        ISerialChannel *displaySerial_ = nullptr; // Optional mirror sink for SET commands
        ISerialChannel *usbCdc1Serial_ = nullptr; // Optional second CDC output (CDC1)
        tcp_cat_bridge::TcpCatBridge *tcp0Bridge_ = nullptr; // Optional TCP port 0 bridge
        tcp_cat_bridge::TcpCatBridge *tcp1Bridge_ = nullptr; // Optional TCP port 1 bridge
        RadioMacroManager *macroManager_ = nullptr; // Optional macro manager for user macros

        // Command processing system (moved from Cat)
        std::unique_ptr<CommandDispatcher> commandDispatcher_;
        std::unique_ptr<CATHandler> localHandler_; // For USB commands
        std::unique_ptr<CATHandler> remoteHandler_; // For radio responses
        std::unique_ptr<CATHandler> panelHandler_; // For on-device panel/button commands (Panel source)
        std::unique_ptr<CATHandler> macroHandler_; // For internal macro commands (Macro source)
        mutable RtosRecursiveMutex dispatchMutex_; // Serializes CAT command dispatch (recursive allows nested macro execution)
        mutable RtosMutex radioTxMutex_; // Serializes direct UART writes

        // Antenna switching system
        std::unique_ptr<antenna::AntennaSwitch> antennaSwitch_;

        // Statistics and monitoring
        Statistics stats_;
        std::atomic<int64_t> previousMicros_{0};

        // Immediate polling callback
        void (*immediatePollingCallback_)() = nullptr;

        // Power state change callback
        void (*powerStateChangeCallback_)(bool powerOn, bool oldState) = nullptr;


        // TX timeout monitoring task
        TaskHandle_t txTimeoutTaskHandle_ = nullptr;
        static void txTimeoutTask(void *pvParameters);

        // Manual keepalive task for RRC-1258 compatibility
        TaskHandle_t keepaliveTaskHandle_ = nullptr;
        static void keepaliveTask(void *pvParameters);

        // Boot sequence task (non-blocking paced command sending)
        static void bootSequenceTask(void *pvParameters);

        // Boot sequence queries baseline radio state.
        // Note: PS/AI are NOT included - handled separately by InterfaceSystemCommandHandler.
        // Commands that may return ?; in certain modes are avoided or placed last.
        static constexpr size_t BOOT_SEQUENCE_SIZE = 22;
        static constexpr std::array<const char *, BOOT_SEQUENCE_SIZE> bootSequence_{
            // Core state: VFO, mode, frequencies
            "IF;", "FR;", "FT;", "MD;", "FA;", "FB;",
            // DSP/Filter settings
            "FL;", "SH;", "SL;",
            // TX/RX settings
            "PC;", "SQ0;", "XT;", "RT;",
            // Button states for panel sync
            "PA;", "RA;", "NB;", "NR;", "NT;", "VX;",
            // Optional (may fail in some modes)
            "GC;", "TO;", "SP;"};

        // Allocation-free radio send helpers
        void sendRadioCommand(std::string_view command) const;
        void sendRadioCommand(std::string_view part1, std::string_view part2) const;
        void sendRadioCommand(const char *command) const;

        // State validation and updates
        bool updateFrequency(uint64_t newFreq);
        bool updateMode(int8_t newMode);
        bool updateSplit(bool newSplit);
        bool updateTx(bool newTx);

        // Command system initialization (moved from Cat)
        void initializeCommandHandlers();

        // Utility methods
        static uint64_t getCurrentTimestamp();
        void checkTuningTimeout();

        // Constants
        static constexpr uint64_t PENDING_TIMEOUT_US = 5000000; // 5 seconds
        static constexpr const char *TAG = "RadioManager";

        // === Minimal origin-memory routing for query→answer pairing ===
    public:
        // Record the origin of a just-sent READ for a given 2-char prefix (e.g., "FA")
        void noteQueryOrigin(std::string_view prefix, CommandSource src, uint64_t nowUs);

        // Try to route a radio Answer based on the last recorded origin for this prefix
        // Returns true if routed to a specific interface; false if no recent origin found
        bool routeMatchedAnswer(std::string_view prefix, std::string_view response, uint64_t nowUs) const;

        // Overloaded version that returns the CommandSource that received the message
        // Returns std::nullopt if no recent origin found
        std::optional<CommandSource> routeMatchedAnswerWithSource(std::string_view prefix, std::string_view response,
                                                                  uint64_t nowUs) const;

    private:
        // Lightweight prefix→id interning for origin tracking
        uint8_t internOriginPrefix(const std::string &prefix);
        static constexpr size_t ORIGIN_TABLE_SIZE = 128;
        static constexpr uint64_t ORIGIN_TTL_US = 2000000; // 2s window to match answer to origin
        mutable std::unordered_map<std::string, uint8_t> originIds_{};
        mutable std::atomic<uint8_t> nextOriginId_{0};
        std::array<std::atomic<uint64_t>, ORIGIN_TABLE_SIZE> lastOriginTime_{}; // per prefix
        std::array<std::atomic<int>, ORIGIN_TABLE_SIZE> lastOriginSrc_{}; // stores int(CommandSource)

        // (sendToSource is public)
    };

} // namespace radio
