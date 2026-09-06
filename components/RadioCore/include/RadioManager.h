#pragma once

#include <array>
#include <atomic>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string_view>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "rtos_mutex.h"
#include "../../AntennaSwitch/include/AntennaSwitch.h"
#include "../../CommonConstants/include/radio_constants.h"
#include "CATHandler.h"
#include "CommandDispatcher.h"
#include "RadioState.h"
#include "ISerialChannel.h"
#include "PacedRadioChannel.h"

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

    // Outcome of a CAT dispatch attempt. Distinguishes a genuine "handler declined"
    // (Unhandled) from a dropped command due to dispatch-lock contention (LockTimeout),
    // which callers previously could not tell apart via the bool dispatchMessage() API.
    enum class DispatchOutcome
    {
        Handled,     // A handler consumed the command
        Unhandled,   // Parsed but no handler claimed it (forwarding may apply)
        LockTimeout  // Could not acquire dispatchMutex_ within the timeout; not processed
    };

    // Targets for atomic button toggles (M3): the read-current-and-invert is
    // resolved under dispatchMutex_ so a concurrent CAT set cannot make the press
    // a no-op or flip the wrong direction.
    enum class ToggleTarget
    {
        Processor,   // PR
        Attenuator,  // RA
        Preamp,      // PA
        Rit,         // RT
        Xit,         // XT
        Vox,         // VX
        TxAtu,       // AC (TX-AT in/thru; RX-AT cannot be set)
        Antenna,     // AN (main antenna P1 only; P2/P3 = 9 = no change)
    };

    // Targets for atomic multi-state button cycles (M3 follow-up): read-current-and-
    // advance is resolved under dispatchMutex_, same guarantee as ToggleTarget but for
    // settings that step through >2 states instead of flipping a boolean.
    enum class CycleTarget
    {
        NoiseReduction,      // NR: OFF -> NR1 -> NR2 -> OFF        (state_.noiseReductionMode)
        NoiseBlanker,        // NB: OFF -> NB1 -> NB2 -> NB3 -> OFF (state_.noiseBlanker)
        NoiseBlankerActive,  // NB: NB1 -> NB2 -> NB3 -> NB1 (skip OFF; used while the NB level popup is open)
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
        // Plain snapshot returned by value. Live counters are the atomics below;
        // read/set split is surfaced separately via the CAT parser statistics, and no
        // feedback-loop detector exists, so those fields were removed rather than left
        // reading permanent zeros.
        struct Statistics
        {
            uint32_t totalCommandsProcessed{0};
            uint32_t localCommandsProcessed{0};
            uint32_t remoteCommandsProcessed{0};
        };

        Statistics getStatistics() const
        {
            return Statistics{
                totalCommandsProcessed_.load(std::memory_order_relaxed),
                localCommandsProcessed_.load(std::memory_order_relaxed),
                remoteCommandsProcessed_.load(std::memory_order_relaxed)};
        }
        void resetStatistics()
        {
            totalCommandsProcessed_.store(0, std::memory_order_relaxed);
            localCommandsProcessed_.store(0, std::memory_order_relaxed);
            remoteCommandsProcessed_.store(0, std::memory_order_relaxed);
        }

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
         * @brief Set NvsManager pointer for boot-time EX menu save
         */
        void setNvsManager(void *nvsManager) { nvsManager_ = nvsManager; }

        /**
         * @brief Synchronize transverter-related menu settings with the radio on startup
         *
         * Runs inline (contains paced vTaskDelay); safe only when NOT called under the
         * dispatch lock (e.g. startup load-and-sync path).
         */
        void syncTransverterMenuSettings() const;

        /**
         * @brief Deferred variant of syncTransverterMenuSettings()
         *
         * Spawns a one-shot task to run the paced sync off the dispatch lock. Use this
         * from CAT dispatch context (e.g. PS1 answer handling) so the inter-command
         * delays never hold dispatchMutex_.
         */
        void syncTransverterMenuSettingsAsync() const;

        /**
         * @brief Send PS0; (power off) to the radio, paced, off the dispatch lock
         *
         * Spawns a one-shot task that repeats PS0; three times with 50 ms pacing (the
         * TS-590SG can drop the command during busy periods). Kept off the dispatch
         * lock so the retries/delays do not stall other interfaces' dispatch.
         */
        void sendPowerOffToRadioAsync() const;

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
         * Record the current receive band's desired ATU IN/THRU state and persist it.
         * Bands without a recorded preference default to THRU.
         */
        void setCurrentBandTunerEnabled(bool enabled);

        /** Apply the configured ATU state for the current receive band, if any. */
        void applyCurrentBandTunerSetting();

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
         * @brief Record button activity timestamp (used for tuning debounce)
         */
        void recordButtonActivity();

        /**
         * @brief Record encoder activity timestamp (used for tuning debounce)
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

        /**
         * @brief Undo the display dedup commit for a frame that failed to reach the display
         *
         * sendToDisplay() calls this itself on a failed send. Exposed publicly so callers
         * that write to displaySerial_ directly (e.g. main.cpp's radio_task, which mirrors
         * radio Answers to the display outside of sendToDisplay) can report the same
         * failure and keep dedup state consistent with what was actually delivered.
         * @param frames The frame that failed to be delivered to the display
         */
        void onDisplayForwardFailed(std::string_view frames) const;

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

        // Thread-safe CAT dispatch entry point used by all tasks/macros.
        // Retained for existing callers: returns true only when a handler consumed
        // the command (equivalent to dispatchMessageEx() == DispatchOutcome::Handled).
        bool dispatchMessage(CATHandler &handler, std::string_view message) const;

        // Richer dispatch entry point that distinguishes a lock timeout from an
        // unhandled command, so callers can surface a defined CAT error ("?;") or
        // retry instead of silently dropping the message.
        DispatchOutcome dispatchMessageEx(CATHandler &handler, std::string_view message) const;

        // Atomically toggle a boolean radio setting: reads the current cached value and
        // dispatches the inverted absolute command while holding dispatchMutex_, so the
        // read and dispatch are one critical section (fixes M3 TOCTOU). Returns the
        // dispatch outcome; LockTimeout if the lock could not be acquired in time.
        DispatchOutcome dispatchToggle(CATHandler &handler, ToggleTarget target) const;

        // Atomically advance a multi-state radio setting to its next value: reads the
        // current cached value and dispatches the advanced absolute command while holding
        // dispatchMutex_, so read+dispatch is one critical section (M3). Returns the
        // dispatch outcome; LockTimeout if the lock could not be acquired in time.
        DispatchOutcome dispatchCycleLocked(CATHandler &handler, CycleTarget target) const;

        /**
         * @brief Atomically advance the mutually-exclusive transmit processing mode.
         *
         * The cycle is OFF -> PROC -> DATA -> OFF.  PROC and DATA are never enabled
         * together; transitions that switch between them first disable the active mode.
         */
        DispatchOutcome cycleProcessorDataMode(CATHandler &handler) const;

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

        /**
         * @brief Apply the auto IF-filter policy (UIFF)
         *
         * When enabled, split is active (RX VFO != TX VFO) and the mode is CW, the IF filter
         * follows the RX VFO: VFO A -> FL1;, VFO B -> FL2;. When any precondition is no longer
         * met the policy simply stops adjusting - it never reverts to a default filter.
         * Called after mode/split/RX-VFO transitions and when the option is switched on.
         */
        void applyAutoFilterPolicy() const;

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

        // === STUCK-TUNING WATCHDOG ===

        /**
         * @brief Pure decision helper for the stuck-tuning watchdog (txTimeoutTask).
         *
         * isTuning is normally cleared by the RX/TX0/TX1 Answer paths or the IF P8=RX
         * branch, but a missed/raced Answer can leave it set forever, which makes
         * ForwardingPolicy's tuning-suppression logic permanently block IF/FA/FB
         * updates to the display. This watchdog is the backstop: if isTuning is set,
         * we are not actively transmitting, and no encoder activity has been seen for
         * over a second, the tune is considered abandoned.
         *
         * Static and pure (no I/O, no side effects) so it can be unit-tested directly
         * against a RadioState fixture without needing the 1s task tick to fire.
         *
         * @param state Radio state to evaluate (read-only)
         * @param nowUs Current timestamp (esp_timer_get_time()), passed in for testability
         * @return true if isTuning should be cleared
         */
        static bool shouldClearStuckTuning(const RadioState &state, uint64_t nowUs);

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
        void sendLocal(const std::string_view frames) const { (void)dispatchMessage(*localHandler_, frames); }
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
        // ISerialChannel adapter handed to the 4 CATHandlers below in place of
        // radioSerial_ directly, so that command handlers calling sendMessage()
        // on "the radio" (BaseCommandHandler::sendToRadio) go through the paced
        // TX queue (sendRadioCommand) instead of writing the UART inline. See
        // PacedRadioChannel.h.
        PacedRadioChannel pacedRadioSerial_;
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

        // Global radio-TX pacing queue (replaces the former mutex + inline-sleep gate).
        // The TS-590SG rejects (?;) commands that arrive faster than it can process, so
        // the aggregate wire rate to the radio must be bounded (commit f9ab2b6). The old
        // gate held a mutex across a pacing vTaskDelay; because callers hold the recursive
        // dispatchMutex_ (via dispatchMessageEx) while sending, that parked the global
        // dispatch lock on a sleep and stalled radio_task's answer processing — and thus
        // the remote display. Now sendRadioCommand only validates and enqueues (never
        // blocks), and a single dedicated drainer task (radioTxDrainTask) owns the min-gap
        // pacing and the UART write, off every lock. That keeps the aggregate rate bound
        // while restoring the no-sleep-under-lock invariant (commit 0ac2d8d).
        //
        // CAT frames are short; 64 matches SerialHandler::MAX_MESSAGE_LENGTH (the wire cap).
        static constexpr size_t RADIO_TX_CMD_MAX = 64;
        struct RadioTxItem
        {
            char data[RADIO_TX_CMD_MAX];
            uint8_t len;
            uint64_t enqueueUs; // esp_timer at enqueue; drainer reports enqueue->wire latency
        };
        // Depth 64: at the 10 ms min gap (~100 cmds/s drain) this absorbs ~640 ms of burst
        // backlog (e.g. the boot / EX-menu sync) before it must shed load. Doubled from the
        // original 32 once PacedRadioChannel (see above) started routing every CAT handler's
        // radio TX -- not just RadioManager's own internal sends -- through this one queue,
        // widening the set of producers that can burst concurrently (panel + USB/TCP clients
        // + boot sequence all landing here now instead of writing the UART inline).
        static constexpr size_t RADIO_TX_QUEUE_DEPTH = 64;
        QueueHandle_t radioTxQueue_ = nullptr;
        StaticQueue_t radioTxQueueControl_{};                                   // static queue control block
        uint8_t radioTxQueueStorage_[RADIO_TX_QUEUE_DEPTH * sizeof(RadioTxItem)]{}; // static item storage
        TaskHandle_t radioTxDrainTaskHandle_ = nullptr;
        static void radioTxDrainTask(void *pvParameters);
        // Owned EXCLUSIVELY by radioTxDrainTask once tasks start, so it needs no lock.
        mutable uint64_t lastRadioTxUs_ = 0;
        static constexpr uint64_t RADIO_TX_MIN_GAP_US = 10'000; // 10 ms between sends

        // Antenna switching system
        std::unique_ptr<antenna::AntennaSwitch> antennaSwitch_;

        // Statistics and monitoring. mutable: incremented from the const dispatch path
        // (dispatchMessageEx), which runs on many tasks, so the counters are atomic.
        mutable std::atomic<uint32_t> totalCommandsProcessed_{0};
        mutable std::atomic<uint32_t> localCommandsProcessed_{0};
        mutable std::atomic<uint32_t> remoteCommandsProcessed_{0};

        // Power state change callback
        void (*powerStateChangeCallback_)(bool powerOn, bool oldState) = nullptr;

        // NvsManager pointer for boot-time EX menu save (void* to avoid circular include)
        void *nvsManager_ = nullptr;


        // TX timeout monitoring task
        TaskHandle_t txTimeoutTaskHandle_ = nullptr;
        static void txTimeoutTask(void *pvParameters);

        // Manual keepalive task for RRC-1258 compatibility
        TaskHandle_t keepaliveTaskHandle_ = nullptr;
        static void keepaliveTask(void *pvParameters);

        // Boot sequence task (non-blocking paced command sending)
        static void bootSequenceTask(void *pvParameters);
        // Re-arm guard: true while bootSequenceTask is in flight. Prevents
        // performBootSequence() from spawning a second overlapping instance
        // (e.g. a rapid power-cycle or reconnect) that would race the first
        // task's paced sends against its own. compare_exchange false->true
        // gates task creation in performBootSequence(); the task clears it as
        // its last action before self-deleting, and performBootSequence()
        // also clears it immediately if xTaskCreate itself fails.
        mutable std::atomic<bool> bootSequenceRunning_{false};

        // One-shot tasks that run paced radio I/O off the dispatch lock
        static void powerOffTask(void *pvParameters);
        static void transverterSyncTask(void *pvParameters);
        // Re-arm guard for transverterSyncTask; same pattern/rationale as
        // bootSequenceRunning_ above.
        mutable std::atomic<bool> transverterSyncRunning_{false};

        // Boot sequence queries baseline radio state.
        // Note: PS/AI are NOT included - handled separately by InterfaceSystemCommandHandler.
        // Commands that may return ?; in certain modes are avoided or placed last.
        static constexpr size_t BOOT_SEQUENCE_SIZE = 24;
        static constexpr std::array<const char *, BOOT_SEQUENCE_SIZE> bootSequence_{
            // Core state: VFO, mode, frequencies
            "IF;", "FR;", "FT;", "MD;", "FA;", "FB;",
            // DSP/Filter settings
            "FL;", "SH;", "SL;",
            // TX/RX settings
            "PC;", "SQ0;", "XT;", "RT;",
            // Gain knobs: panel encoders accumulate from cached state, so RF/AF
            // gain must be primed from the radio or the first click jumps to ~0.
            "AG0;", "RG;",
            // Button states for panel sync
            "PA;", "RA;", "NB;", "NR;", "NT;", "VX;",
            // Optional (may fail in some modes)
            "GC;", "TO;", "SP;"};

        // Phase 2: additional common commands that TS-590G Programmer queries
        static constexpr size_t BOOT_PHASE2_CMD_COUNT = 13;
        static constexpr std::array<const char *, BOOT_PHASE2_CMD_COUNT> bootPhase2Commands_{
            "FS;", "GT;", "KS;", "LK;", "MG;", "ML;",
            "NL;", "PL;", "PR;", "RL;", "SD;", "VD;", "VG;"};

        static constexpr size_t EX_MENU_COUNT = 100;

        // PacedRadioChannel::sendMessage() forwards into sendRadioCommand() so that
        // command handlers writing through it are paced like every other radio TX.
        friend class PacedRadioChannel;

        // Allocation-free radio send helpers. Return true when the command was
        // handed off (enqueued in production, or sent inline in unit-test
        // builds), false when validation rejected it or the paced queue was
        // full. Existing callers use these in void context and are unaffected.
        bool sendRadioCommand(std::string_view command) const;
        bool sendRadioCommand(const char *command) const;

        // Shared validation for sendRadioCommand/sendUrgentRadioCommand: rejects
        // empty commands, commands too long for RadioTxItem, and embedded
        // control characters.
        bool validateRadioCommand(std::string_view command) const;

        // State validation and updates
        bool updateFrequency(uint64_t newFreq);
        bool updateMode(int8_t newMode);
        bool updateSplit(bool newSplit);
        bool updateTx(bool newTx);

        static int bandFromFrequency(uint64_t frequencyHz);
        void persistTunerBandSettings() const;

        // Command system initialization (moved from Cat)
        void initializeCommandHandlers();

        // Utility methods
        static uint64_t getCurrentTimestamp();

        // Constants
        static constexpr uint64_t PENDING_TIMEOUT_US = 5000000; // 5 seconds
        // Stuck-tuning watchdog threshold (see shouldClearStuckTuning). Checked on
        // txTimeoutTask's 1s tick, so this is also the worst-case detection latency.
        static constexpr uint64_t STUCK_TUNING_TIMEOUT_US = 1'000'000; // 1 second
        static constexpr const char *TAG = "RadioManager";

        // === Minimal origin-memory routing for query→answer pairing ===
    public:
        // Urgent radio send for safety-critical commands (e.g. "RX;" forced by
        // TransmitterCommandHandler::handleRX or the TX watchdog). Jumps the
        // paced TX queue via xQueueSendToFront instead of the back, and if the
        // queue is saturated, falls back to writing directly to the radio
        // rather than silently dropping like sendRadioCommand does. Returns
        // true only if the command was actually handed to the queue or
        // written directly and accepted by the serial channel -- callers that
        // must not proceed on failure (e.g. handleRX keeping TX ownership)
        // rely on this being a true delivery signal, not a fire-and-forget one.
        bool sendUrgentRadioCommand(std::string_view command) const;

        // Record the origin of a just-sent READ for a given 2-char prefix (e.g., "FA")
        // When cacheServed=true, the origin is recorded but routeMatchedAnswerWithSource
        // will return the source WITHOUT sending (to suppress AI forwarding duplicates
        // when a cached response was already delivered to the client).
        void noteQueryOrigin(std::string_view prefix, CommandSource src, uint64_t nowUs, bool cacheServed = false);

        // Try to route a radio Answer based on the last recorded origin for this prefix
        // Returns true if routed to a specific interface; false if no recent origin found
        bool routeMatchedAnswer(std::string_view prefix, std::string_view response, uint64_t nowUs) const;

        // Overloaded version that returns the CommandSource that received the message
        // Returns std::nullopt if no recent origin found
        std::optional<CommandSource> routeMatchedAnswerWithSource(std::string_view prefix, std::string_view response,
                                                                  uint64_t nowUs) const;

    private:
        // Lightweight prefix→slot mapping for origin tracking using compact hash
        // 2-char CAT prefixes hash directly to a fixed array (no heap allocation)
        static constexpr size_t ORIGIN_TABLE_SIZE = 128;
        static constexpr uint64_t ORIGIN_TTL_US = 2000000; // 2s window to match answer to origin
        static constexpr int ORIGIN_CACHE_SERVED_BIT = 0x100; // Flag: cached response already sent to client
        static constexpr uint8_t originHash(char c1, char c2) {
            // Simple hash of 2-char prefix into ORIGIN_TABLE_SIZE slots
            return static_cast<uint8_t>(((static_cast<unsigned>(c1) * 31) + static_cast<unsigned>(c2)) % ORIGIN_TABLE_SIZE);
        }
        static constexpr uint16_t originPrefix(char c1, char c2) {
            return static_cast<uint16_t>((static_cast<uint16_t>(static_cast<uint8_t>(c1)) << 8) |
                                         static_cast<uint8_t>(c2));
        }
        std::array<std::atomic<uint64_t>, ORIGIN_TABLE_SIZE> lastOriginTime_{}; // per prefix
        std::array<std::atomic<int>, ORIGIN_TABLE_SIZE> lastOriginSrc_{}; // stores int(CommandSource)
        std::array<std::atomic<uint16_t>, ORIGIN_TABLE_SIZE> lastOriginPrefix_{}; // exact prefix, validates hash slot

        // (sendToSource is public)
    };

} // namespace radio
