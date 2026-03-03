#pragma once

#include <array>
#include <atomic>
#include <string>
#include "rtos_mutex.h"

// Forward declaration for CommandSource with matching underlying type
namespace radio
{
    enum class CommandSource : uint8_t;
}

namespace radio
{

    /**
     * @brief Represents a single channel in the Auto Mode memory map
     */
    struct AutoModeChannel
    {
        std::atomic<uint64_t> frequency{0};
        std::atomic<int8_t> mode{-1};
        std::atomic<bool> dataMode{false};
    };

    /**
     * @brief High-performance unified timestamp tracker for CAT commands
     *
     * Uses string interning to map command strings to array indices for O(1)
     * timestamp access. Replaces both CommandTimestamps and SentQueryTracker
     * with a single, optimized implementation.
     */
    class CommandTimestampTracker
    {
    private:
        static constexpr size_t PRIMARY_BUCKET_SIZE = 48;
        static constexpr size_t TIMESTAMP_TABLE_SIZE = PRIMARY_BUCKET_SIZE * PRIMARY_BUCKET_SIZE;
        static size_t bucketIndex(char ch)
        {
            if (ch >= 'A' && ch <= 'Z')
            {
                return static_cast<size_t>(ch - 'A');
            }
            if (ch >= '0' && ch <= '9')
            {
                return 26u + static_cast<size_t>(ch - '0');
            }
            if (ch >= 'a' && ch <= 'z')
            {
                return static_cast<size_t>(ch - 'a');
            }
            switch (ch)
            {
            case ';':
                return 42u;
            case '?':
                return 43u;
            case '!':
                return 44u;
            case '#':
                return 45u;
            default:
                return 47u;
            }
        }

        static size_t computeTableIndex(const std::string &command)
        {
            if (command.empty())
            {
                return TIMESTAMP_TABLE_SIZE - 1;
            }

            const char first = command[0];
            const size_t firstIdx = bucketIndex(first);

            if (command.size() == 1)
            {
                return (firstIdx * PRIMARY_BUCKET_SIZE) % TIMESTAMP_TABLE_SIZE;
            }

            const char second = command[1];
            const size_t secondIdx = bucketIndex(second);
            size_t combined = (firstIdx * PRIMARY_BUCKET_SIZE) + secondIdx;

            // For longer keys (EX056, VS0, etc.), incorporate up to 5 chars total
            // This prevents collisions for EX menu commands and VS visual scan
            constexpr size_t MAX_HASH_CHARS = 5;
            const size_t hashLen = std::min(command.size(), MAX_HASH_CHARS);
            for (size_t i = 2; i < hashLen; ++i)
            {
                combined = combined * 31 + bucketIndex(command[i]);
            }

            return combined % TIMESTAMP_TABLE_SIZE;
        }

        // Fast array-based timestamp storage shared for all CAT prefixes
        mutable std::array<std::atomic<uint64_t>, TIMESTAMP_TABLE_SIZE> timestamps_{};

    public:
        /**
         * @brief Record timestamp for a command
         * @param command CAT command prefix (e.g., "FA", "FB", "MD")
         * @param timestamp Timestamp in microseconds (from esp_timer_get_time)
         */
        void record(const std::string &command, const uint64_t timestamp) const
        {
            const size_t id = computeTableIndex(command);
            timestamps_[id].store(timestamp, std::memory_order_relaxed);
        }

        /**
         * @brief Get timestamp for a command
         * @param command CAT command prefix
         * @return Last update timestamp, or 0 if never updated
         */
        uint64_t get(const std::string &command) const
        {
            const size_t id = computeTableIndex(command);
            return timestamps_[id].load(std::memory_order_relaxed);
        }

        /**
         * @brief Check if cached data for a command is fresh
         * @param command CAT command prefix
         * @param currentTime Current timestamp in microseconds
         * @param ttlUs TTL in microseconds
         * @return true if cache is fresh (within TTL)
         */
        bool isFresh(const std::string &command, const uint64_t currentTime, const uint64_t ttlUs) const
        {
            const uint64_t lastUpdate = get(command);
            return lastUpdate > 0 && currentTime - lastUpdate < ttlUs;
        }

        /**
         * @brief Clear all cached timestamps
         * Used primarily for testing to ensure clean cache state
         */
        void clear() const
        {
            for (auto &timestamp : timestamps_)
            {
                timestamp.store(0, std::memory_order_relaxed);
            }
        }

        /**
         * @brief Invalidate a specific command's cache entry
         * Forces next query for this command to be forwarded to radio
         * @param command CAT command prefix (e.g., "FA", "FB", "IF")
         */
        void invalidate(const std::string &command) const
        {
            const size_t id = computeTableIndex(command);
            timestamps_[id].store(0, std::memory_order_relaxed);
        }

        // Compatibility methods for existing code

        /**
         * @brief Alias for record() - maintains CommandTimestamps API
         */
        void update(const std::string &command, const uint64_t timestamp) const { record(command, timestamp); }

        /**
         * @brief Alias for record() - maintains SentQueryTracker API
         */
        void recordQuery(const std::string &command, const uint64_t timestamp) const { record(command, timestamp); }

        /**
         * @brief Check if USB recently queried this command type
         * Maintains SentQueryTracker API with 5s default TTL
         * @param command CAT command prefix
         * @param currentTime Current timestamp in microseconds
         * @param ttlUs TTL in microseconds (default: 5 seconds)
         * @return true if USB queried this command within TTL
         */
        bool wasRecentlyQueried(const std::string &command, const uint64_t currentTime,
                                const uint64_t ttlUs = 5000000) const
        {
            return isFresh(command, currentTime, ttlUs);
        }
    };

    // Compatibility aliases for existing code
    using CommandTimestamps = CommandTimestampTracker;
    using SentQueryTracker = CommandTimestampTracker;

    /**
     * @brief UI control types for panel-display interaction
     * These define which parameter is being adjusted via the UI overlay
     */
    enum class UIControl : uint8_t
    {
        None = 0,      // No UI active
        Power = 1,     // PC - RF output power (5-100W)
        CarrierLevel = 2, // ML - TX Monitor/Carrier level (0-20)
        AfGain = 3,    // AG - AF gain (0-255)
        RfGain = 4,    // RG - RF gain (0-255)
        MicGain = 5,   // MG - Microphone gain (0-100)
        NrLevel = 6,   // RL - NR1 level (1-10)
        NbLevel = 7,   // NL - Noise Blanker level (1-10)
        Nr2Speed = 8,  // RL - NR2 SPAC speed (0-9, 2-20ms)
        ProcInputLevel = 9,  // PL - Speech processor input level (0-100)
        ProcOutputLevel = 10, // PL - Speech processor output level (0-100)
        NotchFrequency = 11, // BP - Manual notch frequency (0-127)
        IfShift = 12,  // IS - IF shift (0-9999 Hz, CW/CW-R only)
        RitXitOffset = 13, // RU/RD - RIT/XIT offset (-9999 to +9999 Hz)
        DataMode = 14, // DA - Data mode ON/OFF toggle (0=OFF, 1=ON)
    };

    /**
     * @brief UI state for panel-display overlay interactions
     * Tracks when a UI adjustment mode is active (e.g., power slider)
     */
    struct UIState
    {
        std::atomic<uint8_t> activeControl{0};  // UIControl enum value
        std::atomic<int16_t> currentValue{0};   // Current displayed value
        std::atomic<int16_t> minValue{0};       // Minimum allowed value
        std::atomic<int16_t> maxValue{100};     // Maximum allowed value
        std::atomic<int16_t> stepSize{1};       // Step size for encoder adjustments
        std::atomic<uint64_t> activationTime{0}; // When UI mode was entered
        std::atomic<uint64_t> lastUpdateTime{0}; // Last encoder/value update
        std::atomic<uint64_t> timeoutUs{UI_TIMEOUT_DEFAULT_US}; // Configurable timeout

        // UI timeout defaults
        static constexpr uint64_t UI_TIMEOUT_DEFAULT_US = 5000000; // 5 seconds (default)
        static constexpr uint64_t UI_TIMEOUT_SHORT_US = 2000000;   // 2 seconds (for transient displays like IF shift)

        bool isActive() const { return activeControl.load(std::memory_order_relaxed) != static_cast<uint8_t>(UIControl::None); }

        UIControl getActiveControl() const { return static_cast<UIControl>(activeControl.load(std::memory_order_relaxed)); }

        bool isTimedOut(uint64_t currentTime) const {
            if (!isActive()) return false;
            uint64_t lastUpdate = lastUpdateTime.load(std::memory_order_relaxed);
            uint64_t timeout = timeoutUs.load(std::memory_order_relaxed);
            return lastUpdate > 0 && (currentTime - lastUpdate) > timeout;
        }

        void clear() {
            activeControl.store(static_cast<uint8_t>(UIControl::None), std::memory_order_relaxed);
            currentValue.store(0, std::memory_order_relaxed);
            activationTime.store(0, std::memory_order_relaxed);
            lastUpdateTime.store(0, std::memory_order_relaxed);
            timeoutUs.store(UI_TIMEOUT_DEFAULT_US, std::memory_order_relaxed);
        }
    };

    /**
     * @brief Single authoritative data model for all radio state
     *
     * Optimized layout:
     * - Cache-line aligned for hot-path fields (frequencies, mode, TX state)
     * - Grouped by access pattern to minimize cache misses
     * - Packed to minimize padding (8-byte atomics first, then smaller types)
     * - Non-atomic for fields that don't require thread-safety
     **/

    struct alignas(64) RadioState // Align to cache line boundary (64 bytes on ESP32-S3)
    {
        // === HOT PATH: Cache Line 0 (frequently accessed during command processing) ===
        // Group most-accessed fields together for optimal cache utilization

        // VFO frequencies (in Hz) - accessed on every FA/FB/IF command (~40% of traffic)
        std::atomic<uint64_t> vfoAFrequency{0}; // Offset 0
        std::atomic<uint64_t> vfoBFrequency{0}; // Offset 8

        // Timestamps for TTL caching (accessed frequently)
        std::atomic<uint64_t> lastVfoAUpdate{0}; // Offset 16
        std::atomic<uint64_t> lastVfoBUpdate{0}; // Offset 24

        // Mode and TX state (accessed on every MD/TX/RX command)
        std::atomic<int8_t> mode{-1}; // Offset 32 - Operating mode (LSB, USB, CW, etc.)
        std::atomic<int8_t> dataMode{0}; // Offset 33 - Data mode setting
        std::atomic<uint8_t> currentRxVfo{0}; // Offset 34 - Current receive VFO (FR command)
        std::atomic<uint8_t> currentTxVfo{0}; // Offset 35 - Current transmit VFO (FT command)
        std::atomic<uint8_t> splitSetting{0}; // Offset 36 - Split setting status (0=off, 1=in-progress)
        std::atomic<bool> split{false}; // Offset 37 - Split operation status
        std::atomic<bool> isTx{false}; // Offset 38 - Transmit status
        std::atomic<bool> powerOn{false}; // Offset 39 - Radio power state
        std::atomic<bool> powerStateEstablished{false}; // Has PS answer been received from radio?

        // Pending state flags (accessed during command dispatch)
        std::atomic<bool> vfoAFrequencyPending{false}; // Offset 40
        std::atomic<bool> vfoBFrequencyPending{false}; // Offset 41
        std::atomic<bool> modePending{false}; // Offset 42
        std::atomic<bool> splitPending{false}; // Offset 43
        std::atomic<bool> txPending{false}; // Offset 44
        std::atomic<bool> isTuning{false}; // Offset 45 - Currently tuning flag
        std::atomic<bool> radioConnected{false}; // Offset 46 - Physical radio connection status
        uint8_t _padding0{0}; // Offset 47 - Explicit padding for alignment

        std::atomic<uint64_t> vfoAPendingTime{0}; // Offset 48
        std::atomic<uint64_t> vfoBPendingTime{0}; // Offset 56
        // End of Cache Line 0 (64 bytes)

        // === Cache Line 1: Continuation of hot-path state ===
        std::atomic<uint64_t> vfoAPendingValue{0}; // Offset 64
        std::atomic<uint64_t> vfoBPendingValue{0}; // Offset 72
        std::atomic<uint64_t> tuningStartTime{0}; // Offset 80
        std::atomic<uint64_t> tuningStopTime{0}; // Offset 88 - When tuning ended (for grace period)
        std::atomic<uint64_t> lastButtonActivityTime{0}; // Offset 96
        std::atomic<uint64_t> lastEncoderActivityTime{0}; // Offset 104
        mutable std::atomic<uint64_t> lastUipsSendTime{0}; // UIPS1 debounce for display activity signaling
        std::atomic<bool> displayAwake{true}; // Display screensaver state (true = awake, false = asleep)
        std::atomic<bool> displayCommunicationEnabled{true}; // Display communication enable/disable (UIDE command)
        std::atomic<uint8_t> displayScreensaverTimeoutMin{15}; // Screensaver timeout in minutes (0 = disabled)
        std::atomic<uint64_t> lastModeUpdate{0}; // Offset 120
        std::atomic<uint64_t> lastSplitUpdate{0}; // Offset 120
        // End of Cache Line 1 (128 bytes)

        // === Cache Line 2: AI modes and RIT/XIT ===
        std::atomic<uint8_t> aiMode{0}; // Offset 128 - AI mode setting from radio (0, 1, or 2)

        // Per-CDC independent AI modes (grouped for cache efficiency)
        std::atomic<uint8_t> usbCdc0AiMode{0}; // Offset 129 - USB CDC0 AI mode (0,1,2,4 per Kenwood variants)
        std::atomic<uint8_t> usbCdc1AiMode{0}; // Offset 130 - USB CDC1 AI mode (0,1,2,4 per Kenwood variants)
        std::atomic<uint8_t> tcp0AiMode{0};    // Offset 131 - TCP port 0 AI mode (0,1,2,4 per Kenwood variants)
        std::atomic<uint8_t> tcp1AiMode{0};    // Offset 132 - TCP port 1 AI mode (0,1,2,4 per Kenwood variants)
        std::atomic<uint8_t> displayAiMode{2}; // Offset 133 - Display AI mode (default AI2 for rapid updates)
        std::atomic<bool> ritOn{false}; // Offset 134 - RIT enabled
        std::atomic<bool> xitOn{false}; // Offset 135 - XIT enabled
        std::atomic<bool> keepAlive{false}; // Offset 136 - Keep alive status
        uint8_t _padding1{0}; // Offset 137 - Padding to 8-byte boundary

        // Power-off debounce: timestamp when PS0 was requested locally
        // Used to ignore PS1 responses during radio power-down sequence (~2s window)
        std::atomic<uint64_t> powerOffRequestTime{0};

        // AI coordination control
        std::atomic<uint64_t> lastAiCoordinationTime{0}; // Timestamp of last AI coordination (for debouncing)
        std::atomic<bool> macroInProgress{false}; // Flag indicating macro execution in progress

        // TX control and offset values (continuation of Cache Line 2)
        std::atomic<int> txOwner{-1}; // Offset 136 - CommandSource that owns TX (-1 = none)
        std::atomic<int> ifShiftValue{0}; // IF shift value
        std::atomic<int32_t> ritXitOffset{0}; // RIT/XIT offset in Hz
        uint32_t _padding2{0}; // Padding
        std::atomic<uint64_t> txActivationTime{0}; // When TX was activated (microseconds)
        mutable RtosMutex txMutex; // Mutex for TX state changes
        std::atomic<int> controlLeaseOwner{-1}; // Control lease owner (-1 = none)
        std::atomic<uint64_t> controlLeaseExpiry{0}; // Control lease expiry time
        std::atomic<int> controlLeasePriority{-1}; // Priority of current lease owner
        mutable RtosMutex controlLeaseMutex; // Mutex for lease arbitration

        // === SHARED STATE: Accessed from multiple tasks (Button/Encoder/Macro/Dispatch) ===
        // These fields MUST be atomic to prevent data races and torn reads

        // Band and antenna state (accessed by ButtonHandler, EncoderHandler, MacroManager)
        std::atomic<int> bandNumber{0}; // Current band number
        int bandDownSlotIndex{0}; // Band down slot index for BD command
        int bandUpSlotIndex{0}; // Band up slot index for BU command
        uint8_t mainAntenna{0}; // Main antenna (0=ANT1, 1=ANT2)
        std::atomic<bool> transverter{false}; // Transverter mode
        std::atomic<bool> transverterOffsetEnabled{false}; // Controls whether to translate FA/FB/IF for display/USB using XO offset
        std::atomic<bool> transverterOffsetPlus{true}; // Transverter offset direction (true=plus, false=minus)
        std::atomic<uint64_t> transverterOffsetHz{0}; // Transverter offset frequency in Hz

        // Transverter-related menu settings (accessed by MacroManager for state comparison)
        int drvConnectorMode{0}; // EX085: DRV connector output function (0=DRO, 1=ANT)
        std::atomic<int> hfLinearAmpControl{0}; // EX059: HF linear amplifier control (should be 3 for transverter)
        std::atomic<int> vhfLinearAmpControl{0}; // EX060: 50 MHz linear amplifier control (should be 3 for transverter)
        std::atomic<bool> rxAnt{false}; // RX antenna setting
        std::atomic<bool> drvOut{false}; // Drive output setting
        bool attenuator{false}; // Attenuator on/off
        bool rxAtIn{false}; // RX ATU in/thru
        bool txAtIn{false}; // TX ATU in/thru
        bool atTuning{false}; // ATU tuning active

        // Memory and channel (atomic for thread safety)
        std::atomic<uint16_t> memoryChannel{0}; // Current memory channel
        std::atomic<uint8_t> scanStatus{0}; // Scan status

        // Audio and RF settings (accessed during gain adjustments)
        std::atomic<int> afGain{0}; // AF gain level
        std::atomic<int> rfGain{0}; // RF gain level
        std::atomic<bool> isAdjustingRfGain{false}; // Currently adjusting RF gain via potentiometer
        std::atomic<bool> isAdjustingAfGain{false}; // Currently adjusting AF gain via potentiometer
        std::atomic<uint64_t> rfGainAdjustTime{0}; // When RF gain adjustment started
        std::atomic<uint64_t> afGainAdjustTime{0}; // When AF gain adjustment started

        // Other audio/processing settings (infrequent access, no atomic needed)
        int sideTone{0}; // Side tone setting
        int processor{0}; // Processor setting
        int dataFilter{0}; // Data filter setting
        int microphoneGain{0}; // Microphone gain
        int txMonitorLevel{0}; // TX Monitor level
        int voxGain{0}; // VOX gain level
        int speechProcessorInLevel{0}; // Speech Processor input level
        int speechProcessorOutLevel{0}; // Speech Processor output level

        // Tone and squelch settings (infrequent)
        uint8_t toneStatus{0}; // Tone status
        uint8_t toneFrequency{0}; // Tone frequency
        int8_t beatCancelMode{0}; // Beat Cancel mode (0=OFF, 1=BC1, 2=BC2)
        int manualNotchFrequency{0}; // Manual notch frequency
        int toneState{0}; // Tone state

        // Flags (some need atomic for cross-task access)
        bool busy{false}; // Busy status
        bool cwTune{false}; // CW Tune status
        bool morseDecoder{false}; // Morse code decoder status
        bool fineTune{false}; // Fine Tune status
        bool filterCutSelectHighNotLow{true}; // Toggle for filter cut adjustment (true=high cut, false=low cut)
        std::atomic<bool> panelLock{false}; // Panel lock status (accessed by ButtonHandler)
        bool preAmplifier{false}; // Pre-amplifier
        bool tfSet{false}; // TF-Set status
        bool voxEnabled{false}; // VOX enabled status
        bool quickMemoryEnabled{false}; // Quick memory enabled
        uint8_t quickMemoryChannel{0}; // Quick memory channel 0-9

        // Various settings (infrequent access)
        int morseDecoderThreshold{0}; // Morse code decoder threshold
        int carrierLevel{0}; // Carrier level
        int txEqualizer{0}; // TX Equalizer
        int rxEqualizer{0}; // RX Equalizer
        int ifFilter{0}; // IF Filter
        int dspFilterBandwidth{0}; // DSP filter bandwidth (FW command for CW/FSK/FM)
        int fmNarrowMode{0}; // FM narrow mode (0=Normal, 1=Narrow)
        int agcMode{0}; // AGC mode (0=OFF, 1=Slow, 2=Fast, 3=Restore)
        int previousAgcMode{2}; // Previous non-OFF AGC mode for GC3 restore (default: Fast)
        int agcTimeConstant{0}; // AGC time constant
        int keyingSpeed{0}; // Keying speed
        int vgs1Target{0}; // VGS-1 target
        int vgs1Status{0}; // VGS-1 status
        int vgs1Time{0}; // VGS-1 time
        int noiseBlanker{0}; // Noise Blanker
        int noiseBlankerLevel{0}; // Noise Blanker level
        int noiseReductionMode{0}; // Noise Reduction mode (0=OFF, 1=NR1, 2=NR2)
        int nr1Level{5};           // NR1 level (1-10, default 5)
        int nr2Speed{5};           // NR2 SPAC speed (0-9, default 5 = 12ms)
        int notchFilterMode{0}; // Notch Filter mode
        int notchFilterBandwidth{0}; // Notch Filter bandwidth
        int playbackChannel{0}; // Playback channel
        std::array<int, 3> playbackQueue{}; // Playback queue (removed atomic - not thread-critical)
        int transmitPower{5}; // Transmit power (default 5W, minimum legal value)
        uint64_t autoModeFrequency{0}; // Auto mode frequency (removed atomic)
        int meterFunction{1}; // Meter function (1=SWR, 2=COMP, 3=ALC)
        int meterSmRaw{0}; // S-meter reading from SM command (0-30, or 0000-9999 raw format)
        int meterSwr{0}; // SWR meter value
        int meterComp{0}; // COMP meter value
        int meterAlc{0}; // ALC meter value
        int cwBreakInDelay{0}; // CW break-in delay
        int receiveHighCut{0}; // Receive high-cut
        int receiveLowCut{0}; // Receive low-cut
        int txTunePower{0}; // TX Tune power
        int menuBank{0}; // Menu bank (0=A, 1=B)
        int modeKey{0}; // Mode key
        int visualScanStatus{0}; // Visual Scan status (0=OFF, 1=ON, 2=Pause)

        std::array<char, 16> firmwareVersion{}; // Firmware version (fixed-size, e.g., "1.09")

        // Auto Mode channels (large array - keep at end)
        std::array<AutoModeChannel, 32> autoModeChannels;

        // Pending tracking for read-only commands (pack bools together)
        std::atomic<bool> meterPending{false}; // Meter pending
        std::atomic<bool> transmitInfoPending{false}; // Transmit info pending
        std::atomic<bool> idPending{false}; // ID command
        std::atomic<bool> fvPending{false}; // FV firmware version
        std::atomic<bool> ifPending{false}; // IF information function
        std::atomic<bool> mrPending{false}; // MR memory read
        std::atomic<bool> riPending{false}; // RI receive info
        std::atomic<bool> tyPending{false}; // TY radio type
        std::atomic<bool> tcPending{false}; // TC transceiver control
        std::atomic<bool> mePending{false}; // ME message status
        std::atomic<bool> aiPending{false}; // AI auto information mode
        std::atomic<bool> usbIdSent{false}; // Sent ID020; to USB after wakeup
        std::atomic<bool> needsImmediatePoll{false}; // Flag to trigger immediate polling

        // Generic command timestamp cache for TTL-based query optimization
        mutable CommandTimestamps commandCache;

        // USB query tracking for CAT protocol compliance
        mutable SentQueryTracker queryTracker;

        // Interface forwarding state (keep atomics for thread safety)
        struct InterfaceForwardState
        {
            std::atomic<uint64_t> lastFAMessage{0};
            std::atomic<uint64_t> lastFBMessage{0};
            std::atomic<uint64_t> lastIFMessage{0};
            std::atomic<int> lastSMValue{0};
            std::atomic<uint64_t> lastSMTime{0};
            std::atomic<uint64_t> lastForwardTime{0};
            std::atomic<bool> lastForwardedWasSM{false};
            // RM meter deduplication: track last value per meter type (1=SWR, 2=COMP, 3=ALC)
            std::atomic<int> lastRM1Value{-1}; // SWR
            std::atomic<int> lastRM2Value{-1}; // COMP/PWR
            std::atomic<int> lastRM3Value{-1}; // ALC

            // Per-interface query tracking for AI0 isolation.
            // Only queries from THIS interface are recorded here, so AI0 mode
            // can distinguish "I queried IF" from "the display queried IF".
            mutable CommandTimestampTracker localQueryTracker;
        };

        mutable InterfaceForwardState usb0ForwardState;
        mutable InterfaceForwardState usb1ForwardState;
        mutable InterfaceForwardState tcp0ForwardState;
        mutable InterfaceForwardState tcp1ForwardState;
        mutable InterfaceForwardState displayForwardState;

        InterfaceForwardState &accessForwardState(CommandSource sink) const;

        // UI state for panel-display overlay interactions (power slider, etc.)
        mutable UIState uiState;

        /**
         * @brief Check if a frequency value is valid
         */
        static bool isValidFrequency(const uint64_t freq)
        {
            return freq >= 30000 && freq <= 450000000; // 30kHz to 450MHz
        }

        /**
         * @brief Check if mode value is valid
         */
        static bool isValidMode(const int8_t mode)
        {
            return mode >= 0 && mode <= 9; // Typical mode range for most radios
        }

        /**
         * @brief Get a snapshot of current VFO A frequency with pending status
         */
        struct FrequencyState
        {
            uint64_t frequency;
            bool isPending;
            uint64_t lastUpdate;
        };

        FrequencyState getVfoAState() const
        {
            // Use relaxed ordering - these are independent reads for display purposes
            return {vfoAFrequency.load(std::memory_order_relaxed),
                    vfoAFrequencyPending.load(std::memory_order_relaxed),
                    lastVfoAUpdate.load(std::memory_order_relaxed)};
        }

        FrequencyState getVfoBState() const
        {
            // Use relaxed ordering - these are independent reads for display purposes
            return {vfoBFrequency.load(std::memory_order_relaxed),
                    vfoBFrequencyPending.load(std::memory_order_relaxed),
                    lastVfoBUpdate.load(std::memory_order_relaxed)};
        }

        /**
         * @brief Check if radio is in split mode based on VFO selection
         * @return true if RX and TX VFOs are different (split mode)
         */
        bool isInSplitMode() const {
            return currentRxVfo.load(std::memory_order_relaxed) != currentTxVfo.load(std::memory_order_relaxed);
        }

        /**
         * @brief Get the current active VFO for receive
         * @return 0 for VFO A, 1 for VFO B
         */
        uint8_t getCurrentRxVfo() const { return currentRxVfo.load(std::memory_order_relaxed); }

        /**
         * @brief Get the current active VFO for transmit
         * @return 0 for VFO A, 1 for VFO B
         */
        uint8_t getCurrentTxVfo() const { return currentTxVfo.load(std::memory_order_relaxed); }

        /**
         * @brief Get the frequency of the currently active receive VFO
         * @return Current RX frequency in Hz
         */
        uint64_t getCurrentRxFrequency() const
        {
            return getCurrentRxVfo() == 0 ? vfoAFrequency.load(std::memory_order_relaxed)
                                          : vfoBFrequency.load(std::memory_order_relaxed);
        }

        /**
         * @brief Get the frequency of the currently active transmit VFO
         * @return Current TX frequency in Hz
         */
        uint64_t getCurrentTxFrequency() const
        {
            return getCurrentTxVfo() == 0 ? vfoAFrequency.load(std::memory_order_relaxed)
                                          : vfoBFrequency.load(std::memory_order_relaxed);
        }

        /**
         * @brief Check if there was recent button activity within the specified time window
         * @param currentTime Current timestamp in microseconds
         * @param timeoutUs Timeout window in microseconds
         * @return true if button was pressed within the timeout window
         */
        bool hasRecentButtonActivity(const uint64_t currentTime, const uint64_t timeoutUs) const
        {
            const uint64_t lastActivity = lastButtonActivityTime.load(std::memory_order_relaxed);
            return lastActivity > 0 && (currentTime - lastActivity) < timeoutUs;
        }

        /**
         * @brief Check if there was recent encoder activity within the specified time window
         * @param currentTime Current timestamp in microseconds
         * @param timeoutUs Timeout window in microseconds
         * @return true if encoder was turned within the timeout window
         */
        bool hasRecentEncoderActivity(const uint64_t currentTime, const uint64_t timeoutUs) const
        {
            const uint64_t lastActivity = lastEncoderActivityTime.load(std::memory_order_relaxed);
            return lastActivity > 0 && (currentTime - lastActivity) < timeoutUs;
        }

        /**
         * @brief Check if there was any recent user activity (button or encoder)
         * @param currentTime Current timestamp in microseconds
         * @param timeoutUs Timeout window in microseconds
         * @return true if any user activity occurred within the timeout window
         */
        bool hasRecentUserActivity(const uint64_t currentTime, const uint64_t timeoutUs) const
        {
            return hasRecentButtonActivity(currentTime, timeoutUs) || hasRecentEncoderActivity(currentTime, timeoutUs);
        }

        /**
         * @brief TX ownership management methods for mutual exclusion
         */

        // TX timeout in microseconds (30 seconds)
        static constexpr uint64_t TX_TIMEOUT_US = 30 * 1000 * 1000;

        /**
         * @brief Try to acquire TX ownership
         * @param source CommandSource requesting TX
         * @param currentTime Current timestamp in microseconds
         * @return true if TX was acquired successfully
         */
        bool tryAcquireTx(CommandSource source, uint64_t currentTime);

        /**
         * @brief Release TX ownership
         * @param source CommandSource releasing TX
         * @param currentTime Current timestamp in microseconds
         * @return true if TX was released successfully
         */
        bool releaseTx(CommandSource source, uint64_t currentTime);

        /**
         * @brief Force release TX (for timeouts or disconnects)
         * @param currentTime Current timestamp in microseconds
         * @return true if TX was released
         */
        bool forceReleaseTx(uint64_t currentTime);

        /**
         * @brief Check if TX has timed out
         * @param currentTime Current timestamp in microseconds
         * @return true if TX should be released due to timeout
         */
        bool isTxTimedOut(uint64_t currentTime) const
        {
            if (!isTx.load(std::memory_order_relaxed))
                return false;
            uint64_t activationTime = txActivationTime.load(std::memory_order_relaxed);
            return activationTime > 0 && (currentTime - activationTime) > TX_TIMEOUT_US;
        }

        /**
         * @brief Get current TX owner
         * @return CommandSource that owns TX, or -1 if none
         */
        int getTxOwner() const { return txOwner.load(std::memory_order_relaxed); }

        // Control lease constants and helpers
        static constexpr uint64_t CONTROL_LEASE_DURATION_US = 1500000ULL; // 1.5 seconds default lease window

        bool tryAcquireControlLease(CommandSource source, int priority, uint64_t currentTime, uint64_t leaseDurationUs);
        bool refreshControlLease(CommandSource source, uint64_t currentTime, uint64_t leaseDurationUs);
        void releaseControlLease(CommandSource source);
        void forceReleaseControlLease();
        bool isControlLeaseActive(CommandSource source, uint64_t currentTime) const;
        int getControlLeaseOwner() const { return controlLeaseOwner.load(); }
    };

} // namespace radio
