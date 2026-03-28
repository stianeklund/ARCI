#include "include/RadioManager.h"
#include <algorithm>
#include <charconv>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include "NvsManager.h"
#include "ExtendedCommandHandler.h"
#include "RadioCommand.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/ForwardingPolicy.h"
#include "FrequencyFormatter.h"
#include "TcpCatBridge.h"

#include "AntennaCommandHandler.h"
#include "AudioEqualizerCommandHandler.h"
#include "BaseCommandHandler.h"
#include "CwCommandHandler.h"
#include "FrequencyVfoCommandHandler.h"
#include "GainLevelCommandHandler.h"
#include "InterfaceSystemCommandHandler.h"
#include "MemoryCommandHandler.h"
#include "MenuConfigCommandHandler.h"
#include "ModeCommandHandler.h"
#include "MXCommandHandler.h"
#include "ReceiverProcessingCommandHandler.h"
#include "ScanCommandHandler.h"
#include "StatusInfoCommandHandler.h"
#include "ToneSquelchCommandHandler.h"
#include "TransmitterCommandHandler.h"
#include "UICommandHandler.h"
#include "VisualScanCommandHandler.h"
#include "VoiceMessageCommandHandler.h"
#include "include/CommandDispatcher.h"
#include "MacroStorage.h"

namespace radio
{
    namespace
    {
        constexpr uint64_t SLOW_COMMAND_WARN_THRESHOLD_US = 10000; // warn only when handler exceeds 10 ms (serial I/O latency)
        constexpr uint64_t MUTEX_CONTENTION_WARN_THRESHOLD_US = 5000; // log contention only when wait > 5 ms
        constexpr uint64_t MIN_FREQUENCY = 30000; // 30 kHz minimum valid frequency
    } // namespace

    // Fast, stack-based frequency formatter for high-frequency VFO updates
    // Eliminates std::to_string heap allocations and string concatenations
    class FastFreqFormatter
    {
        static constexpr size_t BUFFER_SIZE = 16; // "FA" + 11 digits + ";" + null
        char buffer_[BUFFER_SIZE] = {};

    public:
        // Format VFO A frequency command: "FA<frequency>;"
        const char *formatFA(const uint64_t freq)
        {
            char *p = buffer_ + BUFFER_SIZE - 1;
            *p = '\0';
            *--p = ';';

            // Fast reverse decimal conversion
            uint64_t n = freq;
            do
            {
                *--p = '0' + n % 10;
                n /= 10;
            }
            while (n > 0);

            *--p = 'A';
            *--p = 'F';
            return p;
        }

        // Format VFO B frequency command: "FB<frequency>;"
        const char *formatFB(const uint64_t freq)
        {
            char *p = buffer_ + BUFFER_SIZE - 1;
            *p = '\0';
            *--p = ';';

            // Fast reverse decimal conversion
            uint64_t n = freq;
            do
            {
                *--p = '0' + n % 10;
                n /= 10;
            }
            while (n > 0);

            *--p = 'B';
            *--p = 'F';
            return p;
        }
    };

    namespace
    {
        constexpr int controlPriority(const CommandSource source)
        {
            switch (source)
            {
            case CommandSource::Remote:
                return 5;
            case CommandSource::Panel:
                return 4;
            case CommandSource::UsbCdc0:
            case CommandSource::Tcp0:
                return 3;
            case CommandSource::UsbCdc1:
            case CommandSource::Tcp1:
                return 2;
            case CommandSource::Display:
                return 1;
            case CommandSource::Macro:
                return 0;
            default:
                return 0;
            }
        }
    } // namespace

    RadioManager::RadioManager(ISerialChannel &radioSerial, ISerialChannel &usbSerial) :
        radioSerial_(radioSerial), usbSerial_(usbSerial)
    {
        commandDispatcher_ = std::make_unique<CommandDispatcher>();

        if (!commandDispatcher_)
        {
            ESP_LOGE(RadioManager::TAG, "Failed to create CommandDispatcher");
            return;
        }

        ESP_LOGD(RadioManager::TAG, "Initializing command handlers...");
        initializeCommandHandlers();

        ESP_LOGV(RadioManager::TAG, "Creating local CATHandler...");
        localHandler_ =
            std::make_unique<CATHandler>(*commandDispatcher_, *this, radioSerial_, usbSerial_, CommandSource::UsbCdc0);
        ESP_LOGV(RadioManager::TAG, "Creating remote CATHandler...");
        remoteHandler_ =
            std::make_unique<CATHandler>(*commandDispatcher_, *this, radioSerial_, usbSerial_, CommandSource::Remote);
        ESP_LOGV(RadioManager::TAG, "Creating panel CATHandler (Panel source)...");
        panelHandler_ =
            std::make_unique<CATHandler>(*commandDispatcher_, *this, radioSerial_, usbSerial_, CommandSource::Panel);
        ESP_LOGV(RadioManager::TAG, "Creating macro CATHandler (Macro source)...");
        macroHandler_ =
            std::make_unique<CATHandler>(*commandDispatcher_, *this, radioSerial_, usbSerial_, CommandSource::Macro);

        // Skip networked client init during unit tests
#ifndef CONFIG_RUN_UNIT_TESTS
        ESP_LOGV(RadioManager::TAG, "Creating antenna switch...");
        antennaSwitch_ = std::make_unique<antenna::AntennaSwitch>();

        // Initialize antenna switch with KConfig settings
        antenna::AntennaConfig antennaConfig;
#ifdef CONFIG_ANTENNA_SWITCH_ENABLED
        antennaConfig.enabled = true;
        // Construct base URL from host and port
        antennaConfig.baseUrl =
            "http://" + std::string(CONFIG_ANTENNA_SWITCH_HOST) + ":" + std::to_string(CONFIG_ANTENNA_SWITCH_PORT);
        antennaConfig.timeoutMs = CONFIG_ANTENNA_SWITCH_TIMEOUT_MS;
        antennaConfig.maxRetries = CONFIG_ANTENNA_SWITCH_MAX_RETRIES;
#ifdef CONFIG_ANTENNA_SWITCH_FALLBACK_TO_RADIO
        antennaConfig.fallbackToRadio = CONFIG_ANTENNA_SWITCH_FALLBACK_TO_RADIO;
#else
        antennaConfig.fallbackToRadio = true; // Default to true if not defined
#endif
        antennaConfig.statusEndpoint = CONFIG_ANTENNA_SWITCH_STATUS_ENDPOINT;
        antennaConfig.controlEndpoint = CONFIG_ANTENNA_SWITCH_CONTROL_ENDPOINT;

        // Configure transport protocol
#ifdef CONFIG_ANTENNA_SWITCH_TRANSPORT_HTTP
        antennaConfig.transportType = antenna::TransportType::Http;
#elif defined(CONFIG_ANTENNA_SWITCH_TRANSPORT_WEBSOCKET)
        antennaConfig.transportType = antenna::TransportType::WebSocket;
#else // CONFIG_ANTENNA_SWITCH_TRANSPORT_AUTO (default)
        antennaConfig.transportType = antenna::TransportType::Auto;
#endif

        // WebSocket configuration (only if not HTTP-only)
#ifndef CONFIG_ANTENNA_SWITCH_TRANSPORT_HTTP
#ifdef CONFIG_ANTENNA_SWITCH_WS_URL
        antennaConfig.wsUrl = CONFIG_ANTENNA_SWITCH_WS_URL;
#else
        antennaConfig.wsUrl = "ws://antenna-switch.local/ws";
#endif

#ifdef CONFIG_ANTENNA_SWITCH_WS_EVENTS
        antennaConfig.enableWebSocketEvents = CONFIG_ANTENNA_SWITCH_WS_EVENTS;
#else
        antennaConfig.enableWebSocketEvents = true;
#endif

#ifdef CONFIG_ANTENNA_SWITCH_WS_RECONNECT_MS
        antennaConfig.wsReconnectTimeoutMs = CONFIG_ANTENNA_SWITCH_WS_RECONNECT_MS;
#else
        antennaConfig.wsReconnectTimeoutMs = 5000;
#endif

#ifdef CONFIG_ANTENNA_SWITCH_WS_KEEPALIVE_MS
        antennaConfig.wsKeepAliveMs = CONFIG_ANTENNA_SWITCH_WS_KEEPALIVE_MS;
#else
        antennaConfig.wsKeepAliveMs = 30000;
#endif
#endif

        ESP_LOGI(RadioManager::TAG, "Antenna switch configured: %s (transport: %s)", antennaConfig.baseUrl.c_str(),
                 antennaConfig.transportType == antenna::TransportType::Http            ? "HTTP"
                     : antennaConfig.transportType == antenna::TransportType::WebSocket ? "WebSocket"
                                                                                        : "Auto");
#else
        antennaConfig.enabled = false;
        antennaConfig.transportType = antenna::TransportType::Http;
        antennaConfig.baseUrl = "http://antenna-switch.local:80"; // Default URL for manual enabling
        antennaConfig.timeoutMs = 2000;
        antennaConfig.maxRetries = 3;
        antennaConfig.fallbackToRadio = true;
        antennaConfig.statusEndpoint = "/status";
        antennaConfig.controlEndpoint = "/api/antenna/switch";
#endif
        antennaSwitch_->initialize(antennaConfig);

        ESP_LOGI(RadioManager::TAG, "Setting radio connected...");
        setRadioConnected(true);
#else
        ESP_LOGI(RadioManager::TAG, "Unit tests: skipping AntennaSwitch init and radio connection flag");
#endif

        // Transverter display offset starts disabled; enable via UIXD1; command
        state_.transverterOffsetEnabled.store(false, std::memory_order_relaxed);

        ESP_LOGI(RadioManager::TAG, "RadioManager initialized with command system (tasks not started)");
    }

    esp_err_t RadioManager::startTasks()
    {
        // Initialize TX timeout monitoring task
        constexpr uint32_t TX_TIMEOUT_STACK_WORDS = 4096; // ~16 KB on Xtensa
        ESP_LOGI(RadioManager::TAG, "Starting TX timeout monitoring task (stack=%lu words)",
                 static_cast<unsigned long>(TX_TIMEOUT_STACK_WORDS));

        BaseType_t result = xTaskCreate(txTimeoutTask, "tx_timeout", TX_TIMEOUT_STACK_WORDS, this, 5, &txTimeoutTaskHandle_);
        if (result != pdPASS)
        {
            ESP_LOGE(RadioManager::TAG, "Failed to create TX timeout task");
            return ESP_FAIL;
        }

        // Initialize manual keepalive task if enabled
#ifdef CONFIG_RADIOCORE_MANUAL_KEEPALIVE
        ESP_LOGI(RadioManager::TAG, "Starting manual keepalive task (interval: %dms)",
                 CONFIG_RADIOCORE_MANUAL_KEEPALIVE_TIME);
        result = xTaskCreate(keepaliveTask, "keepalive", 4096, this, 4, &keepaliveTaskHandle_);
        if (result != pdPASS)
        {
            ESP_LOGE(RadioManager::TAG, "Failed to create keepalive task");
            // Clean up TX timeout task before returning
            if (txTimeoutTaskHandle_ != nullptr)
            {
                vTaskDelete(txTimeoutTaskHandle_);
                txTimeoutTaskHandle_ = nullptr;
            }
            return ESP_FAIL;
        }
#endif

        ESP_LOGI(RadioManager::TAG, "RadioManager tasks started successfully");
        return ESP_OK;
    }

    RadioManager::~RadioManager()
    {
        if (txTimeoutTaskHandle_ != nullptr)
        {
            ESP_LOGI(RadioManager::TAG, "Terminating TX timeout monitoring task");
            vTaskDelete(txTimeoutTaskHandle_);
            txTimeoutTaskHandle_ = nullptr;
        }
#ifdef CONFIG_RADIOCORE_MANUAL_KEEPALIVE
        if (keepaliveTaskHandle_ != nullptr)
        {
            ESP_LOGI(RadioManager::TAG, "Terminating manual keepalive task");
            vTaskDelete(keepaliveTaskHandle_);
            keepaliveTaskHandle_ = nullptr;
        }
#endif
    }

    void RadioManager::setDisplaySerial(ISerialChannel *display)
    {
        displaySerial_ = display;
        if (displaySerial_)
        {
            ESP_LOGI(RadioManager::TAG, "Display serial registered for mirroring SET commands");
        }
        else
        {
            ESP_LOGW(RadioManager::TAG, "Display serial cleared; no mirroring will occur");
        }
    }

    ISerialChannel *RadioManager::getDisplaySerial() const { return displaySerial_; }

    void RadioManager::setTcp0Bridge(tcp_cat_bridge::TcpCatBridge *bridge)
    {
        tcp0Bridge_ = bridge;
        if (tcp0Bridge_)
        {
            ESP_LOGI(RadioManager::TAG, "TCP0 bridge registered for CAT command routing");
        }
        else
        {
            ESP_LOGW(RadioManager::TAG, "TCP0 bridge cleared");
        }
    }

    void RadioManager::setTcp1Bridge(tcp_cat_bridge::TcpCatBridge *bridge)
    {
        tcp1Bridge_ = bridge;
        if (tcp1Bridge_)
        {
            ESP_LOGI(RadioManager::TAG, "TCP1 bridge registered for CAT command routing");
        }
        else
        {
            ESP_LOGW(RadioManager::TAG, "TCP1 bridge cleared");
        }
    }

    void RadioManager::sendToDisplay(std::string_view frames) const
    {
        if (!displaySerial_)
            return;
        if (frames.empty())
            return;

        // Check if display communication is enabled (UIDE command state)
        // UI commands bypass this check via sendToDisplayOnly
        if (!state_.displayCommunicationEnabled.load(std::memory_order_relaxed))
        {
            ESP_LOGV(TAG, "Display communication disabled, blocking frame: %.*s",
                     static_cast<int>(frames.size()), frames.data());
            return;
        }

        displaySerial_->sendMessage(frames);
    }

    void RadioManager::signalUserActivity()
    {
        if (!displaySerial_)
            return;

        const uint64_t now = esp_timer_get_time();
        const bool displayAwake = state_.displayAwake.load(std::memory_order_relaxed);
        const uint64_t lastUips = state_.lastUipsSendTime.load(std::memory_order_relaxed);

        // If display is asleep, send UIPS1 immediately to wake it (with small debounce)
        if (!displayAwake)
        {
            constexpr uint64_t WAKE_DEBOUNCE_US = 100000; // 100ms debounce for wake attempts
            if (lastUips > 0 && (now - lastUips) < WAKE_DEBOUNCE_US)
                return;

            state_.lastUipsSendTime.store(now, std::memory_order_relaxed);
            displaySerial_->sendMessage("UIPS1;");
            ESP_LOGI(TAG, "Display asleep - sending UIPS1 to wake");
            return;
        }

        // Display is awake - periodically query state to verify and self-correct
        // Use moderate interval since normal CAT traffic (FA/FB) also keeps LVGL active
        constexpr uint64_t STATE_QUERY_INTERVAL_US = 15000000; // 15 seconds
        if (lastUips > 0 && (now - lastUips) < STATE_QUERY_INTERVAL_US)
            return;

        state_.lastUipsSendTime.store(now, std::memory_order_relaxed);
        displaySerial_->sendMessage("UIPS;");
        ESP_LOGD(TAG, "Display state query: sent UIPS; to verify awake status");
    }

    void RadioManager::setDisplayAwake(bool awake)
    {
        const bool wasAwake = state_.displayAwake.exchange(awake, std::memory_order_relaxed);
        if (wasAwake != awake)
        {
            ESP_LOGI(TAG, "Display state changed: %s", awake ? "AWAKE" : "ASLEEP");
        }
    }

    void RadioManager::setDisplayScreensaverTimeout(uint8_t minutes)
    {
        state_.displayScreensaverTimeoutMin.store(minutes, std::memory_order_relaxed);
        ESP_LOGI(TAG, "Display screensaver timeout: %u minutes%s", minutes, minutes == 0 ? " (disabled)" : "");
    }

    void RadioManager::sendToSource(CommandSource src, std::string_view frames) const
    {
        if (frames.empty())
            return;
        switch (src)
        {
        case CommandSource::UsbCdc0:
            usbSerial_.sendMessage(frames);
            break;
        case CommandSource::UsbCdc1:
            if (usbCdc1Serial_)
            {
                usbCdc1Serial_->sendMessage(frames);
            }
            else
            {
                // Fallback to CDC0 if CDC1 not registered
                usbSerial_.sendMessage(frames);
            }
            break;
        case CommandSource::Tcp0:
            if (tcp0Bridge_)
            {
                tcp0Bridge_->sendToActiveClient(frames);
            }
            break;
        case CommandSource::Tcp1:
            if (tcp1Bridge_)
            {
                tcp1Bridge_->sendToActiveClient(frames);
            }
            break;
        case CommandSource::Display:
            if (displaySerial_)
            {
                displaySerial_->sendMessage(frames);
            }
            break;
        case CommandSource::Panel:
            // Panel-origin responses are forwarded to USB to keep host in sync
            usbSerial_.sendMessage(frames);
            break;
        case CommandSource::Macro:
            // Macro responses should not be forwarded (handled by AI policy)
            ESP_LOGV(RadioManager::TAG, "sendToSource called for Macro (no-op by design)");
            break;
        case CommandSource::Remote:
            // Should not happen for routing answers
            break;
        }
    }


    void RadioManager::sendRawRadioCommand(std::string_view command) const
    {
        sendRadioCommand(command);
    }

    void RadioManager::requestFrequencyUpdate()
    {
        ESP_LOGI(TAG, "📡 Requesting frequency update: invalidating FA/FB/IF cache and querying radio");

        // Invalidate cache entries for frequency-related commands
        state_.commandCache.invalidate("FA");
        state_.commandCache.invalidate("FB");
        state_.commandCache.invalidate("IF");

        // Send queries to get fresh frequency values from radio
        // Using raw commands to bypass cache and go directly to radio
        sendRadioCommand("FA;");
        sendRadioCommand("FB;");
    }

    void RadioManager::sendDirectResponse(std::string_view response) const
    {
        ESP_LOGD(RadioManager::TAG, "Sending direct response to USB: %.*s", (int)response.size(), response.data());

        if (const esp_err_t result = usbSerial_.sendMessage(response); result != ESP_OK)
        {
            // Rate-limit backpressure errors to avoid log spam during USB buffer congestion
            static uint32_t backpressureErrorCount = 0;
            static uint64_t lastBackpressureLogUs = 0;
            constexpr uint64_t BACKPRESSURE_LOG_INTERVAL_US = 5000000; // Log once per 5 seconds max

            if (result == ESP_ERR_NO_MEM) {
                // Backpressure - CDC buffer full. Rate-limit logging.
                backpressureErrorCount++;
                const uint64_t nowUs = esp_timer_get_time();
                if (nowUs - lastBackpressureLogUs > BACKPRESSURE_LOG_INTERVAL_US) {
                    ESP_LOGW(RadioManager::TAG,
                             "USB backpressure (direct): dropped %lu responses in last %.1f seconds (buffer full)",
                             backpressureErrorCount, (nowUs - lastBackpressureLogUs) / 1000000.0);
                    backpressureErrorCount = 0;
                    lastBackpressureLogUs = nowUs;
                }
            } else {
                // Other errors are unexpected and should always be logged
                ESP_LOGE(RadioManager::TAG, "Failed to send direct response to USB: %s", esp_err_to_name(result));
            }
        }
    }

    bool RadioManager::updateVfoAFrequency(const uint64_t frequency)
    {
        if (!RadioState::isValidFrequency(frequency))
        {
            ESP_LOGW(RadioManager::TAG, "Invalid VFO A frequency: %llu", frequency);
            return false;
        }

        const uint64_t currentTime = getCurrentTimestamp();
        const uint64_t oldFreq = state_.vfoAFrequency.exchange(frequency);
        state_.lastVfoAUpdate.store(currentTime);
        state_.commandCache.update("FA", currentTime);

        if (oldFreq != frequency)
        {
            ESP_LOGD(RadioManager::TAG, "VFO A frequency updated: %llu -> %llu", oldFreq, frequency);
            return true;
        }
        return false;
    }

    bool RadioManager::updateVfoBFrequency(const uint64_t frequency)
    {
        if (!RadioState::isValidFrequency(frequency))
        {
            ESP_LOGW(RadioManager::TAG, "Invalid VFO B frequency: %llu", frequency);
            return false;
        }

        const uint64_t currentTime = getCurrentTimestamp();
        const uint64_t oldFreq = state_.vfoBFrequency.exchange(frequency);
        state_.lastVfoBUpdate.store(currentTime);
        state_.commandCache.update("FB", currentTime);

        if (oldFreq != frequency)
        {
            ESP_LOGD(RadioManager::TAG, "VFO B frequency updated: %llu -> %llu", oldFreq, frequency);
            return true;
        }
        return false;
    }

    // === TRANSVERTER FREQUENCY CONVERSION (Centralized implementation) ===

    bool RadioManager::isTransverterOffsetActive() const
    {
        return state_.transverterOffsetEnabled.load(std::memory_order_relaxed) &&
               state_.transverter.load(std::memory_order_relaxed) &&
               state_.transverterOffsetHz.load(std::memory_order_relaxed) > 0;
    }

    uint64_t RadioManager::baseToDisplayFrequency(const uint64_t baseFreq) const
    {
        // Atomic snapshot of transverter state to prevent race conditions
        const bool displayOffsetEnabled = state_.transverterOffsetEnabled.load(std::memory_order_relaxed);
        const bool transverterEnabled = state_.transverter.load(std::memory_order_relaxed);
        const uint64_t offset = state_.transverterOffsetHz.load(std::memory_order_relaxed);
        const bool offsetPlus = state_.transverterOffsetPlus.load(std::memory_order_relaxed);

        // Check all conditions using the snapshot
        if (!(displayOffsetEnabled && transverterEnabled && offset > 0))
        {
            return baseFreq;
        }

        if (offsetPlus)
        {
            return baseFreq + offset;
        }
        // Offset minus: protect against underflow
        return (baseFreq > offset) ? (baseFreq - offset) : 0ULL;
    }

    uint64_t RadioManager::displayToBaseFrequency(const uint64_t displayFreq) const
    {
        // Atomic snapshot of transverter state to prevent race conditions
        const bool displayOffsetEnabled = state_.transverterOffsetEnabled.load(std::memory_order_relaxed);
        const bool transverterEnabled = state_.transverter.load(std::memory_order_relaxed);
        const uint64_t offset = state_.transverterOffsetHz.load(std::memory_order_relaxed);
        const bool offsetPlus = state_.transverterOffsetPlus.load(std::memory_order_relaxed);

        // Check all conditions using the snapshot
        if (!(displayOffsetEnabled && transverterEnabled && offset > 0))
        {
            return displayFreq;
        }

        if (offsetPlus)
        {
            // Display = Base + Offset => Base = Display - Offset
            if (displayFreq > offset)
            {
                const uint64_t baseFreq = displayFreq - offset;
                // Clamp to valid radio frequency range
                return (baseFreq < MIN_FREQUENCY) ? MIN_FREQUENCY : baseFreq;
            }
            // Safety net: value appears to already be in base-space
            ESP_LOGD(TAG, "Display<offset: treating %llu as base-space (offset=%llu)", displayFreq, offset);
            return (displayFreq < MIN_FREQUENCY) ? MIN_FREQUENCY : displayFreq;
        }

        // Display = Base - Offset => Base = Display + Offset
        const uint64_t baseFreq = displayFreq + offset;
        return (baseFreq < MIN_FREQUENCY) ? MIN_FREQUENCY : baseFreq;
    }

    void RadioManager::refreshDisplayFrequencies() const
    {
        if (!displaySerial_)
        {
            return;
        }

        const uint64_t baseFreqA = state_.vfoAFrequency.load();
        const uint64_t baseFreqB = state_.vfoBFrequency.load();

        const uint64_t displayFreqA = baseToDisplayFrequency(baseFreqA);
        const uint64_t displayFreqB = baseToDisplayFrequency(baseFreqB);

        // Format and send FA/FB commands using shared formatter
        static thread_local FrequencyFormatter formatter;
        displaySerial_->sendMessage(formatter.formatFA(displayFreqA));
        displaySerial_->sendMessage(formatter.formatFB(displayFreqB));

        ESP_LOGD(TAG, "Refreshed display: FA=%llu FB=%llu (base: %llu/%llu)",
                 displayFreqA, displayFreqB, baseFreqA, baseFreqB);
    }

    bool RadioManager::updateMode(const int mode)
    {
        if (!RadioState::isValidMode(static_cast<int8_t>(mode)))
        {
            ESP_LOGW(RadioManager::TAG, "Invalid mode: %d", mode);
            return false;
        }

        const int8_t oldMode = state_.mode.exchange(static_cast<int8_t>(mode));
        const uint64_t currentTime = getCurrentTimestamp();
        state_.lastModeUpdate.store(currentTime);
        state_.commandCache.update("MD", currentTime);

        if (oldMode != mode)
        {
            ESP_LOGD(RadioManager::TAG, "Mode updated: %d -> %d", oldMode, mode);
            return true;
        }
        return false;
    }

    bool RadioManager::updateDataMode(const int dataMode)
    {
        if (dataMode < 0 || dataMode > 1)
        {
            ESP_LOGW(RadioManager::TAG, "Invalid data mode: %d", dataMode);
            return false;
        }

        const int8_t oldDataMode = state_.dataMode.exchange(static_cast<int8_t>(dataMode));
        // Update cache timestamp for DA to enable immediate cached replies
        const uint64_t currentTime = getCurrentTimestamp();
        state_.commandCache.update("DA", currentTime);

        if (oldDataMode != dataMode)
        {
            ESP_LOGD(RadioManager::TAG, "Data mode updated: %d -> %d", oldDataMode, dataMode);
            return true;
        }
        return false;
    }

    bool RadioManager::updateRxVfo(const int vfo)
    {
        if (vfo < 0 || vfo > 2)
        {
            ESP_LOGW(RadioManager::TAG, "Invalid RX VFO: %d", vfo);
            return false;
        }

        if (const uint8_t oldVfo = state_.currentRxVfo.exchange(static_cast<uint8_t>(vfo)); oldVfo != vfo)
        {
            ESP_LOGD(RadioManager::TAG, "RX VFO updated: %d -> %d", oldVfo, vfo);

            // Do not automatically synchronize split flag from VFO state
            // Split mode should only be set explicitly via updateSplitEnabled() or SP commands
            // Automatic detection causes incorrect split mode during simple VFO toggles

            return true;
        }
        return false;
    }

    bool RadioManager::updateTxVfo(const int vfo)
    {
        if (vfo < 0 || vfo > 2)
        {
            ESP_LOGW(RadioManager::TAG, "Invalid TX VFO: %d", vfo);
            return false;
        }

        if (const uint8_t oldVfo = state_.currentTxVfo.exchange(static_cast<uint8_t>(vfo)); oldVfo != vfo)
        {
            ESP_LOGD(RadioManager::TAG, "TX VFO updated: %d -> %d", oldVfo, vfo);

            // Do not automatically synchronize split flag from VFO state
            // Split mode should only be set explicitly via updateSplitEnabled() or SP commands
            // Automatic detection causes incorrect split mode during simple VFO toggles

            return true;
        }
        return false;
    }

    bool RadioManager::updatePowerState(const bool powerOn)
    {
        const bool oldPower = state_.powerOn.exchange(powerOn);
        if (oldPower != powerOn)
        {
            ESP_LOGI(TAG, "🔌 Power state changed: %s -> %s", oldPower ? "ON" : "OFF", powerOn ? "ON" : "OFF");

            // Control display backlight based on power state
            if (displaySerial_)
            {
                if (powerOn)
                {
                    ESP_LOGD(TAG, "Restoring display backlight (UIBL255)");
                    displaySerial_->sendMessage("UIBL255;");
                }
                else
                {
                    ESP_LOGD(TAG, "Turning off display backlight (UIBL000)");
                    displaySerial_->sendMessage("UIBL000;");
                }
            }

            // Call registered power state change callback
            if (powerStateChangeCallback_)
            {
                powerStateChangeCallback_(powerOn, oldPower);
            }
        }
        const uint64_t currentTime = getCurrentTimestamp();
        state_.commandCache.update("PS", currentTime);
        return oldPower != powerOn;
    }

    bool RadioManager::acquirePrimaryControl(const CommandSource source)
    {
        const uint64_t currentTime = getCurrentTimestamp();
        return state_.tryAcquireControlLease(source, controlPriority(source), currentTime,
                                             RadioState::CONTROL_LEASE_DURATION_US);
    }

    bool RadioManager::refreshPrimaryControl(const CommandSource source)
    {
        const uint64_t currentTime = getCurrentTimestamp();
        return state_.refreshControlLease(source, currentTime, RadioState::CONTROL_LEASE_DURATION_US);
    }

    void RadioManager::releasePrimaryControl(const CommandSource source) { state_.releaseControlLease(source); }

    bool RadioManager::hasPrimaryControl(const CommandSource source) const
    {
        const uint64_t currentTime = esp_timer_get_time();
        return state_.isControlLeaseActive(source, currentTime);
    }

    int RadioManager::currentPrimaryControlOwner() const { return state_.getControlLeaseOwner(); }

    void RadioManager::forceReleasePrimaryControl() { state_.forceReleaseControlLease(); }

    bool RadioManager::dispatchMessage(CATHandler &handler, std::string_view message) const
    {
        // Performance instrumentation: separate mutex wait from command processing
        const uint64_t startUs = esp_timer_get_time();

        // Add timeout to prevent watchdog triggers during nested macro execution
        if (!dispatchMutex_.try_lock_for(pdMS_TO_TICKS(2000))) {
            ESP_LOGE(TAG, "dispatchMessage timeout - mutex held for >2s (cmd: %.*s)",
                     static_cast<int>(std::min(message.size(), size_t(16))), message.data());
            return false;
        }
        RtosUniqueLock<RtosRecursiveMutex> lock(dispatchMutex_, std::adopt_lock);

        const uint64_t lockAcquiredUs = esp_timer_get_time();
        const uint64_t mutexWaitUs = lockAcquiredUs - startUs;

        const bool result = handler.parseMessage(message);

        const uint64_t endUs = esp_timer_get_time();
        const uint64_t processingUs = endUs - lockAcquiredUs;
        const uint64_t totalUs = endUs - startUs;

        // Warn on excessive mutex contention (only if it exceeds relaxed threshold)
        if (mutexWaitUs > MUTEX_CONTENTION_WARN_THRESHOLD_US)
        {
            if (mutexWaitUs >= 1000)
            {
                ESP_LOGW(RadioManager::TAG, "🔒 Mutex contention: %.1f ms wait for: %.*s",
                         mutexWaitUs / 1000.0, static_cast<int>(std::min(message.size(), size_t(16))), message.data());
            }
            else
            {
                ESP_LOGW(RadioManager::TAG, "🔒 Mutex contention: %llu us wait for: %.*s",
                         mutexWaitUs, static_cast<int>(std::min(message.size(), size_t(16))), message.data());
            }
        }

        // Warn on slow command processing (only if it exceeds relaxed threshold)
        if (processingUs > SLOW_COMMAND_WARN_THRESHOLD_US)
        {
            if (processingUs >= 1000)
            {
                ESP_LOGW(RadioManager::TAG, "⚠️ Slow command: %.1f ms processing: %.*s",
                         processingUs / 1000.0, static_cast<int>(std::min(message.size(), size_t(16))), message.data());
            }
            else
            {
                ESP_LOGW(RadioManager::TAG, "⚠️ Slow command: %llu us processing: %.*s",
                         processingUs, static_cast<int>(std::min(message.size(), size_t(16))), message.data());
            }
        }

        // Detailed performance logging at VERBOSE level
        if (totalUs >= 1000)
        {
            ESP_LOGV(RadioManager::TAG, "📊 Dispatch: mutex=%.1f ms, processing=%.1f ms, total=%.1f ms",
                     mutexWaitUs / 1000.0, processingUs / 1000.0, totalUs / 1000.0);
        }
        else
        {
            ESP_LOGV(RadioManager::TAG, "📊 Dispatch: mutex=%llu us, processing=%llu us, total=%llu us",
                     mutexWaitUs, processingUs, totalUs);
        }

        return result;
    }


    void RadioManager::sendRadioCommand(const std::string_view command) const
    {
        if (command.empty())
        {
            ESP_LOGE(RadioManager::TAG, "❌ BLOCKED empty command to radio");
            return;
        }
        if (!command.ends_with(';'))
        {
            ESP_LOGE(RadioManager::TAG, "❌ BLOCKED command missing semicolon: '%.*s'", (int)command.size(),
                     command.data());
            return;
        }
        if (command.size() < 3)
        {
            ESP_LOGE(RadioManager::TAG, "❌ BLOCKED command too short: len=%zu", command.size());
            return;
        }
        for (const char c : command)
        {
            if (c < 32 && c != ';')
            {
                ESP_LOGE(RadioManager::TAG, "❌ BLOCKED command with invalid char (ASCII %d)", (int)c);
                return;
            }
        }
        // Special tracking for ID commands
        if (command.find("ID;") != std::string_view::npos)
        {
            ESP_LOGW(RadioManager::TAG, "WARNING: ID command being sent to radio! This should be handled locally!");
            ESP_LOGW(RadioManager::TAG, "Stack trace needed to find source of ID; command");
        }

        // Track last command sent for error diagnostics
        commandDispatcher_->recordCommandSentToRadio(command);

        const RtosLockGuard<RtosMutex> txLock(radioTxMutex_);
        radioSerial_.sendMessage(command);
        if (state_.isTx.load() && state_.getTxOwner() == static_cast<int>(CommandSource::Remote))
        {
            ESP_LOGD(RadioManager::TAG, "TXMON: remote TX active -> command '%.*s'", (int)command.size(),
                     command.data());
        }
        ESP_LOGD(RadioManager::TAG, "SEND->RADIO: '%.*s' (len=%zu)", (int)command.size(), command.data(),
                 command.size());
    }

    void RadioManager::sendRadioCommand(const std::string_view part1, const std::string_view part2) const
    {
        // Validate combined
        if (const size_t total = part1.size() + part2.size(); total < 3)
        {
            ESP_LOGE(RadioManager::TAG, "❌ BLOCKED 2-part command too short: total=%zu", total);
            return;
        }
        if (!(!part2.empty() && part2.back() == ';') && !(!part1.empty() && part1.back() == ';'))
        {
            ESP_LOGE(RadioManager::TAG, "❌ BLOCKED 2-part command missing semicolon");
            return;
        }
        // Basic ASCII control check on each part

        auto valid = [](std::string_view s)
        { return std::ranges::all_of(s, [](const char c) { return c >= 32 || c == ';'; }); };

        if (!valid(part1) || !valid(part2))
        {
            ESP_LOGE(RadioManager::TAG, "❌ BLOCKED 2-part command invalid char");
            return;
        }
        // Track combined command for error diagnostics (stack buffer, no heap)
        {
            char buf[48];
            const size_t total = part1.size() + part2.size();
            const size_t n = std::min(total, sizeof(buf) - 1);
            const size_t n1 = std::min(part1.size(), n);
            std::memcpy(buf, part1.data(), n1);
            const size_t n2 = std::min(part2.size(), n - n1);
            std::memcpy(buf + n1, part2.data(), n2);
            buf[n1 + n2] = '\0';
            commandDispatcher_->recordCommandSentToRadio(std::string_view{buf, n1 + n2});
        }
        const RtosLockGuard<RtosMutex> txLock(radioTxMutex_);
        radioSerial_.sendMessage(part1, part2);
        if (state_.isTx.load() && state_.getTxOwner() == static_cast<int>(CommandSource::Remote))
        {
            ESP_LOGD(RadioManager::TAG, "TXMON: remote TX active -> command '%.*s%.*s'",
                     (int)part1.size(), part1.data(), (int)part2.size(), part2.data());
        }
        ESP_LOGD(RadioManager::TAG, "SEND->RADIO(2-part): '%.*s' + '%.*s'", (int)part1.size(), part1.data(),
                 (int)part2.size(), part2.data());
    }

    void RadioManager::sendRadioCommand(const char *command) const
    {
        if (command == nullptr)
        {
            ESP_LOGE(RadioManager::TAG, "❌ BLOCKED null command pointer");
            return;
        }
        sendRadioCommand(std::string_view{command});
    }

    bool RadioManager::updateFrequency(const uint64_t newFreq)
    {
        if (!RadioState::isValidFrequency(newFreq))
        {
            ESP_LOGW(RadioManager::TAG, "Invalid frequency: %llu", newFreq);
            return false;
        }

        const uint64_t currentTime = getCurrentTimestamp();
        // Update VFO A frequency by default
        const uint64_t oldFreq = state_.vfoAFrequency.exchange(newFreq);
        state_.lastVfoAUpdate.store(currentTime);
        return oldFreq != newFreq;
    }

    bool RadioManager::updateMode(const int8_t newMode)
    {
        if (!RadioState::isValidMode(newMode))
        {
            ESP_LOGW(RadioManager::TAG, "Invalid mode: %d", newMode);
            return false;
        }

        const int8_t oldMode = state_.mode.exchange(newMode);
        const uint64_t currentTime = getCurrentTimestamp();
        state_.lastModeUpdate.store(currentTime);
        state_.commandCache.update("MD", currentTime);
        return oldMode != newMode;
    }

    bool RadioManager::updateSplit(const bool newSplit)
    {
        const bool oldSplit = state_.split.exchange(newSplit);
        const uint64_t currentTime = getCurrentTimestamp();
        state_.lastSplitUpdate.store(currentTime);
        state_.commandCache.update("FT", currentTime);
        return oldSplit != newSplit;
    }

    bool RadioManager::updateTx(const bool newTx)
    {
        const bool oldTx = state_.isTx.exchange(newTx);
        const uint64_t currentTime = getCurrentTimestamp();
        state_.commandCache.update("TX", currentTime); // TX/RX status
        return oldTx != newTx;
    }

    uint64_t RadioManager::getCurrentTimestamp() { return esp_timer_get_time(); }

    // Business logic methods moved from Cat facade

    void RadioManager::bootSequenceTask(void *pvParameters)
    {
        auto *self = static_cast<RadioManager *>(pvParameters);

        // Send commands individually with delay to prevent radio buffer overflow.
        // The TS-590SG can't process commands as fast as we can send them at 57600 baud,
        // resulting in ?; errors when batched. Individual pacing ensures reliable sync.
        constexpr TickType_t INTER_COMMAND_DELAY_MS = 10;

        // Phase 1: core state queries
        ESP_LOGI(TAG, "Boot sequence phase 1: %zu core commands", BOOT_SEQUENCE_SIZE);

        for (const auto &command : bootSequence_)
        {
            if (*command == '\0')
                break;
            self->sendRadioCommand(command);
            vTaskDelay(pdMS_TO_TICKS(INTER_COMMAND_DELAY_MS));
        }

        ESP_LOGI(TAG, "Boot sequence phase 1 completed");

        // Phase 2: common programmer commands + all 100 EX menu queries
        ESP_LOGI(TAG, "Boot sequence phase 2: %zu common + %zu EX queries",
                 BOOT_PHASE2_CMD_COUNT, EX_MENU_COUNT);

        for (const auto &command : bootPhase2Commands_)
        {
            self->sendRadioCommand(command);
            vTaskDelay(pdMS_TO_TICKS(INTER_COMMAND_DELAY_MS));
        }

        char exCmd[12]; // "EXnnn0000;" + null
        for (size_t i = 0; i < EX_MENU_COUNT; i++)
        {
            snprintf(exCmd, sizeof(exCmd), "EX%03u0000;", static_cast<unsigned>(i));
            self->sendRadioCommand(exCmd);
            vTaskDelay(pdMS_TO_TICKS(INTER_COMMAND_DELAY_MS));
        }

        // Wait for final responses to be processed before saving
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "Boot sequence phase 2 completed - saving EX menu to NVS");

        // Save verified EX menu state to NVS
        if (self->nvsManager_)
        {
            auto *nvs = static_cast<NvsManager *>(self->nvsManager_);
            nvs->saveExtendedMenu();
        }

        vTaskDelete(nullptr); // Self-delete when done
    }

    void RadioManager::performBootSequence() const
    {
#ifndef CONFIG_RUN_UNIT_TESTS
        if (state_.powerOn.load())
        {
            // Spawn background task for non-blocking paced command sending
            xTaskCreate(
                bootSequenceTask,
                "boot_seq",
                4096, // Needs stack for sendRadioCommand + logging + NVS save
                const_cast<RadioManager *>(this),
                5, // Lower priority - sync can happen in background
                nullptr
            );
        }
#endif
    }

    void RadioManager::syncTransverterMenuSettings() const
    {
        // Only sync when powered on, keepAlive set, and radio connected
        if (!state_.powerOn.load() || !state_.keepAlive.load() || !isRadioConnected())
        {
            ESP_LOGI(RadioManager::TAG, "Skip transverter menu sync: powerOn=%d keepAlive=%d connected=%d",
                     (int)state_.powerOn.load(), (int)state_.keepAlive.load(), (int)isRadioConnected());
            return;
        }
        ESP_LOGI(RadioManager::TAG, "Synchronizing transverter-related menu settings with radio");

        // Query critical transverter-related menu settings with pacing to prevent ?; errors
        constexpr TickType_t INTER_COMMAND_DELAY_MS = 10;

        sendRadioCommand("EX0560000;"); // EX056: Transverter function enable/disable
        vTaskDelay(pdMS_TO_TICKS(INTER_COMMAND_DELAY_MS));
        sendRadioCommand("EX0590000;"); // EX059: HF linear amplifier control
        vTaskDelay(pdMS_TO_TICKS(INTER_COMMAND_DELAY_MS));
        sendRadioCommand("EX0600000;"); // EX060: VHF linear amplifier control
        vTaskDelay(pdMS_TO_TICKS(INTER_COMMAND_DELAY_MS));
        sendRadioCommand("EX0850000;"); // EX085: DRV connector output function
        vTaskDelay(pdMS_TO_TICKS(INTER_COMMAND_DELAY_MS));
        sendRadioCommand("XO;"); // XO: Transverter offset frequency and direction
        vTaskDelay(pdMS_TO_TICKS(INTER_COMMAND_DELAY_MS));
        sendRadioCommand("AN;"); // AN: Antenna configuration

        ESP_LOGD(RadioManager::TAG, "Transverter menu synchronization commands sent");
    }

    void RadioManager::updateBandFromVfoA()
    {
        if (const uint64_t freq = state_.vfoAFrequency.load(); freq > 0)
        {
            decodeBandFromFreq(freq);
        }
    }

    void RadioManager::updateBandFromVfoB()
    {
        if (const uint64_t freq = state_.vfoBFrequency.load(); freq > 0)
        {
            decodeBandFromFreq(freq);
        }
    }

    void RadioManager::decodeBandFromFreq(const uint64_t frequencyHz)
    {
        if (frequencyHz == 0)
            return;

        // Determine band number based on frequency and store in RadioManager state
        int bandNumber = 10; // Default to general coverage (GENE)
        if (frequencyHz >= 1800000 && frequencyHz <= 2000000)
        {
            bandNumber = 0;
        }
        else if (frequencyHz >= 3500000 && frequencyHz <= 4000000)
        {
            bandNumber = 1;
        }
        else if (frequencyHz >= 7000000 && frequencyHz <= 7300000)
        {
            bandNumber = 2;
        }
        else if (frequencyHz >= 10100000 && frequencyHz <= 10150000)
        {
            bandNumber = 3;
        }
        else if (frequencyHz >= 14000000 && frequencyHz <= 14350000)
        {
            bandNumber = 4;
        }
        else if (frequencyHz >= 18068000 && frequencyHz <= 18168000)
        {
            bandNumber = 5;
        }
        else if (frequencyHz >= 21000000 && frequencyHz <= 21450000)
        {
            bandNumber = 6;
        }
        else if (frequencyHz >= 24890000 && frequencyHz <= 24990000)
        {
            bandNumber = 7;
        }
        else if (frequencyHz >= 28000000 && frequencyHz <= 29700000)
        {
            bandNumber = 8;
        }
        else if (frequencyHz >= 50000000 && frequencyHz <= 54000000)
        {
            bandNumber = 9;
        }

        state_.bandNumber.store(bandNumber, std::memory_order_relaxed);
    }

    void RadioManager::decodeBandFromFreq(const std::string_view frequency)
    {
        if (frequency.empty())
        {
            ESP_LOGD(RadioManager::TAG, "Empty frequency in decodeBandFromFreq");
            return;
        }

        uint64_t freq = 0;

        // Trim to max 9 digits (up to 999,999,999 Hz) to prevent integer overflow
        std::string_view trimmed = frequency;
        if (trimmed.length() > 9)
        {
            trimmed = trimmed.substr(0, 9);
        }

        if (auto [ptr, ec] = std::from_chars(trimmed.data(), trimmed.data() + trimmed.size(), freq); ec != std::errc())
        {
            ESP_LOGW(RadioManager::TAG, "Failed to convert frequency to integer: %.*s (trimmed: %.*s)",
                     static_cast<int>(frequency.length()), frequency.data(), static_cast<int>(trimmed.length()),
                     trimmed.data());
            return;
        }

        decodeBandFromFreq(freq);
    }

    void RadioManager::changeBand()
    {
        const int64_t currentTime = esp_timer_get_time() / 1000;
        if (currentTime - previousMicros_ < 500)
        {
            return;
        }

        if (!state_.keepAlive.load())
            return;

        updateBandFromVfoA();
        const int idx = (state_.bandNumber.load(std::memory_order_relaxed) + 1) % 10;

        const std::string bandCommand = "BD" + std::to_string(idx) + ";";
        sendRadioCommand(bandCommand);
        state_.bandNumber.store(idx, std::memory_order_relaxed);

        previousMicros_ = currentTime;
        sendRadioCommand("FA;"); // Request VFO A
        vTaskDelay(pdMS_TO_TICKS(10));
        sendRadioCommand("FB;"); // Request VFO B
    }


    void RadioManager::sendVfoUpdates() const
    {
        static constexpr unsigned long REFRESH_INTERVAL = 100; // ms
        static unsigned long lastRefreshTime = 0;
        static FastFreqFormatter formatter; // Thread-safe for const operations

        if (const uint64_t currentTime = esp_timer_get_time() / 1000; currentTime - lastRefreshTime >= REFRESH_INTERVAL)
        {
            lastRefreshTime = currentTime;

            // Only send frequency refresh if CDC0 is not in AI2 mode (AI2 provides automatic updates)
            if (state_.usbCdc0AiMode.load() != 2)
            {
                if (const uint64_t freqA = state_.vfoAFrequency.load(); freqA > 0)
                {
                    usbSerial_.sendMessage(formatter.formatFA(freqA));
                }
                if (const uint64_t freqB = state_.vfoBFrequency.load(); freqB > 0)
                {
                    usbSerial_.sendMessage(formatter.formatFB(freqB));
                }
            }
        }
    }

    bool RadioManager::shouldForwardToUSB(const std::string_view &response) const
    {
        const uint64_t currentTime = esp_timer_get_time();
        return ForwardingPolicy::shouldForwardToUSB(response, state_, currentTime);
    }

    bool RadioManager::shouldForwardToDisplay(const std::string_view &response) const
    {
        const uint64_t currentTime = esp_timer_get_time();
        return ForwardingPolicy::shouldForwardToDisplay(response, state_, currentTime);
    }

    void RadioManager::recordButtonActivity()
    {
        state_.lastButtonActivityTime.store(esp_timer_get_time(), std::memory_order_relaxed);
    }

    void RadioManager::recordEncoderActivity()
    {
        state_.lastEncoderActivityTime.store(esp_timer_get_time(), std::memory_order_relaxed);
    }

    void RadioManager::setPowerStateChangeCallback(void (*callback)(bool powerOn, bool oldState))
    {
        powerStateChangeCallback_ = callback;
        ESP_LOGD(RadioManager::TAG, "Power state change callback %s", callback ? "registered" : "cleared");
    }

    void RadioManager::checkTuningTimeout()
    {
        static constexpr uint64_t TUNING_TIMEOUT_US = 500000; // 500ms timeout

        if (state_.isTuning.load())
        {
            const uint64_t currentTime = esp_timer_get_time();
            if (const uint64_t tuningStartTime = state_.tuningStartTime.load();
                tuningStartTime > 0 && currentTime - tuningStartTime > TUNING_TIMEOUT_US)
            {
                ESP_LOGW(RadioManager::TAG, "Tuning timeout detected, clearing tuning flag");
                state_.isTuning.store(false);
                state_.tuningStartTime.store(0);
            }
        }
    }

    bool RadioManager::updateSplitEnabled(const bool enabled)
    {
        if (const bool oldSplit = state_.split.exchange(enabled); oldSplit != enabled)
        {
            ESP_LOGI(RadioManager::TAG, "🔀 SPLIT: Split mode updated: %s -> %s", oldSplit ? "ON" : "OFF",
                     enabled ? "ON" : "OFF");
            return true;
        }
        return false;
    }

    bool RadioManager::updateRitEnabled(const bool enabled)
    {
        if (const bool oldRit = state_.ritOn.exchange(enabled); oldRit != enabled)
        {
            ESP_LOGD(RadioManager::TAG, "RIT enabled updated: %s -> %s", oldRit ? "ON" : "OFF", enabled ? "ON" : "OFF");
            return true;
        }
        return false;
    }

    bool RadioManager::updateXitEnabled(const bool enabled)
    {
        if (const bool oldXit = state_.xitOn.exchange(enabled); oldXit != enabled)
        {
            ESP_LOGD(RadioManager::TAG, "XIT enabled updated: %s -> %s", oldXit ? "ON" : "OFF", enabled ? "ON" : "OFF");
            return true;
        }
        return false;
    }

    bool RadioManager::updateRitOffset(const int offset)
    {
        // Validate offset range (typical RIT range is ±9999 Hz)
        if (offset < -9999 || offset > 9999)
        {
            ESP_LOGW(RadioManager::TAG, "Invalid RIT offset: %d", offset);
            return false;
        }

        if (const int32_t oldOffset = state_.ritXitOffset.exchange(offset); oldOffset != offset)
        {
            ESP_LOGD(RadioManager::TAG, "RIT offset updated: %" PRId32 " -> %d Hz", oldOffset, offset);
            return true;
        }
        return false;
    }

    bool RadioManager::updateXitOffset(const int offset)
    {
        // XIT uses the same offset as RIT in TS-590SG
        return updateRitOffset(offset);
    }

    // === METHODS MOVED FROM CAT CLASS ===

    esp_err_t RadioManager::init()
    {
        // Initialize macro storage (user-defined macros)
        // This must be called after nvs_flash_init() - cannot be done in constructor
        // since RadioManager is a file-scope static and NVS isn't ready at static init time
        ESP_LOGD(RadioManager::TAG, "Initializing macro storage");
        if (esp_err_t err = storage::MacroStorage::instance().init(); err != ESP_OK) {
            ESP_LOGE(RadioManager::TAG, "Failed to initialize MacroStorage: %s", esp_err_to_name(err));
            return err;
        }
        return ESP_OK;
    }

    void RadioManager::sendLocal(const std::initializer_list<std::string_view> frames) const
    {
        // Aggregate small frames into one buffer to minimize parser calls
        size_t total = 0;
        for (auto f : frames)
            total += f.size();
        std::string combined;
        combined.reserve(total);
        for (auto f : frames)
            combined.append(f);
        (void)dispatchMessage(*localHandler_, combined);
    }

    // getPanelCATHandler() accessor implemented in header; no additional methods needed here

    void RadioManager::enableSplit(const bool copyVfoBeforeEnable) const
    {
        // Enable split by selecting different TX/RX VFOs (FT/FR).
        // SP is not used here; on TS-590 it is for split frequency/setting, not enable.
        const int currentRx = getRxVfo();
        if (currentRx == 0)
        {
            // RX on A -> TX must be opposite (B)
            sendLocal({"FR0;", "FT1;"});
        }
        else if (currentRx == 1)
        {
            // RX on B -> TX must be opposite (A)
            sendLocal({"FR1;", "FT0;"});
        }

        if (copyVfoBeforeEnable)
        {
            sendLocal({"VV;"});
        }
        // Explicitly set split flag for consistency with VFO A/B button logic
        const_cast<RadioManager *>(this)->updateSplitEnabled(true);

        // Query current FR/FT states directly from radio to confirm split activation
        sendRadioCommand("FR;");
        sendRadioCommand("FT;");
    }

    void RadioManager::disableSplit() const
    {
        // Disable split by making TX same as RX (simplex): FT<current RX>
        // Do not force RX to A; preserve the user's active RX VFO.
        const int rx = getRxVfo();
        if (rx == 0)
        {
            sendLocal("FT0;");
        }
        else if (rx == 1)
        {
            sendLocal("FT1;");
        }

        // Explicitly clear split flag for consistency with VFO A/B button logic
        const_cast<RadioManager *>(this)->updateSplitEnabled(false);

        // Query current FR/FT states directly from radio to confirm split deactivation
        sendRadioCommand("FR;");
        sendRadioCommand("FT;");
    }

    void RadioManager::toggleSplit(const bool copyVfoBeforeEnable) const
    {
        if (state_.isInSplitMode())
        {
            disableSplit();
        }
        else
        {
            enableSplit(copyVfoBeforeEnable);
        }
    }

    void RadioManager::setProcessorState(const int proc) const
    {
        // Update processor state via command system
        const std::string prCommand = "PR" + std::to_string(proc) + ";";
        (void)dispatchMessage(*localHandler_, prCommand);
    }

    void RadioManager::toggleDataMode() const
    {
        const int8_t current = state_.dataMode.load();
        const std::string daCommand = "DA" + std::to_string(current ^ 1) + ";";
        (void)dispatchMessage(*localHandler_, daCommand);
    }

    void RadioManager::setDataMode(const int8_t mode) const
    {
        const std::string daCommand = "DA" + std::to_string(mode) + ";";
        (void)dispatchMessage(*localHandler_, daCommand);
    }

    void RadioManager::setMode(const int mode) const
    {
        // Use command system for mode control
        const std::string mdCommand = "MD" + std::to_string(mode) + ";";
        (void)dispatchMessage(*localHandler_, mdCommand);
    }

    void RadioManager::setAfGain(const int gain) const
    {
        // Use command system for AF gain control
        // AG command format: AGP1P2P2P2; where P1=0, P2P2P2=000-255 (3 digits)
        const int clampedGain = std::max(0, std::min(255, gain));
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "AG0%03d;", clampedGain);
        const std::string agCommand = buffer;

        (void)dispatchMessage(*panelHandler_, agCommand);
    }

    void RadioManager::setRfGain(const int gain) const
    {
        // Use command system for RF gain control
        // RG command format: RGP1P1P1; where P1P1P1=000-255 (3 digits)
        const int clampedGain = std::max(0, std::min(255, gain));
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "RG%03d;", clampedGain);
        const std::string rgCommand = buffer;

        (void)dispatchMessage(*panelHandler_, rgCommand);
    }

    void RadioManager::setRitXitValue(const int value) const
    {
        // Use command system for RIT/XIT offset control
        std::string command;
        if (value > 0)
        {
            char buffer[16];
            snprintf(buffer, sizeof(buffer), "RU%05d;", value);
            command = buffer;
        }
        else if (value < 0)
        {
            char buffer[16];
            snprintf(buffer, sizeof(buffer), "RD%05d;", -value);
            command = buffer;
        }
        else
        {
            command = "RC;"; // Clear RIT/XIT
        }
        (void)dispatchMessage(*panelHandler_, command);
    }

    void RadioManager::setFilterWidth(const int delta) const
    {
        const int currentMode = state_.mode.load();

        // Mode-aware filter width control
        if (currentMode == 3 || currentMode == 7)
        { // CW or CW-R
            // FW command: CW bandwidth values in Hz (not index)
            // Valid: 50, 80, 100, 150, 200, 250, 300, 400, 500, 600, 1000, 1500, 2000, 2500
            static constexpr int CW_BANDWIDTHS[] = {50,  80,  100, 150,  200,  250,  300,
                                                    400, 500, 600, 1000, 1500, 2000, 2500};
            static constexpr int CW_BW_COUNT = sizeof(CW_BANDWIDTHS) / sizeof(CW_BANDWIDTHS[0]);

            // Find current index
            int currentBw = state_.dspFilterBandwidth;
            int currentIndex = 0;
            for (int i = 0; i < CW_BW_COUNT; i++)
            {
                if (CW_BANDWIDTHS[i] == currentBw)
                {
                    currentIndex = i;
                    break;
                }
            }

            // Adjust index
            int newIndex = currentIndex + delta;
            if (newIndex < 0)
                newIndex = 0;
            if (newIndex >= CW_BW_COUNT)
                newIndex = CW_BW_COUNT - 1;

            int newBandwidth = CW_BANDWIDTHS[newIndex];
            char buffer[16];
            snprintf(buffer, sizeof(buffer), "FW%04d;", newBandwidth);
            (void)dispatchMessage(*panelHandler_, buffer);
            // State will be updated when radio responds
        }
        else if (currentMode == 4)
        { // FM
            // Use FW command (0=Normal, 1=Narrow for FM)
            int newMode = state_.fmNarrowMode;
            if (delta > 0 && newMode == 1)
                newMode = 0; // Wider: Normal
            if (delta < 0 && newMode == 0)
                newMode = 1; // Narrower: Narrow

            char buffer[16];
            snprintf(buffer, sizeof(buffer), "FW%04d;", newMode);
            (void)dispatchMessage(*panelHandler_, buffer);
            // State will be updated when radio responds
        }
        else if (currentMode == 6 || currentMode == 9)
        { // FSK or FSK-R
            // FW command: FSK bandwidth values in Hz (not index)
            // Valid: 250, 500, 1000, 1500
            static constexpr int FSK_BANDWIDTHS[] = {250, 500, 1000, 1500};
            static constexpr int FSK_BW_COUNT = sizeof(FSK_BANDWIDTHS) / sizeof(FSK_BANDWIDTHS[0]);

            // Find current index
            int currentBw = state_.dspFilterBandwidth;
            int currentIndex = 0;
            for (int i = 0; i < FSK_BW_COUNT; i++)
            {
                if (FSK_BANDWIDTHS[i] == currentBw)
                {
                    currentIndex = i;
                    break;
                }
            }

            // Adjust index
            int newIndex = currentIndex + delta;
            if (newIndex < 0)
                newIndex = 0;
            if (newIndex >= FSK_BW_COUNT)
                newIndex = FSK_BW_COUNT - 1;

            int newBandwidth = FSK_BANDWIDTHS[newIndex];
            char buffer[16];
            snprintf(buffer, sizeof(buffer), "FW%04d;", newBandwidth);
            (void)dispatchMessage(*panelHandler_, buffer);
            // State will be updated when radio responds
        }
        // Note: LSB/USB/AM modes now use dedicated setHighCut()/setLowCut() methods
        // called directly from encoder handlers - no longer handled here
    }

    void RadioManager::setIfShift(const int valueHz)
    {
        const int currentMode = state_.mode.load();

        // IF Shift only available in CW/CW-R modes
        if (currentMode == 3 || currentMode == 7)
        { // CW or CW-R
            // Clamp to valid range (0-9999 Hz)
            int clampedValue = valueHz;
            if (clampedValue < 0)
                clampedValue = 0;
            if (clampedValue > 9999)
                clampedValue = 9999;

            char buffer[16];
            snprintf(buffer, sizeof(buffer), "IS %04d;", clampedValue);
            (void)dispatchMessage(*panelHandler_, buffer);

            // Show/update transient IF shift popup on display (2 second timeout)
            auto &ui = state_.uiState;
            if (ui.isActive() && ui.getActiveControl() == UIControl::IfShift)
            {
                // Already showing IF shift popup - just update value and reset timeout
                ui.currentValue.store(static_cast<int16_t>(clampedValue));
                ui.lastUpdateTime.store(esp_timer_get_time());
                sendUICommand(UICommandHandler::formatUIIS(clampedValue));
            }
            else
            {
                // Show new IF shift popup with short timeout
                enterUIMode(UIControl::IfShift, clampedValue, 0, 9999, 100, UIState::UI_TIMEOUT_SHORT_US);
            }
        }
        else
        {
            ESP_LOGW(TAG, "IF Shift only available in CW/CW-R modes (current mode: %d)", currentMode);
        }
    }

    void RadioManager::resetIfShift() const
    {
        const int currentMode = state_.mode.load();

        // Reset to center (0 Hz)
        if (currentMode == 3 || currentMode == 7)
        { // CW or CW-R
            (void)dispatchMessage(*panelHandler_, "IS 0000;");
        }
    }

    void RadioManager::setHighCut(const int delta) const
    {
        // Query current value from radio if we've never received it
        // (state initialized to 0, need actual radio value before first adjustment)
        if (state_.commandCache.get("SH") == 0)
        {
            ESP_LOGD(RadioManager::TAG, "SH state not initialized - querying radio before adjustment");
            (void)dispatchMessage(*panelHandler_, "SH;");
            // Radio will respond with current value, state will be updated by handler
            // This first encoder turn won't change anything, but subsequent turns will work correctly
            return;
        }

        // Adjust high cut (SH) - indices 0-99 map to per-mode frequency tables
        // Note: Not all indices are valid in all modes; radio will clamp to valid range
        // At mode boundaries, multiple encoder turns may be needed to reach next valid index
        int newHighCut = state_.receiveHighCut + delta;
        if (newHighCut < 0)
            newHighCut = 0;
        if (newHighCut > 99)
            newHighCut = 99;

        char buffer[16];
        snprintf(buffer, sizeof(buffer), "SH%02d;", newHighCut);
        (void)dispatchMessage(*panelHandler_, buffer);

        ESP_LOGD(RadioManager::TAG, "SH: %d + %d = %d", state_.receiveHighCut, delta, newHighCut);
        // State will be updated by radio's answer (via AI mode or explicit response)
    }

    void RadioManager::setLowCut(const int delta) const
    {
        // Query current value from radio if we've never received it
        // (state initialized to 0, need actual radio value before first adjustment)
        if (state_.commandCache.get("SL") == 0)
        {
            ESP_LOGD(RadioManager::TAG, "SL state not initialized - querying radio before adjustment");
            (void)dispatchMessage(*panelHandler_, "SL;");
            // Radio will respond with current value, state will be updated by handler
            // This first encoder turn won't change anything, but subsequent turns will work correctly
            return;
        }

        // Adjust low cut (SL) - indices 0-99 map to per-mode frequency tables
        // Note: Not all indices are valid in all modes; radio will clamp to valid range
        // At mode boundaries, multiple encoder turns may be needed to reach next valid index
        int newLowCut = state_.receiveLowCut + delta;
        if (newLowCut < 0)
            newLowCut = 0;
        if (newLowCut > 99)
            newLowCut = 99;

        char buffer[16];
        snprintf(buffer, sizeof(buffer), "SL%02d;", newLowCut);
        (void)dispatchMessage(*panelHandler_, buffer);

        ESP_LOGD(RadioManager::TAG, "SL: %d + %d = %d", state_.receiveLowCut, delta, newLowCut);
        // State will be updated by radio's answer (via AI mode or explicit response)
    }

    void RadioManager::initializeCommandHandlers()
    {
        ESP_LOGD(RadioManager::TAG, "Initializing command handlers");

        // Register command handlers
        if (auto frequencyVfoHandler = std::make_unique<FrequencyVfoCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(frequencyVfoHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register FrequencyVfoCommandHandler");
        }

        if (auto modeHandler = std::make_unique<ModeCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(modeHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register ModeCommandHandler");
        }

        if (auto gainLevelHandler = std::make_unique<GainLevelCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(gainLevelHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register GainLevelCommandHandler");
        }

        if (auto receiverProcessingHandler = std::make_unique<ReceiverProcessingCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(receiverProcessingHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register ReceiverProcessingCommandHandler");
        }

        if (auto transmitterHandler = std::make_unique<TransmitterCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(transmitterHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register TransmitterCommandHandler");
        }

        if (auto antennaHandler = std::make_unique<AntennaCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(antennaHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register AntennaCommandHandler");
        }

        if (auto cwHandler = std::make_unique<CwCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(cwHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register CwCommandHandler");
        }

        if (auto memoryHandler = std::make_unique<MemoryCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(memoryHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register MemoryCommandHandler");
        }

        if (auto scanHandler = std::make_unique<ScanCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(scanHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register ScanCommandHandler");
        }

        if (auto toneSquelchHandler = std::make_unique<ToneSquelchCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(toneSquelchHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register ToneSquelchCommandHandler");
        }

        if (auto audioEqualizerHandler = std::make_unique<AudioEqualizerCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(audioEqualizerHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register AudioEqualizerCommandHandler");
        }

        if (auto visualScanHandler = std::make_unique<VisualScanCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(visualScanHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register VisualScanCommandHandler");
        }

        if (auto menuConfigHandler = std::make_unique<MenuConfigCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(menuConfigHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register MenuConfigCommandHandler");
        }

        if (auto statusInfoHandler = std::make_unique<StatusInfoCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(statusInfoHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register StatusInfoCommandHandler");
        }

        if (auto interfaceSystemHandler = std::make_unique<InterfaceSystemCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(interfaceSystemHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register InterfaceSystemCommandHandler");
        }

        if (auto voiceMessageHandler = std::make_unique<VoiceMessageCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(voiceMessageHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register VoiceMessageCommandHandler");
        }

        if (auto uiHandler = std::make_unique<UICommandHandler>();
            !commandDispatcher_->registerHandler(std::move(uiHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register UICommandHandler");
        }

        if (auto mxHandler = std::make_unique<MXCommandHandler>();
            !commandDispatcher_->registerHandler(std::move(mxHandler)))
        {
            ESP_LOGE(RadioManager::TAG, "Failed to register MXCommandHandler");
        }

        ESP_LOGD(RadioManager::TAG, "Command dispatcher initialized with %zu handlers",
                 commandDispatcher_->getRegisteredHandlers().size());
        // Note: MacroStorage is initialized in RadioManager::init() after NVS is ready
        // Note: RadioMacroManager is created separately and set via setMacroManager()
    }

    void RadioManager::noteQueryOrigin(std::string_view prefix, CommandSource src, const uint64_t nowUs,
                                       const bool cacheServed)
    {
        if (prefix.size() < 2)
            return;
        const uint8_t id = originHash(prefix[0], prefix[1]);
        lastOriginTime_[id].store(nowUs, std::memory_order_relaxed);
        int srcVal = static_cast<int>(src);
        if (cacheServed)
            srcVal |= ORIGIN_CACHE_SERVED_BIT;
        lastOriginSrc_[id].store(srcVal, std::memory_order_relaxed);
        ESP_LOGV(RadioManager::TAG, "Recorded origin for %c%c -> %d%s at %llu us", prefix[0], prefix[1],
                 static_cast<int>(src), cacheServed ? " (cache-served)" : "",
                 static_cast<unsigned long long>(nowUs));
    }

    bool RadioManager::routeMatchedAnswer(std::string_view prefix, std::string_view response,
                                          const uint64_t nowUs) const
    {
        auto result = routeMatchedAnswerWithSource(prefix, response, nowUs);
        return result.has_value();
    }

    std::optional<CommandSource> RadioManager::routeMatchedAnswerWithSource(std::string_view prefix,
                                                                            std::string_view response,
                                                                            const uint64_t nowUs) const
    {
        if (prefix.size() < 2)
            return std::nullopt;
        const uint8_t id = originHash(prefix[0], prefix[1]);
        const uint64_t t = lastOriginTime_[id].load(std::memory_order_relaxed);
        if (t == 0 || (nowUs - t) > ORIGIN_TTL_US)
        {
            return std::nullopt; // stale or no origin
        }
        const int srcInt = lastOriginSrc_[id].load(std::memory_order_relaxed);
        const bool cacheServed = (srcInt & ORIGIN_CACHE_SERVED_BIT) != 0;
        const CommandSource src = static_cast<CommandSource>(srcInt & 0xFF);
        if (cacheServed)
        {
            // Client already received a cached response for this query.
            // Return the source (so AI forwarding is suppressed) but don't send again.
            ESP_LOGD(RadioManager::TAG, "Matched answer %c%c to origin=%d (cache-served, suppressing duplicate)",
                     prefix[0], prefix[1], static_cast<int>(src));
            return src;
        }
        ESP_LOGD(RadioManager::TAG, "Routing matched answer %c%c to origin=%d", prefix[0], prefix[1],
                 static_cast<int>(src));
        sendToSource(src, response);
        return src;
    }

    void RadioManager::txTimeoutTask(void *pvParameters)
    {
        auto *radioManager = static_cast<RadioManager *>(pvParameters);
        ESP_LOGI(RadioManager::TAG, "TX timeout monitoring task started (check interval: 1s)");

        const TickType_t interval = pdMS_TO_TICKS(1000); // Check every 1 second for faster recovery

        uint32_t minWatermark = UINT32_MAX;
        while (true)
        {
            vTaskDelay(interval);

            const uint64_t currentTime = esp_timer_get_time();

#if (INCLUDE_uxTaskGetStackHighWaterMark == 1)
            if (const uint32_t watermark = uxTaskGetStackHighWaterMark(nullptr); watermark < minWatermark)
            {
                minWatermark = watermark;
                ESP_LOGW(RadioManager::TAG, "tx_timeout stack watermark: %lu words",
                         static_cast<unsigned long>(watermark));
            }
#endif

            // Check for TX timeout and force release if needed
            if (radioManager->getState().isTxTimedOut(currentTime))
            {
                ESP_LOGW(RadioManager::TAG, "⏰ TX timeout detected (owner: %d), force releasing TX",
                         radioManager->getState().getTxOwner());
                if (radioManager->getState().forceReleaseTx(currentTime))
                {
                    ESP_LOGW(RadioManager::TAG, "🚨 TX timeout recovery: issuing emergency RX command");
                    bool mirrored = false;
                    if (radioManager->panelHandler_)
                    {
                        mirrored = radioManager->dispatchMessage(*radioManager->panelHandler_, "RX;");
                    }
                    if (!mirrored)
                    {
                        ESP_LOGW(RadioManager::TAG, "Emergency RX fallback: direct send");
                        radioManager->radioSerial_.sendMessage("RX;");
                        radioManager->sendToSource(CommandSource::UsbCdc0, "RX;");
                        radioManager->sendToSource(CommandSource::UsbCdc1, "RX;");
                        radioManager->sendToSource(CommandSource::Display, "RX;");
                    }
                    radioManager->forceReleasePrimaryControl();
                }
            }

            // Poll IF status frequently while TX is active to detect RX transition promptly
            // Display expects RX confirmation within ~2s, so poll every 500ms
            static uint64_t lastTxPollTime = 0;
            auto &state = radioManager->getState();
            if (state.isTx.load())
            {
                constexpr uint64_t pollIntervalUs = 500000ULL; // 500ms between polls
                if (currentTime - lastTxPollTime > pollIntervalUs)
                {
                    const bool freshIf = state.commandCache.isFresh("IF", currentTime, 400000ULL);
                    if (!freshIf)
                    {
                        ESP_LOGD(RadioManager::TAG, "🔍 TX active: polling radio status for RX transition");
                        // Send directly to radio to bypass handleLocalQueryStandard's TX-mode cache
                        // The dispatcher's TX-mode optimization would just return cached state
                        // without actually querying the radio, so we'd never detect RX transition
                        state.queryTracker.recordQuery("IF", currentTime);
                        radioManager->radioSerial_.sendMessage("IF;");
                    }
                    lastTxPollTime = currentTime;
                }
            }
            else
            {
                lastTxPollTime = 0;
            }
        }
    }

#ifdef CONFIG_RADIOCORE_MANUAL_KEEPALIVE
    void RadioManager::keepaliveTask(void *pvParameters)
    {
        auto *radioManager = static_cast<RadioManager *>(pvParameters);
        ESP_LOGI(RadioManager::TAG, "Manual keepalive task started (interval: %dms)",
                 CONFIG_RADIOCORE_MANUAL_KEEPALIVE_TIME);

        const TickType_t interval = pdMS_TO_TICKS(CONFIG_RADIOCORE_MANUAL_KEEPALIVE_TIME);

        uint64_t lastPsKeepalive = 0;
        while (true)
        {
            vTaskDelay(interval);

            const uint64_t currentTime = esp_timer_get_time();

            // Check for TX timeout and force release if needed (redundant check for safety)
            if (radioManager->getState().isTxTimedOut(currentTime))
            {
                ESP_LOGW(RadioManager::TAG, "⏰ TX timeout detected in keepalive, force releasing TX");
                if (radioManager->getState().forceReleaseTx(currentTime))
                {
                    ESP_LOGW(RadioManager::TAG, "🚨 TX timeout recovery (keepalive): issuing emergency RX command");
                    bool mirrored = false;
                    if (radioManager->panelHandler_)
                    {
                        mirrored = radioManager->dispatchMessage(*radioManager->panelHandler_, "RX;");
                    }
                    if (!mirrored)
                    {
                        ESP_LOGW(RadioManager::TAG, "Emergency RX fallback (keepalive): direct send");
                        radioManager->radioSerial_.sendMessage("RX;");
                        radioManager->sendToSource(CommandSource::UsbCdc0, "RX;");
                        radioManager->sendToSource(CommandSource::UsbCdc1, "RX;");
                        radioManager->sendToSource(CommandSource::Display, "RX;");
                    }
                    radioManager->forceReleasePrimaryControl();
                }
            }

            // Only send PS1; if interface is powered on and radio not actively transmitting
            auto &state = radioManager->getState();
            if (!state.isTx.load() && state.powerOn.load())
            {
                if (currentTime - lastPsKeepalive >= 5000000ULL)
                { // 5s spacing
                    ESP_LOGV(RadioManager::TAG, "Sending manual keepalive PS1; to radio");
                    radioManager->radioSerial_.sendMessage("PS1;");
                    lastPsKeepalive = currentTime;
                }
            }
            else
            {
                lastPsKeepalive = currentTime;
            }
        }
    }
#endif

    // =============================================================================
    // UI Mode Methods (Panel-Display Interaction)
    // =============================================================================

    void RadioManager::enterUIMode(UIControl control, int initialValue, int minValue, int maxValue, int stepSize,
                                    uint64_t timeoutUs)
    {
        auto &ui = state_.uiState;
        const uint64_t now = esp_timer_get_time();

        ui.activeControl.store(static_cast<uint8_t>(control));
        ui.currentValue.store(static_cast<int16_t>(initialValue));
        ui.minValue.store(static_cast<int16_t>(minValue));
        ui.maxValue.store(static_cast<int16_t>(maxValue));
        ui.stepSize.store(static_cast<int16_t>(stepSize));
        ui.activationTime.store(now);
        ui.lastUpdateTime.store(now);
        ui.timeoutUs.store(timeoutUs > 0 ? timeoutUs : UIState::UI_TIMEOUT_DEFAULT_US);

        ESP_LOGI(TAG, "Entering UI mode: control=%d, value=%d, range=[%d-%d], step=%d, timeout=%llums",
                 static_cast<int>(control), initialValue, minValue, maxValue, stepSize,
                 ui.timeoutUs.load() / 1000);

        // Send initial UI command to display
        switch (control)
        {
        case UIControl::Power:
            sendUICommand(UICommandHandler::formatUIPC(initialValue));
            break;
        case UIControl::CarrierLevel:
            sendUICommand(UICommandHandler::formatUIML(initialValue));
            break;
        case UIControl::NrLevel:
            sendUICommand(UICommandHandler::formatUIRL(initialValue));
            break;
        case UIControl::Nr2Speed:
            sendUICommand(UICommandHandler::formatUIRS(initialValue));
            break;
        case UIControl::NbLevel:
            sendUICommand(UICommandHandler::formatUINL(initialValue));
            break;
        case UIControl::ProcInputLevel:
            sendUICommand(UICommandHandler::formatUIPI(initialValue));
            break;
        case UIControl::ProcOutputLevel:
            sendUICommand(UICommandHandler::formatUIPO(initialValue));
            break;
        case UIControl::DataMode:
            sendUICommand(UICommandHandler::formatUIDA(initialValue));
            break;
        case UIControl::NotchFrequency:
            sendUICommand(UICommandHandler::formatUINF(initialValue));
            break;
        case UIControl::IfShift:
            sendUICommand(UICommandHandler::formatUIIS(initialValue));
            break;
        case UIControl::RitXitOffset:
            sendUICommand(UICommandHandler::formatUIRI(initialValue));
            break;
        case UIControl::KeyingSpeed:
            sendUICommand(UICommandHandler::formatUIKS(initialValue));
            break;
        default:
            // Send generic menu active signal
            sendUICommand(UICommandHandler::formatUIMN(true));
            break;
        }
    }

    void RadioManager::exitUIMode(bool applyValue)
    {
        auto &ui = state_.uiState;

        if (!ui.isActive())
        {
            return;
        }

        const UIControl control = ui.getActiveControl();
        const int value = ui.currentValue.load();

        ESP_LOGI(TAG, "Exiting UI mode: control=%d, value=%d, apply=%s",
                 static_cast<int>(control), value, applyValue ? "yes" : "no");

        if (applyValue)
        {
            // Apply the value by sending actual CAT command to radio
            // Note: Power, NR, NB, PROC, NOTCH, and IF Shift apply values in real-time during adjustment,
            // so we only need to send commands for controls that defer application until confirm
            char cmdBuf[16];
            switch (control)
            {
            case UIControl::CarrierLevel:
                snprintf(cmdBuf, sizeof(cmdBuf), "ML%03d;", value);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::AfGain:
                snprintf(cmdBuf, sizeof(cmdBuf), "AG0%03d;", value);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::RfGain:
                snprintf(cmdBuf, sizeof(cmdBuf), "RG%03d;", value);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::MicGain:
                snprintf(cmdBuf, sizeof(cmdBuf), "MG%03d;", value);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            // Power, NrLevel, Nr2Speed, NbLevel, ProcInputLevel, ProcOutputLevel, NotchFrequency, IfShift, RitXitOffset
            // are applied in real-time during adjustUIValue() - no action needed here
            case UIControl::Power:
            case UIControl::NrLevel:
            case UIControl::Nr2Speed:
            case UIControl::NbLevel:
            case UIControl::ProcInputLevel:
            case UIControl::ProcOutputLevel:
            case UIControl::DataMode:
            case UIControl::NotchFrequency:
            case UIControl::IfShift:
            case UIControl::RitXitOffset:
            case UIControl::KeyingSpeed:
                // Already applied in real-time during adjustment
                break;
            default:
                break;
            }
        }

        // Clear UI state
        ui.clear();

        // Notify display to dismiss UI overlay
        sendUICommand(UICommandHandler::formatUIMN(false));
    }

    int RadioManager::adjustUIValue(int delta)
    {
        auto &ui = state_.uiState;

        if (!ui.isActive())
        {
            return 0;
        }

        const int16_t step = ui.stepSize.load();
        const int16_t minVal = ui.minValue.load();
        const int16_t maxVal = ui.maxValue.load();
        int16_t current = ui.currentValue.load();

        // Apply delta with step size
        int16_t newValue = current + static_cast<int16_t>(delta * step);

        // Clamp to valid range
        if (newValue < minVal)
            newValue = minVal;
        if (newValue > maxVal)
            newValue = maxVal;

        if (newValue != current)
        {
            ui.currentValue.store(newValue);
            ui.lastUpdateTime.store(esp_timer_get_time());

            ESP_LOGI(TAG, "UI value adjusted: %d -> %d (delta=%d, step=%d)", current, newValue, delta, step);

            // Send updated value to display (and for some controls, also to radio immediately)
            const UIControl control = ui.getActiveControl();
            char cmdBuf[16];
            switch (control)
            {
            case UIControl::Power:
                sendUICommand(UICommandHandler::formatUIPC(newValue));
                // Apply immediately to radio for real-time feedback
                state_.transmitPower = newValue;
                snprintf(cmdBuf, sizeof(cmdBuf), "PC%03d;", newValue);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::CarrierLevel:
                sendUICommand(UICommandHandler::formatUIML(newValue));
                break;
            case UIControl::NrLevel:
                sendUICommand(UICommandHandler::formatUIRL(newValue));
                // Apply immediately to radio for real-time feedback
                state_.nr1Level = newValue;
                snprintf(cmdBuf, sizeof(cmdBuf), "RL%02d;", newValue);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::Nr2Speed:
                sendUICommand(UICommandHandler::formatUIRS(newValue));
                // Apply immediately to radio for real-time feedback
                state_.nr2Speed = newValue;
                snprintf(cmdBuf, sizeof(cmdBuf), "RL%02d;", newValue);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::NbLevel:
                sendUICommand(UICommandHandler::formatUINL(newValue));
                // Apply immediately to radio for real-time feedback
                snprintf(cmdBuf, sizeof(cmdBuf), "NL%03d;", newValue);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::ProcInputLevel:
                sendUICommand(UICommandHandler::formatUIPI(newValue));
                // Apply immediately to radio for real-time feedback
                state_.speechProcessorInLevel = newValue;
                snprintf(cmdBuf, sizeof(cmdBuf), "PL%03d%03d;", newValue, state_.speechProcessorOutLevel);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::ProcOutputLevel:
                sendUICommand(UICommandHandler::formatUIPO(newValue));
                // Apply immediately to radio for real-time feedback
                state_.speechProcessorOutLevel = newValue;
                snprintf(cmdBuf, sizeof(cmdBuf), "PL%03d%03d;", state_.speechProcessorInLevel, newValue);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::DataMode:
                {
                    // Data mode is a toggle (0=OFF, 1=ON)
                    // newValue is already the toggled state from encoder (0→1 or 1→0)

                    // Send to display
                    sendUICommand(UICommandHandler::formatUIDA(newValue));

                    // Apply to radio
                    snprintf(cmdBuf, sizeof(cmdBuf), "DA%d;", newValue);
                    dispatchMessage(*panelHandler_, cmdBuf);

                    // If enabling DATA mode, disable PROC immediately
                    if (newValue == 1)
                    {
                        ESP_LOGI(RadioManager::TAG, "DATA mode enabled - disabling PROC");
                        dispatchMessage(*panelHandler_, "PR0;");
                    }

                    ESP_LOGD(RadioManager::TAG, "DATA mode set to: %d", newValue);
                }
                break;
            case UIControl::NotchFrequency:
                sendUICommand(UICommandHandler::formatUINF(newValue));
                // Apply immediately to radio for real-time feedback
                state_.manualNotchFrequency = newValue;
                snprintf(cmdBuf, sizeof(cmdBuf), "BP%03d;", newValue);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::IfShift:
                sendUICommand(UICommandHandler::formatUIIS(newValue));
                // IF shift is applied in real-time via setIfShift(), which also updates display
                // This case handles manual UI adjustment if needed
                state_.ifShiftValue.store(newValue);
                snprintf(cmdBuf, sizeof(cmdBuf), "IS %04d;", newValue);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::RitXitOffset:
                sendUICommand(UICommandHandler::formatUIRI(newValue));
                // Apply RIT/XIT offset immediately to radio for real-time feedback
                // RU/RD with param = CHANGE offset by that amount (not set absolute)
                // RU00010; adds +10 Hz to current offset, RD00010; subtracts 10 Hz
                state_.ritXitOffset.store(newValue);
                // delta > 0 means moving positive (towards higher values)
                // delta < 0 means moving negative (towards lower values)
                if (delta > 0)
                {
                    // Moving positive direction - use RU with step amount
                    snprintf(cmdBuf, sizeof(cmdBuf), "RU%05d;", step);
                }
                else
                {
                    // Moving negative direction - use RD with step amount
                    snprintf(cmdBuf, sizeof(cmdBuf), "RD%05d;", step);
                }
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            case UIControl::KeyingSpeed:
                sendUICommand(UICommandHandler::formatUIKS(newValue));
                // Apply immediately to radio for real-time feedback
                state_.keyingSpeed = newValue;
                snprintf(cmdBuf, sizeof(cmdBuf), "KS%03d;", newValue);
                dispatchMessage(*panelHandler_, cmdBuf);
                break;
            default:
                break;
            }
        }

        return static_cast<int>(newValue);
    }

    void RadioManager::sendUICommand(std::string_view command) const
    {
        if (displaySerial_)
        {
            ESP_LOGD(TAG, "Sending UI command to display: %.*s", static_cast<int>(command.size()), command.data());
            displaySerial_->sendMessage(command);
        }
    }

    void RadioManager::checkUITimeout()
    {
        auto &ui = state_.uiState;

        if (!ui.isActive())
        {
            return;
        }

        const uint64_t now = esp_timer_get_time();
        if (ui.isTimedOut(now))
        {
            ESP_LOGI(TAG, "UI mode timed out - dismissing without applying");
            exitUIMode(false); // Dismiss without applying
        }
    }

} // namespace radio

// ── EX menu NVS helpers (bridge between NvsManager and ExtendedMenuState) ──
// These free functions are called by NvsManager via extern declarations.
// They live here because RadioCore has both CommandHandlers and NvsManager as deps.

esp_err_t packAndSaveExMenu(NvsManager& nvs) {
    auto& menuState = radio::ExtendedCommandHandler::getExtendedMenuState();

    NvsManager::ExNvsData data{};
    data.version = 1;
    memset(data.values, 0, sizeof(data.values));

    size_t populated = 0;
    for (size_t i = 0; i < 100; i++) {
        char key[4];
        snprintf(key, sizeof(key), "%03u", static_cast<unsigned>(i));
        std::string_view val = menuState.getValue(key);
        if (!val.empty() && val.size() < 8) {
            memcpy(data.values[i], val.data(), val.size());
            data.values[i][val.size()] = '\0';
            populated++;
        }
    }

    esp_err_t err = nvs.saveExMenuBlob(data);
    if (err == ESP_OK) {
        ESP_LOGI("NvsManager", "EX menu saved to NVS (%zu populated values)", populated);
    }
    return err;
}

esp_err_t loadAndUnpackExMenu(NvsManager& nvs) {
    NvsManager::ExNvsData data{};
    esp_err_t err = nvs.loadExMenuBlob(data);
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGI("NvsManager", "No EX menu in NVS (first boot) - using defaults");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    if (data.version != 1) {
        ESP_LOGW("NvsManager", "Unknown EX menu NVS version %u - ignoring", data.version);
        return ESP_OK;
    }

    auto& menuState = radio::ExtendedCommandHandler::getExtendedMenuState();
    size_t loaded = 0;
    for (size_t i = 0; i < 100; i++) {
        data.values[i][7] = '\0'; // ensure null-termination
        if (data.values[i][0] != '\0') {
            char key[4];
            snprintf(key, sizeof(key), "%03u", static_cast<unsigned>(i));
            menuState.setValue(key, data.values[i]);
            loaded++;
        }
    }

    ESP_LOGI("NvsManager", "Loaded %zu EX menu values from NVS", loaded);
    return ESP_OK;
}
