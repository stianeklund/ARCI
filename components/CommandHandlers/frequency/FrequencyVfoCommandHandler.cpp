#include "FrequencyVfoCommandHandler.h"
#include <algorithm>
#include <charconv>
#include "DisplayLatencyProfiler.h"
#include "RadioManager.h"
#include "esp_log.h"

// Compile-time toggle for transverter display behavior
// Enable via sdkconfig option CONFIG_TRANSVERTER_DISPLAY_OFFSET (menuconfig)
//  - When enabled: FA/FB local replies and forwarded answers include XO offset for USB display,
//    and local Set commands translate display -> radio base frequency.
//  - When disabled: Strict TS-590SG behavior (no translation).
namespace radio
{
    FrequencyVfoCommandHandler::FrequencyVfoCommandHandler() :
        BaseCommandHandler({"FA", "FB", "DN", "UP", "FR", "FT", "FS", "TS", "SP", "RC", "RT", "XT", "RD", "RU", "RO",
                            "VV", "XO", "EM", "CH"},
                           "Frequency & VFO Control Commands")
    {
        ESP_LOGD(TAG, "FrequencyVfoCommandHandler created");
    }

    bool FrequencyVfoCommandHandler::handleCommand(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                                   ISerialChannel &usbSerial, RadioManager &rm)
    {

        // VFO control commands
        if (cmd.command == "FA")
        {
            return handleFA(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "FB")
        {
            return handleFB(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "FR")
        {
            return handleFR(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "FT")
        {
            return handleFT(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "FS")
        {
            return handleFS(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "SP")
        {
            return handleSP(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "UP")
        {
            return handleUP(cmd, radioSerial, rm);
        }
        if (cmd.command == "DN")
        {
            return handleDN(cmd, radioSerial, rm);
        }
        // RIT/XIT commands
        if (cmd.command == "RT")
        {
            return handleRT(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "XT")
        {
            return handleXT(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "RC")
        {
            return handleRC(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "RU")
        {
            return handleRU(cmd, radioSerial, rm);
        }
        if (cmd.command == "RD")
        {
            return handleRD(cmd, radioSerial, rm);
        }
        if (cmd.command == "RO")
        {
            return handleRO(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "TS")
        {
            return handleTS(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "VV")
        {
            return handleVV(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "XO")
        {
            return handleXO(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "EM")
        {
            return handleEM(cmd, radioSerial, usbSerial, rm);
        }
        if (cmd.command == "CH")
        {
            return handleCH(cmd, radioSerial, usbSerial, rm);
        }
        return false;
    }

    // =============================================================================
    // Frequency Commands (FA, FB)
    // =============================================================================

    bool FrequencyVfoCommandHandler::handleFA(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            // FA; - Query VFO A frequency
            ESP_LOGD(TAG, "🔍 FA Query received - source: %s", cmd.isLocal() ? "LOCAL" : "REMOTE");

            // Block external USB queries during tuning to prevent interference
            /*if (rm.getState().isTuning.load() && cmd.isUsb()) {
                ESP_LOGW(TAG, "Blocking FA query from USB - radio is tuning");
                return false;
            }
            */

            if (cmd.isCatClient())
            {
                const bool cacheFresh = isCacheFresh(rm, "FA", TTL_REALTIME);
                const bool txActive = rm.getState().isTx.load();
                const uint64_t cachedFreq = rm.getVfoAFrequency();
                ESP_LOGD(TAG, "FA query source=%d cacheFresh=%s cached=%llu tx=%s", static_cast<int>(cmd.source),
                         cacheFresh ? "yes" : "no", static_cast<unsigned long long>(cachedFreq),
                         txActive ? "yes" : "no");

                bool respondedFromCache = false;
                if (cachedFreq > 0)
                {
                    const uint64_t freq = rm.baseToDisplayFrequency(cachedFreq);
                    if (cachedFreq != freq)
                    {
                        ESP_LOGD(TAG, "FA applied transverter offset: %llu -> %llu", cachedFreq, freq);
                    }
                    static thread_local FrequencyFormatter formatter;
                    const std::string response = buildCommand("FA", formatter.format(freq));
                    ESP_LOGD(TAG, "FA serving cached frequency=%llu", static_cast<unsigned long long>(freq));
                    respondToSource(cmd, response, usbSerial, rm);
                    rm.getState().commandCache.update("FA", esp_timer_get_time());
                    respondedFromCache = true;
                }

                if (txActive)
                {
                    if (!respondedFromCache)
                    {
                        ESP_LOGW(TAG, "FA query during TX without cached data - skipping radio query");
                    }
                    else if (!cacheFresh)
                    {
                        ESP_LOGD(TAG, "FA query during TX: returned cached value without radio refresh");
                    }
                    return true;
                }

                if (respondedFromCache && cacheFresh)
                {
                    return true;
                }

                if (!respondedFromCache)
                {
                    const uint64_t timestamp = esp_timer_get_time();
                    ESP_LOGD(TAG, "FA cache empty - querying radio for authoritative value");
                    rm.getState().queryTracker.recordQuery("FA", timestamp);
                    rm.noteQueryOrigin("FA", cmd.source, timestamp);
                    sendToRadio(radioSerial, cmd.originalMessage);
                }
                else
                {
                    const uint64_t timestamp = esp_timer_get_time();
                    ESP_LOGD(TAG, "FA cache served but stale - refreshing from radio");
                    rm.getState().queryTracker.recordQuery("FA", timestamp);
                    rm.noteQueryOrigin("FA", cmd.source, timestamp, true);
                    sendToRadio(radioSerial, cmd.originalMessage);
                }
            }
            else
            {
                ESP_LOGD(TAG, "FA non-local query forwarded to radio");
                rm.getState().queryTracker.recordQuery("FA", esp_timer_get_time());
                sendToRadio(radioSerial, cmd.originalMessage);
            }
            return true;
        }

        if (isSet(cmd))
        {
            // FA[frequency]; - Set VFO A frequency
            if (cmd.paramsEmpty())
            {
                ESP_LOGW(TAG, "FA set cmd missing frequency parameter");
                return false;
            }

            if (!rm.acquirePrimaryControl(cmd.source))
            {
                ESP_LOGW(TAG, "FA control lease denied for source %d (owner=%d)", static_cast<int>(cmd.source),
                         rm.currentPrimaryControlOwner());

                if (cmd.isLocal())
                {
                    const uint64_t baseFreq = rm.getVfoAFrequency();
                    const uint64_t freq = rm.baseToDisplayFrequency(baseFreq);
                    if (baseFreq != freq)
                    {
                        ESP_LOGD(TAG, "FA lease denied response offset: %llu -> %llu", baseFreq, freq);
                    }
                    static thread_local FrequencyFormatter formatter;
                    const std::string response = buildCommand("FA", formatter.format(freq));
                    respondToSource(cmd, response, usbSerial, rm);
                }
                return true;
            }

            // During tuning, suppress FA SET commands from AI2 clients to prevent feedback loops
            // AI2 clients receive frequency broadcasts from radio that get parsed as SET commands
            if (rm.getState().isTuning.load())
            {
                bool isAI2Client = false;
                if (cmd.source == CommandSource::UsbCdc0)
                {
                    isAI2Client = (rm.getState().usbCdc0AiMode.load() >= 2);
                }
                else if (cmd.source == CommandSource::UsbCdc1)
                {
                    isAI2Client = (rm.getState().usbCdc1AiMode.load() >= 2);
                }
                else if (cmd.source == CommandSource::Display)
                {
                    isAI2Client = (rm.getState().displayAiMode.load() >= 2);
                }

                if (isAI2Client)
                {
                    ESP_LOGD(TAG, "FA: Suppressing SET command from AI2 client during tuning (source=%d)",
                             static_cast<int>(cmd.source));
                    return true; // Suppress but return success
                }
            }

            const uint64_t frequency = parseFrequency(cmd, 0);
            // Interpret incoming local value as display-space when transverter is enabled
            const uint64_t radioFrequency = rm.displayToBaseFrequency(frequency);
            if (radioFrequency != frequency)
            {
                ESP_LOGD(TAG, "FA converted transverter frequency: %llu (display) -> %llu (radio)", frequency,
                         radioFrequency);
            }

            // Validate the actual radio/base frequency
            if (!isValidFrequency(radioFrequency))
            {
                ESP_LOGW(TAG, "Invalid frequency in FA cmd: %llu", radioFrequency);
                return false;
            }

            // Forward to radio if from local source
            if (cmd.shouldSendToRadio())
            {
                // Track this as a pending command if from Panel (encoder/buttons)
                if (cmd.source == CommandSource::Panel)
                {
                    const uint64_t now = esp_timer_get_time();
                    rm.getState().vfoAFrequencyPending.store(true);
                    rm.getState().vfoAPendingTime.store(now);
                    rm.getState().vfoAPendingValue.store(radioFrequency);
                    ESP_LOGD(TAG, "FA SET from Panel: marked as pending (freq=%llu, time=%llu)", radioFrequency, now);
                }

                if (radioFrequency != frequency)
                {
                    // Send converted frequency to radio - zero-allocation path
                    static thread_local FrequencyFormatter formatter;
                    const char* radioCmd = formatter.formatCommand("FA", radioFrequency);
                    sendToRadio(radioSerial, radioCmd);
                    ESP_LOGV(TAG, "Sent converted frequency to radio: %s", radioCmd);
                }
                else
                {
                    sendToRadio(radioSerial, cmd.originalMessage);
                    ESP_LOGV(TAG, "Sent to radio: %s", cmd.originalMessage.c_str());
                }
            }

            // Update internal state with the actual radio frequency (not the display frequency)
            rm.updateVfoAFrequency(radioFrequency);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            auto &profiler = DisplayLatencyProfiler::instance();
            profiler.markFaAnswerStart();

            ESP_LOGV(TAG, "FA answer received: %s", cmd.originalMessage.c_str());

            // FA[frequency]; - Answer from radio
            const uint64_t frequency = parseFrequency(cmd, 0);
            if (frequency == 0)
            {
                ESP_LOGW(TAG, "Failed to parse FA answer frequency");
                return false;
            }

            // During active local tuning, keep ESP32S3 authoritative: ignore radio freq updates
            const auto &state = rm.getState();
            if (state.isTuning.load())
            {
                ESP_LOGD(TAG, "FA during tuning: ignoring radio frequency update to prevent flicker");
                profiler.markDisplaySuppressed(DisplayLatencyProfiler::SuppressReason::IsTuning);
                return true; // Suppress both state update and routing while tuning
            }

            // Grace period after tuning stops to ignore stale in-flight responses
            const uint64_t tuningStopTime = state.tuningStopTime.load();
            if (tuningStopTime > 0)
            {
                const uint64_t now = esp_timer_get_time();
                const uint64_t timeSinceTuningStop = now - tuningStopTime;
                constexpr uint64_t GRACE_PERIOD_US = 500000; // 500ms grace period
                if (timeSinceTuningStop < GRACE_PERIOD_US)
                {
                    ESP_LOGD(TAG, "FA in grace period (%llu us after tuning): ignoring to prevent jump-back",
                             timeSinceTuningStop);
                    profiler.markDisplaySuppressed(DisplayLatencyProfiler::SuppressReason::GracePeriod);
                    return true; // Ignore stale responses during grace period
                }
            }

            // Check if we have a pending FA SET command from Panel
            const uint64_t now = esp_timer_get_time();
            const bool isPending = rm.getState().vfoAFrequencyPending.load();
            const uint64_t pendingTime = rm.getState().vfoAPendingTime.load();
            const uint64_t pendingValue = rm.getState().vfoAPendingValue.load();

            if (isPending && pendingTime > 0)
            {
                const uint64_t timeSincePending = now - pendingTime;
                constexpr uint64_t PENDING_TIMEOUT_US = 2000000; // 2 seconds

                if (timeSincePending < PENDING_TIMEOUT_US)
                {
                    // We're within the pending window
                    if (frequency == pendingValue)
                    {
                        // Radio confirmed our SET command
                        ESP_LOGD(TAG, "FA answer confirmed pending value %llu (took %llu us)", frequency,
                                 timeSincePending);
                        rm.getState().vfoAFrequencyPending.store(false);
                        rm.getState().vfoAPendingTime.store(0);
                        rm.getState().vfoAPendingValue.store(0);
                    }
                    else
                    {
                        // Radio sent a different value - this is likely a stale response
                        ESP_LOGD(TAG, "FA answer %llu doesn't match pending %llu - ignoring stale response", frequency,
                                 pendingValue);
                        profiler.markDisplaySuppressed(DisplayLatencyProfiler::SuppressReason::PendingValidation);
                        return true; // Ignore this stale response
                    }
                }
                else
                {
                    // Timeout - clear pending and accept radio's value
                    ESP_LOGD(TAG, "FA pending timeout (%llu us) - accepting radio value", timeSincePending);
                    rm.getState().vfoAFrequencyPending.store(false);
                    rm.getState().vfoAPendingTime.store(0);
                    rm.getState().vfoAPendingValue.store(0);
                }
            }

            // Update state after validation
            rm.updateVfoAFrequency(frequency);
            rm.getState().commandCache.update("FA", esp_timer_get_time());
            profiler.markFaStateUpdated();
            ESP_LOGD(TAG, "FA updated state with frequency: %llu", frequency);

            // Build adjusted response (apply transverter offset once) and route by policy
            {
                const uint64_t displayFreq = rm.baseToDisplayFrequency(frequency);
                static thread_local FrequencyFormatter formatter;
                const std::string response = buildCommand("FA", formatter.format(displayFreq));
                routeAnswerResponse(cmd, response, usbSerial, rm);
            }

            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleFB(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            // FB; - Query VFO B frequency

            /*// Block external USB queries during tuning to prevent interference
            if (rm.getState().isTuning.load() && cmd.isUsb()) {
                ESP_LOGW(TAG, "Blocking FB query from USB - radio is tuning");
                return false;
            }*/

            if (cmd.isCatClient())
            {
                const bool cacheFresh = isCacheFresh(rm, "FB", TTL_REALTIME);
                const bool txActive = rm.getState().isTx.load();
                const uint64_t cachedFreq = rm.getVfoBFrequency();
                ESP_LOGD(TAG, "FB query source=%d cacheFresh=%s cached=%llu tx=%s", static_cast<int>(cmd.source),
                         cacheFresh ? "yes" : "no", static_cast<unsigned long long>(cachedFreq),
                         txActive ? "yes" : "no");

                bool respondedFromCache = false;
                if (cachedFreq > 0)
                {
                    const uint64_t freq = rm.baseToDisplayFrequency(cachedFreq);
                    if (cachedFreq != freq)
                    {
                        ESP_LOGD(TAG, "FB applied transverter offset: %llu -> %llu", cachedFreq, freq);
                    }
                    static thread_local FrequencyFormatter formatter;
                    const std::string response = buildCommand("FB", formatter.format(freq));
                    ESP_LOGD(TAG, "FB serving cached frequency=%llu", static_cast<unsigned long long>(freq));
                    respondToSource(cmd, response, usbSerial, rm);
                    rm.getState().commandCache.update("FB", esp_timer_get_time());
                    respondedFromCache = true;
                }

                if (txActive)
                {
                    if (!respondedFromCache)
                    {
                        ESP_LOGW(TAG, "FB query during TX without cached data - skipping radio query");
                    }
                    else if (!cacheFresh)
                    {
                        ESP_LOGD(TAG, "FB query during TX: returned cached value without radio refresh");
                    }
                    return true;
                }

                if (respondedFromCache && cacheFresh)
                {
                    return true;
                }

                const uint64_t now = esp_timer_get_time();
                rm.getState().queryTracker.recordQuery("FB", now);
                if (respondedFromCache)
                {
                    ESP_LOGD(TAG, "FB cache served but stale - refreshing from radio");
                    rm.noteQueryOrigin("FB", cmd.source, now, true);
                }
                else
                {
                    ESP_LOGD(TAG, "FB cache empty - querying radio for authoritative value");
                    rm.noteQueryOrigin("FB", cmd.source, now);
                }
                sendToRadio(radioSerial, buildCommand("FB"));
            }
            else
            {
                const uint64_t now = esp_timer_get_time();
                rm.getState().queryTracker.recordQuery("FB", now);
                rm.noteQueryOrigin("FB", cmd.source, now);
                ESP_LOGD(TAG, "FB non-local query forwarded to radio");
                sendToRadio(radioSerial, cmd.originalMessage);
            }
            return true;
        }

        if (isSet(cmd))
        {
            // FB[frequency]; - Set VFO B frequency
            if (cmd.paramsEmpty())
            {
                ESP_LOGW(TAG, "FB set command missing frequency parameter");
                return false;
            }

            if (!rm.acquirePrimaryControl(cmd.source))
            {
                ESP_LOGW(TAG, "FB control lease denied for source %d (owner=%d)", static_cast<int>(cmd.source),
                         rm.currentPrimaryControlOwner());

                if (cmd.isLocal())
                {
                    const uint64_t baseFreq = rm.getVfoBFrequency();
                    const uint64_t freq = rm.baseToDisplayFrequency(baseFreq);
                    if (baseFreq != freq)
                    {
                        ESP_LOGD(TAG, "FB lease denied response offset: %llu -> %llu", baseFreq, freq);
                    }
                    static thread_local FrequencyFormatter formatter;
                    const std::string response = buildCommand("FB", formatter.format(freq));
                    respondToSource(cmd, response, usbSerial, rm);
                }
                return true;
            }

            // During tuning, suppress FB SET commands from AI2 clients to prevent feedback loops
            // AI2 clients receive frequency broadcasts from radio that get parsed as SET commands
            if (rm.getState().isTuning.load())
            {
                bool isAI2Client = false;
                if (cmd.source == CommandSource::UsbCdc0)
                {
                    isAI2Client = (rm.getState().usbCdc0AiMode.load() >= 2);
                }
                else if (cmd.source == CommandSource::UsbCdc1)
                {
                    isAI2Client = (rm.getState().usbCdc1AiMode.load() >= 2);
                }
                else if (cmd.source == CommandSource::Display)
                {
                    isAI2Client = (rm.getState().displayAiMode.load() >= 2);
                }

                if (isAI2Client)
                {
                    ESP_LOGD(TAG, "FB: Suppressing SET command from AI2 client during tuning (source=%d)",
                             static_cast<int>(cmd.source));
                    return true; // Suppress but return success
                }
            }

            const uint64_t frequency = parseFrequency(cmd, 0);
            // Interpret incoming local value as display-space when transverter is enabled
            const uint64_t radioFrequency = rm.displayToBaseFrequency(frequency);
            if (radioFrequency != frequency)
            {
                ESP_LOGD(TAG, "FB converted transverter frequency: %llu (display) -> %llu (radio)", frequency,
                         radioFrequency);
            }

            // Validate the actual radio/base frequency
            if (!isValidFrequency(radioFrequency))
            {
                ESP_LOGW(TAG, "Invalid frequency in FB command: %llu", radioFrequency);
                return false;
            }

            // Forward to radio if from local source
            if (cmd.shouldSendToRadio())
            {
                // Track this as a pending command if from Panel (encoder/buttons)
                if (cmd.source == CommandSource::Panel)
                {
                    const uint64_t now = esp_timer_get_time();
                    rm.getState().vfoBFrequencyPending.store(true);
                    rm.getState().vfoBPendingTime.store(now);
                    rm.getState().vfoBPendingValue.store(radioFrequency);
                    ESP_LOGD(TAG, "FB SET from Panel: marked as pending (freq=%llu, time=%llu)", radioFrequency, now);
                }

                if (radioFrequency != frequency)
                {
                    // Send converted frequency to radio - zero-allocation path
                    static thread_local FrequencyFormatter formatter;
                    const char* radioCmd = formatter.formatCommand("FB", radioFrequency);
                    sendToRadio(radioSerial, radioCmd);
                    ESP_LOGV(TAG, "Sent converted frequency to radio: %s", radioCmd);
                }
                else
                {
                    sendToRadio(radioSerial, cmd.originalMessage);
                    ESP_LOGV(TAG, "Sent to radio: %s", cmd.originalMessage.c_str());
                }
            }

            // Update internal state with the actual radio frequency (not the display frequency)
            rm.updateVfoBFrequency(radioFrequency);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            // FB[frequency]; - Answer from radio
            const uint64_t frequency = parseFrequency(cmd, 0);
            ESP_LOGV(TAG, "FB answer received: %s (parsed=%llu)", cmd.originalMessage.c_str(),
                     static_cast<unsigned long long>(frequency));
            if (frequency == 0)
            {
                ESP_LOGW(TAG, "Failed to parse FB answer frequency");
                return false;
            }

            // During active local tuning, keep ESP32S3 authoritative: ignore radio freq updates
            const auto &state = rm.getState();
            if (state.isTuning.load())
            {
                ESP_LOGD(TAG, "FB during tuning: ignoring radio frequency update to prevent flicker");
                return true; // Suppress both state update and routing while tuning
            }

            // Grace period after tuning stops to ignore stale in-flight responses
            const uint64_t tuningStopTime = state.tuningStopTime.load();
            if (tuningStopTime > 0)
            {
                const uint64_t now = esp_timer_get_time();
                const uint64_t timeSinceTuningStop = now - tuningStopTime;
                constexpr uint64_t GRACE_PERIOD_US = 500000; // 500ms grace period
                if (timeSinceTuningStop < GRACE_PERIOD_US)
                {
                    ESP_LOGD(TAG, "FB in grace period (%llu us after tuning): ignoring to prevent jump-back",
                             timeSinceTuningStop);
                    return true; // Ignore stale responses during grace period
                }
            }

            // Check if we have a pending FB SET command from Panel
            const uint64_t now = esp_timer_get_time();
            const bool isPending = rm.getState().vfoBFrequencyPending.load();
            const uint64_t pendingTime = rm.getState().vfoBPendingTime.load();
            const uint64_t pendingValue = rm.getState().vfoBPendingValue.load();

            if (isPending && pendingTime > 0)
            {
                const uint64_t timeSincePending = now - pendingTime;
                constexpr uint64_t PENDING_TIMEOUT_US = 2000000; // 2 seconds

                if (timeSincePending < PENDING_TIMEOUT_US)
                {
                    // We're within the pending window
                    if (frequency == pendingValue)
                    {
                        // Radio confirmed our SET command
                        ESP_LOGD(TAG, "FB answer confirmed pending value %llu (took %llu us)", frequency,
                                 timeSincePending);
                        rm.getState().vfoBFrequencyPending.store(false);
                        rm.getState().vfoBPendingTime.store(0);
                        rm.getState().vfoBPendingValue.store(0);
                    }
                    else
                    {
                        // Radio sent a different value - this is likely a stale response
                        ESP_LOGD(TAG, "FB answer %llu doesn't match pending %llu - ignoring stale response", frequency,
                                 pendingValue);
                        return true; // Ignore this stale response
                    }
                }
                else
                {
                    // Timeout - clear pending and accept radio's value
                    ESP_LOGD(TAG, "FB pending timeout (%llu us) - accepting radio value", timeSincePending);
                    rm.getState().vfoBFrequencyPending.store(false);
                    rm.getState().vfoBPendingTime.store(0);
                    rm.getState().vfoBPendingValue.store(0);
                }
            }

            // Update state after validation
            rm.updateVfoBFrequency(frequency);
            rm.getState().commandCache.update("FB", esp_timer_get_time());
            ESP_LOGD(TAG, "FB updated state with frequency: %llu", frequency);

            // Build adjusted response (apply transverter offset once) and route by policy
            {
                const uint64_t displayFreq = rm.baseToDisplayFrequency(frequency);
                static thread_local FrequencyFormatter formatter;
                const std::string response = buildCommand("FB", formatter.format(displayFreq));
                routeAnswerResponse(cmd, response, usbSerial, rm);
            }

            return true;
        }

        return false;
    }

    // =============================================================================
    // VFO Control Commands (FR, FT, FS, SP, UP, DN)
    // =============================================================================

    bool FrequencyVfoCommandHandler::handleFR(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            // Local (USB/TCP/Display) queries should be answered from state (no round-trip)
            if (cmd.isCatClient() || cmd.source == CommandSource::Display)
            {
                const int rxVfo = rm.getRxVfo();
                const std::string response = buildCommand("FR", std::to_string(rxVfo));
                respondToSource(cmd, response, usbSerial, rm);
            }
            else
            {
                const uint64_t timestamp = esp_timer_get_time();
                rm.getState().queryTracker.recordQuery("FR", timestamp);
                rm.noteQueryOrigin("FR", cmd.source, timestamp);
                sendToRadio(radioSerial, buildCommand("FR"));
            }
            return true;
        }

        if (isSet(cmd))
        {
            const int vfo = parseVfoValue(cmd);
            if (vfo < 0 || vfo > 2)
            {
                ESP_LOGW(TAG, "Invalid VFO value in FR command: %d", vfo);
                return false;
            }

            ESP_LOGI(TAG, "🔧 FR SET: Setting RX VFO to %d (source: %s)", vfo, cmd.isLocal() ? "Local" : "Remote");

            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = buildCommand("FR", std::to_string(vfo));
                ESP_LOGI(TAG, "📤 FR SEND: Sending to radio: %s", cmdStr.c_str());
                sendToRadio(radioSerial, cmdStr);
            }

            rm.updateRxVfo(vfo);
            ESP_LOGI(TAG, "✅ FR STATE: Updated local RX VFO to %d", vfo);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int vfo = parseVfoValue(cmd);
            ESP_LOGI(TAG, "📥 FR ANSWER: Received from radio: %s (parsed VFO=%d)", cmd.originalMessage.c_str(), vfo);

            if (vfo >= 0 && vfo <= 2)
            {
                const bool changed = rm.updateRxVfo(vfo);
                ESP_LOGI(TAG, "✅ FR ANSWER: Updated local RX VFO to %d", vfo);
                if (changed && cmd.source == CommandSource::Remote)
                {
                    const uint64_t now = esp_timer_get_time();
                    auto &tracker = rm.getState().queryTracker;
                    if (!tracker.wasRecentlyQueried("IF", now, TTL_REALTIME))
                    {
                        tracker.recordQuery("IF", now);
                        sendToRadio(radioSerial, buildCommand("IF"));
                        ESP_LOGD(TAG, "FR answer triggered IF query for split resync");
                    }
                }
            }
            // Update cache
            rm.getState().commandCache.update("FR", esp_timer_get_time());
            // Route via unified policy
            routeAnswerResponse(cmd, cmd.originalMessage, usbSerial, rm);
            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleFT(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            // Local (USB/TCP/Display) queries should be answered from state (no round-trip)
            if (cmd.isCatClient() || cmd.source == CommandSource::Display)
            {
                const int txVfo = rm.getTxVfo();
                const std::string response = buildCommand("FT", std::to_string(txVfo));
                respondToSource(cmd, response, usbSerial, rm);
            }
            else
            {
                const uint64_t timestamp = esp_timer_get_time();
                rm.getState().queryTracker.recordQuery("FT", timestamp);
                rm.noteQueryOrigin("FT", cmd.source, timestamp);
                sendToRadio(radioSerial, buildCommand("FT"));
            }
            return true;
        }

        if (isSet(cmd))
        {
            const int ftParam = parseVfoValue(cmd);

            if (ftParam != 0 && ftParam != 1)
            {
                ESP_LOGW(TAG, "Invalid FT parameter: %d", ftParam);
                return false;
            }

            // TS-590 semantics: FT parameter is absolute TX VFO selection
            // 0 = VFO A, 1 = VFO B
            const int actualTxVfo = ftParam;
            ESP_LOGI(TAG, "🔧 FT SET: TX VFO set to %s (param=%d)", actualTxVfo ? "B" : "A", ftParam);

            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = buildCommand("FT", std::to_string(ftParam));
                ESP_LOGI(TAG, "📤 FT SEND: Sending to radio: %s", cmdStr.c_str());
                sendToRadio(radioSerial, cmdStr);
            }

            rm.updateTxVfo(actualTxVfo);
            ESP_LOGI(TAG, "✅ FT STATE: Updated local TX VFO to %d (A=0/B=1)", actualTxVfo);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int ftParam = parseVfoValue(cmd);
            ESP_LOGI(TAG, "📥 FT ANSWER: Received from radio: %s (parsed param=%d)", cmd.originalMessage.c_str(),
                     ftParam);

            if (ftParam == 0 || ftParam == 1)
            {
                // Absolute TX VFO from radio
                const int actualTxVfo = ftParam;
                const bool changed = rm.updateTxVfo(actualTxVfo);
                ESP_LOGI(TAG, "✅ FT ANSWER: Updated local TX VFO to %d (A=0/B=1)", actualTxVfo);
                if (changed && cmd.source == CommandSource::Remote)
                {
                    const uint64_t now = esp_timer_get_time();
                    auto &tracker = rm.getState().queryTracker;
                    if (!tracker.wasRecentlyQueried("IF", now, TTL_REALTIME))
                    {
                        tracker.recordQuery("IF", now);
                        sendToRadio(radioSerial, buildCommand("IF"));
                        ESP_LOGD(TAG, "FT answer triggered IF query for split resync");
                    }
                }
            }
            // Update cache timestamp
            rm.getState().commandCache.update("FT", esp_timer_get_time());

            // Route via unified forwarding policy
            routeAnswerResponse(cmd, cmd.originalMessage, usbSerial, rm);
            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleFS(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            // USB/TCP queries should be answered from cached state (no radio round-trip)
            if (cmd.isCatClient())
            {
                // Return current fine tune setting from state (default off if unknown)
                const bool fine = rm.getState().fineTune;
                const std::string response = buildCommand("FS", fine ? "1" : "0");
                respondToSource(cmd, response, usbSerial, rm);
            }
            else
            {
                const uint64_t timestamp = esp_timer_get_time();
                rm.getState().queryTracker.recordQuery("FS", timestamp);
                rm.noteQueryOrigin("FS", cmd.source, timestamp);
                sendToRadio(radioSerial, buildCommand("FS"));
            }
            return true;
        }

        if (isSet(cmd))
        {
            const int fineStep = parseOnOffValue(cmd);
            if (fineStep < 0 || fineStep > 1)
            {
                ESP_LOGW(TAG, "Invalid fine step value in FS command");
                return false;
            }

            // Update state
            rm.getState().fineTune = (fineStep == 1);

            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = buildCommand("FS", std::to_string(fineStep));
                sendToRadio(radioSerial, cmdStr);
            }
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            if (const int fineStep = parseOnOffValue(cmd); fineStep >= 0)
            {
                rm.getState().fineTune = (fineStep == 1);
            }
            // Route via unified policy (forwards to USB and Display based on AI mode)
            routeAnswerResponse(cmd, cmd.originalMessage, usbSerial, rm);
            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleSP(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            // USB/TCP queries should return current state immediately (no radio round-trip)
            if (cmd.isCatClient())
            {
                // Tests expect SP query to reflect splitSetting, not split enabled flag
                const uint8_t setting = rm.getState().splitSetting.load();
                const std::string response = buildCommand("SP", std::to_string(setting));
                respondToSource(cmd, response, usbSerial, rm);
            }
            else
            {
                const uint64_t timestamp = esp_timer_get_time();
                rm.getState().queryTracker.recordQuery("SP", timestamp);
                rm.noteQueryOrigin("SP", cmd.source, timestamp);
                sendToRadio(radioSerial, buildCommand("SP"));
            }
            return true;
        }

        if (isSet(cmd))
        {
            const std::string paramStr = getStringParam(cmd, 0, "");
            if (paramStr.empty())
            {
                ESP_LOGW(TAG, "SP set command missing parameter");
                return false;
            }

            // If the parameter is a single digit, treat it as a simple split on/off/setting command
            if (paramStr.length() == 1)
            {
                int split = -1;
                if (const auto [ptr, ec] = std::from_chars(paramStr.data(), paramStr.data() + paramStr.size(), split);
                    ec != std::errc{} || split < 0 || split > 2)
                {
                    ESP_LOGW(TAG, "Invalid split value in SP command: %s", paramStr.c_str());
                    return false;
                }

                auto &state = rm.getState();
                state.splitSetting.store(split);
                rm.updateSplitEnabled(split == 1);
            }

            // For any other parameter, just forward it. This will cover "SP015;"
            if (shouldSendToRadio(cmd))
            {
                const auto cmdStr = buildCommand("SP", paramStr);
                sendToRadio(radioSerial, cmdStr);
            }

            ESP_LOGD(TAG, "Split setting updated with param %s", paramStr.c_str());
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            if (const int split = parseOnOffValue(cmd); split >= 0)
            {
                rm.updateSplitEnabled(split == 1);
            }
            // Route via unified policy (forwards to USB and Display based on AI mode)
            routeAnswerResponse(cmd, cmd.originalMessage, usbSerial, rm);
            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleUP(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              RadioManager &rm) const
    {
        // Support query form as "UP01;" default step
        if (isQuery(cmd))
        {
            if (shouldSendToRadio(cmd))
            {
                sendToRadio(radioSerial, buildCommand("UP", "01"));
            }
            return true;
        }

        if (isSet(cmd))
        {
            if (shouldSendToRadio(cmd))
            {
                // Forward parameter with zero-padding (e.g., UP15; or default UP01;)
                std::string paramStr = getStringParam(cmd, 0, "01");
                if (paramStr.empty())
                {
                    paramStr = "01";
                }
                else if (paramStr.length() == 1)
                {
                    paramStr = "0" + paramStr; // Zero-pad single digits
                }
                const std::string cmdStr = buildCommand("UP", paramStr);
                sendToRadio(radioSerial, cmdStr);
            }

            // Update VFO A frequency by appropriate step size
            const uint64_t currentFreq = rm.getVfoAFrequency();
            const int currentMode = rm.getMode();
            const uint32_t stepSize = getStepSizeForMode(currentMode);

            std::string paramStr = getStringParam(cmd, 0, "01");
            if (paramStr.empty())
            {
                paramStr = "01";
            }
            int stepCount = 1;
            if (const auto [ptr, ec] = std::from_chars(paramStr.data(), paramStr.data() + paramStr.size(), stepCount);
                ec != std::errc{})
            {
                ESP_LOGW(TAG, "Failed to parse UP step count '%s', defaulting to 1", paramStr.c_str());
                stepCount = 1;
            }
            stepCount = std::clamp(stepCount, 0, 99);

            if (stepCount > 0 && stepSize > 0)
            {
                const uint64_t totalStepHz = static_cast<uint64_t>(stepSize) * static_cast<uint64_t>(stepCount);
                if (const uint64_t newFreq = currentFreq + totalStepHz; isValidFrequency(newFreq))
                {
                    rm.updateVfoAFrequency(newFreq);
                }
            }

            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleDN(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              RadioManager &rm) const
    {
        // Support query form as "DN01;" default step
        if (isQuery(cmd))
        {
            if (shouldSendToRadio(cmd))
            {
                sendToRadio(radioSerial, buildCommand("DN", "01"));
            }
            return true;
        }

        if (isSet(cmd))
        {
            if (shouldSendToRadio(cmd))
            {
                // Forward parameter with zero-padding (e.g., DN15; or default DN01;)
                std::string paramStr = getStringParam(cmd, 0, "01");
                if (paramStr.empty())
                {
                    paramStr = "01";
                }
                else if (paramStr.length() == 1)
                {
                    paramStr = "0" + paramStr; // Zero-pad single digits
                }
                const std::string cmdStr = buildCommand("DN", paramStr);
                sendToRadio(radioSerial, cmdStr);
            }

            // Update VFO A frequency by appropriate step size
            const uint64_t currentFreq = rm.getVfoAFrequency();
            const int currentMode = rm.getMode();

            std::string paramStr = getStringParam(cmd, 0, "01");
            if (paramStr.empty())
            {
                paramStr = "01";
            }
            int stepCount = 1;
            if (const auto [ptr, ec] = std::from_chars(paramStr.data(), paramStr.data() + paramStr.size(), stepCount);
                ec != std::errc{})
            {
                ESP_LOGW(TAG, "Failed to parse DN step count '%s', defaulting to 1", paramStr.c_str());
                stepCount = 1;
            }
            stepCount = std::clamp(stepCount, 0, 99);

            if (stepCount > 0)
            {
                if (const uint32_t stepSize = getStepSizeForMode(currentMode); stepSize > 0)
                {
                    const uint64_t totalStepHz = static_cast<uint64_t>(stepSize) * static_cast<uint64_t>(stepCount);
                    if (currentFreq >= totalStepHz)
                    {
                        if (const uint64_t newFreq = currentFreq - totalStepHz; isValidFrequency(newFreq))
                        {
                            rm.updateVfoAFrequency(newFreq);
                        }
                    }
                }
            }

            return true;
        }

        return false;
    }

    // =============================================================================
    // RIT/XIT Commands (RT, XT, RC, RU, RD, RO)
    // =============================================================================

    bool FrequencyVfoCommandHandler::handleRT(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            // During TX, answer locally to avoid radio overflow (radio issues O; if queried)
            if (rm.getState().isTx.load())
            {
                const bool ritEnabled = rm.isRitEnabled();
                const std::string response = buildCommand("RT", std::to_string(ritEnabled ? 1 : 0));
                respondToSource(cmd, response, usbSerial, rm);
                return true;
            }

            if (shouldSendToRadio(cmd))
            {
                sendToRadio(radioSerial, buildCommand("RT"));
            }
            else
            {
                const bool ritEnabled = rm.isRitEnabled();
                const std::string response = buildCommand("RT", std::to_string(ritEnabled ? 1 : 0));
                respondToSource(cmd, response, usbSerial, rm);
            }
            return true;
        }

        if (isSet(cmd))
        {
            const int rit = parseOnOffValue(cmd);
            if (rit < 0)
            {
                ESP_LOGW(TAG, "Invalid RIT value in RT command");
                return false;
            }

            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = buildCommand("RT", std::to_string(rit));
                sendToRadio(radioSerial, cmdStr);
            }

            rm.updateRitEnabled(rit == 1);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            if (const int rit = parseOnOffValue(cmd); rit >= 0)
            {
                rm.updateRitEnabled(rit == 1);
            }

            // Route via unified policy (forwards to USB and Display based on AI mode)
            routeAnswerResponse(cmd, cmd.originalMessage, usbSerial, rm);
            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleXT(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            if (shouldSendToRadio(cmd))
            {
                rm.getState().queryTracker.recordQuery("XT", esp_timer_get_time());
                sendToRadio(radioSerial, buildCommand("XT"));
            }
            else
            {
                const bool xitEnabled = rm.isXitEnabled();
                const std::string response = buildCommand("XT", std::to_string(xitEnabled ? 1 : 0));
                respondToSource(cmd, response, usbSerial, rm);
            }
            return true;
        }

        if (isSet(cmd))
        {
            const int xit = parseOnOffValue(cmd);
            if (xit < 0)
            {
                ESP_LOGW(TAG, "Invalid XIT value in XT command");
                return false;
            }

            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = buildCommand("XT", std::to_string(xit));
                sendToRadio(radioSerial, cmdStr);
            }

            rm.updateXitEnabled(xit == 1);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            if (const int xit = parseOnOffValue(cmd); xit >= 0)
            {
                rm.updateXitEnabled(xit == 1);
            }

            // Route via unified policy
            routeAnswerResponse(cmd, cmd.originalMessage, usbSerial, rm);
            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleRC(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm)
    {
        // RC clears RIT/XIT offset; some flows issue it without params (query-form)
        if (isSet(cmd) || isQuery(cmd))
        {
            if (shouldSendToRadio(cmd))
            {
                sendToRadio(radioSerial, buildCommand("RC"));
            }
            // Clear RIT/XIT offsets locally
            rm.updateRitOffset(0);
            rm.updateXitOffset(0);
            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleRU(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            // RU; - Only valid when scanning is ON or RIT/XIT is enabled
            // When scan is OFF and RIT/XIT is disabled, radio responds with "?;"
            if (shouldSendToRadio(cmd))
            {
                const auto &state = rm.getState();
                const bool scanActive = state.scanStatus.load() != 0;
                const bool ritXitActive = state.ritOn.load() || state.xitOn.load();

                if (scanActive || ritXitActive)
                {
                    sendToRadio(radioSerial, buildCommand("RU"));
                }
                else
                {
                    ESP_LOGW(TAG, "RU query ignored: scan OFF and RIT/XIT disabled (would cause radio error)");
                }
            }
            return true;
        }

        if (isSet(cmd))
        {
            const std::string paramStr = getStringParam(cmd, 0, "");
            if (paramStr.empty())
            {
                // This is the case for RU; which means step up
                const int currentOffset = rm.getRitOffset();
                if (const int newOffset = currentOffset + RIT_XIT_STEP; newOffset <= MAX_RIT_XIT_OFFSET)
                {
                    rm.updateRitOffset(newOffset);
                }
            }
            else
            {
                int offset = 0;
                if (const auto [ptr, ec] = std::from_chars(paramStr.data(), paramStr.data() + paramStr.size(), offset);
                    ec != std::errc{})
                {
                    ESP_LOGW(TAG, "Invalid offset in RU command: %s", paramStr.c_str());
                    return false;
                }
                // RU sets the RIT offset to the specified positive value (clamped to valid range)
                const int clampedOffset = std::min(offset, 9999);
                rm.updateRitOffset(clampedOffset);
            }

            if (shouldSendToRadio(cmd))
            {
                const auto &state = rm.getState();
                const bool scanActive = state.scanStatus.load() != 0;
                const bool ritXitActive = state.ritOn.load() || state.xitOn.load();

                if (scanActive || ritXitActive)
                {
                    sendToRadio(radioSerial, buildCommand("RU", paramStr));
                }
                else
                {
                    ESP_LOGW(TAG, "RU set command ignored: scan OFF and RIT/XIT disabled (would cause radio error)");
                }
            }
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            // Handle RU answers from radio (scan speed or confirmation)
            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleRD(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            // RD; - Only valid when scanning is ON or RIT/XIT is enabled
            // When scan is OFF and RIT/XIT is disabled, radio responds with "?;"
            if (shouldSendToRadio(cmd))
            {
                const auto &state = rm.getState();
                const bool scanActive = state.scanStatus.load() != 0;
                const bool ritXitActive = state.ritOn.load() || state.xitOn.load();

                if (scanActive || ritXitActive)
                {
                    sendToRadio(radioSerial, buildCommand("RD"));
                }
                else
                {
                    ESP_LOGW(TAG, "RD query ignored: scan OFF and RIT/XIT disabled (would cause radio error)");
                }
            }
            return true;
        }

        if (isSet(cmd))
        {
            const std::string paramStr = getStringParam(cmd, 0, "");
            if (paramStr.empty())
            {
                // This is the case for RD; which means step down
                const int currentOffset = rm.getRitOffset();

                if (const int newOffset = currentOffset - RIT_XIT_STEP; newOffset >= MIN_RIT_XIT_OFFSET)
                {
                    rm.updateRitOffset(newOffset);
                }
            }
            else
            {
                int offset = 0;
                if (const auto [ptr, ec] = std::from_chars(paramStr.data(), paramStr.data() + paramStr.size(), offset);
                    ec != std::errc{})
                {
                    ESP_LOGW(TAG, "Invalid offset in RD command: %s", paramStr.c_str());
                    return false;
                }
                // RD sets the RIT offset to the specified negative value (clamped to valid range)
                const int clampedOffset = std::max(-offset, -9999);
                rm.updateRitOffset(clampedOffset);
            }

            if (shouldSendToRadio(cmd))
            {
                const auto &state = rm.getState();
                const bool scanActive = state.scanStatus.load() != 0;
                const bool ritOn = state.ritOn.load();
                const bool xitOn = state.xitOn.load();
                const bool ritXitActive = ritOn || xitOn;

                ESP_LOGI(TAG, "RD SET Debug: scanActive=%s, ritOn=%s, xitOn=%s, param='%s'", scanActive ? "YES" : "NO",
                         ritOn ? "YES" : "NO", xitOn ? "YES" : "NO", paramStr.c_str());

                if (scanActive || ritXitActive)
                {
                    ESP_LOGI(TAG, "Sending RD command to radio: RD%s;", paramStr.c_str());
                    sendToRadio(radioSerial, buildCommand("RD", paramStr));
                }
                else
                {
                    ESP_LOGW(TAG, "RD set command ignored: scan OFF and RIT/XIT disabled (would cause radio error)");
                    // When we detect this mismatch, it usually means state is out of sync
                    // The ADC thinks RIT is enabled but it's actually not
                    ESP_LOGW(TAG, "Consider checking RIT/XIT state synchronization");
                }
            }
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            // Handle RD answers from radio (scan speed or confirmation)
            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleRO(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        if (isQuery(cmd))
        {
            if (shouldSendToRadio(cmd))
            {
                sendToRadio(radioSerial, buildCommand("RO"));
            }
            else
            {
                const int offset = rm.getRitOffset();
                const std::string response = buildCommand("RO", formatOffsetResponse(offset));
                respondToSource(cmd, response, usbSerial, rm);
            }
            return true;
        }

        if (isSet(cmd))
        {
            const int offset = parseOffsetValue(cmd);
            if (offset < MIN_RIT_XIT_OFFSET || offset > MAX_RIT_XIT_OFFSET)
            {
                ESP_LOGW(TAG, "Invalid RIT/XIT offset in RO command: %d", offset);
                return false;
            }

            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = buildCommand("RO", formatOffsetResponse(offset));
                sendToRadio(radioSerial, cmdStr);
            }

            rm.updateRitOffset(offset);
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            if (const int offset = parseOffsetValue(cmd); offset >= MIN_RIT_XIT_OFFSET && offset <= MAX_RIT_XIT_OFFSET)
            {
                rm.updateRitOffset(offset);
            }

            // Forward to USB if query was sent from USB client
            if (rm.shouldForwardToUSB(cmd.originalMessage))
            {
                respondToSource(cmd, cmd.originalMessage, usbSerial, rm);
                ESP_LOGV(TAG, "Forwarded RO to USB: %s", cmd.originalMessage.c_str());
            }

            return true;
        }

        return false;
    }

    // =============================================================================
    // New Commands (TS, VV, XO, EM)
    // =============================================================================

    bool FrequencyVfoCommandHandler::handleTS(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        // TS: TF-Set (transfer frequency to VFO)
        auto &state = rm.getState();

        if (isQuery(cmd))
        {
            if (cmd.isCatClient())
            {
                // For local queries, check if cached data is fresh
                if (isCacheFresh(rm, "TS", TTL_STATUS))
                {
                    // Cache is fresh - respond with cached data immediately
                    const bool tfSetStatus = state.tfSet;
                    const std::string response = buildCommand("TS", std::to_string(tfSetStatus ? 1 : 0));
                    respondToSource(cmd, response, usbSerial, rm);
                }
                else
                {
                    // Cache is stale - return cached data + background refresh
                    const bool tfSetStatus = state.tfSet;
                    const std::string response = buildCommand("TS", std::to_string(tfSetStatus ? 1 : 0));
                    respondToSource(cmd, response, usbSerial, rm);
                    // Background refresh to update cache
                    sendToRadio(radioSerial, buildCommand("TS"));
                }
            }
            else
            {
                // Non-local query - forward to radio
                const uint64_t timestamp = esp_timer_get_time();
                rm.getState().queryTracker.recordQuery("TS", timestamp);
                rm.noteQueryOrigin("TS", cmd.source, timestamp);
                sendToRadio(radioSerial, buildCommand("TS"));
            }
            return true;
        }

        if (isSet(cmd))
        {
            const int tfState = parseOnOffValue(cmd);
            if (tfState < 0)
            {
                ESP_LOGW(TAG, "Invalid TS TF-Set value: %d", tfState);
                return false;
            }

            // Update state
            state.tfSet = (tfState == 1);

            // Update cache timestamp when state changes
            const uint64_t currentTime = esp_timer_get_time();
            rm.getState().commandCache.update("TS", currentTime);

            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = buildCommand("TS", std::to_string(tfState));
                sendToRadio(radioSerial, cmdStr);
            }

            ESP_LOGD(TAG, "TF-Set %s", tfState ? "enabled" : "disabled");
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            const int tfState = parseOnOffValue(cmd);
            if (tfState >= 0)
            {
                // Update state from radio response
                state.tfSet = (tfState == 1);

                // Update cache timestamp for AI mode compatibility
                const uint64_t currentTime = esp_timer_get_time();
                rm.getState().commandCache.update("TS", currentTime);

                ESP_LOGD(TAG, "Updated TF-Set from radio: %s", tfState ? "enabled" : "disabled");
            }

            if (rm.shouldForwardToUSB(cmd.originalMessage))
            {
                const std::string response = buildCommand("TS", std::to_string(tfState));
                respondToSource(cmd, response, usbSerial, rm);
            }
            return true;
        }

        return true;
    }

    bool FrequencyVfoCommandHandler::handleVV(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm)
    {
        // VV: VFO copy - copies current RX VFO to the other VFO
        if (isSet(cmd))
        {
            if (shouldSendToRadio(cmd))
            {
                sendToRadio(radioSerial, buildCommand("VV"));
            }

            // VV performs VFO copy function based on current RX VFO
            const int currentRxVfo = rm.getRxVfo();
            const uint64_t vfoAFreq = rm.getVfoAFrequency();
            const uint64_t vfoBFreq = rm.getVfoBFrequency();

            if (currentRxVfo == 0)
            {
                // VFO A is current RX VFO, copy A to B
                rm.updateVfoBFrequency(vfoAFreq);

                // Send new frequency to radio for VFO B
                static thread_local FrequencyFormatter formatter;
                const std::string fbCommand = buildCommand("FB", formatter.format(vfoAFreq));
                if (shouldSendToRadio(cmd))
                {
                    sendToRadio(radioSerial, fbCommand);
                }
                ESP_LOGI(TAG, "VV: Copied VFO A (%.3f MHz) to VFO B, sent %s", vfoAFreq / 1e6, fbCommand.c_str());
            }
            else if (currentRxVfo == 1)
            {
                // VFO B is current RX VFO, copy B to A
                rm.updateVfoAFrequency(vfoBFreq);

                // Send new frequency to radio for VFO A
                static thread_local FrequencyFormatter formatter;
                const std::string faCommand = buildCommand("FA", formatter.format(vfoBFreq));
                if (shouldSendToRadio(cmd))
                {
                    sendToRadio(radioSerial, faCommand);
                }
                ESP_LOGI(TAG, "VV: Copied VFO B (%.3f MHz) to VFO A, sent %s", vfoBFreq / 1e6, faCommand.c_str());
            }

            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleXO(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        // XO: Transverter frequency offset
        if (isQuery(cmd))
        {
            ESP_LOGI(TAG, "🔍 XO QUERY: source=%d, isUsb=%s, isTcp=%s", static_cast<int>(cmd.source),
                     cmd.isUsb() ? "true" : "false", cmd.isTcp() ? "true" : "false");
            if (cmd.isCatClient())
            {
                // Record query when it originates from USB/TCP source
                const uint64_t timestamp = esp_timer_get_time();
                rm.getState().queryTracker.recordQuery("XO", timestamp);
                rm.noteQueryOrigin("XO", cmd.source, timestamp);
                ESP_LOGI(TAG, "🔍 XO QUERY: Recorded query for XO at time %llu", timestamp);
            }
            if (shouldSendToRadio(cmd))
            {
                const std::string xoCommand = buildCommand("XO");
                ESP_LOGI(TAG, "🔍 XO QUERY: Sending '%s' to radio", xoCommand.c_str());
                sendToRadio(radioSerial, xoCommand);
            }
            return true;
        }

        if (isSet(cmd))
        {
            // Parse XO set command: XOP1P2P2P2P2P2P2P2P2P2P2P2;
            const std::string paramStr = getStringParam(cmd, 0, "");
            if (paramStr.length() >= 12)
            {
                // 1 direction + 11 frequency digits
                // Parse direction (P1: 0=plus, 1=minus)
                if (const int direction = paramStr[0] - '0'; direction >= 0 && direction <= 1)
                {
                    rm.getState().transverterOffsetPlus.store(direction == 0, std::memory_order_relaxed);

                    // Parse frequency (P2: 11 digits in Hz)
                    const std::string freqStr = paramStr.substr(1, 11);
                    uint64_t frequency = 0;
                    if (const auto [ptr, ec] =
                            std::from_chars(freqStr.data(), freqStr.data() + freqStr.size(), frequency);
                        ec == std::errc{})
                    {
                        rm.getState().transverterOffsetHz.store(frequency, std::memory_order_relaxed);
                        ESP_LOGD(TAG, "Updated transverter offset: %s%llu Hz", direction == 0 ? "+" : "-", frequency);
                    }
                }
            }

            if (shouldSendToRadio(cmd))
            {
                const std::string cmdStr = paramStr.empty() ? "XO;" : "XO" + paramStr + ";";
                sendToRadio(radioSerial, cmdStr);
            }
            return true;
        }

        if (cmd.type == CommandType::Answer)
        {
            // Parse XO answer and update state; tolerate malformed/short frequency fields
            const std::string paramStr = getStringParam(cmd, 0, "");
            bool haveParsed = false;
            const bool exactWidth = (paramStr.size() == 12); // 1 dir + 11 digits
            int parsedDir = -1;
            std::string normFreqStr;
            uint64_t frequency = 0;

            if (!paramStr.empty())
            {
                // Try to interpret first char as direction only when exact width
                if (exactWidth && (paramStr[0] == '0' || paramStr[0] == '1'))
                {
                    parsedDir = paramStr[0] - '0';
                    const std::string freqStr = paramStr.substr(1);
                    if (const auto [ptr, ec] =
                            std::from_chars(freqStr.data(), freqStr.data() + freqStr.size(), frequency);
                        ec == std::errc{})
                    {
                        normFreqStr = freqStr; // already 11 digits
                        haveParsed = true;
                    }
                }
                else
                {
                    // Treat entire field as frequency digits; direction from current state
                    std::string freqStr = paramStr;
                    if (freqStr.size() < 11)
                        freqStr = std::string(11 - freqStr.size(), '0') + freqStr;
                    else if (freqStr.size() > 11)
                        freqStr = freqStr.substr(freqStr.size() - 11);
                    if (const auto [ptr, ec] =
                            std::from_chars(freqStr.data(), freqStr.data() + freqStr.size(), frequency);
                        ec == std::errc{})
                    {
                        normFreqStr = freqStr;
                        haveParsed = true;
                    }
                }
            }

            // Update state and build normalized response
            int outDir = rm.getState().transverterOffsetPlus.load(std::memory_order_relaxed) ? 0 : 1; // default to state
            if (haveParsed)
            {
                // If we had an exact-width frame with a clear direction, trust it; otherwise keep state
                if (exactWidth && (parsedDir == 0 || parsedDir == 1))
                {
                    outDir = parsedDir;
                    rm.getState().transverterOffsetPlus.store(outDir == 0, std::memory_order_relaxed);
                }
                rm.getState().transverterOffsetHz.store(frequency, std::memory_order_relaxed);
                ESP_LOGD(TAG, "XO normalized: dir=%d freq=%s (Hz=%llu)", outDir, normFreqStr.c_str(), frequency);
            }

            // Build normalized response
            std::string response;
            if (haveParsed)
            {
                // Always forward in normalized format: XO + dir + 11-digit freq + ;
                response.reserve(2 + 1 + 11 + 1);
                response.append("XO");
                response.push_back(static_cast<char>('0' + outDir));
                response.append(normFreqStr);
                response.push_back(';');
            }
            else
            {
                // Fallback: forward raw
                response = paramStr.empty() ? std::string("XO;") : "XO" + paramStr + ";";
            }

            // Forward to USB if policy allows
            if (rm.shouldForwardToUSB(cmd.originalMessage))
            {
                respondToSource(cmd, response, usbSerial, rm);
                ESP_LOGD(TAG, "XO answer forwarded to USB: %s", response.c_str());
            }

            // Always forward XO answers to display so it knows the transverter offset
            rm.sendToDisplay(response);
            ESP_LOGD(TAG, "XO answer forwarded to display: %s", response.c_str());
            return true;
        }

        return false;
    }

    bool FrequencyVfoCommandHandler::handleEM(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, const RadioManager &rm)
    {
        // EM: Emergency communication frequency mode - implementation needed
        // For now, just forward to radio
        if (shouldSendToRadio(cmd))
        {
            sendToRadio(radioSerial, buildCommand("EM"));
        }

        // Forward answer if from radio
        if (cmd.type == CommandType::Answer && rm.shouldForwardToUSB(cmd.originalMessage))
        {
            respondToSource(cmd, buildCommand("EM"), usbSerial, rm);
        }

        return true;
    }

    bool FrequencyVfoCommandHandler::handleCH(const RadioCommand &cmd, ISerialChannel &radioSerial,
                                              ISerialChannel &usbSerial, RadioManager &rm) const
    {
        // CH: MULTI/CH encoder single step
        // This is typically a set-only command for stepping through frequency/channels
        if (isSet(cmd))
        {
            const int direction = getIntParam(cmd, 0, -1);
            if (direction < 0 || direction > 1)
            {
                ESP_LOGW(TAG, "Invalid CH direction: %d", direction);
                return false;
            }

            if (shouldSendToRadio(cmd))
            {
                const auto cmdStr = buildCommand("CH", std::to_string(direction));
                sendToRadio(radioSerial, cmdStr);

                // Update display immediately with calculated frequency (like UP/DN commands in EncoderHandler)
                auto& state = rm.getState();
                const bool isTxActive = state.isTx.load();
                const uint8_t currentRxVfo = state.currentRxVfo.load();
                const uint8_t currentTxVfo = state.currentTxVfo.load();
                const uint8_t activeVfo = isTxActive ? currentTxVfo : currentRxVfo;
                const bool tuneVfoB = (activeVfo == 1);

                // Get current frequency
                const uint64_t currentFreq = tuneVfoB ? state.vfoBFrequency.load() : state.vfoAFrequency.load();

                // Use 5 kHz as default step size for MULTI/CH control
                // NOTE: The actual step size is configurable via Menu 016-019 (EX016-EX019)
                // but we use a sensible default here. 5 kHz is a good middle-ground for
                // quick frequency changes with the MULTI/CH encoder.
                // TODO: Query EX016-EX019 at startup and use actual radio settings
                const int32_t stepHz = 5000; // 5 kHz default

                // Apply frequency change
                const int32_t deltaHz = (direction == 0) ? stepHz : -stepHz;
                uint64_t newFreq = currentFreq;
                if (deltaHz < 0 && static_cast<uint64_t>(-deltaHz) > newFreq) {
                    newFreq = 30000; // Clamp to minimum
                } else {
                    newFreq = currentFreq + deltaHz;
                }

                // Update local cache
                if (tuneVfoB) {
                    rm.updateVfoBFrequency(newFreq);
                } else {
                    rm.updateVfoAFrequency(newFreq);
                }

                // Send update to display immediately (like EncoderHandler does)
                // Apply transverter offset for display if enabled (controlled by UIXD1/UIXD0)
                uint64_t displayFreq = newFreq;
                const bool offsetEnabled = state.transverterOffsetEnabled.load(std::memory_order_relaxed);
                const bool transverterOn = state.transverter.load(std::memory_order_relaxed);
                const uint64_t offsetHz = state.transverterOffsetHz.load(std::memory_order_relaxed);
                if (offsetEnabled && transverterOn && offsetHz > 0) {
                    if (state.transverterOffsetPlus.load(std::memory_order_relaxed)) {
                        displayFreq = newFreq + offsetHz;
                    } else {
                        displayFreq = (newFreq > offsetHz) ? (newFreq - offsetHz) : 0ULL;
                    }
                }
                char displayCmd[20];
                snprintf(displayCmd, sizeof(displayCmd), "%s%011llu;", tuneVfoB ? "FB" : "FA", displayFreq);
                rm.sendToDisplay(displayCmd);

                ESP_LOGD(TAG, "MULTI/CH encoder step %s executed, updated VFO %c to %llu Hz (step=%d Hz)",
                         direction == 0 ? "up" : "down", tuneVfoB ? 'B' : 'A', newFreq, stepHz);
            }

            return true;
        }

        return false;
    }

    // =============================================================================
    // Helper Functions
    // =============================================================================

    bool FrequencyVfoCommandHandler::isValidFrequency(const uint64_t frequency)
    {
        return frequency >= MIN_FREQUENCY && frequency <= MAX_FREQUENCY;
    }

    uint64_t FrequencyVfoCommandHandler::parseFrequency(const RadioCommand &cmd, const size_t index,
                                                        const uint64_t defaultValue) const
    {
        if (index >= cmd.paramSize())
        {
            return defaultValue;
        }

        // Try to get string parameter
        const std::string freqStr = getStringParam(cmd, index);
        if (freqStr.empty())
        {
            return defaultValue;
        }

        // Parse as uint64_t
        uint64_t frequency = 0;
        if (const auto [ptr, ec] = std::from_chars(freqStr.data(), freqStr.data() + freqStr.size(), frequency);
            ec != std::errc{})
        {
            return defaultValue;
        }

        return frequency;
    }

    int FrequencyVfoCommandHandler::parseVfoValue(const RadioCommand &cmd) const { return getIntParam(cmd, 0, -1); }


    int FrequencyVfoCommandHandler::parseOffsetValue(const RadioCommand &cmd) const
    {
        const std::string offsetStr = getStringParam(cmd, 0);
        if (offsetStr.empty())
        {
            return 0;
        }

        int offset = 0;
        if (const auto [ptr, ec] = std::from_chars(offsetStr.data(), offsetStr.data() + offsetStr.size(), offset);
            ec != std::errc{})
        {
            return 0;
        }

        return offset;
    }

    std::string FrequencyVfoCommandHandler::formatOffsetResponse(const int offset)
    {
        // Format: +/-NNNN (sign + 4 digits)
        std::string result;
        result.reserve(5);
        int absOffset = offset >= 0 ? offset : -offset;
        result.push_back(offset >= 0 ? '+' : '-');
        result.push_back('0' + (absOffset / 1000) % 10);
        result.push_back('0' + (absOffset / 100) % 10);
        result.push_back('0' + (absOffset / 10) % 10);
        result.push_back('0' + absOffset % 10);
        return result;
    }

    uint32_t FrequencyVfoCommandHandler::getStepSizeForMode(const int mode)
    {
        switch (mode)
        {
        case 1: // LSB
        case 2: // USB
            return STEP_SIZE_SSB;
        case 3: // CW
        case 7: // CW-R
            return STEP_SIZE_CW;
        case 4: // FM
            return STEP_SIZE_FM;
        case 5: // AM
            return STEP_SIZE_AM;
        default:
            return STEP_SIZE_DEFAULT;
        }
    }

    // NOTE: Transverter offset helpers have been moved to RadioManager as the
    // centralized, authoritative implementation. Use rm.baseToDisplayFrequency()
    // and rm.displayToBaseFrequency() instead.

} // namespace radio
