#include "CommandDispatcher.h"
#include "ICommandHandler.h"  // Need full definition for implementation
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cstring>
#include <unordered_map>

#include "RadioManager.h"
#include "sdkconfig.h"

namespace radio {

    // Mapping of SET commands to their corresponding READ commands for auto-query
    static const std::unordered_map<std::string, std::string> setToReadMap = {
        // Frequency and VFO commands
        {"FA", "FA;"},     // VFO A frequency 
        {"FB", "FB;"},     // VFO B frequency
        {"FR", "FR;"},     // RX VFO selection
        {"FT", "FT;"},     // TX VFO selection
        {"MD", "MD;"},     // Operating mode
        {"DA", "DA;"},     // Data mode
        {"FS", "FS;"},     // Fine step size
        {"SP", "SP;"},     // Split operation
        {"RT", "RT;"},     // RIT on/off
        {"XT", "XT;"},     // XIT on/off
        {"RO", "RO;"},     // RIT/XIT offset
        
        // Audio and RF settings
        {"AG", "AG0;"},    // AF Gain (P1=0 required per TS-590SG spec)
        {"RG", "RG;"},     // RF Gain
        {"MG", "MG;"},     // Microphone gain
        {"PC", "PC;"},     // Power control
        {"GT", "GT;"},     // AGC time constant
        {"SH", "SH;"},     // Filter high cut
        {"SL", "SL;"},     // Filter low cut
        
        // Memory commands
        {"MC", "MC;"},     // Memory channel
        
        // Antenna and control
        {"AN", "AN;"},     // Antenna selection
        {"RA", "RA;"},     // RF attenuator 
        {"PA", "PA;"},     // Pre-amplifier
        
        // Band and step commands
        {"BS", "BS;"},     // Band select
        {"SN", "SN;"},     // Step size
        
        // Tone/CTCSS
        {"TN", "TN;"},     // Tone frequency
        {"TO", "TO;"},     // Tone on/off
        
        // Squelch
        {"SQ", "SQ0;"},    // Squelch level (P1=0 per spec)
        
        // DSP and filters
        {"NB", "NB;"},     // Noise blanker
        {"NR", "NR;"},     // Noise reduction
        {"BC", "BC;"},     // Beat cancel
        {"NT", "NT;"}      // Notch filter
    };

    /**
     * @brief Check if automatic READ query should be sent after SET command
     *
     * Always-on behavior (not gated by Kconfig). After a local SET, sends the
     * matching READ from setToReadMap to confirm the radio applied the change.
     * Suppressed when AI2/AI4 is active (radio broadcasts updates itself) and
     * during active encoder/button tuning to prevent race conditions.
     *
     * @param command The command that was processed
     * @param radioManager Reference to radio manager for AI mode check
     * @return true if auto-query should be sent
     */
    static bool shouldAutoQuery(const RadioCommand& command, const RadioManager& radioManager) {
        // Only auto-query for SET commands from local (USB) source
        if (command.type != CommandType::Set || !(command.isUsb() || command.source == CommandSource::Panel)) {
            return false;
        }

        // Don't auto-query if in AI2 or AI4 mode (radio auto-sends updates)
        if (const int aiMode = radioManager.getState().aiMode.load(); aiMode == 2 || aiMode == 4) {
            return false;
        }

        // Check if we have a mapping for this command
        if (!setToReadMap.contains(command.command)) {
            return false;
        }

        // Special handling for frequency commands (FA/FB) during active tuning
        // Skip auto-query during encoder/button tuning to prevent race conditions
        if (command.command == "FA" || command.command == "FB") {
            const auto& state = radioManager.getState();

            // Skip if local tuning is currently active (encoder being turned)
            if (state.isTuning.load()) {
                ESP_LOGD("CommandDispatcher", "Skipping auto-query for %s (local tuning active)",
                         command.command.c_str());
                return false;
            }

            // Also skip if very recent encoder/button activity (within last 100ms)
            const uint64_t currentTime = esp_timer_get_time();
            const uint64_t lastEncoderActivity = state.lastEncoderActivityTime.load();
            const uint64_t lastButtonActivity = state.lastButtonActivityTime.load();

            if (constexpr uint64_t recentActivityThreshold = 100000; (currentTime - lastEncoderActivity) < recentActivityThreshold ||
                                                                     (currentTime - lastButtonActivity) < recentActivityThreshold) {
                ESP_LOGD("CommandDispatcher", "Skipping auto-query for %s (recent user activity)",
                         command.command.c_str());
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Check whether a local SET-shaped command is actually a parameterized READ.
     *
     * The CAT grammar uses the presence of parameters to classify most local frames as
     * SET commands. A small number of commands use parameters to select the item being
     * read, so those commands still need a per-interface pending-query entry. Keep this
     * exception list tied to the documented wire formats instead of treating every SET
     * (including AI0;) as a query.
     */
    static bool isParameterizedRead(const RadioCommand &command) {
        if (command.type != CommandType::Set || !command.isCatClient()) {
            return false;
        }

        const std::string_view frame = command.originalMessage;
        if (frame.size() < 3 || frame.back() != ';' || frame.substr(0, 2) != command.command) {
            return false;
        }

        const std::string_view params = frame.substr(2, frame.size() - 3);
        if (command.command == "MR") {
            return params.size() == 4; // P1 + three-character memory channel
        }
        if (command.command == "EQ") {
            return params.size() == 2; // P1 + P2; three parameters are a SET
        }
        if (command.command == "SQ") {
            return params.size() == 1; // P1; P1 + three-digit level is a SET
        }
        if (command.command == "SS") {
            return params.size() == 2; // P1 + P2; P3 frequency makes it a SET
        }
        if (command.command == "SU") {
            return params.size() == 1; // P1; the remaining fields are a SET
        }
        if (command.command == "AS") {
            return params.size() == 3; // P1 + two-digit channel; full entry is a SET
        }
        if (command.command == "EX") {
            return params.size() == 7 && params.substr(3) == "0000"; // EX[menu]0000;
        }

        return false;
    }

    CommandDispatcher::CommandDispatcher() {
        // Performance: Pre-allocate handler storage
        handlers_.reserve(16);

        // commandMap_ is std::array - zero-initialized at construction, no dynamic allocation

        ESP_LOGD(CommandDispatcher::TAG, "CommandDispatcher initialized with perfect hash table (%zu bytes)",
                 sizeof(commandMap_));
    }

    CommandDispatcher::~CommandDispatcher() {
        ESP_LOGD(CommandDispatcher::TAG, "CommandDispatcher destroyed with %zu handlers", handlers_.size());
    }

    bool CommandDispatcher::registerHandler(CommandHandlerPtr handler) {
        if (!handler) {
            ESP_LOGE(CommandDispatcher::TAG, "Attempted to register null handler");
            return false;
        }

        const std::string_view description = handler->getDescription();
        ESP_LOGD(CommandDispatcher::TAG, "Registering handler: %.*s", (int)description.size(), description.data());

        handlers_.push_back(std::move(handler));

        for (ICommandHandler *raw = handlers_.back().get(); const auto &prefix: raw->getPrefixes()) {
            if (prefix.size() != 2) {
                // 4-char prefixes (UI meta commands) are intentional - log at debug level
                ESP_LOGD(CommandDispatcher::TAG, "Skipping non-2char prefix '%.*s' from handler '%.*s'",
                         (int)prefix.size(), prefix.data(), (int)description.size(), description.data());
                continue;
            }

            // Perfect hash: direct array indexing based on 2-char command
            const size_t hashIdx = compactHash(prefix[0], prefix[1]);

            if (commandMap_[hashIdx] != nullptr) {
                ESP_LOGE(CommandDispatcher::TAG, "Duplicate handler registration for prefix '%.*s' (hash collision at %zu)",
                         (int)prefix.size(), prefix.data(), hashIdx);
                // Prefer first-registered; skip duplicate to avoid ambiguity
                continue;
            }

            commandMap_[hashIdx] = raw;
            ESP_LOGV(CommandDispatcher::TAG, "Mapped prefix '%.*s' -> %.*s (hash: %zu)",
                     (int)prefix.size(), prefix.data(), (int)description.size(), description.data(), hashIdx);
        }
        return true;
    }

    bool CommandDispatcher::dispatchCommand(const RadioCommand &command,
                                            ::ISerialChannel &radioSerial,
                                            ::ISerialChannel &usbSerial,
                                            RadioManager &radioManager) {
        stats_.totalCommandsDispatched++;
        
        // RAII-style depth tracker to ensure proper increment/decrement
        [[maybe_unused]] struct DepthTracker {
            DispatcherStatistics& stats;
            bool valid;
            explicit DepthTracker(DispatcherStatistics& s) : stats(s), valid(true) {
                const int32_t newDepth = stats.currentProcessingDepth.fetch_add(1) + 1;
                // Track peak processing depth atomically
                int32_t expectedMax = stats.maxProcessingDepth.load();
                while (newDepth > expectedMax && 
                       !stats.maxProcessingDepth.compare_exchange_weak(expectedMax, newDepth)) {
                    // Retry if another thread updated maxProcessingDepth
                }
            }
            ~DepthTracker() { 
                if (valid) {
                    const int32_t prevDepth = stats.currentProcessingDepth.fetch_sub(1);
                    if (prevDepth <= 0) {
                        ESP_LOGE("CommandDispatcher", "🚨 DEPTH UNDERFLOW: Depth was %lu before decrement!", prevDepth);
                    }
                }
            }
        } depthTracker(stats_);

        // Log PS queries at DEBUG level (answers are too frequent to log)
        if (command.command == "PS" && command.type == CommandType::Read) {
            ESP_LOGV(CommandDispatcher::TAG, "PS query from %s",
                     command.source == CommandSource::Remote ? "Remote" : "Local");
        }

        // Log local commands at INFO to diagnose programmer tool issues
        if (command.isUsb() || command.isTcp()) {
            ESP_LOGV(CommandDispatcher::TAG, "📥 %s %s from %s: '%s'",
                     command.type == CommandType::Set ? "SET" :
                     command.type == CommandType::Read ? "READ" : "ANS",
                     command.command.c_str(),
                     (command.source == CommandSource::UsbCdc0 ? "CDC0" :
                      command.source == CommandSource::UsbCdc1 ? "CDC1" :
                      command.source == CommandSource::Tcp0 ? "TCP0" : "TCP1"),
                     command.originalMessage.c_str());

            // Record per-interface queries so ForwardingPolicy can distinguish
            // "this interface queried IF" from "the display queried IF" in AI0 mode.
            // Only the explicit parameterized-read forms are allowed through the SET
            // exception; ordinary setters such as AI0; must not create a pending query.
            if (command.type == CommandType::Read || isParameterizedRead(command)) {
                radioManager.getState().accessForwardState(command.source)
                    .localQueryTracker.recordQuery(command.command, esp_timer_get_time());
            }

            // Remember every local Set/Read that is about to reach the radio (see
            // recordPendingLocalRequest) so a later error reply ('?;'/'E;'/'O;')
            // can be routed back to the interface that actually sent it, instead
            // of always assuming CDC0. Must happen before the handler runs.
            if (command.shouldSendToRadio()) {
                recordPendingLocalRequest(command.command, command.source, esp_timer_get_time());
            }
        }

        ESP_LOGV(CommandDispatcher::TAG, "Dispatching command: '%s' (type: %s, source: %s, depth: %lu)",
                 command.command.c_str(),
                 command.type == CommandType::Set ? "Set" :
                 command.type == CommandType::Read ? "Read" : "Answer",
                 (command.source == CommandSource::UsbCdc0 ? "Usb0" : (command.source == CommandSource::UsbCdc1 ? "Usb1" :
                 (command.source == CommandSource::Display ? "Display" :
                 (command.source == CommandSource::Panel ? "Panel" : "Remote")))),
                 stats_.currentProcessingDepth.load());

        if (command.command == ";") {
            if (command.isUsb()) {
                // Usb wakeup/keepalive from USB: respond with ID (no error marker needed)
                ESP_LOGV(CommandDispatcher::TAG, "USB ';' received; responding with ID");

                // Build ID response from Kconfig
                auto idResp = std::string("ID");
                if constexpr (CONFIG_RADIOCORE_ID_STRING != nullptr) {
                    idResp += CONFIG_RADIOCORE_ID_STRING;
                    idResp += ";";
                } else {
                    // ReSharper disable once CppDFAUnreachableCode
                    idResp += "023;";
                }

                // Build PS response directly to avoid stack-heavy std::ostringstream
                std::string ps_state = "PS";
                ps_state += static_cast<char>('0' + static_cast<int>(radioManager.getPowerState()));
                ps_state += ';';
                radioManager.sendToSource(command.source, ps_state);

                radioManager.sendToSource(command.source, idResp);
            } else {
                // Remote stray ';' – ignore
                ESP_LOGV(CommandDispatcher::TAG, "Ignoring remote standalone ';' frame");
            }
            stats_.commandsHandled++;
            return true;
        }

        // Performance: Check for error responses with likely branch prediction
        // Error responses are uncommon in normal operation
        if (command.source == CommandSource::Remote &&
            command.type == CommandType::Answer &&
            (command.command == "?" || command.command == "E" || command.command == "O")) {

            // Track error response for diagnostics
            const uint64_t currentTime = esp_timer_get_time();

            // Thread-safe: capture stats under lock, then log outside lock
            char lastCmdCopy[DispatcherStatistics::CMD_BUF_SIZE];
            char lastSourceCopy[DispatcherStatistics::CMD_BUF_SIZE];
            uint64_t lastErrorTimeCopy;
            uint64_t lastCmdTimeCopy;
            size_t totalErrors, qErrors, eErrorsCnt, oErrorsCnt;
            uint64_t avgInterval;
            size_t bursts;
            bool highErrorRateStarted;
            {
                RtosLockGuard<RtosMutex> lock(statsMutex_);
                std::memcpy(lastCmdCopy, stats_.lastCommandBeforeError, DispatcherStatistics::CMD_BUF_SIZE);
                std::memcpy(lastSourceCopy, stats_.lastCommandSource, DispatcherStatistics::CMD_BUF_SIZE);
                lastErrorTimeCopy = stats_.lastErrorTime;
                lastCmdTimeCopy = stats_.lastCommandTime;
                highErrorRateStarted = stats_.recordError(command.command, lastCmdCopy, lastSourceCopy, currentTime);
                // Capture post-recordError stats for logging
                totalErrors = stats_.totalErrorResponses;
                qErrors = stats_.questionMarkErrors;
                eErrorsCnt = stats_.eErrors;
                oErrorsCnt = stats_.oErrors;
                avgInterval = stats_.averageErrorInterval;
                bursts = stats_.errorBursts;
            }

            // Calculate time since last command for rate analysis
            const uint64_t timeSinceLastCmd = lastCmdTimeCopy > 0 ?
                currentTime - lastCmdTimeCopy : 0;

            if (highErrorRateStarted) {
                ESP_LOGW(CommandDispatcher::TAG,
                         "High radio error rate: %zu responses within %llums (latest='%s;', lastCmd='%s')",
                         DispatcherStatistics::ERROR_RATE_SAMPLE_COUNT,
                         static_cast<unsigned long long>(DispatcherStatistics::ERROR_RATE_WINDOW_US / 1000),
                         command.command.c_str(), lastCmdCopy[0] == '\0' ? "unknown" : lastCmdCopy);
            } else if (command.command == "E" || command.command == "O") {
                ESP_LOGW(CommandDispatcher::TAG, "Radio communication error '%s;' after '%s'",
                         command.command.c_str(), lastCmdCopy[0] == '\0' ? "unknown" : lastCmdCopy);
            }

            // '?;' can be an unsolicited high-rate radio response. Its counters and
            // interval/burst statistics are reported periodically by Diagnostics;
            // printing this block for every frame can itself disturb the CAT stream.
            // Keep detailed per-frame context behind DEBUG. E;/O; remain visible at
            // normal levels because they are exceptional and normally rare.
            const bool logErrorDetails = esp_log_level_get(CommandDispatcher::TAG) >= ESP_LOG_VERBOSE;
            if (logErrorDetails) {
                ESP_LOGV(CommandDispatcher::TAG, "=== ERROR RESPONSE DETECTED ===");
                ESP_LOGV(CommandDispatcher::TAG, "Error Type: '%s;'", command.command.c_str());
                ESP_LOGV(CommandDispatcher::TAG, "Last Command Sent: '%s' (from %s)", lastCmdCopy,
                         lastSourceCopy[0] == '\0' ? "unknown" : lastSourceCopy);
                ESP_LOGV(CommandDispatcher::TAG, "Time Since Last Cmd: %.1fms", timeSinceLastCmd / 1000.0);
                ESP_LOGV(CommandDispatcher::TAG, "Error Interval: %.1fms",
                         (lastErrorTimeCopy > 0) ? (currentTime - lastErrorTimeCopy) / 1000.0 : 0.0);
                ESP_LOGV(CommandDispatcher::TAG, "Total Errors: %zu (?;=%zu E;=%zu O;=%zu)",
                         totalErrors, qErrors, eErrorsCnt, oErrorsCnt);

                if (std::strstr(lastCmdCopy, "RM") != nullptr) {
                    ESP_LOGV(CommandDispatcher::TAG, "Error followed RM command");
                }

                if (totalErrors > 1) {
                    ESP_LOGV(CommandDispatcher::TAG, "Error Pattern: Avg interval=%.1fms, Bursts=%zu",
                             avgInterval / 1000.0, bursts);
                }

                if (timeSinceLastCmd > 0 && timeSinceLastCmd < 100000) {
                    ESP_LOGV(CommandDispatcher::TAG, "Fast error (%.1fms after command)",
                             timeSinceLastCmd / 1000.0);
                }
                ESP_LOGV(CommandDispatcher::TAG, "===============================");
            }
            
            ESP_LOGV(CommandDispatcher::TAG,
                     "Processing error response '%s;' from radio",
                     command.command.c_str());
            
            // Route 'E;'/'O;'/'?;' back to the CAT client whose request the radio is
            // actually replying to, using the pendingRequests_ ring recorded above
            // (see the isUsb()/isTcp() block earlier in this function) instead of
            // always assuming CDC0. A ring miss -- nothing recorded for this prefix
            // within PENDING_REQUEST_TTL_US, e.g. an unsolicited or stale frame, or
            // the ring's 8 slots having wrapped under heavy traffic -- falls back to
            // today's behaviour: 'E;'/'O;' still go to CDC0 via sendDirectResponse;
            // '?;' falls back to the global queryTracker check (pre-existing
            // behaviour) and is forwarded to CDC0 only if that also has a recent
            // matching query, otherwise it is suppressed to avoid flooding clients.
            if (command.command != "?") {
                std::optional<CommandSource> pendingSource;
                if (std::strlen(lastCmdCopy) >= 2) {
                    const std::string_view lastPrefix{lastCmdCopy, 2};
                    pendingSource = findAndConsumePendingRequest(lastPrefix, currentTime);
                }
                if (pendingSource) {
                    ESP_LOGV(CommandDispatcher::TAG, "Routing error response '%s;' to originating interface (source=%d)",
                             command.command.c_str(), static_cast<int>(*pendingSource));
                    radioManager.sendToSource(*pendingSource, command.originalMessage);
                } else {
                    ESP_LOGV(CommandDispatcher::TAG, "Forwarding error response '%s;' to USB", command.command.c_str());
                    radioManager.sendDirectResponse(command.originalMessage);
                }
            } else if (std::strlen(lastCmdCopy) >= 2) {
                const std::string_view prefix{lastCmdCopy, 2};
                if (auto pendingSource = findAndConsumePendingRequest(prefix, currentTime)) {
                    ESP_LOGV(CommandDispatcher::TAG, "Forwarding '?;' to originating interface — response to pending query '%.*s'", 2, lastCmdCopy);
                    radioManager.sendToSource(*pendingSource, "?;");
                } else if (radioManager.getState().queryTracker.wasRecentlyQueried(prefix, currentTime)) {
                    // Ring miss (evicted by later traffic, or sent before this
                    // dispatcher instance's ring existed): fall back to the
                    // pre-existing global-tracker gate rather than newly
                    // suppressing a response a client may still be waiting on.
                    ESP_LOGV(CommandDispatcher::TAG, "Forwarding '?;' to CDC0 (ring miss, queryTracker hit) — response to pending query '%.*s'", 2, lastCmdCopy);
                    radioManager.sendDirectResponse("?;");
                } else {
                    ESP_LOGV(CommandDispatcher::TAG, "Suppressing '?;' — no pending query for '%.*s'", 2, lastCmdCopy);
                }
            } else {
                ESP_LOGV(CommandDispatcher::TAG, "Suppressing '?;' — no command context");
            }
            stats_.commandsHandled++;
            return true;
        }
        // Perfect hash lookup: O(1) guaranteed, zero allocations
        // Fast path for all 2-character commands (99%+ of traffic)
        if (command.command.size() == 2) {
            const size_t hashIdx = compactHash(command.command[0], command.command[1]);
            if (ICommandHandler* handler = commandMap_[hashIdx]; handler != nullptr) {
                ESP_LOGV(CommandDispatcher::TAG, "Found handler for '%s', calling handleCommand", command.command.c_str());

                // Record command timing if it's a local command (will be sent to radio)
                if (command.isUsb()) {
                    {
                        RtosLockGuard<RtosMutex> lock(statsMutex_);
                        DispatcherStatistics::storeToBuf(stats_.lastCommandBeforeError, command.originalMessage);
                        DispatcherStatistics::storeToBuf(stats_.lastCommandSource, RadioCommand::sourceName(command.source));
                        stats_.recordCommandSent(esp_timer_get_time());
                    }
                    ESP_LOGV(CommandDispatcher::TAG, "LOCAL->RADIO: '%s' (via %s)",
                             command.originalMessage.c_str(), handler->getDescription().data());
                }

                if (handler->handleCommand(command, radioSerial, usbSerial, radioManager)) {
                    stats_.commandsHandled++;
                    ESP_LOGV(CommandDispatcher::TAG, "Command '%s' handled successfully via perfect hash", command.command.c_str());
                    // Mirror local SET commands to display (if present) to keep UI in sync
                    // Skip FA/FB Panel commands - EncoderHandler already sends to display with correct transverter offset
                    if ((command.isUsb() || command.source == CommandSource::Panel) && command.type == CommandType::Set) {
                        const bool isPanelFreqCmd = (command.source == CommandSource::Panel) &&
                                                    (command.command == "FA" || command.command == "FB");
                        if (!isPanelFreqCmd) {
                            if (auto* disp = radioManager.getDisplaySerial(); disp && disp != &usbSerial) {
                                ESP_LOGV(CommandDispatcher::TAG, "Mirroring SET to display: %s", command.originalMessage.c_str());
                                disp->sendMessage(command.originalMessage);
                            }
                        }
                    }

                    // Auto-query after SET commands when not in AI2/AI4 mode
                    if (shouldAutoQuery(command, radioManager)) {
                        const auto& readCmd = setToReadMap.at(command.command);
                        ESP_LOGV(CommandDispatcher::TAG, "Auto-querying after SET: %s -> %s",
                                 command.command.c_str(), readCmd.c_str());
                        // Route through the rate-paced TX queue so machine-generated
                        // read-backs share the global wire-rate bound (avoids ?; floods).
                        radioManager.sendRawRadioCommand(readCmd);
                    }

                    return true;
                }

                stats_.handlerErrors++;
                ESP_LOGW(CommandDispatcher::TAG, "Mapped handler failed to process command '%s' (source: %d)",
                         command.command.c_str(), static_cast<int>(command.source));
                if (command.isLocal()) {
                    radioManager.sendToSource(command.source, "?;");
                }
                return false;
            }
        }

        // Fallback: scan handlers (covers special cases or handlers without prefixes)
        for (const auto &handler: handlers_) {
            if (handler->canHandle(command)) {
                // Record command timing if it's a local command (will be sent to radio)
                if (command.isUsb()) {
                    {
                        RtosLockGuard<RtosMutex> lock(statsMutex_);
                        DispatcherStatistics::storeToBuf(stats_.lastCommandBeforeError, command.originalMessage);
                        DispatcherStatistics::storeToBuf(stats_.lastCommandSource, RadioCommand::sourceName(command.source));
                        stats_.recordCommandSent(esp_timer_get_time());
                    }
                    ESP_LOGV(CommandDispatcher::TAG, "LOCAL->RADIO: '%s' (via fallback handler)",
                             command.originalMessage.c_str());
                }
                
                if (handler->handleCommand(command, radioSerial, usbSerial, radioManager)) {
                    stats_.commandsHandled++;
                    ESP_LOGV(CommandDispatcher::TAG, "Command '%s' handled via fallback", command.command.c_str());
                    // Mirror local SET commands to display (if present)
                    // Skip FA/FB Panel commands - EncoderHandler already sends to display with correct transverter offset
                    if ((command.isUsb() || command.source == CommandSource::Panel) && command.type == CommandType::Set) {
                        const bool isPanelFreqCmd = (command.source == CommandSource::Panel) &&
                                                    (command.command == "FA" || command.command == "FB");
                        if (!isPanelFreqCmd) {
                            if (auto* disp = radioManager.getDisplaySerial(); disp && disp != &usbSerial) {
                                ESP_LOGV(CommandDispatcher::TAG, "Mirroring SET to display: %s", command.originalMessage.c_str());
                                disp->sendMessage(command.originalMessage);
                            }
                        }
                    }

                    // Auto-query after SET commands when not in AI2/AI4 mode
                    if (shouldAutoQuery(command, radioManager)) {
                        const auto& readCmd = setToReadMap.at(command.command);
                        ESP_LOGV(CommandDispatcher::TAG, "Auto-querying after SET (fallback): %s -> %s",
                                 command.command.c_str(), readCmd.c_str());
                        // Route through the rate-paced TX queue (see non-fallback path).
                        radioManager.sendRawRadioCommand(readCmd);
                    }
                    
                    return true;
                }

                stats_.handlerErrors++;
                ESP_LOGW(CommandDispatcher::TAG, "Fallback handler failed to process command '%s' (source: %d)",
                         command.command.c_str(), static_cast<int>(command.source));
                if (command.isLocal()) {
                    radioManager.sendToSource(command.source, "?;");
                }
                return false;
            }
        }

        // No handler found — reply ?; to local sources (like a real radio would)
        stats_.commandsUnhandled++;
        ESP_LOGW(CommandDispatcher::TAG, "No handler for command: '%s' (source: %d, original: '%s')",
                 command.command.c_str(),
                 static_cast<int>(command.source),
                 command.originalMessage.c_str());

        if (command.isLocal()) {
            radioManager.sendToSource(command.source, "?;");
        }
        return false;
    }

    std::vector<std::string> CommandDispatcher::getRegisteredHandlers() const {
        std::vector<std::string> descriptions;
        descriptions.reserve(handlers_.size());

        for (const auto &handler: handlers_) {
            descriptions.emplace_back(handler->getDescription());
        }

        return descriptions;
    }

    DispatcherStatistics CommandDispatcher::getStatistics() const {
        RtosLockGuard<RtosMutex> lock(statsMutex_);
        return stats_;  // Returns a copy, safe to access from any task
    }

    void CommandDispatcher::recordCommandSentToRadio(const std::string_view command, const std::string_view source) {
        RtosLockGuard<RtosMutex> lock(statsMutex_);
        DispatcherStatistics::storeToBuf(stats_.lastCommandBeforeError, command);
        DispatcherStatistics::storeToBuf(stats_.lastCommandSource, source);
        stats_.recordCommandSent(esp_timer_get_time());
    }

    void CommandDispatcher::resetStatistics() {
        RtosLockGuard<RtosMutex> lock(statsMutex_);
        stats_.reset();
    }

    void CommandDispatcher::recordPendingLocalRequest(const std::string_view prefix, const CommandSource source,
                                                       const uint64_t nowUs) {
        if (prefix.size() < 2)
            return;
        RtosLockGuard<RtosMutex> lock(statsMutex_);
        auto &e = pendingRequests_[pendingRequestWriteIdx_];
        e.prefix[0] = prefix[0];
        e.prefix[1] = prefix[1];
        e.source = source;
        e.timeUs = nowUs;
        pendingRequestWriteIdx_ = (pendingRequestWriteIdx_ + 1) % PENDING_REQUEST_RING_SIZE;
    }

    std::optional<CommandSource> CommandDispatcher::findAndConsumePendingRequest(const std::string_view prefix,
                                                                                  const uint64_t nowUs) {
        if (prefix.size() < 2)
            return std::nullopt;
        RtosLockGuard<RtosMutex> lock(statsMutex_);
        for (size_t i = 0; i < PENDING_REQUEST_RING_SIZE; ++i) {
            // Scan newest-first: pendingRequestWriteIdx_ is the next slot to be
            // written, so the most recently recorded entry is one behind it.
            const size_t idx = (pendingRequestWriteIdx_ + PENDING_REQUEST_RING_SIZE - 1 - i) % PENDING_REQUEST_RING_SIZE;
            auto &e = pendingRequests_[idx];
            if (e.timeUs == 0)
                continue; // empty/already-consumed slot
            if (nowUs - e.timeUs > PENDING_REQUEST_TTL_US)
                continue; // stale; a newer entry for the same prefix may still exist
            if (e.prefix[0] == prefix[0] && e.prefix[1] == prefix[1]) {
                const CommandSource src = e.source;
                e.timeUs = 0; // consume
                return src;
            }
        }
        return std::nullopt;
    }
} // namespace radio
