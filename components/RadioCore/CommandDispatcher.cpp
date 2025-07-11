#include "CommandDispatcher.h"
#include "ICommandHandler.h"  // Need full definition for implementation
#include "esp_timer.h"
#include <algorithm>
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
        
        // AI mode
        {"AI", "AI;"},     // Auto Information mode
        
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
            ESP_LOGD(CommandDispatcher::TAG, "PS query from %s",
                     command.source == CommandSource::Remote ? "Remote" : "Local");
        }

        ESP_LOGD(CommandDispatcher::TAG, "Dispatching command: '%s' (type: %s, source: %s, depth: %lu)",
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
                ESP_LOGI(CommandDispatcher::TAG, "Usb ';' received: %s responding with ID",  command.originalMessage.data());

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
            stats_.recordError(command.command, stats_.lastCommandBeforeError, currentTime);
            
            // Calculate time since last command for rate analysis
            const uint64_t timeSinceLastCmd = stats_.lastCommandTime > 0 ?
                currentTime - stats_.lastCommandTime : 0;

            // Enhanced error logging for debugging
            ESP_LOGI(CommandDispatcher::TAG, "=== ERROR RESPONSE DETECTED ===");
            ESP_LOGI(CommandDispatcher::TAG, "Error Type: '%s;'", command.command.c_str());
            ESP_LOGI(CommandDispatcher::TAG, "Last Command Sent: '%s'", stats_.lastCommandBeforeError.c_str());
            ESP_LOGI(CommandDispatcher::TAG, "Time Since Last Cmd: %.1fms", timeSinceLastCmd / 1000.0);
            ESP_LOGI(CommandDispatcher::TAG, "Error Interval: %.1fms", 
                     (stats_.lastErrorTime > 0) ? (currentTime - stats_.lastErrorTime) / 1000.0 : 0.0);
            ESP_LOGI(CommandDispatcher::TAG, "Total Errors: %zu (?;=%zu E;=%zu O;=%zu)",
                     stats_.totalErrorResponses, stats_.questionMarkErrors, stats_.eErrors, stats_.oErrors);
            
            // Special handling for RM-related errors
            if (stats_.lastCommandBeforeError.find("RM") != std::string::npos) {
                ESP_LOGW(CommandDispatcher::TAG, "RM command causing errors - may need to disable RM polling");
            }
            
            // Detailed diagnostic if we have multiple errors
            if (stats_.totalErrorResponses > 1) {
                ESP_LOGI(CommandDispatcher::TAG, "Error Pattern: Avg interval=%.1fms, Bursts=%zu",
                         stats_.averageErrorInterval / 1000.0, stats_.errorBursts);
                
                // Check for RM-specific error patterns
                if (stats_.totalErrorResponses > 10 && stats_.lastCommandBeforeError.find("RM") != std::string::npos) {
                    ESP_LOGE(CommandDispatcher::TAG, "HIGH ERROR COUNT with RM commands - consider disabling RM polling");
                }
            }
            
            // Warn about potential command rate issues
            if (timeSinceLastCmd > 0 && timeSinceLastCmd < 100000) { // < 100ms
                ESP_LOGW(CommandDispatcher::TAG, "WARNING: Fast error (%.1fms after command)",
                         timeSinceLastCmd / 1000.0);
            }
            ESP_LOGI(CommandDispatcher::TAG, "===============================");
            
            ESP_LOGD(CommandDispatcher::TAG,
                     "Processing error response '%s;' from radio",
                     command.command.c_str());
            
            // Don't forward '?;' errors to USB clients - they indicate invalid/unsupported commands
            // Only forward 'E;' and 'O;' errors which may be relevant to the USB client
            if (command.command != "?") {
                ESP_LOGD(CommandDispatcher::TAG, "Forwarding error response '%s;' to USB", command.command.c_str());
                radioManager.sendDirectResponse(command.originalMessage);
            } else {
                ESP_LOGD(CommandDispatcher::TAG, "Suppressing '?;' error response from USB forwarding");
            }
            stats_.commandsHandled++;
            return true;
        }
        // Perfect hash lookup: O(1) guaranteed, zero allocations
        // Fast path for all 2-character commands (99%+ of traffic)
        if (command.command.size() == 2) {
            const size_t hashIdx = compactHash(command.command[0], command.command[1]);
            if (ICommandHandler* handler = commandMap_[hashIdx]; handler != nullptr) {
                ESP_LOGD(CommandDispatcher::TAG, "Found handler for '%s', calling handleCommand", command.command.c_str());

                // Record command timing if it's a local command (will be sent to radio)
                if (command.isUsb()) {
                    stats_.recordCommandSent(esp_timer_get_time());
                    stats_.lastCommandBeforeError = command.originalMessage;
                    ESP_LOGD(CommandDispatcher::TAG, "LOCAL->RADIO: '%s' (via %s)",
                             command.originalMessage.c_str(), handler->getDescription().data());
                }

                if (handler->handleCommand(command, radioSerial, usbSerial, radioManager)) {
                    stats_.commandsHandled++;
                    ESP_LOGD(CommandDispatcher::TAG, "Command '%s' handled successfully via perfect hash", command.command.c_str());
                    // Mirror local SET commands to display (if present) to keep UI in sync
                    // Skip FA/FB Panel commands - EncoderHandler already sends to display with correct transverter offset
                    if ((command.isUsb() || command.source == CommandSource::Panel) && command.type == CommandType::Set) {
                        const bool isPanelFreqCmd = (command.source == CommandSource::Panel) &&
                                                    (command.command == "FA" || command.command == "FB");
                        if (!isPanelFreqCmd) {
                            if (auto* disp = radioManager.getDisplaySerial(); disp && disp != &usbSerial) {
                                ESP_LOGD(CommandDispatcher::TAG, "Mirroring SET to display: %s", command.originalMessage.c_str());
                                disp->sendMessage(command.originalMessage);
                            }
                        }
                    }

                    // Auto-query after SET commands when not in AI2/AI4 mode
                    if (shouldAutoQuery(command, radioManager)) {
                        const auto& readCmd = setToReadMap.at(command.command);
                        ESP_LOGD(CommandDispatcher::TAG, "Auto-querying after SET: %s -> %s",
                                 command.command.c_str(), readCmd.c_str());
                        radioSerial.sendMessage(readCmd);
                    }

                    return true;
                }

                stats_.handlerErrors++;
                ESP_LOGW(CommandDispatcher::TAG, "Mapped handler failed to process command '%s'",
                         command.command.c_str());
                return false;
            }
        }

        // Fallback: scan handlers (covers special cases or handlers without prefixes)
        for (const auto &handler: handlers_) {
            if (handler->canHandle(command)) {
                // Record command timing if it's a local command (will be sent to radio)
                if (command.isUsb()) {
                    stats_.recordCommandSent(esp_timer_get_time());
                    stats_.lastCommandBeforeError = command.originalMessage;
                    ESP_LOGD(CommandDispatcher::TAG, "LOCAL->RADIO: '%s' (via fallback handler)",
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
                                ESP_LOGD(CommandDispatcher::TAG, "Mirroring SET to display: %s", command.originalMessage.c_str());
                                disp->sendMessage(command.originalMessage);
                            }
                        }
                    }

                    // Auto-query after SET commands when not in AI2/AI4 mode
                    if (shouldAutoQuery(command, radioManager)) {
                        const auto& readCmd = setToReadMap.at(command.command);
                        ESP_LOGD(CommandDispatcher::TAG, "Auto-querying after SET (fallback): %s -> %s",
                                 command.command.c_str(), readCmd.c_str());
                        radioSerial.sendMessage(readCmd);
                    }
                    
                    return true;
                }

                stats_.handlerErrors++;
                ESP_LOGW(CommandDispatcher::TAG, "Fallback handler failed to process command '%s'",
                         command.command.c_str());
                return false;
            }
        }

        // No handler found
        stats_.commandsUnhandled++;
        ESP_LOGW(CommandDispatcher::TAG, "No handler found for command: '%s'", command.command.c_str());
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
} // namespace radio
