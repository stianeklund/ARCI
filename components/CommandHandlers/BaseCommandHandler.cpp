#include "BaseCommandHandler.h"
#include "DisplayLatencyProfiler.h"
#include "RadioManager.h"
#include "ForwardingPolicy.h"
#include "esp_log.h"
#include <algorithm>
#include <array>
#include <charconv>
#include <string>
#include <vector>
#include "ISerialChannel.h"

namespace radio {
    BaseCommandHandler::BaseCommandHandler(const std::initializer_list<std::string_view> supportedCommands,
                                           const std::string_view description)
        : numCommands_(std::min(supportedCommands.size(), static_cast<size_t>(MAX_COMMANDS)))
          , description_(description) {
        // Copy string_views to our array - bounds already checked
        std::copy_n(supportedCommands.begin(), numCommands_, std::begin(supportedCommands_));

        ESP_LOGD(BaseCommandHandler::TAG, "Created handler: %.*s with %zu commands",
                 static_cast<int>(description_.size()), description_.data(), numCommands_);
    }

    bool BaseCommandHandler::canHandle(const RadioCommand &command) const {
        ESP_LOGV(TAG, "canHandle checking for %.*s in %.*s",
                 static_cast<int>(command.command.size()), command.command.data(),
                 static_cast<int>(description_.size()), description_.data());

        // ReSharper disable twice CppTooWideScopeInitStatement
        const auto *end = supportedCommands_ + numCommands_;
        const auto *found = std::find(supportedCommands_, end, command.command);

        if (found != end) {
            ESP_LOGV(TAG, "canHandle: FOUND %.*s in %.*s",
                     static_cast<int>(command.command.size()), command.command.data(),
                     static_cast<int>(description_.size()), description_.data());
            return true;
        }
        return false;
    }

    std::vector<std::string_view> BaseCommandHandler::getPrefixes() const {
        std::vector<std::string_view> prefixes;
        prefixes.reserve(numCommands_);
        for (size_t i = 0; i < numCommands_; ++i) {
            prefixes.push_back(supportedCommands_[i]);
        }
        return prefixes;
    }

    void BaseCommandHandler::sendToRadio(ISerialChannel &radioSerial, const std::string_view commandStr) {
        if (commandStr.empty()) [[unlikely]] {
            ESP_LOGW(BaseCommandHandler::TAG, "Attempted to send empty command to radio");
            return;
        }
        ESP_LOGD(BaseCommandHandler::TAG, "Sending to radio: %.*s",
                 static_cast<int>(commandStr.size()), commandStr.data());
        const esp_err_t result = radioSerial.sendMessage(commandStr);
        if (result != ESP_OK) {
            ESP_LOGE(BaseCommandHandler::TAG,
                     "❌ Dropped radio frame (%.*s): %s (failures=%u)",
                     static_cast<int>(commandStr.size()),
                     commandStr.data(),
                     esp_err_to_name(result),
                     radioSerial.getSendFailureCount());
        }
    }

    void BaseCommandHandler::sendToUSB(ISerialChannel &usbSerial, const std::string_view responseStr) {
        if (responseStr.empty()) [[unlikely]] {
            ESP_LOGW(BaseCommandHandler::TAG, "Attempted to send empty response to USB");
            return;
        }
        ESP_LOGV(BaseCommandHandler::TAG, "Sending to USB: %.*s",
                 static_cast<int>(responseStr.size()), responseStr.data());

        const esp_err_t result = usbSerial.sendMessage(responseStr);

        if (result != ESP_OK) {
            // Rate-limit backpressure errors to avoid log spam during USB buffer congestion
            static uint32_t backpressureErrorCount = 0;
            static uint64_t lastBackpressureLogUs = 0;
            constexpr uint64_t BACKPRESSURE_LOG_INTERVAL_US = 5000000; // Log once per 5 seconds max

            if (result == ESP_ERR_NO_MEM) {
                // Backpressure - CDC buffer full. Rate-limit logging.
                backpressureErrorCount++;
                const uint64_t nowUs = esp_timer_get_time();
                if (nowUs - lastBackpressureLogUs > BACKPRESSURE_LOG_INTERVAL_US) {
                    ESP_LOGW(BaseCommandHandler::TAG,
                             "USB backpressure: dropped %lu responses in last %.1f seconds (buffer full)",
                             backpressureErrorCount, (nowUs - lastBackpressureLogUs) / 1000000.0);
                    backpressureErrorCount = 0;
                    lastBackpressureLogUs = nowUs;
                }
            } else {
                // Other errors are unexpected and should always be logged
                ESP_LOGE(BaseCommandHandler::TAG, "Failed to send to USB: %s", esp_err_to_name(result));
            }
        }
    }

    void BaseCommandHandler::respondToSource(const RadioCommand &command, const std::string_view response,
                                             ISerialChannel &usbSerial, const RadioManager &radioManager) {
        if (response.empty()) [[unlikely]] {
            ESP_LOGW(BaseCommandHandler::TAG, "Attempted to send empty response to source");
            return;
        }

        switch (command.source) {
            case CommandSource::UsbCdc0:
            case CommandSource::UsbCdc1:
                // Usb commands come from USB interface
                ESP_LOGD(BaseCommandHandler::TAG, "Responding to USB: %.*s",
                         static_cast<int>(response.size()), response.data());
                sendToUSB(usbSerial, response);
                break;

            case CommandSource::Tcp0:
                // TCP port 0 commands go back to TCP0
                ESP_LOGD(BaseCommandHandler::TAG, "Responding to TCP0: %.*s",
                         static_cast<int>(response.size()), response.data());
                radioManager.sendToSource(CommandSource::Tcp0, response);
                break;

            case CommandSource::Tcp1:
                // TCP port 1 commands go back to TCP1
                ESP_LOGD(BaseCommandHandler::TAG, "Responding to TCP1: %.*s",
                         static_cast<int>(response.size()), response.data());
                radioManager.sendToSource(CommandSource::Tcp1, response);
                break;

            case CommandSource::Display:
                // Display commands go back to display
                ESP_LOGD(BaseCommandHandler::TAG, "Responding to Display: %.*s",
                         static_cast<int>(response.size()), response.data());
                radioManager.sendToDisplay(response);
                break;

            case CommandSource::Panel:
                // Panel-origin commands: forward to USB to keep host CAT apps in sync
                ESP_LOGD(BaseCommandHandler::TAG, "Responding to USB (Panel-origin): %.*s",
                         static_cast<int>(response.size()), response.data());
                sendToUSB(usbSerial, response);
                break;

            case CommandSource::Macro:
                // Macro commands: no direct response forwarding
                // Macro-initiated responses update internal state only
                // AI policy will forward to USB when AI mode is 2/4
                ESP_LOGV(BaseCommandHandler::TAG, "Macro query response (internal-only): %.*s",
                         static_cast<int>(response.size()), response.data());
                break;

            case CommandSource::Remote:
                // For Remote/Radio responses, the handler has already made the policy decision
                // to call respondToSource, so we trust that decision and send directly
                // This avoids double policy checks that can fail due to rate limiting
                ESP_LOGD(BaseCommandHandler::TAG, "Sending Remote->USB: %.*s",
                         static_cast<int>(response.size()), response.data());
                sendToUSB(usbSerial, response);
                break;
        }
    }

    void BaseCommandHandler::routeAnswerResponse(const RadioCommand &command, const std::string_view response,
                                                ISerialChannel &usbSerial, const RadioManager &radioManager) {
        if (response.empty()) [[unlikely]] {
            ESP_LOGW(BaseCommandHandler::TAG, "Attempted to route empty response");
            return;
        }

        // This method should only be called for Answer commands from radio
        if (command.type != CommandType::Answer || command.source != CommandSource::Remote) {
            ESP_LOGW(BaseCommandHandler::TAG, "routeAnswerResponse called for non-Answer command: type=%d, source=%d", 
                     (int)command.type, (int)command.source);
            return;
        }

        const uint64_t currentTime = esp_timer_get_time();
        
        // Try strict pairing first: route to the interface that initiated the READ
        std::optional<CommandSource> strictPairedSource = std::nullopt;
        if (response.size() >= 2) {
            const std::string_view prefix = response.substr(0, 2);
            strictPairedSource = radioManager.routeMatchedAnswerWithSource(prefix, response, currentTime);
        }
        
        // Always check AI forwarding policy regardless of strict pairing
        // This ensures AI-enabled interfaces receive unsolicited updates
        // Skip interfaces that already received the message via strict pairing
        
        // Check if should forward to USB CDC0 using policy (avoid duplicate)
        const bool shouldForwardCdc0 = ForwardingPolicy::shouldForwardToUsbCdc0(response, radioManager.getState(), currentTime);

        // DEBUG: Add special logging for IF responses
        if (response.length() >= 2 && response.substr(0, 2) == "IF") {
            ESP_LOGV(BaseCommandHandler::TAG, "🔍 IF ROUTE: strictPaired=%s, shouldForward=%s",
                     strictPairedSource == CommandSource::UsbCdc0 ? "CDC0" : "OTHER",
                     shouldForwardCdc0 ? "YES" : "NO");
        }

        if (strictPairedSource != CommandSource::UsbCdc0 && shouldForwardCdc0) {
            ESP_LOGI(BaseCommandHandler::TAG, "📤 Routing answer to CDC0: %.*s",
                     static_cast<int>(response.size()), response.data());
            sendToUSB(usbSerial, response);
            if (response.length() >= 2 && response.substr(0, 2) == "FA") {
                DisplayLatencyProfiler::instance().markUsbCdc0Forwarded();
            }
        }
        // Check if should forward to USB CDC1 using policy (avoid duplicate)
        const bool shouldForwardCdc1 = ForwardingPolicy::shouldForwardToUsbCdc1(response, radioManager.getState(), currentTime);

        // DEBUG: Add special logging for IF responses
        if (response.length() >= 2 && response.substr(0, 2) == "IF") {
            ESP_LOGV(BaseCommandHandler::TAG, "🔍 IF ROUTE CDC1: strictPaired=%s, shouldForward=%s",
                     strictPairedSource == CommandSource::UsbCdc1 ? "CDC1" : "OTHER",
                     shouldForwardCdc1 ? "YES" : "NO");
        }

        if (strictPairedSource != CommandSource::UsbCdc1 && shouldForwardCdc1) {
            ESP_LOGD(BaseCommandHandler::TAG, "Routing answer to USB1: %.*s",
                     static_cast<int>(response.size()), response.data());
            radioManager.sendToSource(CommandSource::UsbCdc1, response);
        }

        // Check if should forward to TCP0 using policy (avoid duplicate)
        if (strictPairedSource != CommandSource::Tcp0 &&
            ForwardingPolicy::shouldForwardToTcp0(response, radioManager.getState(), currentTime)) {
            ESP_LOGD(BaseCommandHandler::TAG, "Routing answer to TCP0: %.*s",
                     static_cast<int>(response.size()), response.data());
            radioManager.sendToSource(CommandSource::Tcp0, response);
        }

        // Check if should forward to TCP1 using policy (avoid duplicate)
        if (strictPairedSource != CommandSource::Tcp1 &&
            ForwardingPolicy::shouldForwardToTcp1(response, radioManager.getState(), currentTime)) {
            ESP_LOGD(BaseCommandHandler::TAG, "Routing answer to TCP1: %.*s",
                     static_cast<int>(response.size()), response.data());
            radioManager.sendToSource(CommandSource::Tcp1, response);
        }

        // Check if should forward to display using policy (avoid duplicate)
        if (strictPairedSource != CommandSource::Display &&
            ForwardingPolicy::shouldForwardToDisplay(response, radioManager.getState(), currentTime)) {
            ESP_LOGD(BaseCommandHandler::TAG, "Routing answer to Display: %.*s",
                     static_cast<int>(response.size()), response.data());
            radioManager.sendToDisplay(response);
            if (response.length() >= 2 && response.substr(0, 2) == "FA") {
                auto &profiler = DisplayLatencyProfiler::instance();
                profiler.markDisplayForwarded();
                profiler.markDisplayForwardInterval();
            }
        }
    }

    void BaseCommandHandler::routeSetCommandToAIInterfaces(const RadioCommand& originCommand,
                                                          const std::string_view response,
                                                          ISerialChannel& usbSerial,
                                                          const RadioManager& radioManager) {
        if (response.empty()) [[unlikely]] {
            return;
        }

        const uint64_t currentTime = esp_timer_get_time();

        // Don't echo back to the originating interface to avoid loops
        // But do send to other AI-enabled interfaces to keep them in sync

        // Check and route to USB CDC0 if AI mode is enabled and not the origin
        if (originCommand.source != CommandSource::UsbCdc0 &&
            ForwardingPolicy::shouldForwardToUsbCdc0(response, radioManager.getState(), currentTime)) {
            ESP_LOGD(BaseCommandHandler::TAG, "Routing SET to USB0: %.*s",
                     static_cast<int>(response.size()), response.data());
            radioManager.sendToSource(CommandSource::UsbCdc0, response);
        }

        // Check and route to USB CDC1 if AI mode is enabled and not the origin
        if (originCommand.source != CommandSource::UsbCdc1 &&
            ForwardingPolicy::shouldForwardToUsbCdc1(response, radioManager.getState(), currentTime)) {
            ESP_LOGD(BaseCommandHandler::TAG, "Routing SET to USB1: %.*s",
                     static_cast<int>(response.size()), response.data());
            radioManager.sendToSource(CommandSource::UsbCdc1, response);
        }

        // Check and route to TCP0 if AI mode is enabled and not the origin
        if (originCommand.source != CommandSource::Tcp0 &&
            ForwardingPolicy::shouldForwardToTcp0(response, radioManager.getState(), currentTime)) {
            ESP_LOGD(BaseCommandHandler::TAG, "Routing SET to TCP0: %.*s",
                     static_cast<int>(response.size()), response.data());
            radioManager.sendToSource(CommandSource::Tcp0, response);
        }

        // Check and route to TCP1 if AI mode is enabled and not the origin
        if (originCommand.source != CommandSource::Tcp1 &&
            ForwardingPolicy::shouldForwardToTcp1(response, radioManager.getState(), currentTime)) {
            ESP_LOGD(BaseCommandHandler::TAG, "Routing SET to TCP1: %.*s",
                     static_cast<int>(response.size()), response.data());
            radioManager.sendToSource(CommandSource::Tcp1, response);
        }

        // Always send to display for UI updates (unless display is the origin)
        if (originCommand.source != CommandSource::Display) {
            if (auto* disp = radioManager.getDisplaySerial(); disp) {
                ESP_LOGD(BaseCommandHandler::TAG, "Routing SET to Display: %.*s",
                         static_cast<int>(response.size()), response.data());
                disp->sendMessage(response);
            }
        }
    }

    std::string BaseCommandHandler::buildCommand(const std::string_view prefix, const std::string_view params) {
        // Reserve space to avoid reallocations and use safe string concatenation
        std::string result;
        result.reserve(prefix.size() + params.size() + 1); // +1 for semicolon
        result.append(prefix);
        result.append(params);
        result.append(";");
        return result;
    }

    int BaseCommandHandler::getIntParam(const RadioCommand &command, size_t index, int defaultValue) const {
        if (index >= command.params.size()) [[unlikely]] {
            return defaultValue;
        }

        // Use std::visit for cleaner variant handling
        return std::visit([this, index, defaultValue]<typename T0>(const T0 &value) -> int {
            using T = std::decay_t<T0>;

            if constexpr (std::is_same_v<T, int>) {
                return value;
            } else if constexpr (std::is_same_v<T, std::string>) {
                int result = 0;
                const auto [ptr, ec] = std::from_chars(value.data(), value.data() + value.size(), result);

                if (ec == std::errc{} && ptr == value.data() + value.size()) [[likely]] {
                    return result;
                }

                ESP_LOGW(BaseCommandHandler::TAG, "Failed to parse int parameter %zu: invalid format", index);
                return defaultValue;
            } else {
                return defaultValue;
            }
        }, command.params[index]);
    }

    std::string BaseCommandHandler::getStringParam(const RadioCommand &command, const size_t index,
                                                   std::string_view defaultValue) {
        if (index >= command.params.size()) [[unlikely]] {
            return std::string{defaultValue};
        }

        // Use std::visit for cleaner variant handling
        return std::visit([]<typename T0>(const T0 &value) -> std::string {
            using T = std::decay_t<T0>;

            if constexpr (std::is_same_v<T, std::string>) {
                return value;
            } else if constexpr (std::is_same_v<T, int>) {
                return std::to_string(value);
            } else {
                return std::string{};
            }
        }, command.params[index]);
    }

    bool BaseCommandHandler::isCacheFresh(const RadioManager &radioManager, std::string_view command,
                                          const uint64_t ttlUs) {
        const uint64_t currentTime = esp_timer_get_time();
        return radioManager.getState().commandCache.isFresh(std::string{command}, currentTime, ttlUs);
    }

    bool BaseCommandHandler::handleLocalQueryStandard(const RadioCommand &command,
                                                      ISerialChannel &radioSerial,
                                                      ISerialChannel &usbSerial,
                                                      RadioManager &radioManager,
                                                      const std::string_view key,
                                                      const uint64_t ttlUs,
                                                      const std::function<std::string(const RadioState &)> &buildResponse,
                                                      const std::string_view readParam) {
        if (command.isLocal()) {
            const auto &state = radioManager.getState();

            // When user has requested power off, don't forward queries to radio
            // This prevents keeping the radio "awake" with ongoing communication
            if (state.powerOffRequestTime.load() > 0) {
                // Return cached value if available, otherwise return empty/error
                const std::string keyStr{key};
                const uint64_t lastUpdateUs = state.commandCache.get(keyStr);
                if (lastUpdateUs != 0U) {
                    const std::string response = buildResponse(state);
                    respondToSource(command, response, usbSerial, radioManager);
                    ESP_LOGD(BaseCommandHandler::TAG,
                             "⚡ Power off: Serving cached %.*s (not forwarding to radio)",
                             static_cast<int>(key.size()), key.data());
                } else {
                    ESP_LOGD(BaseCommandHandler::TAG,
                             "⚡ Power off: Ignoring %.*s query (no cache, not forwarding to radio)",
                             static_cast<int>(key.size()), key.data());
                }
                return true;
            }

            // During transmission, always respond with current state to match TS-590SG behavior
            // Real TS-590SG continues to respond to CAT queries during TX using internal state
            if (state.isTx.load()) {
                const std::string response = buildResponse(state);
                respondToSource(command, response, usbSerial, radioManager);
                ESP_LOGD(BaseCommandHandler::TAG,
                         "🔄 TX Mode: Responding with current state for %.*s",
                         static_cast<int>(key.size()), key.data());
                return true;
            }

            const std::string keyStr{key};
            const uint64_t lastUpdateUs = state.commandCache.get(keyStr);
            const bool hasCachedValue = lastUpdateUs != 0U;
            const bool cacheFresh = !command.bypassCache && isCacheFresh(radioManager, key, ttlUs);
            const bool shouldRefresh = command.bypassCache || !cacheFresh;

            if (cacheFresh && hasCachedValue) {
                const std::string response = buildResponse(state);
                respondToSource(command, response, usbSerial, radioManager);
                return true;
            }

            if (hasCachedValue) {
                const std::string response = buildResponse(state);
                respondToSource(command, response, usbSerial, radioManager);
                ESP_LOGD(BaseCommandHandler::TAG,
                         "Cache stale for %.*s (last=%llu us) - served cached data and refreshing",
                         static_cast<int>(key.size()), key.data(),
                         static_cast<unsigned long long>(lastUpdateUs));
                // NOTE: Don't record query origin here - we already sent cached response
                // Recording origin would cause radio response to be routed back as duplicate
            } else {
                // Rate-limit "Cache empty" logs to prevent spam during cache stampedes
                // (e.g., after TX/RX transitions that invalidate IF cache while CAT clients poll aggressively)
                // Use fixed-size array with hash for O(1) lookup, no heap allocation
                static constexpr size_t LOG_THROTTLE_SLOTS = 64;  // Power of 2 for fast modulo
                static std::array<uint64_t, LOG_THROTTLE_SLOTS> lastCacheEmptyLogTime{};
                constexpr uint64_t CACHE_EMPTY_LOG_INTERVAL_US = 2000000; // Log once per 2 seconds per command
                const uint64_t nowUs = esp_timer_get_time();

                // Simple hash for 2-char command prefixes
                const size_t slot = (key.size() >= 2)
                    ? ((static_cast<size_t>(key[0]) * 31 + static_cast<size_t>(key[1])) & (LOG_THROTTLE_SLOTS - 1))
                    : 0;
                const uint64_t lastLogTime = lastCacheEmptyLogTime[slot];

                if (lastLogTime == 0 || (nowUs - lastLogTime) > CACHE_EMPTY_LOG_INTERVAL_US) {
                    ESP_LOGI(BaseCommandHandler::TAG,
                             "Cache empty for %.*s - forwarding query to radio (source=%d)",
                             static_cast<int>(key.size()), key.data(), static_cast<int>(command.source));
                    lastCacheEmptyLogTime[slot] = nowUs;
                } else {
                    ESP_LOGV(BaseCommandHandler::TAG,
                             "Cache empty for %.*s (rate-limited, source=%d)",
                             static_cast<int>(key.size()), key.data(), static_cast<int>(command.source));
                }
            }

            if (shouldRefresh) {
                // Record local query and capture origin for strict pairing
                const uint64_t nowUs = esp_timer_get_time();
                constexpr uint64_t MIN_RESEND_DELAY_US = 200000; // 200 ms guard between refreshes
                const bool recentRefresh = state.queryTracker.wasRecentlyQueried(keyStr, nowUs, MIN_RESEND_DELAY_US);

                if (recentRefresh) {
                    ESP_LOGD(BaseCommandHandler::TAG,
                             "Refresh for %.*s already in-flight (last=%llu us); skipping resend",
                             static_cast<int>(key.size()), key.data(),
                             static_cast<unsigned long long>(state.queryTracker.get(keyStr)));
                } else {
                    ESP_LOGD(BaseCommandHandler::TAG,
                             "Refreshing %.*s (TTL=%llu us) after local request (source=%d)",
                             static_cast<int>(key.size()), key.data(),
                             static_cast<unsigned long long>(ttlUs), static_cast<int>(command.source));
                    // Skip query tracking for Macro commands (don't pollute strict pairing)
                    if (command.source != CommandSource::Macro) {
                        state.queryTracker.recordQuery(keyStr, nowUs);
                        // Only record origin if we didn't send cached response (to avoid duplicates)
                        if (!hasCachedValue) {
                            radioManager.noteQueryOrigin(key, command.source, nowUs);
                        }
                    }
                    if (!readParam.empty()) {
                        sendToRadio(radioSerial, buildCommand(key, readParam));
                    } else {
                        sendToRadio(radioSerial, buildCommand(key));
                    }
                }
            }

            // If we do not have any cached value yet, we rely on the radio’s response.
            // The refresh path above will forward the first real answer once available.
            return true;
        }

        if (shouldSendToRadio(command)) {
            if (!readParam.empty()) {
                sendToRadio(radioSerial, buildCommand(key, readParam));
            } else {
                sendToRadio(radioSerial, buildCommand(key));
            }
            return true;
        }

        return true;
    }
} // namespace radio
