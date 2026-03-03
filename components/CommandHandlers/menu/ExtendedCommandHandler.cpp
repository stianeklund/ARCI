#include "ExtendedCommandHandler.h"
#include "RadioManager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <array>
#include <iomanip>
#include <sstream>
#include <string_view>

namespace radio {
    namespace {
        using namespace std::string_view_literals;

        constexpr auto MENU_ITEMS = std::to_array<ExtendedMenuItem>({
            ExtendedMenuItem{"000"sv, "Version information"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"001"sv, "Power on message"sv, ""sv},
            ExtendedMenuItem{"002"sv, "Display brightness"sv, "0|1|2|3|4|5|6"sv},
            ExtendedMenuItem{"003"sv, "Back light color"sv, "0|1|2|3|4|5|6|7|8|9|10"sv},
            ExtendedMenuItem{"004"sv, "Panel key response for double function"sv, "1|2|3"sv},
            ExtendedMenuItem{"005"sv, "Beep volume"sv, "0|1|2|3|4|5|6|7|8|9"sv},
            ExtendedMenuItem{"006"sv, "Sidetone volume"sv, "0|1|2|3|4|5|6|7|8|9"sv},
            ExtendedMenuItem{"007"sv, "Message playback volume"sv, "0|1|2|3|4|5|6|7|8|9"sv},
            ExtendedMenuItem{"008"sv, "Voice guide volume"sv, "0|1|2|3|4|5|6|7|8|9"sv},
            ExtendedMenuItem{"009"sv, "Voice guide speed"sv, "0|1|2|3|4"sv},
            ExtendedMenuItem{"010"sv, "Voice guide language"sv, "EN|JP"sv},
            ExtendedMenuItem{"011"sv, "Auto announcement"sv, "0|1|2"sv},
            ExtendedMenuItem{"012"sv, "MHz step (MHz)"sv, "0|1|2"sv},
            ExtendedMenuItem{"013"sv, "Tuning control adjustment rate (Hz)"sv, "0|1|2"sv},
            ExtendedMenuItem{"014"sv, "MULTI/CH control rounding off process"sv, "0|1"sv},
            ExtendedMenuItem{"015"sv, "Dedicated step change inside BC band (AM)"sv, "0|1"sv},
            ExtendedMenuItem{"016"sv, "MULTI/CH control step change for SSB (kHz)"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"017"sv, "MULTI/CH control step change for CW/FSK (kHz)"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"018"sv, "MULTI/CH control step change for AM (kHz)"sv, "0|1|2|3|4|5|6|7|8|9|10|11|12|13"sv},
            ExtendedMenuItem{"019"sv, "MULTI/CH control step change for FM (kHz)"sv, "0|1|2|3|4|5|6|7|8|9|10|11|12|13"sv},
            ExtendedMenuItem{"020"sv, "Shiftable RX frequency during split transmission"sv, "0|1"sv},
            ExtendedMenuItem{"021"sv, "Maximum number of Quick Memory channels"sv, "3|5|10"sv},
            ExtendedMenuItem{"022"sv, "Temporary variable of the standard/extension memory frequency"sv, "0|1"sv},
            ExtendedMenuItem{"023"sv, "Program Scan slow down function"sv, "0|1"sv},
            ExtendedMenuItem{"024"sv, "Program Scan slow down frequency range (Hz)"sv, "0|1|2|3|4"sv},
            ExtendedMenuItem{"025"sv, "Program Scan hold"sv, "0|1"sv},
            ExtendedMenuItem{"026"sv, "Scan Resume method"sv, "TO|CO"sv},
            ExtendedMenuItem{"027"sv, "Auto mode change"sv, "0|1"sv},
            ExtendedMenuItem{"028"sv, "Low Cut / Low Cut and Width/Shift change (SSB)"sv, "1|2"sv},
            ExtendedMenuItem{"029"sv, "Low Cut / Low Cut and Width/Shift change (SSB-DATA)"sv, "1|2"sv},
            ExtendedMenuItem{"030"sv, "Following speed setting of AUTO NOTCH"sv, "0|1|2|3|4"sv},
            ExtendedMenuItem{"031"sv, "SSB/AM Low Cut transmit filter (Hz)"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"032"sv, "SSB/AM High Cut transmit filter (Hz)"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"033"sv, "SSB-DATA Low Cut transmit filter (Hz)"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"034"sv, "SSB-DATA High Cut transmit filter (Hz)"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"035"sv, "Effective change of Speech Processor"sv, "SOFT|HARD"sv},
            ExtendedMenuItem{"036"sv, "Transmit equalizer"sv, "0|1|2|3|4|5|6|7|8"sv},
            ExtendedMenuItem{"037"sv, "Receive equalizer"sv, "0|1|2|3|4|5|6|7|8"sv},
            ExtendedMenuItem{"038"sv, "Electronic keyer operation mode"sv, "A|B"sv},
            ExtendedMenuItem{"039"sv, "Insert keying ON/OFF"sv, "0|1"sv},
            ExtendedMenuItem{"040"sv, "Side tone/pitch frequency setting (Hz)"sv, "0|1|2|3|4|5|6|7|8|9|10|11|12|13"sv},
            ExtendedMenuItem{"041"sv, "CW clipping (ms)"sv, "1|2|4|6"sv},
            ExtendedMenuItem{"042"sv, "Keying weight ratio"sv, "AUTO|0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15"sv},
            ExtendedMenuItem{"043"sv, "Reverse keying auto weight ratio"sv, "0|1"sv},
            ExtendedMenuItem{"044"sv, "Bug key function"sv, "0|1"sv},
            ExtendedMenuItem{"045"sv, "Paddle dot/dash replacement setting"sv, "PA|PF"sv},
            ExtendedMenuItem{"046"sv, "Mic paddle function"sv, "0|1"sv},
            ExtendedMenuItem{"047"sv, "Auto CW TX in SSB mode"sv, "0|1"sv},
            ExtendedMenuItem{"048"sv, "Frequency correction for changing SSB to CW mode"sv, "0|1"sv},
            ExtendedMenuItem{"049"sv, "Break-in null configuration at time of keying speed configuration"sv, "0|1"sv},
            ExtendedMenuItem{"050"sv, "FSK shift"sv, "0|1|2|3"sv},
            ExtendedMenuItem{"051"sv, "FSK keying polarity"sv, "0|1"sv},
            ExtendedMenuItem{"052"sv, "FSK tone frequency (Hz)"sv, "0|1"sv},
            ExtendedMenuItem{"053"sv, "Mic gain for FM"sv, "1|2|3"sv},
            ExtendedMenuItem{"054"sv, "Power fine"sv, "0|1"sv},
            ExtendedMenuItem{"055"sv, "Time-out Timer"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"056"sv, "Configuring the Transverter function and power down"sv, "0|1|2"sv},
            ExtendedMenuItem{"057"sv, "TX hold when AT completes tuning"sv, "0|1"sv},
            ExtendedMenuItem{"058"sv, "AT operation when receiving"sv, "0|1"sv},
            ExtendedMenuItem{"059"sv, "HF linear amplifier control"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"060"sv, "50 MHz linear amplifier control"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"061"sv, "Constant recording"sv, "0|1"sv},
            ExtendedMenuItem{"062"sv, "Voice/message playback repeat"sv, "0|1"sv},
            ExtendedMenuItem{"063"sv, "Voice/message playback repeat duration (seconds)"sv, "0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16|17|18|19|20"sv},
            ExtendedMenuItem{"064"sv, "Split transfer function"sv, "0|1"sv},
            ExtendedMenuItem{"065"sv, "Write split transfer data to the VFO"sv, "0|1"sv},
            ExtendedMenuItem{"066"sv, "Transmit inhibit"sv, "0|1"sv},
            ExtendedMenuItem{"067"sv, "COM port communication speed"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"068"sv, "USB port communication speed"sv, "0|1|2|3|4|5"sv},
            ExtendedMenuItem{"069"sv, "DATA modulation line"sv, "ACC2|USB"sv},
            ExtendedMenuItem{"070"sv, "Audio source of SEND/PTT transmission for data mode"sv, "FRONT|REAR"sv},
            ExtendedMenuItem{"071"sv, "USB audio input level"sv, "0|1|2|3|4|5|6|7|8|9"sv},
            ExtendedMenuItem{"072"sv, "USB audio output level"sv, "0|1|2|3|4|5|6|7|8|9"sv},
            ExtendedMenuItem{"073"sv, "ACC2 terminal AF input level"sv, "0|1|2|3|4|5|6|7|8|9"sv},
            ExtendedMenuItem{"074"sv, "ACC2 terminal AF output level"sv, "0|1|2|3|4|5|6|7|8|9"sv},
            ExtendedMenuItem{"075"sv, "External AF output beep mix"sv, "0|1"sv},
            ExtendedMenuItem{"076"sv, "DATA VOX"sv, "0|1"sv},
            ExtendedMenuItem{"077"sv, "DATA VOX delay"sv, "0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16|17|18|19|20"sv},
            ExtendedMenuItem{"078"sv, "DATA VOX gain for USB audio input"sv, "0|1|2|3|4|5|6|7|8|9"sv},
            ExtendedMenuItem{"079"sv, "DATA VOX gain for ACC2 terminal input"sv, "0|1|2|3|4|5|6|7|8|9"sv},
            ExtendedMenuItem{"080"sv, "PKS polarity change"sv, "0|1"sv},
            ExtendedMenuItem{"081"sv, "Busy transmit inhibit"sv, "0|1"sv},
            ExtendedMenuItem{"082"sv, "CTCSS mute operation change"sv, "1|2"sv},
            ExtendedMenuItem{"083"sv, "PSQ control signal logic selection"sv, "LO|OPEN"sv},
            ExtendedMenuItem{"084"sv, "PSQ control signal output condition"sv, "0|1|2|3|4|5"sv},
            // Accept both named and numeric synonyms for EX085
            ExtendedMenuItem{"085"sv, "DRV connector output function"sv, "DRO|ANT|0|1"sv},
            ExtendedMenuItem{"086"sv, "APO function (minutes)"sv, "0|1|2|3"sv},
            ExtendedMenuItem{"087"sv, "Panel PF A function"sv, ""sv},
            ExtendedMenuItem{"088"sv, "Panel PF B function"sv, ""sv},
            ExtendedMenuItem{"089"sv, "RIT Key function"sv, ""sv},
            ExtendedMenuItem{"090"sv, "XIT Key function"sv, ""sv},
            ExtendedMenuItem{"091"sv, "CL Key function"sv, ""sv},
            ExtendedMenuItem{"092"sv, "Front panel MULTI/CH key assignment (exclude CW mode)"sv, ""sv},
            ExtendedMenuItem{"093"sv, "Front panel MULTI/CH key assignment (CW mode)"sv, ""sv},
            ExtendedMenuItem{"094"sv, "Mic PF 1 function"sv, ""sv},
            ExtendedMenuItem{"095"sv, "Mic PF 2 function"sv, ""sv},
            ExtendedMenuItem{"096"sv, "Mic PF 3 function"sv, ""sv},
            ExtendedMenuItem{"097"sv, "Mic PF 4 function"sv, ""sv},
            ExtendedMenuItem{"098"sv, "Mic PF (DWN) function"sv, ""sv},
            ExtendedMenuItem{"099"sv, "Mic PF (UP) function"sv, ""sv},
        });
    } // namespace


    ExtendedCommandHandler::ExtendedCommandHandler()
        : BaseCommandHandler({"EX"}, "Extended Menu Commands") {
    }


    bool ExtendedCommandHandler::handleCommand(
        const RadioCommand &command,
        ISerialChannel &radioSerial,
        ISerialChannel &usbSerial,
        RadioManager &radioManager) {
        ESP_LOGV(TAG_EX, "Handling EX command: %s", command.command.c_str());

        std::string_view menuNumber;
        std::string_view value;

        if (isQuery(command)) {
            // EX[ddd]0000;  -> take the 3-digit menu number as a view
            std::string_view msg = command.originalMessage; // originalMessage is std::string
            if (msg.size() >= 5) {
                menuNumber = msg.substr(2, 3); // view into originalMessage
            } else {
                ESP_LOGW(TAG_EX, "EX READ command too short: %s", command.originalMessage.c_str());
                return false;
            }
        } else {
            // Provide an overload: bool parseEXCommand(const RadioCommand&, std::string_view&, std::string_view&);
            if (!parseEXCommand(command, menuNumber, value)) {
                ESP_LOGW(TAG_EX, "Failed to parse EX command");
                return false;
            }
        }

        // Provide an overload: const ExtendedMenuItem* findMenuItem(std::string_view);
        const ExtendedMenuItem *menuItem = findMenuItem(menuNumber);
        if (!menuItem) {
            ESP_LOGW(TAG_EX, "Unknown EX menu number: %.*s",
                     int(menuNumber.size()), menuNumber.data());
            return false;
        }

        auto &menuState = getExtendedMenuState();

        if (isQuery(command)) {
            if (command.isCatClient()) {
                // cacheKey = "EX" + menuNumber (no temporary allocations besides the key itself)
                std::string cacheKey;
                cacheKey.reserve(2 + menuNumber.size());
                cacheKey += "EX";
                cacheKey.append(menuNumber.data(), menuNumber.size());

                if (isCacheFresh(radioManager, cacheKey, TTL_STATIC_CONFIG)) {
                    std::string_view currentValue = menuState.getValue(menuNumber);
                    if (!currentValue.empty()) {
                        std::string response = formatEXResponse(menuNumber, currentValue);
                        respondToSource(command, response, usbSerial, radioManager);
                    } else {
                        const uint64_t nowUs = esp_timer_get_time();
                        radioManager.getState().queryTracker.recordQuery("EX", nowUs);
                        radioManager.noteQueryOrigin("EX", command.source, nowUs);
                        sendToRadio(radioSerial, command.originalMessage);
                    }
                } else {
                    std::string_view currentValue = menuState.getValue(menuNumber);
                    if (!currentValue.empty()) {
                        std::string response = formatEXResponse(menuNumber, currentValue);
                        respondToSource(command, response, usbSerial, radioManager);
                        const uint64_t nowUs = esp_timer_get_time();
                        radioManager.getState().queryTracker.recordQuery("EX", nowUs);
                        radioManager.noteQueryOrigin("EX", command.source, nowUs);
                        sendToRadio(radioSerial, command.originalMessage);
                    } else {
                        const uint64_t nowUs = esp_timer_get_time();
                        radioManager.getState().queryTracker.recordQuery("EX", nowUs);
                        radioManager.noteQueryOrigin("EX", command.source, nowUs);
                        sendToRadio(radioSerial, command.originalMessage);
                    }
                }
            } else {
                const uint64_t nowUs = esp_timer_get_time();
                radioManager.getState().queryTracker.recordQuery("EX", nowUs);
                radioManager.noteQueryOrigin("EX", command.source, nowUs);
                sendToRadio(radioSerial, command.originalMessage);
            }
            return true;
        }

        if (isSet(command)) {
            // EX read format (EX[menu]0000;) has params but empty value — the parser
            // classifies it as Set because it has params from a local source.
            // Treat empty-value Set as a read query so responses get forwarded.
            if (value.empty()) {
                if (command.isCatClient()) {
                    std::string cacheKey;
                    cacheKey.reserve(2 + menuNumber.size());
                    cacheKey += "EX";
                    cacheKey.append(menuNumber.data(), menuNumber.size());

                    std::string_view currentValue = menuState.getValue(menuNumber);
                    if (!currentValue.empty()) {
                        std::string response = formatEXResponse(menuNumber, currentValue);
                        respondToSource(command, response, usbSerial, radioManager);
                    }
                    const uint64_t nowUs = esp_timer_get_time();
                    radioManager.getState().queryTracker.recordQuery("EX", nowUs);
                    radioManager.noteQueryOrigin("EX", command.source, nowUs);
                    sendToRadio(radioSerial, command.originalMessage);
                } else if (shouldSendToRadio(command)) {
                    const uint64_t nowUs = esp_timer_get_time();
                    radioManager.getState().queryTracker.recordQuery("EX", nowUs);
                    radioManager.noteQueryOrigin("EX", command.source, nowUs);
                    sendToRadio(radioSerial, command.originalMessage);
                }
                return true;
            }

            if (!isValidParameterValue(*menuItem, value)) {
                ESP_LOGW(TAG_EX, "Invalid value '%.*s' for EX menu %.*s (%s)",
                         int(value.size()), value.data(),
                         int(menuNumber.size()), menuNumber.data(),
                         menuItem->function.data());
                return false;
            }

            menuState.setValue(menuNumber, value);

            // Dispatch helpers (you already updated these to take string_view)
            if (menuNumber == "056") handleEX056_UpdateTransverter(value, radioManager, /*isAnswer=*/false);
            else if (menuNumber == "059") handleEX059_UpdateHfLinear(value, radioManager, /*isAnswer=*/false);
            else if (menuNumber == "060") handleEX060_UpdateVhfLinear(value, radioManager, /*isAnswer=*/false);
            else if (menuNumber == "085") handleEX085_UpdateDrvConnector(value, radioManager, /*isAnswer=*/false);

            uint64_t now = esp_timer_get_time();
            std::string cacheKey;
            cacheKey.reserve(2 + menuNumber.size());
            cacheKey += "EX";
            cacheKey.append(menuNumber.data(), menuNumber.size());
            radioManager.getState().commandCache.update(cacheKey, now);

            ESP_LOGD(TAG_EX, "Set EX menu %.*s (%s) to '%.*s'",
                     int(menuNumber.size()), menuNumber.data(),
                     menuItem->function.data(),
                     int(value.size()), value.data());

            if (shouldSendToRadio(command)) {
                std::string cmdStr = formatEXResponse(menuNumber, value); // takes string_view
                sendToRadio(radioSerial, cmdStr);
            }
            return true;
        }

        if (command.type == CommandType::Answer) {
            menuState.setValue(menuNumber, value);

            if (menuNumber == "056") handleEX056_UpdateTransverter(value, radioManager, /*isAnswer=*/true);
            else if (menuNumber == "059") handleEX059_UpdateHfLinear(value, radioManager, /*isAnswer=*/true);
            else if (menuNumber == "060") handleEX060_UpdateVhfLinear(value, radioManager, /*isAnswer=*/true);
            else if (menuNumber == "085") handleEX085_UpdateDrvConnector(value, radioManager, /*isAnswer=*/true);

            uint64_t now = esp_timer_get_time();
            std::string cacheKey;
            cacheKey.reserve(2 + menuNumber.size());
            cacheKey += "EX";
            cacheKey.append(menuNumber.data(), menuNumber.size());
            radioManager.getState().commandCache.update(cacheKey, now);

            ESP_LOGD(TAG_EX, "Updated EX menu %.*s (%s) from radio: '%.*s'",
                     int(menuNumber.size()), menuNumber.data(),
                     menuItem->function.data(),
                     int(value.size()), value.data());

            {
                std::string response = formatEXResponse(menuNumber, value);
                routeAnswerResponse(command, response, usbSerial, radioManager);
            }
            return true;
        }

        return false;
    }

    ExtendedMenuState &ExtendedCommandHandler::getExtendedMenuState() {
        static ExtendedMenuState instance;
        return instance;
    }

    const ExtendedMenuItem *ExtendedCommandHandler::findMenuItem(std::string_view menuNumber) const {
        for (const auto &item: MENU_ITEMS) {
            if (item.menuNumber == menuNumber) {
                return &item;
            }
        }
        return nullptr;
    }


    bool ExtendedCommandHandler::isValidParameterValue(const ExtendedMenuItem &menuItem,
                                                       const std::string_view value) const {
        auto is_digits = [](std::string_view s) -> bool {
            if (s.empty()) return false;
            return std::ranges::all_of(s, [](unsigned char ch) { return std::isdigit(ch); });
        };

        auto parse_int = [](std::string_view s, int &out) -> bool {
            const char *first = s.data();
            const char *last = first + s.size();
            auto res = std::from_chars(first, last, out, 10);
            return res.ec == std::errc{} && res.ptr == last; // consumed all chars
        };

        const bool value_is_digits = is_digits(value);
        int value_num{};
        if (value_is_digits && !parse_int(value, value_num)) return false; // overflow or bad

        if (menuItem.allowedValues.empty()) return true;

        std::string_view allowed = menuItem.allowedValues;
        size_t start = 0;
        while (start <= allowed.size()) {
            size_t end = allowed.find('|', start);
            std::string_view token = (end == std::string_view::npos)
                                         ? allowed.substr(start)
                                         : allowed.substr(start, end - start);

            if (token == value) return true;

            if (value_is_digits && is_digits(token)) {
                int allowed_num{};
                if (parse_int(token, allowed_num) && allowed_num == value_num) return true;
            }

            if (end == std::string_view::npos) break;
            start = end + 1;
        }
        return false;
    }

    bool ExtendedCommandHandler::parseEXCommand(
        const RadioCommand &command,
        std::string_view &menuNumber,
        std::string_view &value) {
        // Parse directly from the original message to ensure string_view lifetime safety.
        // Expected formats:
        //  - EX[ddd]0000;             (query)
        //  - EX[ddd]0000[value];      (set/answer, value up to 8 chars)
        const std::string_view msg = command.originalMessage;
        if (msg.size() < 2 + 7 + 1) { // "EX" + "ddd0000" + ";"
            ESP_LOGW(TAG_EX, "EX command too short: %.*s", int(msg.size()), msg.data());
            return false;
        }

        // Verify prefix
        if (!msg.starts_with("EX")) {
            ESP_LOGW(TAG_EX, "EX command missing prefix: %.*s", int(msg.size()), msg.data());
            return false;
        }

        // Locate terminating semicolon
        const size_t semi = msg.rfind(';');
        if (semi == std::string_view::npos || semi < 2 + 7) {
            ESP_LOGW(TAG_EX, "EX command missing terminator: %.*s", int(msg.size()), msg.data());
            return false;
        }

        // Extract menu number and padding from fixed offsets
        menuNumber = msg.substr(2, 3);
        const std::string_view padding = msg.substr(5, 4);
        if (padding != "0000") {
            ESP_LOGW(TAG_EX, "EX command invalid padding: %.*s", int(padding.size()), padding.data());
            return false;
        }

        // Extract value if present (between index 9 and the semicolon)
        if (semi == 2 + 7) {
            value = std::string_view{}; // pure query-style payload
        } else {
            value = msg.substr(2 + 7, semi - (2 + 7));
            if (value.size() > 8) {
                ESP_LOGW(TAG_EX, "EX command value too long (max 8 chars): %.*s", int(value.size()), value.data());
                return false;
            }
        }

        ESP_LOGV(TAG_EX, "Parsed EX command: menu=%.*s, value=%.*s",
                 int(menuNumber.size()), menuNumber.data(),
                 int(value.size()), value.data());
        return true;
    }


    std::string ExtendedCommandHandler::formatEXResponse(const std::string_view menuNumber,
                                                         const std::string_view value) const {
        const std::string_view trunc = value.substr(0, std::min<size_t>(value.size(), 8));

        std::string out;
        out.reserve(2 + menuNumber.size() + 4 + (value.empty() ? 0 : trunc.size()) + 1);

        out += "EX";
        out += menuNumber;
        out += "0000";
        if (!value.empty()) out += trunc;
        out.push_back(';');

        return out;
    }

    void ExtendedCommandHandler::handleEX056_UpdateTransverter(
        const std::string_view value, RadioManager &radioManager, const bool isAnswer) const {
        // Parse decimal strictly: all chars consumed, detect overflow.
        int num{};
        bool parsed = false;
        if (!value.empty()) {
            const char *first = value.data();
            const char *last = first + value.size();
            auto res = std::from_chars(first, last, num, 10);
            parsed = (res.ec == std::errc{} && res.ptr == last);
        }

        if (!parsed) {
            ESP_LOGW(TAG_EX, "EX056: invalid value: %.*s", int(value.size()), value.data());
            return;
        }

        if (num == 1 || num == 2) {
            radioManager.getState().transverter.store(true, std::memory_order_relaxed);
            if (isAnswer)
                ESP_LOGI(TAG_EX, "Transverter enabled via EX056 answer (value: %.*s)", int(value.size()), value.data());
            else
                ESP_LOGD(TAG_EX, "Transverter enabled via EX056 (value: %.*s)", int(value.size()), value.data());
        } else if (num == 0) {
            radioManager.getState().transverter.store(false, std::memory_order_relaxed);
            if (isAnswer)
                ESP_LOGI(TAG_EX, "Transverter disabled via EX056 answer (value: %.*s)", int(value.size()),
                     value.data());
            else
                ESP_LOGD(TAG_EX, "Transverter disabled via EX056 (value: %.*s)", int(value.size()), value.data());
        } else {
            ESP_LOGW(TAG_EX, "EX056: unsupported value: %.*s", int(value.size()), value.data());
        }
    }

    void ExtendedCommandHandler::handleEX059_UpdateHfLinear(
        std::string_view value, RadioManager &radioManager, bool isAnswer) const {
        int num{};
        const char *first = value.data();
        const char *last = first + value.size();
        auto res = std::from_chars(first, last, num, 10);

        // reject junk (e.g. empty, non-digits, or trailing chars) and overflow
        if (res.ec != std::errc{} || res.ptr != last) {
            ESP_LOGW(TAG_EX, "EX059: invalid value: %.*s", int(value.size()), value.data());
            return;
        }

        // (optional) clamp/validate range if the command has limits, e.g.:
        // num = std::clamp(num, 0, 255);

        radioManager.getState().hfLinearAmpControl.store(num, std::memory_order_relaxed);

        if (isAnswer)
            ESP_LOGI(TAG_EX, "HF Linear Amp Control updated to %d from radio EX059", num);
        else
            ESP_LOGD(TAG_EX, "HF Linear Amp Control set to %d via EX059", num);
    }


    void ExtendedCommandHandler::handleEX060_UpdateVhfLinear(
        std::string_view value, RadioManager &radioManager, bool isAnswer) const {
        int num{};
        const char *first = value.data();
        const char *last = first + value.size();
        auto res = std::from_chars(first, last, num, 10);

        if (res.ec != std::errc{} || res.ptr != last) {
            ESP_LOGW(TAG_EX, "EX060: invalid value: %.*s", int(value.size()), value.data());
            return;
        }

        radioManager.getState().vhfLinearAmpControl.store(num, std::memory_order_relaxed);
        if (isAnswer)
            ESP_LOGI(TAG_EX, "VHF Linear Amp Control updated to %d from radio EX060", num);
        else
            ESP_LOGD(TAG_EX, "VHF Linear Amp Control set to %d via EX060", num);
    }

    void ExtendedCommandHandler::handleEX085_UpdateDrvConnector(
        const std::string_view value, RadioManager &radioManager, const bool isAnswer) const {
        auto is_digits = [](std::string_view s) {
            if (s.empty()) return false;
            return std::ranges::all_of(s, [](const unsigned char ch) { return std::isdigit(ch); });
        };

        int mode{};
        bool ok = false;
        if (is_digits(value)) {
            const char *first = value.data();
            const char *last = first + value.size();
            auto [ptr, ec] = std::from_chars(first, last, mode, 10);
            ok = (ec == std::errc{} && ptr == last);
        }

        if (!ok) {
            ESP_LOGW(TAG_EX, "EX085: invalid value: %.*s", int(value.size()), value.data());
            return;
        }

        if (mode == 0) {
            radioManager.getState().drvConnectorMode = 0;
            if (isAnswer)
                ESP_LOGI(TAG_EX, "DRV Connector updated to DRO mode from radio EX085 (value: %.*s)", int(value.size()),
                     value.data());
            else
                ESP_LOGD(TAG_EX, "DRV Connector set to DRO mode via EX085 (value: %.*s)", int(value.size()),
                     value.data());
        } else if (mode == 1) {
            radioManager.getState().drvConnectorMode = 1;
            if (isAnswer)
                ESP_LOGI(TAG_EX, "DRV Connector updated to ANT mode from radio EX085 (value: %.*s)", int(value.size()),
                     value.data());
            else
                ESP_LOGD(TAG_EX, "DRV Connector set to ANT mode via EX085 (value: %.*s)", int(value.size()),
                     value.data());
        } else {
            ESP_LOGW(TAG_EX, "EX085: unsupported mode (must be 0 or 1): %.*s", int(value.size()), value.data());
        }
    }
} // namespace radio
