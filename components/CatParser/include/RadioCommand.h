#pragma once

#include <string>
#include <string_view>
#include <variant>
#include <vector>
#include <array>
#include <cstdint>

namespace radio {

// Defines the source of a CAT command
enum class CommandSource : uint8_t {
    UsbCdc0,  // Command from USB CDC Interface 0
    UsbCdc1,  // Command from USB CDC Interface 1
    Tcp0,     // Command from TCP port 0 (network CAT control)
    Tcp1,     // Command from TCP port 1 (network CAT control)
    Display,  // Command from display interface (external display module)
    Remote,   // Response/command from radio hardware
    Panel,    // Command from on-device control surfaces (e.g., TCA8418 keypad)
    Macro     // Command from internal macro system (responses forwarded via AI policy only)
};

// Defines the type of a CAT command
enum class CommandType : uint8_t {
    Set,     // Command to set a value (e.g., "FA00014150000;")
    Read,    // Query command (e.g., "FA;")
    Answer,  // Response with data (e.g., "FA00014150000;" from radio)
    Unknown  // Unknown/invalid command type (added last to preserve existing values)
};

// Small String Optimization (SSO) parameter storage
// Most CAT commands have ≤3 parameters, avoid heap allocations
struct ParamValue {
    static constexpr size_t SSO_CAPACITY = 16;  // Inline storage for short strings

    enum class Type : uint8_t { Empty, Int, String };

    Type type = Type::Empty;
    union {
        int intValue;
        struct {
            char data[SSO_CAPACITY];
            uint8_t length;
        } ssoString;
    };

    ParamValue() : type(Type::Empty), intValue(0) {}

    explicit ParamValue(int val) : type(Type::Int), intValue(val) {}

    explicit ParamValue(std::string_view str) : type(Type::String) {
        if (str.size() < SSO_CAPACITY) {
            ssoString.length = static_cast<uint8_t>(str.size());
            std::copy(str.begin(), str.end(), ssoString.data);
            ssoString.data[ssoString.length] = '\0';
        } else {
            // Fallback: truncate to SSO capacity
            ssoString.length = SSO_CAPACITY - 1;
            std::copy(str.begin(), str.begin() + ssoString.length, ssoString.data);
            ssoString.data[ssoString.length] = '\0';
        }
    }

    [[nodiscard]] bool isInt() const { return type == Type::Int; }
    [[nodiscard]] bool isString() const { return type == Type::String; }
    [[nodiscard]] bool isEmpty() const { return type == Type::Empty; }

    [[nodiscard]] int asInt() const { return intValue; }
    [[nodiscard]] std::string_view asStringView() const {
        return type == Type::String ? std::string_view(ssoString.data, ssoString.length) : std::string_view();
    }
    [[nodiscard]] std::string asString() const {
        return type == Type::String ? std::string(ssoString.data, ssoString.length) : std::string();
    }
};

// Represents a generic CAT command with source context
// Optimized for minimal heap allocations and cache-friendly layout
struct RadioCommand {
    // Fixed-size inline storage for most common cases (≤4 params)
    static constexpr size_t INLINE_PARAM_CAPACITY = 4;
    std::array<ParamValue, INLINE_PARAM_CAPACITY> inlineParams{};
    uint8_t paramCount = 0;

    // Fallback for rare commands with >4 params (heap allocation only when needed)
    std::vector<std::variant<int, std::string>> overflowParams;

    std::string command;        // Command prefix (e.g., "FA", "FB", "MD") - uses SSO internally
    CommandType type;           // Set, Read, or Answer
    CommandSource source;       // Usb (from host) or Remote (from radio)
    std::string originalMessage; // Original message as received (e.g., "FA;" or "FA00014150000;")
    bool bypassCache = false;   // Flag to bypass TTL caching for button-initiated commands

    // Legacy params field for backward compatibility - publicly accessible
    // NOTE: Prefer addParam()/getParamFast() in new code to avoid heap allocations
    // addParam() populates both inlineParams and params for backward compatibility
    std::vector<std::variant<int, std::string>> params{};

public:

    // Fast accessors without heap allocation
    [[nodiscard]] size_t getParamCount() const {
        return static_cast<size_t>(paramCount) + overflowParams.size();
    }

    [[nodiscard]] bool hasParams() const {
        return paramCount > 0 || !overflowParams.empty();
    }

    // Zero-allocation parameter access
    [[nodiscard]] const ParamValue* getParamFast(size_t index) const {
        if (index < paramCount && index < INLINE_PARAM_CAPACITY) {
            return &inlineParams[index];
        }
        return nullptr;  // Caller should check overflow vector separately
    }

    void addParam(int value) {
        if (paramCount < INLINE_PARAM_CAPACITY) {
            inlineParams[paramCount++] = ParamValue(value);
        } else {
            overflowParams.emplace_back(value);
        }
        // Also populate legacy params vector for backward compatibility
        params.emplace_back(value);
    }

    void addParam(std::string_view value) {
        if (paramCount < INLINE_PARAM_CAPACITY) {
            inlineParams[paramCount++] = ParamValue(value);
        } else {
            overflowParams.emplace_back(std::string(value));
        }
        // Also populate legacy params vector for backward compatibility
        params.emplace_back(std::string(value));
    }

    void clearParams() {
        paramCount = 0;
        overflowParams.clear();
        params.clear();
    }

    // Constructors
    RadioCommand() = default;

    RadioCommand(const std::string& cmd, const CommandType t, const CommandSource src)
        : command(cmd), type(t), source(src) {}

    RadioCommand(std::string_view cmd, const CommandType t, const CommandSource src)
        : command(cmd), type(t), source(src) {}

    RadioCommand(const std::string& cmd, const CommandType t, const CommandSource src, const std::string& original)
        : command(cmd), type(t), source(src), originalMessage(original) {}

    RadioCommand(std::string_view cmd, const CommandType t, const CommandSource src, std::string_view original)
        : command(cmd), type(t), source(src), originalMessage(original) {}

    RadioCommand(const std::string& cmd, const CommandType t, const CommandSource src, const std::string& original, bool bypass)
        : command(cmd), type(t), source(src), originalMessage(original), bypassCache(bypass) {}

    RadioCommand(std::string_view cmd, const CommandType t, const CommandSource src, std::string_view original, bool bypass)
        : command(cmd), type(t), source(src), originalMessage(original), bypassCache(bypass) {}

    /**
     * @brief Check if a source is a USB interface (static helper)
     */
    [[nodiscard]] static constexpr bool isUsbSource(CommandSource src) {
        return src == CommandSource::UsbCdc0 || src == CommandSource::UsbCdc1;
    }

    /**
     * @brief Check if a source is a TCP interface (static helper)
     */
    [[nodiscard]] static constexpr bool isTcpSource(CommandSource src) {
        return src == CommandSource::Tcp0 || src == CommandSource::Tcp1;
    }

    /**
     * @brief Check if a source is an external CAT client (USB or TCP) - static helper
     * These are the interfaces that behave identically as remote control clients,
     * distinct from Panel (physical controls), Display, and Macro (internal).
     */
    [[nodiscard]] static constexpr bool isCatClientSource(CommandSource src) {
        return isUsbSource(src) || isTcpSource(src);
    }

    /**
     * @brief Check if a source is local (USB, TCP, Display, Panel, or Macro) - static helper
     * Use this when you have a CommandSource but not a full RadioCommand
     */
    [[nodiscard]] static constexpr bool isLocalSource(CommandSource src) {
        return isUsbSource(src) || isTcpSource(src) ||
               src == CommandSource::Display || src == CommandSource::Panel || src == CommandSource::Macro;
    }

    /**
     * @brief Check if this command is from a USB interface (either CDC0 or CDC1)
     */
    [[nodiscard]] bool isUsb() const { return isUsbSource(source); }

    /**
     * @brief Check if this command is from a TCP interface (either Tcp0 or Tcp1)
     */
    [[nodiscard]] bool isTcp() const { return isTcpSource(source); }

    /**
     * @brief Check if this command is from an external CAT client (USB or TCP)
     * Use this for behavior that should apply to all remote control interfaces
     * but not to Panel, Display, or Macro sources.
     */
    [[nodiscard]] bool isCatClient() const { return isCatClientSource(source); }

    /**
     * @brief Check if this command is from a local interface (USB, TCP, Display, or Panel)
     */
    [[nodiscard]] bool isLocal() const { return isLocalSource(source); }

    /**
     * @brief Check if this command should be sent to radio
     * Local Set commands should be sent to radio
     * Local Query commands should be sent to radio (for real-time data)
     * Remote Answer commands should NOT be sent to radio (they're FROM radio)
     * Note: Individual handlers may override this for cached responses
     */
    bool shouldSendToRadio() const {
        return isLocal() && type != CommandType::Answer;
    }
    
    /**
     * @brief Get short name for a CommandSource
     */
    [[nodiscard]] static constexpr std::string_view sourceName(CommandSource src) {
        switch (src) {
            case CommandSource::UsbCdc0: return "UsbCdc0";
            case CommandSource::UsbCdc1: return "UsbCdc1";
            case CommandSource::Tcp0:    return "Tcp0";
            case CommandSource::Tcp1:    return "Tcp1";
            case CommandSource::Display: return "Display";
            case CommandSource::Panel:   return "Panel";
            case CommandSource::Remote:  return "Remote";
            case CommandSource::Macro:   return "Macro";
        }
        return "Unknown";
    }

    /**
     * @brief Get a description of this command for debugging
     */
    [[nodiscard]] std::string describe() const {
        std::string desc;
        switch (source) {
            case CommandSource::UsbCdc0: desc = "UsbCdc0 "; break;
            case CommandSource::UsbCdc1: desc = "UsbCdc1 "; break;
            case CommandSource::Tcp0: desc = "Tcp0 "; break;
            case CommandSource::Tcp1: desc = "Tcp1 "; break;
            case CommandSource::Display: desc = "Display "; break;
            case CommandSource::Panel: desc = "Panel "; break;
            case CommandSource::Remote: desc = "Remote "; break;
            case CommandSource::Macro: desc = "Macro "; break;
        }
        desc += (type == CommandType::Set ? "Set" :
                type == CommandType::Read ? "Read" : "Answer");
        desc += " " + command;
        if (!params.empty()) {
            desc += "(";
            for (size_t i = 0; i < params.size(); ++i) {
                if (i > 0) desc += ",";
                if (std::holds_alternative<int>(params[i])) {
                    desc += std::to_string(std::get<int>(params[i]));
                } else {
                    desc += std::get<std::string>(params[i]);
                }
            }
            desc += ")";
        }
        return desc;
    }
};

}
