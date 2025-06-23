#pragma once

#include "BaseCommandHandler.h"
#include "RadioState.h"
#include <string>
#include <string_view>
#include <unordered_map>

namespace radio {
    struct ExtendedMenuItem {
        std::string_view menuNumber;
        std::string_view function;
        std::string_view allowedValues; // '|' separated list; empty means unconstrained
    };


    struct sv_hash {
        using is_transparent = void;
        using hash_type = std::hash<std::string_view>;
        size_t operator()(const std::string_view s) const noexcept { return hash_type{}(s); }
        size_t operator()(const std::string &s) const noexcept { return hash_type{}(s); }
        size_t operator()(const char *s) const noexcept { return hash_type{}(s); }
    };

    /**
 * @brief Separate state for EX menu parameters to avoid bloating RadioState
 *
 * Stores the current values of the TS-590SG's EX menu items. These are relatively
 * static configuration values that don't change frequently during operation.
 */

    struct ExtendedMenuState {
        // Enable heterogeneous lookup with transparent hash + equal
        std::unordered_map<std::string, std::string, sv_hash, std::equal_to<> > menuValues{
            {"000", "3"}, // Display brightness mid-level
            {"002", "3"},
            {"003", "1"},
            {"004", "1"},
            {"013", "0"},
            {"014", "0"},
            {"020", "0"},
            {"056", "0"}, // Transverter - default off
            {"067", "3"}, // COM speed - 38400 (index 3)
            {"068", "3"}, // USB speed - 38400 (index 3)
        };

        // Return a view to avoid copying (be aware it is invalidated if the map mutates)
        std::string_view getValue(const std::string_view menuNumber) const {
            if (const auto it = menuValues.find(menuNumber); it != menuValues.end())
                return it->second;
            return {}; // empty view if not set
        }

        // Construct in-place; no temporary std::string for the value assignment path
        void setValue(const std::string_view menuNumber, std::string_view value) {
            // try_emplace builds the key/value from the views; on existing key, just assign
            auto [it, inserted] = menuValues.try_emplace(std::string(menuNumber), value);
            if (!inserted) it->second.assign(value.data(), value.size());
        }

        void clearCache() { menuValues.clear(); }
    };

    class ExtendedCommandHandler final : public BaseCommandHandler {
    public:
        ExtendedCommandHandler();

        bool handleCommand(const RadioCommand &command,
                           ISerialChannel &radioSerial,
                           ISerialChannel &usbSerial,
                           RadioManager &radioManager) override;

        /**
         * @brief Get shared extended menu state instance
         * @return Reference to the singleton ExtendedMenuState
         */
        static ExtendedMenuState &getExtendedMenuState();

    private:
        /**
         * @brief Find menu item by menu number
         * @param menuNumber Menu number as string (000-099)
         * @return Pointer to menu item, or nullptr if not found
         */
        [[nodiscard]] const ExtendedMenuItem *findMenuItem(std::string_view menuNumber) const;

        /**
         * @brief Validate parameter value for a menu item
         * @param menuItem The menu item to validate against
         * @param value The value to validate
         * @return true if value is valid for this menu item
         */
        [[nodiscard]] bool isValidParameterValue(const ExtendedMenuItem &menuItem, std::string_view value) const;

        /**
         * @brief Parse EX command parameters
         * @param command The EX command to parse
         * @param menuNumber Out parameter for menu number (000-099)
         * @param value Out parameter for value (for Set commands)
         * @return true if parsing succeeded
         */
        static bool parseEXCommand(const RadioCommand &command, std::string_view &menuNumber, std::string_view &value);

        /**
         * @brief Format EX response
         * @param menuNumber Menu number (000-099)
         * @param value Menu value
         * @return Formatted EX response string
         */
        [[nodiscard]] std::string formatEXResponse(std::string_view menuNumber, std::string_view value) const;

    public:
        /**
         * @brief Clear the extended menu state cache
         * Used primarily for testing to ensure clean state between tests
         */
        static void clearMenuCache() {
            getExtendedMenuState().clearCache();
        }

    private:
        // Transverter-related helpers (extracted for clarity and reuse)
        void handleEX056_UpdateTransverter(std::string_view value, RadioManager &radioManager, bool isAnswer) const;

        void handleEX059_UpdateHfLinear(std::string_view value, RadioManager &radioManager, bool isAnswer) const;

        void handleEX060_UpdateVhfLinear(std::string_view value, RadioManager &radioManager, bool isAnswer) const;

        void handleEX085_UpdateDrvConnector(std::string_view value, RadioManager &radioManager, bool isAnswer) const;
    };

    static constexpr const char *TAG_EX = "ExtendedCommandHandler";
} // namespace radio
