#pragma once

#include "BaseCommandHandler.h"
#include "ISerialChannel.h"

namespace radio {
    // Forward declarations
    class RadioManager;

    /**
     * @brief Handler for user-definable macro commands (MX protocol)
     *
     * Implements the MX protocol for querying, defining, and executing
     * user-defined CAT command macros. Macros are stored persistently in panel NVS
     * and can be assigned to F1-F6 buttons.
     *
     * Protocol:
     * - MXW<id>,<name>,<cmd>|<cmd>|...;   Write macro (id=01-50)
     * - MXR<id>;                          Read macro query
     * - MXR<id>,<name>,<cmd>|<cmd>|...;   Read response
     * - MXA;                              Get all F-key assignments
     * - MXA<s1>,<s2>,...,<s6>;            Assignments response (00=empty)
     * - MXA<slot>,<id>;                   Assign slot (1-6) to macro
     * - MXE<slot>;                        Execute slot
     * - MXD<id>;                          Delete macro
     *
     * Key features:
     * - Standard 2-char "MX" prefix (no parser workarounds needed)
     * - Pipe '|' separates commands internally (converted to ';' at execution)
     * - Standard ';' terminator throughout
     */
    class MXCommandHandler final : public BaseCommandHandler {
    public:
        MXCommandHandler() : BaseCommandHandler({"MX"}, "User-Definable Macros") {}

        ~MXCommandHandler() override = default;

        bool handleCommand(const RadioCommand &command,
                          ISerialChannel &radioSerial,
                          ISerialChannel &usbSerial,
                          RadioManager &radioManager) override;

    private:
        static constexpr const char *TAG = "MXCommandHandler";

        // Protocol handlers
        bool handleMXW(const RadioCommand &command,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager);

        bool handleMXR(const RadioCommand &command,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager);

        bool handleMXA(const RadioCommand &command,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager);

        bool handleMXE(const RadioCommand &command,
                       ISerialChannel &radioSerial,
                       RadioManager &radioManager);

        bool handleMXD(const RadioCommand &command,
                       ISerialChannel &usbSerial,
                       RadioManager &radioManager);

        // Helper: Build MXR response for a single macro
        // Format: MXR<ID>,<name>,<commands>;
        static std::string buildMXRResponse(uint8_t macro_id);

        // Helper: Parse macro ID from command (format: 2-digit zero-padded 01-50)
        static uint8_t parseMacroId(std::string_view param);
    };

} // namespace radio
