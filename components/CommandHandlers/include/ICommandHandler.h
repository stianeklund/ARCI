#pragma once

#include "RadioCommand.h"
#include "ISerialChannel.h"
#include <string>
#include <vector>
#include <memory>

namespace radio {

class RadioManager; // Forward declaration

/**
 * @brief Interface for command handlers that process specific RadioCommand types
 */
class ICommandHandler {
public:
    virtual ~ICommandHandler() = default;

    /**
     * @brief Handle a radio command
     * @param command The RadioCommand to process
     * @param radioSerial Serial interface to the radio hardware
     * @param usbSerial Serial interface to the USB/host
     * @param radioManager Reference to the radio manager for state access
     * @return true if command was handled successfully, false otherwise
     */
    virtual bool handleCommand(const RadioCommand& command,
                               ISerialChannel& radioSerial,
                               ISerialChannel& usbSerial,
                               RadioManager& radioManager) = 0;

    /**
     * @brief Check if this handler can process the given command
     * @param command The command to check
     * @return true if this handler can process the command
     */
    virtual bool canHandle(const RadioCommand& command) const = 0;

    /**
     * @brief Get a description of what commands this handler processes
     * @return Human-readable description
     */
    virtual std::string_view getDescription() const = 0;

    /**
     * @brief Get the 2-character CAT command prefixes this handler supports
     * Default returns empty; BaseCommandHandler overrides to provide actual prefixes.
     */
    virtual std::vector<std::string_view> getPrefixes() const { return {}; }
};

// Note: CommandHandlerPtr is defined in CommandDispatcher.h to avoid circular dependency

} // namespace radio
