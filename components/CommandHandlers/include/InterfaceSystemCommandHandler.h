#pragma once

#include "BaseCommandHandler.h"

namespace radio
{

    /**
     * @brief Handler for interface and system control commands
     *
     * These commands manage the CAT interface itself, transceiver power state,
     * and other system-level functions like panel lock and resets.
     *
     * Commands handled:
     * - AI: Auto Information (AI) function
     * - PS: Power control
     * - SR: Reset the transceiver
     * - LK: Panel lock
     * - TC: Terminal Control (indicates a program like ARCP is connected)
     */
    class InterfaceSystemCommandHandler final : public BaseCommandHandler
    {
    public:
        InterfaceSystemCommandHandler();

        // ICommandHandler interface
        bool handleCommand(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                           RadioManager &rm) override;

        // Public AI coordination helper for external monitoring
        static void logAiCoordinationSnapshot(const RadioState &state);

        // Public logging tag for use by helper functions
        static constexpr auto TAG = "InterfaceSystemCommandHandler";

    private:
        bool handleAI(const RadioCommand &command, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                      RadioManager &radioManager) const;
        bool handlePS(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                      RadioManager &rm) const;

        static bool handleSR(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                             RadioManager &rm);
        bool handleLK(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                      RadioManager &rm) const;
        bool handleTC(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                      const RadioManager &rm) const;

        // Helper functions
        static bool isValidAIMode(int mode);

        static bool isValidPowerState(int state);

        static bool isValidLockState(int state);

        static std::string formatAIResponse(int mode);

        static std::string formatPSResponse(int state);

        // AI coordination helper methods
        static int calculateRadioAIMode(const RadioState &state);
        void updateClientAIMode(CommandSource source, int mode, ISerialChannel &radioSerial,
                                RadioManager &radioManager) const;

        static std::string formatLKResponse(int state);

        static std::string formatTCResponse(int state);

        // Constants - Per TS-590SG specification
        static constexpr int AI_OFF = 0; // AI off
        static constexpr int AI_ON = 2; // AI on (no backup)
        static constexpr int AI_BACKUP_ON = 4; // AI on (with backup)
        static constexpr int PS_OFF = 0; // Power off
        static constexpr int PS_ON = 1; // Power on
        static constexpr int LK_OFF = 0; // Panel unlocked
        static constexpr int LK_ON = 1; // Panel locked
        static constexpr int TC_OFF = 0; // Terminal control off
        static constexpr int TC_ON = 1; // Terminal control on
    };

} // namespace radio
