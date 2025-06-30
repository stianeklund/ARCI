#pragma once

#include "BaseCommandHandler.h"
#include "ISerialChannel.h"

namespace radio {
    struct RadioState;

    // Forward declarations
    class RadioManager;
    /**
     * @brief Handler for antenna and tuner control commands
     *
     * Handles antenna-related commands based on ts590sg_cat_commands_v3.json:
     * - AC: Internal antenna tuner status (RX/TX AT, tuning control)
     * - AN: Antenna selection (main antenna, RX antenna, drive output)
     *
     * Note: RA (Attenuator) and PA (Preamp) commands have been moved to
     * ReceiverProcessingCommandHandler as they are part of the receiver
     * signal processing chain.
     */
    class AntennaCommandHandler final : public BaseCommandHandler {
    public:
        AntennaCommandHandler() : BaseCommandHandler({"AC", "AN"}, "Antenna Control") {
        }

        ~AntennaCommandHandler() override = default;

        bool handleCommand(const RadioCommand &command,
                           ISerialChannel &radioSerial,
                           ISerialChannel &usbSerial,
                           RadioManager &radioManager) override;

    private:
        static constexpr const char *TAG = "AntennaCommandHandler";

        bool handleAC(const RadioCommand &command,
                      ISerialChannel &radioSerial,
                      ISerialChannel &usbSerial,
                      RadioManager &radioManager);

        bool handleAN(const RadioCommand &command,
                      ISerialChannel &radioSerial,
                      ISerialChannel &usbSerial,
                      RadioManager &radioManager);

        // AC command parsing (P1P2P3 format)
        struct ACParams {
            int rxAt; // P1: RX AT (0=THRU, 1=IN)
            int txAt; // P2: TX AT (0=THRU, 1=IN)
            int tuning; // P3: Tuning (0=Stop, 1=Start)
            bool valid;
        };

        // AN command parsing (P1P2P3 format)
        struct ANParams {
            int mainAnt; // P1: Main antenna (1=ANT1, 2=ANT2, 9=no change)
            int rxAnt; // P2: RX antenna (0=OFF, 1=ON, 9=no change)
            int drvOut; // P3: Drive out (0=OFF, 1=ON, 9=no change)
            bool valid;
        };

        [[nodiscard]] ACParams parseACParams(const RadioCommand &command) const;
        [[nodiscard]] ANParams parseANParams(const RadioCommand &command) const;
        [[nodiscard]] std::string formatACResponse(const RadioState &state) const;
        [[nodiscard]] std::string formatANResponse(const RadioState &state) const;
    };
} // namespace radio