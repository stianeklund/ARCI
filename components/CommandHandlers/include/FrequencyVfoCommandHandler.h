#pragma once

#include "BaseCommandHandler.h"
#include "FrequencyFormatter.h"

namespace radio {
    /**
     * @brief Consolidated handler for frequency, VFO, and RIT/XIT commands
     *
     * Combines functionality from FrequencyCommandHandler, VfoCommandHandler, and RitXitCommandHandler
     * into a single comprehensive handler for all frequency and VFO control operations.
     *
     * Handles commands:
     * - FA: VFO A frequency
     * - FB: VFO B frequency
     * - DN: Frequency/Channel down
     * - UP: Frequency/Channel up
     * - FR: Select VFO/Memory (simplex)
     * - FT: Select VFO (split)
     * - FS: Fine Tuning function
     * - TS: TF-Set (transfer frequency to VFO) - NEW
     * - SP: Split operation frequency settings
     * - RC: RIT/XIT clear
     * - RT: RIT on/off
     * - XT: XIT on/off
     * - RD: RIT/XIT frequency down
     * - RU: RIT/XIT frequency up
     * - RO: RIT/XIT offset (direct control) - NEW
     * - VV: VFO copy (A=B) - NEW
     * - XO: Transverter frequency offset - NEW
     * - EM: Emergency communication frequency mode - NEW
     * - CH: MULTI/CH encoder single step - NEW
     */
    class FrequencyVfoCommandHandler final : public BaseCommandHandler {
    public:
        FrequencyVfoCommandHandler();

        // ICommandHandler interface
        bool handleCommand(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) override;

    private:
        bool handleFA(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;
        bool handleFB(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;
        bool handleFR(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;
        bool handleFT(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;
        bool handleFS(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;
        bool handleSP(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;
        bool handleUP(const RadioCommand &cmd, ISerialChannel &radioSerial, RadioManager &rm) const;
        bool handleDN(const RadioCommand &cmd, ISerialChannel &radioSerial, RadioManager &rm) const;
        bool handleRT(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;
        bool handleXT(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;

        static bool handleRC(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm);
        bool handleRU(const RadioCommand &cmd, ISerialChannel &radioSerial, RadioManager &rm) const;
        bool handleRD(const RadioCommand &cmd, ISerialChannel &radioSerial, RadioManager &rm) const;
        bool handleRO(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;
        bool handleTS(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;

        static bool handleVV(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm);
        bool handleXO(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;

        static bool handleEM(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, const RadioManager &rm);
        bool handleCH(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial, RadioManager &rm) const;

        // Helper functions
        [[nodiscard]] static bool isValidFrequency(uint64_t frequency);

        [[nodiscard]] uint64_t parseFrequency(const RadioCommand &cmd, size_t index, uint64_t defaultValue = 0) const;
        [[nodiscard]] int parseVfoValue(const RadioCommand &cmd) const;
        [[nodiscard]] int parseOffsetValue(const RadioCommand &cmd) const;
        [[nodiscard]] static std::string formatOffsetResponse(int offset);
        [[nodiscard]] static uint32_t getStepSizeForMode(int mode);
        // NOTE: Transverter offset helpers moved to RadioManager (baseToDisplayFrequency/displayToBaseFrequency)

        // Constants
        static constexpr uint64_t MIN_FREQUENCY = 30000; // 30 kHz
        static constexpr uint64_t MAX_FREQUENCY = 2000000000; // 2 GHz (covers full TS-590SG range)

        // Frequency step sizes based on mode (Hz)
        static constexpr uint32_t STEP_SIZE_SSB = 2500; // 2.5 kHz
        static constexpr uint32_t STEP_SIZE_CW = 500; // 500 Hz
        static constexpr uint32_t STEP_SIZE_AM = 5000; // 5 kHz
        static constexpr uint32_t STEP_SIZE_FM = 5000; // 5 kHz
        static constexpr uint32_t STEP_SIZE_DEFAULT = 2500;

        // RIT/XIT limits (Hz)
        static constexpr int MIN_RIT_XIT_OFFSET = -9999;
        static constexpr int MAX_RIT_XIT_OFFSET = 9999;
        static constexpr int RIT_XIT_STEP = 10; // 10 Hz per step

        static constexpr const char *TAG = "FrequencyVfoCommandHandler";
    };
}
