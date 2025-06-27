#pragma once

#include "BaseCommandHandler.h"

namespace radio
{

    /**
     * @brief Handler for UI-only meta commands between panel and display
     *
     * These commands are NEVER forwarded to the radio - they exist solely for
     * panel-display communication to support UI overlays like sliders.
     *
     * Commands handled:
     * - UIPC: Power Control UI (slider for PC command, 5-100W)
     * - UIML: Carrier/Monitor Level UI (slider for ML command, 0-20)
     * - UIRL: NR1 Level UI (slider for RL command, 1-10)
     * - UIRS: NR2 SPAC Speed UI (slider for RL command, 0-9)
     * - UINL: Noise Blanker Level UI (slider for NL command, 1-10)
     * - UIPI: Processor Input Level UI (slider for PL command, 0-100)
     * - UIPO: Processor Output Level UI (slider for PL command, 0-100)
     * - UINF: Notch Frequency UI (slider for BP command, 0-127)
     * - UIIS: IF Shift UI (slider for IS command, 0-9999 Hz)
     * - UIRI: RIT/XIT Offset UI (slider for RU/RD commands, -9999 to +9999 Hz)
     * - UIDA: Data Mode UI (toggle for DA command, 0=OFF, 1=ON)
     * - UIMN: Menu state (0=dismissed, 1=active)
     * - UIBL: Display backlight level (0-255)
     * - UIXD: Transverter display offset toggle (0=off, 1=on)
     * - UIPS: Panel status / display wake (0=idle/sleep, 1=active/awake)
     * - UIDE: Display communication enable/disable (0=disabled, 1=enabled)
     *
     * Command format: UIxxPPP; where xx is the control type and PPP is the value
     *
     * These commands are intercepted by the interface and used to:
     * 1. Update the display with a slider/menu overlay
     * 2. Track encoder adjustments in UI mode
     * 3. Confirm values and send actual CAT commands to radio
     */
    class UICommandHandler final : public BaseCommandHandler
    {
    public:
        UICommandHandler();

        // ICommandHandler interface
        bool handleCommand(const RadioCommand &cmd, ISerialChannel &radioSerial, ISerialChannel &usbSerial,
                           RadioManager &rm) override;

        // Static helper to format UI commands
        static std::string formatUIPC(int value);    // Power control: UIPC050;
        static std::string formatUIML(int value);    // Carrier level: UIML010;
        static std::string formatUIRL(int value);    // NR1 level: UIRL005;
        static std::string formatUIRS(int value);    // NR2 SPAC speed: UIRS005;
        static std::string formatUINL(int value);    // NB level: UINL005;
        static std::string formatUIPI(int value);    // Proc input: UIPI050;
        static std::string formatUIPO(int value);    // Proc output: UIPO050;
        static std::string formatUINF(int value);    // Notch freq: UINF064;
        static std::string formatUIIS(int value);    // IF shift: UIIS1234;
        static std::string formatUIDA(int value);    // Data mode: UIDA0; or UIDA1;
        static std::string formatUIRI(int value);    // RIT/XIT offset: UIRI+1234; or UIRI-1234;
        static std::string formatUIMN(bool active);  // Menu state: UIMN0; or UIMN1;
        static std::string formatUIBL(int value);    // Backlight level: UIBL128;
        static std::string formatUIXD(bool enabled); // Transverter display offset: UIXD0; or UIXD1;
        static std::string formatUIPS(bool active);  // Panel status: UIPS0; or UIPS1;
        static std::string formatUIDE(bool enabled); // Display communication: UIDE0; or UIDE1;

        static constexpr auto TAG = "UICommandHandler";

    private:
        // UI command handlers (never sent to radio)
        bool handleUIPC(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIML(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIRL(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIRS(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUINL(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIPI(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIPO(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUINF(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIIS(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIDA(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIRI(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIMN(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIBL(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIXD(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIPS(const RadioCommand &cmd, RadioManager &rm) const;
        bool handleUIDE(const RadioCommand &cmd, RadioManager &rm) const;

        // Helper to send UI command to display only
        static void sendToDisplayOnly(const std::string &cmd, RadioManager &rm);

        // Validation helpers
        static bool isValidPowerValue(int value);      // 5-100
        static bool isValidCarrierValue(int value);    // 0-20
        static bool isValidNrLevelValue(int value);    // 1-10 (NR1)
        static bool isValidNr2SpeedValue(int value);   // 0-9 (NR2 SPAC)
        static bool isValidNbLevelValue(int value);    // 1-10
        static bool isValidProcLevelValue(int value);  // 0-100 (Processor)
        static bool isValidNotchFreqValue(int value);  // 0-127 (Manual notch)
        static bool isValidIfShiftValue(int value);   // 0-9999 (IF shift Hz)
        static bool isValidDataModeValue(int value);   // 0-1 (Data mode toggle)
        static bool isValidRitXitValue(int value);    // -9999 to +9999 (RIT/XIT Hz)
        static bool isValidBacklightValue(int value);  // 0-255 (Display backlight)
    };

} // namespace radio
