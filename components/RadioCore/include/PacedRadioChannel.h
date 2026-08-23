#pragma once

#include "ISerialChannel.h"

namespace radio
{

    class RadioManager;

    /**
     * @brief ISerialChannel adapter that routes writes to the radio through
     *        RadioManager's paced TX queue instead of writing the UART directly.
     *
     * Command handlers (via BaseCommandHandler::sendToRadio) hold an
     * ISerialChannel& for "the radio" and call sendMessage() on it directly to
     * transmit a CAT command. Called that way, they bypass
     * RadioManager::sendRadioCommand()'s pacing queue (RADIO_TX_MIN_GAP_US),
     * which exists specifically because the TS-590SG rejects (?;) commands sent
     * faster than it can process them (commits f9ab2b6, f27b01b). This adapter
     * is a drop-in ISerialChannel: sendMessage() enqueues through
     * RadioManager::sendRadioCommand() instead of writing the wire directly,
     * while every other method (receive-side, failure counters, frame
     * callback) delegates unchanged to the real underlying channel, since only
     * the send-to-radio path needs pacing.
     */
    class PacedRadioChannel : public ISerialChannel
    {
    public:
        PacedRadioChannel(RadioManager &radioManager, ISerialChannel &realRadioSerial) :
            radioManager_(radioManager), realRadioSerial_(realRadioSerial)
        {
        }

        // Routed through RadioManager's paced TX queue instead of the UART directly.
        esp_err_t sendMessage(std::string_view message) override;

        // Receive-side and diagnostics are unaffected by pacing; delegate as-is.
        bool hasMessage() const override { return realRadioSerial_.hasMessage(); }
        std::pair<esp_err_t, std::string> getMessage() override { return realRadioSerial_.getMessage(); }

        std::pair<esp_err_t, std::string_view> getMessageView() override
        {
            return realRadioSerial_.getMessageView();
        }

        uint32_t getSendFailureCount() const override { return realRadioSerial_.getSendFailureCount(); }

        void setOnFrameCallback(std::function<void()> callback) override
        {
            realRadioSerial_.setOnFrameCallback(std::move(callback));
        }

    private:
        RadioManager &radioManager_;
        ISerialChannel &realRadioSerial_;
    };

} // namespace radio
