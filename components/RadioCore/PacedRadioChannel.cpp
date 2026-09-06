#include "PacedRadioChannel.h"
#include "RadioManager.h"

namespace radio
{

    esp_err_t PacedRadioChannel::sendMessage(std::string_view message)
    {
        // sendRadioCommand validates the command and either enqueues it on the
        // paced TX queue (production) or sends it inline synchronously to the
        // same underlying mock (CONFIG_RUN_UNIT_TESTS builds). It returns false
        // when validation rejected the command or the paced queue was full
        // (drop accounting is handled inside sendRadioCommand either way); we
        // surface that as ESP_ERR_NO_MEM so callers going through this channel
        // (e.g. BaseCommandHandler::sendToRadio) can tell delivery apart from
        // success instead of always seeing ESP_OK.
        return radioManager_.sendRadioCommand(message) ? ESP_OK : ESP_ERR_NO_MEM;
    }

} // namespace radio
