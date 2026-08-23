#include "PacedRadioChannel.h"
#include "RadioManager.h"

namespace radio
{

    esp_err_t PacedRadioChannel::sendMessage(std::string_view message)
    {
        // sendRadioCommand validates the command and either enqueues it on the
        // paced TX queue (production) or sends it inline synchronously to the
        // same underlying mock (CONFIG_RUN_UNIT_TESTS builds); drop accounting
        // on a full queue is already handled inside sendRadioCommand. There is
        // no queue-specific failure to surface to the caller here, so this
        // always returns ESP_OK -- matching sendRadioCommand's own
        // fire-and-forget (void) contract.
        radioManager_.sendRadioCommand(message);
        return ESP_OK;
    }

} // namespace radio
