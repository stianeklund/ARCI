#include "MacroStorage.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

namespace storage {

MacroStorage& MacroStorage::instance() {
    static MacroStorage instance;
    return instance;
}

MacroStorage::~MacroStorage() {
    if (nvsHandle_ != 0) {
        nvs_close(nvsHandle_);
        nvsHandle_ = 0;
    }
}

esp_err_t MacroStorage::init() {
    if (initialized_) {
        return ESP_OK;
    }

    // Open persistent NVS handle
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &nvsHandle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", kNvsNamespace, esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG, "Opened NVS namespace '%s' (handle=%lu)", kNvsNamespace, nvsHandle_);

    // Mark as initialized so save/load can work during init
    initialized_ = true;

    // Check if data exists, otherwise load defaults
    if (!nvsHasMacroData()) {
        ESP_LOGI(TAG, "No macro data found in NVS - loading defaults");
        populateDefaults();
        err = persist();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save defaults: %s", esp_err_to_name(err));
            initialized_ = false;
            return err;
        }
    } else {
        // Load existing data into cache
        err = load(cache_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to load existing macros: %s", esp_err_to_name(err));
            populateDefaults();
        } else {
            ESP_LOGI(TAG, "Loaded %d existing macros from NVS", cache_.macroCount);
        }
    }

    return ESP_OK;
}

bool MacroStorage::nvsHasMacroData() const {
    if (nvsHandle_ == 0) return false;

    size_t requiredSize = 0;
    esp_err_t err = nvs_get_blob(nvsHandle_, "macro_data", nullptr, &requiredSize);

    ESP_LOGD(TAG, "NVS macro_data check: err=%s, size=%zu (expected=%zu)",
             esp_err_to_name(err), requiredSize, sizeof(MacroStorageData));

    return (err == ESP_OK && requiredSize > 0);
}

void MacroStorage::populateDefaults() {
    cache_ = MacroStorageData{};

    // Macro 01: "20M FT8" -> frequency 14.074 MHz, USB mode, DATA
    std::strncpy(cache_.macros[0].name, "20M FT8", kMacroNameMaxLength - 1);
    std::strncpy(cache_.macros[0].command, "FA00014074000|MD2|DA1", kMacroCommandMaxLength - 1);
    cache_.macros[0].enabled = true;

    // Macro 02: "10M FT8"
    std::strncpy(cache_.macros[1].name, "10M FT8", kMacroNameMaxLength - 1);
    std::strncpy(cache_.macros[1].command, "FA00028074000|MD2|DA1", kMacroCommandMaxLength - 1);
    cache_.macros[1].enabled = true;

    // Macro 03: "30M CW"
    std::strncpy(cache_.macros[2].name, "30M CW", kMacroNameMaxLength - 1);
    std::strncpy(cache_.macros[2].command, "FA00010105000|MD3", kMacroCommandMaxLength - 1);
    cache_.macros[2].enabled = true;

    // Macro 04: "CQ Call"
    std::strncpy(cache_.macros[3].name, "CQ Call", kMacroNameMaxLength - 1);
    std::strncpy(cache_.macros[3].command, "KY CQ CQ DE LB1TI LB1TI K  ", kMacroCommandMaxLength - 1);
    cache_.macros[3].enabled = true;

    // Macro 05: "Send Call"
    std::strncpy(cache_.macros[4].name, "Send Call", kMacroNameMaxLength - 1);
    std::strncpy(cache_.macros[4].command, "KY LB1TI                   ", kMacroCommandMaxLength - 1);
    cache_.macros[4].enabled = true;

    // F1-F5 assigned to macros 01-05, F6 empty
    cache_.slotAssignments[0] = 1;
    cache_.slotAssignments[1] = 2;
    cache_.slotAssignments[2] = 3;
    cache_.slotAssignments[3] = 4;
    cache_.slotAssignments[4] = 5;
    cache_.slotAssignments[5] = 0;

    cache_.macroCount = computeMacroCount();
    ESP_LOGI(TAG, "Loaded defaults: %d macros, F1-F5 assigned", cache_.macroCount);
}

uint8_t MacroStorage::computeMacroCount() const {
    uint8_t count = 0;
    for (const auto& macro : cache_.macros) {
        if (macro.enabled) {
            count++;
        }
    }
    return count;
}

esp_err_t MacroStorage::load(MacroStorageData& data) {
    if (!initialized_ || nvsHandle_ == 0) {
        ESP_LOGE(TAG, "MacroStorage not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Load entire structure as blob
    size_t requiredSize = sizeof(MacroStorageData);
    esp_err_t err = nvs_get_blob(nvsHandle_, "macro_data", &data, &requiredSize);

    if (err == ESP_OK && requiredSize == sizeof(MacroStorageData)) {
        data.macroCount = 0;
        for (const auto& macro : data.macros) {
            if (macro.enabled) data.macroCount++;
        }
        ESP_LOGD(TAG, "Loaded %d macros from NVS", data.macroCount);
        return ESP_OK;
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No macro data in NVS");
        return ESP_ERR_NOT_FOUND;
    } else if (err == ESP_OK && requiredSize != sizeof(MacroStorageData)) {
        ESP_LOGE(TAG, "Macro data size mismatch: expected %zu, got %zu",
                 sizeof(MacroStorageData), requiredSize);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "NVS read error: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
}

esp_err_t MacroStorage::save(const MacroStorageData& data) {
    if (!initialized_ || nvsHandle_ == 0) {
        ESP_LOGE(TAG, "MacroStorage not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Saving macro data to NVS (size=%zu bytes)...", sizeof(MacroStorageData));

    esp_err_t err = nvs_set_blob(nvsHandle_, "macro_data", &data, sizeof(MacroStorageData));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob FAILED: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_commit(nvsHandle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit FAILED: %s", esp_err_to_name(err));
        return err;
    }

    // Update cache
    cache_ = data;
    cache_.macroCount = computeMacroCount();

    ESP_LOGI(TAG, "Saved %d macros to NVS successfully", cache_.macroCount);
    return ESP_OK;
}

esp_err_t MacroStorage::persist() {
    return save(cache_);
}

esp_err_t MacroStorage::getMacro(uint8_t macroId, MacroDefinition& macro) const {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (macroId < 1 || macroId > kMacroCountMax) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t index = macroId - 1;
    if (cache_.macros[index].enabled) {
        macro = cache_.macros[index];
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t MacroStorage::setMacro(uint8_t macroId, const MacroDefinition& macro) {
    if (!initialized_) {
        ESP_LOGE(TAG, "setMacro: not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (macroId < 1 || macroId > kMacroCountMax) {
        ESP_LOGE(TAG, "setMacro: invalid ID %d", macroId);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Setting macro %d: name='%s', cmd='%.40s...'",
             macroId, macro.name, macro.command);

    const uint8_t index = macroId - 1;
    cache_.macros[index] = macro;
    cache_.macros[index].enabled = true;
    cache_.macroCount = computeMacroCount();

    return persist();
}

esp_err_t MacroStorage::deleteMacro(uint8_t macroId) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (macroId < 1 || macroId > kMacroCountMax) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t index = macroId - 1;
    cache_.macros[index].clear();

    // Remove from any slot assignments
    for (auto& slot : cache_.slotAssignments) {
        if (slot == macroId) {
            slot = 0;
        }
    }

    cache_.macroCount = computeMacroCount();
    return persist();
}

esp_err_t MacroStorage::getSlotAssignments(std::array<uint8_t, kMacroSlotCount>& assignments) const {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    assignments = cache_.slotAssignments;
    return ESP_OK;
}

esp_err_t MacroStorage::getSlotAssignments(uint8_t* assignments) const {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (assignments == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    std::memcpy(assignments, cache_.slotAssignments.data(), kMacroSlotCount);
    return ESP_OK;
}

esp_err_t MacroStorage::setSlotAssignment(uint8_t slot, uint8_t macroId) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (slot >= kMacroSlotCount || macroId > kMacroCountMax) {
        return ESP_ERR_INVALID_ARG;
    }

    cache_.slotAssignments[slot] = macroId;
    return persist();
}

uint8_t MacroStorage::getCount() const {
    if (!initialized_) {
        return 0;
    }
    return cache_.macroCount;
}

esp_err_t MacroStorage::factoryReset() {
    if (!initialized_ || nvsHandle_ == 0) {
        ESP_LOGE(TAG, "MacroStorage not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Erase all keys in the macro namespace
    esp_err_t err = nvs_erase_all(nvsHandle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase macro namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_commit(nvsHandle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit erase: %s", esp_err_to_name(err));
        return err;
    }

    // Reload defaults and save
    populateDefaults();
    err = persist();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save defaults after reset: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Factory reset complete - loaded %d default macros", cache_.macroCount);
    return ESP_OK;
}

}  // namespace storage
