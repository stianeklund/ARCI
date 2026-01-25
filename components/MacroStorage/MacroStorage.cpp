#include "MacroStorage.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "MacroStorage";

// Static NVS handle - persistent across calls
static nvs_handle_t g_nvs_handle = 0;
static bool initialized = false;
static macro_storage_t in_memory_cache = {};

// Helper: compute macro count from storage
static uint8_t compute_macro_count(const macro_storage_t *storage) {
    uint8_t count = 0;
    for (int i = 0; i < MACRO_COUNT_MAX; i++) {
        if (storage->macros[i].enabled) {
            count++;
        }
    }
    return count;
}

// Default macros for first boot
static void populate_defaults(macro_storage_t *storage) {
    std::memset(storage, 0, sizeof(macro_storage_t));

    // Macro 01: "20M FT8" -> frequency 14.074 MHz, USB mode, DATA
    // Commands separated by '|' (pipe), converted to ';' at execution
    std::strncpy(storage->macros[0].name, "20M FT8", MACRO_NAME_MAX_LENGTH - 1);
    std::strncpy(storage->macros[0].command, "FA00014074000|MD2|DA1", MACRO_COMMAND_MAX_LENGTH - 1);
    storage->macros[0].enabled = true;

    // Macro 02: "10M FT8"
    std::strncpy(storage->macros[1].name, "10M FT8", MACRO_NAME_MAX_LENGTH - 1);
    std::strncpy(storage->macros[1].command, "FA00028074000|MD2|DA1", MACRO_COMMAND_MAX_LENGTH - 1);
    storage->macros[1].enabled = true;

    // Macro 03: "30M CW"
    std::strncpy(storage->macros[2].name, "30M CW", MACRO_NAME_MAX_LENGTH - 1);
    std::strncpy(storage->macros[2].command, "FA00010105000|MD3", MACRO_COMMAND_MAX_LENGTH - 1);
    storage->macros[2].enabled = true;

    // Macro 04: "CQ Call" - single KY command (no separator needed)
    std::strncpy(storage->macros[3].name, "CQ Call", MACRO_NAME_MAX_LENGTH - 1);
    std::strncpy(storage->macros[3].command, "KY CQ CQ DE LB1TI LB1TI K  ", MACRO_COMMAND_MAX_LENGTH - 1);
    storage->macros[3].enabled = true;

    // Macro 05: "Send Call" - single KY command (no separator needed)
    std::strncpy(storage->macros[4].name, "Send Call", MACRO_NAME_MAX_LENGTH - 1);
    std::strncpy(storage->macros[4].command, "KY LB1TI                   ", MACRO_COMMAND_MAX_LENGTH - 1);
    storage->macros[4].enabled = true;

    // F1-F5 assigned to macros 01-05, F6 empty
    storage->slot_assignments[0] = 1;  // F1 -> Macro 01
    storage->slot_assignments[1] = 2;  // F2 -> Macro 02
    storage->slot_assignments[2] = 3;  // F3 -> Macro 03
    storage->slot_assignments[3] = 4;  // F4 -> Macro 04
    storage->slot_assignments[4] = 5;  // F5 -> Macro 05
    storage->slot_assignments[5] = 0;  // F6 -> Empty

    storage->macro_count = compute_macro_count(storage);
    ESP_LOGI(TAG, "Loaded defaults: %d macros, F1-F5 assigned", storage->macro_count);
}

// Check if NVS has existing macro data
static bool nvs_has_macro_data(void) {
    if (g_nvs_handle == 0) return false;

    size_t required_size = 0;
    esp_err_t err = nvs_get_blob(g_nvs_handle, "macro_data", nullptr, &required_size);

    ESP_LOGI(TAG, "NVS macro_data check: err=%s, size=%zu (expected=%zu)",
             esp_err_to_name(err), required_size, sizeof(macro_storage_t));

    return (err == ESP_OK && required_size > 0);
}

esp_err_t macro_storage_init(void) {
    if (initialized) {
        return ESP_OK;
    }

    // Open persistent NVS handle
    esp_err_t err = nvs_open(NVS_NAMESPACE_MACROS, NVS_READWRITE, &g_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", NVS_NAMESPACE_MACROS, esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG, "Opened NVS namespace '%s' (handle=%d)", NVS_NAMESPACE_MACROS, g_nvs_handle);

    // Mark as initialized immediately so save/load can work during init
    initialized = true;

    // Check if data exists, otherwise load defaults
    if (!nvs_has_macro_data()) {
        ESP_LOGI(TAG, "No macro data found in NVS - loading defaults");
        populate_defaults(&in_memory_cache);
        err = macro_storage_save(&in_memory_cache);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save defaults: %s", esp_err_to_name(err));
            initialized = false;
            return err;
        }
    } else {
        // Load existing data into cache
        err = macro_storage_load(&in_memory_cache);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to load existing macros: %s", esp_err_to_name(err));
            populate_defaults(&in_memory_cache);
        } else {
            ESP_LOGI(TAG, "Loaded %d existing macros from NVS", in_memory_cache.macro_count);
        }
    }

    return ESP_OK;
}

esp_err_t macro_storage_load(macro_storage_t *storage) {
    if (!initialized || g_nvs_handle == 0) {
        ESP_LOGE(TAG, "MacroStorage not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (storage == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    // Load entire structure as blob
    size_t required_size = sizeof(macro_storage_t);
    esp_err_t err = nvs_get_blob(g_nvs_handle, "macro_data", storage, &required_size);

    if (err == ESP_OK && required_size == sizeof(macro_storage_t)) {
        storage->macro_count = compute_macro_count(storage);
        ESP_LOGD(TAG, "Loaded %d macros from NVS", storage->macro_count);
        return ESP_OK;
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "No macro data in NVS");
        return ESP_ERR_NOT_FOUND;
    } else if (err == ESP_OK && required_size != sizeof(macro_storage_t)) {
        ESP_LOGE(TAG, "Macro data size mismatch: expected %zu, got %zu", sizeof(macro_storage_t), required_size);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "NVS read error: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
}

esp_err_t macro_storage_save(const macro_storage_t *storage) {
    if (!initialized || g_nvs_handle == 0) {
        ESP_LOGE(TAG, "MacroStorage not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (storage == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Saving macro data to NVS (size=%zu bytes)...", sizeof(macro_storage_t));

    esp_err_t err = nvs_set_blob(g_nvs_handle, "macro_data", storage, sizeof(macro_storage_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ nvs_set_blob FAILED: %s (size=%zu)", esp_err_to_name(err), sizeof(macro_storage_t));
        return err;
    }

    ESP_LOGI(TAG, "nvs_set_blob OK, committing...");

    // Commit changes to flash
    err = nvs_commit(g_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ nvs_commit FAILED: %s", esp_err_to_name(err));
        return err;
    }

    // Update in-memory cache
    std::memcpy(&in_memory_cache, storage, sizeof(macro_storage_t));
    in_memory_cache.macro_count = compute_macro_count(&in_memory_cache);

    ESP_LOGI(TAG, "✅ Saved %d macros to NVS successfully", in_memory_cache.macro_count);
    return ESP_OK;
}

esp_err_t macro_storage_get_macro(uint8_t macro_id, macro_definition_t *macro) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (macro == nullptr || macro_id < 1 || macro_id > MACRO_COUNT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    // Convert 1-based ID to 0-based array index
    uint8_t index = macro_id - 1;

    if (in_memory_cache.macros[index].enabled) {
        std::memcpy(macro, &in_memory_cache.macros[index], sizeof(macro_definition_t));
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t macro_storage_set_macro(uint8_t macro_id, const macro_definition_t *macro) {
    if (!initialized) {
        ESP_LOGE(TAG, "set_macro: not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (macro == nullptr || macro_id < 1 || macro_id > MACRO_COUNT_MAX) {
        ESP_LOGE(TAG, "set_macro: invalid arg (macro=%p, id=%d)", macro, macro_id);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Setting macro %d: name='%s', cmd='%.40s...'",
             macro_id, macro->name, macro->command);

    uint8_t index = macro_id - 1;
    std::memcpy(&in_memory_cache.macros[index], macro, sizeof(macro_definition_t));
    in_memory_cache.macros[index].enabled = true;

    // Recompute count and save
    in_memory_cache.macro_count = compute_macro_count(&in_memory_cache);

    esp_err_t err = macro_storage_save(&in_memory_cache);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to persist macro %d: %s", macro_id, esp_err_to_name(err));
    }
    return err;
}

esp_err_t macro_storage_delete_macro(uint8_t macro_id) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (macro_id < 1 || macro_id > MACRO_COUNT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t index = macro_id - 1;
    in_memory_cache.macros[index].enabled = false;
    std::memset(&in_memory_cache.macros[index].name, 0, MACRO_NAME_MAX_LENGTH);
    std::memset(&in_memory_cache.macros[index].command, 0, MACRO_COMMAND_MAX_LENGTH);

    // Also remove from any slot assignments
    for (int i = 0; i < MACRO_SLOT_COUNT; i++) {
        if (in_memory_cache.slot_assignments[i] == macro_id) {
            in_memory_cache.slot_assignments[i] = 0;
        }
    }

    in_memory_cache.macro_count = compute_macro_count(&in_memory_cache);
    return macro_storage_save(&in_memory_cache);
}

esp_err_t macro_storage_get_slot_assignments(uint8_t *assignments) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (assignments == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    std::memcpy(assignments, in_memory_cache.slot_assignments, MACRO_SLOT_COUNT);
    return ESP_OK;
}

esp_err_t macro_storage_set_slot_assignment(uint8_t slot, uint8_t macro_id) {
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (slot >= MACRO_SLOT_COUNT || macro_id > MACRO_COUNT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    in_memory_cache.slot_assignments[slot] = macro_id;
    return macro_storage_save(&in_memory_cache);
}

esp_err_t macro_storage_load_defaults(macro_storage_t *storage) {
    if (storage == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    populate_defaults(storage);
    return ESP_OK;
}

uint8_t macro_storage_get_count(void) {
    if (!initialized) {
        return 0;
    }
    return in_memory_cache.macro_count;
}

uint8_t macro_storage_get_max_count(void) {
    return MACRO_COUNT_MAX;
}

esp_err_t macro_storage_factory_reset(void) {
    if (!initialized || g_nvs_handle == 0) {
        ESP_LOGE(TAG, "MacroStorage not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Erase all keys in the macro namespace
    esp_err_t err = nvs_erase_all(g_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase macro namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_commit(g_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit erase: %s", esp_err_to_name(err));
        return err;
    }

    // Reload defaults and save
    populate_defaults(&in_memory_cache);
    err = macro_storage_save(&in_memory_cache);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save defaults after reset: %s", esp_err_to_name(err));
        return err;
    }

    // Log first macro to verify pipe separators
    ESP_LOGI(TAG, "Factory reset complete - loaded %d default macros", in_memory_cache.macro_count);
    ESP_LOGI(TAG, "Macro 1 command: '%s'", in_memory_cache.macros[0].command);
    return ESP_OK;
}
