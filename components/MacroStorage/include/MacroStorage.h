#pragma once

#include "esp_err.h"
#include <cstdint>
#include <cstring>

#ifdef __cplusplus
extern "C" {
#endif

// Storage configuration constants
#define MACRO_NAME_MAX_LENGTH 32
#define MACRO_COMMAND_MAX_LENGTH 64
#define MACRO_COUNT_MAX 50
#define MACRO_SLOT_COUNT 12  // F1-F6 buttons (short press: 0-5, long press: 6-11)
#define NVS_NAMESPACE_MACROS "arci_macros"

/**
 * @brief Macro definition with name and CAT command sequence
 */
typedef struct {
    char name[MACRO_NAME_MAX_LENGTH];           // Human-readable macro name (e.g., "20M FT8")
    char command[MACRO_COMMAND_MAX_LENGTH];     // CAT command sequence (semicolon-separated)
    bool enabled;                               // Whether this macro slot is active
} macro_definition_t;

/**
 * @brief Complete macro storage structure
 * Contains all macros and slot-to-macro bindings for F1-F6 buttons
 */
typedef struct {
    uint8_t slot_assignments[MACRO_SLOT_COUNT]; // F1-F6 macro IDs (0=empty, 1-50=macro index)
    macro_definition_t macros[MACRO_COUNT_MAX]; // All 50 possible macro slots
    uint8_t macro_count;                        // Count of defined (enabled) macros
} macro_storage_t;

/**
 * @brief Initialize macro storage - must be called once at startup
 * Opens persistent NVS handle for later operations.
 * If no stored data exists, loads defaults on first boot.
 *
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_INITIALIZED if NVS not ready, ESP_FAIL on other errors
 */
esp_err_t macro_storage_init(void);

/**
 * @brief Load all macros from NVS into memory
 *
 * @param storage Pointer to macro_storage_t to populate
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no data, ESP_FAIL on error
 */
esp_err_t macro_storage_load(macro_storage_t *storage);

/**
 * @brief Save all macros to NVS
 * Persists the entire macro storage structure to NVS.
 *
 * @param storage Pointer to macro_storage_t to save
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t macro_storage_save(const macro_storage_t *storage);

/**
 * @brief Get a single macro by ID
 *
 * @param macro_id ID of macro (1-50)
 * @param macro Pointer to macro_definition_t to populate
 * @return ESP_OK if found and enabled, ESP_ERR_NOT_FOUND if not found or disabled
 */
esp_err_t macro_storage_get_macro(uint8_t macro_id, macro_definition_t *macro);

/**
 * @brief Set/update a macro definition by ID
 *
 * @param macro_id ID of macro (1-50)
 * @param macro Pointer to macro_definition_t to store
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if ID out of range
 */
esp_err_t macro_storage_set_macro(uint8_t macro_id, const macro_definition_t *macro);

/**
 * @brief Delete a macro by ID (mark as disabled)
 *
 * @param macro_id ID of macro to delete (1-50)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if ID out of range
 */
esp_err_t macro_storage_delete_macro(uint8_t macro_id);

/**
 * @brief Get current F-button slot assignments
 *
 * @param assignments Pointer to uint8_t[MACRO_SLOT_COUNT] to populate
 * @return ESP_OK on success
 */
esp_err_t macro_storage_get_slot_assignments(uint8_t *assignments);

/**
 * @brief Set slot assignment for an F-button
 *
 * @param slot F-button slot (0-5 for F1-F6)
 * @param macro_id Macro ID to assign (0=empty, 1-50=macro)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if slot/ID out of range
 */
esp_err_t macro_storage_set_slot_assignment(uint8_t slot, uint8_t macro_id);

/**
 * @brief Load default macros (first boot initialization)
 * Populates storage with default macro set and button assignments.
 *
 * @param storage Pointer to macro_storage_t to populate with defaults
 * @return ESP_OK on success
 */
esp_err_t macro_storage_load_defaults(macro_storage_t *storage);

/**
 * @brief Query total macro count
 *
 * @return Number of enabled (active) macros
 */
uint8_t macro_storage_get_count(void);

/**
 * @brief Query maximum macro capacity
 *
 * @return Maximum number of macros (always MACRO_COUNT_MAX = 50)
 */
uint8_t macro_storage_get_max_count(void);

/**
 * @brief Erase all macro data and reload defaults
 * Use this to migrate from old protocol format or reset to factory defaults.
 *
 * @return ESP_OK on success
 */
esp_err_t macro_storage_factory_reset(void);

#ifdef __cplusplus
}
#endif
