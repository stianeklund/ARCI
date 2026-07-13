#pragma once

#include "esp_err.h"
#include "rtos_mutex.h"
#include <array>
#include <cstdint>
#include <cstring>

namespace storage {

// Storage configuration constants
constexpr size_t kMacroNameMaxLength = 32;
constexpr size_t kMacroCommandMaxLength = 64;
constexpr size_t kMacroCountMax = 50;
constexpr size_t kMacroSlotCount = 12;  // F1-F6 buttons (short: 0-5, long: 6-11)
constexpr const char* kNvsNamespace = "arci_macros";

/**
 * @brief Macro definition with name and CAT command sequence
 */
struct MacroDefinition {
    char name[kMacroNameMaxLength]{};
    char command[kMacroCommandMaxLength]{};
    bool enabled = false;

    void clear() {
        std::memset(name, 0, sizeof(name));
        std::memset(command, 0, sizeof(command));
        enabled = false;
    }
};

/**
 * @brief Complete macro storage data structure
 */
struct MacroStorageData {
    std::array<uint8_t, kMacroSlotCount> slotAssignments{};
    std::array<MacroDefinition, kMacroCountMax> macros{};
    uint8_t macroCount = 0;
};

/**
 * @brief Persistent storage for user-defined CAT command macros
 *
 * Singleton class managing macro definitions and F-button slot assignments.
 * Uses ESP-IDF NVS for persistence with an in-memory cache for fast access.
 *
 * Usage:
 *   MacroStorage::instance().init();
 *   MacroDefinition macro;
 *   MacroStorage::instance().getMacro(1, macro);
 */
class MacroStorage {
public:
    /**
     * @brief Get singleton instance
     */
    static MacroStorage& instance();

    // Non-copyable
    MacroStorage(const MacroStorage&) = delete;
    MacroStorage& operator=(const MacroStorage&) = delete;

    /**
     * @brief Initialize storage - must be called once at startup
     * Opens NVS handle and loads existing data or populates defaults.
     */
    esp_err_t init();

    /**
     * @brief Check if storage is initialized
     */
    bool isInitialized() const { return initialized_; }

    // === Macro Operations ===

    /**
     * @brief Get a macro by ID
     * @param macroId ID of macro (1-50)
     * @param macro Output parameter
     * @return ESP_OK if found and enabled, ESP_ERR_NOT_FOUND otherwise
     */
    esp_err_t getMacro(uint8_t macroId, MacroDefinition& macro) const;

    /**
     * @brief Set/update a macro
     * @param macroId ID of macro (1-50)
     * @param macro Macro definition to store
     */
    esp_err_t setMacro(uint8_t macroId, const MacroDefinition& macro);

    /**
     * @brief Delete a macro (marks as disabled, clears from slots)
     * @param macroId ID of macro (1-50)
     */
    esp_err_t deleteMacro(uint8_t macroId);

    // === Slot Assignment Operations ===

    /**
     * @brief Get all F-button slot assignments
     * @param assignments Output array (size kMacroSlotCount)
     */
    esp_err_t getSlotAssignments(std::array<uint8_t, kMacroSlotCount>& assignments) const;

    /**
     * @brief Get slot assignments as raw pointer (for C compatibility)
     */
    esp_err_t getSlotAssignments(uint8_t* assignments) const;

    /**
     * @brief Set slot assignment
     * @param slot Slot index (0-11)
     * @param macroId Macro ID (0=empty, 1-50=macro)
     */
    esp_err_t setSlotAssignment(uint8_t slot, uint8_t macroId);

    // === Utility ===

    /**
     * @brief Get count of enabled macros
     */
    uint8_t getCount() const;

    /**
     * @brief Get maximum macro capacity (always 50)
     */
    static constexpr uint8_t getMaxCount() { return kMacroCountMax; }

    /**
     * @brief Reset to factory defaults
     */
    esp_err_t factoryReset();

    /**
     * @brief Load all data from NVS
     */
    esp_err_t load(MacroStorageData& data);

    /**
     * @brief Save all data to NVS
     */
    esp_err_t save(const MacroStorageData& data);

private:
    MacroStorage() = default;
    ~MacroStorage();

    // NOTE ON THREAD SAFETY:
    // cache_ is written from the CAT dispatch task (setMacro/deleteMacro/
    // setSlotAssignment/factoryReset) and read from the ButtonHandler task
    // (getMacro/getSlotAssignments/getCount via RadioMacroManager). All cache_
    // accesses are guarded by cacheMutex_. NVS flash I/O is performed OUTSIDE
    // the mutex (on a snapshot copy) so the latency-sensitive button read path
    // never blocks on a flash commit. cache_ is the authoritative in-memory
    // copy; a persist failure leaves cache_ ahead of NVS, which is acceptable.
    //
    // populateDefaults() and computeMacroCount() touch cache_ directly and MUST
    // be called with cacheMutex_ held by the caller.
    void populateDefaults();
    uint8_t computeMacroCount() const;
    bool nvsHasMacroData() const;
    esp_err_t persist();

    // Serialize `data` to NVS (nvs_set_blob + nvs_commit). Pure flash I/O:
    // touches neither cache_ nor cacheMutex_, so it is safe to call unlocked.
    esp_err_t writeBlobToNvs(const MacroStorageData& data) const;

    uint32_t nvsHandle_ = 0;
    bool initialized_ = false;
    MacroStorageData cache_;
    mutable RtosMutex cacheMutex_;  // Guards all cache_ reads and writes

    static constexpr const char* TAG = "MacroStorage";
};

}  // namespace storage
