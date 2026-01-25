#include "MXCommandHandler.h"
#include "RadioManager.h"
#include "RadioMacroManager.h"
#include "MacroStorage.h"
#include "esp_log.h"
#include <cstring>
#include <cctype>

namespace radio {

bool MXCommandHandler::handleCommand(const RadioCommand &command,
                                     ISerialChannel &radioSerial,
                                     ISerialChannel &usbSerial,
                                     RadioManager &radioManager) {
    std::string rawCommand = command.originalMessage;

    // Extract subcommand (MX + 1 char: W, R, A, E, D)
    if (rawCommand.size() < 3) {
        ESP_LOGW(TAG, "Invalid MX command format: %s", rawCommand.c_str());
        return true; // Consume but ignore malformed
    }

    char subcmd = rawCommand[2];

    // Route to appropriate handler
    if (subcmd == 'W') {
        return handleMXW(command, usbSerial, radioManager);
    } else if (subcmd == 'R') {
        return handleMXR(command, usbSerial, radioManager);
    } else if (subcmd == 'A') {
        return handleMXA(command, usbSerial, radioManager);
    } else if (subcmd == 'E') {
        return handleMXE(command, radioSerial, radioManager);
    } else if (subcmd == 'D') {
        return handleMXD(command, usbSerial, radioManager);
    } else {
        ESP_LOGW(TAG, "Unknown MX subcommand: %c", subcmd);
        return true;
    }
}

bool MXCommandHandler::handleMXW(const RadioCommand &command,
                                 ISerialChannel &usbSerial,
                                 RadioManager &radioManager) {
    // Write/Save macro definition
    // Command format: MXW<ID>,<name>,<commands>;
    // Example: MXW01,20M FT8,FA00014074000|MD2|DA1;
    // Pipe '|' separates commands internally

    std::string raw = command.originalMessage;

    // Find the terminator ;
    size_t semi_pos = raw.rfind(';');
    if (semi_pos == std::string::npos || semi_pos < 6) {
        ESP_LOGW(TAG, "MXW missing terminator or too short");
        return true;
    }

    // Parse: MXW<ID>,<name>,<commands>;
    // Positions: 0-2="MXW", 3-4=ID, 5=",", 6+...

    if (raw.size() < 8) { // At least MXW + ID + comma
        ESP_LOGW(TAG, "MXW command too short");
        return true;
    }

    uint8_t macro_id = parseMacroId(raw.substr(3, 2));
    if (macro_id < 1 || macro_id > 50) {
        ESP_LOGW(TAG, "Invalid MXW macro ID: %d", macro_id);
        return true;
    }

    // Extract name and commands from comma-separated values
    // Find first comma at position 5
    if (raw[5] != ',') {
        ESP_LOGW(TAG, "MXW malformed: expected comma at position 5");
        return true;
    }

    // Find second comma (separates name and commands)
    size_t second_comma = raw.find(',', 6);
    if (second_comma == std::string::npos || second_comma >= semi_pos) {
        ESP_LOGW(TAG, "MXW malformed: missing second comma");
        return true;
    }

    std::string name = raw.substr(6, second_comma - 6);
    std::string commands = raw.substr(second_comma + 1, semi_pos - second_comma - 1);

    // Validate lengths
    if (name.length() >= storage::kMacroNameMaxLength ||
        commands.length() >= storage::kMacroCommandMaxLength) {
        ESP_LOGW(TAG, "MXW name or commands too long");
        return true;
    }

    // Create and save macro
    storage::MacroDefinition macro{};
    std::strncpy(macro.name, name.c_str(), storage::kMacroNameMaxLength - 1);
    std::strncpy(macro.command, commands.c_str(), storage::kMacroCommandMaxLength - 1);
    macro.enabled = true;

    esp_err_t err = storage::MacroStorage::instance().setMacro(macro_id, macro);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save macro %d: %s", macro_id, esp_err_to_name(err));
        return true;
    }

    // Send acknowledgment: MXW<ID>;
    char ack[16];
    std::snprintf(ack, sizeof(ack), "MXW%02d;", macro_id);

    // MX commands are local-only - always respond to source, never to radio
    respondToSource(command, ack, usbSerial, radioManager);

    ESP_LOGI(TAG, "MXW saved macro %d: '%s' (%.40s...)", macro_id, name.c_str(), commands.c_str());
    return true;
}

bool MXCommandHandler::handleMXR(const RadioCommand &command,
                                 ISerialChannel &usbSerial,
                                 RadioManager &radioManager) {
    // Read macro by ID
    // Command format: MXR<ID>; (e.g., MXR01;)
    // Response format: MXR<ID>,<name>,<commands>;

    std::string raw = command.originalMessage;
    // Extract ID: MXR[01-50]; -> positions 3-4
    if (raw.size() < 5) {
        ESP_LOGW(TAG, "Invalid MXR format: %s", raw.c_str());
        return true;
    }

    uint8_t macro_id = parseMacroId(raw.substr(3, 2));
    if (macro_id < 1 || macro_id > 50) {
        ESP_LOGW(TAG, "Invalid macro ID: %d", macro_id);
        return true;
    }

    std::string response = buildMXRResponse(macro_id);

    // MX commands are local-only - always respond to source, never to radio
    respondToSource(command, response, usbSerial, radioManager);

    ESP_LOGI(TAG, "MXR%02d response: %s", macro_id, response.c_str());
    return true;
}

bool MXCommandHandler::handleMXA(const RadioCommand &command,
                                 ISerialChannel &usbSerial,
                                 RadioManager &radioManager) {
    // F-button slot assignment query/set
    // Query: MXA;
    //   Response: MXA01,02,03,04,05,00; (F1-F6 macro IDs, 00 = empty)
    // Set: MXA<slot>,<macro_id>;  (e.g., MXA3,05; assigns macro 05 to F3)
    //   Response: MXA<slot>,<macro_id>; (acknowledgment)

    std::string raw = command.originalMessage;

    // Check if this is a set command (has parameters after MXA)
    // Format: MXA<slot>,<macro_id>; where slot=1-12, macro_id=00-50
    // Single digit slot: MXA3,05; (length 8)
    // Two digit slot: MXA10,05; (length 9)
    if (raw.size() >= 8 && raw[3] >= '1' && raw[3] <= '9') {
        // SET command: MXA<slot>,<macro_id>;
        uint8_t slot;
        size_t macro_id_pos;

        // Parse slot number (1-2 digits)
        if (raw[4] == ',') {
            // Single digit slot (1-9)
            slot = raw[3] - '1';  // Convert '1'-'9' to 0-8
            macro_id_pos = 5;
        } else if (raw.size() >= 9 && raw[4] >= '0' && raw[4] <= '2' && raw[5] == ',') {
            // Two digit slot (10-12)
            int slot_num = (raw[3] - '0') * 10 + (raw[4] - '0');
            if (slot_num < 10 || slot_num > 12) {
                ESP_LOGW(TAG, "Invalid MXA slot: %d (must be 1-12)", slot_num);
                return true;
            }
            slot = slot_num - 1;  // Convert 10-12 to 9-11
            macro_id_pos = 6;
        } else {
            ESP_LOGW(TAG, "Invalid MXA format: %s", raw.c_str());
            return true;
        }

        // Validate slot range
        if (slot >= 12) {
            ESP_LOGW(TAG, "Invalid MXA slot: %d (must be 0-11)", slot);
            return true;
        }

        // Parse macro ID (2 digits)
        uint8_t macro_id = parseMacroId(raw.substr(macro_id_pos, 2));
        if (macro_id > 50) {
            ESP_LOGW(TAG, "Invalid MXA macro ID: %d", macro_id);
            return true;
        }

        // Set the slot assignment
        esp_err_t err = storage::MacroStorage::instance().setSlotAssignment(slot, macro_id);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set slot %d to macro %d: %s",
                     slot + 1, macro_id, esp_err_to_name(err));
            return true;
        }

        // Send acknowledgment: MXA<slot>,<macro_id>;
        char ack[16];
        std::snprintf(ack, sizeof(ack), "MXA%d,%02d;", slot + 1, macro_id);

        // MX commands are local-only - always respond to source, never to radio
        respondToSource(command, ack, usbSerial, radioManager);

        ESP_LOGI(TAG, "MXA: Slot %d assigned to macro %d", slot + 1, macro_id);
        return true;
    }

    // QUERY command: MXA; - return all 12 slot assignments
    std::string response = "MXA";
    uint8_t assignments[storage::kMacroSlotCount] = {0};

    if (storage::MacroStorage::instance().getSlotAssignments(assignments) == ESP_OK) {
        for (size_t i = 0; i < storage::kMacroSlotCount; i++) {
            if (i > 0) response += ",";
            char slot_str[3];
            std::snprintf(slot_str, sizeof(slot_str), "%02d", assignments[i]);
            response += slot_str;
        }
    }
    response += ";";

    // MX commands are local-only - always respond to source, never to radio
    respondToSource(command, response, usbSerial, radioManager);

    ESP_LOGD(TAG, "MXA query response: %s", response.c_str());
    return true;
}

bool MXCommandHandler::handleMXE(const RadioCommand &command,
                                 ISerialChannel &radioSerial,
                                 RadioManager &radioManager) {
    // Execute macro from F-button slot
    // Command format: MXE<slot>; where slot is 1-12 (1-6: short press, 7-12: long press)

    std::string raw = command.originalMessage;

    if (raw.size() < 4) {
        ESP_LOGW(TAG, "Invalid MXE format: %s", raw.c_str());
        return true;
    }

    // Extract slot number (1-2 digits)
    int slot_num;
    if (raw.size() >= 5 && raw[4] >= '0' && raw[4] <= '9') {
        // Two digit slot (10-12)
        slot_num = (raw[3] - '0') * 10 + (raw[4] - '0');
    } else {
        // Single digit slot (1-9)
        slot_num = raw[3] - '0';
    }

    if (slot_num < 1 || slot_num > 12) {
        ESP_LOGW(TAG, "Invalid MXE slot: %d (must be 1-12)", slot_num);
        return true;
    }

    // Convert 1-based slot to 0-based array index
    uint8_t slot_idx = slot_num - 1;

    // Execute macro via RadioMacroManager
    RadioMacroManager *macroManager = radioManager.getMacroManager();
    if (macroManager == nullptr) {
        ESP_LOGE(TAG, "MacroManager not available");
        return true;
    }

    esp_err_t err = macroManager->executeSlot(slot_idx);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to execute macro from slot %d: %s", slot_num, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "MXE executed macro from slot %d", slot_num);
    }

    return true;
}

bool MXCommandHandler::handleMXD(const RadioCommand &command,
                                 ISerialChannel &usbSerial,
                                 RadioManager &radioManager) {
    // Delete macro by ID
    // Command format: MXD<ID>; (e.g., MXD05;)

    std::string raw = command.originalMessage;

    if (raw.size() < 5) {
        ESP_LOGW(TAG, "Invalid MXD format: %s", raw.c_str());
        return true;
    }

    uint8_t macro_id = parseMacroId(raw.substr(3, 2));
    if (macro_id < 1 || macro_id > 50) {
        ESP_LOGW(TAG, "Invalid MXD macro ID: %d", macro_id);
        return true;
    }

    esp_err_t err = storage::MacroStorage::instance().deleteMacro(macro_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete macro %d: %s", macro_id, esp_err_to_name(err));
        return true;
    }

    // Send acknowledgment: MXD<ID>;
    char ack[16];
    std::snprintf(ack, sizeof(ack), "MXD%02d;", macro_id);

    // MX commands are local-only - always respond to source, never to radio
    respondToSource(command, ack, usbSerial, radioManager);

    ESP_LOGI(TAG, "MXD deleted macro %d", macro_id);
    return true;
}

std::string MXCommandHandler::buildMXRResponse(uint8_t macro_id) {
    storage::MacroDefinition macro{};

    if (storage::MacroStorage::instance().getMacro(macro_id, macro) != ESP_OK) {
        // Macro not found - return empty response
        char response[16];
        std::snprintf(response, sizeof(response), "MXR%02d,,;", macro_id);
        return std::string(response);
    }

    // Build response: MXR<ID>,<name>,<commands>;
    std::string response = "MXR";
    char id_str[3];
    std::snprintf(id_str, sizeof(id_str), "%02d", macro_id);
    response += id_str;
    response += ",";
    response += macro.name;
    response += ",";
    response += macro.command;
    response += ";";

    return response;
}

uint8_t MXCommandHandler::parseMacroId(std::string_view param) {
    // Parse 2-digit zero-padded ID (01-50)
    if (param.size() < 2) return 0;

    // Handle digits properly
    if (!std::isdigit(static_cast<unsigned char>(param[0])) ||
        !std::isdigit(static_cast<unsigned char>(param[1]))) {
        return 0;
    }

    uint8_t id = (param[0] - '0') * 10 + (param[1] - '0');
    return (id >= 1 && id <= 50) ? id : 0;
}

} // namespace radio
