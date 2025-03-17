 // radio_constants.h
 #pragma once
#include <cstdint>

 namespace radio {
     // AI (Auto Information) mode constants
     enum class AIMode : uint8_t {
         Off = 0,         // AI0 - Auto Information OFF
         On = 2,          // AI2 - Auto Information ON
         OnWithBackup = 4 // AI2 - Auto Information with polling
     };
     
     extern const int FW_BANDWIDTH[];
     extern const int FW_BANDWIDTH_SIZE;
     extern const int IF_SHIFT_SIZE;
     extern const int IF_SHIFT_VALUES[];
 }
