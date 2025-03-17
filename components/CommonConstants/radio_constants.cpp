 // radio_constants.cpp
 #include "radio_constants.h"

 namespace radio {
     extern const int FW_BANDWIDTH[] = {50, 80, 100, 150, 200, 250, 300, 400, 500, 600, 1000, 1500, 2000, 2500};
     extern const int FW_BANDWIDTH_SIZE = sizeof(FW_BANDWIDTH) / sizeof(FW_BANDWIDTH[0]);
     extern const int IF_SHIFT_VALUES[] = {300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000};
     extern const int IF_SHIFT_SIZE = sizeof(IF_SHIFT_VALUES) / sizeof(IF_SHIFT_VALUES[0]);
 }

