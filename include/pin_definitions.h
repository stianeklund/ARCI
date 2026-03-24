#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#pragma once

// Common definitions for both platforms
#define ROTARY_ENCODER_STEPS 4

// Platform-specific pin definitions
#if defined(CONFIG_IDF_TARGET_ESP32)
    // Original ESP32 pin definitions
    #define PIN_ENCODER_A GPIO_NUM_21
    #define PIN_ENCODER_B GPIO_NUM_22
    #define PIN_MODE_SWITCH GPIO_NUM_13
    #define PIN_BAND_SWITCH GPIO_NUM_26
    #define PIN_BAND_MEMORY_SWITCH GPIO_NUM_15
    #define PIN_SW GPIO_NUM_12
    #define PIN_GP28 GPIO_NUM_10
    #define PIN_ON_OFF GPIO_NUM_5
    #define PIN_SPLIT_ON_OFF GPIO_NUM_25
    #define PIN_ANALOG_OUT GPIO_NUM_9
    #define PIN_AF_GAIN GPIO_NUM_33
    #define PIN_RF_GAIN GPIO_NUM_32
    #define PIN_RIT_XIT GPIO_NUM_34
    #define PIN_IF_SHIFT GPIO_NUM_35
    #define PIN_TF_SET GPIO_NUM_4
    #define PIN_TRANSVERTER_MACRO GPIO_NUM_16

    // I2C Pins
    #define PIN_I2C_SDA GPIO_NUM_1
    #define PIN_I2C_SCL GPIO_NUM_18

    // TCA8418 Key Matrix Interrupt Pin
    #define PIN_TCA8418_INT GPIO_NUM_17

#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    // ESP32-S3 pin definitions (Revised for ADC1 usage for all analog inputs)
    // Digital Inputs (Encoder, Switches)
    #define PIN_ENCODER_A GPIO_NUM_1           // Digital Input (Original: GPIO1)
    #define PIN_ENCODER_B GPIO_NUM_2           // Digital Input (Original: GPIO2)
    // UART1 pins (for radio communication)
    #define PIN_UART1_TX GPIO_NUM_17
    #define PIN_UART1_RX GPIO_NUM_18

    // #define PIN_MODE_SWITCH GPIO_NUM_21        // Digital Input (moved from GPIO17 to avoid UART conflict)
    // #define PIN_SPLIT_ON_OFF GPIO_NUM_11       // Digital Input (Original: GPIO9)
    // #define PIN_ON_OFF GPIO_NUM_12             // Digital Input (Original: GPIO8)
    // #define PIN_GP28 GPIO_NUM_13               // Digital Input (Original: GPIO7)
    // #define PIN_SW GPIO_NUM_14                 // Digital Input (Original: GPIO6)
    // #define PIN_BAND_MEMORY_SWITCH GPIO_NUM_15 // Digital Input (Original: GPIO5)
    // #define PIN_BAND_SWITCH GPIO_NUM_16        // Digital Input (Original: GPIO4)

    // Analog Inputs (All on ADC1)
    // #define PIN_TRANSVERTER_MACRO GPIO_NUM_4   // Analog Input (Original: GPIO16 on ADC2)
    // #define PIN_TF_SET GPIO_NUM_5              // Analog Input (Original: GPIO15 on ADC2)
    // GPIO6-9 freed - previously IF_SHIFT, RIT_XIT, RF_GAIN, AF_GAIN (now controlled by encoders)
    #define PIN_ANALOG_OUT GPIO_NUM_10         // Analog Input (Original: GPIO10 on ADC1)

    // I2C Pins
    #define PIN_I2C_SDA GPIO_NUM_41
    #define PIN_I2C_SCL GPIO_NUM_42
    
    // TCA8418 Key Matrix Interrupt Pins
    // Per ARCI-SMT.kicad_pcb: TCA1_INT → GPIO16, TCA2_INT → GPIO38
    #define PIN_TCA8418_INT GPIO_NUM_16        // TCA8418 #1 (Left & Right matrixes)
    #define PIN_TCA8418_INT_2 GPIO_NUM_38      // TCA8418 #2 (F1-F6 macros)

    // USB CDC Control Pins (DTR, RTS, CTS)
    #define PIN_USB_DTR GPIO_NUM_19            // Data Terminal Ready - output from ESP32
    #define PIN_USB_RTS GPIO_NUM_20            // Request To Send - output from ESP32
    #define PIN_USB_CTS GPIO_NUM_47            // Clear To Send - input to ESP32

    // Display UART pins (UART2 for display communication)
    #define PIN_DISPLAY_RX GPIO_NUM_11         // UART2 RX from display
    #define PIN_DISPLAY_TX GPIO_NUM_12         // UART2 TX to display

    // LED Control (WS2812 RGB LED)
    #define PIN_LED GPIO_NUM_8                 // WS2812 RGB LED

    // EC11E Dual-Axis Encoder #1 (AF/RF Gain) - NOW ON PCF8575 I2C EXPANDER
    // These GPIO pins (5-9) are now freed and available for other purposes
    // See MultiEncoderHandler.h for PCF8575 pin mappings (P10-P14)
    // #define PIN_EC11E1_ENC1_A GPIO_NUM_8       // FREED - was RF gain encoder axis A
    // #define PIN_EC11E1_ENC1_B GPIO_NUM_9       // FREED - was RF gain encoder axis B
    // #define PIN_EC11E1_ENC2_A GPIO_NUM_5       // FREED - was AF gain encoder axis A
    // #define PIN_EC11E1_ENC2_B GPIO_NUM_6       // FREED - was AF gain encoder axis B
    // #define PIN_EC11E1_ENC2_SW GPIO_NUM_7      // FREED - was AF gain encoder switch

    // Multi-Knob Encoder (Direct GPIO)
    #define PIN_MULTIKNOB_A GPIO_NUM_13        // Multi-knob CLK (KY-040)
    #define PIN_MULTIKNOB_B GPIO_NUM_14        // Multi-knob DT (KY-040)
    #define PIN_MULTIKNOB_SW GPIO_NUM_15       // Multi-knob switch

    // PCF8575 I2C GPIO Expander (for EC11E #2)
    #define PIN_PCF8575_INT GPIO_NUM_21        // PCF8575 interrupt pin
    
    // I2C Device Addresses
    #define TCA9548_I2C_ADDR 0x70              // TCA9548 I2C multiplexer
    #define PCF8575_I2C_ADDR 0x20              // PCF8575 GPIO expander

    // TCA9548 channel assignments
    #define TCA9548_CHANNEL_TCA8418_1 0
    #define TCA9548_CHANNEL_TCA8418_2 1
    #define TCA9548_CHANNEL_PCF8575 2

#elif defined(CONFIG_IDF_TARGET_ESP32P4)
    // ESP32-P4 pin definitions
    // NOTE: Adjust these pins to match your actual ESP32-P4 hardware layout
    #define PIN_ENCODER_A GPIO_NUM_1
    #define PIN_ENCODER_B GPIO_NUM_2

    // UART1 pins (for radio communication)
    #define PIN_UART1_TX GPIO_NUM_17
    #define PIN_UART1_RX GPIO_NUM_18

    // Analog Inputs
    #define PIN_ANALOG_OUT GPIO_NUM_10

    // I2C Pins
    #define PIN_I2C_SDA GPIO_NUM_41
    #define PIN_I2C_SCL GPIO_NUM_42

    // TCA8418 Key Matrix Interrupt Pins
    #define PIN_TCA8418_INT GPIO_NUM_16
    #define PIN_TCA8418_INT_2 GPIO_NUM_38

    // USB CDC Control Pins
    #define PIN_USB_DTR GPIO_NUM_19
    #define PIN_USB_RTS GPIO_NUM_20
    #define PIN_USB_CTS GPIO_NUM_47

    // Display UART pins
    #define PIN_DISPLAY_RX GPIO_NUM_11
    #define PIN_DISPLAY_TX GPIO_NUM_12

    // LED Control (WS2812 RGB LED)
    #define PIN_LED GPIO_NUM_8

    // Multi-Knob Encoder (Direct GPIO)
    #define PIN_MULTIKNOB_A GPIO_NUM_13
    #define PIN_MULTIKNOB_B GPIO_NUM_14
    #define PIN_MULTIKNOB_SW GPIO_NUM_15

    // PCF8575 I2C GPIO Expander
    #define PIN_PCF8575_INT GPIO_NUM_21

    // I2C Device Addresses
    #define TCA9548_I2C_ADDR 0x70
    #define PCF8575_I2C_ADDR 0x20

    // TCA9548 channel assignments
    #define TCA9548_CHANNEL_TCA8418_1 0
    #define TCA9548_CHANNEL_TCA8418_2 1
    #define TCA9548_CHANNEL_PCF8575 2

#else
    #error "Unsupported ESP32 variant. Please define pins for your specific target."
#endif

#endif // PIN_DEFINITIONS_H
