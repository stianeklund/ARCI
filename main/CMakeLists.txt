# Conditionally require unit_test component
if(CONFIG_RUN_UNIT_TESTS)
    idf_component_register(SRCS "main.cpp"
                        INCLUDE_DIRS "."
                        REQUIRES RadioCore ButtonHandler ADCHandler EncoderHandler MultiEncoderHandler nvs_flash unit_test UsbCdc TCA8418Handler TCA9548Handler PCF8575Handler NvsManager Diagnostics driver RadioMacroManager CommonConstants WiFiManager TcpCatBridge)
else()
    idf_component_register(SRCS "main.cpp"
                        INCLUDE_DIRS "."
                        REQUIRES RadioCore ButtonHandler ADCHandler EncoderHandler MultiEncoderHandler nvs_flash UsbCdc TCA8418Handler TCA9548Handler PCF8575Handler NvsManager Diagnostics driver RadioMacroManager CommonConstants WiFiManager TcpCatBridge
                        PRIV_REQUIRES unity)
endif()

# Add test components to the main component
add_dependencies(${COMPONENT_LIB} __idf_ButtonHandler)