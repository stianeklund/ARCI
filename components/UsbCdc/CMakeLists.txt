idf_component_register(SRCS "UsbCdc.cpp" "CdcSerialHandler.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_driver_gpio ISerialChannel
                       PRIV_REQUIRES tinyusb)