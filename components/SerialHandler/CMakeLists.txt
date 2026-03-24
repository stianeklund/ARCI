idf_component_register(SRCS "SerialHandler.cpp" "mock/MockSerialHandler.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_uart esp_timer ISerialChannel)
