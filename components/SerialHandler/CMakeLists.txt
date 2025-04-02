idf_component_register(SRCS "SerialHandler.cpp" "mock/MockSerialHandler.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_timer ISerialChannel)
