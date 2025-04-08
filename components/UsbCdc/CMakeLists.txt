idf_component_register(SRCS "UsbCdc.cpp" "CdcSerialHandler.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos driver ISerialChannel
                       PRIV_REQUIRES tinyusb)