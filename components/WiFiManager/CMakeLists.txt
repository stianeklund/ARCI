# ESP32-P4 and ESP32-H2 do not have WiFi hardware
if(IDF_TARGET STREQUAL "esp32p4" OR IDF_TARGET STREQUAL "esp32h2")
    idf_component_register(
        SRCS
        INCLUDE_DIRS "include"
    )
else()
    idf_component_register(
        SRCS "WiFiManager.cpp"
        INCLUDE_DIRS "include"
        REQUIRES esp_wifi esp_netif nvs_flash esp_event
    )
endif()