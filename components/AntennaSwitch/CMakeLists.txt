idf_component_register(
    SRCS "AntennaSwitch.cpp" "AntennaWebSocketClient.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_client json nvs_flash esp_netif esp_wifi esp_websocket_client
    PRIV_REQUIRES json
)