idf_component_register(
    SRCS "test_antenna_websocket.cpp"
    INCLUDE_DIRS "../include"
    REQUIRES AntennaSwitch unity
    PRIV_REQUIRES esp_websocket_client json
)