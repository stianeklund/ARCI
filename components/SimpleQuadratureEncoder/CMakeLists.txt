idf_component_register(
    SRCS "SimpleQuadratureEncoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_timer freertos QuadratureDecoder
)
