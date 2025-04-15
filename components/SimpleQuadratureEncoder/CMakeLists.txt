idf_component_register(
    SRCS "SimpleQuadratureEncoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos QuadratureDecoder
)
