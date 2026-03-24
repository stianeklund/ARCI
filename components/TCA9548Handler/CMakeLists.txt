idf_component_register(
    SRCS "TCA9548Handler.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c
)