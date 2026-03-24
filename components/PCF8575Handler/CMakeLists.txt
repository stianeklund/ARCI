idf_component_register(
    SRCS "PCF8575Handler.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_i2c TCA9548Handler
)