idf_component_register(
    SRCS "TCA8418Handler.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "esp_driver_gpio" "esp_driver_i2c" "freertos" "esp_common" "ButtonHandler" "TCA9548Handler"
)