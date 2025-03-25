idf_component_register(
    SRCS "TCA8418Handler.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "driver" "freertos" "esp_common" "ButtonHandler" "TCA9548Handler"
)