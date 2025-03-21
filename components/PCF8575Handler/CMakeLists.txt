idf_component_register(
    SRCS "PCF8575Handler.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver TCA9548Handler
)