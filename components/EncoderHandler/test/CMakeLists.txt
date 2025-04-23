# Only register test component when unit tests are enabled
if(CONFIG_RUN_UNIT_TESTS)
    idf_component_register(
            SRCS "test_encoder_handler.cpp"
            INCLUDE_DIRS "../include"
            REQUIRES unity MockSerialHandler esp_timer
    )
else()
    # Create empty component when tests disabled
    idf_component_register(SRCS
                          INCLUDE_DIRS)
endif()
