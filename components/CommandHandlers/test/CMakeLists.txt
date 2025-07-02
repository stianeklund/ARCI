# Only register test component when unit tests are enabled
if(CONFIG_RUN_UNIT_TESTS)
    idf_component_register(
        SRCS "test_consolidated_command_handlers.cpp"
             "test_transverter_features.cpp"
        INCLUDE_DIRS "include"
        REQUIRES "unity" "CommandHandlers" "RadioCore" "CatParser" "StateManager" "SerialHandler" "RadioMacroManager" "CommonConstants" "NvsManager" "esp_timer"
    )
else()
    # Create empty component when tests disabled
    idf_component_register(SRCS
                          INCLUDE_DIRS)
endif()