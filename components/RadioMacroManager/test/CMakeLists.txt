# Only register test component when unit tests are enabled
if(CONFIG_RUN_UNIT_TESTS)
    idf_component_register(
        SRCS
            "test_radio_macro_manager.cpp"
        INCLUDE_DIRS
            ".."
        REQUIRES
            "unity"
            "RadioMacroManager"
            "RadioCore"
            "SerialHandler"
        PRIV_REQUIRES
    )
else()
    # Create empty component when tests disabled
    idf_component_register(SRCS
                          INCLUDE_DIRS)
endif()