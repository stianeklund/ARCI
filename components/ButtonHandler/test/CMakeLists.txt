# Only register test component when unit tests are enabled
if(CONFIG_RUN_UNIT_TESTS)
    idf_component_register(SRCS "test_button_handler.cpp"
                          INCLUDE_DIRS "../include"
                          REQUIRES unity RadioCore Button SerialHandler ButtonHandler)

    target_link_libraries(${COMPONENT_LIB} PUBLIC "-Wl,--undefined=run_button_handler_tests")
else()
    # Create empty component when tests disabled
    idf_component_register(SRCS
                          INCLUDE_DIRS)
endif()