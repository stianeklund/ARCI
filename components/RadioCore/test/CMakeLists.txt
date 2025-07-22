# Only register test component when unit tests are enabled
if(CONFIG_RUN_UNIT_TESTS)
    idf_component_register(SRCS "test_radiocat_handler.cpp" "test_cat_parser_utils.cpp"
            "test_unified_cat_system.cpp" "test_consolidated_command_handlers.cpp"
            "test_cache_system.cpp" "test_radiomanager_cat_commands.cpp" "CATTestUtils.cpp"
            "test_cat_parser.cpp" "test_command_handlers.cpp" "test_cat_integration.cpp"
            "test_cache_behavior.cpp"
            INCLUDE_DIRS "../include" "include"
            REQUIRES unity driver esp_timer CommonConstants SerialHandler RadioCore)

    target_link_libraries(${COMPONENT_LIB} PUBLIC "-Wl,--undefined=run_radiocat_handler_tests")
    target_link_libraries(${COMPONENT_LIB} PUBLIC "-Wl,--undefined=run_cat_parser_utils_tests")
    target_link_libraries(${COMPONENT_LIB} PUBLIC "-Wl,--undefined=run_unified_cat_system_tests")
    target_link_libraries(${COMPONENT_LIB} PUBLIC "-Wl,--undefined=run_consolidated_command_handlers_tests")
    target_link_libraries(${COMPONENT_LIB} PUBLIC "-Wl,--undefined=run_cache_system_tests")
    target_link_libraries(${COMPONENT_LIB} PUBLIC "-Wl,--undefined=run_radiomanager_cat_tests")
    target_link_libraries(${COMPONENT_LIB} PUBLIC "-Wl,--undefined=run_cat_parser_tests")
    target_link_libraries(${COMPONENT_LIB} PUBLIC "-Wl,--undefined=run_cache_behavior_tests")
else()
    # Create empty component when tests disabled
    idf_component_register(SRCS
                          INCLUDE_DIRS)
endif()