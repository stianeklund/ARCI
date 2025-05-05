# Conditionally include test files
if(CONFIG_RUN_UNIT_TESTS)
    idf_component_register(SRCS "ButtonHandler.cpp" "MatrixButton.cpp" "test/test_button_handler.cpp"
                           INCLUDE_DIRS "include"
                           REQUIRES driver RadioCore Button SerialHandler TCA8418Handler RadioMacroManager NvsManager FrequencyFormatter unity)
else()
    idf_component_register(SRCS "ButtonHandler.cpp" "MatrixButton.cpp"
                           INCLUDE_DIRS "include"
                           REQUIRES driver RadioCore Button SerialHandler TCA8418Handler RadioMacroManager NvsManager FrequencyFormatter)
endif()

# Force the test function symbol to be included in the build
if(CONFIG_RUN_UNIT_TESTS)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC CONFIG_RUN_UNIT_TESTS)
endif()
