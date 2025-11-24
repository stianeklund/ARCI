idf_component_register(
    SRCS "tests_main.cpp"
         "test_output_capture.cpp"
         "../ButtonHandler/test/test_button_handler.cpp"
         "../RadioCore/test/test_radiocat_handler.cpp"
         "../RadioCore/test/test_radiomanager_cat_commands.cpp"
         "../RadioCore/test/CATTestUtils.cpp"
         "../CatParser/test/test_cat_parser.cpp"
         "../CatParser/test/test_cat_parser_utils.cpp"
         "../CommandHandlers/test/test_consolidated_command_handlers.cpp"
         "../RadioCore/test/test_cache.cpp"
         "../RadioCore/test/test_meter_polling_performance.cpp"
         "../CommandHandlers/test/test_transverter_features.cpp"
         "../EncoderHandler/test/test_encoder_handler.cpp"
         "../RadioMacroManager/test/test_radio_macro_manager.cpp"
    INCLUDE_DIRS "include"
                 "../ButtonHandler/include"
                 "../SerialHandler/mock/include"
                 "../RadioCore/include"
                 "../RadioCore/test/include"
                 "../CatParser/include"
                 "../StateManager/include"
                 "../CommandHandlers/include"
                 "../EncoderHandler/include"
                 "../RadioMacroManager/include"
    REQUIRES unity 
             ButtonHandler 
             SerialHandler 
             Button 
             RadioCore 
             CatParser
             StateManager
             CommandHandlers
             EncoderHandler 
             RadioMacroManager
)

# Add explicit include directories for the test files
target_include_directories(${COMPONENT_LIB} PRIVATE 
    "../ButtonHandler/test"
    "../RadioCore/test"
    "../CatParser/test"
    "../CommandHandlers/test"
    "../EncoderHandler/test"
    "../RadioMacroManager/test"
)

# Ensure Unity pulls in our config header to control output verbosity
target_compile_definitions(${COMPONENT_LIB} PRIVATE UNITY_INCLUDE_CONFIG_H)

# Also apply the config to the Unity component itself so it routes output
idf_component_get_property(UNITY_LIB unity COMPONENT_LIB)
if(UNITY_LIB)
    target_compile_definitions(${UNITY_LIB} PRIVATE UNITY_INCLUDE_CONFIG_H)
    target_include_directories(${UNITY_LIB} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
endif()

# Unity source files are vendored under CMock; apply config there too
idf_component_get_property(CMOCK_LIB cmock COMPONENT_LIB)
if(CMOCK_LIB)
    target_compile_definitions(${CMOCK_LIB} PRIVATE UNITY_INCLUDE_CONFIG_H)
    target_include_directories(${CMOCK_LIB} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
endif()
