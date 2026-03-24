idf_component_register(
    SRCS "tests_main.cpp"
         "test_output_capture.cpp"
    INCLUDE_DIRS "include"
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
