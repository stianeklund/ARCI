idf_component_register(SRCS "Diagnostics.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES RadioCore TCA9548Handler TCA8418Handler nvs_flash)
