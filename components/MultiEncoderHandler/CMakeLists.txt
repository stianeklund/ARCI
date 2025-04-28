idf_component_register(
    SRCS "MultiEncoderHandler.cpp"
    INCLUDE_DIRS "include"
    REQUIRES SimpleQuadratureEncoder PCF8575Handler RadioCore QuadratureDecoder
)