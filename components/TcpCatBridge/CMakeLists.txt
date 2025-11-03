idf_component_register(
    SRCS "TcpCatBridge.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        lwip
        ISerialChannel
)
