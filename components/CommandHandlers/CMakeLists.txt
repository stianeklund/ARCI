set(srcs
    "BaseCommandHandler.cpp"
    "frequency/FrequencyVfoCommandHandler.cpp"
    "mode/ModeCommandHandler.cpp"
    "antenna/AntennaCommandHandler.cpp"
    "audio/AudioEqualizerCommandHandler.cpp"
    "audio/VoiceMessageCommandHandler.cpp"
    "cw/CwCommandHandler.cpp"
    "gain/GainLevelCommandHandler.cpp"
    "interface/InterfaceSystemCommandHandler.cpp"
    "macro/MXCommandHandler.cpp"
    "memory/MemoryCommandHandler.cpp"
    "menu/MenuConfigCommandHandler.cpp"
    "menu/ExtendedCommandHandler.cpp"
    "receiver/ReceiverProcessingCommandHandler.cpp"
    "scan/ScanCommandHandler.cpp"
    "scan/VisualScanCommandHandler.cpp"
    "status/StatusInfoCommandHandler.cpp"
    "tone/ToneSquelchCommandHandler.cpp"
    "transmitter/TransmitterCommandHandler.cpp"
    "ui/UICommandHandler.cpp"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES "CatParser" "StateManager" "ISerialChannel" "CommonConstants" "FrequencyFormatter" "MacroStorage" "RadioMacroManager"
    PRIV_REQUIRES "esp_timer" "RadioCore"
)