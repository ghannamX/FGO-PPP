add_library(gnss_io OBJECT)

target_sources(gnss_io PRIVATE
    # Stubs — populated as parsers are implemented
    # rinex/ObsParser.cpp
    # rinex/NavParser.cpp
    # rinex/ClockParser.cpp
    # sp3/Sp3Parser.cpp
    # antex/AntexParser.cpp
    # ionex/IonexParser.cpp
    # dcb/DcbParser.cpp
    # ssr/SsrDecoder.cpp
)

target_include_directories(gnss_io
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gnss_io
    PUBLIC Eigen3::Eigen spdlog::spdlog fgo_compile_options
)
