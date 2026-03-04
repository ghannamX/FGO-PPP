# Promoted from INTERFACE to OBJECT when the first parser .cpp is added.
# Pattern:
#   add_library(gnss_input OBJECT)
#   target_sources(gnss_input PRIVATE rinex/RinexObsParser.cpp ...)

add_library(gnss_input INTERFACE)

target_include_directories(gnss_input
    INTERFACE ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gnss_input
    INTERFACE
        Eigen3::Eigen
        spdlog::spdlog
        gnss_time
        fgo_compile_options
)
