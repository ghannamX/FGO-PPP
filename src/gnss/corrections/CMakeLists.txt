add_library(gnss_corrections OBJECT)

target_sources(gnss_corrections PRIVATE
    # Troposphere.cpp
    # Ionosphere.cpp
    # AntennaPCC.cpp
    # PhaseWindup.cpp
    # SolidEarthTide.cpp
    # RelativisticEffect.cpp
)

target_include_directories(gnss_corrections
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gnss_corrections
    PUBLIC Eigen3::Eigen gnss_time fgo_compile_options
)
