add_library(gnss_solution OBJECT)

target_sources(gnss_solution PRIVATE
    # PositionSolution.cpp
    # NmeaSerializer.cpp
    # ResidualMonitor.cpp
)

target_include_directories(gnss_solution
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gnss_solution
    PUBLIC Eigen3::Eigen spdlog::spdlog fgo_compile_options
)
