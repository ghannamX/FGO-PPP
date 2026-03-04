add_library(gnss_time OBJECT)

target_sources(gnss_time PRIVATE
    GnssTime.cpp
    CoordFrame.cpp
)

target_include_directories(gnss_time
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gnss_time
    PUBLIC Eigen3::Eigen fgo_compile_options
)
