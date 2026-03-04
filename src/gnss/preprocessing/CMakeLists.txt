add_library(gnss_preprocessing OBJECT)

target_sources(gnss_preprocessing PRIVATE
    # CycleSlipDetector.cpp
    # OutlierRejector.cpp
    # HatchFilter.cpp
)

target_include_directories(gnss_preprocessing
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gnss_preprocessing
    PUBLIC Eigen3::Eigen gnss_time fgo_compile_options
)
