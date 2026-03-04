add_library(sensor_fusion OBJECT)

target_sources(sensor_fusion PRIVATE
    # imu/ImuPreintegrator.cpp
    # imu/ImuBiasModel.cpp
    # fusion/GnssFgoFusion.cpp
)

target_include_directories(sensor_fusion
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(sensor_fusion
    PUBLIC
        Eigen3::Eigen
        fgo_core
        fgo_compile_options
)
