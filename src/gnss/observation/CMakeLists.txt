add_library(gnss_observation OBJECT)

target_sources(gnss_observation PRIVATE
    # PseudorangeModel.cpp
    # CarrierPhaseModel.cpp
    # DopplerModel.cpp
    # IFCombination.cpp
    # UCModel.cpp
)

target_include_directories(gnss_observation
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gnss_observation
    PUBLIC Eigen3::Eigen gnss_satellite gnss_corrections fgo_compile_options
)
