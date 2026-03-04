add_library(fgo_core OBJECT)

target_sources(fgo_core PRIVATE
    # factors/PseudorangeFactor.cpp
    # factors/CarrierPhaseFactor.cpp
    # factors/DopplerFactor.cpp
    # factors/TroposphereFactor.cpp
    # factors/ClockFactor.cpp
    # graph/FactorGraphBuilder.cpp
    # smoother/SlidingWindowSmoother.cpp
)

target_include_directories(fgo_core
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(fgo_core
    PUBLIC
        Eigen3::Eigen
        gnss_observation
        fgo_compile_options
        $<$<BOOL:${FGO_USE_GTSAM}>:gtsam>
)
