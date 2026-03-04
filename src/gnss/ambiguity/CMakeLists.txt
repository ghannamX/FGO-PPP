add_library(gnss_ambiguity OBJECT)

target_sources(gnss_ambiguity PRIVATE
    # LambdaSolver.cpp
    # WideLaneNarrowLane.cpp
    # PhaseBiasCorrector.cpp
    # AmbiguityPool.cpp
)

target_include_directories(gnss_ambiguity
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gnss_ambiguity
    PUBLIC Eigen3::Eigen fgo_compile_options
)
