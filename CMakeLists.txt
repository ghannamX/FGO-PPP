cmake_minimum_required(VERSION 3.25)

project(FGO_PPP
    VERSION 0.1.0
    DESCRIPTION "Factor Graph Optimization for Precise Point Positioning"
    LANGUAGES CXX
)

# ── Language standard ────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build options ────────────────────────────────────────────────────────────
option(FGO_BUILD_TESTS    "Build unit tests"          ON)
option(FGO_BUILD_APPS     "Build application targets" ON)
option(FGO_USE_GTSAM      "Enable GTSAM factor graph" OFF)  # Phase 3+

# ── Compile options ──────────────────────────────────────────────────────────
add_library(fgo_compile_options INTERFACE)
target_compile_options(fgo_compile_options INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:
        -Wall -Wextra -Wpedantic
        -Wno-unused-parameter
        $<$<CONFIG:Release>:-O3 -march=native>
    >
    $<$<CXX_COMPILER_ID:MSVC>:
        /W4
        $<$<CONFIG:Release>:/O2>
    >
)

# ── Conan-provided dependencies ──────────────────────────────────────────────
find_package(Eigen3  REQUIRED CONFIG)
find_package(spdlog  REQUIRED CONFIG)
find_package(fmt     REQUIRED CONFIG)

if(FGO_BUILD_TESTS)
    find_package(GTest REQUIRED CONFIG)
    enable_testing()
endif()

# ── Optional: GTSAM via FetchContent (not on ConanCenter) ───────────────────
if(FGO_USE_GTSAM)
    include(FetchContent)
    FetchContent_Declare(
        gtsam
        GIT_REPOSITORY https://github.com/borglab/gtsam.git
        GIT_TAG        4.2a9
        GIT_SHALLOW    TRUE
    )
    set(GTSAM_BUILD_TESTS     OFF CACHE BOOL "" FORCE)
    set(GTSAM_BUILD_EXAMPLES  OFF CACHE BOOL "" FORCE)
    set(GTSAM_BUILD_DOCS      OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(gtsam)
endif()

# ── Subdirectories ───────────────────────────────────────────────────────────
add_subdirectory(src)

if(FGO_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(FGO_BUILD_APPS)
    add_subdirectory(src/apps)
endif()
