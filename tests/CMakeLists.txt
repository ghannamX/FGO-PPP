include(GoogleTest)

function(fgo_add_test target_name)
    add_executable(${target_name} ${ARGN})
    target_link_libraries(${target_name}
        PRIVATE
            fgo_ppp
            GTest::gtest_main
    )
    gtest_discover_tests(${target_name})
endfunction()

# Phase 1 tests — uncomment as modules are implemented
# fgo_add_test(test_gnss_time   gnss/test_GnssTime.cpp)
# fgo_add_test(test_coord_frame gnss/test_CoordFrame.cpp)
# fgo_add_test(test_rinex_obs   gnss/io/test_ObsParser.cpp)
# fgo_add_test(test_rinex_nav   gnss/io/test_NavParser.cpp)
# fgo_add_test(test_sp3         gnss/io/test_Sp3Parser.cpp)
