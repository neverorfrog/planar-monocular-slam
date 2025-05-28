enable_testing()

# Create test executable with explicit source files
add_executable(pms_tests 
    test_manifolds.cpp
    test_state.cpp
    # test_constraints.cpp
    # test_bundle_adjuster.cpp
    test_camera.cpp
    test_main.cpp
)

# Link libraries
target_link_libraries(pms_tests 
    PRIVATE
    ${PMS_CPP_LIB}
    gtest
    gtest_main
)

# Include directories
target_include_directories(pms_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Discover tests
include(GoogleTest)
gtest_discover_tests(pms_tests)
