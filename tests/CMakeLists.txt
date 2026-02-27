enable_testing()

# ─── Unit Tests ───────────────────────────────────────────────────────────────
set(KINETRA_TEST_SOURCES
    unit/test_types.cpp
    unit/test_trajectory.cpp
    unit/test_se2_space.cpp
    unit/test_dubins.cpp
    unit/test_robot_models.cpp
    unit/test_collision.cpp
    unit/test_qp_solver.cpp
    unit/test_rrt_star.cpp
    unit/test_ilqr.cpp
)

add_executable(kinetra_tests ${KINETRA_TEST_SOURCES})

target_link_libraries(kinetra_tests PRIVATE
    kinetra
    GTest::gtest_main
)

target_include_directories(kinetra_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

include(GoogleTest)
gtest_discover_tests(kinetra_tests)

# ─── Integration Tests ────────────────────────────────────────────────────────
add_executable(kinetra_integration_tests
    integration/test_planning_scenarios.cpp
)
target_link_libraries(kinetra_integration_tests PRIVATE
    kinetra
    GTest::gtest_main
)
gtest_discover_tests(kinetra_integration_tests)
