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
    unit/test_stomp.cpp
    unit/test_nlp.cpp
    unit/test_json_export.cpp
    unit/test_polygon_obstacles.cpp
    unit/test_scenario_loader.cpp
    unit/test_sqp.cpp
    unit/test_reference_path.cpp
    unit/test_mpcc.cpp
    unit/test_mpcc_perf.cpp
    unit/test_riccati.cpp
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
if(CMAKE_CROSSCOMPILING)
    gtest_discover_tests(kinetra_tests DISCOVERY_MODE PRE_TEST)
else()
    gtest_discover_tests(kinetra_tests)
endif()

# ─── Integration Tests ────────────────────────────────────────────────────────
add_executable(kinetra_integration_tests
    integration/test_planning_scenarios.cpp
)
target_link_libraries(kinetra_integration_tests PRIVATE
    kinetra
    GTest::gtest_main
)
if(CMAKE_CROSSCOMPILING)
    gtest_discover_tests(kinetra_integration_tests DISCOVERY_MODE PRE_TEST)
else()
    gtest_discover_tests(kinetra_integration_tests)
endif()
