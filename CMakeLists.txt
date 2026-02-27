cmake_minimum_required(VERSION 3.20)

project(Kinetra
    VERSION 0.1.0
    DESCRIPTION "World-class standalone trajectory planning library for robotics"
    HOMEPAGE_URL "https://github.com/JoenHune/Kinetra"
    LANGUAGES CXX
)

# ─── C++20 Standard ──────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ─── Build Options ────────────────────────────────────────────────────────────
option(KINETRA_BUILD_TESTS      "Build unit tests"                   ON)
option(KINETRA_BUILD_BENCHMARKS "Build performance benchmarks"       ON)
option(KINETRA_BUILD_EXAMPLES   "Build example programs"             ON)
option(KINETRA_BUILD_DOCS       "Build documentation"                OFF)
option(KINETRA_USE_FLOAT        "Use float instead of double"        OFF)
option(KINETRA_ENABLE_ASAN      "Enable AddressSanitizer"            OFF)
option(KINETRA_ENABLE_TSAN      "Enable ThreadSanitizer"             OFF)

# ─── Scalar Type Configuration ────────────────────────────────────────────────
if(KINETRA_USE_FLOAT)
    add_compile_definitions(KINETRA_SCALAR_FLOAT)
    message(STATUS "Kinetra: Using float precision (optimized for ARMv7 NEON)")
else()
    message(STATUS "Kinetra: Using double precision")
endif()

# ─── Compiler Warnings (target-specific, so FetchContent deps are unaffected) ─
set(KINETRA_WARNINGS
    -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor
    -Wold-style-cast -Wcast-align -Wunused -Woverloaded-virtual
    -Wconversion -Wsign-conversion -Wnull-dereference
    -Wformat=2 -Wimplicit-fallthrough
)

# ─── Third Party Dependencies ────────────────────────────────────────────────
include(FetchContent)

# Eigen (header-only linear algebra — the ONLY required dependency)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(eigen)

# ─── Kinetra Core Library ────────────────────────────────────────────────────
add_library(kinetra STATIC
    # Core
    src/core/trajectory.cpp
    src/core/result.cpp

    # State spaces
    src/spaces/se2.cpp
    src/spaces/real_vector.cpp
    src/spaces/dubins.cpp

    # Robot models
    src/robots/differential_drive.cpp
    src/robots/ackermann.cpp
    src/robots/omnidirectional.cpp

    # Collision
    src/collision/occupancy_grid.cpp
    src/collision/polygon_obstacles.cpp

    # Solvers
    src/solvers/qp_admm.cpp
    src/solvers/lqr.cpp

    # Planners
    src/planners/rrt_star.cpp
    src/planners/stomp.cpp
    src/planners/ilqr.cpp

    # IO
    src/io/json_export.cpp
    src/io/scenario_loader.cpp
)

target_include_directories(kinetra
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(kinetra PUBLIC Eigen3::Eigen)

target_compile_features(kinetra PUBLIC cxx_std_20)

# Apply warnings only to our code
target_compile_options(kinetra PRIVATE ${KINETRA_WARNINGS})

# Sanitizers (applied globally only when explicitly requested)
if(KINETRA_ENABLE_ASAN)
    target_compile_options(kinetra PUBLIC -fsanitize=address -fno-omit-frame-pointer)
    target_link_options(kinetra PUBLIC -fsanitize=address)
endif()

if(KINETRA_ENABLE_TSAN)
    target_compile_options(kinetra PUBLIC -fsanitize=thread)
    target_link_options(kinetra PUBLIC -fsanitize=thread)
endif()

set_target_properties(kinetra PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

# ─── Alias for clean usage ───────────────────────────────────────────────────
add_library(Kinetra::kinetra ALIAS kinetra)

# ─── Tests ────────────────────────────────────────────────────────────────────
if(KINETRA_BUILD_TESTS)
    enable_testing()
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.15.2
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(googletest)
    add_subdirectory(tests)
endif()

# ─── Benchmarks ──────────────────────────────────────────────────────────────
if(KINETRA_BUILD_BENCHMARKS)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG        v1.9.1
        GIT_SHALLOW    TRUE
    )
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(benchmark)
    add_subdirectory(benchmarks)
endif()

# ─── Examples ─────────────────────────────────────────────────────────────────
if(KINETRA_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ─── Install ──────────────────────────────────────────────────────────────────
include(GNUInstallDirs)
install(TARGETS kinetra EXPORT KinetraTargets
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/kinetra DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT KinetraTargets
    FILE KinetraTargets.cmake
    NAMESPACE Kinetra::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Kinetra
)
