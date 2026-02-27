# ─── Kinetra WebAssembly Build ─────────────────────────────────────────────
# Compiles the core planning library to WebAssembly via Emscripten.
#
# Usage:
#   source /tmp/emsdk/emsdk_env.sh
#   mkdir -p build-wasm && cd build-wasm
#   emcmake cmake ../wasm -DCMAKE_BUILD_TYPE=Release
#   emmake make -j$(sysctl -n hw.ncpu)
#   cp kinetra_wasm.{js,wasm} ../docs/
# ──────────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.20)
project(kinetra_wasm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Fetch Eigen (header-only) ────────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    ON
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

# ── Library root ─────────────────────────────────────────────────────────
set(KINETRA_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# ── Source files (exclude scenario_loader — uses std::filesystem) ────────
set(KINETRA_WASM_SOURCES
    bindings.cpp
    # Core
    ${KINETRA_ROOT}/src/core/trajectory.cpp
    ${KINETRA_ROOT}/src/core/result.cpp
    # Spaces
    ${KINETRA_ROOT}/src/spaces/se2.cpp
    ${KINETRA_ROOT}/src/spaces/dubins.cpp
    ${KINETRA_ROOT}/src/spaces/real_vector.cpp
    # Robot models (stubs — header-only)
    ${KINETRA_ROOT}/src/robots/differential_drive.cpp
    ${KINETRA_ROOT}/src/robots/ackermann.cpp
    ${KINETRA_ROOT}/src/robots/omnidirectional.cpp
    # Collision
    ${KINETRA_ROOT}/src/collision/occupancy_grid.cpp
    ${KINETRA_ROOT}/src/collision/polygon_obstacles.cpp
    # Solvers
    ${KINETRA_ROOT}/src/solvers/lqr.cpp
    ${KINETRA_ROOT}/src/solvers/qp_admm.cpp
    ${KINETRA_ROOT}/src/solvers/sqp.cpp
    # Optimization
    ${KINETRA_ROOT}/src/optimization/nlp_problem.cpp
    # Planners
    ${KINETRA_ROOT}/src/planners/rrt_star.cpp
    ${KINETRA_ROOT}/src/planners/stomp.cpp
    ${KINETRA_ROOT}/src/planners/lattice.cpp
    ${KINETRA_ROOT}/src/planners/ilqr.cpp
    # IO (json_export only — no filesystem)
    ${KINETRA_ROOT}/src/io/json_export.cpp
)

add_executable(kinetra_wasm ${KINETRA_WASM_SOURCES})

target_include_directories(kinetra_wasm PRIVATE
    ${KINETRA_ROOT}/include
)

target_link_libraries(kinetra_wasm PRIVATE Eigen3::Eigen)

# ── Emscripten-specific link flags ───────────────────────────────────────
target_link_options(kinetra_wasm PRIVATE
    --bind                          # Embind
    -sENVIRONMENT=web
    -sMODULARIZE=1
    -sEXPORT_NAME='KinetraModule'
    -sALLOW_MEMORY_GROWTH=1
    -sINITIAL_MEMORY=33554432       # 32 MB
    -sSTACK_SIZE=2097152             # 2 MB stack
    -sNO_EXIT_RUNTIME=1
    -sDISABLE_EXCEPTION_CATCHING=0  # Keep exceptions for error reporting
)

# ── Optimisation ─────────────────────────────────────────────────────────
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(kinetra_wasm PRIVATE -O3 -DNDEBUG)
    target_link_options(kinetra_wasm PRIVATE -O3)
else()
    target_compile_options(kinetra_wasm PRIVATE -O1 -g)
    target_link_options(kinetra_wasm PRIVATE -O1 -g)
endif()
