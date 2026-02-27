# ──────────────────────────────────────────────────────────────────────────────
# CMake Toolchain File for ARMv7 (arm-linux-gnueabihf)
# ──────────────────────────────────────────────────────────────────────────────
# Usage:
#   cmake -B build-arm \
#     -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/armv7-linux-gnueabihf.cmake \
#     -DKINETRA_USE_FLOAT=ON
# ──────────────────────────────────────────────────────────────────────────────

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armv7l)

# Cross compilers
set(CMAKE_C_COMPILER   arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

# Sysroot (set if needed)
# set(CMAKE_SYSROOT /usr/arm-linux-gnueabihf)
# set(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)

# Search rules: host for programs, target for libraries/includes
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ARMv7 with hardware FPU and NEON
set(CMAKE_C_FLAGS_INIT   "-march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -mthumb")
set(CMAKE_CXX_FLAGS_INIT "-march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -mthumb")

# Optimize for size + speed on embedded
set(CMAKE_C_FLAGS_RELEASE_INIT   "-O2 -DNDEBUG -flto")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-O2 -DNDEBUG -flto")

# Enable Eigen NEON vectorization
add_definitions(-DEIGEN_DONT_VECTORIZE=0)
