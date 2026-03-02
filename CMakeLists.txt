cmake_minimum_required(VERSION 3.10)
project(MVDR_Engine LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Global NEON Optimization Flags for Pi Zero 2 W
add_compile_options(-O3 -mcpu=cortex-a53 -ffast-math -ftree-vectorize -funsafe-math-optimizations)

# Find ALSA
find_package(ALSA REQUIRED)

# Include the current directory so main.cpp can find pffft.h
include_directories(${ALSA_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

# Compile both our engine and the raw PFFFT math together
add_executable(mvdr_engine main.cpp pffft.c)

# Force the SIMD flag just in case the compiler misses the architecture hint
target_compile_definitions(mvdr_engine PRIVATE PFFFT_ENABLE_SIMD)

# Link ALSA, Math, and Threads
target_link_libraries(mvdr_engine ${ALSA_LIBRARIES} m pthread)
