cmake_minimum_required(VERSION 3.20.0)
project(dynamic_game_trajectory_planner VERSION 0.1.0 LANGUAGES C CXX)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Choose precision at configure time: 16, 32, or 64
set(REAL_BITS "64" CACHE STRING "Precision in bits for real_t (16|32|64)")
set_property(CACHE REAL_BITS PROPERTY STRINGS 16 32 64)

option(ENABLE_POSIT "Enable posit datatype via the Universal library" OFF)
option(ENABLE_QUANTIZATION "Enable quantization feature" TRUE)

set(POSIT_ES_ALLOWED 0 1 2 3)
set(POSIT_ES "2" CACHE STRING "Posit exponent size es (posit<REAL_BITS, POSIT_ES>)")
set_property(CACHE POSIT_ES PROPERTY STRINGS ${POSIT_ES_ALLOWED})

if(ENABLE_POSIT)
    if(NOT POSIT_ES IN_LIST POSIT_ES_ALLOWED)
        message(FATAL_ERROR "POSIT_ES must be one of: ${POSIT_ES_ALLOWED} (got '${POSIT_ES}')")
    endif()
    add_compile_definitions(USE_POSIT=1 POSIT_ES=${POSIT_ES})
    message(STATUS "Datatype        : posit<${REAL_BITS}, ${POSIT_ES}>")
else()
    add_compile_definitions(USE_POSIT=0)
    if(REAL_BITS STREQUAL "16")
        message(STATUS "Datatype        : half (IEEE binary16)")
    elseif(REAL_BITS STREQUAL "32")
        message(STATUS "Datatype        : float (fp32)")
    else()
        message(STATUS "Datatype        : double (fp64)")
    endif()
endif()

if(ENABLE_QUANTIZATION)
    message(STATUS "Quantization is ENABLED")
    add_compile_definitions(ENABLE_QUANTIZATION=1) # Define macro for quantization
else()
    message(STATUS "Quantization is DISABLED")
    add_compile_definitions(ENABLE_QUANTIZATION=0) # Define macro to disable quantization
endif()

message(STATUS "REAL_BITS = ${REAL_BITS}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_definitions(REAL_BITS=${REAL_BITS})

include_directories(include Recorder4Cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/universal/include/sw)

option(USE_RECORDER "Use recorder" OFF)

if (USE_RECORDER)
  MESSAGE(STATUS "USE_RECORDER=ON")
  add_definitions(-DUSE_RECORDER)
  set(recorder_file ${recorder_file} Recorder4Cpp/recorder.h
  	Recorder4Cpp/recorder.cpp)
endif()

set(source_files
    src/main.cpp
    src/dynamic_game_planner.cpp
    src/utils.cpp
    src/vehicle_state.cpp
)

add_executable(dynamic_game_trajectory_planner ${source_files} ${recorder_file})

target_compile_options(dynamic_game_trajectory_planner PRIVATE
    -O3
    -fno-fast-math -fno-associative-math -fno-reciprocal-math -fno-finite-math-only
    -ffp-contract=off)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
    COMMAND ${CMAKE_CXX_COMPILER}
            -S -g -fverbose-asm -O3 -std=c++${CMAKE_CXX_STANDARD}
            -DREAL_BITS=${REAL_BITS}
            -DENABLE_QUANTIZATION=$<IF:$<BOOL:${ENABLE_QUANTIZATION}>,1,0>
            -DUSE_POSIT=$<IF:$<BOOL:${ENABLE_POSIT}>,1,0>
            -DPOSIT_ES=${POSIT_ES}
            -I${CMAKE_CURRENT_SOURCE_DIR}/include
            -I${CMAKE_CURRENT_SOURCE_DIR}/Recorder4Cpp
            -I${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/universal/include/sw
            -o ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
            ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamic_game_planner.cpp
    DEPENDS src/dynamic_game_planner.cpp
    COMMENT "Generating annotated assembly for dynamic_game_planner.cpp"
)

add_custom_target(generate_cpp_asm
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
    COMMENT "Target to generate assembly code for dynamic_game_planner.cpp"
)
