cmake_minimum_required(VERSION 3.18)
project(AUVPhysics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

add_library(AUVPhysics SHARED
    src/HydrodynamicsSolver.cpp
    src/AUVState.cpp
    src/SonarSimulator.cpp
)

target_include_directories(AUVPhysics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(WIN32)
    target_compile_definitions(AUVPhysics PRIVATE AUVPHYSICS_EXPORTS)
endif()

set_target_properties(AUVPhysics PROPERTIES
    OUTPUT_NAME "AUVPhysics"
    PREFIX ""
)
