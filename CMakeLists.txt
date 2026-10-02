cmake_minimum_required(VERSION 3.20)
project(LedasGeometry LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(geometry
    src/aabb.cpp
    src/maths.cpp
    src/vector3d.cpp
)
target_include_directories(geometry PUBLIC include)

add_executable(ledas_demo main.cpp)
target_link_libraries(ledas_demo PRIVATE geometry)

option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    include(CTest)
    add_subdirectory(tests)
endif()