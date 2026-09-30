cmake_minimum_required(VERSION 3.20)

project(LedasGeometry)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTS "Build unit tests" ON)

add_library(project_warnings INTERFACE)

file(GLOB_RECURSE GEOMETRY_HEADERS CONFIGURE_DEPENDS "include/*.h")
file(GLOB_RECURSE GEOMETRY_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

add_library(geometry
    ${GEOMETRY_HEADERS}
    ${GEOMETRY_SOURCES}
)

target_include_directories(geometry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(geometry
    PRIVATE
        project_warnings
)

add_executable(ledas_demo main.cpp)
target_link_libraries(ledas_demo
    PRIVATE
        geometry
        project_warnings
)

if(BUILD_TESTS)
    include(CTest)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
        add_subdirectory(tests)
    endif()
endif()
