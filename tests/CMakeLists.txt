include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
)

FetchContent_MakeAvailable(googletest)

add_executable(geometry_tests
    test_maths.cpp
)

target_link_libraries(geometry_tests
    PRIVATE
        geometry
        project_warnings
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(geometry_tests)
