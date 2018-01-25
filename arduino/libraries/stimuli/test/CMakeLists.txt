# copied from github.com/tom-f-oconnell/gtest_template @ {VERSION}
# TODO autofill version w/ script to copy this
cmake_minimum_required(VERSION 2.8)
# TODO version #?
project()

# TODO look into ctest / enable_testing(), add_test(...) etc

# TODO why no prefix required here?
include(gtest.cmake)

# GoogleTest seems to only be available starting in cmake 3.9
# so I'm just going to distribute the relevant .cmake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
include(GoogleTest)
include(CTest)

# TODO so disabled by default?
SET(COVERAGE OFF CACHE BOOL "Coverage")

# TODO TODO how does this example work without the main.cpp calling 
# run_all_${PROJECT_NAME} and other init? why are there examples of both?

# TODO is it not good to use the project name this way for some reason?

# TODO maybe i want to put all of this stuff in the src directory?
# I'm worried that will screw up the Arduino stuff?

# TODO are these ${CMAKE_CURRENT_SOURCE_DIR} prefixes necessary, or are paths
# relative to that directory by default?
# There are cases both for and against using wildcard (and similar?) to round up
# sources. I might be leaning more towards using it.
add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/${PROJECT_NAME}.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/${PROJECT_NAME}.cpp
    test_${PROJECT_NAME}.cpp)

# to trigger unit-testing specific pre-processor conditionals
# (e.g. to excise hardware dependent components)
target_compile_definitions(${PROJECT_NAME} PRIVATE UNIT_TESTING)

# because gtest requires the pthreads library, and I suppose this installs that
find_package(Threads)

# TODO so is this all of the linking to gtest required to get the default main?
# the docs made it sound like it would have to be intentional?
# (and this is required anyway? any kind of settings that would lead to no
# default main?
target_link_libraries(${PROJECT_NAME} gtest ${CMAKE_THREAD_LIBS_INIT})

# TODO actually test the windows part?
if (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /std:c++17 /W4 /WX)

    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od")

    # VS 2017 removed tr1
    add_definitions(-DGTEST_HAS_TR1_TUPLE=0)

    # TODO no coverage checking in windows? why? coverall just not work here?
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -std=c++14 -g -Wall -Werror
        -Wextra -Wpedantic -Wconversion -Wswitch-default -Wswitch-enum
        -Wunreachable-code -Wwrite-strings -Wcast-align -Wshadow -Wundef)

    if (COVERAGE)
        target_compile_options(${PROJECT_NAME} PRIVATE --coverage)
        target_link_libraries(${PROJECT_NAME} PRIVATE --coverage)
    endif()
endif()

enable_testing()

# In cmake >= 3.10, there is a slightly improved gtest_discover_tests
# but this version isn't (as) easily installable in Ubuntu 16.04
# TODO running ./test -> "No tests were found!!!" how to debug?
#gtest_discover_tests(${PROJECT_NAME})
gtest_add_tests(${PROJECT_NAME} "" AUTO)

# see below if you want to run tests in same `make` command, without `make test`
# https://stackoverflow.com/questions/32901679/unit-testing-as-part-of-the-build
