cmake_minimum_required(VERSION 3.12)
project(sgmwsmcpp VERSION 0.0.1 DESCRIPTION "Sequential graph matching with sequential Monte Carlo")

find_package(Boost 1.67.0 )

set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.cpp" "src/*.cc")
add_executable(sgmwsmc ${SOURCES})

add_executable(test "test/test.cpp" "test/utils.container.Counter.cpp")

include_directories(sgmwsmc "src/")
include_directories(test "src/" "test/")
