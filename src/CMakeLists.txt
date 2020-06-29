cmake_minimum_required(VERSION 3.12)
project(sgmwsmcpp VERSION 0.0.1 DESCRIPTION "Sequential graph matching with sequential Monte Carlo")

set(CMAKE_CXX_STANDARD 17)

find_package(Boost 1.67.0 COMPONENTS filesystem REQUIRED)
find_package(Threads REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.cpp" "src/*.h")
add_executable(sgmwsmc ${SOURCES})
target_include_directories(sgmwsmc PRIVATE "vendor")
target_link_libraries(sgmwsmc PRIVATE ${Boost_LIBRARIES} Threads::Threads)

add_executable(test
        "test/test.cpp"
        "test/utils.container.Counter.cpp"
        "test/utils.container.Indexer.cpp"
        "test/utils.type_traits.cpp")

include_directories(sgmwsmc "src/")
include_directories(test "src/" "test/")
