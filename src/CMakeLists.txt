cmake_minimum_required(VERSION 3.12)
project(sgmwsmcpp VERSION 0.0.1 DESCRIPTION "Sequential graph matching with sequential Monte Carlo")

set(CMAKE_CXX_STANDARD 14)
set(default_build_type "Release")

find_package(Boost 1.67.0 COMPONENTS filesystem REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.cpp" "src/*.h")

add_executable(sgmwsmc ${SOURCES})
target_include_directories(sgmwsmc SYSTEM PRIVATE ${Boost_INCLUDE_DIRS} "vendor")
target_link_libraries(sgmwsmc PRIVATE ${Boost_LIBRARIES} Eigen3::Eigen Threads::Threads)

set($ENV{CPLUS_INCLUDE_PATH} "${Boost_INCLUDE_DIRS}:vendor")

target_compile_options(sgmwsmc PRIVATE -Wall)

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "3.7")
        message("Detected Clang >=3.7, using -Wpessimizing-move and -Wredundant-move")
        target_compile_options(sgmwsmc PRIVATE -Wpessimizing-move -Wredundant-move)
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "9.0.0")
        message("Detected GCC >=9, using -Wpessimizing-move and -Wredundant-move")
        target_compile_options(sgmwsmc PRIVATE -Wpessimizing-move -Wredundant-move)
    endif()
endif()

set(TEST_SOURCES ${SOURCES})
list(REMOVE_ITEM TEST_SOURCES "src/main.cpp")

add_executable(test
        ${TEST_SOURCES}
        "test/test.cpp"
#        "test/utils.container.Counter.cpp"
#        "test/utils.container.Indexer.cpp"
        "test/utils.opt.LBFGSMinimizer.cpp"
        "test/utils.container.vector_list.cpp")

target_include_directories(test PRIVATE ${Boost_INCLUDE_DIRS} "vendor")
target_link_libraries(test PRIVATE ${Boost_LIBRARIES} Eigen3::Eigen Threads::Threads)

include_directories(sgmwsmc "src/")
include_directories(test "src/" "test/")
