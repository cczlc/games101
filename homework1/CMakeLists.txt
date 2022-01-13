cmake_minimum_required (VERSION 2.8.11)

project (Transformation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(Eigen3 REQUIRED)

include_directories(EIGEN3_INCLUDE_DIR)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable (Transformation main.cpp src/transform.cpp)
