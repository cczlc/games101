cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(Rasterizer main.cpp src/rasterizer.cpp src/Triangle.cpp)

target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
