cmake_minimum_required(VERSION 3.8)

set(CMAKE_BUILD_TYPE DEBUG)

project(posesolve)

add_executable(slovepose slovepose.cpp)

find_package(OpenCV REQUIRED)
target_include_directories(slovepose PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(slovepose ${OpenCV_LIBS})
