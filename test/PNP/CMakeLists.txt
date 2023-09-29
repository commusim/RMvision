cmake_minimum_required(VERSION 3.8)


set(CMAKE_BUILD_TYPE DEBUG)

project(PNPsolve) 

add_executable(PNP PNP.cpp)


# 配置opecnv
find_package(OpenCV REQUIRED)
target_include_directories(PNP PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(PNP ${OpenCV_LIBS})
 