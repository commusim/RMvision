cmake_minimum_required(VERSION 3.8)


set(CMAKE_BUILD_TYPE DEBUG)

project(cam_clibrate) 

add_executable(clibrate clibrate.cpp)

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    target_include_directories(clibrate PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(clibrate ${OpenCV_LIBS})
 
endif()