cmake_minimum_required(VERSION 3.8)


set(CMAKE_BUILD_TYPE DEBUG)

project(recog)
 
add_executable(recognition recognition.cpp)
 
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    target_include_directories(recognition PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(recognition ${OpenCV_LIBS})
 
endif()