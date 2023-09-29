cmake_minimum_required(VERSION 3.8)

project(demo)
 
set(SOURCES test.cpp)
 
 
add_executable(Main ${SOURCES})
 
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    target_include_directories(Main PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(Main ${OpenCV_LIBS})
 
endif()