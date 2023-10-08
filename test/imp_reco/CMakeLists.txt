cmake_minimum_required(VERSION 3.8)


set(CMAKE_BUILD_TYPE DEBUG)

project(recog)
add_executable(imp_reco imp_reco.cpp)
 
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    target_include_directories(imp_reco PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(imp_reco ${OpenCV_LIBS})
 
endif()