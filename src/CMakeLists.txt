aux_source_directory(SRC_LIST .)

add_executable(slovepose ${SRC_LIST})

find_package(OpenCV REQUIRED)
target_include_directories(slovepose PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(slovepose ${OpenCV_LIBS})
