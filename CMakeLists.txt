cmake_minimum_required(VERSION 3.17)
project(Detection)

set(CMAKE_CXX_STANDARD 17)
find_package( OpenCV REQUIRED )
set(SOURCE_FILES main.cpp detection.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )