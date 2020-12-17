cmake_minimum_required(VERSION 3.13)
project(main)

set(CMAKE_CXX_STANDARD 17)
find_package( OpenCV REQUIRED )
find_package(Python3)
set(SOURCE_FILES main.cpp detection.cpp)
add_library(parsers SHARED ${SOURCE_FILES})
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
include_directories(${OpenCV_INCLUDE_DIRS} ${PYTHON3_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})


