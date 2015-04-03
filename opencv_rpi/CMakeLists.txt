cmake_minimum_required (VERSION 2.8)
project (detect)

set (CMAKE_CXX_FLAGS			"-Wall -g -O3 -std=c++11")

find_package( OpenCV REQUIRED )

include_directories (${CMAKE_SOURCE_DIR})

add_executable(detect detect.cpp)

target_link_libraries (detect
	${OpenCV_LIBS}
)
