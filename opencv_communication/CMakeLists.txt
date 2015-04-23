cmake_minimum_required (VERSION 2.8)
project (opencv_communication)

set (CMAKE_CXX_FLAGS			"-Wall -g -O0 -std=c++11")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.49.0 COMPONENTS system thread REQUIRED) 

find_package( OpenCV REQUIRED )

include_directories (${Boost_INCLUDE_DIRS})
include_directories (${CMAKE_SOURCE_DIR})

add_executable(rpi_fusion manager.cpp)

target_link_libraries (rpi_fusion
	${Boost_SYSTEM_LIBRARY}
	${Boost_THREAD_LIBRARY}
	${OpenCV_LIBS}
	pthread
)
