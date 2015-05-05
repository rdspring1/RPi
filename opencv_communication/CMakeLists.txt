cmake_minimum_required (VERSION 2.8)
project (rpi_fusion)

set (CMAKE_CXX_FLAGS			"-Wall -g -O0 -std=c++11")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.49.0 COMPONENTS system thread filesystem REQUIRED) 

find_package( OpenCV REQUIRED )

include_directories (${Boost_INCLUDE_DIRS})
include_directories (${CMAKE_SOURCE_DIR})

file(GLOB PROJECT_SRC_FILES "*.cpp")
add_executable(rpi_fusion ${PROJECT_SRC_FILES})

target_link_libraries (rpi_fusion
	${Boost_SYSTEM_LIBRARY}
	${Boost_THREAD_LIBRARY}
	${Boost_FILESYSTEM_LIBRARY}
	${OpenCV_LIBS}
	pthread
)
