cmake_minimum_required (VERSION 2.8)
project (Adhoc)

set (CMAKE_CXX_FLAGS			"-Wall -g -O0 -std=c++11")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.45.0 COMPONENTS system REQUIRED) 

include_directories (${Boost_INCLUDE_DIRS})
include_directories (${CMAKE_SOURCE_DIR})

add_executable(Adhoc wireless.cpp)

target_link_libraries (Adhoc
	${Boost_SYSTEM_LIBRARY}
	pthread
)
