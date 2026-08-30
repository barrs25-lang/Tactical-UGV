cmake_minimum_required (VERSION 3.2)

project(client)

set(CMAKE_BUILD_TYPE Debug)
include_directories(./include)

# The main program
add_executable( client 
	     		src/my_client.cpp)

target_link_libraries(client PRIVATE ${CMAKE_DL_LIBS} X11 Xrandr Xi dl)

