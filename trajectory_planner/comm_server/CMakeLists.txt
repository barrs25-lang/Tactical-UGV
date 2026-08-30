cmake_minimum_required (VERSION 3.2)

project(server)

set(CMAKE_BUILD_TYPE Debug)
include_directories(./include)

#add_compile_options(-fsanitize=address)
#add_link_options(-fsanitize=address)

# The main program
add_executable( server 
	     		src/my_server.cpp)

target_link_libraries(server PRIVATE ${CMAKE_DL_LIBS} X11 Xrandr Xi dl)

