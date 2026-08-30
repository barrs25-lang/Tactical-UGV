cmake_minimum_required (VERSION 3.2)

project(path_planner)

set(CMAKE_BUILD_TYPE Debug)
#add_compile_options(-fsanitize=address)
#add_link_options(-fsanitize=address)
# Include directory
include_directories(include)
include_directories(../comm_server/src/client/include)

# The main program
add_executable(path_planner 
	       	src/main/main.cpp 
		src/logging/logging.cpp
		src/LPAstar/LPAstar.cpp)

target_link_libraries(path_planner PRIVATE ${CMAKE_DL_LIBS} X11 pthread Xrandr Xi dl)

