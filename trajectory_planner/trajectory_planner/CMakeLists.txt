cmake_minimum_required (VERSION 3.2)

project(fmpc_uncut)

set(CMAKE_BUILD_TYPE Debug)
#add_compile_options(-fsanitize=address)
#add_link_options(-fsanitize=address)
#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# Include directory
include_directories(include)
include_directories(../comm_server/src/client/include)

# The main program
add_executable(fmpc_uncut 
	       	src/main/main.cpp 
		src/logging/logging.cpp
		src/f_mpc_uncut/f_mpc_uncut.cpp
		src/line_search/line_search.cpp)

target_link_libraries(fmpc_uncut PRIVATE ${CMAKE_DL_LIBS} X11 pthread Xrandr Xi dl)

