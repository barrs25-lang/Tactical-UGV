cmake_minimum_required (VERSION 3.2)

project(goal_generation)

# Curses(Key_event)
set(CMAKE_BUILD_TYPE Debug)
#add_compile_options(-fsanitize=address)
#add_link_options(-fsanitize=address)
if(UNIX)
  find_package(Curses REQUIRED)
  include_directories(${CURSES_INCLUDE_DIR})  
endif()

# Include directory
include_directories(include)
include_directories(../comm_server/src/client/include)

# The main program
add_executable(goal_generation 
	       	src/main/main.cpp 
		src/logging/logging.cpp
		src/goal_generation/goal_generation.cpp
		src/astar/astar.cpp)

target_link_libraries(goal_generation PRIVATE ${CMAKE_DL_LIBS} X11 pthread Xrandr Xi dl ${CURSES_LIBRARIES})

