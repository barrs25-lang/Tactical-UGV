cmake_minimum_required(VERSION 3.2)

project(constraint_generation)

set(CMAKE_BUILD_TYPE Debug)
#add_compile_options(-fsanitize=address)
#add_link_options(-fsanitize=address)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# Include directory
include_directories(include)
include_directories(../comm_server/src/client/include)
include_directories(
    ${PROJECT_SOURCE_DIR}/include/sdpa-7.3.8
)

# The main program
add_executable(constraint_generation 
	       	src/main/main.cpp 
		src/logging/logging.cpp
		src/constraint/constraint.cpp)

target_link_libraries(constraint_generation PRIVATE ${CMAKE_DL_LIBS} X11 pthread lapack cblas f77blas dmumps_seq sdpa Xrandr Xi dl)

