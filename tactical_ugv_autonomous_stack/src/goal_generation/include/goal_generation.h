// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 6th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This header file defines the GOAL_GENERATION class
// End File Decscription ################################################################################


#ifndef GOAL_GENERATION_H
#define GOAL_GENERATION_H


// List include files
#include <time.h>
#include <chrono>
#include <sys/time.h>
#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <sched.h>
#include <numeric>
#include <pthread.h> // This uses POSIX Threads
#include <read_files.h>
#include <logging.h>
#include <octree.h>
#include <dataSend.h>
#include <astar.h>
#include <csignal>

// List namespaces
using namespace std;


// Define GOAL_GENERATION class
class GOAL_GENERATION 
{

// Publically available members
public:

	// Constructor
	GOAL_GENERATION();

	// Destructor
	~GOAL_GENERATION();

	// Float array storing the vehicle's pose
	float pose[12];	

	// Float representing the resolution of the voxel map in meters
	float voxel_resolution = 0.2;

	// Float representing the range of the simulated camera
	const float range = 7.0; // [meters]

	// Integer arrays storing voxel cell transitions and their reversals
	int dx[DIRECTIONS] = {1, 0, -1, 0, 1,  1, -1, -1, 1, 0, -1,  0, 1,  1, -1, -1,  1,  0, -1,  0,  1,  1, -1, -1};
	int dy[DIRECTIONS] = {0, 0, 0,  0, 0,  0,  0,  0, 1, 1,  1,  1, 1,  1,  1,  1, -1, -1, -1, -1, -1, -1, -1, -1};
	int dz[DIRECTIONS] = {0, 1, 0, -1, 1, -1,  1, -1, 0, 1,  0, -1, 1, -1,  1, -1,  0,  1,  0, -1,  1, -1,  1, -1};
	int flip[DIRECTIONS] = {2, 3, 0, 1, 7, 6, 5, 4, 18, 19, 16, 17, 23, 22, 21, 20, 10, 11, 8, 9, 15, 14, 13, 12};
  	
	// Wrapper containing pointers to sockets
  	DataSendWrapper socketWrapper_quit;
  	
  	// DataSend sockets for communication with the simulator and path planner
  	DataSend in_map_sim_socket, in_pose_sim_socket, out_goal_sim_socket, out_bin_sim_socket, out_goal_path_socket;
  	
	// 3D array of integers representing the voxel map
	int map[100][30][100]; 

  	// Prototype functions //
	void start_goal_generation(const string& param_file_name);
	void simulator_socket_thread(LOGGER* log);
	void octree_thread(const string& param_file_name, LOGGER* log);
	bool ray_tracing(Eigen::MatrixXf* position, int (&local_map)[100][30][100]);

	LOGGER log_data;

// Private members
private:

	// Declare Eigen::MatrixXf storing the goal position
	Eigen::MatrixXf goalPosition;

	// Declare Eigen::MatrixXf to store the goal position deduced by the goal generation algorithm
	Eigen::MatrixXf octree_goalPosition = Eigen::MatrixXf::Zero(1,3);	

	// Declare Eigen::MatrixXf storing the previous goal position
	Eigen::MatrixXf prev_goalPosition;

	// Declare Eigen::MatrixXf storing the vertices of partitions
	Eigen::MatrixXf binVertices;

	// Declare vector of floats storing the x,y,z coordinates of explored voxels
	std::vector<float> ex_x, ex_y, ex_z;

	// Declare vector of floats storing the occupancy status of explored voxels
	// 0 indicates free, 1 indicates occupied
	std::vector<int> obs;
	bool firstPassComplete = false;

	// Pthread thread IDs
	pthread_t goal_generation_tid;
	pthread_t octree_tid;

	// Pthread mutexs for sychronization
	pthread_mutex_t goal_lock;
	pthread_mutex_t pose_lock;
	pthread_mutex_t map_lock;


	// Prototype functions //
	static void quit_handler( int sig );

}; // class GOAL_GENERATION 

// Object wrapper, used to tell threads where to look for information that they may need to read or write from.
class ObjWrapper 
{

	// The class ObjWrapper has these members, which are points to different objects.
	public: GOAL_GENERATION* _goal_generation;
	public: const string& _param_file_name;
	public: LOGGER* _log; 

	// Class constructor, when instantiating an object of type ObjWrapper, pass the address of the necessary variables.
	public: ObjWrapper(GOAL_GENERATION* goal_generation, const string& param_file_name, LOGGER* log) : _goal_generation(goal_generation), _param_file_name(param_file_name), _log(log)
	{

	}

}; // class ObjWrapper 

#endif