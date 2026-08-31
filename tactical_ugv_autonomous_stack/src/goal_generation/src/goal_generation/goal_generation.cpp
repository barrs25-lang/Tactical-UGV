// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 6th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file communicates with the simulator and path planner,
// computes a partitioning of the voxel map, and computes a goal point.
// End File Decscription ################################################################################

// List include files
#include <goal_generation.h>
#include <my_client.h>

// Boolean to indicate whether or not to exit the thread
bool exit_thread = 0;

// Boolean to indicate thread status
bool simulator_socket_status = 0, octree_status = 0;

// Constant integers representing the map's dimensions
const int grid_x = 100;
const int grid_y = 30;
const int grid_z = 100;

int post_map[grid_x][grid_y][grid_z]; 

// Constructor
GOAL_GENERATION::GOAL_GENERATION()
{
	
	// Define a few variables
	goalPosition = Eigen::MatrixXf::Zero(1,3);
	prev_goalPosition = Eigen::MatrixXf::Zero(1,3);
	prev_goalPosition(0,0) = -1000000;
	prev_goalPosition(0,1) = -1000000;
	prev_goalPosition(0,2) = -1000000;

} // GOAL_GENERATION::GOAL_GENERATION()


// Desctructor
GOAL_GENERATION::~GOAL_GENERATION()
{
	
} // GOAL_GENERATION::~GOAL_GENERATION()


// Local method
// start_octree_thread: this function checks if the octree thread is already running,
// if not, it starts the octree thread.
// INPUTS: pointer to the GOAL_GENERATION object, a string representing the parameter file's name,
// a pointer to a LOGGER Object for logging data
// OUTPUTS: none
void* start_octree_thread(GOAL_GENERATION* goal_generation, const string& param_file_name, LOGGER* log)
{

	if( octree_status != 0 )
	{

		fprintf(stderr,"octree thread already running\n");
		return NULL;

	}
	else
	{

		goal_generation->octree_thread(param_file_name, log);
		return NULL;

	}
	return NULL;

} // void* start_octree_thread(GOAL_GENERATION* goal_generation, const string& param_file_name, LOGGER* log)


// Local method
// start_octree_interface_thread: this function first dereferences the pointer to args,
// which contains pointers to other objects, then initiates the octree thread.
// INPUTS: void pointer to some argument
// OUTPUTS: none
void* start_octree_interface_thread(void *args)
{

	// Define a pointer to an ObjWrapper, which was passed to the function
	ObjWrapper* objContainer = (ObjWrapper* )args;

	// Define a pointer to a GOAL_GENERATION object
	GOAL_GENERATION *goal_generation = objContainer->_goal_generation;

	// Define a string representing the parameter file's name
	string param_file_name = objContainer->_param_file_name;

	// Define a pointer to a LOGGER object
	LOGGER* log = objContainer->_log;

	start_octree_thread(goal_generation, param_file_name, log);

	return NULL;

} // void* start_octree_interface_thread(void *args)


// Local method
// start_simulator_socket_thread: this function checks if the simulator socket thread
// is already running, if not, it starts the simulator socket thread
void* start_simulator_socket_thread(GOAL_GENERATION* goal_generation, LOGGER* log)
{

	// If the simulator socket thread is already running
	if( simulator_socket_status != 0 )
	{

		fprintf(stderr,"simulator socket thread already running\n");
		return NULL;

	} // if( simulator_socket_status != 0 )
	else
	{

		// Start the simulator socket thread
		goal_generation->simulator_socket_thread(log);
		return NULL;

	} // if( simulator_socket_status != 0 )

	return NULL;

} // void* start_simulator_socket_thread(GOAL_GENERATION* goal_generation, LOGGER* log)


// Local method
// start_simulator_socket_interface_thread: this function first dereferences the pointer to args,
// which contains pointers to other objects, then initiates the simulator socket thread.
// INPUTS: void pointer to some argument
// OUTPUTS: none
void* start_simulator_socket_interface_thread(void *args)
{

	// Define a pointer to an ObjWrapper, which was passed to the function
	ObjWrapper* objContainer = (ObjWrapper* )args;

	// Define a pointer to a GOAL_GENERATION object
	GOAL_GENERATION *goal_generation = objContainer->_goal_generation;

	// Define a pointer to a LOGGER object
	LOGGER* log = objContainer->_log;

	start_simulator_socket_thread(goal_generation, log);

	return NULL;

} // void* start_simulator_socket_interface_thread(void *args)


// Member function of GOAL_GENERATION object
// quit_handler: this function is executed whenever specified signals are raised
// INPUTS: integer representing the signal that was raised
// OUTPUTS: none
void GOAL_GENERATION::quit_handler( int sig )
{

	// If the user pressed ctrl+c
  	if (sig == SIGINT)
	{
		printf("\n");
		printf("<QUIT HANDLER> TERMINATING AT USER REQUEST\n");
		printf("\n");
	}

	// If a segmentation fault occured
  	if (sig == SIGSEGV)
	{
		printf("\n");
		printf("<QUIT HANDLER> TERMINATING AFTER SEGMENTATION FAULT\n");
		printf("\n");
	}

	// Signal to all threads that it is time to exit
	exit_thread = 1;

	cout << "<QUIT-HANDLER> Set exit_thread to true." << endl;

	// Pause for 1 second
	usleep(1000000);

	// Exit the program
	exit(0);

} // void GOAL_GENERATION::quit_handler( int sig )


// Member function of GOAL_GENERATION
// start_goal_generation: this function initializes all threads and mutexs for
// thread synchronization, sets up quit handling
// INPUTS: string representing the parameter file's name
// OUTPUTS: None
void GOAL_GENERATION::start_goal_generation(const string& param_file_name)
{

	// Refer signals to different functions such as the quit_handler
	signal(SIGPIPE, quit_handler); 		// Ignore broken pipes (disconnected sockets)
	signal(SIGINT, quit_handler);
	signal(SIGSEGV, quit_handler);
	signal(SIGABRT, quit_handler);

	// Initialize pthread mutex
	pthread_mutex_init(&pose_lock, NULL);
	pthread_mutex_init(&goal_lock, NULL);
	pthread_mutex_init(&map_lock, NULL);

	// Declare a LOGGER object
	LOGGER log;

	// Integer representing the success/failure of a call to write to a log
	int writeStatus;

	// Open a log file with base name "GOAL_GENERATION"
	log.openLogFile("GOAL GENERATION");

	// Open a log file with base name "GOAL_GENERATION_DATA"
	log_data.openLogFile("GOAL_GENERATION_DATA");

	// Declare a string for storing messages to be written to logs or output to terminal
	string message;

	message = "<GOAL GENERATION> Starting goal generation.";
	cout << message << endl;
	writeStatus = log.writeToLog(message);

	// Declare an ObjWrapper object, which encompasses pointers to different data
	ObjWrapper info_wrapper(this, param_file_name, &log);

	// Start the simulator socket thread
	int	result_goal_generation = pthread_create( &goal_generation_tid, NULL, &start_simulator_socket_interface_thread, &info_wrapper);	
	
	// If we failed to start the thread
	if ( result_goal_generation )
	{
		
		// Alert the user
		cout << "<GOAL GENERATION> Unable to start interface thread." << endl;
		log.writeToLog( "<GOAL GENERATION> Unable to start interface thread.");
		
		// Exit the program
		// exit(0);
		std::raise(SIGINT);

	} // if ( result_goal_generation )

	// Start the octree thread
	int	result_octree = pthread_create( &octree_tid, NULL, &start_octree_interface_thread, &info_wrapper);	

	// If we failed to start the thread
	if ( result_octree )
	{

		// Alert the user
		cout << "<GOAL GENERATION> Unable to start octree thread." << endl;
		log.writeToLog( "<GOAL GENERATION> Unable to start octree thread.");
		
		// Exit the program
		// exit(0);
		std::raise(SIGINT);

	} // if ( result_octree )

	message = "<GOAL GENERATION> Threads started.";
	cout << message << endl;
	writeStatus = log.writeToLog(message);

	// While loop to ensure the program does not exit prematurely
	while(!exit_thread)
	{

	} // while(1)

	// Join threads
	pthread_join(goal_generation_tid, NULL);
	pthread_join(octree_tid, NULL);

} // void GOAL_GENERATION::start_goal_generation(const string& param_file_name)


// Member function of GOAL_GENERATION
// simulator_socket_thread: this function sets up connections to the simulator and path planner
// sends data to the simulator and path planner, and receives data from the simulator making it available to other threads
// INPUTS: pointer to a LOGGER object for writing data to logs 
// OUTPUTS: none
void GOAL_GENERATION::simulator_socket_thread(LOGGER* log)
{

	CLIENT client("Parameter_Files/socket_parameters_goal_generation.txt", &exit_thread);

	// Character array storing a message to be sent to servers (so that the server can obtain the client socket's information)
	char goal_generation_message[30] = "Hi, this is goal generation.\n";
	
	// String to store messages to write to logs or output to terminal
	string message;

	message = "<SIM-SOCKET THREAD> Started simulator socket thread.";
	cout << message << endl;
	log->writeToLog(message);

	// Booleans representing whether or not the map and pose have been received from the simulator
	bool mapPass = false, posePass = false;

	// Array of floats storing the goal to send to the simulator or the path planner
	float sim_socket_thread_goal[3];

	// Define an Eigen::MatrixXf which stores the previous sent partitions
	// Used to deduce newBins
	Eigen::MatrixXf prevBinVertices = Eigen::MatrixXf::Zero(1,1);	

	auto current_time_map = std::chrono::high_resolution_clock::now();
	auto current_time_pose = std::chrono::high_resolution_clock::now();
	auto current_time_goal = std::chrono::high_resolution_clock::now();
	auto current_time_path = std::chrono::high_resolution_clock::now();
	auto current_time_constraints = std::chrono::high_resolution_clock::now();
	auto current_time_trajectory = std::chrono::high_resolution_clock::now();
	auto start_time = std::chrono::high_resolution_clock::now();	
	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed_map = end_time - current_time_map;	
	std::chrono::duration<double> timeElapsed_pose = end_time - current_time_pose;	
	std::chrono::duration<double> timeElapsed_goal = end_time - current_time_goal;	
	std::chrono::duration<double> timeElapsed_path = end_time - current_time_path;	
	std::chrono::duration<double> timeElapsed_constraints = end_time - current_time_constraints;	
	std::chrono::duration<double> timeElapsed_trajectory = end_time - current_time_trajectory;	
	std::chrono::duration<double> end_time_stamp = end_time - start_time;	

	float map_freq = 30; // Hz
	float pose_freq = 30; // Hz
	float goal_freq = 1; // Hz
	float path_freq = 2; // Hz
	float constraint_freq = 5; // Hz
	float trajectory_freq = 1; // Hz

	ifstream file("Parameter_Files/communication_parameters.txt");
		
	// String to store file lines
	string file_line;
	stringstream ss;
	
	//read communication parameters
	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> map_freq;	
	cout << "map_freq: " << map_freq << endl;

	//read communication parameters
	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> pose_freq;	
	cout << "pose_freq: " << pose_freq << endl;

	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> goal_freq;	
	cout << "goal_freq: " << goal_freq << endl;

	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> path_freq;	
	cout << "path_freq: " << path_freq << endl;

	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> constraint_freq;	
	cout << "constraint_freq: " << constraint_freq << endl;

	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> trajectory_freq;	
	cout << "trajectory_freq: " << trajectory_freq << endl;				

	float map_time_us = 1000000/(map_freq); // Microseconds	
	float pose_time_us = 1000000/(pose_freq); // Microseconds	
	float goal_time_us = 1000000/goal_freq; // Microseconds	
	float path_time_us = 1000000/path_freq; // Microseconds	
	float constraint_time_us = 1000000/constraint_freq; // Microseconds	
	float trajectory_time_us = 1000000/trajectory_freq; // Microseconds	

	int bytes_read_map = 0, bytes_read_pose = 0, bytes_read_goal = 0, bytes_read_path = 0, bytes_read_constraints = 0, bytes_read_trajectory = 0;
	int counter = 0;	

	// Pose communication vars
	std::vector<string> pose_recv;
	string pose_recv_string;
	char* pose_recv_buffer = new char[1000];

	// Map communication variables
	// char mapBuffer[300000];
	int prev_map[100][30][100]; 
	bool validMap = true;
	std::vector<float> prev_ex_x, prev_ex_y, prev_ex_z;
	std::vector<int> prev_obs;

	// Goal communication vars
	string goal_send;
	char* goal_send_buffer = new char[100];	
	char* cstr_goal = new char[100];

	bool firstLayerSaved = false;

	int _interface_loops = 0;	

	int binVertices_size = 0;

	// While loop which stops when the exit_thread signal is raised
	while(!exit_thread)
	{

		// Get the "end" time
		end_time = std::chrono::high_resolution_clock::now();
		end_time_stamp = end_time - start_time;

		/////////////////////////
		// POSE COMMUNICATIONS //
		/////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_pose).count() >= pose_time_us)
		{	

			// cout << "timeElapsed_pose: " << timeElapsed_pose.count() << endl;			

			// RECEIVE THE POSE //
			if (client.socket_active[2])
			{

				bytes_read_pose = client.sockets[2]->process_receiving(pose_recv_buffer,1000*sizeof(char),1);

					pose_recv_string = pose_recv_buffer;
					// cout << "pose_recv_string: " << endl << pose_recv_string << endl;
					counter = 0;
					pose_recv.clear();

					if (pose_recv_string[0] == 'Q')
					{

						for (int i = 1; i < 1000; i++)
						{
							if (pose_recv_string[i] == '!')
							{
								break;
							}
							else if (pose_recv_string[i] == ',')
							{
								counter++;
								pose_recv.resize(counter);
							}
							else
							{
								pose_recv[counter-1] += pose_recv_string[i];
							}
						}
						
						// Attempt to obtain possession of the goal_lock
						while(pthread_mutex_trylock(&pose_lock))
						{

							// wait for 20 microseconds
							usleep(1);

						} // while(pthread_mutex_trylock(&pose_lock))

						// If the thread successfully obtains possession of the goal_lock, relinquish possession momentarily
						pthread_mutex_unlock(&pose_lock);
						pthread_mutex_lock(&pose_lock);

							pose[0] = boost::lexical_cast<float>(pose_recv[0]);
							pose[1] = boost::lexical_cast<float>(pose_recv[1]);
							pose[2] = boost::lexical_cast<float>(pose_recv[2]);
							pose[3] = boost::lexical_cast<float>(pose_recv[12]);
							pose[4] = boost::lexical_cast<float>(pose_recv[13]);
							pose[5] = boost::lexical_cast<float>(pose_recv[14]);

						pthread_mutex_unlock(&pose_lock);
					
					}
				
				counter = 0;
				posePass = true;

			}			
			
			current_time_pose = std::chrono::high_resolution_clock::now();			

		}

		////////////////////////
		// MAP COMMUNICATIONS //
		////////////////////////

		// If enough time has passed
		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_map).count() >= map_time_us)
		{			

			// cout << "timeElapsed_map: " << timeElapsed_map.count() << endl;			

			// RECEIVE THE MAP //
			if (client.socket_active[0])
			{
				char mapBuffer[300000];
				bytes_read_map = client.sockets[0]->process_receiving(mapBuffer,300000*sizeof(char),0);
			
				// if (bytes_read_map == 300000)
				// {

					// Attempt to obtain possession of the goal_lock
					while(pthread_mutex_trylock(&map_lock))
					{

						// wait for 10 microseconds
						usleep(1);

					} // while(pthread_mutex_trylock(&pose_lock))

					// If the thread successfully obtains possession of the goal_lock, relinquish possession momentarily
					pthread_mutex_unlock(&map_lock);
					pthread_mutex_lock(&map_lock);

						counter = 0;
						validMap = true;

						// cout << "map[0]:" << +mapBuffer[0] << endl;
						// Iterate over lateral direction
						for (int k = 0; k < grid_z; k++) 
						{
							// Iterate over vertical direction
							for (int j = 0; j < grid_y; j++) 
							{
								// Iterate over depth direction
								for (int i = 0; i < grid_x; i++) 
								{

									// Convert RHS from char to int
									// Save map to class member
									map[k][j][i] = +mapBuffer[counter];

									if (i == 0 || k == 0 || i == 99 || k == 99)
									{
										if (!(map[k][j][i] == 2 || map[k][j][i] == 3))	
										{
											validMap = false;
										}
									}

									if (!validMap)
									{
										continue;
									}
									
									// if (post_map[k][j][i] == 4 && map[k][j][i] != 1 && map[k][j][i] != 3 && map[k][j][i] != 4)
									// {
									// 	map[k][j][i] = 4; // Designate the voxel as occupied (this does not happen if the voxel is designated free with a 1)
									// 	ex_x.push_back(0.2*k);
									// 	ex_y.push_back(0.2*i);
									// 	ex_z.push_back(0.2*j);
									// 	obs.push_back(1);							
									// 	explored_points++;
									// }

									counter++;
									
									
								} // for (int i = 0; i < grid_x; i++) 

							} // for (int j = 0; j < grid_y; j++) 

						} // for (int k = 0; k < grid_z; k++) 

						// if (explored_points > 100000 && !firstLayerSaved)
						// {

						// 	ofstream mappy("map0.txt");
						// 	ofstream prev_mappy("map1.txt");
						// 	int alt = floor( pose[2]/voxel_resolution + 0.5 );
						// 	for (int k = 0; k < grid_z; k++) 
						// 	{
						// 		// Iterate over depth direction
						// 		for (int i = 0; i < grid_x-1; i++) 
						// 		{	
						// 			mappy << map[k][alt][i] << ',';
						// 			prev_mappy << prev_map[k][alt][i] << ',';
						// 		}
						// 		mappy << map[k][alt][99] << endl;
						// 		prev_mappy << prev_map[k][alt][99] << endl;

						// 	}
						// 	mappy.close();
						// 	prev_mappy.close();
						// 	firstLayerSaved = true;
						// }


						// Indicate that we have received a map
						// Wait until we have enough points
						// if (!mapPass && ex_x.size() > 100)
						if (!mapPass)
						{
							mapPass = true;
						}

						if (validMap)
						{

							for (int k = 0; k < grid_z; k++) 
							{
								// Iterate over vertical direction
								for (int j = 0; j < grid_y; j++) 
								{
									// Iterate over depth direction
									for (int i = 0; i < grid_x; i++) 
									{
										prev_map[i][j][k] = map[i][j][k];
									}
								}
							}
		
							// prev_ex_x.clear();
							// prev_ex_y.clear();
							// prev_ex_z.clear();
							// prev_obs.clear();

							// prev_ex_x = ex_x;
							// prev_ex_y = ex_y;
							// prev_ex_z = ex_z;
							// prev_obs = obs;

						}
						// else
						// {
						// 	for (int k = 0; k < grid_z; k++) 
						// 	{
						// 		// Iterate over vertical direction
						// 		for (int j = 0; j < grid_y; j++) 
						// 		{
						// 			// Iterate over depth direction
						// 			for (int i = 0; i < grid_x; i++) 
						// 			{
						// 				map[i][j][k] = prev_map[i][j][k];
						// 			}
						// 		}
						// 	}
						// 	ex_x.clear();
						// 	ex_y.clear();
						// 	ex_z.clear();
						// 	obs.clear();

						// 	ex_x = prev_ex_x;
						// 	ex_y = prev_ex_y;
						// 	ex_z = prev_ex_z;
						// 	obs = prev_obs;					
						// 	explored_points = ex_x.size();
						// }

					pthread_mutex_unlock(&map_lock);
				}

			// } // if (mapread == sizeof(mapBuffer))

			current_time_map = std::chrono::high_resolution_clock::now();			

		}

		/////////////////////////
		// GOAL COMMUNICATIONS //
		/////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_goal).count() >= goal_time_us)
		{	

			// cout << "timeElapsed_goal: " << timeElapsed_goal.count() << endl;			

			// SEND THE GOAL //
			if (client.socket_active[5])
			{

				while(pthread_mutex_trylock(&goal_lock))
				{
					// wait for 20 microseconds
					usleep(1);
				} // while(pthread_mutex_trylock(&goal_lock))

				pthread_mutex_unlock(&goal_lock);
				pthread_mutex_lock(&goal_lock);
	
					if (firstPassComplete)
					{
						sim_socket_thread_goal[0] = goalPosition(0,0); sim_socket_thread_goal[1] = goalPosition(0,1); sim_socket_thread_goal[2] = goalPosition(0,2);
					}
					else if (posePass)
					{
						sim_socket_thread_goal[0] = abs(pose[1]); sim_socket_thread_goal[1] = abs(pose[0]); sim_socket_thread_goal[2] = abs(pose[2]);
					}
					else
					{
						sim_socket_thread_goal[0] = 0.0; sim_socket_thread_goal[1] = 0.0; sim_socket_thread_goal[2] = 0.0;
					}

					// Record the previous goal position
					prev_goalPosition = goalPosition;

					binVertices_size = binVertices.rows();

				pthread_mutex_unlock(&goal_lock);

				// cout << "sim_socket_thread_goal: " <<sim_socket_thread_goal[0] << "," << sim_socket_thread_goal[1] << "," << sim_socket_thread_goal[2] << endl;

				goal_send.clear();
				goal_send.append("G");
				goal_send.append(",");
				for (int i = 0; i < 2; i++)
				{
					goal_send.append(boost::lexical_cast<string>(sim_socket_thread_goal[i]));
					goal_send.append(",");
				}
				goal_send.append(boost::lexical_cast<string>(sim_socket_thread_goal[2]));
				goal_send.append("!");

				// cstr_goal = new char[100];
				strcpy(cstr_goal, goal_send.c_str());
				memcpy(goal_send_buffer, cstr_goal, strlen(cstr_goal)+1);

				string temp(goal_send_buffer);

				// cout << "goal send: " << goal_send << endl;

				if (firstPassComplete)
				{
					client.sockets[5]->process_sending(goal_send_buffer,100*sizeof(char));
				}

			}
			
			current_time_goal = std::chrono::high_resolution_clock::now();			

		}

		// Indicate to other thread that all necessary data has been received
		if (!firstPassComplete && mapPass && posePass)
		{

			firstPassComplete = true;

		} // if (!firstPassComplete && mapPass && posePass)

		timeElapsed_map = end_time - current_time_map;
		timeElapsed_pose = end_time - current_time_pose;
		timeElapsed_goal = end_time - current_time_goal;

		// if (!(_interface_loops % 1000))
		// {

		// 	cout << "\033[2J\033[1;1H";
		// 	cout << "Position: " << std::fixed << std::setprecision(5) << -pose[0] << ", " << pose[1] << ", " << -pose[2] << " [m], Heading: " << pose[5] << " [rad]" << endl;
		// 	cout << "Goal:     " << std::fixed << std::setprecision(5) << sim_socket_thread_goal[1] << ", " << sim_socket_thread_goal[0] << ", " << sim_socket_thread_goal[2] << endl;
		// 	cout << "Explored: " << std::fixed << std::setprecision(5) << ((float) explored_points)/300000 << endl;
		// 	cout << "# of Bin: " << std::fixed << std::setprecision(5) << binVertices_size << endl;
		// 	cout << "Timers:   " << "  MAP  " << "  POS  " << "  GOA  " << endl;
		// 	cout << "Time [s]: " << std::fixed << std::setprecision(5) << timeElapsed_map.count() << " " << timeElapsed_pose.count() << " " << timeElapsed_goal.count() << endl;

		// }

		_interface_loops++;

	} // while(!exit_thread)

} // void GOAL_GENERATION::simulator_socket_thread(LOGGER* log)

// Local method
// get_sign: this function gets the sign of the argument d and returns s, which is +/- 1 unless d is identically 0
// INPUTS: two template parameters, the address of the variable to store the sign of d, and d, whose parity is in question
template <class T> T get_sign(T& s, T d)
{

	if (d == 0)
	{
		
		//
		s = 0;

	} // if (d == 0)
	else
	{

		// Check the sign of d using signbit, which returns true if d is negative, 0 otherwise
		if (signbit(d))
		{

			s = -1;

		} // if (signbit(d))
		else
		{

			s = 1;

		} // if (signbit(d))

	} // if (d == 0)

	return s;

} // template <class T> T get_sign(T& s, T d)



bool GOAL_GENERATION::ray_tracing(Eigen::MatrixXf* position, int (&local_map)[100][30][100])
{

	// cout << "(*position): " << (*position) << endl;

	// Record the position of the camera
	int x = floor( (*position)(0,1)/voxel_resolution + 0.5 );
	int y = floor( (*position)(0,2)/voxel_resolution + 0.5 );
	int z = floor( (*position)(0,0)/voxel_resolution + 0.5 );

	// cout << "ray trace x,y,z: " << x  << ", " << y  << ", " << z << endl;

	// Record the goal point
	Eigen::MatrixXf local_goal = Eigen::MatrixXf::Zero(1,3);

	local_goal(0,0) = floor(octree_goalPosition(0,0)/voxel_resolution + 0.5);
	local_goal(0,1) = floor(octree_goalPosition(0,2)/voxel_resolution + 0.5);
	local_goal(0,2) = floor(octree_goalPosition(0,1)/voxel_resolution + 0.5);

	// cout << "local_goal x,y,z: " << local_goal(0,0) << ", " << local_goal(0,1)  << ", " << local_goal(0,2) << endl;

	// Record the difference in position of the camera and sample point
	int dxrt = (int) (local_goal(0,0) - x);
	int dyrt = (int) (local_goal(0,1) - y);
	int dzrt = (int) (local_goal(0,2) - z);

	int n, sx, sy, sz, exy, exz, ezy, ax, ay, az, bx, by, bz;

	// cout << "d: " << dxrt << ", " << dyrt << ", " << dzrt << endl;
	
	// Retrieve the sign of the change in each direction, store in sx/y/z
	get_sign(sx,dxrt); get_sign(sy,dyrt); get_sign(sz,dzrt);
	
	// magnitude of change in each direction
	ax = abs(dxrt); ay = abs(dyrt); az = abs(dzrt);
	
	bx = 2*ax; by = 2*ay; bz = 2*az;
	
	exy = ay-ax; exz = az-ax; ezy = ay-az; 
	
	n = 2+ax+ay+az;
	// n = (int) floor(range/0.2 + 0.5);

	// Iterative voxel ray tracing See Ch V.3 of Graphics Gems 4
	while( n-- && x < 100 && y < 30 && z < 100)
	{
		// cout << "n: " << n << endl;
		// cout << "x,y,z: " << x << ", " << y << ", " << z << endl;

		// If we have hit an obstacle, return false, goal is not in LOS
		if (local_map[(z)][(y)][(x)] == 3 && x != local_goal(0,0) && y != local_goal(0,1) && z != local_goal(0,2))
		{
			// cout << "local_map[" << x << "][" << y << "][" << z << "]: " << local_map[(x)][(y)][(z)] << endl;
			// cout << "here" << endl;
			return false;

		} // if (x == local_goal(0,0) && y == local_goal(0,1) && z == local_goal(0,2))
		// If ray tracing goes beyond the maps boundaries, stop
		else if (x + sx < 0 || y + sy < 0 || z + sz < 0 || x + sx >= 100 || y + sy >= 30 || z + sz >= 100)
		{
			// cout << "here 3" << endl;
			break;

		} // if (x + sx < 0 || y + sy < 0 || z + sz < 0 || x + sx >= 100 || y + sy >= 30 || z + sz >= 100)

		else if (abs(x - local_goal(0,0)) <= 0.1 && abs(y - local_goal(0,1)) <=0.1 && abs(z - local_goal(0,2)) <= 0.1)
		{

			// cout << "here 1" << endl;
			return true;

		} // if (x == local_goal(0,0) && y == local_goal(0,1) && z == local_goal(0,2))
		else
		{

			// Determine which face of a voxel the 3D ray exits from (this allows to determine which voxel will be pierced by the ray next)
			if (exy < 0)
			{

				if (exz < 0)
				{

					x += sx;
					exy += by;
					exz += bz;

				} // if (exz < 0)
				else
				{

					z += sz;
					exz -= bx; 
					ezy += by;

				} // if (exz < 0)

			} // if (exy < 0)
			else
			{

				if (ezy < 0)
				{
					
					z += sz;
					exz -= bx; 
					ezy += by;	

				} // if (ezy < 0)
				else
				{
					
					y += sy;
					exy -= bx;
					ezy -= bz;

				} // if (ezy < 0)

			} // if (exy < 0)

		} // if (x == local_goal(0,0) && y == local_goal(0,1) && z == local_goal(0,2))

	} // while( n-- && x < 100 && y < 30 && z < 100)

	return false;

} // bool ray_tracing(Eigen::MatrixXf, int (&local_map)[100][30][100])


// Member function of GOAL_GENERATION class
// octree_thread: this function is responsible for formatting inputs and processig outputs
// of a customized octree algorithm used for generating a goal point for the path planner
// INPUTS: string representing the parameter file's name, a pointer to a LOGGER object
void GOAL_GENERATION::octree_thread(const string& param_file_name, LOGGER* log)
{

	// Indicate to upstream code that the octree thread is running
	octree_status = 1;

	// Integers representing the number of explored points, number of 
	int explored_points;

	// Integer to store the number of partitions
	int numBins;

	// Integer representing the number of loops through the octree thread
	int octree_loops = 0; 

	// Define matrices to store initial partition vertices and quadrotor position
	Eigen::MatrixXf r = Eigen::MatrixXf::Zero(3, 2), quad_position = Eigen::MatrixXf::Zero(1,3), quad_attitude = Eigen::MatrixXf::Zero(1,3), exM;
	Eigen::MatrixXf localGoal = Eigen::MatrixXf::Zero(1,3);

	// Float representing the vehicle's position
	float quad_heading = 0.0;

	// Float representing the half field of view
	float halfFOV = 0.0;

	// Second vertex of initial partition is set so that the initial partition covers the entire map (in meters)
	r(0, 1) = 20.0;
	r(1, 1) = 20.0;
	r(2, 1) = 6.0;

	// Declare Eigen::MatrixXf to store the vertices of the partitions deduced by the goal generation algorithm
	Eigen::MatrixXf octree_binVertices;

	// Declare input file stream for parameter file
	ifstream param_file;

	// Declare string to store messages or data for logging or output to terminal
	string message;
	message = "<OCTREE THREAD> Opening parameter file: " + param_file_name;
	cout << message << endl;
	int writeStatus = log->writeToLog(message);
	
	// Open the parameter file
	param_file.open("Parameter_Files/" + param_file_name + ".txt");

	// If the parameter file failed to open
	if (!param_file) 
	{

		// Alert the user
		message = "<OCTREE THREAD> Unable to open file: Parameter_Files/" + param_file_name + ".txt";
		cout << message << endl;
		writeStatus = log->writeToLog(message);
		// exit(1); // terminate with error
		std::raise(SIGINT);

	} // if (!param_file) 
	else
	{

		message = "<OCTREE THREAD> Opened file: Parameter_Files/" + param_file_name + ".txt";
		cout << message << endl;
		writeStatus = log->writeToLog(message);

	} // if (!param_file) 

	// Declare a string to store the file lines
	string file_line;
	stringstream ss;

	// Declare floats to store parameters for the goal generation algorithm
	float pet, pet_orig, minEdgeSize, maxEdgeSize, muProx;

	// Declare integers to store parameters for the goal generation algorithm
	int binExploredCapacity, binExploredCapacity_orig, binOccupiedCapacity;

	// Read the percent explored threshold (\mu_1)
	do{ss.clear(); getline(param_file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> pet;

	pet_orig = pet;

	// Read the maximum number of explored voxels in a partition (\mu_3)
	do{ss.clear(); getline(param_file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> binExploredCapacity;

	binExploredCapacity_orig = binExploredCapacity;

	// Read the maximum number of occupied voxels in a partition (\mu_4)
	do{ss.clear(); getline(param_file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> binOccupiedCapacity;

	// Read the minimum edge size of a partition (\mu_4)
	do{ss.clear(); getline(param_file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> minEdgeSize;

	// Read the minimum edge size of a partition (\mu_4)
	do{ss.clear(); getline(param_file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> maxEdgeSize;

	// Read the convex weighting parameter for computing the goal (\mu_2)
	do{ss.clear(); getline(param_file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> muProx;

	cout << "binExploredCapacity: " << binExploredCapacity << endl;
	cout << "binOccupiedCapacity: " << binOccupiedCapacity << endl;
	cout << "minEdgeSize: " << minEdgeSize << endl;
	cout << "maxEdgeSize: " << maxEdgeSize << endl;
	cout << "muProx: " << muProx << endl;

	message = "<OCTREE THREAD> Finished reading file: Parameter_Files/" + param_file_name + ".txt";
	cout << message << endl;
	writeStatus = log->writeToLog(message);

	// Close the parameter file
	param_file.close();
	message = "<OCTREE THREAD> Closed file: Parameter_Files/" + param_file_name + ".txt";
	cout << message << endl;
	writeStatus = log->writeToLog(message);	

	// Open the parameter file
	param_file.open("../../trajectory_planner/build/Parameter_Files/System_params.txt");

	// If the parameter file failed to open
	if (!param_file) 
	{

		// Alert the user
		message = "<OCTREE THREAD> Unable to open file: ../../trajectory_planner/build/Parameter_Files/System_params.txt";
		cout << message << endl;
		writeStatus = log->writeToLog(message);
		// exit(1); // terminate with error
		std::raise(SIGINT);		

	} // if (!param_file) 
	else
	{

		message = "<OCTREE THREAD> Opened file: ../../trajectory_planner/build/Parameter_Files/System_params.txt";
		cout << message << endl;
		writeStatus = log->writeToLog(message);

	} // if (!param_file) 

	// Get the next line of the parameter file
	ss.clear();
	getline(param_file, file_line);
	ss.str(file_line);

	// Attempt to find the string
	std::size_t found = file_line.find("// Half of camera");

	// While the camera half field of view parameter has not been found
	while(found == std::string::npos)
	{

		// get the next line
		ss.clear();
		getline(param_file, file_line);
		ss.str(file_line);

		// Attempt to find the string
		found = file_line.find("// Half of camera");

	} // while(found != std::string::npos)

	// Get the next line
	ss.clear();	getline(param_file, file_line);	ss.str(file_line);

	// Read the halfFOV [degrees]
	ss >> halfFOV;

	// Convert to radians
	halfFOV = halfFOV*M_PI/180;

	param_file.close();
	message = "<OCTREE THREAD> Closed file: ../../trajectory_planner/build/Parameter_Files/System_params.txt";
	cout << message << endl;
	writeStatus = log->writeToLog(message);	

	// Forces the octree thread to wait for a map and pose
	cout << "<OCTREE THREAD> Waiting for first pass." << endl;
	cout << message << endl;
	writeStatus = log->writeToLog(message);		

	while(!firstPassComplete)
	{

		// Pause for 100 microseconds
		usleep(100);
	
	} // while(!firstPassComplete)

	message = "<OCTREE THREAD> Octree thread begins.";
	cout << message << endl;
	writeStatus = log->writeToLog(message);		

	// Integer representing the number of explored points in the previous map
	int prevExploredPointsCount = 0;

	// Boolean representing whether or not an octree has been created
	bool firstOctree = true;

	// Eigen::Vector2f storing the normalized vector representing the vehicle's orientation
	Eigen::Vector2f vehicle_orientation = Eigen::Vector2f::Zero(2);

	// Eigen::Vector2f storing the normalized vector representing the goal's orientation w.r.t the vehicle expressed in the inertial frame
	Eigen::Vector2f goal_orientation = Eigen::Vector2f::Zero(2);

	// Float capturing the angle between vehicle_orientation and goal_orientation
	float angle_bt_heading_and_goal = 0.0;

	// Boolean indicating that the goal is both in the field of view and line of sight of the vehicle's vision-based navigation system
	bool goalInFOV_LOS = false;

	// Boolean indicating that the goal is coincident with an obstacle
	bool goalIsObstacle = false;

	Eigen::MatrixXf goal_history = Eigen::MatrixXf::Zero(1,3);
	goal_history(0,0) = 1.0;
	goal_history(0,1) = 1.0;
	goal_history(0,2) = 1.0;
	Eigen::VectorXi goal_fov_los_history = Eigen::VectorXi::Zero(1);
	bool foundPreviousGoal = true;

	auto current_time = std::chrono::high_resolution_clock::now();
	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed = end_time - current_time;

	int local_map[grid_x][grid_y][grid_z];

	// While loop which exits when the exit_thread signal is raised
	while(!exit_thread)
	{
		// Attempt to obtain possession of the pose_lock
		while(pthread_mutex_trylock(&pose_lock))
		{

			// Pause for 20 microseconds
			usleep(1);

		} // while(pthread_mutex_trylock(&pose_lock))

		// If the thread obtained possession of the pose_lock, relinquish possession momentarily
		pthread_mutex_unlock(&pose_lock);
		pthread_mutex_lock(&pose_lock);

			// Record the quadrotor pose into a variable local to this function
			quad_position(0,0) = abs(pose[0]); 
			quad_position(0,1) = abs(pose[1]);
			quad_position(0,2) = abs(pose[2]);
			quad_attitude(0,0) = pose[4];
			quad_attitude(0,1) = pose[3];
			// quad_attitude(0,2) = M_PI/2+pose[5];
			quad_attitude(0,2) = -pose[5];
			// quad_heading = M_PI/2+pose[5];
			quad_heading = -pose[5];

			// cout << "quad_heading: " << quad_heading << endl;

		// Relinquish possession of the pose lock
		pthread_mutex_unlock(&pose_lock);

		// Compute the vector representing the camera's focal axis
		vehicle_orientation(0) = cos(quad_heading);
		vehicle_orientation(1) = sin(quad_heading);

		// Attempt to obtain possession of the goal_lock
		while(pthread_mutex_trylock(&map_lock))
		{

			// Pause for 10 microseconds
			usleep(1);

		} // while(pthread_mutex_trylock(&map_lock))

		// If the thread obtained possession of the goal_lock, relinquish possession momentarily
		pthread_mutex_unlock(&map_lock);
		pthread_mutex_lock(&map_lock);

			// Iterate over lateral direction
			for (int k = 0; k < grid_z; k++) 
			{
				// Iterate over vertical direction
				for (int j = 0; j < grid_y; j++) 
				{
					// Iterate over depth direction
					for (int i = 0; i < grid_x; i++) 
					{

						// Convert RHS from char to int
						// Save map to class member
						local_map[i][j][k] = map[i][j][k];		

					}
				}
			}		

			// Relinquish possession of map_lock
			pthread_mutex_unlock(&map_lock);			

			// Clear vectors storing the x,y,z coordinates of explored voxels
			// and clear the vector which indicates that the explored voxel is
			// occupied or not
			ex_x.clear();
			ex_y.clear();
			ex_z.clear();
			obs.clear();

			// Iterate over lateral direction
			for (int k = 0; k < grid_z; k++) 
			{
				// Iterate over vertical direction
				for (int j = 0; j < grid_y; j++) 
				{
					// Iterate over depth direction
					for (int i = 0; i < grid_x; i++) 
					{

						// If the current voxel is explored and unoccupied
						if (local_map[k][j][i] == 1)
						{

							ex_x.push_back(0.2*k);
							ex_y.push_back(0.2*i);
							ex_z.push_back(0.2*j);
							obs.push_back(0);

						} // if (map[i][j][k] == 1)
						else if (local_map[k][j][i] == 3)
						{

							ex_x.push_back(0.2*k);
							ex_y.push_back(0.2*i);
							ex_z.push_back(0.2*j);
							obs.push_back(1);

						} // if (map[i][j][k] == 3)


					}
				}
			}										

			// If the latest map contains more explored voxels than the previous map
			if (prevExploredPointsCount <= ex_x.size())
			{			

				// Indicate how many explored points there are
				explored_points = ex_x.size();

				// Define an Eigen::MatrixXf to store the coordinates of explored points and their occupancy status
				exM = Eigen::MatrixXf::Zero(explored_points, 4);

				// Iterate over the number of explored points
				for (int i = 0; i < explored_points; i++)
				{

					// Record the explored points' coordinates and occupancy status to a variable local to this function
					exM(i, 0) = ex_x[i];
					exM(i, 1) = ex_y[i];
					exM(i, 2) = ex_z[i];
					exM(i, 3) = obs[i];

				} // for (int i = 0; i < explored_points; i++)

				// Record the number of explored points
				prevExploredPointsCount = explored_points;

			} // if (prevExploredPointsCount <= ex_x.size())

			// Pause for 100 microseconds
			usleep(1000);
 	
			// If we have computed an octree
			if (!firstOctree)
			{

				// Integer for an iterator used for moving the goal arbitrarily far in an
				// arbitrary direction in search for a goal that is not in an occupied voxel
				int ii = 1;

				// Integers representing temporary moves from the goal position
				int mx, my, mz;

				// Define the temporary moves, starting at the goal position
				mx = (int) floor((octree_goalPosition)(0,0)/voxel_resolution + 0.5);
				my = (int) floor((octree_goalPosition)(0,2)/voxel_resolution + 0.5);
				mz = (int) floor((octree_goalPosition)(0,1)/voxel_resolution + 0.5);

				// cout << "quad_position: " << quad_position << endl;

				goalIsObstacle = false;

				// If the goal coincides with an obstacle
				if (local_map[mx][my][mz] == 3)
				{
					cout << "goal coincides with an obstacle" << endl;
					cout << "octree_goalPosition: " << octree_goalPosition(0,0) << ", " << octree_goalPosition(0,2) << ", " << octree_goalPosition(0,1) << endl;
					cout << "mx, my, mz: " << mx << ", " << my << ", " << mz << endl;
					ofstream mappy;
					mappy.open("out_map.txt");
					for (int j = 0; j < 100; j++)
					{
						for (int i = 0; i < 99; i++)
						{
							mappy << local_map[i][my][j] << ",";
						}
							mappy << local_map[99][my][j] << endl;
					}
					// exit(0);
					goalIsObstacle = true;

				} // if (local_map[mx][my][mz] == 3)

			} // if (!firstOctree)

		// If there exists some explored points
		if (explored_points > 0)
		{

				// ******************************************************** //
				// This block determines if the goal is in FOV & LOS of the //
				// vehicle's navigation system								//
				// ******************************************************** //

				// If we have computed a goal
				if (!firstOctree)
				{

					// Compute the goal orientation with respect to the vehicle
					goal_orientation(0) = ( octree_goalPosition(0,0) - quad_position(0,0) ); 
					goal_orientation(1) = ( octree_goalPosition(0,1) - quad_position(0,1) ); 

					// cout << "octree_goalPosition: " << octree_goalPosition << endl;
					// cout << "quad_position: " << quad_position << endl;
					// cout << "goal_orientation: " << goal_orientation << endl;
					// cout << "goal_orientation.norm(): " << goal_orientation.norm() << endl;
					// If the goal is within the camera's range
					if (goal_orientation.norm() < 0.5*range)
					{

						// cout << "goal in range" << endl;
						// cout << "octree_goalPosition: " << octree_goalPosition << endl;
						// cout << "quad_position: " << quad_position << endl;
						// cout << "goal_orientation: " << goal_orientation << endl;

						// Normalize the goal orientation
						goal_orientation = goal_orientation / goal_orientation.norm();

						// Check if goal is in FOV 
						angle_bt_heading_and_goal = acos(vehicle_orientation.dot(goal_orientation));

						// cout << "angle_bt_heading_and_goal: " << angle_bt_heading_and_goal << endl;
						// cout << "halfFOV: " << halfFOV<< endl;
						// If the angle between the focal axis and ray connecting the vehicle to the goal is small enough
						if (angle_bt_heading_and_goal < halfFOV)
						{

							// Perform ray tracing from quadrotor position to the goal point,
							goalInFOV_LOS = ray_tracing(&quad_position, local_map);
							// cout << "goalInFOV_LOS: " << goalInFOV_LOS << endl;
						} // if (abs(angle_bt_heading_and_goal) < halfFOV)
						else
						{
							
							// Indicate that the goal is not in FOV
							goalInFOV_LOS = false;

						} // if (abs(angle_bt_heading_and_goal) < halfFOV)

					}
					else
					{
						goalInFOV_LOS = false;
					}

					// for (int i = 0; i < goal_history.rows(); i++)
					// {
					// 	if ( ( ( octree_goalPosition - goal_history.row(i) ).norm() <= 0.01 ) )
					// 	{
					// 		goal_fov_los_history.conservativeResize(goal_history.size() + 1);
					// 		goal_fov_los_history(i) = goalInFOV_LOS;
					// 		break;
					// 	}
					// }

				} // if (firstOctree)

				// ************************************************************** //
				// This block first determines if a new goal needs to be computed //
				// and creates an Octree for that purpose						  //
				// ************************************************************** //

				// If the distance between the quadrotor and goal position is small enough or if the first Octree has never ran or if we have not had to revise the goal
				bool temp = ( (octree_goalPosition) - (quad_position).block(0, 0, 1, 3) ).norm() <= 1.0;

				end_time = std::chrono::high_resolution_clock::now();
				timeElapsed = end_time - current_time;
				// if ( ( ( (goalPosition) - (quad_position).block(0, 0, 1, 3) ).norm() <= 1.0 || firstOctree || goalInFOV_LOS || goalIsObstacle))
				if ( ( ( ( (octree_goalPosition) - (quad_position).block(0, 0, 1, 3) ).norm() <= 1.0 || goalInFOV_LOS) && timeElapsed.count() > 20.0) || firstOctree || goalIsObstacle || timeElapsed.count() > 100.0)
				// if ( ( ( ( (octree_goalPosition) - (quad_position).block(0, 0, 1, 3) ).norm() <= 1.0 || goalInFOV_LOS) && timeElapsed.count() > 40.0) || firstOctree || goalIsObstacle)
				{
					cout << "goal gen conditions: " << temp << "," << firstOctree << "," << goalInFOV_LOS << "," << goalIsObstacle << endl;
					cout << "goal_orientation: " << goal_orientation.transpose() << endl;
					// Create an octree and compute a goal position
					Octree ocTree(explored_points, binExploredCapacity, pet, binOccupiedCapacity, minEdgeSize, maxEdgeSize, muProx, &r, "normal", &exM, &quad_position, &octree_binVertices, &numBins, &octree_goalPosition, local_map, firstOctree, &goal_history, &foundPreviousGoal, &quad_attitude);
					
					for (int k = 0; k < grid_z; k++) 
					{
						for (int j = 0; j < grid_y; j++) 
						{

							for (int i = 0; i < grid_x; i++) 
							{

								post_map[i][j][k] = local_map[i][j][k];

							}

						}

					}

					// Indicate that the first octree has been created
					firstOctree = false;

					// Set an Eigen::MatrixXf to store the vertices of the partitions that were just generated
					binVertices = Eigen::MatrixXf::Zero(octree_binVertices.rows(),6);

					// Set the matrix to the partition vertices
					binVertices = octree_binVertices;

					while(pthread_mutex_trylock(&goal_lock))
					{
						usleep(1);
					} // while(pthread_mutex_trylock(&goal_lock))
					pthread_mutex_unlock(&goal_lock);
					pthread_mutex_lock(&goal_lock);
			
						// Set the goal position
						goalPosition = octree_goalPosition;

					// Relinquish possession of goal_lock
					pthread_mutex_unlock(&goal_lock);

					cout << "\r" << "Goal: " << octree_goalPosition(0,0) << ", " << octree_goalPosition(0,1) << ", " << octree_goalPosition(0,2);
					log_data.writeToLog(to_string(quad_position(0,0)) + "," + to_string(quad_position(0,1)) + "," + to_string(quad_position(0,2)) + "," + to_string(octree_goalPosition(0,0)) + "," + to_string(octree_goalPosition(0,1)) + "," + to_string(octree_goalPosition(0,2)) );

					current_time = std::chrono::high_resolution_clock::now();
					
				} // if ( ( (goalPosition) - (quad_position).block(0, 0, 1, 3) ).norm() <= 0.8 || firstOctree || !foundGoodGoal)



		} // if (explored_points > 0)

	} // while(!exit_thread)

	log_data.close();

} // void GOAL_GENERATION::octree_thread(const string& param_file_name, LOGGER* log)