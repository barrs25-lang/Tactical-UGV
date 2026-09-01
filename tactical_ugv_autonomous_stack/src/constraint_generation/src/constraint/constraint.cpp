// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 14th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file contains the collision avoidance thread,
// and socket interface thread
// End File Decscription ################################################################################


// List include files
#include "constraint.h"
#include <my_client.h>

// Constructor
CONSTRAINT::CONSTRAINT()
{

} // CONSTRAINT::CONSTRAINT()

// Destructor
CONSTRAINT::~CONSTRAINT()
{
	
	// Close LOGGER objects
	log.close();
	log1.close();

} // CONSTRAINT::~CONSTRAINT()

// Timing variables
auto current_time = std::chrono::high_resolution_clock::now();
auto end_time = std::chrono::high_resolution_clock::now();
std::chrono::duration<double> timeElapsed = end_time - current_time;

// Boolean flags to indicate that it is time to exit the program (exit_thread) and whether or not
// to output all variables to terminal (debugging)
bool exit_thread = false, debugging = false;

// Speed diagnostics - floats capture the average loop speed
float average_collision_avoidance_loop_speed, average_flightstackinterface_loop_speed;

// Integers capturing the number of loops through threads
int flightstackinterface_loops, collision_avoidance_loops;

// Local method
// quit_handler: handles program behavior when a signal is raised
// INPUTS: integer capturing the raised signal
// OUTPUTS: none
void quit_handler( int sig )
{

	// If the user pressed ctrl+c
  	if (sig == SIGINT)
	{
		
		printf("\n");
		printf(">> TERMINATING AT USER REQUEST\n");
		printf("\n");

	} // if (sig == SIGINT)


	// If a socket disconnected
  	if (sig == SIGPIPE)
	{
		
		printf("\n");
		printf(">> TERMINATING AFTER BROKEN PIPE\n");
		printf("\n");

	} // if (sig == SIGPIPE)


	// If a segmentation fault occured
  	if (sig == SIGSEGV)
	{
		
		printf("\n");
		printf(">> TERMINATING AFTER SEGMENTATION FAULT\n");
		printf("\n");

	} // if (sig == SIGSEGV)

	// Alert threads to stop
	exit_thread = 1;

	// end program here
	exit(0);

} // void quit_handler( int sig )


// Local method
// start_flightstack_interface: function first dereferences pointers
// to objects, and passes them to the flight stack interface thread
// INPUTS: void pointer to some object
// OUTPUTS: void pointer to NULL
void* start_flightstack_interface(void *args)
{
	
	// Pointer to ObjWrapper
	ObjWrapper *ObjWrapper_ = (ObjWrapper* )args;

	// Pointer to CONSTRAINT object
	CONSTRAINT *constraint_ = ObjWrapper_->_constraint;

	// Pointer to System object
	System *quad_ = ObjWrapper_->_quad;

	// Start the thread, pass the system object
	constraint_->start_flightstack_interface_thread(quad_);

	return NULL;

} // void* start_flightstack_interface(void *args)


// Local method
// start_collision_avoidance: function first dereferences pointers
// to objects, and passes them to the collision avoidance thread
// INPUTS: void pointer to some object
// OUTPUTS: void pointer to NULL
void* start_collision_avoidance(void *args)
{
	
	// Pointer to ObjWrapper
	ObjWrapper *ObjWrapper_ = (ObjWrapper* )args;

	// Pointer to CONSTRAINT object
	CONSTRAINT *constraint_ = ObjWrapper_->_constraint;

	// Pointer to System object
	System *quad_ = ObjWrapper_->_quad;

	// Start the thread, pass the system object
	constraint_->start_collision_avoidance_thread(quad_);
	return NULL;

} // void* start_collision_avoidance(void *args)


// Member function of CONSTRAINT object
// init_constraint_generator(): function which initializes variables that are used by the constraint generating algorithm
// INPUTS: bool modifying_constraints: boolean to indicate if constraints are currently being modfied so that external functions can properly use them
// OUTPUTS: Updated constraints in _quadrotor
bool CONSTRAINT::init_constraint_generator(struct System* quad)
{

	// Boolean flag indicating when the constraint generator is initialized
	bool initialized = false;

	// Read from constraint parameter file
	string param_filename = "Parameter_Files/Constraint_params.txt";

	//open a parameter file to read in
	ifstream file(param_filename);

	// String storing the file lines
	string file_line;
	stringstream ss;

	// read the soft constraint offset
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _soft_offset;

	// read the boolean indicating whether or not to log data
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _loggingEnabled;

	// read the maximum number of iterations for SDPA
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _SDPA_params[0];//maxIter;

	// read the epsilonStar for SDPA
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _SDPA_params[1];//epsilonStar;

	// read the epsilonDash for SDPA
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _SDPA_params[2];//epsilonDash;

	// read the lambdaStar for SDPA
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _SDPA_params[3];//lambdaStar;

	// read the omegaStar for SDPA
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _SDPA_params[4];//omegaStar;

	// read the betaStar for SDPA
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _SDPA_params[5];//betaStar;

	// read the betaBar for SDPA
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _SDPA_params[6];//betaBar;

	// read the gammaStar for SDPA
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _SDPA_params[7];//gammaStar;

	// read the parameterType for SDPA
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _parameterType;

	//read map file name
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _map_filename;

	//read bounding map file name)
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _bound_filename;

	//read bounding map file name)
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _initial_center_filename;

	//read epsilon for MULTI_ELLIPSOID
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _epsilon;

	//read number of sectors for fastApproximateConvexHull
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _num_xy_sectors;

	//read number of sectors for fastApproximateConvexHull
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _num_z_sectors;

	//read number of sectors for fastApproximateConvexHull
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _bCenters_source;

	//read number of sectors for fastApproximateConvexHull
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _bCenters_samples;

	//read number of sectors for fastApproximateConvexHull
	do { ss.clear(); getline(file, file_line); ss.str(file_line); } while (file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> _bar_v;

	//close the parameter file
	file.close();

	//std::cout << "Debugging? (0 for no, 1 for yes)" << std::endl;
	//std::cin >> debugging;

	// Compute the total number of sectors
	total_sectors = _num_xy_sectors*_num_z_sectors/2;

	std::cout << "total_sectors: " << total_sectors << std::endl;

	// If the debugging flag is true
	if (debugging)
	{

		std::cout << "_nopts: " << _nopts << std::endl;
		std::cout << "_nangles: " << _nangles << std::endl;
		std::cout << "_soft_offset: " << _soft_offset << std::endl;
		std::cout << "_method: " << _method << std::endl;
		std::cout << "_loggingEnabled: " << _loggingEnabled << std::endl;
		
		// Iterate over the number of SDPA parameters
		for (unsigned short int i = 0; i < 8; i++)
		{

			std::cout << "_SDPA_params[" << i << "]: " << _SDPA_params[i] << std::endl;

		} // for (int i = 0; i < 8; i++)

		std::cout << "_parameterType: " << _parameterType << std::endl;
		std::cout << "_map_filename: " << _map_filename << std::endl;
		std::cout << "_bound_filename: " << _bound_filename << std::endl;
		std::cout << "_initial_center_filename: " << _initial_center_filename << std::endl;
		std::cout << "_epsilon: " << _epsilon << std::endl;
		std::cout << "_num_xy_sectors: " << _num_xy_sectors << std::endl;
		std::cout << "_num_z_sectors: " << _num_z_sectors << std::endl;
		std::cout << "_bCenters_source: " << _bCenters_source << std::endl;

	} // if (debugging)


	// Open log files to output data
	if (log.is_open() != 1)
	{

		log.open("Diagnostic_Logs/Dgnstc_Ellipsoid_Log.txt");

	} // if (log.is_open() != 1)

	if (log1.is_open() != 1)
	{
		
		log1.open("Diagnostic_Logs/Dgnstc_Close_Point_Log.txt");

	} // if (log1.is_open() != 1)

	if (log2.is_open() != 1)
	{
		
		log2.open("Diagnostic_Logs/Dgnstc_RASP_Log.txt");

	} // if (log2.is_open() != 1)

	if (log3.is_open() != 1)
	{
		
		log3.open("../../Diagnostic_Logs/Dgnstc_COLLISION_AVOIDANCE_Log.txt");

	} // if (log3.is_open() != 1)

	// Open the map's file
	inFile1.open(_map_filename);

	// If the file did not open
	if (!inFile1)
	{
		
		std::cout << ">> Unable to open " << _map_filename << std::endl;
		
		// End program indicate that the constraint generator did not initialize
		return !initialized;

	} // if (!inFile1)

	// Open the bounding point cloud filename
	inFile2.open(_bound_filename);

	// If the file did not open
	if (!inFile2)
	{
		
		std::cout << ">> Unable to open " << _bound_filename << std::endl;

		// End program indicate that the constraint generator did not initialize
		return !initialized;

	} // if (!inFile2)

	// Clear the point cloud vectors
	y.clear();
	yy.clear();

	numberConstraints += extra_variables;

	std::cout << ">> Initialized constraint generator" << std::endl;
	std::cout << ">> Initializing threads" << std::endl;

	// Initialize pthread mutex for thread sync
	pthread_mutex_init(&map_lock, NULL);
	pthread_mutex_init(&pose_lock, NULL);
	pthread_mutex_init(&constraint_lock, NULL);
	pthread_mutex_init(&initial_lock, NULL);
	pthread_mutex_init(&ellipsoid_lock, NULL);

	// Setup the signal handler, see quit_handler() function above.
	signal(SIGINT,quit_handler);
	signal(SIGPIPE,quit_handler);
	signal(SIGSEGV,quit_handler);
	signal(SIGTERM,quit_handler);

	// Obtain possession of the initial_lock to stall the collision avoidance algorithm
	// until all necessary data is obtained (position, velocity, and map)
	pthread_mutex_lock(&initial_lock);

	// Initialize ObjWrapper object to store pointers to some objects
	ObjWrapper systemObj(&collision_avoidance_tid, quad, this);

	// Start the flightstack interface thread
	int result_flightstack = pthread_create( &flightstack_tid , NULL, &start_flightstack_interface, &systemObj);

	// Start the collision avoidance thread
	int result_collision_avoidance = pthread_create( &collision_avoidance_tid , NULL, &start_collision_avoidance, &systemObj);

	// If the thread fails to start, throw an error
	if ( result_flightstack ) throw result_flightstack;
	if ( result_collision_avoidance ) throw result_collision_avoidance;

	std::cout << ">> Threads started." << std::endl;

	// While loop to ensure the program does not exit
	while(!exit_thread)
	{
		
		// ensures the program does not exit

	} // while(!exit_thread)

	// Join threads 
	pthread_join(flightstack_tid, NULL);
	pthread_join(collision_avoidance_tid, NULL);

	return 1;

} // bool CONSTRAINT::init_constraint_generator(struct System* quad)


// Member function of CONSTRAINT
// start_collision_avoidance_thread: starts the collision avoidance thread if it has not started
// already.
// INPUTS: pointer to a System structure
// OUTPUTS: none
void CONSTRAINT::start_collision_avoidance_thread(struct System* quad)
{

	// If the collision avoidance algorithm has started alread
	if( collision_avoidance_status != 0 )
	{

		// Alert the user
		fprintf(stderr,">> Collision Avoidance thread already running\n");
		return;

	} // if( collision_avoidance_status != 0 )
	else
	{

		// Start the collision avoidance thread
		collision_avoidance_thread(quad);
		return;

	} // if( collision_avoidance_status != 0 )

} // void CONSTRAINT::start_collision_avoidance_thread(struct System* quad)


// Member function of CONSTRAINT
// start_flightstack_interface_thread: starts the flightstack interface thread if it has not started
// already.
// INPUTS: pointer to a System structure
// OUTPUTS: none
void CONSTRAINT::start_flightstack_interface_thread(struct System* quad)
{

	// If the flightstack interface has started alread
	if( flightstack_status != 0 )
	{

		// Alert the user
		fprintf(stderr,"flightsack interface thread already running\n");
		return;

	} // if( flightstack_status != 0 )
	else
	{

		// Start the collision avoidance thread
		flightstack_interface_thread(quad);
		return;

	} // if( flightstack_status != 0 )
}


// Member function of CONSTRAINT class
// collision_avoidance_thread: this thread runs the collision avoidance algorithm
// INPUTS: pointer to a System object
// OUTPUTS: none
void CONSTRAINT::collision_avoidance_thread(struct System* quad)
{

	collision_avoidance_status = 0;

	// Attempt to obtain possession of the initial_lock
	while(pthread_mutex_trylock(&initial_lock) != 0)
	{
		
		// Pause 1000 microseconds
		usleep(1000);

	} // while(pthread_mutex_trylock(&initial_lock) != 0)
	
	// Relinquish possession of the lock
	pthread_mutex_unlock(&initial_lock);

	// usleep(100000000);

	int runcount = 0;

	// While loop that exits when exit_thread signal is raised
	while(!exit_thread)
	{ 

		// Pause 1000 microseconds
		// usleep(1400000);
		usleep(10000);

		// Setup the problem, read in data
		setup_problem(quad);
		
		// Run SDPA to find the parent ellipsoid, and find centers for child ellipsoids
		determineCenters(quad);

		// Run SDPA to find child ellipsoids
		quadraticDiscrimination(quad);


		// Determine the "close points", points that are close to the ellipsoids
		closePoints();

		// Determine a convex hull
		fastApproximateConvexHull();

		cout << "Runs: " << ++runcount << endl;

	} // while(!exit_thread)

	return;

} // void CONSTRAINT::collision_avoidance_thread(struct System* quad)


// Member function of CONSTRAINT class
// setup_problem: read in data for semidefinite programs to be solved
// INPUTS: pointer to a System object
// OUTPUTS: none 
void CONSTRAINT::setup_problem(struct System* quad)
{

	// Attempt to obtain possession of the map_lock
	while(pthread_mutex_trylock(&pose_lock))
	{

		// Wait 10 microseconds
		usleep(1);

	} // while(pthread_mutex_trylock(&map_lock))
	
	// Relinquish possession of the map_lock, immediately reobtain possession
	pthread_mutex_unlock(&pose_lock);		
	pthread_mutex_lock(&pose_lock);

		// Read quadrotor position
		x0(0) = (*quad).X0[0];
		x0(1) = (*quad).X0[1];
		x0(2) = (*quad).X0[2];

		// Read quadrotor velocity
		v0(0) = (*quad).V0[0];
		v0(1) = (*quad).V0[1];
		v0(2) = (*quad).V0[2];

	pthread_mutex_unlock(&pose_lock);	

	// Create point cloud representing quadrotor boundary
	float quad_points[8*3] = 
	{
		
		x0(0) - halfx,   x0(1) - halfy,   x0(2) - halfz,
		x0(0) - halfx,   x0(1) - halfy,   x0(2) + halfz,
		x0(0) - halfx,   x0(1) + halfy,   x0(2) - halfz,
		x0(0) - halfx,   x0(1) + halfy,   x0(2) + halfz,
		x0(0) + halfx,   x0(1) - halfy,   x0(2) - halfz,
		x0(0) + halfx,   x0(1) - halfy,   x0(2) + halfz,
		x0(0) + halfx,   x0(1) + halfy,   x0(2) - halfz,
		x0(0) + halfx,   x0(1) + halfy,   x0(2) + halfz

	}; // float quad_points[8*3] = 


	// Compute threashold for point cloud points to be "close enough"
	rho_bb = max(v0.norm(),_bar_v);
	rho_bb_inv = 1/rho_bb;

	if (debugging)
	{
		std::cout << "x0: " << x0 << std::endl;
		std::cout << "v0: " << v0 << std::endl;
		std::cout << "rho_bb: " << rho_bb << std::endl;
	}

	// Create placeholder variables for file reading
	std::string temp;
	std::stringstream ss;

	// Clear the vector storing the quadrotor points
	x.clear();
	
	// Iterate over the number of points representing the quadrotor
	for (unsigned short int i = 0; i < 8*dimensions; i++)
	{
		
		// Load vector "x" with quadrotor point cloud
		x.push_back(quad_points[i]);

	} // for (int i = 0; i < 8*dimensions; i++)

	// Clear the vector storing the point cloud
	y.clear();
	// yy.clear();

	float norm_point_cloud;

	// Attempt to obtain possession of the map_lock
	while(pthread_mutex_trylock(&map_lock))
	{

		// Wait 10 microseconds
		usleep(1);

	} // while(pthread_mutex_trylock(&map_lock))
	
	// Relinquish possession of the map_lock, immediately reobtain possession
	pthread_mutex_unlock(&map_lock);		
	pthread_mutex_lock(&map_lock);

		for (int i = 0; i < point_cloud.size()/3; ++i)
		{

			norm_point_cloud = sqrt( (point_cloud[3*i] - x0(0))*(point_cloud[3*i] - x0(0)) + (point_cloud[3*i+1] - x0(1))*(point_cloud[3*i+1] - x0(1)) + 0.2*(point_cloud[3*i+2] - x0(2))*(point_cloud[3*+2] - x0(2)) );
			if ( norm_point_cloud < 3 )
			{
				y.push_back(point_cloud[3*i]);
				y.push_back(point_cloud[3*i+1]);
				y.push_back(point_cloud[3*i+2]);
			}

		}
	
		// Resize the vectors storing the point cloud
		y.resize(point_cloud.size());
		// yy.resize(point_cloud.size());

		// Inputs the point clound
		y = point_cloud;
		// yy = point_cloud;

	pthread_mutex_unlock(&map_lock);

	map_size = y.size()/3;

	// clear the end of file (eof) flag from the istream
	// inFile2.clear();
	// // reset the file stream
	// inFile2.seekg(0);
	
	// // only read in bounding sphere cloud and center it at the quadcopter's
	// // position.
	// while (std::getline(inFile2,temp)) // include the bounding sphere in the y set
	// {
		
	// 	ss << temp;
	// 	ss >> current_value;
	// 	y.push_back(current_value + (*quad).X0[coordinate++]);
	// 	coordinate %= dimensions;
	// 	ss.clear();

	// } // while (std::getline(inFile2,temp))

	// record the size of the sets to be separated
	x_size = x.size();
	y_size = y.size();

	// Determine if the number of points in point sets x and y are valid
	if ((x_size % dimensions) || (y_size % dimensions))
	{
	
		std::cout << ">> Invalid point array length detected." << std::endl;
		return;
	
	} // if ((x_size % dimensions) || (y_size % dimensions))

	// Record the number of points in the sets x and y
	// N = x_size / dimensions;
	M = y_size / dimensions;

	return;

} // void CONSTRAINT::setup_problem(struct System* quad)


// Member function of CONSTRAINT class
// quadraticDiscrimination: calls SDPA library to solve semidefinite programming
// problem (quadratic discrimination problem)
// INPUTS: pointer to a System object
// OUTPUTS: boolean indicating success
bool CONSTRAINT::quadraticDiscrimination(struct System* quad)
{

	// Define RASP_Vars as matrix of 7 (optimization variables) by # of child ellipsoids
	RASP_Vars = Eigen::MatrixXf::Zero(7,_bCenters_samples);

	// Clear vector of booleans storing the parity of eigenvalues 
	// of the shape matrix for each child ellipsoid 
	RASP_PSignDefinite.clear();

	// Resize the vector
	RASP_PSignDefinite.resize(_bCenters_samples);

	// Define a matrix to store the shape matrix of the child ellipsoids
	Eigen::Matrix3f P_pull;

	// Initialize SDPA objects, one for each child ellipsoid

	auto current_time = std::chrono::high_resolution_clock::now();
	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed= end_time - current_time;	

	// Iterate over the number of child ellipsoids
	for (unsigned short int k = 1; k <= _bCenters_samples; k++)
	{

		SDPA Problem1;
		// if (parameterType == 1)
		// {

		// 	// If the user selected the unstable but fast setting

		// } // if (parameterType == 1)
		// else if (parameterType == 2)
		// {

			// If the user selected the stable but slow setting
			// Problem1.setParameterType(SDPA::PARAMETER_STABLE_BUT_SLOW);
			Problem1.setParameterType(SDPA::PARAMETER_UNSTABLE_BUT_FAST);
			// Problem1.setParameterMaxIteration(_SDPA_params[0]);//100
			// Problem1.setParameterEpsilonStar(_SDPA_params[1]);//1*10^(-7));
			// Problem1.setParameterEpsilonDash(_SDPA_params[2]);//1*10^(-7));
			// Problem1.setParameterLambdaStar(_SDPA_params[3]);//100000);
			// Problem1.setParameterOmegaStar(_SDPA_params[4]);//1000);
			// Problem1.setParameterBetaStar(_SDPA_params[5]);//0.1);
			// Problem1.setParameterBetaBar(_SDPA_params[6]);//0.2);
			// Problem1.setParameterGammaStar(_SDPA_params[7]);//0.9);;

		// } // else if (parameterType == 2)
		// else
		// {

		// 	// Default to the the default parameter setting
		// 	Problem1.setParameterType(SDPA::PARAMETER_DEFAULT);

		// } 

		// ********************** //
		// Set up SDPA parameters //
		// ********************** //
		Problem1.setParameterLowerBound(-10000000.0);

		Problem1.inputConstraintNumber(8);
		Problem1.inputBlockNumber(2); // 1 for checking that A is PD, 1 for point separation, add more if needed
		Problem1.inputBlockSize(1,dimensions);
		Problem1.inputBlockType(1,SDPA::SDP);
		// Problem1.inputBlockSize(2,-(M+N)); // negative tells solver that block is diagonal
		Problem1.inputBlockSize(2,-(M)); // negative
		Problem1.inputBlockType(2,SDPA::SDP);
		Problem1.initializeUpperTriangleSpace();

		// set objective function. Currently: minimize gamma where gamma is the offset of the ellipse from the obstacle points. 
		Problem1.inputCVec(7,10);
	

		// ************************************************** //
		// Set special case F matrices (A is PD, for example) //
		// ************************************************** //
	
		// Iterate over the number of dimensions
		for (unsigned short int i = 1; i <= dimensions; i++)
		{
			// This is the RHS of the sign-definite constraint for the shape matrix 
			Problem1.inputElement(0, 1, i, i, 0);


		} // for (int i = 1; i <= dimensions; i++) 

		// We only invoke 6 constraints because P is symmetric (hence the other 3 would be redundant).
		// Note: inputElement(constraint #, block #, i entry, j entry, sign of (i,j entry)).
		Problem1.inputElement(1, 1, 1, 1, -1); 
		Problem1.inputElement(2, 1, 2, 2, -1);
		Problem1.inputElement(3, 1, 3, 3, -1);
		Problem1.inputElement(4, 1, 1, 2, -1);
		Problem1.inputElement(5, 1, 1, 3, -1);
		Problem1.inputElement(6, 1, 2, 3, -1);

		// Initialize upper triangular storage
		Problem1.initializeUpperTriangleSpace();

		// ********************************************** //
		// Set problem elements with consistent structure //
		// ********************************************** //

		// Create coefficient vector so problem data can be entered automatically
		float coefficient[7 + 1] = { 0 }; // Extra one for populating F0
		coefficient[0] = 0.0f;

		// Iterate over the number of points in the point cloud
		for (unsigned int i = 1; i <= M; i++)
		{
			if (i > 1 && sqrt( (y[(i - 2) * dimensions]-y[(i - 1) * dimensions])*(y[(i - 2) * dimensions]-y[(i - 1) * dimensions]) + (y[(i - 2) * dimensions+1]-y[(i - 1) * dimensions+1])*(y[(i - 2) * dimensions+1]-y[(i - 1) * dimensions+1]) + (y[(i - 2) * dimensions+2]-y[(i - 1) * dimensions+2])*(y[(i - 2) * dimensions+2]-y[(i - 1) * dimensions+2]) ) <= 0.2 )
			{
				continue;
			}

			// initialize x1,x2,x3 based on vector x or y as appropriate
			x1 = y[(i - 1) * dimensions];
			x2 = y[(i - 1) * dimensions + 1];
			x3 = y[(i - 1) * dimensions + 2];

			// load the coefficent vector. There's no real pattern so just do element by element
			// xT P x <= 1
			coefficient[1] = -(x1 - bCenters(k-1,0)) * (x1 - bCenters(k-1,0));
			coefficient[2] = -(x2 - bCenters(k-1,1)) * (x2 - bCenters(k-1,1));
			coefficient[3] = -(x3 - bCenters(k-1,2)) * (x3 - bCenters(k-1,2));
			coefficient[4] = -2 * (x1 - bCenters(k-1,0)) * (x2 - bCenters(k-1,1));
			coefficient[5] = -2 * (x1 - bCenters(k-1,0)) * (x3 - bCenters(k-1,2));
			coefficient[6] = -2 * (x2 - bCenters(k-1,1)) * (x3 - bCenters(k-1,2));
			coefficient[7] = 1;

			// load F matrices
			// Iterate over the number of constraints
			for (unsigned short int j = 0; j < Problem1.getConstraintNumber(); j++) 
			{

				// Fill the constraint elements (constraints are linear matrix inequalities, where the opt vars are the elements of the shape matrix and gamma)
				Problem1.inputElement(j, 2, i, i, coefficient[j]);

			} // for (int j = 0; j < Problem1.getConstraintNumber(); j++) 

		} // for (int i = 1; i <= M; i++)

		// ************* //
		// Solve the SDP //
		// ************* //

		Problem1.initializeUpperTriangle();
	  	Problem1.initializeSolve(); 
 		Problem1.solve();

	  	// Get pointer, xVec, points at memory containing ellipsoid coefficients
	  	double * SDP_vars = new double[7];	

	  	// Get optimization results
	  	SDP_vars = Problem1.getResultXVec();

	  	// Print the results
	  	Problem1.printResultXVec();

		// Extract values at memory location pointed at by SDP_vars, store into new
		// static array SDP_vars_new
		
	  	// Iterate over the number of unique elements of the shape matrix
		for (unsigned short int i = 0; i < 6; i++)
		{
			
			RASP_Vars(i,k-1) = (float) 1*(SDP_vars[i]);

		} // for (int i = 0; i < 6; i++)

		// Get the gamma variable 
		RASP_Vars(6,k-1) = (float) -1*SDP_vars[6];

		// Fill the shape matrix
		P_pull << RASP_Vars(0,k-1), RASP_Vars(3,k-1), RASP_Vars(4,k-1), RASP_Vars(3,k-1), RASP_Vars(1,k-1), RASP_Vars(5,k-1), RASP_Vars(4,k-1), RASP_Vars(5,k-1), RASP_Vars(2,k-1);

		if (debugging)
		{
			std::cout << "P:" << P_pull << std::endl;
			std::cout << "k: " << RASP_Vars(6,k-1) << std::endl;
		}
			
		// Instantiate an object to store a matrix ready for decomposition
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 3, 3>> SAES_P(P_pull/RASP_Vars(6,k-1));

		// Compute the eigenvalues of the proper level set of the shape matrix
		eigenValues = SAES_P.eigenvalues().real();

		if (debugging)
		{
			std::cout << "eigenValues: " << eigenValues << std::endl;
		}
		if (((double)eigenValues[0] < 0.0) || ((double)eigenValues[1] < 0.0) || ((double)eigenValues[2] < 0.0))
		{
			RASP_PSignDefinite[k-1] = false;
		}
		else
		{
			RASP_PSignDefinite[k-1] = true;
		}

	} // for (int k = 1; k <= _bCenters_samples; k++)

	end_time = std::chrono::high_resolution_clock::now();
	timeElapsed= end_time - current_time;	

	cout << "quadraticDiscrimination time: " << timeElapsed.count() << endl;

	if(_loggingEnabled)
	{
		for (unsigned short int j = 0; j < 7; j++)
		{
			for (unsigned short int i = 0; i < _bCenters_samples-1; i++)
			{
				log2 << RASP_Vars(j,i) << ",";
			}
			log2 << RASP_Vars(j,_bCenters_samples-1) << std::endl;
		}
	}

	return 1;

} // bool CONSTRAINT::quadraticDiscrimination(struct System* quad)


// Member function of CONSTRAINT class
// closePoints: this function determines which of the points in the point cloud are near 
// the surface of a child ellipsoid or a parent ellipsoid. It keeps track
// of which points have been designated "close" so there are no duplicates
// INPUTS: none
// OUTPUTS: none
void CONSTRAINT::closePoints()
{

	// Initialize matrices to store various values
	Eigen::MatrixXf nrmP_Y_xC, Y_xC;
	Y_xC = Eigen::MatrixXf::Zero(3,1);
	nrmP_Y_xC = Eigen::MatrixXf::Zero(1,1);
	Y_temp = Eigen::MatrixXf::Zero(3,1);

	// Set the integer capturing the number of close points to zero
	num_closePoints = 0;

	// Define a vector storing which points in the point cloud have been allocated
	vector<int> pointCloudChecked;

	// Resize the vector
	pointCloudChecked.resize(map_size);

	// Define a matrix to store the close points
	Eigen::MatrixXf tempClosePointsMat = Eigen::MatrixXf::Zero(3,1);

	// Iterate over the number of child ellipsoids
	for (unsigned short int i = 0; i < _bCenters_samples; i++)
	{

		// Full the shape matrix
		P_temp(0, 0) = RASP_Vars(0,i);
		P_temp(1, 1) = RASP_Vars(1,i);
		P_temp(2, 2) = RASP_Vars(2,i);
		P_temp(0, 1) = RASP_Vars(3,i);
		P_temp(1, 0) = RASP_Vars(3,i);
		P_temp(0, 2) = RASP_Vars(4,i);
		P_temp(2, 0) = RASP_Vars(4,i);
		P_temp(1, 2) = RASP_Vars(5,i);
		P_temp(2, 1) = RASP_Vars(5,i);

		// Initialize a self adjoint eigen solver instance so we can calculate matrix square root
		// properly.
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> P_ts(-P_temp/RASP_Vars(6,i));

		// Compute the matrix square root
		sqrtP_transpose = P_ts.operatorSqrt().transpose();

		// Iterate over the number of points in the point cloud
		for (unsigned int k = 1; k <= map_size; k++)
		{

			if (k > 1 && sqrt( (y[(k - 2) * dimensions]-y[(k - 1) * dimensions])*(y[(k - 2) * dimensions]-y[(k - 1) * dimensions]) + (y[(k - 2) * dimensions+1]-y[(k - 1) * dimensions+1])*(y[(k - 2) * dimensions+1]-y[(k - 1) * dimensions+1]) + (y[(k - 2) * dimensions+2]-y[(k - 1) * dimensions+2])*(y[(k - 2) * dimensions+2]-y[(k - 1) * dimensions+2]) ) <= 0.2 )
			{
				continue;
			}
			// initialize x1,x2,x3 based on vector x or y as appropriate
			Y_temp(0,0) = y[(k-1) * dimensions];
			Y_temp(1,0) = y[(k-1) * dimensions + 1];
			Y_temp(2,0) = y[(k-1) * dimensions + 2];

			// Compute the distance between the center of the ellipsoid and the point ( (x-xc)T P (x-xc) <= 1 )
			Y_xC = Y_temp - bCenters.row(i).transpose();

			// Compute the norm of the LHS of the ellipsoid expression
			nrmP_Y_xC(0,0) = (sqrtP_transpose * Y_xC).norm(); 

			// If the norm is small enough and the point in the point cloud has not already been designated as "close"
			if (nrmP_Y_xC(0,0) <= 1 + _epsilon && !pointCloudChecked[k-1])
			{
				// Increase the size of the closepoints matrix
				tempClosePointsMat.conservativeResize(3, tempClosePointsMat.cols()+1);

				// Add the point from the point cloud to the matrix
				tempClosePointsMat.col(tempClosePointsMat.cols()-1) = Y_temp;

				// Indicate that the point has been designated "close"
				pointCloudChecked[k-1] = 1;

			} // if (nrmP_Y_xC(0,0) <= 1 + _epsilon && !pointCloudChecked[k-1])

		} // for (int k = 1; k <= M; k++)

	} // for (int i = 0; i < _bCenters_samples; i++)

	while(pthread_mutex_trylock(&ellipsoid_lock))
	{
		usleep(10);
	}
	pthread_mutex_unlock(&ellipsoid_lock);
	pthread_mutex_lock(&ellipsoid_lock);

		// Get the shape matrix of the parent ellipsoid
		P_temp(0, 0) = MIN_DIST_Vars(0,0)/MIN_DIST_Vars(6,0);
		P_temp(1, 1) = MIN_DIST_Vars(1,0)/MIN_DIST_Vars(6,0);
		P_temp(2, 2) = MIN_DIST_Vars(2,0)/MIN_DIST_Vars(6,0);
		P_temp(0, 1) = MIN_DIST_Vars(3,0)/MIN_DIST_Vars(6,0);
		P_temp(1, 0) = MIN_DIST_Vars(3,0)/MIN_DIST_Vars(6,0);
		P_temp(0, 2) = MIN_DIST_Vars(4,0)/MIN_DIST_Vars(6,0);
		P_temp(2, 0) = MIN_DIST_Vars(4,0)/MIN_DIST_Vars(6,0);
		P_temp(1, 2) = MIN_DIST_Vars(5,0)/MIN_DIST_Vars(6,0);
		P_temp(2, 1) = MIN_DIST_Vars(5,0)/MIN_DIST_Vars(6,0);

	pthread_mutex_unlock(&ellipsoid_lock);

	// Initialize a self adjoint eigen solver instance so we can calculate matrix square root
	// properly.
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> P_ts(P_temp);

	// Compute the matrix square root
	sqrtP_transpose = P_ts.operatorSqrt().transpose();

	// Iterate over the number of points in the point cloud
	for (unsigned int k = 1; k <= map_size; k++)
	{

		if (k > 1 && sqrt( (y[(k - 2) * dimensions]-y[(k - 1) * dimensions])*(y[(k - 2) * dimensions]-y[(k - 1) * dimensions]) + (y[(k - 2) * dimensions+1]-y[(k - 1) * dimensions+1])*(y[(k - 2) * dimensions+1]-y[(k - 1) * dimensions+1]) + (y[(k - 2) * dimensions+2]-y[(k - 1) * dimensions+2])*(y[(k - 2) * dimensions+2]-y[(k - 1) * dimensions+2]) ) <= 0.2 )
		{
			continue;
		}

		// initialize x1,x2,x3 based on vector x or y as appropriate
		Y_temp(0,0) = y[(k-1) * dimensions];
		Y_temp(1,0) = y[(k-1) * dimensions + 1];
		Y_temp(2,0) = y[(k-1) * dimensions + 2];

		Y_xC = Y_temp - xC;

		// Compute the distance between the center of the ellipsoid and the point ( (x-xc)T P (x-xc) <= 1 )
		nrmP_Y_xC(0,0) = (sqrtP_transpose * Y_xC).norm(); 

		// If the norm is small enough and the point in the point cloud has not already been designated as "close"
		if (nrmP_Y_xC(0,0) <= 1 + _epsilon && !pointCloudChecked[k-1])
		{

			// Increase the size of the closepoints matrix
			tempClosePointsMat.conservativeResize(3, tempClosePointsMat.cols()+1);

			// Add the point from the point cloud to the matrix
			tempClosePointsMat.col(tempClosePointsMat.cols()-1) = Y_temp;

			// Indicate that the point has been designated "close"
			pointCloudChecked[k-1] = 1;

		} // if (nrmP_Y_xC(0,0) <= 1 + _epsilon && !pointCloudChecked[k-1])

	} // for (int k = 1; k <= M; k++)

	num_closePoints = tempClosePointsMat.cols();

	// If there are some close points detected
	if (num_closePoints > 1)
	{

		// Resize the close points matrix
		// closePointsMat = Eigen::MatrixXf::Zero(3,num_closePoints-1);
		closePointsMat = Eigen::MatrixXf::Zero(3,num_closePoints);
		
		// Iterate over the number of close points
		// for (int i = 0; i < num_closePoints-1; i++)
		for (int i = 0; i < num_closePoints; i++)
		{
			
			closePointsMat.col(i) = tempClosePointsMat.col(i);
			// closePointsMat.col(i) = tempClosePointsMat.col(i+1);

		} // for (int i = 0; i < num_closePoints-1; i++)

	} // if (num_closePoints > 1)

	// Decrease the number of close points by 1
	if (num_closePoints > 0)
	{
		num_closePoints--;

	} // if (num_closePoints > 0)

	if (debugging)
	{

		std::cout << "num_closePoints: " << num_closePoints << std::endl;
		std::cout << "closePointsMat: " << closePointsMat.transpose() << std::endl;
	}

	return;

} // void CONSTRAINT::closePoints()


// Member function of CONSTRAINT class
// fastApproximateConvexHull: compute an approximation of a convex hull of the space
// containing the UAV and excluding the obstacles
// INPUTS: none
// OUTPUTS: none 
void CONSTRAINT::fastApproximateConvexHull()
{

		// Points defining the intersection of the sectors with the sphere of influence
	// double xs, ys, zs;
	// int kk = 1;
	int k = 0;

	// Timing variables
	// auto end_time = std::chrono::high_resolution_clock::now();
	// std::chrono::duration<double> timeElapsed = end_time - current_time;

	// Determine the angle for which each sector line is defined
	// Note 4/21/2020: Leaving this calculation here in case we want to update the number of sectors 
	// and thus the angle defining them online.
	// THIS WILL BE COMPUTED ON STARTUP
	k1 = 2*M_PI/_num_xy_sectors;
	k2 = 2*M_PI/_num_z_sectors;

	alpha = linspace(0.0,2*M_PI,_num_xy_sectors+1);
	beta = linspace(0.0,2*M_PI,_num_z_sectors+1);

	// if (debugging)
	// {
	// 	for (int i = 0; i < _num_xy_sectors; i++)
	// 	{
	// 		cout << "alpha[" << i << "]: " << alpha[i] << ',';
	// 	}
	// 	cout << endl;
	// 	for (int j = 0; j < _num_xy_sectors; j++)
	// 	{
	// 		cout << "beta[" << j << "]: " << beta[j] << ',';
	// 	}
	// }

	// ******************************************* //
	// determine points defining the sector planes //
	// ******************************************* //

	// Set matrix storing sector points to zero
	sectorPoint_vector = Eigen::MatrixXf::Zero(_num_xy_sectors*_num_z_sectors,3);

	// Iterate over the number of horizontal segments 
	for (unsigned short int i = 0; i < _num_xy_sectors; i++)
	{

		// if (debugging)
		// {
		// 	std::cout << "Slice #" << i << " -----------------------------"<< std::endl;
		// }
		
		// Iterate over the number of vertical segments
		for (unsigned short int j = 0; j < _num_z_sectors; j++)
		{
			
			// Compute a coordinate for a sector
			xs = cos(alpha[i])*cos(beta[j]);
			ys = sin(alpha[i])*cos(beta[j]);
			zs = sin(beta[j]);

			// If the coordinate is arbitrarily small, set it to zero
			if (fabs(xs) < 0.0000001)
			{
			
				xs = 0.0;
			
			} // if (abs(xs) < 0.0000001)

			if (fabs(ys) < 0.0000001)
			{
			
				ys = 0.0;
			
			} // if (abs(ys) < 0.0000001)

			if (fabs(zs) < 0.0000001)
			{
			
				zs = 0.0;
			
			} // if (abs(zs) < 0.0000001)


			sectorPoint_vector(k,0) = xs; 
			sectorPoint_vector(k,1) = ys; 
			sectorPoint_vector(k,2) = zs; 

			
			// if (debugging)
			// {
			// 	std::cout << "sectorPoint_vector: " << sectorPoint_vector.row(k) << std::endl;
			// }
			k++;

		} // for (int j = 0; j < _num_z_sectors; j++)

	} // for (int i = 0; i < _num_xy_sectors; i++)

	// First, we find the points Bi,Ei,Ci,Di, which correspond to sector i
	// Note that, the slices created above make Bi,Ei,Bi+1,Ei+1 live on slice i
	// Ci,Di,Ci+1,Di+1 live on slice i+1

	// Set the sector points to zero
	b = Eigen::MatrixXf::Zero(_num_xy_sectors*_num_z_sectors/2,3);
	e = Eigen::MatrixXf::Zero(_num_xy_sectors*_num_z_sectors/2,3);
	c = Eigen::MatrixXf::Zero(_num_xy_sectors*_num_z_sectors/2,3);
	d = Eigen::MatrixXf::Zero(_num_xy_sectors*_num_z_sectors/2,3);

	// Iterate over the number of horizontal sectors
	for (unsigned short int i = 0; i < _num_xy_sectors ; i++)
	{

		// ith slice
		k = i;
		// i+1th slice 
		kk = k+1 >= _num_xy_sectors ? 0 : k+1;

		// compute the points on the ith slice
		// b.row(2*i) = sectorPoint_vector.row(k*_num_xy_sectors+1) + x0.transpose();
		// e.row(2*i) = sectorPoint_vector.row(k*_num_xy_sectors)  + x0.transpose();
		// c.row(2*i) = sectorPoint_vector.row(kk*_num_xy_sectors+1)  + x0.transpose();
		// d.row(2*i) = sectorPoint_vector.row(kk*_num_xy_sectors)  + x0.transpose();
		
		// // compute the points on the i+1th slice
		// b.row(2*i+1) = sectorPoint_vector.row(k*_num_xy_sectors+2)  + x0.transpose();
		// e.row(2*i+1) = sectorPoint_vector.row(k*_num_xy_sectors+1)  + x0.transpose();
		// c.row(2*i+1) = sectorPoint_vector.row(kk*_num_xy_sectors+2)  + x0.transpose();
		// d.row(2*i+1) = sectorPoint_vector.row(kk*_num_xy_sectors+1)  + x0.transpose();
		b.row(2*i) = sectorPoint_vector.row(k*_num_xy_sectors+1);
		e.row(2*i) = sectorPoint_vector.row(k*_num_xy_sectors);
		c.row(2*i) = sectorPoint_vector.row(kk*_num_xy_sectors+1);
		d.row(2*i) = sectorPoint_vector.row(kk*_num_xy_sectors);
		
		// compute the points on the i+1th slice
		b.row(2*i+1) = sectorPoint_vector.row(k*_num_xy_sectors+2);
		e.row(2*i+1) = sectorPoint_vector.row(k*_num_xy_sectors+1);
		c.row(2*i+1) = sectorPoint_vector.row(kk*_num_xy_sectors+2);
		d.row(2*i+1) = sectorPoint_vector.row(kk*_num_xy_sectors+1);
	} // for (int i = 0; i < _num_xy_sectors ; i++)

	// Iterate over the number of horizontal sectors
	for (unsigned short int i = 0; i < _num_xy_sectors ; i++)
	{

		// ith slice
		k = i;
		// i+1th slice 
		kk = k+1 >= _num_xy_sectors ? 0 : k+1;

		// // compute the points on the ith slice
		// b.row(2*(i+_num_xy_sectors)) = sectorPoint_vector.row(k*_num_xy_sectors+7)  + x0.transpose();
		// e.row(2*(i+_num_xy_sectors)) = sectorPoint_vector.row(k*_num_xy_sectors)  + x0.transpose();
		// c.row(2*(i+_num_xy_sectors)) = sectorPoint_vector.row(kk*_num_xy_sectors+7)  + x0.transpose();
		// d.row(2*(i+_num_xy_sectors)) = sectorPoint_vector.row(kk*_num_xy_sectors)  + x0.transpose();
		
		// // compute the points on the i+1th slice
		// b.row(2*(i+_num_xy_sectors)+1) = sectorPoint_vector.row(k*_num_xy_sectors+6)  + x0.transpose();
		// e.row(2*(i+_num_xy_sectors)+1) = sectorPoint_vector.row(k*_num_xy_sectors+7)  + x0.transpose();
		// c.row(2*(i+_num_xy_sectors)+1) = sectorPoint_vector.row(kk*_num_xy_sectors+6)  + x0.transpose();
		// d.row(2*(i+_num_xy_sectors)+1) = sectorPoint_vector.row(kk*_num_xy_sectors+7)  + x0.transpose();

		// compute the points on the ith slice
		b.row(2*(i+_num_xy_sectors)) = sectorPoint_vector.row(k*_num_xy_sectors+7);
		e.row(2*(i+_num_xy_sectors)) = sectorPoint_vector.row(k*_num_xy_sectors)  ;
		c.row(2*(i+_num_xy_sectors)) = sectorPoint_vector.row(kk*_num_xy_sectors+7);
		d.row(2*(i+_num_xy_sectors)) = sectorPoint_vector.row(kk*_num_xy_sectors);
		
		// compute the points on the i+1th slice
		b.row(2*(i+_num_xy_sectors)+1) = sectorPoint_vector.row(k*_num_xy_sectors+6);
		e.row(2*(i+_num_xy_sectors)+1) = sectorPoint_vector.row(k*_num_xy_sectors+7);
		c.row(2*(i+_num_xy_sectors)+1) = sectorPoint_vector.row(kk*_num_xy_sectors+6);
		d.row(2*(i+_num_xy_sectors)+1) = sectorPoint_vector.row(kk*_num_xy_sectors+7);


	} // for (int i = 0; i < _num_xy_sectors ; i++)

	// if (debugging)
	// {
	// 	std::cout << "#########################################################" << std::endl;
	// 		std::cout << "b: " << b << std::endl;
	// 	std::cout << "#########################################################" << std::endl;
	// 		std::cout << "e: " << e << std::endl;
	// 	std::cout << "#########################################################" << std::endl;
	// 		std::cout << "c: " << c << std::endl;
	// 	std::cout << "#########################################################" << std::endl;
	// 		std::cout << "d: " << d << std::endl;
	// }


	// ************************************************************* //
	// Using previously determined points, create a plane 			 //
	// equation, and find the normals to the plane. 				 //
	// This is done by finding vectors  							 //
	// AB_k = B-A ... 												 //
	// For now A is the quad position, so having the points defining //
	// the vertices of the sector will suffice. 					 //
	// Order for northern hemisphere is: 							 //
	// Right plane: BcrossE  										 //
	// Top plane: CcrossB 											 //
	// Left plane: DcrossC 											 //
	// Bottom plane: EcrossD 										 //
	// Order for southern hemisphere is: 							 //
	// Right plane: EcrossB 										 //
	// Bottom plane: BcrossC 										 //
	// Left plane: CcrossD 											 //
	// Top plane: DcrossE        									 //
	// THIS WILL BE COMPUTED ON SETUP                                //
	// Does not matter how large \overline{B}_{\rho} is, 			 //
	// the planes will have a constant orientation.					 //
	// ************************************************************* //

	// Set matrix storing plane equations for sectors to zero
	sectorPlanes = Eigen::MatrixXf::Zero(_num_xy_sectors*_num_z_sectors*2, 3);

	// Set matrix storing plane normals for sectors to zero
	sectorNormals = Eigen::MatrixXf::Zero(total_sectors,3);

	// // Vector storing the ith sector's points
	// Eigen::Vector3f bi, ei, ci, di;

	// Iterate over the total number of sectors
	for (unsigned short int i = 0; i < _num_xy_sectors*_num_z_sectors/2; i++)
	{

		// Compute a-(b,e,c,d) where a is the quadrotor's position
		// bi = b.row(i) - x0.transpose();
		// ei = e.row(i) - x0.transpose();
		// ci = c.row(i) - x0.transpose();
		// di = d.row(i) - x0.transpose();
		bi = b.row(i);
		ei = e.row(i);
		ci = c.row(i);
		di = d.row(i);
		// if (debugging)
		// {
		// 	std::cout << "bi: " << setprecision(6) << bi[0] << ", " << bi[1] << ", " << bi[2] << std::endl;
		// 	std::cout << "ei: " << setprecision(6) << ei[0] << ", " << ei[1] << ", " << ei[2] << std::endl;
		// 	std::cout << "ci: " << setprecision(6) << ci[0] << ", " << ci[1] << ", " << ci[2] << std::endl;
		// 	std::cout << "di: " << setprecision(6) << di[0] << ", " << di[1] << ", " << di[2] << std::endl;
		// }

		// if the current sector is in the northern hemisphere
		if (i < _num_xy_sectors*_num_z_sectors/4)
		{
			
			sectorPlanes.block(0+4*i, 0, 1, 3) = (bi).cross(ei).transpose();
			sectorPlanes.block(1+4*i, 0, 1, 3) = (ci).cross(bi).transpose();
			sectorPlanes.block(2+4*i, 0, 1, 3) = (di).cross(ci).transpose();
			sectorPlanes.block(3+4*i, 0, 1, 3) = (ei).cross(di).transpose();
			sectorPlanes.row(0+4*i).normalize();
			sectorPlanes.row(1+4*i).normalize();
			sectorPlanes.row(2+4*i).normalize();
			sectorPlanes.row(3+4*i).normalize();
			if (sectorPlanes.row(1+4*i).norm() != 0)
			{
				sectorNormals.row(i) = sectorPlanes.block(0+4*i,0,4,3).colwise().sum();
			}
			else
			{
				sectorNormals.row(i) = sectorPlanes.block(0+4*i,0,4,3).colwise().sum();
			}						
			sectorNormals.row(i).normalize();		
			// cout << "here1" << endl;

		} // if (i < _num_xy_sectors*_num_z_sectors/4)
		else
		{
		
			// otherwise, if the sector is in the southern hemisphere	
			sectorPlanes.block(0+4*i, 0, 1, 3) = (ei).cross(bi).transpose();
			sectorPlanes.block(1+4*i, 0, 1, 3) = (bi).cross(ci).transpose();
			sectorPlanes.block(2+4*i, 0, 1, 3) = (ci).cross(di).transpose();
			sectorPlanes.block(3+4*i, 0, 1, 3) = (di).cross(ei).transpose();
			sectorPlanes.row(0+4*i).normalize();
			sectorPlanes.row(1+4*i).normalize();
			sectorPlanes.row(2+4*i).normalize();
			sectorPlanes.row(3+4*i).normalize();
			if (sectorPlanes.row(1+4*i).norm() != 0)
			{
				sectorNormals.row(i) = sectorPlanes.block(0+4*i,0,4,3).colwise().sum();
			}
			else
			{
				sectorNormals.row(i) = sectorPlanes.block(0+4*i,0,4,3).colwise().sum();
			}
			sectorNormals.row(i).normalize();		
			// cout << "here2" << endl;

		} // if (i < _num_xy_sectors*_num_z_sectors/4)
		// else if (_num_xy_sectors*_num_z_sectors-i <= _num_xy_sectors*_num_z_sectors/8)


	} // for (int i = 0; i < _num_xy_sectors*_num_z_sectors/2; i++)

	// Timing variables
	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed = end_time - current_time;

	if (debugging)
	{
		std::cout << "#########################################################" << std::endl;
			std::cout << "b: " << b << std::endl;
		std::cout << "#########################################################" << std::endl;
			std::cout << "e: " << e << std::endl;
		std::cout << "#########################################################" << std::endl;
			std::cout << "c: " << c << std::endl;
		std::cout << "#########################################################" << std::endl;
			std::cout << "d: " << d << std::endl;
	}

	sectorPoint_vector = sectorPoint_vector_init*rho_bb;

	if (debugging)
	{
		for (unsigned short int i = 0; i < sectorPlanes.rows()/4 ; i++)
		{
			std::cout << "#########################################################" << std::endl;
			std::cout << "sector: " << i << std::endl;
			std::cout << "sectorPlanes" << sectorPlanes.row(0+4*i) << std::endl;
			std::cout << "sectorPlanes" << sectorPlanes.block(1+4*i, 0, 1, 3) << std::endl;
			std::cout << "sectorPlanes" << sectorPlanes.block(2+4*i, 0, 1, 3) << std::endl;
			std::cout << "sectorPlanes" << sectorPlanes.block(3+4*i, 0, 1, 3) << std::endl;
		} 
	 	std::cout << ">> plane equations done" << std::endl;

		std::cout << ">> computed bisections " << sectorNormals << std::endl;
		std::cout << ">> num_closePoints: " << num_closePoints << std::endl;
	}
	
	// ************************************************************************ //
	// With plane normals, if the dot product between a closePoint and the 		//
	// /three/four planes defining a sector is positive, the closePoint belongs //
	// to that sector.															//
	// Every point can only be a member of one sector.							//
	// If the point is assigned a sector, we can switch to the next				//
	// point without having to check the rest of the sectors.					//
	// THIS WILL BE UPDATED EVERY iterations									//
	// ************************************************************************ //

	// Set the matrix storing the membership of each point in the set of close points to ones
	closePoint_membership = Eigen::VectorXf::Ones(num_closePoints);

	// Set the matrix storing the distance between the closepoints and the quadrotor's position to zero
	closePointsMat_x0 = Eigen::MatrixXf::Zero(3,num_closePoints);

	// This vector is used to easily identify sectors that share an edge with the north or south pole of the
	// unit sphere. These sectors only have 3 planes, whereas other sectors have 4.
	// std::vector<int> n2zero;

	// Integer storing the number of 0s in the sector plane's definition
	int n2zeros;

	// Resize the vector
	n2zero.resize(3);

	// Iterate over the number of close points
	for (unsigned int i = 0; i < num_closePoints; i++)
	{	

		// Compute the difference between the close points and the quadrotor position
		closePointsMat_x0.col(i) = closePointsMat.col(i) - x0;

		// Negate the vector storing the membership of close points
		// when this variable gets set to a positive number, we stop checking the
		// remaining sectors
		closePoint_membership(i) *= -1;

		// Iterate over the number of total sectors
		for (unsigned short int j = 0; j < total_sectors; j++)
		{

			// Record the point of interest
			point = closePointsMat_x0.col(i);

			// record the sector planes
			n1 = sectorPlanes.block(4*j ,0,1,3).transpose();
			n2 = sectorPlanes.block(4*j+1,0,1,3).transpose();
			n3 = sectorPlanes.block(4*j+2,0,1,3).transpose();
			n4 = sectorPlanes.block(4*j+3,0,1,3).transpose();

			// if (debugging)
			// {
				// std::cout << "#####################################" << std::endl;
				// std::cout << "i,j: " << i << ", "<< j << std::endl;
				// std::cout << "n1: " << n1 << " n2: " << n2 << " n3: " << n3 << " n4: " << n4 << endl;
				// std::cout << "point dot ni: " << point.dot(n1) << ", " << point.dot(n2) << ", " << point.dot(n3) << ", " << point.dot(n4) << std::endl;
			// }

			// the sector plane that is "absent" when the sector shares an edge with the
			// north or south pole is always n2

			// Determine if the ith element of n2 is zero
			n2zero[0] = (int) (sectorPlanes(4*j+1,0) == 0);
			n2zero[1] = (int) (sectorPlanes(4*j+1,1) == 0);
			n2zero[2] = (int) (sectorPlanes(4*j+1,2) == 0);

			// Compute the sum of zeros
			n2zeros = n2zero[0] + n2zero[1] + n2zero[2];

			// If the close point in the frame fixed to the quadrotor, aligned with the inertial frame
			// points in the same direction as the normal n1
			if (point.dot(n1) >= 0) 
			{		

				// If the vector n2 is non-zero
				if (n2zeros < 3)
				{		

					// If the close point in the frame fixed to the quadrotor, aligned with the inertial frame
					// points in the same direction as the normal n2
					if (point.dot(n2) >= 0) //n2zeros == 3 means one of the plane normal's is (0,0,0). So, skip this evaluation if that is the case.
					{				

						// If the close point in the frame fixed to the quadrotor, aligned with the inertial frame
						// points in the same direction as the normal n3
						if (point.dot(n3) >= 0)
						{	

							// If the close point in the frame fixed to the quadrotor, aligned with the inertial frame
							// points in the same direction as the normal n4			
							if (point.dot(n4) >= 0)
							{
								
								closePoint_membership(i) = j;

							} // if (point.dot(n4) >= 0)

						} // if (point.dot(n3) >= 0)

					} // if (point.dot(n2) >= 0)

				} // if (n2zeros < 3)
				else
				{
					// Otherwise, only check n3 and n4

					// If the close point in the frame fixed to the quadrotor, aligned with the inertial frame
					// points in the same direction as the normal n3
					if (point.dot(n3) >= 0)
					{				

						// If the close point in the frame fixed to the quadrotor, aligned with the inertial frame
						// points in the same direction as the normal n4	
						if (point.dot(n4) >= 0)
						{
							
							closePoint_membership(i) = j;

						} // if (point.dot(n4) >= 0)

					} // if (point.dot(n3) >= 0)

				} // if (n2zeros < 3)

			} // if (point.dot(n1) >= 0) 

			// If we found a sector that the point belongs to, stop checking sectors
			if (closePoint_membership(i) != -1)
			{
				
				break;

			} // if (closePoint_membership(i) != -1)

		} // for (int j = 0; j < total_sectors; j++)

	} // for (int i = 0; i < num_closePoints; i++)

	if (debugging)
	{
		std::cout << "closePointsMat_x0: " << closePointsMat_x0.transpose() << std::endl;
		std::cout << "closePoint_membership done: " << closePoint_membership << std::endl;
	}

// ************************************************************************* //
	// Find the point in each sector which achieves minimal dot product <n_i,x>. //
	// THIS WILL BE UPDATED EVERY iterations									 //
	// ************************************************************************* //

	// Set the matrix storing the closest point in each sector to zeros
	closePoint_minimalProj_sectors = Eigen::MatrixXf::Zero(3,total_sectors);

	// Set the minimal projection to infinity
	minimalProj_sectors = Eigen::VectorXf::Ones(total_sectors)*1000000;

	// Set vector storing minimal projections to zero
	minimalProj_temp = Eigen::VectorXf::Zero(1);

	// Set vector storing minimal projections for each sector to zero
	minimalProj_offsets =  Eigen::VectorXf::Zero(total_sectors);


	// This block tells us which obstacle point in a given sector 
	// the plane will pass through
	// // Iterate over the number of close points
	for (unsigned int j = 0; j < num_closePoints; j++)
	{

		// Iterate over the total number of sectors
		for (unsigned short int i = 0; i < total_sectors; i++)
		{


			
			// If the jth close point is in the ith sector
			if (closePoint_membership(j) == i)
			{
				
				ni = sectorNormals.row(i).transpose();
				oj = closePointsMat_x0.col(j);
				
				// if (oj.norm() < 0.2)
				// {
				// 	continue;
				// }

				minimalProj_temp(0) = ni.dot(oj);

				// std::cout << "#####################################" << std::endl;
				// std::cout << "i,j: " << i << ", " << j << std::endl;
				// std::cout << "minimal proj temp: " << minimalProj_temp(0) << std::endl;

				if (minimalProj_temp(0) < minimalProj_sectors(i))
				{

					closePoint_minimalProj_sectors.col(i) = ni;
					minimalProj_offsets(i) = minimalProj_temp(0);
					minimalProj_sectors(i) = minimalProj_temp(0);

				} // if (minimalProj_temp(0) < minimalProj_sectors(i))

				break;

			} // if (closePoint_membership(j) == i)

		} // for (int i = 0; i < total_sectors; i++)

	} // for(int j = 0; j < num_closePoints; j++)

	// Define matrices to store the collision avoidance constraints
	dummy_ObstaclesFf = Eigen::MatrixXf::Zero(total_sectors,5);

	// Define a vector to store the sector number corresponding to the constraint
	dummy_s = Eigen::MatrixXf::Zero(total_sectors,1);

	// Integer to keep track of the number of constraints
	int count = 0;

	// Eigen::MatrixXf p1 = Eigen::MatrixXf::Zero(3,1), n = Eigen::MatrixXf::Zero(3,1), n1 = Eigen::MatrixXf::Zero(3,1), n2 = Eigen::MatrixXf::Zero(3,1);

	// cout << "Quad pos: " << x0.transpose() << endl;

	// Iterate over the total number of sectors
	for (unsigned short int i = 0; i < total_sectors; i++)
	{

		// If the rhs of the constraint is not zero
		if (minimalProj_offsets(i) != 0)
		{

			if( closePoint_minimalProj_sectors(2,i) < 0)
			{
				continue;
			}

			// Record the constraints
			closePoint_minimalProj_sectors.col(i) = closePoint_minimalProj_sectors.col(i).normalized(); 
			dummy_s(count,0) = i;

			dummy_ObstaclesFf(count,0) = closePoint_minimalProj_sectors(0,i);
			dummy_ObstaclesFf(count,1) = closePoint_minimalProj_sectors(1,i);
			dummy_ObstaclesFf(count,2) = closePoint_minimalProj_sectors(2,i);
			vn = Eigen::Map<Eigen::Vector3f>(closePoint_minimalProj_sectors.col(i).data(),3);
			vn1(0) = -vn(1);
			vn1(1) = vn(0);
			vn1(2) = 0.0;
			vn1.normalize();
			vn2 = vn.cross(vn1);
			p1 = x0 + vn2 + vn1 + minimalProj_offsets(i)*vn;
			// cout << "Plane " << i << " point: " << p1.transpose() << endl;
			dummy_ObstaclesFf(count,3) = vn.dot(p1);
			dummy_ObstaclesFf(count,4) = minimalProj_offsets(i); // 5th col is used for drawing

			// Increment the counter
			count++;

		} // if (minimalProj_offsets(i) != 0)

	} // for (int i = 0; i < total_sectors; i++)

	// Resize the matrix storing the constraints
	dummy_ObstaclesFf.conservativeResize(count,5);

	// If there are no constraints
	if (count == 0)
	{

		// resize the constraint matrix
		dummy_ObstaclesFf.conservativeResize(total_sectors,5);

		// Iterate over the total number of sectors
		for (unsigned short int i = 0; i < total_sectors; i++)
		{

			// Activate all constraints, but set the rhs of the plane equation to (essentially) infinity
			dummy_ObstaclesFf.block(i,0,1,3) = sectorNormals.row(i);
			dummy_ObstaclesFf(i,3) = 1000.0;
			dummy_ObstaclesFf(i,4) = 1000.0;

		} // for (int i = 0; i < total_sectors; i++)

		count = total_sectors;

	} // if (count == 0)


	// Define a matrix to store the lhs of the plane equation computed using the quad position
	Eigen::MatrixXf FxX = Eigen::MatrixXf::Zero(dummy_ObstaclesFf.rows(), 3);

	// Define a matrix to store the quad position
	Eigen::MatrixXf temp_pos = Eigen::MatrixXf::Zero(3,1);
	temp_pos(0,0) = x0(0);
	temp_pos(1,0) = x0(1);
	temp_pos(2,0) = x0(2);

	// cout << "temp_pos: " << temp_pos << endl;
	
	// log3 << "t = " << timeElapsed.count() << endl;
	// log3 << "pose: " << temp_pos(0,0) << ", " << temp_pos(1,0) << ", " << temp_pos(2,0) << endl;

	// Iterate over the number of constraints
	for (int i = 0; i < dummy_ObstaclesFf.rows(); ++i)
	{
		// dummy_ObstaclesFf(i,3)+= 1;
		// Iterate over the number of coefficients for the plane equation
		for (int j = 0; j < 3; ++j)
		{
			
			// FxX(i,j) = dummy_ObstaclesFf(i,j)*temp_pos(j,0);
			log3 << dummy_ObstaclesFf(i,j) << ", ";

		} // for (int j = 0; j < 3; j++)

		log3 << dummy_ObstaclesFf(i,3) << endl;

		// cout << "FxX row" << i << ": " << FxX.row(i).sum() << ", fx: " << dummy_ObstaclesFf(i,3) << " : " << dummy_s(i,0) << endl;
	
		// if (FxX.row(i).sum() >= dummy_ObstaclesFf(i,3))
		// {
		// 	dummy_ObstaclesFf(i,3) = 100;
		// }

	} // for (int i = 0; i < dummy_ObstaclesFf.rows(); i++)


	// cout << "obs: " << endl << dummy_ObstaclesFf << endl;
	// cout << "dummy_ObstaclesFf: " << endl << dummy_ObstaclesFf << endl;

	while(pthread_mutex_trylock(&constraint_lock))
	{
		usleep(1);
	}
	pthread_mutex_unlock(&constraint_lock);
	pthread_mutex_lock(&constraint_lock);

		// ObstaclesFf_hard.conservativeResizeLike(dummy_ObstaclesFf);
		ObstaclesFf_hard = Eigen::MatrixXf::Zero(dummy_ObstaclesFf.rows(),4);
		dummy_s.conservativeResize(dummy_ObstaclesFf.rows(),1);
		// ObstaclesFf_soft.conservativeResizeLike(dummy_ObstaclesFf);
		ObstaclesFf_soft = Eigen::MatrixXf::Zero(dummy_ObstaclesFf.rows(),4);
		
		// ObstaclesFf_hard.block(0,0,dummy_ObstaclesFf.rows(),3) = dummy_ObstaclesFf.block(0,0,dummy_ObstaclesFf.rows(),3);
		// ObstaclesFf_hard.block(0,3,dummy_ObstaclesFf.rows(),1) = dummy_ObstaclesFf.block(0,4,dummy_ObstaclesFf.rows(),1);
		ObstaclesFf_hard = dummy_ObstaclesFf.block(0,0,dummy_ObstaclesFf.rows(),4);
		ObstaclesFf_soft = dummy_ObstaclesFf;

		// ObstaclesFf_soft.block(0,0,dummy_ObstaclesFf.rows(),3) = ObstaclesFf_hard.block(0,0,dummy_ObstaclesFf.rows(),3);

		constraintSize = dummy_ObstaclesFf.rows();

		end_time = std::chrono::high_resolution_clock::now();
		timeElapsed = end_time - current_time;

		// std::cout << "ObstaclesFf_hard: " << dummy_ObstaclesFf << std::endl;
		// std::cout << "ObstaclesFf_soft: " << ObstaclesFf_soft << std::endl;

		Eigen::MatrixXf constraint_sat = Eigen::MatrixXf::Zero(ObstaclesFf_hard.rows(),2);
		for (int i = 0; i < ObstaclesFf_hard.rows(); i++)
		{
			if (-ObstaclesFf_hard(i,0)*x0(0) - ObstaclesFf_hard(i,1)*x0(1) - ObstaclesFf_hard(i,2)*x0(2) <= ObstaclesFf_hard(i,3) )
			{
				constraint_sat(i,0) = 1;
			}
				constraint_sat(i,1) = dummy_s(i,0);
		}

	pthread_mutex_unlock(&constraint_lock);

	// std::cout << "constraint_sat: " << constraint_sat << std::endl;
	if (debugging)
	{
		std::cout << "t = " << timeElapsed.count() << std::endl;
	}

	return;

} // void CONSTRAINT::fastApproximateConvexHull()


// Member function of CONSTRAINT class
// determineCenters: finds a parent ellipsoid and determines centers for the child ellipsoids
// INPUTS: pointer to a System object
// OUTPUTS: boolean indicating success/failure
bool CONSTRAINT::determineCenters(struct System* quad) 
{

	// Record the center for the parent ellipsoid
	xC = { x0(0), x0(1), x0(2) };

	// Set the matrix storing the results of the SDP to zeros
	MIN_DIST_Vars = Eigen::MatrixXf::Zero(7,1);

	// Instantiate an SDPA problem
	SDPA Problem;

	// Boolean indicating the parity of the shape matrix
	bool PSignDefinite;

	// If the user selected the unstable but fast setting
	// if (parameterType == 1)
	// {
	// 	Problem.setParameterType(SDPA::PARAMETER_UNSTABLE_BUT_FAST);
	// }
	// else if (parameterType == 2)
	// {
		// If the user selected the stable but slow setting
		Problem.setParameterType(SDPA::PARAMETER_STABLE_BUT_SLOW);
	// }
	// else
	// {
	// 	// Default parameter setting
	// 	Problem.setParameterType(SDPA::PARAMETER_DEFAULT);
	// }

	Problem.setParameterLowerBound(-1000000000.0);
	// Problem.setParameterMaxIteration(_SDPA_params[0]);//100
	// Problem.setParameterEpsilonStar(_SDPA_params[1]);//1*10^(-7));
	// Problem.setParameterEpsilonDash(_SDPA_params[2]);//1*10^(-7));
	// Problem.setParameterLambdaStar(_SDPA_params[3]);//100000);
	// Problem.setParameterOmegaStar(_SDPA_params[4]);//1000);
	// Problem.setParameterBetaStar(_SDPA_params[5]);//0.1);
	// Problem.setParameterBetaBar(_SDPA_params[6]);//0.2);
	// Problem.setParameterGammaStar(_SDPA_params[7]);//0.9);;

	// Set up problem blocks and dimensions
	Problem.inputConstraintNumber(8);
	Problem.inputBlockNumber(2); // 1 for checking that A is PD, 1 for point separation, add more if needed
	Problem.inputBlockSize(1, dimensions);
	Problem.inputBlockType(1, SDPA::SDP);
	//Problem.inputBlockSize(2, -(M)); // negative tells solver that block is diagonal
	Problem.inputBlockSize(2, -(map_size + N)); // negative tells solver that block is diagonal
	Problem.inputBlockType(2, SDPA::SDP);

	// Initialize upper triangular space to take advantage of problem structure
	Problem.initializeUpperTriangleSpace();

	// set objective function. Currently: minimize gamma where gamma is the offset of the elipse from the obstacle points. 
	Problem.inputCVec(7, 10);

	auto current_time = std::chrono::high_resolution_clock::now();
	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed = end_time - current_time;	

	// Set special case F matrices (A is PD, for example)
	// Iterate over the number of dimensions
	for (int i = 1; i <= dimensions; i++) 
	{
		
		Problem.inputElement(0, 1, i, i, 0);

	} // for (int i = 1; i <= dimensions; i++) 

	// The lines below represent the P >= I constraint.
	// We only invoke 6 constraints because P is symmetric (hence the other 3 would be redundant).
	// Note: inputElement(constraint #, block #, i entry, j entry, value of (i,j entry)).
	Problem.inputElement(1, 1, 1, 1, -1);
	Problem.inputElement(2, 1, 2, 2, -1);
	Problem.inputElement(3, 1, 3, 3, -1);
	Problem.inputElement(4, 1, 1, 2, -1);
	Problem.inputElement(5, 1, 1, 3, -1);
	Problem.inputElement(6, 1, 2, 3, -1);

	// Set problem elements with consistent structure
	// Create coefficient vector so problem data can be entered automatically
	float coefficient[7 + 1] = { 0 }; // Extra one for populating F0
	coefficient[0] = 0.0f;

	// Iterate over number of points in the point cloud
	cout << "map_size: " << map_size << endl;

	for (int i = 1; i <= map_size; i++)
	{

		if (i > 1 && sqrt( (y[(i - 2) * dimensions]-y[(i - 1) * dimensions])*(y[(i - 2) * dimensions]-y[(i - 1) * dimensions]) + (y[(i - 2) * dimensions+1]-y[(i - 1) * dimensions+1])*(y[(i - 2) * dimensions+1]-y[(i - 1) * dimensions+1]) + (y[(i - 2) * dimensions+2]-y[(i - 1) * dimensions+2])*(y[(i - 2) * dimensions+2]-y[(i - 1) * dimensions+2]) ) <= 0.2 )
		{
			continue;
		}

		// initialize x1,x2,x3 based on vector x or y as appropriate
		x1 = y[(i - 1) * dimensions];
		x2 = y[(i - 1) * dimensions + 1];
		x3 = y[(i - 1) * dimensions + 2];

		// load the coefficent vector. There's no real pattern so just do element by element
		coefficient[1] = -(x1 - xC[0]) * (x1 - xC[0]);
		coefficient[2] = -(x2 - xC[1]) * (x2 - xC[1]);
		coefficient[3] = -(x3 - xC[2]) * (x3 - xC[2]);
		if (fabs(coefficient[1]) < 0.01 && fabs(coefficient[2]) < 0.01 && fabs(coefficient[3]) < 0.01)
		{
			continue;
		}		
		coefficient[4] = -2 * (x1 - xC[0]) * (x2 - xC[1]);
		coefficient[5] = -2 * (x1 - xC[0]) * (x3 - xC[2]);
		coefficient[6] = -2 * (x2 - xC[1]) * (x3 - xC[2]);
		coefficient[7] = 1;

		// load F matrices
		// Iterate over the number of constraints
		for (int j = 0; j < Problem.getConstraintNumber(); j++) 
		{
			
			Problem.inputElement(j, 2, i, i, coefficient[j]);

		} // for (int j = 0; j < Problem.getConstraintNumber(); j++)

	} // for (int i = 1; i <= M; i++)

	// Iterate over the number of quadcopter points
	for (int i = map_size + 1; i <= map_size + N; i++)
	{
		//initialize x1,x2,x3 based on vector x or y as appropriate
		x1 = x[(i - (map_size + 1)) * dimensions];
		x2 = x[(i - (map_size + 1)) * dimensions + 1];
		x3 = x[(i - (map_size + 1)) * dimensions + 2];

		// load the coefficent vector. There's no real pattern so just do element by element
		coefficient[1] = (x1 - xC[0]) * (x1 - xC[0]);
		coefficient[2] = (x2 - xC[1]) * (x2 - xC[1]);
		coefficient[3] = (x3 - xC[2]) * (x3 - xC[2]);
		if (fabs(coefficient[1]) < 0.01 && fabs(coefficient[2]) < 0.01 && fabs(coefficient[3]) < 0.01)
		{
			continue;
		}		
		coefficient[4] = 2 * (x1 - xC[0]) * (x2 - xC[1]);
		coefficient[5] = 2 * (x1 - xC[0]) * (x3 - xC[2]);
		coefficient[6] = 2 * (x2 - xC[1]) * (x3 - xC[2]);
		coefficient[7] = -1;

		// load F matrices
		// Iterate over the number of constraints
		for (int j = 0; j < Problem.getConstraintNumber(); j++) 
		{

			Problem.inputElement(j, 2, i, i, coefficient[j]);
			
		} // for (int j = 0; j < Problem.getConstraintNumber(); j++)

	} // for (int i = M + 1; i <= M + N; i++)

	// ************* //
	// Solve the SDP //
	// ************* //

	Problem.initializeUpperTriangle();
	Problem.initializeSolve();
	Problem.solve();

	// Get times
	end_time = std::chrono::high_resolution_clock::now();
	timeElapsed = end_time - current_time;

	// Get pointer, xVec, points at memory containing ellipsoid coefficients, assign to memory 			pointed at by xVec_temp.
	double * SDP_vars = new double[7];

	// Get the results from the optimization
	SDP_vars = Problem.getResultXVec();

	// Extract values at memory location pointed at by SDP_vars, store into new
	// static array SDP_vars_new
	for (int k = 0; k < 6; k++)
	{
		MIN_DIST_Vars(k,0) = (float)1 * (SDP_vars[k]);
	}

	// get the gamma value
	MIN_DIST_Vars(6,0) = SDP_vars[6];
	true_k = SDP_vars[6];

	// Instantiate a matrix to store the shape matrix
	Eigen::Matrix3f P_pull;

	// Fill the shape matrix
	P_pull << MIN_DIST_Vars(0,0), MIN_DIST_Vars(3,0), MIN_DIST_Vars(4,0), MIN_DIST_Vars(3,0), MIN_DIST_Vars(1,0), MIN_DIST_Vars(5,0), MIN_DIST_Vars(4,0), MIN_DIST_Vars(5,0), MIN_DIST_Vars(2,0);

	if (debugging)
	{
		std::cout << "P:" << P_pull << std::endl;
		std::cout << "k: " << SDP_vars[6] << std::endl;
	}

	// Set up a matrix solver so we can compute the eigenvalues of the shape matrix
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 3, 3>> SAES_P(1.5*P_pull/true_k);

	// Compute the eigenvalues of the shape matrix
	eigenValues = SAES_P.eigenvalues().real();

	if (debugging)
	{
		std::cout << "eigenValues: " << eigenValues << std::endl;
	}

	// Determine if all eigen values are negative (negative definite matrix)
	if (((double)eigenValues[0] < 0.0) || ((double)eigenValues[1] < 0.0) || ((double)eigenValues[2] < 0.0))
	{
		
		PSignDefinite = false;

	} // if (((double)eigenValues[0] < 0.0) || ((double)eigenValues[1] < 0.0) || ((double)eigenValues[2] < 0.0))
	else
	{
		
		PSignDefinite = true;

	} // if (((double)eigenValues[0] < 0.0) || ((double)eigenValues[1] < 0.0) || ((double)eigenValues[2] < 0.0))

	end_time = std::chrono::high_resolution_clock::now();
	timeElapsed= end_time - current_time;	

	cout << "determineCenters time: " << timeElapsed.count() << endl;

	// Log data
	if (_loggingEnabled)
	{
		
		log << "t = " << timeElapsed.count() << std::endl;
		log1 << "t = " << timeElapsed.count() << std::endl;
		log << "P: ";
		for (int k = 0; k < 5; k++)
		{
			log << MIN_DIST_Vars(k,0) << ",";
		}
		log << MIN_DIST_Vars(5,0) << endl;

		log << "k: " << MIN_DIST_Vars(6,0) << endl;

		log << "Eigenvalues: " << eigenValues[0] << ", " << eigenValues[1] << ", " << eigenValues[2] << std::endl;

	} // if (_loggingEnabled)

	// If the shape matrix is sign definite
	if (PSignDefinite)
	{

		// Set the centers to zero
		bCenters = Eigen::MatrixXf::Zero(_bCenters_samples,3);

		// Compute some angles used to sample the parent ellipsoid
		std::vector<double> angles = linspace(0.0,2*M_PI, _bCenters_samples+1); 

		// Iterate over the number of sample points
		for (int i = 0; i < _bCenters_samples; i++)
		{

			// Compute the 3D coordinate for the child ellipsoid's center
			vecTemp(0) = cos(angles[i]);
			vecTemp(1) = sin(angles[i]);
			vecTemp(2) = 0.0;
			bCenters.block(i,0,1,3) = ( SAES_P.operatorInverseSqrt() * vecTemp + xC).transpose();
		
		} // for (int i = 0; i < _bCenters_samples; i++)
			
		return PSignDefinite;

	} // if (PSignDefinite)
	else
	{
		
		return false;

	} // if (PSignDefinite)

} // bool CONSTRAINT::determineCenters(struct System* quad) 


// Member function of CONSTRAINT
void CONSTRAINT::flightstack_interface_thread(struct System* quad)
{

	CLIENT client("Parameter_Files/socket_parameters_constraint_generation.txt", &exit_thread);

	// Variables to store how many bytes are read over each socket.
	int mapread = 0, quadread = 0, waypointread = 0;

	// Setup buffers.
	int point_cloudSize = 0;

	// Let upstream code know that the flightstack_interface started properly.
	flightstack_status = 1;

	// FirstPassComplete indicates if a start, goal, and map have been read.
	bool mapPass = false, quadPass = false, firstPassComplete = false;

	int newConstraints = 0, prevconstraintSize = 0;
	Eigen::MatrixXf prevObstaclesFf_hard = Eigen::MatrixXf::Zero(33,4);

	char map[300000];
	int map_i[100][30][100]; 	

	// Pose communication vars
	std::vector<string> pose_recv;
	string pose_recv_string;
	char* pose_recv_buffer = new char[1000];
	char* cstr_pose = new char[1000];	

	// Constraint communication vars
	string constraints_send;
	char* constraints_send_buffer = new char[5000];
	char* cstr_constraints = new char[5000];

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

	float map_time_us = 1000000/map_freq; // Microseconds	
	float pose_time_us = 1000000/(pose_freq); // Microseconds	
	float goal_time_us = 1000000/goal_freq; // Microseconds	
	float path_time_us = 1000000/path_freq; // Microseconds	
	float constraint_time_us = 1000000/constraint_freq; // Microseconds	
	float trajectory_time_us = 1000000/trajectory_freq; // Microseconds		

	int bytes_read_map = 0, bytes_read_pose = 0, bytes_read_goal = 0, bytes_read_path = 0, bytes_read_constraints = 0, bytes_read_trajectory = 0;
	int counter = 0, _interface_loops = 0, occ = 0;

	bool firstLayerSaved = false;

	float pose[3];
	int numConstraints = 0;

	// Enter loop
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
				// if (bytes_read_pose == 1000*sizeof(char))
				// {

					pose_recv_string = pose_recv_buffer;
					// cout << "pose recv:" << endl;
					// cout << pose_recv_string << endl;
					counter = 0;
					pose_recv.clear();

					if (pose_recv_string[0] == 'Q' && pose_recv_string[2] != '!')
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

						while(pthread_mutex_trylock(&pose_lock))
						{
							usleep(1);
						}
						pthread_mutex_unlock(&pose_lock);
						pthread_mutex_lock(&pose_lock);

							if (counter > 0)
							{

								(*quad).X0[0] = boost::lexical_cast<float>(pose_recv[1]);
								(*quad).X0[1] = -boost::lexical_cast<float>(pose_recv[0]);
								(*quad).X0[2] = -boost::lexical_cast<float>(pose_recv[2]);
								(*quad).V0[0] = boost::lexical_cast<float>(pose_recv[4]);
								(*quad).V0[1] = -boost::lexical_cast<float>(pose_recv[3]);
								(*quad).V0[2] = -boost::lexical_cast<float>(pose_recv[5]);
								pose[0] = boost::lexical_cast<float>(pose_recv[0]);
								pose[1] = boost::lexical_cast<float>(pose_recv[1]);
								pose[2] = boost::lexical_cast<float>(pose_recv[2]);

							}

							// cout << "Pose: " << pose[0] << "," << pose[1] << "," << pose[2] << endl;

						pthread_mutex_unlock(&pose_lock);

					}

				// }

				counter = 0;
				quadPass = true;

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
				// Only update vars if properly sized data was read. 
				while(pthread_mutex_trylock(&map_lock))
				{
					usleep(1);
				}
				pthread_mutex_unlock(&map_lock);
				pthread_mutex_lock(&map_lock);

					bytes_read_map = client.sockets[0]->process_receiving(map,300000*sizeof(char),0);
					// cout << "bytes_read_map: " << bytes_read_map << endl;

					// cout << "map[0]: " << +map[0] << endl;
					
					point_cloud.clear();

					occ = 0;
					
					for (int k = 0; k < grid_z; k++) 
					{
						for (int j = 0; j < grid_y; j++) 
						{
							for (int i = 0; i < grid_x; i++) 
							{

								map_i[i][j][k] = +map[counter];
								if (map_i[i][j][k] == 3)
								{
									point_cloud.push_back(0.2*k);
									point_cloud.push_back(0.2*i);
									point_cloud.push_back(0.2*j);
									occ++;
								}
								counter++;

							}
						}
					}

					// if (!firstLayerSaved)
					// {
					// 	firstLayerSaved = true;
					// 	ofstream out_file;
					// 	out_file.open("map.txt");
					// 	if (!out_file.is_open())
					// 	{
					// 		exit(0);
					// 	}
					// 	// Iterate over lateral direction
					// 	for (int k = 0; k < 100; k++) 
					// 	{
					// 		// Iterate over depth direction
					// 		for (int i = 0; i < 100; i++) 
					// 		{
					// 			if (i < grid_x-1)
					// 			{
					// 				out_file << map_i[i][1][k] << ",";
					// 			}
					// 			else
					// 			{
					// 				out_file << map_i[i][1][k] << endl;
					// 			}
					// 		}
					// 	}
					// 	out_file.close();
					// }					
			
				pthread_mutex_unlock(&map_lock);
	
				mapPass = true;

			}

			current_time_map = std::chrono::high_resolution_clock::now();

		}

		////////////////////////////////
		// CONSTRAINTS COMMUNICATIONS //
		////////////////////////////////		

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_constraints).count() >= constraint_time_us)
		{

			// SEND THE CONSTRAINTS // 
			if (client.socket_active[9])
			{
				
				// Make some test data and append to string buffer
				constraints_send.clear();
				constraints_send.append("C");
				constraints_send.append(",");

				while(pthread_mutex_trylock(&constraint_lock))
				{
					usleep(1);
				}
				pthread_mutex_unlock(&constraint_lock);
				pthread_mutex_lock(&constraint_lock);		
					
					numConstraints = ObstaclesFf_hard.rows(); 

					for (int i = 0; i < ObstaclesFf_hard.rows(); i++)
					{
						float temp = ObstaclesFf_hard(i,0);
						constraints_send.append( boost::lexical_cast<string>(temp) );
						constraints_send.append(",");
						temp = ObstaclesFf_hard(i,1);
						constraints_send.append( boost::lexical_cast<string>(temp) );
						constraints_send.append(",");
						temp = ObstaclesFf_hard(i,2);
						constraints_send.append( boost::lexical_cast<string>(temp) );
						constraints_send.append(",");
						temp = ObstaclesFf_hard(i,3);
						constraints_send.append( boost::lexical_cast<string>(temp) );
						if (i < ObstaclesFf_hard.rows()-1)
						{
							constraints_send.append(",");
						}
						else
						{
							// constraints_send.append(",");
							// constraints_send.append(boost::lexical_cast<string>(end_time_stamp.count()));
							constraints_send.append("!");
						}
					}

					if (ObstaclesFf_hard.rows() == 0)
					{	
						// constraints_send.append(boost::lexical_cast<string>(end_time_stamp.count()));
						constraints_send.append("!");
					}

					// cout << "end_time_stamp.count(): " << boost::lexical_cast<string>(end_time_stamp.count()) << endl;

				pthread_mutex_unlock(&constraint_lock);

				// cout << "constraints_send: " << constraints_send << endl;

				// Copy the string to a char buffer
				// cstr_constraints = new char[5000];
				strcpy(cstr_constraints, constraints_send.c_str());
				memcpy(constraints_send_buffer, cstr_constraints, strlen(cstr_constraints)+1);

				// Send the path char buffer					
				client.sockets[9]->process_sending(constraints_send_buffer,5000*sizeof(char));

			}
			
			current_time_constraints = std::chrono::high_resolution_clock::now();

		}		

		if ( !firstPassComplete && mapPass && quadPass)
		{
			firstPassComplete = true;
			pthread_mutex_unlock(&initial_lock);
			std::cout << ">> All necessary data has been obtained from the flightstack." << std::endl;
			std::cout << ">> Unlocking so collision avoidance may proceed." << std::endl;

		}

		timeElapsed_map = end_time - current_time_map;
		timeElapsed_pose = end_time - current_time_pose;
		timeElapsed_constraints = end_time - current_time_constraints;

		// if (!(_interface_loops % 1000))
		// {

		// 	cout << "\033[2J\033[1;1H";
		// 	cout << "Position: " << std::fixed << std::setprecision(5) << -pose[0] << ", " << pose[1] << ", " << -pose[2] << " [m]" << endl;
		// 	cout << "Occupied: " << std::fixed << std::setprecision(5) << occ << endl;
		// 	cout << "# of Con: " << std::fixed << std::setprecision(5) << numConstraints << endl;
		// 	cout << "Timers:   " << "  MAP  " << "  POS  " << "  CON  " << endl;
		// 	cout << "Time [s]: " << std::fixed << std::setprecision(5) << timeElapsed_map.count() << " " << timeElapsed_pose.count() << " " << timeElapsed_constraints.count() << endl;

		// }

		// _interface_loops++;		

	}

}
