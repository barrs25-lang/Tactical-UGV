// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 6th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file implements the path planning algorithm.
// End File Decscription ################################################################################

// List include files
#include <LPAstar.h>
#include <my_client.h>

// boolean capturing if the first pass of the communication thread has occured
bool firstPassComplete = false;

// Timer variable
auto current_time = std::chrono::high_resolution_clock::now();

// Speed diagnostics - floats capture the average loop speed
float average_LPAstar_loop_speed, average_gnc_connection_loop_speed;

// Integers capturing the number of loops through threads
int gnc_connection_interface_loops, LPAstar_loops;

// Define the connectivity of the graph which represents a voxel map
#define DIRECTIONS 26

// Define the movements allowable on this graph, as well as the reverse movements
int dx[DIRECTIONS] = {1, 0, -1, 0, 1,  1, -1, -1, 1, 0, -1,  0, 1,  1, -1, -1,  1,  0, -1,  0,  1,  1, -1, -1, 0, 0};
int dy[DIRECTIONS] = {0, 1, 0, -1, 1, -1,  1, -1, 0, 1,  0, -1, 1, -1,  1, -1,  0,  1,  0, -1,  1, -1,  1, -1, 0, 0};
int dz[DIRECTIONS] = {0, 0, 0,  0, 0,  0,  0,  0, 1, 1,  1,  1, 1,  1,  1,  1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1};
int flip[DIRECTIONS] = {2, 3, 0, 1, 7, 6, 5, 4, 18, 19, 16, 17, 23, 22, 21, 20, 10, 11, 8, 9, 15, 14, 13, 12, 25, 24};

// Integers capturing the previous goal position on the voxel map
int xgoalprev, ygoalprev, zgoalprev;

// Integers capturing a base used to store the LPA* key in one value as opposed to two
int BASE = 1000;

// Integer capturing "infinity"
int LARGE = 999;

// Prototype of cell structure
struct cell;

// Type definition of cell
typedef struct cell cell;

// Cell structure definition
struct cell
{
	// Cell coordinates (in the grid)
  int x, y, z;

  // Pointer to a cell, this encodes the movements from the current cell
  cell *move[DIRECTIONS];

  // Pointer to the predecessor of the current cell in the search algorithm
  cell *searchtree;

  // Boolean capturing the occupancy status of the cell
  bool obstacle;

  // float captuing the cost-to-come
  float g;

  // float captuing the rhs value (it's like a pseudo cost-to-come that should be equal to the cost-to-come for the cell to be "locally consistent")
  float rhs;

  // float captuing the heuristic value
  float h;

  // float captuing the cell's key (utilizes BASE to store two values in one number)
  float key;

  // integer used in maintaining the heap (open queue)
  int heapindex;

  // integer capturing how many times the cell has been updated relative to the voxel map
  int iteration;

  bool danger;

  float kappa_sqrt;

  int x_weight, y_weight, z_weight;  

  // Boolean capturing if the voxel is inferred, perhaps occupied by a structure such as a ceiling or floor, but not having been explored yet and thus true occupancy status is unknown.
  bool inferred = 0;

}; // struct cell

// triple pointer to the maze, initially set to NULL
cell ***maze = NULL;
cell ***maze_prev = NULL;

// pointers to cells that capture the start, goal, and previous goal
cell *mazestart, *mazegoal, *prevmazegoal;

// integer captuing the number of times the voxel map has been updated (without replanning from scratch)
int mazeiteration = 0;

// Define the heapsize (used to preallocate memory for the heap)
// This should be an overestimate of the number of expected locally inconsistent cells in the voxel map (could be the total number of voxels)
#define HEAPSIZE 100000

// Pointer to HEAPSIZE cells
cell *heap[HEAPSIZE];

// Integer captuing the actual size of the heap (how many cells from the maze are in the heap)
int heapsize = 0;

// Bool signaling when the threads should shut down.
bool exit_thread = 0;

// Local method
// start_gnc_connection_interface: this function provides an interface between the setup code and the flightstack thread
// INPUTS: void pointer to some object
// OUTPUTS: void pointer (NULL)
void* start_gnc_connection_interface(void *args)
{
	
	// define a pointer to a Planner object to be equal to the dereferenced pointer
	Planner *planner_ = (Planner* )args;
	
	// Start the gnc_connection interface thread
	planner_->gnc_connection_thread();
	return NULL;

} // void* start_gnc_connection_interface(void *args)


// Local method
// start_LPAstar: this function provides an interface between the setup code and the LPAstar thread
// INPUTS: void pointer to some object
// OUTPUTS: void pointer (NULL)
void* start_LPAstar(void *args)
{
	
	// define a pointer to a Planner object to be equal to the dereferenced pointer
	Planner *planner_ = (Planner* )args;

	// Start the LPAstar path planner thread
	planner_->LPAstar_thread();
	return NULL;

} // void* start_LPAstar(void *args)


// Member function of Planner object
// setup: this function initializes mutex, starts threads, setups quit handling settings, initializes the maze, and reads from parameter files.
// INPUTS: none
// OUTPUTS: none
void Planner::setup()
{

	// Initialize pthread mutexs
	pthread_mutex_init(&path_lock, NULL);
	pthread_mutex_init(&map_lock, NULL);
	pthread_mutex_init(&start_lock, NULL);
	pthread_mutex_init(&goal_lock, NULL);
	pthread_mutex_init(&initialStartGoalMap_lock, NULL);

	// String to store file lines
	string file_line;
	stringstream ss;

	// String to store messages
	string message;

	log.openLogFile("PLANNER");
	log_path.openLogFile("PLANNER_DATA");

	message = "<PLANNER-SETUP> Setup pthread mutex.";
	cout << message << endl;
	log.writeToLog(message);

	// Setup the signal handler, see quit_handler() function above.
	signal(SIGINT,quit_handler);
	signal(SIGPIPE,quit_handler);
	signal(SIGSEGV,quit_handler);

	message = "<PLANNER-SETUP> Setup quit handler.";
	cout << message << endl;
	log.writeToLog(message);

	//******************************************//
	// Load heurestic tunables from a text file //
	//******************************************//
	
	// Open the parameter file
	ifstream file("Parameter_Files/Heuristic_params.txt");

	message = "<PLANNER-SETUP> Opened file: Parameter_Files/Heuristic_params.txt.";
	cout << message << endl;
	log.writeToLog(message);

	// mu1: \kappa(\alpha)'s left boundary. See eqn 4 of "Autonomous Unmanned Aerial Vehicles and 
	// Tactical Mapping".
	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> params.mu1;

	// mu2: \kappa(\alpha)'s right boundary. See eqn 4 of "Autonomous Unmanned Aerial Vehicles and 
	// Tactical Mapping".
	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> params.mu2;

	//mu3: \kappa(\alpha)'s minimum when \alpha = (mu1+mu2)/2. See eqn 4 of "Autonomous Unmanned 
	// Aerial Vehicles and Tactical Mapping".
	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> params.mu3;

	params.mu1_init = params.mu1;
	params.mu2_init = params.mu2;
	params.mu3_init = params.mu3;

	message = "<PLANNER-SETUP> Read file: Parameter_Files/Heuristic_params.txt.";
	cout << message << endl;
	log.writeToLog(message);

	// Close the input file stream
	file.close();

	message = "<PLANNER-SETUP> Closed file: Parameter_Files/Heuristic_params.txt.";
	cout << message << endl;
	log.writeToLog(message);

	message = "<PLANNER-SETUP> Opening file: Parameter_Files/Safety_params.txt.";
	cout << message << endl;
	log.writeToLog(message);

	ifstream file1("Parameter_Files/Safety_params.txt");

	float temp;

	// Danger Cone parameters
	// Angle [rad]: Angle made by the cone's axis and its boundary 
	do{ss.clear(); getline(file1, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> dzone.angle;

	// Danger Cylinder parameters
	// Radius [m]: Radius of cylinder.
	do{ss.clear(); getline(file1, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> temp;

	dzone.setRadius(temp);

	// Height [m]: Total height of cylinder
	do{ss.clear(); getline(file1, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> temp;

	dzone.setHeight(temp);

	file1.close();

	// Obtain possession of initialStartGoalMap_lock (ensures that the planner waits until the start, goal, and map have been received) 
	pthread_mutex_lock(&initialStartGoalMap_lock);

	// Start thread to constantly read the map, start, and goal from the gnc_connection.
	int result_gnc_connection = pthread_create( &gnc_connection_tid , NULL, &start_gnc_connection_interface, this);

	// Start thread to plan paths
	int result_astar = pthread_create( &LPAstar_tid , NULL, &start_LPAstar, this);

	// If the flight stack interface thread failed to start, throw an error
	if ( result_gnc_connection ) throw result_gnc_connection;

	// If the LPA* thread failed to start, throw an error
	if ( result_astar ) throw result_astar;

	message = "<PLANNER-SETUP> Threads started successfully.";
	cout << message << endl;
	log.writeToLog(message);

	// Allocate memory for the maze and establish each cell's x,y,z coordinates     
	maze = (cell ***)calloc(MAZEHEIGHT, sizeof(cell ***));
	maze_prev = (cell ***)calloc(MAZEHEIGHT, sizeof(cell ***));
	
	// Iterate over the height of the maze
	for (int z = 0; z < MAZEHEIGHT; ++z)
	{

		maze[z] = (cell **)calloc(MAZEWIDTH, sizeof(cell **));
		maze_prev[z] = (cell **)calloc(MAZEWIDTH, sizeof(cell **));

		// Iterate over the width of the maze
		for (int y = 0; y < MAZEWIDTH; ++y)
		{

			maze[z][y] = (cell *)calloc(MAZEDEPTH, sizeof(cell ));
			maze_prev[z][y] = (cell *)calloc(MAZEDEPTH, sizeof(cell ));

			// Iterate over the depth of the maze
			for (int x = 0; x < MAZEDEPTH; ++x)
			{
	      		
	      		maze[z][y][x].z = z;
	      		maze_prev[z][y][x].z = z;
	      		maze[z][y][x].y = y;
	      		maze_prev[z][y][x].y = y;
	      		maze[z][y][x].x = x;
	      		maze_prev[z][y][x].x = x;
	      		maze[z][y][x].obstacle = 0;
	      		maze_prev[z][y][x].obstacle = 0;

			} // for (int x = 0; x < MAZEDEPTH; ++x)

		} // for (int y = 0; y < MAZEWIDTH; ++y)

  } // for (int z = 0; z < MAZEHEIGHT; ++z)

	// ensures the program does not exit
	while(!exit_thread)
	{
	
	}

	// Join threads
	pthread_join(gnc_connection_tid, NULL);
	pthread_join(LPAstar_tid, NULL);

} // void Planner::setup()

// ********************************************** // 
// Heap (priority queue and map) helper functions //
// ********************************************** // 

bool query_heap(cell* check)
{

	if (heapsize != 0)
	{
		
		for (int i = 0; i < heapsize; i++)
		{
		
			// Cell is in the heap?
			if (heap[i] == check)
			{
				return true;
			}

		}

	}

	return false;

}

// Local method
// percolatedown: exchanges a parent cell with a child cell if the child's key is smaller than the parents key
// INPUTS: integer representing the heapindex, pointer to cell representing cell to be added or removed from the heap
// OUTPUTS: none
void percolatedown(int hole, cell *tmp)
{

  int child;

  if (heapsize != 0)
  {

    for (; 2*hole <= heapsize; hole = child)
		{
		  child = 2*hole;

		  // If we are not at the end of the heap, and the key of the next cell in the heap is smaller than the child cell's key
		  if (child != heapsize && heap[child+1]->key < heap[child]->key)
		  {

		  	// Move the child cell down the heap
		    ++child;

		  } // if (child != heapsize && heap[child+1]->key < heap[child]->key)

		  // If the child cell's key is smaller than the tmp cell's key, swap the parent and child
		  if (heap[child]->key < tmp->key)
	    {
		      
		      heap[hole] = heap[child];
		      heap[hole]->heapindex = hole;

	    } // if (heap[child]->key < tmp->key)
		  else
		  {

		    break;

		  } // if (heap[child]->key < tmp->key)

		} // for (; 2*hole <= heapsize; hole = child)

	      heap[hole] = tmp;
	      heap[hole]->heapindex = hole;

  } // if (heapsize != 0)

} // void percolatedown(int hole, cell *tmp)


// Local method
// percolateup: exchanges a parent cell with a child cell if the child's key is greater than the parents key
// INPUTS: integer representing the heapindex, pointer to cell representing cell to be added or removed from the heap
// OUTPUTS: none
void percolateup(int hole, cell *tmp)
{

  if (heapsize != 0)
  {

    for (; hole > 1 && tmp->key < heap[hole/2]->key; hole /= 2)
		{

		  heap[hole] = heap[hole/2];
		  heap[hole]->heapindex = hole;

		} // for (; hole > 1 && tmp->key < heap[hole/2]->key; hole /= 2) 

    heap[hole] = tmp;
    heap[hole]->heapindex = hole;

  } // if (heapsize != 0)

} // void percolateup(int hole, cell *tmp)


// Local method
// percolateupordown: decides whether or not the cell needs to perc up or down 
// INPUTS: integer representing the heapindex of the cell in question, a pointer to the cell in question in the heap
// OUTPUTS: none
void percolateupordown(int hole, cell *tmp)
{
  
	// If there is something in the heap
  if (heapsize != 0)
  {
    
    // If the heapindex is greater than 1 and the cell corresponding to hole/2 in the heap has a key less than that of the tmp cell
    if (hole > 1 && heap[hole/2]->key > tmp->key)
    {

			percolateup(hole, tmp);

    } // if (hole > 1 && heap[hole/2]->key > tmp->key)
    else
    {
			
			percolatedown(hole, tmp);

    } // if (hole > 1 && heap[hole/2]->key > tmp->key)

  } // if (heapsize != 0)

} // void percolateupordown(int hole, cell *tmp)


// Local method
// insertheap: insert a cell into the heap
// INPUTS: pointer to the cell that should be inserted into the heap
// OUTPUTS: none
void insertheap(cell *thiscell)
{

	// If the cell has not be percolated, percolate it up first
  if (thiscell->heapindex == 0)
  {

  	// Make sure the current heapsize does not exceed that which was allocated 
    assert(heapsize < HEAPSIZE-1);

    // Pre-increment heapsize, then use the result as the argument for percolateup
    percolateup(++heapsize, thiscell);

  } // if (thiscell->heapindex == 0)
  else
  {
    
    percolateupordown(thiscell->heapindex, heap[thiscell->heapindex]);

  } // if (thiscell->heapindex == 0)

} // void insertheap(cell *thiscell)


// Local method
// deleteheap: remove a cell from the heap
// INPUTS: pointer to cell
// OUTPUTS: none
void deleteheap(cell *thiscell)
{
  
  if (thiscell->heapindex != 0)
  {
    	
    	// Decide to perc up or down, post-decrement heapsize after removing the cell
      percolateupordown(thiscell->heapindex, heap[heapsize--]);
      thiscell->heapindex = 0;

  } // if (thiscell->heapindex != 0)

} // void deleteheap(cell *thiscell)


// Local method
// topheap: returns a pointer to the first element of the heap 
// INPUTS: none
// OUTPUTS: pointer to a cell
cell *topheap()
{
  
  // If there is nothing in the heap
  if (heapsize == 0)
  {

  	// Return a NULL pointer
    return NULL;

  } // if (heapsize == 0)

  return heap[1]; // what is heap[0]?

} // cell *topheap()


// Local method
// emptyheap: clears all cells from the heap
// INPUTS: none
// OUTPUTS: none
void emptyheap()
{

	// Iterate over the number of cells in the heap
  for (int i = 1; i <= heapsize; ++i)
  {
    
    // Set the heap indexes to zero (so when we insert a cell we perc up)
    heap[i]->heapindex = 0;

  } // for (int i = 1; i <= heapsize; ++i)

  heapsize = 0;

} // void emptyheap()


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


// Member function of Planner class
// getKappa: get the weighing function value for the cell with coordinates [x][y][z]
// INPUTS: integers representing the coordinates of the cell in the voxel grid
// OUTPUTS: float representing the value of the weighing function
float Planner::getKappa(int x, int y, int z)
{

	// Integer representing the distance (on the voxel grid) to the nearest obstacle
	int alpha = 0;
	float kappa = 0;

	int startx_int = floor(startx/MAP_RESOLUTION);
	int starty_int = floor(starty/MAP_RESOLUTION);
	int startz_int = floor(startz/MAP_RESOLUTION);
	
	int goalx_int = floor(goalx/MAP_RESOLUTION);
	int goaly_int = floor(goaly/MAP_RESOLUTION);
	int goalz_int = floor(goalz/MAP_RESOLUTION);

	float dist = sqrt((startx_int - x)*(startx_int - x) + (starty_int - y)*(starty_int - y) + (startz_int - z)*(startz_int - z));
	float dist1 = sqrt((goalx_int - x)*(goalx_int - x) + (goaly_int - y)*(goaly_int - y) + (goalz_int - z)*(goalz_int - z));

	// if (dist > 30.0 && dist1 > 30.0)
	// {
	// 	return 1.0;
	// }

	// get the distance to nearest obstacle in voxels
  alpha = getAlpha(x, y, z);

  // If the distance to the nearest obstacle lies outside the interval (\mu_1,\mu_2)
	if (alpha < params.mu1 || alpha > params.mu2)
	{	

		// Set the weighing function to 1.0
		kappa = 1.0;

	} // if (alpha < params.mu1 || alpha > params.mu2)
	else
	{

		// Otherwise, compute the weighing function
		kappa = params.mu3 - 0.5*(1-params.mu3)*(cos((2*M_PI*(alpha-params.mu1)/(params.mu2-params.mu1)) + M_PI) - 1);

	} // if (alpha < params.mu1 || alpha > params.mu2)

	// return the weighing function
	return kappa;

} // float Planner::getKappa(int x, int y, int z)

bool time_to_stop, terminate_timing_thread, earlyTermination = false;

// Member function of Planner class
// getAlpha: this function uses the DIRECTIONS to find the nearest obstacle to the cell located at x,y,z
// INPUTS: integers representing the coordinates of a cell in the voxel grid
// OUTPUTS: integer representing the distance to the nearest obstacle
int Planner::getAlpha(int x, int y, int z)
{

	// Float capturing the euclidean distance to the nearest obstacle
	float alpha = 0;
	
	// Integer capturing the distance to the nearest obstacle
	int alpha_int = 0;

	// Float array capturing the closest obstacle in each of the DIRECTIONS
	// If there is no obstacle in DIRECTION d within max_k steps, closeWalls[d] = infinity
	float closeWalls[DIRECTIONS];

	// Integers capturing some iterators
	int d, k, newx, newy, newz;

	// Integer capturing the search "radius" for finding the nearest obstacle
	int max_k = 20;

	// Iterate over DIRECTIONS, Set initial closeWalls to infinity
	for (int i = 0; i < DIRECTIONS; i++)
	{
		
		closeWalls[i] = 100000;

	} // for (int i = 0; i < DIRECTIONS; i++)

	// Iterate over DIRECTIONS
	for (d = 0; d < DIRECTIONS; ++d)
	{
		
    newz = z;
    newy = y;
    newx = x;

    // Iterate over discretized search radius
		for (k = 0; k < max_k; k++)
		{

    	newz = newz + dz[d];
    	if (abs(newz - z) >= 2.0)
    	{
    		newz -= dz[d];
    	}
	    newy = newy + dy[d];
	    newx = newx + dx[d];

	    // Check if newx,y,z are in the boundaries of the maze
	    if (newz > 0 && newz < MAZEHEIGHT && newy >= 0 && newy < MAZEWIDTH && newx >= 0 && newx < MAZEDEPTH )
 			{
				
				// If newx,y,z is an obstacle
				if (maze[newz][newy][newx].obstacle)
				{
					
					// Determine the distance to the obstacle from the cell at x,y,z
					closeWalls[d] = sqrt( 2*(newz-z)*(newz-z) + (newy-y)*(newy-y) + (newx-x)*(newx-x) );

					// Move on to the next direction
					break;

				} // if (maze[newz][newy][newx].obstacle)

			} // if (newz > 0 && newz < MAZEHEIGHT && newy >= 0 && newy < MAZEWIDTH && newx >= 0 && newx < MAZEDEPTH )
			else if ((newz <= 0) || (newy < 0) || (newx < 0) || (newz > MAZEDEPTH) || (newy > MAZEWIDTH) || (newx > MAZEHEIGHT))
			{
				
				// Set closeWalls to infinity
				closeWalls[d] = 100000;
				
				// Move on to the next direction
				break;

			} // else if ((newz <= 0) || (newy < 0) || (newx < 0) || (newz > MAZEDEPTH) || (newy > MAZEWIDTH) || (newx > MAZEHEIGHT))
			else
			{

				// Set closeWalls to infinity
				closeWalls[d] = 100000;

			} // // if (newz > 0 && newz < MAZEHEIGHT && newy >= 0 && newy < MAZEWIDTH && newx >= 0 && newx < MAZEDEPTH )

/*			if (k == max_k-1)
			{
				// Set closeWalls to infinity
				closeWalls[d] = 100000;
			}
*/
		} // for (k = 0; k < max_k; k++)

	} // for (d = 0; d < DIRECTIONS; ++d)

	// Find the closest obstacle in any direction  
	alpha = *min_element(closeWalls,closeWalls + DIRECTIONS - 1);

	// Round the closest obstacle to an integer
	alpha_int = ceil(alpha);

	// return the value of the distance to the obstacles' set
	return alpha_int;

} // int Planner::getAlpha(int x, int y, int z)


void* trace(void* args)
{

	ray_tracing_parameters* rtp = (ray_tracing_parameters*) (args);
	int x = (*rtp).x;
	int y = (*rtp).y;
	int z = (*rtp).z;
	int gx = (*rtp).gx;
	int gy = (*rtp).gy;
	int gz = (*rtp).gz;
	int exy = (*rtp).exy;
	int exz = (*rtp).exz;
	int ezy = (*rtp).ezy;
	int bx = (*rtp).bx;
	int by = (*rtp).by;
	int bz = (*rtp).bz;
	int sx = (*rtp).sx;
	int sy = (*rtp).sy;
	int sz = (*rtp).sz;
	int n = (*rtp).n;
	Planner* p = (*rtp)._planner;

	int z_steps = 0;
	
	while( n-- && x < 100 && y < 100 && z < 30)
	{

		// If ray tracing goes beyond the maps boundaries, stop
		if (x + sx < 0 || y + sy < 0 || z + sz < 0 || x + sx >= 100 || y + sy >= 100 || z + sz >= 30)
		{

			break;

		} // if (x + sx < 0 || y + sy < 0 || z + sz < 0 || x + sx >= 100 || y + sy >= 30 || z + sz >= 100)

		// If we have hit the xy-plane that the goal resides, return true, no unexplored along this ray
		if ( fabs(z - gz) < 0.1 || (*p).map_full[x][y][z] == 3 )
		{
			(*rtp).intersect_unexplored = 0;
			return NULL;

		} // if ( abs(z - gz) < 0.1 )
		else if ((*p).map_full[x][y][z] == 0 || (*p).map_full[x][y][z] == 2)
		{
			(*p).dzone.setHeight((z_steps+1)*MAP_RESOLUTION);
			(*rtp).intersect_unexplored = 1;
			return NULL;
		} // else if ((*p).tracersMap[x][y][z] == 0)
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
					z_steps++;
					exz -= bx; 
					ezy += by;

				} // if (exz < 0)

			} // if (exy < 0)
			else
			{

				if (ezy < 0)
				{
					
					z += sz;
					z_steps++;
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

		} // if ( abs(z - gz) < 0.1 )

	} // while( n-- && x < 100 && y < 30 && z < 100)

}

void* trace_infer(void* args)
{

	ray_tracing_parameters* rtp = (ray_tracing_parameters*) (args);
	int x = (*rtp).x;
	int y = (*rtp).y;
	int z = (*rtp).z;
	int gx = (*rtp).gx;
	int gy = (*rtp).gy;
	int gz = (*rtp).gz;
	int exy = (*rtp).exy;
	int exz = (*rtp).exz;
	int ezy = (*rtp).ezy;
	int bx = (*rtp).bx;
	int by = (*rtp).by;
	int bz = (*rtp).bz;
	int sx = (*rtp).sx;
	int sy = (*rtp).sy;
	int sz = (*rtp).sz;
	int n = (*rtp).n;
	Planner* p = (*rtp)._planner;

	int z_steps = 0;
	
	while( n-- && x < 100 && y < 100 && z < 30)
	{

		// If ray tracing goes beyond the maps boundaries, stop
		if (x + sx < 0 || y + sy < 0 || z + sz < 0 || x + sx >= 100 || y + sy >= 100 || z + sz >= 30)
		{

			break;

		} // if (x + sx < 0 || y + sy < 0 || z + sz < 0 || x + sx >= 100 || y + sy >= 30 || z + sz >= 100)

		// If we have hit the xy-plane that the goal resides, return true, no unexplored along this ray
		if ( (*p).map_full[x][y][z] == 3 )
		{
			
		} 
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

		} // if ( (*p).map_full[x][y][z] == 3 )

	} // while( n-- && x < 100 && y < 30 && z < 100)

	if (n > 0)
	{
		(*p).izone.poi_info_above[(*rtp).ray_number] = 1;
	}
	else
	{
		(*p).izone.poi_info_below[(*rtp).ray_number] = 0;
	}

	return NULL;

}

bool Planner::ray_tracing()
{

	// Record the position of the camera
	int x = floor(startx/MAP_RESOLUTION);
	int y = floor(starty/MAP_RESOLUTION);
	int z = floor(startz/MAP_RESOLUTION);

	// Record the goal point
	int gx = floor(goalx/MAP_RESOLUTION);
	int gy = floor(goaly/MAP_RESOLUTION);
	int gz = floor(goalz/MAP_RESOLUTION);

	// Record the difference in position of the camera and sample point
	int dxrt = gx - x;
	int dyrt = gy - y;
	int dzrt = gz - z;

	// Determine all ray origins
	int directions[8] = {0,0,0,0,0,0,0,0};

	if (dzone.radius_in_voxels > 0)
	{

		for (int i = 1; i <= dzone.radius_in_voxels; i++)
		{

			for (int d = 0; d < 8; d++)
			{
				
				int tx = i*dx[d];
				int ty = i*dy[d];

				if (tx >= 0 && tx < 100 && ty >= 0 && ty < 100 && sqrt(tx*tx + ty*ty) < dzone.radius_in_voxels*dzone.radius_in_voxels)
				{
					directions[d] = i; 
				}

			}

		}

	}

	int num_ray_origins = 0;

	for (int i = 0; i < 8; i++)
	{
		num_ray_origins+=directions[i]; 
	}

	auto ray_origins = new float[1][3];	
	if (num_ray_origins > 0)
	{
		delete [] ray_origins;
		ray_origins = new float[num_ray_origins+1][3];	
	}
	else
	{
		cout << "no ray origins" << endl;
	}

	ray_origins[0][0] = x;
	ray_origins[0][1] = y;
	ray_origins[0][2] = z;

	int counter = 0;
	for (int d = 0; d < 8; d++)
	{
		for (int j = 0; j < directions[d]; j++)
		{
			ray_origins[counter][0] = x+j*dx[d];
			ray_origins[counter][1] = y+j*dy[d];
			ray_origins[counter][2] = z;
			counter++;
		}
	}
	
	// Retrieve the sign of the change in each direction, store in sx/y/z
	get_sign(sx,dxrt); get_sign(sy,dyrt); get_sign(sz,dzrt);
	
	// magnitude of change in each direction
	ax = abs(dxrt); ay = abs(dyrt); az = abs(dzrt);
	
	bx = 2*ax; by = 2*ay; bz = 2*az;
	
	exy = ay-ax; exz = az-ax; ezy = ay-az; 
	
	n = ax+ay+az;

	ray_tracing_parameters* rayTraceVars = new ray_tracing_parameters[num_ray_origins];

 	pthread_t thread[num_ray_origins]; 

 	if (thread == NULL)
  {
     printf("out of memory\n");
     // exit(EXIT_FAILURE);
     std::raise(SIGINT);
  }

	for (int i = 0; i < num_ray_origins; i++)
	{

		rayTraceVars[i].setRayOrigin(ray_origins[i][0],ray_origins[i][1],ray_origins[i][2]);
		rayTraceVars[i].setGoal(gx,gy,gz);
		rayTraceVars[i].setS(sx,sy,sz);
		rayTraceVars[i].setN(n);
		rayTraceVars[i].setE(exy,exz,ezy);
		rayTraceVars[i].setB(bx,by,bz);
		rayTraceVars[i]._planner = this;

		pthread_create(&thread[i], NULL, &trace, &rayTraceVars[i]);

	}

	for (int i = 0; i < num_ray_origins; i++)
	{
		pthread_join(thread[i], NULL);
	}

	for (int i = 0; i < num_ray_origins; i++)
	{
		if (rayTraceVars[i].intersect_unexplored)
		{
			return true;
		}
	}

	return false;

} // bool ray_tracing()


void Planner::checkGoalSafety()
{

	int ex, ey, ez;

	// ****************************************************** //
	// Step zero: Check if there is an active danger cylinder //
	// ****************************************************** //
	// if (dzone.active)
	if (active_zones)
	{
		// cout << "here1" << endl;

		// ---------------------------------------------- //
		// Step 0.1: Check if start has left the cylinder //
		// ---------------------------------------------- //

		// If the UAV has not left the cylinder
		for (int i = 0; i < dzone_vec.size(); i++)
		{

			dzone_vec[i].vehicle_in_az = 0;
			ex = dzone_vec[i].center[0]-startx;
			ey = dzone_vec[i].center[1]-starty;
			ez = dzone_vec[i].center[2]-startz;
			if (sqrt(ex*ex + ey*ey) < dzone_vec[i].radius)
			{		
				cout << "has not left the cylinder" << endl;
				dzone_vec[i].vehicle_in_az = 1;
				return;
			}

		}
		

	}

	// only for standalone
	original_startx = startx;
	original_starty = starty;
	original_startz = startz;

	// In meters
	dzone.center[0] = startx;
	dzone.center[1] = starty;
	dzone.center[2] = startz;

	cout << dzone.center[0] << "," << dzone.center[1] << "," << dzone.center[2] << endl;

	// ************************************************* //
	// Step one: Check if the goal is in the danger cone //
	// ************************************************* //
	ex = -dzone.center[0]+goalx;
	ey = -dzone.center[1]+goaly;
	ez = -dzone.center[2]+goalz;

	int norm = sqrt(ex*ex + ey*ey + ez*ez);
	int nex = ex/norm;
	int ney = ey/norm;
	int nez = ez/norm;

	if (nex*dzone.axis[0] + ney*dzone.axis[1] + nez*dzone.axis[2] > cos(dzone.angle) || - nex*dzone.axis[0] - ney*dzone.axis[1] - nez*dzone.axis[2] > cos(dzone.angle))
	{

		// **************************************************** //
		// Step two: Perform ray tracing from start to the goal //
		// if the goal is in the danger cone 										//
		// **************************************************** //
			// cout << "here2" << endl;
					// Attempt to obtain possession of the map lock
		while (pthread_mutex_trylock(&map_lock))
		{

			// Wait 10 microseconds
			usleep(10);

		} // while (pthread_mutex_trylock(&map_lock))

		// Relinquish possession of the lock, immediately reobtain
		pthread_mutex_unlock(&map_lock);
		pthread_mutex_lock(&map_lock);

			dzone.active = ray_tracing();

		pthread_mutex_unlock(&map_lock);

		if (dzone.active)
		{
			dzone.vehicle_in_az = 1;
			dzone_vec.push_back(dzone);	
			active_zones++;
		}
	
	}
	else
	{
		// cout << "no danger" << endl;
	}


} // void Planner::checkGoalSafety()


void Planner::ceiling_floor_infer()
{

	int x,y,z,dxrt,dyrt,dzrt,gx,gy,gz;

	ray_tracing_parameters* rayTraceVars = new ray_tracing_parameters[izone.hSteps*izone.vSteps];

	pthread_t thread[izone.hSteps*izone.vSteps]; 

	if (thread == NULL)
  {
     printf("out of memory\n");
     // exit(EXIT_FAILURE);
     std::raise(SIGINT);
  }

	for (int i = 0; i < izone.hSteps*izone.vSteps; i++)
	{

		// Record the position of the camera
		x = floor( startx/MAP_RESOLUTION);
		y = floor( starty/MAP_RESOLUTION);
		z = floor( startz/MAP_RESOLUTION);

		// Record the goal point
		gx = floor(goalx/MAP_RESOLUTION);
		gy = floor(goaly/MAP_RESOLUTION);
		gz = floor(goalz/MAP_RESOLUTION);

		// Record the difference in position of the camera and sample point
		dxrt = gx - x;
		dyrt = gy - y;
		dzrt = gz - z;

		// Retrieve the sign of the change in each direction, store in sx/y/z
		get_sign(sx,dxrt); get_sign(sy,dyrt); get_sign(sz,dzrt);
		
		// magnitude of change in each direction
		ax = abs(dxrt); ay = abs(dyrt); az = abs(dzrt);
		
		bx = 2*ax; by = 2*ay; bz = 2*az;
		
		exy = ay-ax; exz = az-ax; ezy = ay-az; 
		
		n = ax+ay+az;

		rayTraceVars[i].setRayOrigin(x,y,z);
		rayTraceVars[i].setGoal(gx,gy,gz);
		rayTraceVars[i].setS(sx,sy,sz);
		rayTraceVars[i].setN(n);
		rayTraceVars[i].setE(exy,exz,ezy);
		rayTraceVars[i].setB(bx,by,bz);
		rayTraceVars[i].ray_number = i;
		rayTraceVars[i]._planner = this;

		pthread_create(&thread[i], NULL, &trace_infer, &rayTraceVars[i]);

	}

	for (int i = 0; i < izone.hSteps*izone.vSteps; i++)
	{
		pthread_join(thread[i], NULL);
	}

	Eigen::MatrixXf poi = Eigen::MatrixXf::Zero(3,1);
	Eigen::MatrixXf poi_trace_end_above_scatter = Eigen::MatrixXf::Zero(3,1);
	Eigen::MatrixXf poi_trace_end_below_scatter = Eigen::MatrixXf::Zero(3,1);

	for (int i = 0; i < izone.hSteps*izone.vSteps; i++)
	{
		if (izone.poi_info_above[i])
		{
			poi_trace_end_above_scatter.conservativeResize(3,poi_trace_end_above_scatter.cols()+1);
			poi_trace_end_above_scatter.block(0,poi_trace_end_above_scatter.cols()-1,3,1) = izone.poi_trace_end_above_voxel.block(0,i,3,1);
			poi_trace_end_above_scatter.col(poi_trace_end_above_scatter.cols()-1) = poi + poi_trace_end_above_scatter.col(poi_trace_end_above_scatter.cols()-1);
		}

		if (izone.poi_info_below[i])
		{
			poi_trace_end_below_scatter.conservativeResize(3,poi_trace_end_below_scatter.cols()+1);
			poi_trace_end_below_scatter.block(0,poi_trace_end_below_scatter.cols()-1,3,1) = izone.poi_trace_end_below_voxel.block(0,i,3,1);
			poi_trace_end_below_scatter.col(poi_trace_end_below_scatter.cols()-1) = poi - poi_trace_end_below_scatter.col(poi_trace_end_below_scatter.cols()-1);
		}
	}

	// Add an additional loop to account for izone.num_poi > 1
	Eigen::MatrixXf sample_mean_above = Eigen::MatrixXf::Zero(3,izone.num_poi);
	Eigen::MatrixXf sample_mean_below = Eigen::MatrixXf::Zero(3,izone.num_poi);

	for (int i = 0; i < poi_trace_end_above_scatter.cols(); i++)
	{
		sample_mean_above = sample_mean_above + poi_trace_end_above_scatter.col(i);
	}
	
	for (int i = 0; i < poi_trace_end_below_scatter.cols(); i++)
	{
		sample_mean_below = sample_mean_below + poi_trace_end_below_scatter.col(i);
	}

	sample_mean_above = sample_mean_above/poi_trace_end_above_scatter.cols();
	sample_mean_below = sample_mean_below/poi_trace_end_below_scatter.cols();

	Eigen::MatrixXf scatter_matrix_above = Eigen::MatrixXf::Zero(3,3);
	Eigen::MatrixXf scatter_matrix_below = Eigen::MatrixXf::Zero(3,3);
	Eigen::MatrixXf temp = Eigen::MatrixXf::Zero(3,1);

	for (int i = 0; i < poi_trace_end_above_scatter.cols(); i++)
	{
		temp = poi_trace_end_above_scatter.col(i) - sample_mean_above;
		scatter_matrix_above = scatter_matrix_above + temp*temp.transpose();
	}	

	for (int i = 0; i < poi_trace_end_below_scatter.cols(); i++)
	{
		temp = poi_trace_end_below_scatter.col(i) - sample_mean_below;
		scatter_matrix_below = scatter_matrix_below + temp*temp.transpose();
	}

	cout << "scatter_matrix_above: " << endl << scatter_matrix_above << endl;
	cout << "scatter_matrix_below: " << endl << scatter_matrix_below << endl;

	// Find eigenvectors of scatter matrices
	Eigen::ComplexEigenSolver<Eigen::MatrixXf> eigSolver_scatter_matrix_above(scatter_matrix_above);
	Eigen::ComplexEigenSolver<Eigen::MatrixXf> eigSolver_scatter_matrix_below(scatter_matrix_below);

	Eigen::MatrixXcf scatter_matrix_above_eigenvectors = eigSolver_scatter_matrix_above.eigenvectors();
	Eigen::MatrixXcf scatter_matrix_below_eigenvectors = eigSolver_scatter_matrix_below.eigenvectors();

	cout << "scatter_matrix_above_eigenvectors: " << endl << scatter_matrix_above_eigenvectors << endl;
	cout << "scatter_matrix_below_eigenvectors: " << endl << scatter_matrix_below_eigenvectors << endl;

	float max_eig_z_component_above = -2.0; int max_eig_z_component_above_index = -1;
	float max_eig_z_component_below = -2.0; int max_eig_z_component_below_index = -1;

	for (int i = 0; i < 3; i++)
	{
		if (abs(scatter_matrix_above_eigenvectors(2,i).real()) > max_eig_z_component_above)
		{
			max_eig_z_component_above = scatter_matrix_above_eigenvectors(2,i).real();
			max_eig_z_component_above_index = i;
		}
	}

	for (int i = 0; i < 3; i++)
	{
		if (abs(scatter_matrix_below_eigenvectors(2,i).real()) > max_eig_z_component_below)
		{
			max_eig_z_component_below = scatter_matrix_below_eigenvectors(2,i).real();
			max_eig_z_component_below_index = i;
		}
	}

	Eigen::Vector3f below_norm;
	below_norm(0) = scatter_matrix_below_eigenvectors(2,max_eig_z_component_below_index).real();
	below_norm(2) = scatter_matrix_below_eigenvectors(0,max_eig_z_component_below_index).real();
	below_norm.normalized();

	Eigen::Vector3f above_norm;
	above_norm(0) = scatter_matrix_above_eigenvectors(0,max_eig_z_component_above_index).real();
	above_norm(1) = scatter_matrix_above_eigenvectors(1,max_eig_z_component_above_index).real();
	above_norm(2) = scatter_matrix_above_eigenvectors(2,max_eig_z_component_above_index).real();
	above_norm.normalized();

	Eigen::VectorXf orientation_above_normal = Eigen::VectorXf::Zero(1);
	Eigen::VectorXf orientation_below_normal = Eigen::VectorXf::Zero(1);
	Eigen::Vector3f local_up_axis;
	Eigen::Vector3f local_down_axis;
	local_up_axis(2) = 1.0;
	local_down_axis(2) = -1.0;

	cout << "above_norm: " << endl << above_norm << endl;
	cout << "below_norm: " << endl << below_norm << endl;

	orientation_above_normal(0) = above_norm.dot(local_up_axis);
	orientation_below_normal(0) = below_norm.dot(local_down_axis);

	orientation_above_normal(0) = acos(abs(orientation_above_normal(0)));
	orientation_below_normal(0) = acos(abs(orientation_below_normal(0)));

	cout << "orientation_above_normal: " << endl << orientation_above_normal << endl;
	cout << "orientation_below_normal: " << endl << orientation_below_normal << endl;

	for (int ii = 0; ii < izone.num_poi; ii++)
	{	
		for (int i = 0; i < 3; i++)
		{
			izone.sample_mean_above_voxel(i,ii) = (int) floor(sample_mean_above(i,0)/MAP_RESOLUTION + 0.5f);
			izone.sample_mean_below_voxel(i,ii) = (int) floor(sample_mean_below(i,0)/MAP_RESOLUTION + 0.5f);
		}
	}

	for (int i = 0; i < izone.num_poi; i++)	
	{
		if (!isnan(orientation_above_normal(0)) && orientation_above_normal(0) < 0.125)
		{
			izone.ceiling_found[i] = 1;
			cout << "Found a possible ceiling" << endl;
			cout << "Voxel average location: " << endl << izone.sample_mean_above_voxel.col(i) << endl;
		}
		else
		{
			izone.ceiling_found[i] = 0;
		}
		if (!isnan(orientation_below_normal(0)) && orientation_below_normal(0) < 0.125)
		{
			izone.floor_found[i] = 1;
			cout << "Found a possible floor" << endl;
			cout << "Voxel average location: " << endl << izone.sample_mean_below_voxel.col(i) << endl;
		}
		else
		{
			izone.floor_found[i] = 0;
		}
	}

	for (int ii = 0; ii < izone.num_poi; ii++)
	{	
		if (izone.ceiling_found[ii])
		{		
			for (int i = 0; i < MAZEWIDTH; i++)
			{
				for (int j = 0; j < MAZEWIDTH; j++)
				{
					if (inferred_map[i][j][izone.sample_mean_above_voxel(0,ii)] != 3 && inferred_map[i][j][izone.sample_mean_above_voxel(0,ii)] != 1)
					{
						inferred_map[i][j][izone.sample_mean_above_voxel(0,ii)] = 5;
					}
				}
			}
		}
	}

	for (int ii = 0; ii < izone.num_poi; ii++)
	{	
		if (izone.floor_found[ii])
		{		
			for (int i = 0; i < MAZEWIDTH; i++)
			{
				for (int j = 0; j < MAZEWIDTH; j++)
				{
					if (inferred_map[i][j][izone.sample_mean_below_voxel(0,ii)] != 3 && inferred_map[i][j][izone.sample_mean_below_voxel(0,ii)] != 1)
					{
						inferred_map[i][j][izone.sample_mean_below_voxel(0,ii)] = 5;
					}
				}
			}
		}
	}	

}


bool firstsave = false;


// Member function of Planner class
// updatemap: this function updates the maze including changing occupancy status, computing heuristics, updating the start and goal points
// INPUTS: none
// OUTPUTS: boolean capturing success or failure of the update
bool Planner::updatemap()
{

	// Integers for iterators
  int d, l, x, y, z;
  int newx, newy, newz;

  // Floats storing the Euclidean distance between two cells, and the weighing function
  float root, kappa;

  // Reset mazeiteration to zero to indicate that the maze is being substantially changed
  mazeiteration = 0;

  // Set the iterator for the 1D voxel map array to zero
	l = 0;

	// Declare an output file stream for outputting some level (height-wise) of the map to a text file for verification
	ofstream outfile;

	// If we have not saved the map once yet, open the output file
	if (!firstsave)
	{

		outfile.open("Diagnostic_Logs/map_level.txt");

	} // if (!firstsave)

	// Pause until the flightstack interface thread has stopped writing to the map variables..
	while (pthread_mutex_trylock(&start_lock))
	{
		
		// Pause for 10 microseconds
		usleep(1);

	} // while (pthread_mutex_trylock(&start_lock))
	
	// Relinquish ownership of the lock
	pthread_mutex_unlock(&start_lock);	

	// Lock so LPA* thread can read the map variables.
	pthread_mutex_lock(&start_lock);

		// integers local to this function storing the start and goal coordinates in the voxel map 
  	starty = ystart; startx = xstart; startz = zstart;
			
	// Relinquish ownership of the lock			
	pthread_mutex_unlock(&start_lock);	

	// Pause until the flightstack interface thread has stopped writing to the map variables..
	while (pthread_mutex_trylock(&goal_lock))
	{
		
		// Pause for 10 microseconds
		usleep(10);

	} // while (pthread_mutex_trylock(&goal_lock))
	
	// Relinquish ownership of the lock
	pthread_mutex_unlock(&goal_lock);	

	// Lock so LPA* thread can read the map variables.
	pthread_mutex_lock(&goal_lock);

  	goaly = ygoal; goalx = xgoal; goalz = zgoal;
  	cout << "goalx: " << goalx  << " goaly: " << goaly << " goalz: " << goalz << endl;
	pthread_mutex_unlock(&goal_lock);	

	// Ensure that the start is in the boundary of the map
	if ( (startx >= MAZEDEPTH) || (startx < 0) || (starty >= MAZEWIDTH) || (starty < 0) || (startz >= MAZEHEIGHT) || (startz < 0) ) 
	{

		cout << "Start is outside of the map!" << endl;
		
		// Return failure
		return false;

	} // if ( (startx >= MAZEDEPTH) || (startx < 0) || (starty >= MAZEWIDTH) || (starty < 0) || (startz >= MAZEHEIGHT) || (startz < 0) )
	else if ( (goalx >= MAZEDEPTH) || (goalx < 0) || (goaly >= MAZEWIDTH) || (goaly < 0) || (goalz >= MAZEHEIGHT) || (goalz < 0) )
	{

		// Ensure that the goal is in the boundary of the map

		cout << "Goal is outside of the map!" << endl;
		if (!(prev_goalx <= 0 && prev_goaly <= 0 && prev_goalz <= 0))
		{
			goalx = prev_goalx; 
			goaly = prev_goaly; 
			goalz = prev_goalz; 
		}
		else
		{
			goalx = 0;
			goaly = 0;
			goalz = 0;
		}


		// Return failure
		// return false;

	} // else if ( (goalx >= MAZEDEPTH) || (goalx < 0) || (goaly >= MAZEWIDTH) || (goaly < 0) || (goalz >= MAZEHEIGHT) || (goalz < 0) )

	// goalx = 50;
	// goaly = 50;
	// goalz = 29;

	if (abs(startx - goalx) <= 1 && abs(starty - goaly) <= 1 && abs(startz - goalz) <= 1)
	{
		return false;
	}

	active_zones = 0;

	if (zgoalprev != goalz && ygoalprev != goaly && xgoalprev != goalx)
	{
		for (int i = 0; i < dzone_vec.size(); i++)
		{
			dzone_vec[i].active = 0;
		}
	}

	for (int i = 0; i < dzone_vec.size(); i++)
	{
		active_zones += dzone_vec[i].active;
	}
	
	// Check if the goal is potentially dangerous
	checkGoalSafety();

	// Attempt to obtain possession of the map_lock
	while (pthread_mutex_trylock(&map_lock))
	{

		// Wait 10 microseconds
		usleep(10);

	} // while (pthread_mutex_trylock(&map_lock))

	// Relinquish possession of the map_lock, immediately reobtain
	pthread_mutex_unlock(&map_lock);
	pthread_mutex_lock(&map_lock);	


		memcpy(map_full_copy, map_full, Height*Width*Depth*sizeof(int));
		// memcpy(inferred_map, map_full_copy, Height*Width*Depth*sizeof(int));

		// ceiling_floor_infer();

		changes = 0;
		int obs = 0;

		// Iterate over the maze height
		for (y = 0; y < MAZEWIDTH; ++y)
		{
			
			// Iterate over the maze width
			for (z = 0; z < MAZEHEIGHT; ++z)
    	{
					
				// Iterate over the maze depth
	  		for (x = 0; x < MAZEDEPTH; ++x)
		    {
		    		
      		voxel_map_changes[z][y][x] = false;

					// Record the occupancy status
					if (( map_full[x][y][z] == 3))
					{

	        	maze[z][y][x].obstacle = 1;
	      //   	obs++;
	      //   	if (maze_prev[z][y][x].obstacle != maze[z][y][x].obstacle)
	      //   	{
	      //   		voxel_map_changes[z][y][x] = true;
							// changes++;
	      //   	}
	      	}
	      	else
	      	{

	        	maze[z][y][x].obstacle = 0;
	      //   	if (maze_prev[z][y][x].obstacle != maze[z][y][x].obstacle)
	      //   	{
	      //   		voxel_map_changes[z][y][x] = true;
							// changes++;
	      //   	}
	      	}

	      	// if (z == izone.sample_mean_above_voxel(0,0) && izone.ceiling_found[0])
	      	// {
	      	// 	maze[z][y][x].inferred = 1;
	      	// }
	      	// else if (z == izone.sample_mean_below_voxel(0,0) && izone.ceiling_found[0])
	      	// {
	      	// 	maze[z][y][x].inferred = 1;
	      	// } 

          maze[z][y][x].danger = 0;
	        maze[z][y][x].x_weight = 1;
	        maze[z][y][x].y_weight = 1;
	        maze[z][y][x].z_weight = 5;

	        if (active_zones)
	        {

		        for (int i = 0; i < dzone_vec.size(); i++)
		        {
			        if (dzone_vec[i].active)
			        {
			        	int tempx, tempy, tempz;

			        	tempz = maze[z][y][x].z-dzone_vec[i].center[2]/MAP_RESOLUTION;
			        	tempy = maze[z][y][x].y-dzone_vec[i].center[1]/MAP_RESOLUTION;
			        	tempx = maze[z][y][x].x-dzone_vec[i].center[0]/MAP_RESOLUTION;
			        	if (abs(tempy) <= dzone.radius_in_voxels && abs(tempx) <= dzone.radius_in_voxels && abs(tempz) <= dzone_vec[i].height_in_voxels)
			        	{
			        		maze[z][y][x].danger = 1;
			        		maze[z][y][x].z_weight = 50000;
			        	}
			        }
		        }

	        }


		    	// Calculate the radicand of the Euclidean-distance between the goal cell and the cell at x,y,z	
	        root = (float) maze[z][y][x].z_weight*(z-goalz)*(z-goalz) + (y-goaly)*(y-goaly) + (x-goalx)*(x-goalx);	        

	        if (maze[z][y][x].obstacle)
	        {
  	  			maze[z][y][x].h = LARGE*LARGE;
	        }
	        else
	        {
	        	
	        	int floor_weight = params.mu1;

        		for (int iii = 1; iii <= params.mu1; iii++)
        		{      			
		        	if (z-iii < 0)
		        	{
		        		floor_weight = 1;
		        		break;
		        	}
		        	else
		        	{
		        		if (maze[z-iii][y][x].obstacle)
		        		{
		        			floor_weight = 1.5;
		        		}
		        	}

        		}

	       //  	// Compute the heuristic function
  	  			maze[z][y][x].h = floor_weight*params.mu3*sqrt(root);
							        
	        }

  				maze[z][y][x].g = LARGE;
  				maze[z][y][x].rhs = 0;
  	  		
  	  		// Compute the mazeiteration (should be zero)
					maze[z][y][x].iteration = mazeiteration;

	        maze[z][y][x].searchtree = NULL;

	        // If the cell at x,y,z is on the boundary of the map, set it to occupied
	        if (z == 0 || y == 0 || x == 0 || x == 99 || y == 99 || z == 29)
	        {

	        	maze[z][y][x].obstacle = 1;

	        } // if (z == 0 || y == 0 || x == 0 || x == 99 || y == 99 || z == 29)

	        // Increment the index for the 1D voxel map array
	        l++;

	        // if x,y,z are not on the boundaries
	        if ( (x < MAZEDEPTH - 1) && (y < MAZEWIDTH - 1) && (z < MAZEHEIGHT - 1) )
	        {   

	        	// Iterate over the number of directions
						for (d = 0; d < DIRECTIONS; ++d)
						{

							// Set the edges of the graph to NULL pointers
							maze[z][y][x].move[d] = NULL;

						} // for (d = 0; d < DIRECTIONS; ++d)

      	  } // if ( (x < MAZEDEPTH - 1) && (y < MAZEWIDTH - 1) && (z < MAZEHEIGHT - 1) )

      	  // maze_prev[z][y][x].obstacle = maze[z][y][x].obstacle;

	    	} // for (x = 0; x < MAZEDEPTH; ++x)

			} // for (y = 0; y < MAZEWIDTH; ++y)

		} // for (z = 0; z < MAZEHEIGHT; ++z)

		// cout << "obs: " << obs << endl;
		// cout << "changes: " << changes << endl;	

	// Relinquish possession of the map_lock
	pthread_mutex_unlock(&map_lock);

	// std::cout << "Start: " << xstart << ", " << ystart << ", " << zstart << " [voxels]" << endl;
	// std::cout << "Goal: " << xgoal << ", " << ygoal << ", " << zgoal << " [voxels]" << endl;
	// std::cout << "Start: " << xstart * MAP_RESOLUTION << ", " << ystart * MAP_RESOLUTION << ", " << zstart * MAP_RESOLUTION << " [meters]" << endl;
	// std::cout << "Goal: " << xgoal * MAP_RESOLUTION << ", " << ygoal * MAP_RESOLUTION << ", " << zgoal * MAP_RESOLUTION << " [meters]" << endl;

	// If the start cell is an obstacle
	if (maze[startz][starty][startx].obstacle)
	{

		// Alert the user
		cout << "Start is occupied!" << endl;
		bool foundUnoccupiedStart = false;
		// Find closest unoccupied voxel
		for (int dd = 1; dd <= 5; dd++)
		{
			if (foundUnoccupiedStart)
			{
				break;
			}

			for (d = 0; d < DIRECTIONS; d++)
		  {

		  	// Compute temporary cell position
		    newz = startz + dd*dz[d];
		    newy = starty + dd*dy[d];
		    newx = startx + dd*dx[d];

		    // If temp cell position is feasible
		    if (newz >= 0 && newz < MAZEHEIGHT && newy >= 0 && newy < MAZEWIDTH && newx >= 0 && newx < MAZEDEPTH && !maze[newz][newy][newx].obstacle)
		    {

		    	// Set the edge in the graph
		    	startz = newz;
		    	starty = newy;
		    	startx = newx;
		    	root = (float) (startz-goalz)*(startz-goalz) + (starty-goaly)*(starty-goaly) + (startx-goalx)*(startx-goalx);
					maze[startz][starty][startx].h = params.mu3*sqrt(root);
					foundUnoccupiedStart = true;
					cout << "new start: " << startx << ", " << starty << "," << startz << endl;
					break; 

		  	} // if (newz >= 0 && newz < MAZEHEIGHT && newy >= 0 && newy < MAZEWIDTH && newx >= 0 && newx < MAZEDEPTH && !maze[newz][newy][newx].obstacle)

			}
		}

		if (!foundUnoccupiedStart)
		{
			cout << "Could not find nearby unoccupied voxel to take the start." << endl;
			return false;
		}

	} // if (maze[startz][starty][startx].obstacle)
		
	// Set the mazestart
	mazestart = &maze[startz][starty][startx];
	
	mazegoal = &maze[goalz][goaly][goalx];

	// Set the mazegoal's heuristic to zero (it is always zero)
	mazegoal->h = 0;

	// Set the mazegoal
	if (maze[goalz][goaly][goalx].obstacle)
	{
		cout << "Goal is occupied!" << endl;
		return false;
	}

	// Record the previous goal for upstream code
	xgoalprev = goalx;
	ygoalprev = goaly;
	zgoalprev = goalz;

	// Iterate over the maze height
	for (z = 0; z < MAZEHEIGHT; ++z)
	{  

		// Iterate over the maze width
		for (y = 0; y < MAZEWIDTH; ++y)
		{
			
			// Iterate over the maze depth
			for (x = 0; x < MAZEDEPTH; ++x)
			{

				// If the cell is not an obstacle
      	if (!maze[z][y][x].obstacle)
      	{

      		// If the x,y,z position is feasible
      		if ( (x > 0 && y > 0 && z > 0) && (x < MAZEDEPTH-1 && y < MAZEWIDTH-1 && z < MAZEHEIGHT-1) )
      		{

      			// If adjacent voxels are not occupied
      			if ( (( ( (!maze[z][y][x-1].obstacle || !maze[z][y-1][x].obstacle) && !maze[z][y-1][x-1].obstacle) || ( (!maze[z][y][x-1].obstacle || !maze[z][y+1][x].obstacle) && !maze[z][y+1][x-1].obstacle) || ( (!maze[z][y][x+1].obstacle || !maze[z][y-1][x].obstacle) && !maze[z][y-1][x+1].obstacle) || ( (!maze[z][y][x+1].obstacle || !maze[z][y+1][x].obstacle) && !maze[z][y+1][x+1].obstacle) || !maze[z][y][x+1].obstacle || !maze[z][y][x-1].obstacle || !maze[z][y+1][x].obstacle || !maze[z][y-1][x].obstacle )) && !maze[z][y+1][x+1].obstacle && !maze[z][y-1][x+1].obstacle && !maze[z][y+1][x-1].obstacle && !maze[z][y-1][x-1].obstacle && !maze[z+1][y+1][x+1].obstacle && !maze[z-1][y+1][x+1].obstacle && !maze[z+1][y-1][x+1].obstacle && !maze[z-1][y-1][x+1].obstacle && !maze[z+1][y+1][x-1].obstacle && !maze[z-1][y+1][x-1].obstacle && !maze[z-1][y+1][x].obstacle && !maze[z+1][y+1][x].obstacle && !maze[z-1][y][x-1].obstacle && !maze[z+1][y][x-1].obstacle && !maze[z-1][y][x+1].obstacle && !maze[z+1][y][x+1].obstacle && !maze[z+1][y+1][x+1].obstacle && !maze[z+1][y-1][x+1].obstacle && !maze[z+1][y+1][x-1].obstacle && !maze[z+1][y-1][x-1].obstacle && !maze[z-1][y+1][x+1].obstacle && !maze[z-1][y-1][x+1].obstacle && !maze[z-1][y+1][x-1].obstacle && !maze[z-1][y-1][x-1].obstacle)
      			{

      				// Iterate over the number of directions
							for (d = 0; d < DIRECTIONS; ++d)
						  {

						  	// Compute temporary cell position
						    newz = z + dz[d];
						    newy = y + dy[d];
						    newx = x + dx[d];

						    // If temp cell position is feasible
						    if (newz >= 0 && newz < MAZEHEIGHT && newy >= 0 && newy < MAZEWIDTH && newx >= 0 && newx < MAZEDEPTH && !maze[newz][newy][newx].obstacle)
						    {

						    	// Set the edge in the graph
						      maze[z][y][x].move[d] = &maze[newz][newy][newx];

						  	} // if (newz >= 0 && newz < MAZEHEIGHT && newy >= 0 && newy < MAZEWIDTH && newx >= 0 && newx < MAZEDEPTH && !maze[newz][newy][newx].obstacle)

		      		} // for (d = 0; d < DIRECTIONS; ++d)

    				} // if ( (( ( (!maze[z][y][x-1].obstacle || !maze[z][y-1][x].obstacle) && !maze[z][y-1][x-1].obstacle) || ( (!maze[z][y][x-1].obstacle || !maze[z][y+1][x].obstacle) && !maze[z][y+1][x-1].obstacle) || ( (!maze[z][y][x+1].obstacle || !maze[z][y-1][x].obstacle) && !maze[z][y-1][x+1].obstacle) || ( (!maze[z][y][x+1].obstacle || !maze[z][y+1][x].obstacle) && !maze[z][y+1][x+1].obstacle) || !maze[z][y][x+1].obstacle || !maze[z][y][x-1].obstacle || !maze[z][y+1][x].obstacle || !maze[z][y-1][x].obstacle )) && !maze[z][y+1][x+1].obstacle && !maze[z][y-1][x+1].obstacle && !maze[z][y+1][x-1].obstacle && !maze[z][y-1][x-1].obstacle && !maze[z+1][y+1][x+1].obstacle && !maze[z-1][y+1][x+1].obstacle && !maze[z+1][y-1][x+1].obstacle && !maze[z-1][y-1][x+1].obstacle && !maze[z+1][y+1][x-1].obstacle && !maze[z-1][y+1][x-1].obstacle && !maze[z-1][y+1][x].obstacle && !maze[z+1][y+1][x].obstacle && !maze[z-1][y][x-1].obstacle && !maze[z+1][y][x-1].obstacle && !maze[z-1][y][x+1].obstacle && !maze[z+1][y][x+1].obstacle && !maze[z+1][y+1][x+1].obstacle && !maze[z+1][y-1][x+1].obstacle && !maze[z+1][y+1][x-1].obstacle && !maze[z+1][y-1][x-1].obstacle && !maze[z-1][y+1][x+1].obstacle && !maze[z-1][y-1][x+1].obstacle && !maze[z-1][y+1][x-1].obstacle && !maze[z-1][y-1][x-1].obstacle)

      		} // if ( (x > 0 && y > 0 && z > 0) && (x < MAZEDEPTH-1 && y < MAZEWIDTH-1 && z < MAZEHEIGHT-1) )
      		
      	}
    		else
    		{

    			// Iterate over the number of directions
					for (d = 0; d < DIRECTIONS; ++d)
				  {

				  	// Compute temporary cell position (neighbor)
				    newz = z + dz[d];
				    newy = y + dy[d];
				    newx = x + dx[d];

				    // If temporary cell position is feasible
				    if (newz >= 0 && newz < MAZEHEIGHT && newy >= 0 && newy < MAZEWIDTH && newx >= 0 && newx < MAZEDEPTH && !maze[newz][newy][newx].obstacle)
				    {

				    	// Set the edge in the graph
				      maze[z][y][x].move[d] = &maze[newz][newy][newx];

				  	} // if (newz >= 0 && newz < MAZEHEIGHT && newy >= 0 && newy < MAZEWIDTH && newx >= 0 && newx < MAZEDEPTH && !maze[newz][newy][newx].obstacle)

      		} // for (d = 0; d < DIRECTIONS; ++d)

				} // if (!maze[z][y][x].obstacle)

			} // for (x = 0; x < MAZEDEPTH; ++x)

		} // for (y = 0; y < MAZEWIDTH; ++y)

	} // for (z = 0; z < MAZEHEIGHT; ++z)

	prev_goalx = goalx;
	prev_goaly = goaly;
	prev_goalz = goalz;

	// Return success
	return true;

} // bool Planner::updatemap()


// Local method
// lpastarinitialize: this function initializes the planner search tree and heap
// INPUTS: none
// OUTSPUTS: none
void lpastarinitialize()
{
  
	// Pre-increment mazeiteration (map should have been updated before calling this function)
  ++mazeiteration;

  // Clear the heap
  emptyheap();

  // Set the mazestart cost-to-come to infinity
  mazestart->g = LARGE;

  // Set the mazestart cost-to-come to zero (note that this makes the mazestart locally inconsistent, and hence, will be expanded by the LPA* algorithm)
  mazestart->rhs = 0;

  // Set the mazestart's key
  mazestart->key = mazestart->h * BASE;
  // mazestart->key = mazestart->h;

  // Clear the search tree
  mazestart->searchtree = NULL;

  // Set the mazeiteration
  mazestart->iteration = mazeiteration;

  // Insert the mazestart cell into the heap (it is locally inconsistent)
  insertheap(mazestart);

  // Set the mazegoal cost-to-come to infinity
  mazegoal->g = LARGE;

  // Set the mazegoal RHS value to infinity (locally consistent)
  mazegoal->rhs = LARGE;

  // Clear the search tree
  mazegoal->searchtree = NULL;

  // Set the mazeiteration
  mazegoal->iteration = mazeiteration;

} // void lpastarinitialize()


// Local method
// initializecell: this function initializes a cell in the maze. Note that 
// cells are only initialized when they are a neighbor of an expanded cell
// INPUTS: pointer to a cell
// OUTPUTS: none
void initializecell(cell *thiscell)
{

  if (thiscell->iteration != mazeiteration)
  {
    
  	// Set thiscell's cost-to-come to infinity
    thiscell->g = LARGE;
    
  	// Set thiscell's RHS value to infinity
    thiscell->rhs = LARGE;
    
  	// Clear thiscell's searchtree
    thiscell->searchtree = NULL;
    
  	// Set the mazeiteration
    thiscell->iteration = mazeiteration;

  } // if (thiscell->iteration != mazeiteration)

} // void initializecell(cell *thiscell)


// Local method
// lpastarupdatecell: updates a cell's key if the cell is locally inconsistent and
// adds the cell to the heap, otherwise, removes the cell from the heap
// INPUTS: pointer to a cell
// OUTPUTS: none
void lpastarupdatecell(cell *thiscell)
{
  
  // If the cost-to-come does not match the RHS value (local inconsistency)
  if (thiscell->g != thiscell->rhs)
  {
    
  	// Set the key
    thiscell->key = (min(thiscell->g, thiscell->rhs) + thiscell->h) * BASE + min(thiscell->g, thiscell->rhs);
    
    // Insert the cell into the heap
    insertheap(thiscell);

  } // if (thiscell->g != thiscell->rhs)
  else 
  {
    
  	// Remove the cell from the heap 
    deleteheap(thiscell);

  } // if (thiscell->g != thiscell->rhs)

} // void lpastarupdatecell(cell *thiscell)


// local method
// lpastarupdaterhs: updates the RHS value of the cell, clears the search tree,
// INPUTS: pointer to a cell, float representing the edge cost
// OUTPUTS: none
void lpastarupdaterhs(cell *thiscell, float c)
{

	// Declare iterator
  int d;

  // Set thiscell's RHS value to infinity
  thiscell->rhs = LARGE;

  // Clear thiscell's searchtree
  thiscell->searchtree = NULL;
  
  // Iterate over the number of DIRECTIONS
  for (d = 0; d < DIRECTIONS; ++d)
  {

  	// If there exists a dth edge, mazeiteration is compatible, and thiscell's RHS value is greater than the cost-to-come plus the edge cost 
		if (thiscell->move[d] != NULL && thiscell->move[d]->iteration == mazeiteration && thiscell->rhs > thiscell->move[d]->g + c)
	  {

			thiscell->rhs = thiscell->move[d]->g + c; 

			thiscell->searchtree = thiscell->move[d];

	  } // if (thiscell->move[d] != NULL && thiscell->move[d]->iteration == mazeiteration && thiscell->rhs > thiscell->move[d]->g + c)

  } // for (d = 0; d < DIRECTIONS; ++d)

  // Update thiscell
  lpastarupdatecell(thiscell);

} // void lpastarupdaterhs(cell *thiscell, float c)


// Member function of Planner class
// lpastarcomputeshortestpath: executes the core of the LPA* algorithm, computing a shortest path
// INPUTS: none
// OUTPUTS: none
void Planner::lpastarcomputeshortestpath() 
{

	// Declare pointers to temporary cells
  cell *tmpcell1, *tmpcell2, *tmpcell3;

  // Declare integer for iterate
  int d;

  // Floats storing the weighing function
  float kappa, root, kappa_sqrt;

  string message = "<LPA* THREAD> Computing 'shortest' path.";
	log.writeToLog(message);
	// cout << message << endl;

	if ( topheap() == NULL )
	{
		insertheap(mazestart);
	}
	bool temp = mazegoal->rhs > mazegoal->g;
	cout << "BEFORE" << endl;
	cout << "mazegoal->rhs > mazegoal->g: " << temp << endl;
	temp = topheap() != NULL;
	cout << "topheap(heapsize,heap) != NULL: " << temp << endl;
	if (temp)
	{
		temp = topheap()->key < min(mazegoal->g, mazegoal->rhs) * BASE + min(mazegoal->g, mazegoal->rhs);
		cout << "last: " << temp << endl;
	}

	float z_weight, z_weight2, y_weight, y_weight2, x_weight, x_weight2;

	// While the goal's rhs is greater than the cost-to-come or the heap contains a cell AND that cell's key is smaller than ...
	while (mazegoal->rhs != mazegoal->g || (topheap() != NULL && topheap()->key < min(mazegoal->g, mazegoal->rhs) * BASE + min(mazegoal->g, mazegoal->rhs)))
  {

  	// If it's time to stop searching (if the search is taking too long)
		if (time_to_stop)
		{
			cout << "time_to_stop: " << time_to_stop << endl;

			return;

		} // if (time_to_stop)

		// Set the temp cell to the cell at the top of the heap (expanded cell)
  	tmpcell1 = topheap();
  
  	// If the cell's cost-to-come is greater than the RHS value (locally overconsistent)
  	if (tmpcell1->g > tmpcell1->rhs)
		{
			// Set the cost-to-come to the RHS value
  		tmpcell1->g = tmpcell1->rhs;
  		
  		// Remove the cell from the heap (last line makes the cell locally consistent)
  		deleteheap(tmpcell1);
			
  		// Iterate over the number of directions (which lead to the neighbors)
			for (d = 0; d < DIRECTIONS; ++d)
    	{

    		// If the neighbor exists
    		if (tmpcell1->move[d] != NULL)
      	{
					
					// Set the second cell to the expanded cell's neighbor
					tmpcell2 = tmpcell1->move[d];
					tmpcell3 = tmpcell1->move[d];

					// Initialize the second cell
					initializecell(tmpcell2);

					for (int dd = 1; dd <= d_stride; dd++)
					{

						
						if (dd > 1)
						{

			    		if (tmpcell3->move[d] != NULL)
							{
								tmpcell3 = tmpcell3->move[d];
							} // if (tmpcell3->move[d] != NULL)

						} // if (dd > 1)

						initializecell(tmpcell3);

						z_weight = maze[tmpcell3->z][tmpcell3->y][tmpcell3->x].z_weight;
						y_weight = maze[tmpcell3->z][tmpcell3->y][tmpcell3->x].y_weight;
						x_weight = maze[tmpcell3->z][tmpcell3->y][tmpcell3->x].x_weight;
						
						z_weight2 = maze[tmpcell2->z][tmpcell2->y][tmpcell2->x].z_weight;
						y_weight2 = maze[tmpcell2->z][tmpcell2->y][tmpcell2->x].y_weight;
						x_weight2 = maze[tmpcell2->z][tmpcell2->y][tmpcell2->x].x_weight;

						if (tmpcell3 != NULL)
						{

							// Compute the radicand
							// root = (float) 10*(tmpcell2->z - tmpcell1->z)*(tmpcell2->z - tmpcell1->z) + (tmpcell2->y - tmpcell1->y)*(tmpcell2->y - tmpcell1->y) + (tmpcell2->x - tmpcell1->x)*(tmpcell2->x - tmpcell1->x);
							root = (float) z_weight*(tmpcell3->z - tmpcell1->z)*(tmpcell3->z - tmpcell1->z) + y_weight*(tmpcell3->y - tmpcell1->y)*(tmpcell3->y - tmpcell1->y) + x_weight*(tmpcell3->x - tmpcell1->x)*(tmpcell3->x - tmpcell1->x);
							
							// Get the weighing function
							// kappa = getKappa(tmpcell2->x,tmpcell2->y,tmpcell2->z);
							kappa = getKappa(tmpcell3->x,tmpcell3->y,tmpcell3->z);

							// Compute the edge cost
							kappa_sqrt = kappa*sqrt(root);

							tmpcell3->kappa_sqrt = kappa_sqrt;

							// If the neighbor is not the start, is not an obstacle, and whose RHS value is less than the cost-to-come plus the edge cost of the expanded cell
							// if (tmpcell2 != mazestart && !tmpcell2->obstacle && tmpcell2->z > 0 && tmpcell2->rhs > tmpcell1->g + kappa_sqrt)
							if (tmpcell3 != mazestart && !tmpcell3->obstacle && tmpcell3->z > 0 && tmpcell3->rhs > tmpcell1->g + kappa_sqrt)
							{
								
								// Set the RHS value
								tmpcell2 = tmpcell3;
								tmpcell2->rhs = tmpcell1->g + kappa_sqrt;
								
								// Set the predecessor of the neighbor (the predecessor is the expanded cell)
								tmpcell2->searchtree = tmpcell1;
								
								// update the neighbor
								lpastarupdatecell(tmpcell2);

							} // if (tmpcell2 != mazestart && !tmpcell2->obstacle && tmpcell2->z > 0 && tmpcell2->rhs > tmpcell1->g + kappa_sqrt)

						}
						else
						{

							root = (float) z_weight2*(tmpcell2->z - tmpcell1->z)*(tmpcell2->z - tmpcell1->z) + y_weight2*(tmpcell2->y - tmpcell1->y)*(tmpcell2->y - tmpcell1->y) + x_weight2*(tmpcell2->x - tmpcell1->x)*(tmpcell2->x - tmpcell1->x);
							kappa = getKappa(tmpcell2->x,tmpcell2->y,tmpcell2->z);
							kappa_sqrt = kappa*sqrt(root);

							tmpcell2->kappa_sqrt = kappa_sqrt;

							if (tmpcell2 != mazestart && !tmpcell2->obstacle && tmpcell2->z > 0 && tmpcell2->rhs > tmpcell1->g + kappa_sqrt)
							{
								
								// Set the RHS value
								tmpcell2->rhs = tmpcell1->g + kappa_sqrt;
								
								// Set the predecessor of the neighbor (the predecessor is the expanded cell)
								tmpcell2->searchtree = tmpcell1;
								
								// update the neighbor
								lpastarupdatecell(tmpcell2);

							}

						}
					
					}

      	} // if (tmpcell1->move[d] != NULL)

    	} // for (d = 0; d < DIRECTIONS; ++d)

		} // if (tmpcell1->g > tmpcell1->rhs)
		else
		{

			// Otherwise if the cell is locally consistent or underconsistent

			// Set the cell's cost-to-come to infinity
  		tmpcell1->g = LARGE*LARGE;

  		// Update the cell
  		lpastarupdatecell(tmpcell1);

  		// Iterate over the number of directions
  		for (d = 0; d < DIRECTIONS; ++d)
  		{ 

  			// If the dth neighbor exists
    		if (tmpcell1->move[d] != NULL)
    		{
						
  				// Set the neighbor
					tmpcell2 = tmpcell1->move[d];
 					tmpcell3 = tmpcell1->move[d];

        	// Initialize the neighbor
        	initializecell(tmpcell2);

					for (int dd = 1; dd <= d_stride; dd++)
					{
     	

						if (dd > 1)
						{

			    		if (tmpcell3->move[d] != NULL)
							{
								tmpcell3 = tmpcell3->move[d];
							} // if (tmpcell3->move[d] != NULL)

						} // if (dd > 1)

						initializecell(tmpcell3);

						z_weight = maze[tmpcell3->z][tmpcell3->y][tmpcell3->x].z_weight;
						y_weight = maze[tmpcell3->z][tmpcell3->y][tmpcell3->x].y_weight;
						x_weight = maze[tmpcell3->z][tmpcell3->y][tmpcell3->x].x_weight;
						
						z_weight2 = maze[tmpcell2->z][tmpcell2->y][tmpcell2->x].z_weight;
						y_weight2 = maze[tmpcell2->z][tmpcell2->y][tmpcell2->x].y_weight;
						x_weight2 = maze[tmpcell2->z][tmpcell2->y][tmpcell2->x].x_weight;

						if (tmpcell3 != NULL)
						{						
						
							// Compute the radicand
							// root = (float) 10*(tmpcell2->z - tmpcell1->z)*(tmpcell2->z - tmpcell1->z) + (tmpcell2->y - tmpcell1->y)*(tmpcell2->y - tmpcell1->y) + (tmpcell2->x - tmpcell1->x)*(tmpcell2->x - tmpcell1->x);
							root = (float) z_weight*(tmpcell3->z - tmpcell1->z)*(tmpcell3->z - tmpcell1->z) + y_weight*(tmpcell3->y - tmpcell1->y)*(tmpcell3->y - tmpcell1->y) + x_weight*(tmpcell3->x - tmpcell1->x)*(tmpcell3->x - tmpcell1->x);
							
							// Get the weighing function
							// kappa = getKappa(tmpcell2->x,tmpcell2->y,tmpcell2->z);
							kappa = getKappa(tmpcell3->x,tmpcell3->y,tmpcell3->z);
		    	  	
		    	  	// Compute the edge cost
		    	  	kappa_sqrt = kappa*sqrt(root);

		    	  	tmpcell3->kappa_sqrt = kappa_sqrt;
							
							// If the neighbor is not the start, is a part of the search tree, and is not an obstacle
							// if (tmpcell2 != mazestart && tmpcell2->searchtree == tmpcell1 && !tmpcell2->obstacle)
							if (tmpcell3 != mazestart && tmpcell3->searchtree == tmpcell1 && !tmpcell3->obstacle)
							{

								// Update the RHS value
		  					// lpastarupdaterhs(tmpcell2,kappa_sqrt);
		  					lpastarupdaterhs(tmpcell3,kappa_sqrt);

							} // if (tmpcell2 != mazestart && tmpcell2->searchtree == tmpcell1 && !tmpcell2->obstacle)
	    		
						}
						else
						{
							root = (float) z_weight2*(tmpcell2->z - tmpcell1->z)*(tmpcell2->z - tmpcell1->z) + y_weight2*(tmpcell2->y - tmpcell1->y)*(tmpcell2->y - tmpcell1->y) + x_weight2*(tmpcell2->x - tmpcell1->x)*(tmpcell2->x - tmpcell1->x);
							kappa = getKappa(tmpcell2->x,tmpcell2->y,tmpcell2->z);
		    	  	kappa_sqrt = kappa*sqrt(root);

		    	  	tmpcell2->kappa_sqrt = kappa_sqrt;

							if (tmpcell2 != mazestart && tmpcell2->searchtree == tmpcell1 && !tmpcell2->obstacle)
							{

								// Update the RHS value
		  					lpastarupdaterhs(tmpcell2,kappa_sqrt);

							} // if (tmpcell2 != mazestart && tmpcell2->searchtree == tmpcell1 && !tmpcell2->obstacle)
	    								
						}

	    		}

				} // if (tmpcell1->move[d] != NULL)
    	
    	} // for (d = 0; d < DIRECTIONS; ++d)

		} // if (tmpcell1->g > tmpcell1->rhs)

  } // while (mazegoal->rhs > mazegoal->g || (topheap() != NULL && topheap()->key < min(mazegoal->g, mazegoal->rhs) * BASE + min(mazegoal->g, mazegoal->rhs)))

  cout << "AFTER" << endl;
temp = mazegoal->rhs > mazegoal->g;
	cout << "mazegoal->rhs > mazegoal->g: " << temp << endl;
	temp = topheap() != NULL;
	cout << "topheap(heapsize,heap) != NULL: " << temp << endl;
	if (temp)
	{
		temp = topheap()->key < min(mazegoal->g, mazegoal->rhs) * BASE + min(mazegoal->g, mazegoal->rhs);
		cout << "last: " << temp << endl;
	}

} // void Planner::lpastarcomputeshortestpath()

// Member function of Planner class
// testlpastar: this function is responsible for calling the core LPA* search and recording the resulting path
// INPUTS: none
// OUTPUTS: none
void Planner::testlpastar()
{

	// Integer for an iterator
	int d;

	// Pointer to a cell
	cell *tmpcell1;

	// Float representing the total objective function value
	float objectiveValue = 0.0;

	// String storing messages to be written to logs or output to the terminal
	string message;

	// Compute the shortest path
	lpastarcomputeshortestpath();

	bool potential_collision = false;
	int potential_collision_index = -1;

	// ****************************************************************** //
	// This block traces the search tree from the goal back to the start, //
	// computes the objective function value and write the path to a file //
	// ****************************************************************** //

	// If the mazegoal's RHS value is not infinity (implying that it has been reached)
	cout << "mazegoal->rhs: " << mazegoal->rhs<< endl;
	cout << "mazegoal rhs: " << (mazegoal->rhs != LARGE) << endl;
	if (mazegoal->rhs != LARGE)
	{

		// Set the cell pointer to the goal
		tmpcell1 = mazegoal;

		// Attempt to obtain possession of the path_lock
		while(pthread_mutex_trylock(&path_lock))
		{

			// Wait 10 microseconds 
			usleep(10);

		} // while(pthread_mutex_trylock(&path_lock))

		// Relinquish possession of the path_lock, immediately, reobtain
		pthread_mutex_unlock(&path_lock);
		pthread_mutex_lock(&path_lock);

			// Clear the path vector 
			newpath.clear();
			log_path.writeToLog("Path:");

			// ****************************************************** //
			// Fill the path vector and output the waypoints to a log //
			// ****************************************************** //

			planSuccess = true;
			// Indicate that a new path has been computed
			newPath = 1;

			// Iterate until the start is traced back from the goal
			for (d = 0; tmpcell1 != mazestart; ++d, tmpcell1 = tmpcell1->searchtree)
			{
				
				printf(">> %.1f, %.1f, %.1f \n", (tmpcell1->x)*MAP_RESOLUTION,(tmpcell1->y)*MAP_RESOLUTION,(tmpcell1->z)*MAP_RESOLUTION);

				// Add waypoint to the path vector
				newpath.push_back((tmpcell1->x)*MAP_RESOLUTION);
				newpath.push_back((tmpcell1->y)*MAP_RESOLUTION);
				newpath.push_back((tmpcell1->z)*MAP_RESOLUTION);

				// Increment the objective value
				objectiveValue += tmpcell1->rhs + tmpcell1->h;

				if (d > 1000)
				{
					cout << "Planner failed to trace back from the goal." << endl;
					planSuccess = false;
					newPath = 0;
					break;
				}
				// else if (izone.ceiling_found[0] && maze[tmpcell1->z][tmpcell1->y][tmpcell1->x].inferred)
				// else if (maze[tmpcell1->z][tmpcell1->x][tmpcell1->y].obstacle)
				// {
				// 	if (~potential_collision)
				// 	{
				// 		cout << "Path traverses through a voxel with inferred status, potential collision detected." << endl;
				// 		cout << "Path will terminate at next voxel which is not inferred" << endl;
				// 		potential_collision = true;
				// 	}
				// 	potential_collision_index = d;
				// }


			} // for (d = 0; tmpcell1 != mazestart; ++d, tmpcell1 = tmpcell1->searchtree)

			// Add the start to the path vector
			newpath.push_back(mazestart->x*MAP_RESOLUTION);
			newpath.push_back(mazestart->y*MAP_RESOLUTION);
			newpath.push_back(mazestart->z*MAP_RESOLUTION);

			// if (potential_collision)
			// {
			// 	newpath.erase(newpath.begin(),newpath.begin()+d);
			// }

			// Write the waypoint to a log
			if (planSuccess)
			{	
				for (int i = 0; i < newpath.size()/3; i++)
				{
					// printf(">> %.1f, %.1f, %.1f \n", newpath[3*i],newpath[3*i+1],newpath[3*i+2]);
					message = to_string(i) + ": " + to_string(newpath[3*i]) + "," + to_string(newpath[3*i+1]) + "," + to_string(newpath[3*i+2]);
					log_path.writeToLog(message,1);
				}
			}

			cout << "newpath.size() 1: " << newpath.size()<< endl;
			// Compute the number of waypoints
			pathSize_f = floor(newpath.size()/3);

		// Relinquish possession of the path_lock
		pthread_mutex_unlock(&path_lock);

	} // if (mazegoal->rhs != LARGE)
	else
	{
		planSuccess = false;
		newPath = 0;
		cout << "<LPA*> Did not reach goal (despite it not being occupied), reseting the maze." << endl;
	}

} // void Planner::testlpastar()


// Local method
// lpa_timing_thread: This thread runs a timer, if the lpastar search takes too long (as measured by the timer in this thread)
// then the lpastar search is terminated, and this thread resets once the next call to the lpastar search is executed
// INPUTS: void pointer to some object (no needed)
// OUTPUTS: void pointer
void* lpa_timing_thread(void *args)
{

	// Timing variables
	auto current_time = std::chrono::high_resolution_clock::now();
	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed = end_time - current_time;

	// cout <<" Started timing thread!" << endl;

	// While loop that ends when LPA* finds a path or the timer expires 
	while(!terminate_timing_thread)
	{

		// Update the time
		end_time = std::chrono::high_resolution_clock::now();
		
		// Compute the elapsed time
		timeElapsed = end_time - current_time;
		
		// If the timer has expired (10 seconds have passed)
		if (std::chrono::duration_cast<chrono::seconds>(timeElapsed).count() > 80)
		{
			cout << "Time to stop LPA*, taking too long!" << endl;
			
			// Signal to end the while loop
			terminate_timing_thread = true;
			
			// Signal to LPA* search process to end
			time_to_stop = true;
			earlyTermination = true;

		} // if (std::chrono::duration_cast<chrono::seconds>(timeElapsed).count() > 10)
	
	} // while(!terminate_timing_thread)

	// cout << "Ending timing thread!" << endl;

	return NULL;

} // void* lpa_timing_thread(void *args)


// Member function of Planner class
// LPAstar_thread: this thread continuously runs the LPA* search algorithm
// INPUTS: none
// OUTPUTS: none
void Planner::LPAstar_thread()
{

	// Integer capturing success/failure of writing to log
	int writeStatus = 0;

	// String storing messages to write to logs or output to termina;
	string message;

	// boolean capturing success of map update
	bool success;

	// While loop which pauses the thread until all necessary information is gathered
	while(!firstPassComplete)
	{
		
		// Wait 10 microseconds
		usleep(10);

	} // while(!firstPassComplete)

	// usleep(100000000);
	
	std::cout << "<LPA* THREAD> Started LPA* thread" << std::endl;
	writeStatus = log.writeToLog("<LPA* THREAD> Started LPA* thread");

	// Timing variables
	auto current_time = std::chrono::high_resolution_clock::now();
	auto current_time1 = std::chrono::high_resolution_clock::now();
	auto end_time = std::chrono::high_resolution_clock::now();
	auto end_time1 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed = end_time - current_time;
	std::chrono::duration<double> timeElapsed1 = end_time1 - current_time1;
	double waitTime, sleepTime = 750000;
	
	// Pthread ID for the timing thread
	pthread_t lpa_timing_tid;

	// Temporary cell positions
	unsigned short int* nx = new unsigned short int[1];
	unsigned short int* ny = new unsigned short int[1];
	unsigned short int* nz = new unsigned short int[1]; 

	// Boolean capturing if the last computed path collides with an obstacle in the latest map
	bool pathCollision = true;

	int newx = 0, newy = 0, newz = 0;
	float root = 0.0, kappa = 0.0, kappa_sqrt = 0.0, min_f = 1000000000.0;
	cell* min_cell;

	// While loop that ends when it is time to end the program
	while(!exit_thread)
	{

		// Wait 100000 microseconds
		if (firstPassComplete)
		{
			if (planSuccess)
			{
				timeElapsed1 = end_time - current_time1;
				waitTime = sleepTime-(std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed1)).count();
				if (waitTime > 0)
				{
					usleep(waitTime);
				}
			}
			else
			{
				usleep(sleepTime);
			}
		}

		// Update the map
		message = "<LPA* THREAD> Updating map.";
		log.writeToLog(message);
		// cout << message << endl;

		// Update the map
		success = updatemap();

		// Attempt to obtain possession of the path lock 
		while(pthread_mutex_trylock(&path_lock))
		{
			
			// Wait 10 microseconds
			usleep(10);

		} // while(pthread_mutex_trylock(&path_lock))
		
		// Relinquish possession of the path lock, immediately reobtain
		pthread_mutex_unlock(&path_lock);
		pthread_mutex_lock(&path_lock);		

			// If there is at least 1 waypoint
			if (newpath.size() > 0)
			{

				cout << "newpath.size() 2: " << newpath.size()<< endl;

				// cin.ignore();

				delete [] nx;
				delete [] ny;
				delete [] nz;

				nx = new unsigned short int[newpath.size()/3];
				ny = new unsigned short int[newpath.size()/3];
				nz = new unsigned short int[newpath.size()/3];

				// Reset the pathCollision flag
				pathCollision = false;

				// Iterate over the number of waypoints
				for (unsigned short int i = 0; i < newpath.size()/3; i++)
				{

					// Get the iTH waypoint
					nx[i] = (unsigned short int) (newpath[3*i] / MAP_RESOLUTION);
					ny[i] = (unsigned short int) (newpath[3*i+1] / MAP_RESOLUTION);
					nz[i] = (unsigned short int) (newpath[3*i+2] / MAP_RESOLUTION);
				
				} // for (int i = 0; i < newpath.size()/3; i++)

			} // if (newpath.size() > 0)

		// Relinquish possession of the path lock
		pthread_mutex_unlock(&path_lock);

		// Attempt to obtain possession of the path lock 
		while(pthread_mutex_trylock(&map_lock))
		{
			
			// Wait 100 microseconds
			usleep(10);

		} // while(pthread_mutex_trylock(&map_lock))
		
		// Relinquish possession of the path lock, immediately reobtain
		pthread_mutex_unlock(&map_lock);
		pthread_mutex_lock(&map_lock);	

			for (unsigned short int i = 0; i < newpath.size()/3; i++)
			{

					cout << "nz,ny,nx: " << nz[i] << ", " << ny[i] << ", " << nx[i] << endl;


					// If the waypoint at the temp position is an obstacle
					if (maze[nz[i]][ny[i]][nx[i]].obstacle == 1)
					{

						// Indicate that the path collides with the current path
						pathCollision = true;

					} // if (maze[nz][ny][nx].obstacle == 1)

			}

		pthread_mutex_unlock(&map_lock);
			
		// Update the timer
		end_time = std::chrono::high_resolution_clock::now();
		timeElapsed = end_time - current_time;

		// If the map update was success AND ( the path collides with an obstacle OR the current mazegoal and previous mazegoal are not the same
		// OR the mazegoal is on the boundary of the map)
		if (pathCollision)
		{
			// quickly create a new path
			params.mu3 = 1.0;
		}
		else
		{
			params.mu3 = params.mu3_init;
		}


		cout << "Conditions: " << success << ", " << pathCollision << ", " << (mazegoal != prevmazegoal) << ", " <<   earlyTermination<< ", " << (timeElapsed.count() > 10.0) << ", " << !planSuccess << endl;
		if ((success && (pathCollision || (mazegoal != prevmazegoal && (mazegoal->x != 0 || mazegoal->y != 0 || mazegoal->z != 0) ) || earlyTermination || timeElapsed.count() > 10.0 || firstPath) ) || !planSuccess || pathCollision)
		{

			message = "<LPA* THREAD> Running LPA*";
			log.writeToLog(message);
			cout << message << endl;

			if (earlyTermination)
			{
				params.mu3 = min(1.5*params.mu3_init,1.0);
				cout << "params.mu3:" << params.mu3 << endl;
			}

			// Update the map
			updatemap();

			// Initialize the LPA* search
			lpastarinitialize();

			// Reset the timing thread variable
		  time_to_stop = false; 	terminate_timing_thread = false;
	
		  // Start the timing thread
			int	result_lpa_timing = pthread_create( &lpa_timing_tid, NULL, &lpa_timing_thread, this);	

			// Start LPA* search
			testlpastar();	

			if (earlyTermination)
			{
				params.mu3 = params.mu3_init;
			}

			if (planSuccess)
			{
				firstPath = false;
				earlyTermination = false;
			}

			// If the LPA* search ends before the timing thread expires, terminate the timing thread
			terminate_timing_thread = true;

			// Join the timing thread (ending it)
			pthread_join(lpa_timing_tid,NULL);

			// Set the previous mazegoal
			prevmazegoal = mazegoal;

			// Reset the timer
			current_time = std::chrono::high_resolution_clock::now();


		} // if (success && (pathCollision || (mazegoal != prevmazegoal && (mazegoal->x != 0 || mazegoal->y != 0 || mazegoal->z != 0) ) ) )
		else if (!success)
		{
			
			message = "<LPA* THREAD> Update map failure, temporary shut down.";
			log.writeToLog(message);
			cout << message << endl;

			// Reset the timing thread variables
			time_to_stop = false;
			terminate_timing_thread = true;

			// End the timing thread
			// pthread_join(lpa_timing_tid,NULL);

			// Wait 0.5 seconds
			usleep(100000);

			// Indicate that we should send the previous path
			newPath = 1;

			message = "<LPA* THREAD> Restarting.";
			log.writeToLog(message);
			cout << message << endl;

		} // else if (!success)
		else
		{

			// Update the map
			updatemap();

			// if (mazegoal == prevmazegoal)
			// {

			// 	for (int x = 0; x < MAZEDEPTH; ++x)
			// 	{
			// 		for (int z = 0; z < MAZEHEIGHT; ++z)
			// 		{
			// 			for (int y = 0; y < MAZEWIDTH; ++y)
			// 	    {
			// 	      // if (voxel_map_changes[z][y][x] && !maze[z][y][x].obstacle)
			// 	      if (voxel_map_changes[z][y][x] && &maze[z][y][x] != mazestart && &maze[z][y][x] != mazegoal && maze[z][y][x].searchtree != NULL)
			// 	      {

			// 	      	// initializecell(&maze[z][y][x]);
			// 	      	// Compute new edge costs
			// 	      	for (int d = 0; d < DIRECTIONS; ++d)
			// 					{

			// 					  newz = z + dz[d];
			// 					  newy = y + dy[d];
			// 					  newx = x + dx[d];

			// 					  if (newz >= 0 && newz < MAZEHEIGHT && newy >= 0 && newy < MAZEWIDTH && newx >= 0 && newx < MAZEDEPTH && !maze[newz][newy][newx].obstacle)
			// 				    {

			// 				    	// Update the neighbor (could have previously been NULL)
			// 	      			maze[z][y][x].move[d] = &maze[newz][newy][newx];

			// 	      			// Remember the predecessor
			// 	     				maze[newz][newy][newx].move[flip[d]] = &maze[z][y][x]; // remembers the predecessor
							      
			// 				      // initializecell(&maze[newz][newy][newx]);
										
			// 							// Compute new edge cost (kappa_sqrt)
			// 							root = (float) 10*(maze[newz][newy][newx].z - maze[z][y][x].z)*(maze[newz][newy][newx].z - maze[z][y][x].z) + (maze[newz][newy][newx].y - maze[z][y][x].y)*(maze[newz][newy][newx].y - maze[z][y][x].y) + (maze[newz][newy][newx].x - maze[z][y][x].x)*(maze[newz][newy][newx].x - maze[z][y][x].x);
			// 							kappa = getKappa(maze[newz][newy][newx].x,maze[newz][newy][newx].y,maze[newz][newy][newx].z);										
			// 							kappa_sqrt = kappa*sqrt(root);

			// 							// Detect a change in edge cost
			// 							if (maze[newz][newy][newx].kappa_sqrt != kappa_sqrt)
			// 							{
			// 								maze[newz][newy][newx].kappa_sqrt = kappa_sqrt;
			// 							}
			// 							else
			// 							{
			// 								continue;
			// 							}

			// 							// Update the vertex
			// 				      if (&maze[newz][newy][newx] != mazestart) 
			// 							{						

			// 								min_f = 1000000000.0;
									
			// 								for (int i = 0; i < DIRECTIONS; i++)
			// 								{
												
			// 									int tx = maze[newz][newy][newx].move[i]->x;
			// 									int ty = maze[newz][newy][newx].move[i]->y;
			// 									int tz = maze[newz][newy][newx].move[i]->z;

			// 									if (maze[tz][ty][tx].searchtree == &maze[newz][newy][newx])
			// 									{
			// 										if (maze[tz][ty][tx].g + maze[tz][ty][tx].kappa_sqrt < min_f)
			// 										{
			// 											min_f = maze[tz][ty][tx].g + maze[tz][ty][tx].kappa_sqrt;
			// 											min_cell = &maze[tz][ty][tx]; 
			// 										}
			// 									}

			// 								}

			// 								maze[newz][newy][newx].rhs = min_f;

			// 							}

			// 							if (query_heap(&maze[newz][newy][newx]))
			// 							{
			// 								deleteheap(&maze[newz][newy][newx]);
			// 							}

			// 							if (maze[newz][newy][newx].g != maze[newz][newy][newx].rhs)
			// 							{
			// 								insertheap(&maze[newz][newy][newx]);
			// 							}


			// 				    }

			// 					}

			// 	      }

			//       }
		 //    	}
		 //    }
		 //  }

			// Wait 0.5 seconds
			usleep(1000000);

		} // else if (!success)

	} // while(!exit_thread)

} // void Planner::LPAstar_thread()


// Member function of Planner class
// flighstack_interface_thread: this function is responsible for communicating with other modules
// INPUTS: none
// OUTPUTS: none
void Planner::gnc_connection_thread()
{
	
	CLIENT client("Parameter_Files/socket_parameters_path_planner.txt", &exit_thread);

	// Integer storing status of writing to a log
	int writeStatus = 0;

	// String storing messages to write to logs or output to terminal
	string message;

	// Declare a logger object
	LOGGER log_interface;

	log_interface.openLogFile("INTERFACE");
	cout << "<INTERFACE-THREAD> Starting interface thread." << endl;
	writeStatus = log_interface.writeToLog("<INTERFACE-THREAD> Starting interface thread.");

	// Variables to store how many bytes are read over each socket.
	int poseread = 0, goalread = 0, mapread = 0;

	// Message to send to server, letting it know this client's credentials.
	char client_message[30];

	// Define iterator for 1D voxel map array, define integers storing the number of occupied voxels
	int l = 0, occ = 0, occ_prev = 0;

	// Start position and goal position buffers.
	float poseBuffer[12] = { 0 }, goalBuffer[3] = { 0 };

	// Unsigned char map buffer. 0:255 unsigned char is not interpreted by
	// most functions as a number, but as an ASCII character, requiring 8 bits.
	// To interpret the information stored in this buffer, cast it as an integer.
	int mapBuffer[100][30][100] = { 0 };

	// FirstPassComplete indicates if a start, goal, and map have been read.
	bool startPass = false, goalPass = false, mapPass = false;

	// String variable used to store a line from the ofstream Mapfile
	string temp;

	// Counter to keep track of how many voxels are marked as an obstacle.
	int count = 0;

	// Vector of booleans storing the previous 1D voxel map 
	vector<bool> map_initial_prev;

	// Resize the vector
	map_initial_prev.resize(MAZEWIDTH*MAZEHEIGHT*MAZEDEPTH);

	// Boolean indicating if the first path has been sent
	bool firstPathSent = 0;

	// Eigen::MatrixXfs storing the current path, and the previous path
	Eigen::MatrixXf prevPath = Eigen::MatrixXf::Zero(1,3);
	Eigen::MatrixXf eigPath = Eigen::MatrixXf::Zero(1,3);

	// Integer indicating that a new map is coming 
	int newMap = 0;

	float headingstart = 0;

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
	float goal_time_us = 1000000/(goal_freq); // Microseconds	
	float path_time_us = 1000000/path_freq; // Microseconds	
	float constraint_time_us = 1000000/constraint_freq; // Microseconds	
	float trajectory_time_us = 1000000/trajectory_freq; // Microseconds	

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

	char map[300000];
	float pose[4];
	float goal_vec[3];
	int _interface_loops = 0;

	// Pose communication vars
	std::vector<string> pose_recv;
	string pose_recv_string;
	char* pose_recv_buffer;
	pose_recv_buffer = new char[1000];

	// Goal communication vars
	std::vector<string> goal_recv;
	string goal_recv_string;
	char* goal_recv_buffer = new char[100];	

	// Path communication vars
	string path_send;
	char* path_send_buffer = new char[5000];
	char* cstr_path = new char[5000];

	int bytes_read_map = 0, bytes_read_pose = 0, bytes_read_goal = 0, bytes_read_path = 0, bytes_read_constraints = 0, bytes_read_trajectory = 0;
	int counter = 0;	

	// Enter loop, exit when it's time to exit the program
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

						// Attempt to obtain possession of the start lock
						while (pthread_mutex_trylock(&start_lock))
						{
							
							// Wait 10 microseconds
							usleep(1);

						} // while (pthread_mutex_trylock(&start_lock))

						// Relinquish possession of the start lock, immediately reobtain
						pthread_mutex_unlock(&start_lock);
						pthread_mutex_lock(&start_lock);

							xstart = abs((round(boost::lexical_cast<float>(pose_recv[1])/MAP_RESOLUTION) * MAP_RESOLUTION) / MAP_RESOLUTION);
							ystart = abs((round(boost::lexical_cast<float>(pose_recv[0])/MAP_RESOLUTION) * MAP_RESOLUTION) / MAP_RESOLUTION);
							zstart = abs((round(boost::lexical_cast<float>(pose_recv[2])/MAP_RESOLUTION) * MAP_RESOLUTION) / MAP_RESOLUTION);
							headingstart = -boost::lexical_cast<float>(pose_recv[14]);

							pose[0] = boost::lexical_cast<float>(pose_recv[0]);
							pose[1] = boost::lexical_cast<float>(pose_recv[1]);
							pose[2] = boost::lexical_cast<float>(pose_recv[2]);
							pose[3] = boost::lexical_cast<float>(pose_recv[14]);

							// cout << "pose: " << xstart << "," << ystart << "," << zstart << "," << headingstart << endl;

							counter = 0;

							if (headingstart < 0.4636 && headingstart > -0.4636 )
							{
								ystart+=1;	
							}
							else if (headingstart < 0.4636 + 0.6435 && headingstart >= 0.4636)
							{
								ystart+=1;	
								xstart+=1;
							}
							else if (headingstart < 3*0.4636 + 0.6435 && headingstart >= 0.4636 + 0.6435)
							{
								xstart+=1;
							}
							else if (headingstart < 3*0.4636 + 2*0.6435 && headingstart >= 3*0.4636 + 0.6435)
							{
								ystart-=1;
								xstart+=1;
							} 
							else if (headingstart < 5*0.4636 + 2*0.6435 && headingstart >= 3*0.4636 + 2*0.6435)
							{
								ystart-=1;
							}
							else if (headingstart > -0.4636 - 0.6435 && headingstart <= -0.4636)
							{
								ystart+=1;	
								xstart-=1;				
							}
							else if (headingstart > -3*0.4636 - 0.6435 && headingstart <= - 0.4636 - 0.4636)
							{
								xstart-=1;				
							}
							else if (headingstart > -3*0.4636 - 2*0.6435 && headingstart <= -3*0.4636 - 0.6435)
							{
								ystart-=1;	
								xstart-=1;				
							}
							else if (headingstart < 5*0.4636 + 2*0.6435 && headingstart >= 3*0.4636 + 2*0.6435)
							{
								ystart-=1;
							}

							startPass = true;
	

						// Relinquish possession of the start lock 
						pthread_mutex_unlock(&start_lock);			
					
					}
				// }
			}	
			
			current_time_pose = std::chrono::high_resolution_clock::now();

		}

		////////////////////////
		// MAP COMMUNICATIONS //
		////////////////////////

		// If enough time has passed
		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_map).count() >= map_time_us)
		{	

			// RECEIVE THE MAP //
			if (client.socket_active[0])
			{

				// cout << "timeElapsed_map: " << timeElapsed_map.count() << endl;

				bytes_read_map = client.sockets[0]->process_receiving(map,300000*sizeof(char),0);
				// cout << "bytes_read_map: " << bytes_read_map << endl;

				// cout << "map[0]: " << +map[0] << endl;

				// reset the interator for the 1D voxel map array
				counter = 0;

				// Reset the counter for the number of occupied voxels in the map
				occ = 0;
	
				// Attempt to obtain possession of the map lock
				while (pthread_mutex_trylock(&map_lock))
				{

					// Wait 10 microseconds
					usleep(10);

				} // while (pthread_mutex_trylock(&map_lock))

				// Relinquish possession of the lock, immediately reobtain
				pthread_mutex_unlock(&map_lock);
				pthread_mutex_lock(&map_lock);

					occ = 0;

					// Iterate over the height
					for (int j = 0; j < MAZEWIDTH; j++) 
					{

						// Iterate over the width
						for (int k = 0; k < MAZEHEIGHT; k++) 
						{

							// Iterate over the depth
							for (int i = 0; i < MAZEDEPTH; i++) 
							{	

								// If the voxel i,j,k is explored and occupied		
								// RHS: convert char to int
 								map_full[i][j][k] = +map[counter];
								
 								if (map_full[i][j][k] == 3)
 								{
 									occ++;
 								}

								// Increment the counter for the 1D voxel map array
								counter++;

							} // for (int i = 0; i < MAZEDEPTH; i++) 

						} // for (int k = 0; k < MAZEWIDTH; k++) 

					} // for (int j = 0; j < MAZEHEIGHT; j++) 

				// Relinquish possession of the lock
				pthread_mutex_unlock(&map_lock);

				// Indicate that a map has been received
				mapPass = true;

				current_time_map = std::chrono::high_resolution_clock::now();

			}

		}
		
		// cout << "occ: " << occ << endl;

		/////////////////////////
		// GOAL COMMUNICATIONS //
		/////////////////////////		

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_goal).count() >= goal_time_us)
		{	

				// cout << "timeElapsed_goal: " << timeElapsed_goal.count() << endl;

			// RECEIVE THE GOAL //
			if (client.socket_active[4])
			{
				// cout << "receiving the pose" << endl;
				bytes_read_goal = client.sockets[4]->process_receiving(goal_recv_buffer,100*sizeof(char),1);
				

				goal_recv_string.clear();
				goal_recv_string.resize(100);
				for (int i = 0; i < 100; i++)
				{
					goal_recv_string[i] = goal_recv_buffer[i];
				}
				// cout << "goal_recv_string: " << goal_recv_string << endl;
				
				counter = 0;
				goal_recv.clear();

				if (goal_recv_string[0] == 'G')
				{

					for (int i = 1; i < 100; i++)
					{
						if (goal_recv_string[i] == '!')
						{
							break;
						}
						else if (goal_recv_string[i] == ',')
						{
							counter++;
							goal_recv.resize(counter);
						}
						else
						{
							goal_recv[counter-1] += goal_recv_string[i];
						}
					}

				}

				// cout << "goal: "; 
				// for (int i = 0; i < counter; i++)
				// {
				// 	cout << boost::lexical_cast<float>(goal_recv[i]) << ",";
				// }
				// cout << endl;

				while(pthread_mutex_trylock(&goal_lock))
				{
					usleep(20);
				} // while(pthread_mutex_trylock(&goal_lock))
				pthread_mutex_unlock(&goal_lock);
				pthread_mutex_lock(&goal_lock);

					if (counter > 0)
					{

						// Record the goal position, round to nearest 0.2
						xgoal = (round((boost::lexical_cast<float>(goal_recv[1])/MAP_RESOLUTION)) * MAP_RESOLUTION) / MAP_RESOLUTION;
						ygoal = (round((boost::lexical_cast<float>(goal_recv[0])/MAP_RESOLUTION)) * MAP_RESOLUTION) / MAP_RESOLUTION;
						zgoal = (round((boost::lexical_cast<float>(goal_recv[2])/MAP_RESOLUTION)) * MAP_RESOLUTION) / MAP_RESOLUTION;

						///////////////////////////////
						// For testing purposes only //
						// xgoal = 10;
						// ygoal = 30;
						// zgoal = 5;
						///////////////////////////////

						goal_vec[0] = boost::lexical_cast<float>(goal_recv[1]);
						goal_vec[1] = boost::lexical_cast<float>(goal_recv[0]);
						goal_vec[2] = boost::lexical_cast<float>(goal_recv[2]);

						goalPass = true;

						// If the goal is not feasible
						if (xgoal == 0 || ygoal == 0 || zgoal == 0)
						{
							// set the goal to the previous
							xgoal = xgoalprev;
							ygoal = ygoalprev;
							zgoal = zgoalprev;

							if (!firstPassComplete)
							{
								goalPass = false;		
							}
							// Indicate that a goal has been received

						} // if (xgoal == 0 || ygoal == 0 || zgoal == 0)
						else
						{

							// Otherwise, set the previous goal to the current
							xgoalprev = xgoal;
							ygoalprev = ygoal;
							zgoalprev = zgoal;

						} // if (xgoal == 0 || ygoal == 0 || zgoal == 0)

					}

				pthread_mutex_unlock(&goal_lock);
				
			}
			
				current_time_goal = std::chrono::high_resolution_clock::now();

		}

		// If the first pass has not completed, and all data has been received
		if ( !firstPassComplete && startPass && goalPass && mapPass)
		{

			// Indicate that the first pass has completed
			firstPassComplete = true;

			// Relinquish possession of the lock, allowing LPA* to proceed
			pthread_mutex_unlock(&initialStartGoalMap_lock);

			std::cout << ">> All necessary data has been obtained from the flightstack." << std::endl;
			std::cout << ">> Unlocking so LPA* may proceed." << std::endl;

		} // if ( !firstPassComplete && startPass && goalPass && mapPass)

		/////////////////////////
		// PATH COMMUNICATIONS //
		/////////////////////////		

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_path).count() >= path_time_us)
		{
			
			// SEND THE PATH // 
			if (client.socket_active[7])
			{

				// Record the number of waypoints
				int pathsize = newpath.size()/3;
				
				// Make some test data and append to string buffer
				path_send.clear();
				path_send.append("P");
				path_send.append(",");

				while(pthread_mutex_trylock(&path_lock))
				{
					usleep(10);

				} // while(pthread_mutex_trylock(&path_lock))
				pthread_mutex_unlock(&path_lock);
				pthread_mutex_lock(&path_lock);

					for (int i = 0; i < pathsize; i++)
					{
						float temp = newpath.end()[-3 - 3*i];
						path_send.append( boost::lexical_cast<string>(temp) );
						path_send.append(",");
						temp = newpath.end()[-2 - 3*i];
						path_send.append( boost::lexical_cast<string>(temp) );
						path_send.append(",");
						temp = newpath.end()[-1 - 3*i];					
						path_send.append( boost::lexical_cast<string>(temp) );

						if (i < pathsize-1)
						{
							path_send.append(",");
						}
						else
						{
							// path_send.append(",");
							// path_send.append(boost::lexical_cast<string>(end_time_stamp.count()));
							path_send.append("!");
						}
					}

				pthread_mutex_unlock(&path_lock);

				if (pathsize == 0)
				{					
					// path_send.append(boost::lexical_cast<string>(end_time_stamp.count()));
					path_send.append("!");
				}

				// cout << "path_send: " << path_send << endl;

				// Copy the string to a char buffer
				// cstr_path = new char[5000];
				strcpy(cstr_path, path_send.c_str());
				memcpy(path_send_buffer, cstr_path, strlen(cstr_path)+1);

				// Send the path char buffer					
				client.sockets[7]->process_sending(path_send_buffer,5000*sizeof(char));

			}
			
			current_time_path = std::chrono::high_resolution_clock::now();

		}

		timeElapsed_map = end_time - current_time_map;		
		timeElapsed_pose = end_time - current_time_pose;		
		timeElapsed_goal = end_time - current_time_goal;		
		timeElapsed_path = end_time - current_time_path;	

		// if (!(_interface_loops % 400))
		// {

		// 	cout << "\033[2J\033[1;1H";
		// 	cout << "Position: " << std::fixed << std::setprecision(5) << -pose[0] << ", " << pose[1] << ", " << -pose[2] << " [m], Heading: " << pose[5] << " [rad]" << endl;
		// 	cout << "Goal:     " << std::fixed << std::setprecision(5) << goal_vec[1] << ", " << goal_vec[0] << ", " << goal_vec[2] << endl;
		// 	cout << "Occupied: " << std::fixed << std::setprecision(5) << occ << endl;
		// 	cout << "Timers:   " << "  MAP  " << "  POS  " << "  GOA  " << "  PAT  " << endl;
		// 	cout << "Time [s]: " << std::fixed << std::setprecision(5) << timeElapsed_map.count() << " " << timeElapsed_pose.count() << " " << timeElapsed_goal.count() << " " << timeElapsed_path.count() << endl;
		// 	for (int i = 0; i < min(newpath.size()/3,18); i++)
		// 	{
		// 		cout << i << ": " << newpath[3*i] << ", " << newpath[3*i+1] << ", " << newpath[3*i+2] << endl;
		// 	}
		// 	cout << "..." << endl;
		// }

		_interface_loops++;


	} // while(!exit_thread)
	
} // void Planner::LPAstar_thread()

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler( int sig )
{

  	// std::ofstream Dgnstc_Map_Log;

  	// if (Dgnstc_Map_Log.is_open() != 1){
   //    Dgnstc_Map_Log.open("Diagnostic_Logs/Dgnstc_Map_Log.txt");
  	// }

  	if (sig == SIGINT)
	{
		printf("\n");
		printf(">> TERMINATING AT USER REQUEST\n");
		printf("\n");
	}
  	if (sig == SIGPIPE)
	{
		printf("\n");
		printf(">> TERMINATING AFTER BROKEN PIPE\n");
		printf("\n");
	}
  	if (sig == SIGSEGV)
	{
		printf("\n");
		printf(">> TERMINATING AFTER SEGMENTATION FAULT\n");
		printf("\n");
	}

	exit_thread = 1;

	// Call the quit handlers for each interface object.

	// Wait until the threads are finished.
	usleep(1000000);
	// end program here
	exit(0);

}
