#ifndef ASTAR_H_
#define ASTAR_H_

// Standard header files
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <string.h>
#include <vector>
#include <algorithm> 
#include <numeric>
#include <math.h>     
#include <chrono>
#include <cstdlib>
#include <pthread.h>
#include <fcntl.h>
#include <mutex>
#include <time.h>
#include <Eigen/Dense>

// Planner parameters
#define MAZEWIDTH 100
#define MAZEHEIGHT 30
#define MAZEDEPTH 100
#define THREEDIM24CON
#define RESOLUTION 0.2

// #define LARGE 999
// #define BASE 1000
#define max(x, y) ((x) > (y) ? (x) : (y))
#define min(x, y) ((x) < (y) ? (x) : (y))

// Namespaces
using namespace std;

// Quit handler prototype.
void quit_handler( int sig );

// Data Structures //
struct Parameters
{
  float mu1, mu2, mu3;
};

class Planner{

public:

  Parameters params;

  // Start and goal positions
  float STARTX, STARTY, STARTZ, GOALX, GOALY, GOALZ; // This is used by LPA* in the planning routine
  float xstart, ystart, zstart, xgoal, ygoal, zgoal; // This is updated by the flightstack interface thread.
                                                     // These values are only used once LPA* has completed a planning episode. 

  // Unitless Sizes -- Normal Coordinate system
  const int xsize = 100;
  const int ysize = 100;
  const int zsize = 30;

  // Unitless Sizes -- Normal Coordinate system
  static const int Height = 100;
  static const int Width = 100;
  static const int Depth = 30;

  bool newMap = false, newPose;
 
  float start[3] = { 0 };
  float goals[3] = { 0 };

  int np = 0, stepCount = 0;

  ifstream inFile, inFile1, mapFile;

  int map_full[Height][Width][Depth];
  int goal_mat[Height][Width][Depth] = { 0 };

  int flightstack_status, astar_status, mapMode;

  pthread_t flightstack_tid, LPAstar_tid;

  int pathSize, pathSize_f;

  bool optimalPathReady = false, Astar_start_planning = true, success = false;

  string map_filename;
  int d_stride = 1;

  void initialize();

  bool astar();

  bool updatemap(int map[][30][100], Eigen::MatrixXf* start, Eigen::MatrixXf* goal);
  void testastar(); 
  void astarcomputeshortestpath();
  void update_start_and_goal(Eigen::MatrixXf* start, Eigen::MatrixXf* goal);

protected:

  mutex Astar_start_mutex;


private:

};

#endif