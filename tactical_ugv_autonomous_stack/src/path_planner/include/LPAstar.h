// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 9th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This header file defines the Planner class.
// End File Decscription ################################################################################

#ifndef LPASTAR_H_
#define LPASTAR_H_

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
#include <signal.h>
#include <pthread.h>
#include <fcntl.h>
#include <mutex>
#include <time.h>
#include <csignal>

// Custom header files
#include <logging.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

// Planner parameters
#define INFORMEDSEARCH
#define MAZEWIDTH 100
#define MAZEHEIGHT 30
#define MAZEDEPTH 100
#define MAZEDENSITY 0.4
#define MAZECHANGE 0.005
#define RUNS 1
#define REPLANNINGEPISODES 10
#define STATTIMES 5
#define THREEDIM24CON

// Define some symbols
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
  float mu1_init, mu2_init, mu3_init;
};

inline float MAP_RESOLUTION = 0.2;

// Local method
// linspace: Template which produces a vector of equally space points from "start" to "end" by "num" many points
// INPUTS: integer indicating the first element of the array,
// integer indicating the last element of the array,
// integer indicating the number of elements in the array
template <typename T> std::vector<T> linspace(T start, T end, int num)
{

  // Declare vector of template type
  std::vector<T> linspaced;

  // If the number of elements in the array is non-zero
  if (0 != num)
  {

    // If the number of elements is 1
    if (1 == num)
    {

      // Add the start point
      linspaced.push_back(static_cast<T>(start));

    } // if (1 == num)
    else
    {

      // Compute the stride needed
      double delta = (end - start) / (num - 1);

      // Iterate over the number of steps
      for (int i = 0; i < (num - 1); i++)
      {

        // Add the elements to the vector
        linspaced.push_back(start + delta * i);

      } // for (int i = 0; i < (num - 1); i++)

      // ensure that start and end are exactly the same as the input
      linspaced.push_back(static_cast<T>(end-1));

    } // if (1 == num)
  
  } // if (0 != num)

  return linspaced;

} // template <typename T> std::vector<T> linspace(int start, int end, int num)


// Define the Planner class
class Planner
{

// Publically available members
public:

  int tracersMap[MAZEDEPTH][MAZEWIDTH][MAZEHEIGHT];

  struct DANGER_ZONE
  {

    float center[3] = {0.0,0.0,0.0};
    float radius = 0;
    int radius_in_voxels = 0;
    float height = 0;
    float height_in_voxels = 0;
    bool active = 0;
    float angle = 0;
    float axis[3] = {0.0,0.0,1.0};
    float area_circle = M_PI*radius*radius;
    float area_rectangle = height*2*radius;
    float volume_cylinder = M_PI*radius*radius*height;
    bool vehicle_in_az = 0;

    void setRadius(float r)
    {
      radius = r;
      radius_in_voxels = floor(r/MAP_RESOLUTION);
      area_circle = M_PI*r*r;
      volume_cylinder = M_PI*r*r*height;
    }

    void setHeight(float h)
    {
      height = h;
      height_in_voxels = floor(h/MAP_RESOLUTION);
      area_rectangle = h*2*radius;
      volume_cylinder = M_PI*radius*radius*h;
    }

    void setRadiusAndHeight(float r, float h)
    {

      height = h;
      radius = r;
      radius_in_voxels = floor(r/MAP_RESOLUTION);
      height_in_voxels = floor(h/MAP_RESOLUTION);
      area_circle = M_PI*r*r;
      area_rectangle = h*2*r;
      volume_cylinder = M_PI*r*r*h;

    }

  } dzone;

  vector<DANGER_ZONE> dzone_vec;
  int active_zones = 0;

  struct INFER_ZONE
  {
    float cone_height = 4;
    float cone_angle = 45*3.1415/180;
    int hSteps = 6;
    int vSteps = 6;
    std::vector<int> poi_info_above, poi_info_below;
    int num_poi = 1;
    std::vector<int> ceiling_found;
    std::vector<int> floor_found;

    Eigen::MatrixXi sample_mean_above_voxel = Eigen::MatrixXi::Zero(3,num_poi);
    Eigen::MatrixXi sample_mean_below_voxel = Eigen::MatrixXi::Zero(3,num_poi);    

    INFER_ZONE()
    {

      poi_info_above.resize(hSteps*vSteps);
      poi_info_below.resize(hSteps*vSteps);
      ceiling_found.resize(num_poi);
      floor_found.resize(num_poi);
      INFER_ZONE::compute_sample_points();
    }

    Eigen::MatrixXf poi_trace_end_above_voxel = Eigen::MatrixXf::Zero(3,hSteps*vSteps);
    Eigen::MatrixXf poi_trace_end_below_voxel = Eigen::MatrixXf::Zero(3,hSteps*vSteps);

    void compute_sample_points()
    {

      int count = 0;
      std::vector<float> azimuth = linspace((float) (-cone_angle/sqrt(2)),(float) (cone_angle/sqrt(2)),hSteps);
      std::vector<float> elevation = linspace((float) (-cone_angle/sqrt(2)),(float) (cone_angle/sqrt(2)),vSteps);
      Eigen::MatrixXf R2 = Eigen::MatrixXf::Zero(3,3), R3 = Eigen::MatrixXf::Zero(3,3), R32 = Eigen::MatrixXf::Zero(3,3);
      Eigen::MatrixXf local_vertical = Eigen::MatrixXf::Zero(3,1);
      local_vertical(0,0) = 0.0;
      local_vertical(1,0) = 0.0;
      local_vertical(2,0) = 1.0;

      Eigen::MatrixXf aoi = Eigen::MatrixXf::Zero(3,1);

      for (int i = 0; i < hSteps; i++)
      {
        for (int j = 0; j < vSteps; j++)
        {

          R2(0,0) = cos(elevation[j]); R2(0,2) = sin(elevation[j]);
          R2(1,1) = 1.0; 
          R2(2,0) = -sin(elevation[j]); R2(2,2) = cos(elevation[j]);

          R3(0,0) = 1.0; 
          R3(1,1) = cos(azimuth[i]); R3(1,2) = -sin(azimuth[i]);
          R3(2,2) = sin(azimuth[i]); R3(2,2) = cos(azimuth[i]);

          R32 = R3*R2;

          aoi = cone_height*R32.transpose()*local_vertical;
          aoi = aoi/MAP_RESOLUTION + 0.5f*Eigen::MatrixXf::Ones(3,1);

          poi_trace_end_above_voxel.block(0,count,3,1) = (aoi.array()).floor();
          poi_trace_end_below_voxel.block(0,count,3,1) = (-1*aoi).array().floor();

          count++;

        }
      }
    }

    void setConeParameters(float h, float a, int _hSteps, int _vSteps)
    {
      cone_height = h;
      cone_angle = a;
      hSteps = _hSteps;
      vSteps = _vSteps;

      poi_trace_end_above_voxel = Eigen::MatrixXf::Zero(3,hSteps*vSteps);
      poi_trace_end_below_voxel = Eigen::MatrixXf::Zero(3,hSteps*vSteps);

      compute_sample_points();

    }

    // Function in case we decide to make # of steps change over time
    void setConeSampleParameters(int _hSteps, int _vSteps)
    {
      hSteps = _hSteps;
      vSteps = _vSteps;

      poi_trace_end_above_voxel = Eigen::MatrixXf::Zero(3,hSteps*vSteps);
      poi_trace_end_below_voxel = Eigen::MatrixXf::Zero(3,hSteps*vSteps);

      compute_sample_points();

    }    



} izone;

  int n, sx, sy, sz, exy, exz, ezy, ax, ay, az, bx, by, bz;  

  // Declare Parameters structure
  Parameters params;

  // Start and goal positions
  float STARTX, STARTY, STARTZ, GOALX, GOALY, GOALZ; // This is used by LPA* in the planning routine
  float xstart, ystart, zstart, xgoal, ygoal, zgoal; // This is updated by the flightstack interface thread.

  // Unitless Sizes -- Normal Coordinate system
  const int xsize = 100;
  const int ysize = 100;
  const int zsize = 30;

  // Unitless Sizes -- Normal Coordinate system
  static const int Height = 100;
  static const int Width = 100;
  static const int Depth = 30;

  // Define ray tracing quantities
  const float range = 4.0;  // maximum range of sensor
  int hSteps = 12;      // number of angles to discretize the horizontal FOV
  int vSteps = 12;      // number of angles to discretize the vertical FOV

  // Define integer representing the number of rays to trace
  int sample_points_count = (hSteps+1)*(vSteps+1);    

  // Path skip
  int d_stride = 1;

  // Input file streams
  ifstream inFile, inFile1, mapFile;

  // 3D Integer array storing the map 
  int map_full[Height][Width][Depth];
  int map_full_copy[Height][Width][Depth];
  int inferred_map[Height][Width][Depth]; 
  bool voxel_map_changes[Height][Width][Depth];   
  int changes = 0;

  int goaly, goalx, goalz;
  int prev_goaly = 0, prev_goalx = 0, prev_goalz = 0;
  int starty, startx, startz;
  float original_startx, original_starty, original_startz;

  // Vector storing the path
  vector<float> newpath;

  // Integers capturing the number of waypoints
  int pathSize, pathSize_f;

  // Integer indicating that a new path is about to be sent
  int newPath = 0;

  // Integer indicating that a new goal is about to be received
  int newGoal = 0;

  bool firstPath = true;

  bool planSuccess = false;

  // Pthread IDs
  pthread_t gnc_connection_tid, LPAstar_tid;
  
  // Pthread mutex for thread synchronization
  pthread_mutex_t path_lock;
  pthread_mutex_t start_lock;
  pthread_mutex_t goal_lock;
  pthread_mutex_t map_lock;
  pthread_mutex_t astar_lock;
  pthread_mutex_t initialStartGoalMap_lock;

  // Prototype functions //
  void setup();
  void gnc_connection_thread();
  void LPAstar_thread();
  bool updatemap();
  float getKappa(int x, int y, int z);
  int getAlpha(int x, int y, int z);
  void testlpastar(); 
  void lpastarcomputeshortestpath();
  void checkGoalSafety();
  bool ray_tracing();
  void ceiling_floor_infer();


// Private members
private:

  // Declare LOGGER objects
  LOGGER log;
  LOGGER log_path;

}; // class Planner

class ray_tracing_parameters
{

public:
  int x, y, z;
  int gx, gy, gz;
  int sx, sy, sz;
  int exy, exz, ezy;
  int bx, by, bz;
  int n;
  int intersect_unexplored = 0;
  int ray_number;
  Planner* _planner;

  void setRayOrigin(int _x, int _y, int _z)
  {
    x = _x; y = _y; z = _z; 
  }

  void setGoal(int _gx, int _gy, int _gz)
  {
    gx = _gx; gy = _gy; gz = _gz; 
  }

  void setS(int _sx, int _sy, int _sz)
  {
    sx = _sx; sy = _sy; sz = _sz; 
  }  

  void setN(int _n)
  {
    n = _n; 
  }  

  void setE(int _exy, int _exz, int _ezy)
  {
    exy = _exy; exz = _exz; ezy = _ezy; 
  }  

  void setB(int _bx, int _by, int _bz)
  {
    bx = _bx; by = _by; bz = _bz; 
  }  

  int operator[] (int i)
  {
    switch(i)
    {
      case 0:
        return n;
      case 1:
        return x;
      case 2:
        return y;
      case 3:
        return z;
      case 4:
        return gx;
      case 5:
        return gy;
      case 6:
        return gz;
      case 7:
        return sx;
      case 8:
        return sy;
      case 9:
        return sz;                                                        
      case 10:
        return exy;
      case 11:
        return exz;
      case 12:
        return ezy;
      case 13:
        return bx;
      case 14:
        return by;
      case 15:
        return bz; 
      case 16:        
        return intersect_unexplored;
      default:
        return 0;
    }

  }

  void print(int i)
  {

    cout << (*this)[i] << endl; 
  }

}; 

#endif