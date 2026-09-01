#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

#define _USE_MATH_DEFINES
// Unix headers
#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <pthread.h>
#include <signal.h>
#include <sys/socket.h>	//unix socket
#include <sys/types.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sched.h>
#include <termios.h> 
#include <fcntl.h>
// SDPA
#include <sdpa-7.3.8/sdpa_call.h>
// Eigen
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
// User-defined
#include <structures.h>

using namespace std;

template <typename T> std::vector<T> linspace(T start, T end, const int num)
{
	std::vector<T> linspaced;

	if (0 != num)
	{
		if (1 == num)
		{
			linspaced.push_back(static_cast<T>(start));
		}
		else
		{
			double delta = (end - start) / (num - 1);

			for (auto i = 0; i < (num-1); ++i)
			{
				linspaced.push_back(static_cast<T>(start + delta * i));
			}

			linspaced.push_back(static_cast<T>(end));

		}
	}
	return linspaced;
}

class CONSTRAINT{

public:

	const int num_ellipsoids = 4;
	const int grid_x = 100;
	const int grid_y = 30;
	const int grid_z = 100;
	const float resolution = 0.2;
	int _nopts, _nangles, _method, _parameterType, _num_xy_sectors, _num_z_sectors, _bCenters_source, _bCenters_samples;
	float _soft_offset, _x_CNTR, _y_CNTR, _z_CNTR, _epsilon, _bar_v;
	bool _loggingEnabled, _auto_CNTR, _mapUpdate;
	string _map_filename, _bound_filename, _initial_center_filename;
	double _SDPA_params[8];
	double maxIter, epsilonStar, epsilonDash, lambdaStar, omegaStar, betaStar, betaBar, gammaStar;
	int parameterType;
	// log for ellipsoid parameters
	ofstream log, log1, log2, log3;

	pthread_t flightstack_tid, collision_avoidance_tid;
	pthread_mutex_t initial_lock, map_lock, constraint_lock, ellipsoid_lock, pose_lock;
	int flightstack_status = 0, collision_avoidance_status = 0;

	// Constructor
	CONSTRAINT();
	// Destructor
	~CONSTRAINT();

	bool init_constraint_generator(struct System* quad);
	void start_collision_avoidance_thread(struct System* quad);
	void start_flightstack_interface_thread(struct System* quad);

	// this function is now virtual: can be replaced by a child class implementation but
	// is not required.
	bool loggingEnabled;
	int PassesComplete;

	// map size
	int map_size, constraintSize = 0;

	// files for reading maps from text files
	ifstream inFile1, inFile2, inFile3;
	string map_filename, bound_filename, initial_center_filename;

    // runConstraint variables
    int dimensions = 3;
		// number of points to sample at each angle/number of angles to sample  
		// Range of these angles will be [-pi/2,0]
    int nopts, nangles; 
    float soft_offset;
    int pointNumber = 0;
		// Since we find symmetric constraints by mirroring, there are 2 for every point EXCEPT
		//		the final sweep which is already fully sampled. Final number is therefore
		//		2*points - (points in last sweep) 
    int Fsize = 0; 
		// flag to indicate whether sampling was carried out successfully
    int unsuccessfulSample = 0; 
    int block_start = 0;
    //

	int sampleEllipsoid(Eigen::MatrixXf* points);

		// sampleEllipsoid variables
		Eigen::MatrixXf rootP;	// matrix square root of P
		Eigen::MatrixXf angleVector; //(dimensions, 1) 
		Eigen::MatrixXf xc; //(int dimensions, 1);	ellipsoid center
		//float c = 1.0;
		//


	void generateConstraints(Eigen::MatrixXf* samples, Eigen::MatrixXf* obstaclesF, Eigen::MatrixXf* obstaclesf, Eigen::MatrixXf* softF, Eigen::MatrixXf* softf, struct System* quad);

		// generateConstraints variables
		Eigen::MatrixXf offset; //(1, 1);	soft constraint offset
		Eigen::MatrixXf eigEpsilon; //(1, 1); epsilon = center_D - sample_D	
		Eigen::MatrixXf eigD; //(1,1); D = Ax + By + Cz = x_vec^T * X_sample
		Eigen::MatrixXf eigD_soft;
		Eigen::MatrixXf x_vec; //(int dimensions, 1); x_vec = sample point - center point
		//

	void setup_problem(struct System* _quad);

	bool determineCenters(struct System* quad);

	bool quadraticDiscrimination(struct System* quad) ;

		// Create vectors to hold point cloud data. "x" is quadrotor, "y" is obstacle 
		std::vector<float> x;
		std::vector<float> y, yy, point_cloud;;

		float current_value = 0;	// Used to convert strings to floats when reading from file
		int coordinate = 0;			
		int x_size = 3;			// Size of x vector (quadrotor points)
		int y_size = 0;				// Size of y vector (obstacle points)
		int N = x_size / dimensions;// Number of data points = [x,y,z] in x vector 
		int M = y_size / dimensions;// Number of data points = [x,y,z] in y vector

		// quadraticDiscrimination variables
		Eigen::Vector3f x0;		// Current position of vehicle
		Eigen::Vector3f v0;		// Current velocity of vehicle
		float rho_bb, rho_bb_inv;
		float halfx = 0.1;		// Half the x-length of vehicle
		float halfy = 0.1;		// Half the y-length of vehicle
		float halfz = 0.1;		// Half the z-length of vehicle
		float constant = 0.0f;	// Set constant value for inequality constraints
		int numberConstraints = (dimensions*dimensions*dimensions)/2+dimensions+1;
		float x1,x2,x3; 		// Holds environment information
		Eigen::MatrixXf RASP_Vars;
		Eigen::MatrixXf MIN_DIST_Vars;
		std::vector<bool> RASP_PSignDefinite;
		Eigen::Vector3f xC; 	// User-Defined Ellipsoid Center
		//

		// Ellipsoid variables
		Eigen::MatrixXf P; //(int dimensions, int dimensions);
		Eigen::MatrixXf q; //(int dimensions, 1);
		float r = 0.0;
		Eigen::MatrixXf C;//(1, 1);
		float true_k = 0.0;
		float gamma = 0.0;
		Eigen::VectorXf eigenValues;

		// User defined Ellipsoid center
		float x_CNTR, y_CNTR, z_CNTR;
		// Boolean for user-defined center or auto-centering (quadcopter position).
		bool auto_CNTR;

		// Vars for MULTI_ELLIPSOID
		// Centers for bounding ellipsoids
		Eigen::MatrixXf bCenters;
		Eigen::Vector3f vecTemp;
		// number of sectors
		int num_xy_sectors, num_z_sectors, total_sectors;
		// small offset for detecting if a point is close to an ellipsoid
		float epsilon;

		int extra_variables = 0;
		int k;
		bool multiEllipsoid_firstPass = true;

	void closePoints();
		// vars for closePoints()
		int num_closePoints = 0;
		Eigen::MatrixXf closePointsMat;
		Eigen::MatrixXf closePointsMat_x0;
		Eigen::Matrix3f P_temp;
		Eigen::MatrixXf Y_temp;
		Eigen::Matrix3f sqrtP_transpose;
		Eigen::MatrixXf norm_closePoints;


	void fastApproximateConvexHull();
		
		// vars for fastApproximateConvexHull()
		float k1, k2; 				// k1 is the angle between samples for the sector azimuth
									// k2 is the angle between samples for the sector inclination 
		std::vector<double> alpha;	// Alpha is a vector storing the azimuth angles
		std::vector<double> beta;	// Beta is a vector storing the inclination angles

		// Slices of the ball \overline{B}_{\rho}(r_k(i\Delta T))
		// First _num_z_sectors of sectorPoint_vector indicate a single slice.
		// there are _num_xy_sectors slices in total.
		Eigen::MatrixXf sectorPoint_vector;
		Eigen::MatrixXf sectorPoint_vector_init;

		// Stores the planes defining every sector.
		// s_1: right; top; left; bottom
		// s_2: right; top; left; bottom
		// ...
		// s_total_sectors/2: right, bottom, left, top
		Eigen::MatrixXf sectorPlanes, sectorPlanes1;
		// Row i of closePoint_membership corresponds to urgent point i of closePointsMat,
		// indicating the sector that the urgent point is a member of.
		Eigen::VectorXf closePoint_membership; 
		Eigen::MatrixXf b,c,d,e;
		Eigen::Vector3f sectorNormals_temp, minimalProj;		// the vector that points in the center of the sector.
		Eigen::MatrixXf sectorNormals, sectorNormals1;		// the vector that points in the center of the sector.
		Eigen::MatrixXf closePoint_minimalProj_sectors;	// closePoint obtaining maximal dot product in a sector.
		double xs, ys, zs;
		int kk = 1;	

		// Vector storing the ith sector's points
		Eigen::Vector3f bi, ei, ci, di;


		// Stores the projection onto the vector that bisects each sector.
		// temp variable for the urgent point that minimizes the projection.
		// Stores the offset, which corresponds to the urgent point that minimizes the projection.
		Eigen::VectorXf minimalProj_sectors, minimalProj_temp, minimalProj_offsets;	

		// Stores the plane equations Ax+By+Cz+D = 0.
		Eigen::MatrixXf ObstaclesFf_hard, ObstaclesFf_soft;

		// Stores the urgent point, and plane normals for easy dot products.
		Eigen::Vector3f point, n1, n2, n3, n4; 

		Eigen::VectorXf ni, oj;

		Eigen::Vector3f p1, vn, vn1, vn2;
		float z1 = 0.0;
		// This vector is used to easily identify sectors that share an edge with the north or south pole of the
		// unit sphere. These sectors only have 3 planes, whereas other sectors have 4.
		std::vector<int> n2zero;
		Eigen::MatrixXf dummy_ObstaclesFf = Eigen::MatrixXf::Zero(32,5);
		Eigen::MatrixXf dummy_s = Eigen::MatrixXf::Zero(32,1);

		// Integer storing the number of 0s in the sector plane's definition
		int n2zeros;


	void chebyshevCenter();


protected:

	void collision_avoidance_thread(struct System* quad);
	void flightstack_interface_thread(struct System* quad);

};

// Object wrapper, used to tell threads where to look for information that they may need to read or write from.
class ObjWrapper 
{

	// The class ObjWrapper has these members, which are points to different objects.
	public: void* _threadId;
	public: System* _quad;
	public: CONSTRAINT* _constraint;

	// Class constructor, when instantiating an object of type ObjWrapper, pass the address of the necessary variables.
	public: ObjWrapper(void* threadId, struct System* quad, CONSTRAINT* constraint)
	{
		_threadId = threadId;
		_quad = quad;
		_constraint = constraint;
	}

};

#endif
