#ifndef F_MPC_UNCUT_H
#define F_MPC_UNCUT_H


#include <read_files.h>
#include <logging.h>
#include <Eigen/Dense>
#include <structures.h>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <complex>
#include <cmath>
#include <algorithm>

#include <time.h>
#include <chrono>
#include <sys/time.h>
#include <ctime>

#include <sys/ioctl.h>


using namespace std;
using namespace std::complex_literals;

class F_MPC_UNCUT
{

public:

	F_MPC_UNCUT();
	~F_MPC_UNCUT();

	void start();

	void trajectory_planner_thread();
	void communication_thread();
	System quadrotor;
	MPC_Params mpc_params;
	Matrix_Set bomt;
	Line_Search_Params lsp;

	// Goal
	Eigen::MatrixXf goal;
	// Copy of the planned path
	Eigen::MatrixXf localPath;

	int numTrajectoryPlans = 1;

private:

	static void quit_handler(int sig);
	void find_closest_waypoint(float* xdiff, float* ydiff, float* zdiff, int* closest_traj_iterator);
	void find_new_waypoint(Eigen::MatrixXf* goal, float* xdiff, float* ydiff, float* zdiff);
	float fsat(Eigen::MatrixXf* arg);
	void find_closest_obstacle(Eigen::MatrixXf* temp_goal_pos, int segment_number);
	void update_weighting_matrices(int segment_number, bool success);
	void update_and_discount_weighting_matrices(int segment_number, bool success);
	bool compute_force_steering_for_lambda_k_and_g_barrier(Eigen::MatrixXf* chi, Eigen::MatrixXf* lambda);
	void get_Ginv_pos_k(float psi, Eigen::MatrixXf* G_inv);
	void get_f_pos_k(float psi, float dpsi, float vx, float vy, Eigen::MatrixXf* f);
	void compute_box_constraints(int segment_number, int goalStride);
	bool fmpcsolve(Eigen::MatrixXf* X, Eigen::MatrixXf* U, int segment_number);
	void fmpc_hard_constraints(Eigen::MatrixXf* eig_X, int segment_number, int goalStride, int numFailures);
	void fmpc_soft_constraints(Eigen::MatrixXf* eig_X, int segment_number, int goalStride);
	void update_quadrotor_pose();
	void get_trajectory_goal();
	void objective_function_value(Eigen::MatrixXf* X, Eigen::MatrixXf* U, int segment_number);

	pthread_t traj_tid;
	pthread_t comm_tid;

	pthread_mutex_t pose_lock;
	pthread_mutex_t path_lock;
	pthread_mutex_t map_lock;
	pthread_mutex_t constraint_lock;
	pthread_mutex_t trajectory_lock;
	pthread_mutex_t ellipsoid_lock;

	// Stores constraints sent from constraint generator
	Eigen::MatrixXf collisionConstraints;

	// Stores the parent ellipsoid from the collision avoidance algorithm
	Eigen::MatrixXf parentEllipsoid;

	// Stores the planned path from the path planner
	Eigen::MatrixXf plannedPath, prev_plannedPath;

	// Stores differences between consecutive planned paths
	Eigen::MatrixXf plannedPathDifference;

	// Stores the pose from the simulator
	float pose[18], initialPose[18]; // x,y,z,dx,dy,dz,ddx,ddy,ddz,dddx,dddy,dddz,phi,theta,psi,omega1,omega2,omega3

	// Current and previous 3D position [x,y,z], tracked independently of the planar QP state
	// chi = [x,y,xdot,ydot] (all that F_x/delta_f can directly command). Kept so the existing
	// 3D obstacle-avoidance/waypoint infrastructure keeps working and z stays available for
	// future terrain-aware planning.
	Eigen::Vector3f current_position;
	Eigen::Vector3f prev_position;

	// Obstacle coordinates
	Eigen::MatrixXf obs;
	Eigen::MatrixXf local_obs;

	// Map
	float voxel_map[100][30][100];

	// Nearest obstacle
	Eigen::MatrixXf r_obs;

	bool firstPassComplete = false;
	
	// Keeps track if the first planning episode was ever successful
	bool firstPlanningEpisode = false;

	// Keeps track of the success of planning segments
	bool trajectory_plan_success = false;

	// Keeps tracking of intermediate planning success
	bool intermediate_success = false;

	bool firstSuccess = false;

	// keeps track of nearest waypoint in the planned path
	int traj_iterator = 0;
	int prev_traj_iterator = -1;

	// Keeps track of how many times planning a segment failed
	int numFailures = 0;

	// Boolean indicating the unfortunate failure to plan
	bool failedPlan = false;

	// keeps track of the segments needed to be planned to interpolate the planned path 
	int remaining_segments = 0;

	// Barrier function used to ensure F_x and delta_f remain within (-Fx_max, Fx_max) and (-delta_f_max, delta_f_max), respectively.
	Eigen::MatrixXf g_barrier;

	int newTraj = 0;

	// Matrix storing the full remaining trajectory which outlines the planned path
	Eigen::MatrixXf* full_trajectory;
	Eigen::MatrixXf* sendfull_trajectory;
	Eigen::MatrixXf* prevfull_trajectory;
	Eigen::MatrixXf* full_trajectory_interf;
	Eigen::MatrixXf segmentGoals;
	Eigen::MatrixXf* full_policy;
	Eigen::MatrixXf* full_attitude;
	Eigen::MatrixXf* prev_seg_eig_X;
	Eigen::MatrixXf* prev_seg_eig_U;

	Eigen::MatrixXf* u_k_nuX;
	Eigen::MatrixXf* v_k_nuX;
	Eigen::MatrixXf* lambda_k_nuX;

	Eigen::MatrixXf u_k;
	Eigen::MatrixXf v_k;
	Eigen::MatrixXf lambda_k;

	// Matrix (will be sized as scalar) storing the objective function value
	Eigen::MatrixXf Obj;

};

// Thread trampolines: bridge pthread's C-style function pointers to the
// F_MPC_UNCUT instance methods that actually run the trajectory planner
// and communication loops. Defined in f_mpc_trajectory.cpp / f_mpc_communication.cpp.
void* start_communication_thread(F_MPC_UNCUT* f_mpc_uncut);
void* start_comm_interface_thread(void *args);
void* start_trajectory_planning_thread(F_MPC_UNCUT* f_mpc_uncut);
void* start_traj_interface_thread(void *args);

#endif