#ifndef STRUCTURES_H_
#define STRUCTURES_H_

#include "Eigen/Dense"
using namespace std;

struct System {

	int n;
	Eigen::MatrixXf eig_A;

	int num_hard_x = 32; //the number of hard state constraints
	Eigen::MatrixXf eig_Fx_hard;
	Eigen::MatrixXf eig_Fpsi_hard;
	Eigen::MatrixXf eig_Fz_hard_ceiling;
	Eigen::MatrixXf eig_Fv_hard_boundary_conditions;
	int num_hard_u; //the number of hard control constraints
	Eigen::MatrixXf eig_Fu_hard;
	Eigen::VectorXf eig_x_hard_bounds;
	Eigen::MatrixXf eig_x_hard_bounds_horizon; 
	Eigen::Vector2f eig_psi_hard_bounds;
	Eigen::VectorXf eig_u_hard_bounds;
	float eig_z_hard_ceiling_bounds;

	int num_soft_x = 32; //the number of soft state constraints
	float* eig_Fx_soft_array;
	Eigen::MatrixXf eig_Fx_soft;
	Eigen::MatrixXf eig_Fpsi_soft;
	Eigen::MatrixXf eig_Fz_soft_ceiling;
	Eigen::MatrixXf eig_Fv_soft_boundary_conditions;
	int num_soft_u; //the number of soft control constraints (should be zero?)
	Eigen::MatrixXf eig_Fu_soft;
	float* eig_x_soft_bounds_array;
	Eigen::VectorXf eig_x_soft_bounds;
	Eigen::MatrixXf eig_x_soft_bounds_horizon; 
	Eigen::Vector2f eig_psi_soft_bounds;
	Eigen::VectorXf eig_u_soft_bounds;
	float eig_z_soft_ceiling_bounds;

	Eigen::VectorXf X0;
	Eigen::VectorXf V0;

	Eigen::MatrixXf eig_boundary_conditions;

	float* x0;
	Eigen::VectorXf eig_goal_difference;

	float length, width, height, half_camera_FOV;

	Eigen::MatrixXf X_goal;

};

#endif

