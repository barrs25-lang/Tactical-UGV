// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 16th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file implements the primal-dual interior-point QP solver that produces the final control
// request: the box-constraint/weighting-matrix setup, the log-barrier/KS-approximation hard and soft
// constraint machinery, and fmpcsolve itself (which drives dnudz/gfgphp/rd_tilde_rp/resdresp).
// End File Decscription ################################################################################

#include <f_mpc_uncut.h>
#include <f_mpc_globals.h>
#include <line_search.h>
#include <dnudz.h>
#include <gfgphp.h>
#include <rdrp.h>
#include <resdresp.h>

// Member function of F_MPC_UNCUT object
// fsat: computes the fsat function, whose output is in the range [0,1]
// INPUTS: pointer to a matrix storing the value to check
// OUTPUTS: float representing the output of fsat function, [0,1]
float F_MPC_UNCUT::fsat(Eigen::MatrixXf* arg)
{

	// define float representing the input's norm (either Equi-induced Euclidean norm for matrix or Euclidean norm for vector)
	float arg_norm = (*arg).norm();

	// If the norm is less than 1, set the output to 1.0
	if (arg_norm < 1)
	{
		
		return 1.0f;
	
	} // if (arg_norm < 1)
	else
	{
		
		return 1.0f/arg_norm;

	} // if (arg_norm < 1)

	cout << "<FMPC> fsat complete" << endl;

} // float F_MPC_UNCUT::fsat(Eigen::MatrixXf* arg)


// Member function of F_MPC_UNCUT object
// update_weighting_matrices: updates the tilde_R matrix in the cost function
// INPUTS: integer representing the segment number
// OUTPUTS: none
void F_MPC_UNCUT::update_weighting_matrices(int segment_number, bool success)
{

	// Declare float to store update weighting
	float recast_weight;

	// Matrix storing
	Eigen::MatrixXf priority_weight;
	Eigen::MatrixXf fsat_arg, temp;

	temp = Eigen::MatrixXf::Zero(3,1);
	fsat_arg = Eigen::MatrixXf::Zero(3,1);
	priority_weight = Eigen::MatrixXf::Zero(quadrotor.n,1);
	temp(0,0) = quadrotor.X0(0);       // x
	temp(1,0) = quadrotor.X0(1);       // y
	temp(2,0) = current_position(2);   // z (tracked separately from chi, see current_position)

	// If the planned path exists, and we are not looking past the end of the path
	if (localPath.rows() > 0 && traj_iterator > 0 && traj_iterator < localPath.rows())
	{
	
		// Compute fsat's argument
		fsat_arg = mpc_params.mu_11*( localPath.row(traj_iterator).transpose() - r_obs );
	
	}
	else
	{
		
		fsat_arg = mpc_params.mu_11*( temp - r_obs ); 

	} // if (localPath.rows() > 0 && traj_iterator < localPath.rows())

	// Compute the weighting
	recast_weight = (1 + mpc_params.mu_10*(1 - fsat( &fsat_arg ) ) );

	cout << "pos: " << temp << endl;
	cout << "r_obs: " << r_obs << endl;

	// If the planned path exists
	if (localPath.rows() > 0 && traj_iterator > 0 && traj_iterator < localPath.rows())
	{

		priority_weight.block(0,0,2,1) = mpc_params.mu_10 * localPath.row(traj_iterator).transpose().block(0,0,2,1) + (1 - mpc_params.mu_10)*fsat( &fsat_arg ) * r_obs.block(0,0,2,1);

	}
	else
	{

		priority_weight.block(0,0,2,1) = mpc_params.mu_10 * temp.block(0,0,2,1) + (1 - mpc_params.mu_10)*fsat( &fsat_arg ) * r_obs.block(0,0,2,1);

	} // if (localPath.rows() > 0 && traj_iterator > 0)

	// Reset the matrix weighting the control input (it does not have a time dependence unlike the others)
	mpc_params.eig_R_lambda = mpc_params.eig_R_lambda_tilde; // have to reset this matrix due to the discounting

	Eigen::MatrixXf RHS_LMI = Eigen::MatrixXf::Zero(quadrotor.n,quadrotor.n);

	Eigen::VectorXcf discrete_LMI_eigenvalues;

	string message;	

	// Iterate over the number of time steps
	for (unsigned short int i = 0; i < mpc_params.T; i++)
	{
		
		// Set the weighting matrices to zero
		mpc_params.eig_R_rk[i] = Eigen::MatrixXf::Zero(quadrotor.n,quadrotor.n);
		mpc_params.eig_R_r_lambda_k[i] = Eigen::MatrixXf::Zero(quadrotor.n,quadrotor.m);
		mpc_params.eig_q_rk[i] = Eigen::MatrixXf::Zero(quadrotor.n,1);
		mpc_params.eig_q_lambda_k[i] = Eigen::MatrixXf::Zero(quadrotor.m,1);
		mpc_params.eig_q[i] = Eigen::MatrixXf::Zero(quadrotor.n,1);

		// Compute the weighting matrices
		if (success || segment_number == 0)
		{

			mpc_params.eig_R_rk[i] = recast_weight*(1/g_barrier(i,0))*mpc_params.eig_R_r_tilde;
			mpc_params.eig_R_r_lambda_k[i] = recast_weight*(1/g_barrier(i,0))*mpc_params.eig_R_r_lambda_tilde;
			mpc_params.eig_q_rk[i] = recast_weight*mpc_params.eig_q_r_tilde;
			mpc_params.eig_q_lambda_k[i] = mpc_params.eig_q_lambda_tilde - 0 * mpc_params.eig_R_r_lambda_tilde.transpose() * priority_weight;
			mpc_params.eig_R_lambda = (1/g_barrier(i,0))*mpc_params.eig_R_lambda_init;
			mpc_params.eig_q[i] = mpc_params.eig_q_rk[i];

			// Compute the condition
			RHS_LMI = mpc_params.eig_R_rk[i] - 2*mpc_params.eig_R_r_lambda_k[i]*mpc_params.eig_R_lambda.inverse()*mpc_params.eig_R_r_lambda_k[i].transpose();

			// Compute the eigenvalues
			discrete_LMI_eigenvalues = RHS_LMI.eigenvalues();

			// Iterate over the number of states
			for (unsigned short int j = 0; j < quadrotor.n; j++)
			{

				if (discrete_LMI_eigenvalues(j).real() <= 0.0)
				{
					cout << "discrete_LMI_eigenvalues(j).real(): " << discrete_LMI_eigenvalues(j).real() << endl;
					cout << "mpc_params.eig_R_rk[i]: " << endl << mpc_params.eig_R_rk[i] << endl;
					message = "Schur complement condition time step " + to_string(i) + " state " + to_string(j) + ", resetting " + to_string(i) + "th block.";
					cout << message << endl;
					log_f_mpc.writeToLog(message);

					mpc_params.eig_R_rk[i] = mpc_params.eig_R_r_tilde;
					mpc_params.eig_R_r_lambda_k[i] = mpc_params.eig_R_r_lambda_tilde;
					mpc_params.eig_q[i] = mpc_params.eig_q_r_tilde;

					break;

				} // if (discrete_LMI_eigenvalues(i).real() <= 0)

			} // for (short int i = 0; i < n_in; i++)

		}
		else
		{
			mpc_params.eig_R_rk[i] = mpc_params.eig_R_r_tilde;
			mpc_params.eig_R_r_lambda_k[i] = mpc_params.eig_R_r_lambda_tilde;
			mpc_params.eig_q_rk[i] = Eigen::MatrixXf::Zero(quadrotor.n,1);
			mpc_params.eig_q_lambda_k[i] = 0*mpc_params.eig_q_lambda_tilde - 0 * mpc_params.eig_R_r_lambda_tilde.transpose() * priority_weight;
			mpc_params.eig_R_lambda = mpc_params.eig_R_lambda_init;
			mpc_params.eig_q[i] = mpc_params.eig_q_rk[i];
		}

	} // for (int i = 0; i < mpc_params.T; i++)
	
	cout << "<FMPC> update_weighting_matrices complete" << endl;

} // void F_MPC_UNCUT::update_weighting_matrices(int segment_number)


// Member function of F_MPC_UNCUT object
// update_and_discount_weighting_matrices: updates the tilde_R matrix in the cost function and discounts them
// INPUTS: integer representing the segment number
// OUTPUTS: none
void F_MPC_UNCUT::update_and_discount_weighting_matrices(int segment_number, bool success)
{

	// Update the \tilde{R} matrix
	update_weighting_matrices(segment_number, success);

	// Update the discount factor
	mpc_params.beta_pl = pow(mpc_params.mu_21,segment_number);

	// Iterate over the number of time steps 
	for (unsigned short int i = 0; i < mpc_params.T; i++)
	{
		
		mpc_params.eig_R_rk[i] = mpc_params.eig_R_rk[i]*mpc_params.beta_pl;
		mpc_params.eig_R_r_lambda_k[i] = mpc_params.eig_R_r_lambda_k[i]*mpc_params.beta_pl;
		mpc_params.eig_q_rk[i] = mpc_params.eig_q_rk[i]*mpc_params.beta_pl;
		mpc_params.eig_q_lambda_k[i] = mpc_params.eig_q_lambda_k[i]*mpc_params.beta_pl;
		mpc_params.eig_q[i] = mpc_params.eig_q[i]*mpc_params.beta_pl;
		mpc_params.eig_R_lambda = mpc_params.eig_R_lambda*mpc_params.beta_pl;

	} // for (int i = 0; i < mpc_params.T; i++)

	cout << "<FMPC> update_and_discount_weighting_matrices complete" << endl;
	
} // void F_MPC_UNCUT::update_and_discount_weighting_matrices(int segment_number)


// Member function of F_MPC_UNCUT object
// compute_box_constraints: computes box constraints used to compute
// later segements of the planned trajectory
// INPUTS: integer storing the current segment number, and an integer indicating
// the index of the goal position in the planned path
// OUTPUTS: none
void F_MPC_UNCUT::compute_box_constraints(int segment_number, int goalStride)
{

	// Define Eigen::MatrixXf to store the current and previous goal position 
	Eigen::MatrixXf rk1 = Eigen::MatrixXf::Zero(3,1);
	Eigen::MatrixXf rk2 = Eigen::MatrixXf::Zero(3,1);

	// Float storing the angle between the x-axis
	float dir;

	// Declare Eigen::Vector3f to store points defining box constraints
	Eigen::Vector3f n; 		// normal vector
	Eigen::Vector3f per;	// vector perpendicular to n
	Eigen::Vector3f p1;		// p1 is a point on the first box constraint
	Eigen::Vector3f p2;		// p2 is a point on the second box constraint
	Eigen::Vector3f p3;		// p3 is a point on the third box constraint
	Eigen::Vector3f p4;		// p4 is a point on the fourth box constraint

	// Floats capturing offsets for each box constraint 
	float dp1,dp2,dp3,dp4;

	// Define Eigen::MatrixXf to store the parameters for the box constraints (a,b as in a plane equation over x,y)
	Eigen::MatrixXf temp_Fx = Eigen::MatrixXf::Zero(4,quadrotor.n); // Stores ai, bi in the first two columns, zeros in the remaining columns
	Eigen::MatrixXf temp_fx = Eigen::MatrixXf::Zero(4,1);			// Store di in the ith row

	// If the goal stride does not exceed the length of the path
	if (goalStride < localPath.rows() && goalStride-1 >= 0)
	{
		
		// Set rk1 and rk2 (previous waypoint and current waypoint for this segment)
		rk1 = localPath.row(goalStride-1).transpose();
		rk2 = localPath.row(goalStride).transpose();

	} // if (goalStride < localPath.rows())
	else
	{
		
		// Set rk1 and rk2 (previous waypoint and current waypoint for this segment)
		rk1 = localPath.row(localPath.rows()-2).transpose();
		rk2 = localPath.row(localPath.rows()-1).transpose();

	} // if (goalStride < localPath.rows())


	// If the points are not too close together (on voxel map with resolution = 0.2m, this means the points are at the same xy position, and different z-positions)
	if ( (rk1-rk2).norm() > 0.1 )
	{
		
		dir = atan2(rk2(1,0)-rk1(1,0),rk2(0,0)-rk1(0,0));
		// dir = atan2(rk2(0,0)-rk1(0,0),rk2(1,0)-rk1(1,0));

	} // if ( (rk1-rk2).norm() > 0.1 )
	else
	{
		
		dir = 0;
	
	} // if ( (rk1-rk2).norm() > 0.1 )

	// Set the coefficients of the normal vector
	n(0) = sin(dir);
	n(1) = cos(dir);
	n(2) = 0;

	// Set the coefficients of the perpendicular vector
	per(0) = -n(1);
	per(1) = n(0);
	per(2) = 0;

	float offset = 6.0;

	// Compute the points on the planes
	p1 = rk1 + offset * ( per - n );
	p2 = rk1 + offset * (- per - n );
	p3 = rk2 + offset * ( per + n );
	p4 = rk2 + offset * (- per + n );

	// Scalar representation of the points on the planes
	dp1 = n.dot(p1);
	dp2 = per.dot(p2);
	dp3 = -per.dot(p3);
	dp4 = -n.dot(p4);

	// Flip the sign for proper comparison
	dp1*=-1;dp2*=-1;dp3*=-1;dp4*=-1;

	// Set the matrix storing the plane coefficients (x,y columns only; chi has no z/heading state)
	temp_Fx.block(0,0,1,2) = -n.head(2).transpose();
	temp_Fx.block(1,0,1,2) = -per.head(2).transpose();
	temp_Fx.block(2,0,1,2) = per.head(2).transpose();
	temp_Fx.block(3,0,1,2) = n.head(2).transpose();

	// Set the vector storing the offset coefficients
	temp_fx(0,0) = dp1;
	temp_fx(1,0) = dp2;
	temp_fx(2,0) = dp3;
	temp_fx(3,0) = dp4;

	// Set the hard constraint matrix
	quadrotor.eig_Fx_hard.block(0,0,4,quadrotor.n) = temp_Fx;

	// Store the hard constraint matrix in P
	mpc_params.P[segment_number].block(0,0,4,quadrotor.n) = temp_Fx;

	// Iterate over the number of time steps
	for (int i = 0; i < mpc_params.T; i++)
	{

		quadrotor.eig_x_hard_bounds.block(0,i,4,1) = temp_fx;
		mpc_params.h[segment_number].block(0,i,4,1) = temp_fx;

	} // for (int i = 0; i < mpc_params.T; i++)

	cout << "<FMPC> compute_box_constraints complete" << endl;

} // void F_MPC_UNCUT::compute_box_constraints(int segment_number, int goalStride)


// Member function of F_MPC_UNCUT object
// fmpc_hard_constraints: computes box constraints used to compute
// later segements of the planned trajectory
// INPUTS: pointer to eigen matrix storing the state, integer representing the segment number, 
// integer representing the index of the goal in the planned path
// OUTPUTS: none
void F_MPC_UNCUT::fmpc_hard_constraints(Eigen::MatrixXf* eig_X, int segment_number, int goalStride, int numFailures)
{

	// Delete float arrays storing constraint data
	delete[] bomt.FxX_array;
	delete[] bomt.dx_array;

	// If the current segment is the first
	if (segment_number == 0)
	{

		// 28 for the initial boundary conditions (chi has no heading/altitude state to constrain)
		quadrotor.num_hard_x = collisionConstraints.rows() + 28;
		int num_obstacle_constraints = quadrotor.num_hard_x - 28;

		// Set new float arrays to store constraint data
		bomt.FxX_array = new float[quadrotor.num_hard_x*mpc_params.T];
		bomt.dx_array = new float[quadrotor.num_hard_x*mpc_params.T];

		// Set the values of the elements stored in the containers above to zero
		memset(bomt.FxX_array,0,quadrotor.num_hard_x*mpc_params.T*sizeof(*bomt.FxX_array));
		memset(bomt.dx_array,0,quadrotor.num_hard_x*mpc_params.T*sizeof(*bomt.dx_array));

		// Allocate memory for the Eigen::Matrix versions
		bomt.FxtildeX = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> (bomt.FxX_array, quadrotor.num_hard_x, mpc_params.T);
		bomt.dx = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> (bomt.dx_array, quadrotor.num_hard_x, mpc_params.T);

		// Set the size of the constraint matrix Fx and fx
		quadrotor.eig_Fx_hard = Eigen::MatrixXf::Zero(quadrotor.num_hard_x,quadrotor.n);
		quadrotor.eig_x_hard_bounds = Eigen::MatrixXf::Zero(quadrotor.num_hard_x,mpc_params.T);

		// Total number of constraints for MPC problem
		mpc_params.l = quadrotor.eig_x_hard_bounds.rows() + quadrotor.eig_u_hard_bounds.size();

		// Iterate over the number of collision avoidance constraints (x,y columns only; chi has no z state)
		for (unsigned short int i = 0; i < num_obstacle_constraints; i++)
		{
			// Set the coefficient of the collision avoidance constraints
			quadrotor.eig_Fx_hard(i,0) = collisionConstraints(i,0);
			quadrotor.eig_Fx_hard(i,1) = collisionConstraints(i,1);

			// Iterate over the number of constraints
			for (unsigned short int j = 0; j < mpc_params.T; j++)
			{

				quadrotor.eig_x_hard_bounds(i,j) = collisionConstraints(i,3);

			} // for (unsigned short int j = 0; j < mpc_params.T; j++)

		} // for (unsigned short int i = 0; i < num_obstacle_constraints; i++)

		// Set the boundary conditions
		quadrotor.eig_Fx_hard.block(num_obstacle_constraints,0,2*quadrotor.n,quadrotor.n) = quadrotor.eig_Fx_hard_boundary_conditions;

		// Iterate over the number of time steps
		for (unsigned short int j = 0; j < mpc_params.T; j++)
		{

			// Iterate over the number of states
			for (unsigned short int i = 0; i < quadrotor.n; i++)
			{

				quadrotor.eig_x_hard_boundary_conditions(i,j) = 1000; // Aka big-M constraint
				quadrotor.eig_x_hard_boundary_conditions(i+quadrotor.n,j) = 1000; // Aka big-M constraint

			} // for (unsigned short int i = 0; i < quadrotor.n; i++)

			quadrotor.eig_x_hard_bounds.block(num_obstacle_constraints,j,2*quadrotor.n,1) = quadrotor.eig_x_hard_boundary_conditions.col(j);
		}

		// Set the size of the P matrix and its elements to zero
		mpc_params.P[segment_number] = Eigen::MatrixXf::Zero(mpc_params.l, quadrotor.n+quadrotor.m);

		// Set the size of the h matrix and its elements to zero
		mpc_params.h[segment_number] = Eigen::MatrixXf::Zero(mpc_params.l, mpc_params.T);

		// Fill the P and h matrices
		mpc_params.P[segment_number].block(0,0,quadrotor.num_hard_x,quadrotor.n) = quadrotor.eig_Fx_hard;
		mpc_params.P[segment_number].block(quadrotor.num_hard_x,quadrotor.n,quadrotor.num_hard_u,quadrotor.m) = quadrotor.eig_Fu_hard;
		mpc_params.h[segment_number].block(0,0,quadrotor.num_hard_x,mpc_params.T) = quadrotor.eig_x_hard_bounds;

		// Iterate over the number of time steps
		for (unsigned short int j = 0; j < mpc_params.T; j++)
		{

			mpc_params.h[segment_number].block(quadrotor.num_hard_x,j,quadrotor.num_hard_u,1) = quadrotor.eig_u_hard_bounds;

		} // for (unsigned short int j = 0; j < mpc_params.T; j++)

	} // if (segment_number == 0)
	else if (segment_number < mpc_params.nu_sk-1)
	{

		// State constraints: 4 for box constraints, 28 for boundary conditions
		quadrotor.num_hard_x = 4 + 28;

		// Set the size of the hard constraints and bounds. Their elements are set to zero
		quadrotor.eig_Fx_hard = Eigen::MatrixXf::Zero(quadrotor.num_hard_x,quadrotor.n);
		quadrotor.eig_x_hard_bounds = Eigen::MatrixXf::Zero(quadrotor.num_hard_x,mpc_params.T);

		// New float arrays to store fmpc data
		bomt.FxX_array = new float[quadrotor.num_hard_x*mpc_params.T];
		bomt.dx_array = new float[quadrotor.num_hard_x*mpc_params.T];

		// Set the elements of the containers above to zero
		memset(bomt.FxX_array, 0, quadrotor.num_hard_x*mpc_params.T*sizeof(*bomt.FxX_array));
		memset(bomt.dx_array, 0, quadrotor.num_hard_x*mpc_params.T*sizeof(*bomt.dx_array));

		// Map the containers to their eigen counterparts
		bomt.FxtildeX = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> (bomt.FxX_array, quadrotor.num_hard_x, mpc_params.T);
		bomt.dx = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> (bomt.dx_array, quadrotor.num_hard_x, mpc_params.T);

		// Set the size of P and h, as well as their elements to zero
		mpc_params.P[segment_number] = Eigen::MatrixXf::Zero(quadrotor.num_hard_x+quadrotor.num_hard_u, quadrotor.n+quadrotor.m);
		mpc_params.h[segment_number] = Eigen::MatrixXf::Zero(quadrotor.num_hard_x+quadrotor.num_hard_u, mpc_params.T);

		// Compute box constraints for the segments occuring later in the planned trajectory
		compute_box_constraints(segment_number, goalStride);

		// Set the orientation of boundary conditions
		quadrotor.eig_Fx_hard.block(4,0,2*quadrotor.n,quadrotor.n) = quadrotor.eig_Fx_hard_boundary_conditions;

		// Iterate over the number of time steps
		for (unsigned short int j = 0; j < mpc_params.T; j++)
		{

			// Iterate over the number of states
			for (unsigned short int i = 0; i < quadrotor.n; i++)
			{

				quadrotor.eig_x_hard_boundary_conditions(i,j) = 1000; // Aka big-M constraint
				quadrotor.eig_x_hard_boundary_conditions(i+quadrotor.n,j) = 1000; // Aka big-M constraint

			} // for (int i = 0; i < quadrotor.n; i++)

			// Set the boundary conditions
			quadrotor.eig_x_hard_bounds.block(4,j,2*quadrotor.n,1) = quadrotor.eig_x_hard_boundary_conditions.col(j);

		} // for (unsigned short int j = 0; j < mpc_params.T; j++)

		// Control constraints
		mpc_params.P[segment_number].block(4,0,2*quadrotor.n,quadrotor.n) = quadrotor.eig_Fx_hard_boundary_conditions;
		mpc_params.P[segment_number].block(quadrotor.num_hard_x,quadrotor.n,quadrotor.num_hard_u,quadrotor.m) = quadrotor.eig_Fu_hard;

		// Iterate over the number of time steps
		for (unsigned short int j = 0; j < mpc_params.T; j++)
		{

			mpc_params.h[segment_number].block(4,j,2*quadrotor.n,1) = quadrotor.eig_x_hard_boundary_conditions.col(j);
			mpc_params.h[segment_number].block(quadrotor.num_hard_x,j,quadrotor.num_hard_u,1) = quadrotor.eig_u_hard_bounds;

		} // for (unsigned short int j = 0; j < mpc_params.T; j++)

	} // else if (segment_number < mpc_params.nu_sk-1)
	else
	{
		// State constraints: 4 for box constraints, 28 for boundary conditions
		quadrotor.num_hard_x = 4 + 28;

		// Set the size of the hard constraints and bounds. Their elements are set to zero
		quadrotor.eig_Fx_hard = Eigen::MatrixXf::Zero(quadrotor.num_hard_x,quadrotor.n);
		quadrotor.eig_x_hard_bounds = Eigen::MatrixXf::Zero(quadrotor.num_hard_x,mpc_params.T);

		// New float arrays to store fmpc data
		bomt.FxX_array = new float[quadrotor.num_hard_x*mpc_params.T];
		bomt.dx_array = new float[quadrotor.num_hard_x*mpc_params.T];

		// Set the elements of the containers above to zero
		memset(bomt.FxX_array, 0, quadrotor.num_hard_x*mpc_params.T*sizeof(*bomt.FxX_array));
		memset(bomt.dx_array, 0, quadrotor.num_hard_x*mpc_params.T*sizeof(*bomt.dx_array));

		// Map the containers to their eigen counterparts
		bomt.FxtildeX = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> (bomt.FxX_array, quadrotor.num_hard_x, mpc_params.T);
		bomt.dx = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> (bomt.dx_array, quadrotor.num_hard_x, mpc_params.T);

		// Set the size of P and h, as well as their elements to zero
		mpc_params.P[segment_number] = Eigen::MatrixXf::Zero(quadrotor.num_hard_x+quadrotor.num_hard_u, quadrotor.n+quadrotor.m);
		mpc_params.h[segment_number] = Eigen::MatrixXf::Zero(quadrotor.num_hard_x+quadrotor.num_hard_u, mpc_params.T);

		// Compute box constraints for the segments occuring later in the planned trajectory
		compute_box_constraints(segment_number, goalStride);

		// Set the orientation of boundary conditions
		quadrotor.eig_Fx_hard.block(4,0,2*quadrotor.n,quadrotor.n) = quadrotor.eig_Fx_hard_boundary_conditions;

		// Iterate over the number of time steps
		for (unsigned short int j = 0; j < mpc_params.T; j++)
		{

			// Iterate over the number of states
			for (unsigned short int i = 0; i < quadrotor.n; i++)
			{

				quadrotor.eig_x_hard_boundary_conditions(i,j) = 1000; // Aka big-M constraint
				quadrotor.eig_x_hard_boundary_conditions(i+quadrotor.n,j) = 1000; // Aka big-M constraint

			} // for (unsigned short int i = 0; i < quadrotor.n; i++)

			quadrotor.eig_x_hard_bounds.block(4,j,2*quadrotor.n,1) = quadrotor.eig_x_hard_boundary_conditions.col(j);

		} // for (unsigned short int j = 0; j < mpc_params.T; j++)

		// Control constraints
		mpc_params.P[segment_number].block(4,0,2*quadrotor.n,quadrotor.n) = quadrotor.eig_Fx_hard_boundary_conditions;
		mpc_params.P[segment_number].block(quadrotor.num_hard_x,quadrotor.n,quadrotor.num_hard_u,quadrotor.m) = quadrotor.eig_Fu_hard;

		// Iterate over the number of time steps
		for (unsigned short int j = 0; j < mpc_params.T; j++)
		{

			mpc_params.h[segment_number].block(4,j,2*quadrotor.n,1) = quadrotor.eig_x_hard_boundary_conditions.col(j);
			mpc_params.h[segment_number].block(quadrotor.num_hard_x,j,quadrotor.num_hard_u,1) = quadrotor.eig_u_hard_bounds;

		} // for (unsigned short int j = 0; j < mpc_params.T; j++)

	} // if (segment_number == 0)

	cout << "<FMPC> fmpc_hard_constraints complete" << endl;

} // void F_MPC_UNCUT::fmpc_hard_constraints(Eigen::MatrixXf* eig_X, int segment_number, int goalStride, int numFailures)


// Member function of F_MPC_UNCUT object
// fmpc_soft_constraints: computes or organizes the constraints
// INPUTS: pointer to eigen matrix storing the state, integer representing the segment number, 
// integer representing the index of the goal in the planned path
// OUTPUTS: none
void F_MPC_UNCUT::fmpc_soft_constraints(Eigen::MatrixXf* eig_X, int segment_number, int goalStride)
{

	// Delete old storage containers
	delete[] bomt.FxtildeX_array;
	delete[] bomt.d_tilde_x_array;
	delete[] bomt.d_hat_x_array;
	delete[] bomt.rho_i_x_array;

	// Set the number of soft constraints on the state (the number of soft constraints on the control is constant)
	quadrotor.num_soft_x = quadrotor.num_hard_x;

	// Set new storage contaniners for a few
	bomt.FxtildeX_array = new float[quadrotor.num_soft_x*mpc_params.T];
	bomt.d_tilde_x_array = new float[quadrotor.num_soft_x*mpc_params.T];
	bomt.d_hat_x_array = new float[quadrotor.num_soft_x*mpc_params.T];
	bomt.rho_i_x_array = new float[quadrotor.num_soft_x];

	// Set the elements of the new containers to zero
	memset(bomt.FxtildeX_array, 0, quadrotor.num_soft_x*mpc_params.T*sizeof(*bomt.FxtildeX_array));
	memset(bomt.d_tilde_x_array, 0, quadrotor.num_soft_x*mpc_params.T*sizeof(*bomt.d_tilde_x_array));
	memset(bomt.d_hat_x_array, 0, quadrotor.num_soft_x*mpc_params.T*sizeof(*bomt.d_hat_x_array));
	memset(bomt.rho_i_x_array, 0, quadrotor.num_soft_x*sizeof(*bomt.rho_i_x_array));

	// Map the containers to Eigen Matrices (easier to perform linear algebra using Eigen)
	bomt.FxtildeX = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> (bomt.FxtildeX_array, quadrotor.num_hard_x, mpc_params.T);
	bomt.d_tilde_x = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> (bomt.d_tilde_x_array, quadrotor.num_hard_x, mpc_params.T);
	bomt.d_hat_x = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> (bomt.d_hat_x_array, quadrotor.num_hard_x, mpc_params.T);
	bomt.rho_i_x = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> (bomt.rho_i_x_array, quadrotor.num_hard_x, 1);

	// Resize the vector storing the value of rho for each constraint
	bomt.rho_i_x_vec.resize(quadrotor.num_soft_x);

	// Set the soft constraint matrix and bound size and elements to zero
	quadrotor.eig_Fx_soft = Eigen::MatrixXf::Zero(quadrotor.num_soft_x,quadrotor.n);
	quadrotor.eig_x_soft_bounds = Eigen::MatrixXf::Zero(quadrotor.num_soft_x,mpc_params.T);

	if (segment_number == 0)
	{

		// Compute the number of collision avoidance constraints
		int num_obstacle_constraints = quadrotor.num_hard_x - 28;

		// ******************************************** //
		// Set the soft collision avoidance constraints //
		// ******************************************** //

		// Iterate over the number of collision avoidance constraints (x,y columns only; chi has no z state)
		for (int i = 0; i < num_obstacle_constraints; i++)
		{

			quadrotor.eig_Fx_soft(i,0) = collisionConstraints(i,0);
			quadrotor.eig_Fx_soft(i,1) = collisionConstraints(i,1);

			// Iterate over the number of time steps
			for (int j = 0; j < mpc_params.T; j++)
			{

				// Set the bounds for the constraint. They are constant over the time horizon.
				quadrotor.eig_x_soft_bounds(i,j) = collisionConstraints(i,3)*mpc_params.collisionAvoidanceSoftConstraintOffset;

			} // for (int j = 0; j < mpc_params.T; j++)

		} // for (int i = 0; i < num_obstacle_constraints; i++)

		// Set the remaining constraints on boundary conditions
		quadrotor.eig_Fx_soft.block(num_obstacle_constraints,0,2*quadrotor.n,quadrotor.n) = quadrotor.eig_Fx_hard_boundary_conditions;

		// ******************************* //
		// Compute the boundary conditions //
		// ******************************* //

		// Iterate over the number of time steps
		for (int j = 0; j < mpc_params.T; j++)
		{

			// Iterate over the number of states
			for (int i = 0; i < quadrotor.n; i++)
			{

				// On first time step, for x,y states
				if (j == 0 && i < 2)
				{

					quadrotor.eig_x_soft_boundary_conditions(i,j) = (*eig_X)(i,0) + mpc_params.bc_epsilon*mpc_params.collisionAvoidanceSoftConstraintOffset;
					quadrotor.eig_x_soft_boundary_conditions(i+quadrotor.n,j) = - (*eig_X)(i,0) + mpc_params.bc_epsilon*mpc_params.collisionAvoidanceSoftConstraintOffset;

				} // if (j == 0 && i < 2)
				else
				{

					quadrotor.eig_x_soft_boundary_conditions(i,j) = 1000; // Aka big-M constraint
					quadrotor.eig_x_soft_boundary_conditions(i+quadrotor.n,j) = 1000; // Aka big-M constraint

				} // if (j == 0 && i < 2)

			} // for (int i = 0; i < quadrotor.n; i++)

			// Set the bounds for the soft constraints on boundary conditions
			quadrotor.eig_x_soft_bounds.block(num_obstacle_constraints,j,2*quadrotor.n,1) = quadrotor.eig_x_soft_boundary_conditions.col(j);

		} // for (int j = 0; j < mpc_params.T; j++)

		// ********************************************************** //
		// Compute the vector h, used in computation of the objective //
		// ********************************************************** //

		// Set the size of h and its elements to zero
		mpc_params.h_tilde[segment_number] = Eigen::MatrixXf::Zero(mpc_params.l, mpc_params.T);
		mpc_params.h_tilde[segment_number].block(0,0,quadrotor.num_soft_x,mpc_params.T) = quadrotor.eig_x_soft_bounds;

		// Iterate over the number of time steps
		for (unsigned short int j = 0; j < mpc_params.T; j++)
		{

			mpc_params.h_tilde[segment_number].block(quadrotor.num_soft_x,j,quadrotor.num_soft_u,1) = quadrotor.eig_u_soft_bounds;

		} // for (int j = 0; j < mpc_params.T; j++)

	}
	else if (segment_number < mpc_params.nu_sk-1)
	{

		// Set the soft constraints
		quadrotor.eig_Fx_soft = quadrotor.eig_Fx_hard;

		// ******************************* //
		// Compute the boundary conditions //
		// ******************************* //

		// Iterate over the number of time steps
		for (unsigned short int j = 0; j < mpc_params.T; j++)
		{

			// Iterate over the number of states
			for (unsigned short int i = 0; i < quadrotor.n; i++)
			{

				if (j == 0)
				{
					quadrotor.eig_x_soft_boundary_conditions(i,j) = (*eig_X)(i,0) + mpc_params.bc_epsilon*0.95;
					quadrotor.eig_x_soft_boundary_conditions(i+quadrotor.n,j) = - (*eig_X)(i,0) + mpc_params.bc_epsilon*0.95;
				}
				else
				{
					quadrotor.eig_x_soft_boundary_conditions(i,j) = 1000; // Aka big-M constraint
					quadrotor.eig_x_soft_boundary_conditions(i+quadrotor.n,j) = 1000; // Aka big-M constraint
				}

			} // for (unsigned short int i = 0; i < quadrotor.n; i++)

			// Set the soft bounds for the box constraints
			quadrotor.eig_x_soft_bounds.block(0,j,4,1) = quadrotor.eig_x_hard_bounds.block(0,j,4,1);

		} // for (unsigned short int j = 0; j < mpc_params.T; j++)

		// Iterate over the number of time steps
		for (unsigned short int i = 0; i < mpc_params.T; i++)
		{

			// Set the soft bounds for the boundary conditions
			quadrotor.eig_x_soft_bounds.block(4,i,2*quadrotor.n,1) = quadrotor.eig_x_soft_boundary_conditions.col(i);

		}

		// ********************************************************** //
		// Compute the vector h, used in computation of the objective //
		// ********************************************************** //

		// Set the size of h and its elements to zero
		mpc_params.h_tilde[segment_number] = Eigen::MatrixXf::Zero(4+2*quadrotor.n+quadrotor.num_soft_u, mpc_params.T);
		mpc_params.h_tilde[segment_number].block(0,0,4+2*quadrotor.n,mpc_params.T) = quadrotor.eig_x_soft_bounds;

		// Iterate over the number of time steps
		for (unsigned short int j = 0; j < mpc_params.T; j++)
		{

			mpc_params.h_tilde[segment_number].block(4+2*quadrotor.n,j,quadrotor.num_soft_u,1) = quadrotor.eig_u_soft_bounds;

		} // for (unsigned short int j = 0; j < mpc_params.T; j++)

	} // else if (segment_number < mpc_params.nu_sk-1)
	else
	{

		// Set the soft constraints
		quadrotor.eig_Fx_soft = quadrotor.eig_Fx_hard;

		// ******************************* //
		// Compute the boundary conditions //
		// ******************************* //

		// Iterate over the number of time steps
		for (int j = 0; j < mpc_params.T; j++)
		{

			// Iterate over the number of states
			for (int i = 0; i < quadrotor.n; i++)
			{
				if (j == 0)
				{
					quadrotor.eig_x_soft_boundary_conditions(i,j) = (*eig_X)(i,0) + mpc_params.bc_epsilon*0.95;
					quadrotor.eig_x_soft_boundary_conditions(i+quadrotor.n,j) = - (*eig_X)(i,0) + mpc_params.bc_epsilon*0.95;
				}
				else if (j == mpc_params.T-1 && i < 2)
				{

					quadrotor.eig_x_soft_boundary_conditions(i,j) = localPath(goalStride-1,i) + mpc_params.bc_epsilon*0.95; // Aka big-M constraint
					quadrotor.eig_x_soft_boundary_conditions(i+quadrotor.n,j) = -localPath(goalStride-1,i) + mpc_params.bc_epsilon*0.95; // Aka big-M constraint

				} // if (j == mpc_params.T-1 && i < 2)
				else
				{

					quadrotor.eig_x_soft_boundary_conditions(i,j) = 1000; // Aka big-M constraint
					quadrotor.eig_x_soft_boundary_conditions(i+quadrotor.n,j) = 1000; // Aka big-M constraint

				} // if (j == mpc_params.T-1 && i < 2)

			} // for (int i = 0; i < quadrotor.n; i++)

			// Set the bounds for the box soft constraints
			quadrotor.eig_x_soft_bounds.block(0,j,4,1) = quadrotor.eig_x_hard_bounds.block(0,j,4,1);

		} // for (int j = 0; j < mpc_params.T; j++)

		// Set the bounds for soft constraints on the boundary conditions
		for (int i = 0; i < mpc_params.T; i++)
		{

			quadrotor.eig_x_soft_bounds.block(4,i,2*quadrotor.n,1) = quadrotor.eig_x_soft_boundary_conditions.col(i);

		} // for (int i = 0; i < mpc_params.T; i++)

		// ********************************************************** //
		// Compute the vector h, used in computation of the objective //
		// ********************************************************** //

		mpc_params.h_tilde[segment_number] = Eigen::MatrixXf::Zero(4+2*quadrotor.n+quadrotor.num_soft_u, mpc_params.T);
		mpc_params.h_tilde[segment_number].block(0,0,4+2*quadrotor.n,mpc_params.T) = quadrotor.eig_x_soft_bounds;
		for (int j = 0; j < mpc_params.T; j++)
		{

			mpc_params.h_tilde[segment_number].block(4+2*quadrotor.n,j,quadrotor.num_soft_u,1) = quadrotor.eig_u_soft_bounds;

		} // for (int j = 0; j < mpc_params.T; j++)

	} // if (segment_number == 0)

	cout << "<FMPC> fmpc_soft_constraints complete" << endl;

} // void F_MPC_UNCUT::fmpc_soft_constraints(Eigen::MatrixXf* eig_X, int segment_number, int goalStride)


// Member function of F_MPC_UNCUT
// fmpcsolve: Solves the model predictive control problem minimize (42) subject to (21), (23), and (34)
// input: quadrotor, mpc_params, bag_of_many_things: structures containing system and problem parameters; see <structures.h> for details
// output: X, U: matrices containing the state and control inputs across the time horizon, respectively. The ith column corresponds to the state or control input at time i*delta_t
bool F_MPC_UNCUT::fmpcsolve(Eigen::MatrixXf* X, Eigen::MatrixXf* U, int segment_number)
{

	// Declare floats to store the step length and residual values
	float s;
	float res, newres;

	//initialize / reset some variables
	mpc_params.mu_12 = mpc_params.mu_12_initial;
	mpc_params.mu_13 = mpc_params.mu_13_initial;

	// Iterate over the number of states
	for (int i = 0; i < quadrotor.n; i++)
	{

		// Iterate over the number of time steps
		for (int j = 0; j < mpc_params.T; j++)
		{
			
			bomt.nu(i,j) = 0.0;

		} // for (int j = 0; j < mpc_params.T; j++)

	} // for (int i = 0; i < quadrotor.n; i++)

	// Calculate Ax and store into b for the initial step
	bomt.b.col(0) = quadrotor.eig_A * (*X).col(0);

	// Indicate to underlying code that this is the first run
	bomt.initial_run = true;

	// Setup timing variables
	auto current_time = std::chrono::high_resolution_clock::now();
	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed = end_time - current_time;

	// Iterate over the number of iterations
	for ( unsigned short int i = 0; i < mpc_params.num_iterations; ++i )
	{		

		// These functions are used to determine feasible directions \deltaZ and \delta nu 
		gfgphp(&quadrotor, &mpc_params, X, U, &bomt);
		dtildhat(&quadrotor, &mpc_params, X, U, &bomt);
		rd_tilde_rp(&quadrotor, &mpc_params, X, U, &bomt);
		res = resdresp(bomt);

		// Indicate that the first run has begun
		bomt.initial_run = false;

		// If the residual is nan or inf, return
		if (isnan(res) || isinf(res))
		{
			log_f_mpc.writeToLog("<FMPC-SOLVE> Residual not finite.");
			cout << "<FMPC-SOLVE> Residual not finite." << endl;

			// Reset a few things
			bomt.res_p = 0.0;
			bomt.res_dx = 0.0;
			bomt.res_du = 0.0;
			bomt.rp = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
			bomt.rd_tilde_x = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
			bomt.Ctnu_x = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
			bomt.dz_x = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
			bomt.rd_tilde_u = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
			bomt.Ctnu_u = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
			bomt.dz_u = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
			// bomt.TwoQX = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
			// bomt.TwoRU = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
			bomt.dx = Eigen::MatrixXf::Zero(quadrotor.num_hard_x,mpc_params.T);
			bomt.du = Eigen::MatrixXf::Zero(quadrotor.num_hard_u,mpc_params.T);
			bomt.d_tilde_x = Eigen::MatrixXf::Zero(quadrotor.num_soft_x,mpc_params.T);
			bomt.d_hat_x = Eigen::MatrixXf::Zero(quadrotor.num_soft_x,mpc_params.T);
			bomt.d_tilde_u = Eigen::MatrixXf::Zero(quadrotor.num_soft_u,mpc_params.T);
			bomt.d_hat_u = Eigen::MatrixXf::Zero(quadrotor.num_soft_u,mpc_params.T);
			bomt.nu = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
			bomt.PtTdt_x = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
			bomt.xdz_x = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
			bomt.PtTdt_u = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
			bomt.udz_u = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
			bomt.newx = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
			bomt.newu = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);

			return false;

		} // if (isnan(res) || isinf(res))

		// If the residual is small enough
		if (res < mpc_params.tol_sq)
			break;

		// Compute search directions
		dnudz(&quadrotor, &mpc_params, &bomt);
		s = 1.0f;

		// ****************************************************************** //
		// feasibility search (iterate until z satisfies all the constraints) //
		// ****************************************************************** //

		// Start a timer
		clock_t startTime = clock();
		double duration = 0.0;
		bool cont = true;
		int fx_count = 0;

		//this loop establishes a trajectory proposal from the start to the goal.
		while (cont)
		{
			// If, for whatever reason, a trajectory proposal cannot be found within a conservative amount of time, abort the process
			duration = (clock() - startTime) / (double)CLOCKS_PER_SEC;
			if(duration > mpc_params.delta_t)
			{

				// If the trajectory proposal after the time above is still infeasible (violates any constraints or boundary conditions)
				// then revert to the ray trajectory or the previous feasible trajectory.
				cout << "returning from duration" << endl;
				return false;

			}

			// Switch between different line search techniques
			switch(mpc_params.line_search_style)
			{
				case 0: 
					s *= mpc_params.beta;
					break;
				case 1: 
					// s = uniform_line_search();
					break;
				case 2: 
					s = dichotomous_line_search(this,X,U,&(bomt.dz_x),&(bomt.dz_u),lsp.dichoto_a1,lsp.dichoto_b1,lsp.dichoto_l,lsp.dichoto_epsilon, segment_number,s);
					break;
			}

			// Move z along the direction of dz by magnitude specified by s and store this temporary answer to zdz (p271)
			bomt.xdz_x = (*X) + s * bomt.dz_x;
			bomt.udz_u = (*U) + s * bomt.dz_u;

			// for (int k = 0; k < mpc_params.T; k++)
			// {
			// 	while (bomt.xdz_x(12,k) > M_PI){ bomt.xdz_x(12,k) -= 2*M_PI; }
			// 	while (bomt.xdz_x(12,k) < -M_PI){ bomt.xdz_x(12,k) += 2*M_PI; }
			// }			

			// Calculate Pz based on zdz calculated above
			bomt.FxX = quadrotor.eig_Fx_hard * bomt.xdz_x;
			bomt.FuU = quadrotor.eig_Fu_hard * bomt.udz_u;

			// Check for hard constraint violations. If any are present, revise the trajectory proposal.
			cont = false;

			// Iterate over the number of hard state constraints
			for(int i = 0; i < quadrotor.num_hard_x; ++i)
			{

				// Iterate over the number of time steps
				for(int j = 0; j <  mpc_params.T ; ++j)
				{

					// if (i != 1 && i != 2)
					// {

						// If the constraint is violated
						if(bomt.FxX(i,j) > quadrotor.eig_x_hard_bounds(i,j))
						{
						
							// if (segment_number > 0)
							// {
							// 	if (i < 8)
							// 	{
									// cout << "state: " << (*X).col(j).transpose() << endl;
									// cout << "goal: " << goal.transpose() << endl;
									// cout << "segment_nu1mber: " << segment_number << endl;
									// cout << "Fx rows" << i << ": " << quadrotor.eig_Fx_hard.row(i) << endl << quadrotor.eig_x_hard_bounds(i,j) << endl;		
									// std::cout << "(i, j)" << i << ", " << j << ": " << bomt.FxX(i,j) << ", " << quadrotor.eig_x_hard_bounds(i,j) << std::endl;
									// // fx_count = 0;
							// 	}
	
							// 	fx_count++;
							// }
							cont = true;
							break;

						} // if(bomt.FxX(i,j) > quadrotor.eig_x_hard_bounds(i,j))

					// } // if (i != 1 && i != 2)

				} // for(int j = 0; j <  mpc_params.T ; ++j)

				if (cont)
				{
					
					break;

				} // if (cont)

			} // for(int i = 0; i < quadrotor.num_hard_x; ++i)

			// Iterate over the number of hard control constraints
			for (int i = 0; i < quadrotor.num_hard_u; i++)
			{

				// Iterate over the number of time steps
				for(int j = 0; j < mpc_params.T; ++j)
				{

					// If the constraint is violated
					if(bomt.FuU(i,j) > quadrotor.eig_u_hard_bounds[i])
					{
						
						// if (fx_count == 5)
						// {
						// 	std::cout << "ctrl: (i, j)" << i << ", " << j << ": " << bomt.FuU(i,j) << " " << quadrotor.eig_u_hard_bounds[i] << std::endl;
							
						// 	fx_count=0;
						// }

						// fx_count++;
						cont = true;

					} // if(bomt.FuU(i,j) > quadrotor.eig_u_hard_bounds[i])

				} // for(int j = 0; j < mpc_params.T; ++j)
				
				if (cont)
				{
					
					break;

				} // if (cont)	

			} // for (int i = 0; i < quadrotor.num_hard_u; i++)

		} 

		if (mpc_params.line_search_style == 0)
		{
			s /= mpc_params.beta; //through the changes made i the preceding loop, s is multiplied by beta one more time on exit. This corrects that.
		}
		// s = golden_section_line_search();
		// s = fibonacci_line_search();

		// Move z along the direction of dz by magnitude specified by s and store this temporary answer to zdz (p271)
		bomt.xdz_x = (*X) + s * bomt.dz_x;
		bomt.udz_u = (*U) + s * bomt.dz_u;

		// for (int k = 0; k < mpc_params.T; k++)
		// {
		// 	while (bomt.xdz_x(12,k) > M_PI){ bomt.xdz_x(12,k) -= 2*M_PI; }
		// 	while (bomt.xdz_x(12,k) < -M_PI){ bomt.xdz_x(12,k) += 2*M_PI; }
		// }				

		// Calculate Pz based on zdz calculated above
		bomt.FxX = quadrotor.eig_Fx_hard * bomt.xdz_x;
		bomt.FuU = quadrotor.eig_Fu_hard * bomt.udz_u;

	 	// Store new nu and z as determined by dnu, dz, and the s found in the feasibility search
		bomt.nu_proposed.noalias() = bomt.nu;
		bomt.x_proposed.noalias() = (*X);
		bomt.u_proposed.noalias() = (*U);

		//nu := nu + s*dnu
		bomt.nu.noalias() += s*bomt.dnu;

		//z := z + s*dz
		(*X).noalias() += s*bomt.dz_x;
		(*U).noalias() += s*bomt.dz_u;

		// :Zero(quadrotor.n,mpc_params.T)

		// for (int k = 0; k < mpc_params.T; k++)
		// {
		// 	while ((*X)(12,k) > M_PI){ (*X)(12,k).noalias() -= 2*M_PI; }
		// 	while ((*X)(12,k) < -M_PI){ (*X)(12,k).noalias() += 2*M_PI; }
		// }

		// insert backtracking line search
		// while(1) vs. for loop has negligible effect on outcome of the path but while can get stuck in an infinite loop and runs much slower
		for (int j = 0; j < 10; j++)
		{
			gfgphp(&quadrotor, &mpc_params, X, U, &bomt);
			dtildhat(&quadrotor, &mpc_params, X, U, &bomt);
			rd_tilde_rp(&quadrotor, &mpc_params, X, U, &bomt);
			newres = resdresp(bomt);

			if (isnan(newres) || isinf(newres))
			{
				cout << "residual is inf" << endl;
				cout << "BoMT.res_p: " << bomt.res_p << " BoMT.res_dx: " << bomt.res_dx << " BoMT.res_du: " << bomt.res_du << endl;
				bomt.res_p = 0.0;
				bomt.res_dx = 0.0;
				bomt.res_du = 0.0;
				bomt.rp = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
				bomt.rd_tilde_x = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
				bomt.Ctnu_x = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
				bomt.dz_x = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
				bomt.rd_tilde_u = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
				bomt.Ctnu_u = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
				bomt.dz_u = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
				// bomt.TwoQX = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
				// bomt.TwoRU = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
				bomt.dx = Eigen::MatrixXf::Zero(quadrotor.num_hard_x,mpc_params.T);
				bomt.du = Eigen::MatrixXf::Zero(quadrotor.num_hard_u,mpc_params.T);
				bomt.d_tilde_x = Eigen::MatrixXf::Zero(quadrotor.num_soft_x,mpc_params.T);
				bomt.d_hat_x = Eigen::MatrixXf::Zero(quadrotor.num_soft_x,mpc_params.T);
				bomt.d_tilde_u = Eigen::MatrixXf::Zero(quadrotor.num_soft_u,mpc_params.T);
				bomt.d_hat_u = Eigen::MatrixXf::Zero(quadrotor.num_soft_u,mpc_params.T);
				bomt.nu = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
				bomt.PtTdt_x = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
				bomt.xdz_x = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
				bomt.PtTdt_u = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
				bomt.udz_u = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
				bomt.newx = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
				bomt.newu = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
				return false;
			}

			// search for better residual. If one is found, break
			if (newres <= (1 - mpc_params.alpha * s)*(1 - mpc_params.alpha * s)*res) //First factor is squared because we're using a squared norm now
				break;

			switch(mpc_params.line_search_style)
			{
				case 0: 
					s *= mpc_params.beta;
					break;
				case 1: 
					// s = uniform_line_search();
					break;
				case 2: 
					s = dichotomous_line_search(this,X,U,&(bomt.dz_x),&(bomt.dz_u),lsp.dichoto_a1,lsp.dichoto_b1,lsp.dichoto_l,lsp.dichoto_epsilon, segment_number,s);
					break;
			}			

			//nu := nu + s*dnu
			bomt.nu.noalias() = bomt.nu_proposed + s*bomt.dnu;

			//z := z + s*dz
			(*X).noalias() = bomt.x_proposed + s*bomt.dz_x;
			(*U).noalias() = bomt.u_proposed + s*bomt.dz_u;

			// for (int k = 0; k < mpc_params.T; k++)
			// {
			// 	while ((*X)(12,k) > M_PI){ (*X)(12,k).noalias() -= 2*M_PI; }
			// 	while ((*X)(12,k) < -M_PI){ (*X)(12,k).noalias() += 2*M_PI; }
			// }

		}
		// //move to next step with new rho value
		mpc_params.mu_12 *= mpc_params.beta;
		mpc_params.mu_13 *= mpc_params.gamma;

	} // for(int i = 0; i < mpc_params.num_iterations; ++i)

	 end_time = std::chrono::high_resolution_clock::now();
	 timeElapsed = end_time - current_time;
	 log_f_mpc_runtime.writeToLog( to_string( timeElapsed.count() ), 1);
	 // std::cout << ">> fMPC solve time: " << timeElapsed.count() << " sec" << std::endl;

	 cout << "<FMPC> fmpcsolve complete" << endl;
	return true;

} // bool F_MPC_UNCUT::fmpcsolve(Eigen::MatrixXf* X, Eigen::MatrixXf* U, int segment_number)


