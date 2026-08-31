// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 16th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file implements the UGV's feedback-linearization layer: it maps the flat/virtual control
// inputs (lambda) produced by the MPC solver onto the physical longitudinal force F_x and front steering
// angle delta_f the planar bicycle-model vehicle actually needs, per the exact non-linear tire-slip
// derivation in ugv_feedback_linearization_state_space.md. F_x and delta_f are algebraic functions of
// the predicted state chi and virtual control lambda at each horizon step, so unlike the quadrotor's
// thrust chain, no auxiliary integration/rotational-kinematics machinery is required.
// End File Decscription ################################################################################

#include <f_mpc_uncut.h>
#include <f_mpc_globals.h>

// Member function of F_MPC_UNCUT object
// get_Ginv_pos_k: computes the inverse decoupling matrix G_pos^{-1}(psi) (Sec. 3 of
// ugv_feedback_linearization_state_space.md) that maps target inertial accelerations minus drift
// onto the physical control inputs [F_x, delta_f]^T.
// INPUTS: float representing the heading angle psi, pointer to a matrix representing G_inv (2x2)
// OUTPUTS: none (this function modifies values pointed at by the last argument)
void F_MPC_UNCUT::get_Ginv_pos_k(float psi, Eigen::MatrixXf* G_inv)
{

	float mass = quadrotor.mass;
	float Pcf = quadrotor.cornering_stiffness_front;

	float ci = cos(psi);
	float si = sin(psi);

	(*G_inv)(0,0) = mass*ci;
	(*G_inv)(0,1) = mass*si;

	(*G_inv)(1,0) = -(mass*si) / (2.0*Pcf);
	(*G_inv)(1,1) = (mass*ci) / (2.0*Pcf);

	// cout << "<FMPC> get_Ginv_pos_k complete" << endl;

} // void F_MPC_UNCUT::get_Ginv_pos_k(float psi, Eigen::MatrixXf* G_inv)


// Member function of F_MPC_UNCUT object
// get_f_pos_k: computes the non-linear drift vector f_pos(chi, psi, dpsi) (Sec. 3 of
// ugv_feedback_linearization_state_space.md), using the full arctan tire sideslip formulation with
// numerical regularization near zero longitudinal velocity.
// INPUTS: floats representing the heading angle psi, yaw rate dpsi, and body-frame velocities vx, vy,
// pointer to a matrix representing f (2x1)
// OUTPUTS: none (this function modifies values pointed at by the last argument)
void F_MPC_UNCUT::get_f_pos_k(float psi, float dpsi, float vx, float vy, Eigen::MatrixXf* f)
{

	float mass = quadrotor.mass;
	float a = quadrotor.dist_cg_to_front_axle;
	float b = quadrotor.dist_cg_to_rear_axle;
	float Pcf = quadrotor.cornering_stiffness_front;
	float Pcr = quadrotor.cornering_stiffness_rear;

	float vx_reg = max(vx, quadrotor.vx_regularization_eps);
	float theta_vf = atan((vy + a*dpsi) / vx_reg);
	float theta_vr = atan((vy - b*dpsi) / vx_reg);

	// Body-frame accelerations
	float ax_B = vy*dpsi;
	float ay_B = -(2.0*Pcf*theta_vf + 2.0*Pcr*theta_vr)/mass - vx*dpsi;

	float ci = cos(psi);
	float si = sin(psi);

	// Rotate the body-frame drift acceleration into the inertial frame: f_pos = R(psi) [ax_B; ay_B]
	(*f)(0,0) = ci*ax_B - si*ay_B;
	(*f)(1,0) = si*ax_B + ci*ay_B;

	// cout << "<FMPC> get_f_pos_k complete" << endl;

} // void F_MPC_UNCUT::get_f_pos_k(float psi, float dpsi, float vx, float vy, Eigen::MatrixXf* f)


// Member function of F_MPC_UNCUT object
// compute_force_steering_for_lambda_k_and_g_barrier: computes the physical control inputs
// [F_x, delta_f]^T and the g_barrier for a given virtual control policy lambda, using the exact
// non-linear feedback-linearized bicycle-model control law of Sec. 5 of
// ugv_feedback_linearization_state_space.md.
// INPUTS: pointer to a matrix storing the predicted state trajectory chi = [x, y, xdot, ydot]^T (4 x T),
// pointer to a matrix storing the virtual control policy lambda = [lambda_x, lambda_y]^T (2 x T)
// OUTPUTS: boolean indicating whether or not any element of g_barrier is <= 0
bool F_MPC_UNCUT::compute_force_steering_for_lambda_k_and_g_barrier(Eigen::MatrixXf* chi, Eigen::MatrixXf* lambda)
{

	// Define eigen matrices to store various values necessary for computing F_x, delta_f, and g_barrier
	Eigen::MatrixXf chi_dot_k = Eigen::MatrixXf::Zero(quadrotor.n, mpc_params.T);
	Eigen::MatrixXf G_inv_k = Eigen::MatrixXf::Zero(2,2);
	Eigen::MatrixXf f_pos_k = Eigen::MatrixXf::Zero(2,1);
	Eigen::MatrixXf accel_target_k = Eigen::MatrixXf::Zero(2,mpc_params.T);

	// Heading and yaw rate are treated as measured/exogenous quantities in G_pos(psi) and f_pos(.),
	// not as states of chi (see Sec. 2-3 of ugv_feedback_linearization_state_space.md)
	float psi = pose[14];
	float dpsi = pose[17];

	// Iterate over the number of time steps
	for (unsigned short int i = 0; i < mpc_params.T; i++)
	{

		// Target linear accelerations dictated by the controllable tilde-A/tilde-B state-space (Sec. 4)
		chi_dot_k.col(i) = quadrotor.eig_A*chi->col(i) + quadrotor.eig_B*(*lambda).col(i);
		accel_target_k(0,i) = chi_dot_k(2,i); // xddot_lin
		accel_target_k(1,i) = chi_dot_k(3,i); // yddot_lin

		// Body-frame velocity components (Sec. 2): [vx; vy] = R^{-1}(psi) [xdot; ydot]
		float xdot = (*chi)(2,i);
		float ydot = (*chi)(3,i);
		float vx =  xdot*cos(psi) + ydot*sin(psi);
		float vy = -xdot*sin(psi) + ydot*cos(psi);

		// Get the decoupling matrix inverse G_pos^{-1}(psi) and the drift vector f_pos(.)
		get_Ginv_pos_k(psi,&G_inv_k);
		get_f_pos_k(psi,dpsi,vx,vy,&f_pos_k);

		// Solve the feedback-linearized control law: [F_x; delta_f] = G_pos^{-1}(psi) (accel_target - f_pos)
		u_k.col(i) = G_inv_k*(accel_target_k.col(i) - f_pos_k);

		g_barrier(i,0) = (quadrotor.Fx_max*quadrotor.Fx_max - u_k(0,i)*u_k(0,i)) * (quadrotor.delta_f_max*quadrotor.delta_f_max - u_k(1,i)*u_k(1,i));

		if (g_barrier(i,0) <= 0)
		{
			g_barrier = Eigen::MatrixXf::Ones(mpc_params.T,1);
			return true;
		}

	} // for (unsigned short int i = 0; i < mpc_params.T; i++)

	lambda_k = accel_target_k;

	cout << "<FMPC> compute_force_steering_for_lambda_k_and_g_barrier complete" << endl;

	return false;

} // bool F_MPC_UNCUT::compute_force_steering_for_lambda_k_and_g_barrier(Eigen::MatrixXf* chi, Eigen::MatrixXf* lambda)
