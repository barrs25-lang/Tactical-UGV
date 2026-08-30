// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 16th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file implements the quadrotor's feedback-linearization layer: it maps the flat/virtual
// control inputs produced by the MPC solver onto the physical roll, pitch, and thrust commands the
// vehicle actually needs, along with the supporting rotational-kinematics helpers (Gamma, Omega, Euler
// angle/rate computations). This is the piece that will need to be reworked to realize UGV dynamics
// while preserving the same interface contract with the MPC solver.
// End File Decscription ################################################################################

#include <f_mpc_uncut.h>
#include <f_mpc_globals.h>

// Member function of F_MPC_UNCUT object
// compute_Gamma: this function computes the Gamma matrix used in rotational kinematic eqn
// INPUTS: none
// OUTPUTS: none (this function modifies member variables)
void F_MPC_UNCUT::compute_Gamma()
{
	float phi = pose[12], theta = pose[13];

	Gamma(0,0) = 1.0; 
	Gamma(1,0) = 0.0; 
	Gamma(2,0) = 0.0; 

	Gamma(0,1) = sin(phi)*tan(theta); 
	Gamma(1,1) = cos(phi); 
	Gamma(2,1) = sin(phi)*(1/cos(theta)); 

	Gamma(0,2) = cos(phi)*tan(theta); 
	Gamma(1,2) = -sin(phi); 
	Gamma(2,2) = cos(phi)*(1/cos(theta)); 	

} // void F_MPC_UNCUT::compute_Gamma()


// Member function of F_MPC_UNCUT object
// compute_Gamma: this function computes the Gamma matrix used in rotational kinematic eqn
// INPUTS: float representing roll angle, float representing pitch angle, float representing heading angle
// pointer to a matrix representing Gamma
// OUTPUTS: none (this function modifies values pointed at by the last argument)
void F_MPC_UNCUT::compute_Gamma(float phi, float theta, float psi, Eigen::MatrixXf* Gamma_k)
{

	(*Gamma_k)(0,0) = 1.0; 
	(*Gamma_k)(1,0) = 0.0; 
	(*Gamma_k)(2,0) = 0.0; 

	(*Gamma_k)(0,1) = sin(phi)*tan(theta); 
	(*Gamma_k)(1,1) = cos(phi); 
	(*Gamma_k)(2,1) = sin(phi)*(1/cos(theta)); 

	(*Gamma_k)(0,2) = cos(phi)*tan(theta); 
	(*Gamma_k)(1,2) = -sin(phi); 
	(*Gamma_k)(2,2) = cos(phi)*(1/cos(theta)); 	

} // void F_MPC_UNCUT::compute_Gamma(float phi, float theta, float psi, Eigen::MatrixXf* Gamma_k)


// Member function of F_MPC_UNCUT object
// get_Omega: gets the vector storing the angular velocity
// INPUTS: none
// OUTPUTS: none
void F_MPC_UNCUT::get_Omega()
{
	
	omega(0,0) = pose[15];
	omega(1,0) = pose[16];
	omega(2,0) = pose[17];

} // void F_MPC_UNCUT::get_Omega()


// Member function of F_MPC_UNCUT object
// compute_dEuler: computes the rotational kinematic equation
// INPUTS: none
// OUTPUTS: none
void F_MPC_UNCUT::compute_dEuler()
{
	
	dEuler = Gamma*omega;

} // void F_MPC_UNCUT::compute_dEuler()


// Member function of F_MPC_UNCUT object
// hat_operator: computes a skew-symmetric matrix representing the
// hat operation of the input vectors
// INPUTS: pointer to a matrix storing the value to convert, pointer to a matrix
// which will store the result, integer representing the time step
// OUTPUTS: none
void hat_operator(Eigen::MatrixXf* omega, Eigen::MatrixXf* omega_cross, int i)
{

	(*omega_cross) = Eigen::MatrixXf::Zero(3,3);
	(*omega_cross)(0,1) = -(*omega)(2,i);
	(*omega_cross)(0,2) = (*omega)(1,i);
	(*omega_cross)(1,0) = (*omega)(2,i);
	(*omega_cross)(1,2) = -(*omega)(0,i);
	(*omega_cross)(2,0) = -(*omega)(1,i);
	(*omega_cross)(2,1) = (*omega)(0,i);

	// cout << "<FMPC> hat_operator complete" << endl;

} // void hat_operator(Eigen::MatrixXf* omega, Eigen::MatrixXf* omega_cross, int i)


// Member function of F_MPC_UNCUT object
// get_Ginv_k: computes G_inv
// INPUTS: floats representing the roll, pitch, and heading angles,
// float representing the total thrust, pointer to a matrix representing 
// Ginv_k.
// OUTPUTS: none
void F_MPC_UNCUT::get_Ginv_k(float phi, float theta, float psi, float u1, Eigen::MatrixXf* G_inv)
{

	float cp = cos(phi);
	float ct = cos(theta);
	float ci = cos(psi);
	float sp = sin(phi);
	float st = sin(theta);
	float si = sin(psi);

	float I1 = quadrotor.inertia_matrix(0,0);
	float I2 = quadrotor.inertia_matrix(1,1);
	float I3 = quadrotor.inertia_matrix(2,2);

	(*G_inv)(0,0) = quadrotor.mass*(sp*si + cp*ci*st);  
	(*G_inv)(0,1) = quadrotor.mass*(cp*si*st - ci*sp);
	(*G_inv)(0,2) = quadrotor.mass*(cp*ct);
	(*G_inv)(0,3) = 0;

	(*G_inv)(1,0) = (quadrotor.mass*I1*(cp*si-ci*sp*st)) / (u1);
	(*G_inv)(1,1) = -(quadrotor.mass*I1*(cp*ci+si*sp*st)) / (u1);
	(*G_inv)(1,2) = -(quadrotor.mass*I1*(ct*sp)) / (u1);
	(*G_inv)(1,3) = 0;

	(*G_inv)(2,0) = (quadrotor.mass*I2*(ci*ct)) / (u1);
	(*G_inv)(2,1) = (quadrotor.mass*I2*(ct*si)) / (u1);
	(*G_inv)(2,2) = -(quadrotor.mass*I2*(st)) / (u1);
	(*G_inv)(2,3) = 0;

	(*G_inv)(3,0) = -(quadrotor.mass*I3*(ci*ct*sp)) / (u1*cp);
	(*G_inv)(3,1) = -(quadrotor.mass*I3*(ct*si*sp)) / (u1*cp);
	(*G_inv)(3,2) = (quadrotor.mass*I3*(st*sp)) / (u1*cp);
	(*G_inv)(3,3) = (quadrotor.mass*I3*(ct)) / cp;

	// cout << "<FMPC> get_Ginv_k complete" << endl;

} // void F_MPC_UNCUT::get_Ginv_k(float phi, float theta, float psi, float u1, Eigen::MatrixXf* G_inv)


// Member function of F_MPC_UNCUT object
// get_f_k: computes f
// INPUTS: floats representing the roll, pitch, and heading angles,
// pointer to a matrix the angular velocity, float representing the total thrust
// pointer to a matrix representing a float, integer representing the time step
// OUTPUTS: none
void F_MPC_UNCUT::get_f_k(float phi, float theta, float psi, Eigen::MatrixXf* omega, float u1, float u1_dot, Eigen::MatrixXf* f, int i)
{

	float cp = cos(phi);
	float ct = cos(theta);
	float ci = cos(psi);
	float sp = sin(phi);
	float st = sin(theta);
	float si = sin(psi);

	float I1 = quadrotor.inertia_matrix(0,0);
	float I2 = quadrotor.inertia_matrix(1,1);
	float I3 = quadrotor.inertia_matrix(2,2);

	float mass = quadrotor.mass;

	float omega1 = (*omega)(0,i);
	float omega2 = (*omega)(1,i);
	float omega3 = (*omega)(2,i);

	(*f)(0,0) = u1_dot*(((cp*si - ci*sp*st)*(omega1*ct + omega3*cp*st + omega2*sp*st))/(mass*ct) + ((ci*sp - cp*si*st)*(omega3*cp + omega2*sp))/(mass*ct) + (cp*ci*ct*(omega2*cp - omega3*sp))/mass) + ((omega3*cp + omega2*sp)*((u1_dot*(ci*sp - cp*si*st))/mass - (u1*(sp*si + cp*ci*st)*(omega3*cp + omega2*sp))/(mass*ct) + (u1*(cp*ci + sp*si*st)*(omega1*ct + omega3*cp*st + omega2*sp*st))/(mass*ct) - (u1*cp*ct*si*(omega2*cp - omega3*sp))/mass))/ct - (ci*(omega2*cp - omega3*sp)*(omega2*u1*st - u1_dot*cp*ct + omega1*u1*ct*sp))/mass - ((omega1*ct + omega3*cp*st + omega2*sp*st)*(omega1*u1*sp*si - u1_dot*cp*si + u1_dot*ci*sp*st + omega1*u1*cp*ci*st))/(mass*ct) + (omega2*omega3*u1*(I2 - I3)*(cp*si - ci*sp*st))/(I1*mass) - (omega1*omega3*u1*ci*ct*(I1 - I3))/(I2*mass);
	(*f)(1,0) = u1_dot*(((sp*si + cp*ci*st)*(omega3*cp + omega2*sp))/(mass*ct) - ((cp*ci + sp*si*st)*(omega1*ct + omega3*cp*st + omega2*sp*st))/(mass*ct) + (cp*ct*si*(omega2*cp - omega3*sp))/mass) + ((omega3*cp + omega2*sp)*((u1_dot*(sp*si + cp*ci*st))/mass + (u1*(ci*sp - cp*si*st)*(omega3*cp + omega2*sp))/(mass*ct) + (u1*(cp*si - ci*sp*st)*(omega1*ct + omega3*cp*st + omega2*sp*st))/(mass*ct) + (u1*cp*ci*ct*(omega2*cp - omega3*sp))/mass))/ct - ((omega1*ct + omega3*cp*st + omega2*sp*st)*(u1_dot*cp*ci + u1_dot*sp*si*st - omega1*u1*ci*sp + omega1*u1*cp*si*st))/(mass*ct) - (si*(omega2*cp - omega3*sp)*(omega2*u1*st - u1_dot*cp*ct + omega1*u1*ct*sp))/mass - (omega2*omega3*u1*(I2 - I3)*(cp*ci + sp*si*st))/(I1*mass) - (omega1*omega3*u1*ct*si*(I1 - I3))/(I2*mass);
	(*f)(2,0) = (omega1*omega3*u1*st*(I1 - I3))/(I2*mass) - ((u1_dot*sp + omega1*u1*cp)*(omega1*ct + omega3*cp*st + omega2*sp*st))/mass - ((omega2*cp - omega3*sp)*(omega2*u1*ct + u1_dot*cp*st - omega1*u1*sp*st))/mass - (u1_dot*(omega2*st + omega1*ct*sp))/mass - (omega2*omega3*u1*ct*sp*(I2 - I3))/(I1*mass);
	(*f)(3,0) = ((omega2*cp - omega3*sp)*(omega1*ct + omega3*cp*st + omega2*sp*st))/(ct*ct) + (st*(omega3*cp + omega2*sp)*(omega2*cp - omega3*sp))/(ct*ct) + (omega1*omega2*cp*(I1 - I2))/(I3*ct) - (omega1*omega3*sp*(I1 - I3))/(I2*ct);

	// cout << "<FMPC> get_f_k complete" << endl;

} // void F_MPC_UNCUT::get_f_k(float phi, float theta, float psi, Eigen::MatrixXf* omega, float u1, float u1_dot, Eigen::MatrixXf* f, int i)


rot_dyn_state sys_state;

// Member function of F_MPC_UNCUT object
// sys: copies ODEs over, ready for integration by runge_kutta4.
// INPUTS: rot_dyn_state variable representing the result of integration,
// rot_dyn_state variable representing the integrand, double representing time
// OUTPUTS: none
void F_MPC_UNCUT::sys(const rot_dyn_state &y, rot_dyn_state &ydot, double t)
{
	
	// Iterate over the number of states to integrate
	for (unsigned short int i = 0; i < rot_dyn_states; i++)
	{	

		ydot[i] = sys_state[i];

	} // for (short int i = 0; i < rot_dyn_states; i++)

} // void F_MPC_UNCUT::sys(const rot_dyn_state &y, rot_dyn_state &ydot, double t)


// Member function of F_MPC_UNCUT object
// compute_roll_pitch_thrust_for_lambda_k_and_g_barrier: computes the roll, pitch, thrust, and g_barrier for a given control policy
// INPUTS: pointer to a variable storing the ofl state, pointer to a matrix storing the control policy
// OUTPUTS: boolean indicating whether or not any element of g_barrier is <= 0
bool F_MPC_UNCUT::compute_roll_pitch_thrust_for_lambda_k_and_g_barrier(Eigen::MatrixXf* chi, Eigen::MatrixXf* lambda)
{

	// Double array storing ode rhs, and time
	double ode_rhs[8], t = 0.0;

	// Define eigen matrices to store various values necessary for computing roll, pitch, thrust, and g_barrier
	Eigen::MatrixXf temp_rhs = Eigen::MatrixXf::Zero(3,1);
	Eigen::MatrixXf chi_k = Eigen::MatrixXf::Zero(14,mpc_params.T);
	Eigen::MatrixXf chi_dot_k = Eigen::MatrixXf::Zero(14,mpc_params.T);
	Eigen::MatrixXf Gamma_k = Eigen::MatrixXf::Zero(3,3);
	Eigen::MatrixXf omega_k = Eigen::MatrixXf::Zero(3,mpc_params.T);
	Eigen::MatrixXf omega_k_cross = Eigen::MatrixXf::Zero(3,3);
	Eigen::MatrixXf G_inv_k = Eigen::MatrixXf::Zero(4,4);
	Eigen::MatrixXf f_k = Eigen::MatrixXf::Zero(4,1);
	Eigen::MatrixXf virtual_control_k = Eigen::MatrixXf::Zero(4,mpc_params.T);
	// Eigen::MatrixXf zeta_k = Eigen::MatrixXf::Zero(4,mpc_params.T);
	// Eigen::MatrixXf u_k = Eigen::MatrixXf::Zero(4,mpc_params.T);
	Eigen::MatrixXf local_u1_k = Eigen::MatrixXf::Zero(mpc_params.T,1);
	Eigen::MatrixXf local_du1_k = Eigen::MatrixXf::Zero(mpc_params.T,1);
	Eigen::MatrixXf phi_k = Eigen::MatrixXf::Zero(mpc_params.T,1);
	Eigen::MatrixXf theta_k = Eigen::MatrixXf::Zero(mpc_params.T,1);
	Eigen::MatrixXf psi_k = Eigen::MatrixXf::Zero(mpc_params.T,1);

	// Declare a runge_kutta object
	runge_kutta4<rot_dyn_state> rk4;

	// Declare a rot_dyn_state data type to store output from integration
	rot_dyn_state y = {0.0,0.0,0.0,0.0,0.0,0.0,9.81,0.0};

	if (abs(y[6]) > 100)
	{
		y[6] = 9.81;
	}
	if (abs(y[7]) > 100)
	{
		y[7] = 0.0;
	}

	// Iterate over the number of time steps
	for (unsigned short int i = 0; i < mpc_params.T; i++)
	{

		// If this is the first step
		if (i == 0)
		{

			// Use information stored in the original state vector
			chi_k.block(0,0,12,1) = quadrotor.X0.block(0,0,12,1);
			phi_k(i,0) = pose[12];
			theta_k(i,0) = pose[13];
			psi_k(i,0) = pose[14];
			Gamma_k = Gamma;
			omega_k.col(i) = omega;
			local_u1_k(i,0) = quadrotor.u1_k(i,0);
			quadrotor.u1_k(i,0) = y[6];
			local_du1_k(i,0) = y[7];

			y[0] = phi_k(i,0);
			y[1] = theta_k(i,0);
			y[2] = psi_k(i,0);
			y[3] = omega_k(0,i);
			y[4] = omega_k(1,i);
			y[5] = omega_k(2,i);

		} // if (i == 0)
		else
		{

			// Otherwise use information stored in the integrator output
			chi_k.col(i) = (*chi).col(i);
			phi_k(i,0) = y[0];
			theta_k(i,0) = y[1];
			// psi_k(i,0) = chi_k(12,i);
			psi_k(i,0) = y[2];
			omega_k(0,i) = y[3]; // or y[2]?
			omega_k(1,i) = y[4];
			omega_k(2,i) = y[5];
			local_u1_k(i,0) = y[6];
			quadrotor.u1_k(i,0) = y[6];
			local_du1_k(i,0) = y[7];
			compute_Gamma(phi_k(i,0),theta_k(i,0),psi_k(i,0),&Gamma_k);

		} // if (i == 0)

		// Compute the lhs of the rotational kinematics
		temp_rhs = Gamma_k*omega_k.col(i);

		// Set the rotational kinematics
		for (unsigned short int j = 0; j < 3; j++)
		{
			ode_rhs[j] = temp_rhs(j,0);
		}

		// If this is the first step, set chi
		if (i == 0)
		{
			chi_k(12,0) = psi_k(i,0);
			chi_k(13,0) = temp_rhs(2,0);
		}

		// Call the hat operator to compute omega_cross
		hat_operator(&omega_k, &omega_k_cross, i);
		
		// Get the G_inv matrix
		get_Ginv_k(phi_k(i,0),theta_k(i,0),psi_k(i,0),quadrotor.u1_k(i,0),&G_inv_k);

		// Get the f vector
		get_f_k(phi_k(i,0),theta_k(i,0),psi_k(i,0),&omega_k,quadrotor.u1_k(i,0),quadrotor.u1_dot_k(i,0),&f_k,i);

		// Compute the RHS of the output feedback linearized dynamical system
		chi_dot_k.col(i) = quadrotor.eig_A*chi_k.col(i) + quadrotor.eig_B*(*lambda).col(i);

		// Get the virtual control, so it is easier to compute the actual control
		virtual_control_k(0,i) = chi_dot_k(9,i);
		virtual_control_k(1,i) = chi_dot_k(10,i);
		virtual_control_k(2,i) = chi_dot_k(11,i);
		virtual_control_k(3,i) = chi_dot_k(13,i);

		// Compute the actual control input
		zeta_k.col(i) = G_inv_k*(-f_k + virtual_control_k.col(i));

		// Compute the rhs of the rotational dynamic equations
		temp_rhs = quadrotor.inertia_matrix_inv*(zeta_k.block(1,i,3,1) - omega_k_cross*quadrotor.inertia_matrix*omega_k.col(i));
		
		for (unsigned short int j = 3; j < 6; j++)
		{
			ode_rhs[j] = temp_rhs(j-3,0);
		}

		// Two states of the integrand are reserved for the total thrust dynamics
		ode_rhs[6] = local_du1_k(i,0);
		ode_rhs[7] = zeta_k(0,i);
		
		for (unsigned short int j = 0; j < rot_dyn_states; j++)
		{
			sys_state[j] = ode_rhs[j];
		}

		// Compute the "time"
		t = i*mpc_params.delta_t;

		// Do a step
		rk4.do_step(sys, y, t, mpc_params.delta_t);

		// compute the control input
		// u_k(0,i) = quadrotor.u1_k(i,0);
		u_k(0,i) = y[6];
		u_k(1,i) = zeta_k(1,i);
		u_k(2,i) = zeta_k(2,i);
		u_k(3,i) = zeta_k(3,i);

		du1_k(0,i) = local_du1_k(i,0);
		ddu1_k(0,i) = (*lambda)(3,i);

		// Compute the total thrust
		quadrotor.T_k.col(i) = -quadrotor.M*u_k.col(i);
		quadrotor.T_k_min.col(i) = -quadrotor.M*u_k.col(i) - Eigen::MatrixXf::Ones(4,1)*quadrotor.T_min;
		quadrotor.T_k_max.col(i) = Eigen::MatrixXf::Ones(4,1)*quadrotor.T_max - quadrotor.M*u_k.col(i);

		g_barrier(i,0) = (quadrotor.phi_max*quadrotor.phi_max - phi_k(i,0)*phi_k(i,0)) * (quadrotor.theta_max*quadrotor.theta_max - theta_k(i,0)*theta_k(i,0) ) * quadrotor.T_k_min.col(i).prod()*quadrotor.T_k_max.col(i).prod(); 

		if (g_barrier(i,0) <= 0)
		{
			g_barrier = Eigen::MatrixXf::Ones(mpc_params.T,1);
			return true;
		}

	} // for (unsigned short int i = 0; i < mpc_params.T; i++)

	quadrotor.eig_phi = phi_k;	
	quadrotor.eig_theta = theta_k;
	lambda_k = virtual_control_k;

	// cout << "phi_k " << phi_k << endl;
	// cout << "theta_k " << theta_k << endl;
	// cout << "quadrotor.T_k: " << quadrotor.T_k << endl;

	cout << "<FMPC> compute_roll_pitch_thrust_for_lambda_k_and_g_barrier complete" << endl;

	return false;

} // bool F_MPC_UNCUT::compute_roll_pitch_thrust_for_lambda_k_and_g_barrier(Eigen::MatrixXf* chi, Eigen::MatrixXf* lambda)


// Member function of F_MPC_UNCUT object
// compute_u1_u1dot: computes the first and second integral of u1ddot using a discrete time 
// model.
// INPUTS: pointer to a variable storing the virtual control input
// OUTPUTS: none
void F_MPC_UNCUT::compute_u1_u1dot(Eigen::MatrixXf* eig_U)
{

	// Compute \delta_k, see equation 6 of Ch3. Marshall et. al. 
	quadrotor.delta_k(0,0) = quadrotor.u1_k(0,0);
	quadrotor.delta_k(1,0) = quadrotor.u1_dot_k(0,0);

	// Iterate over the number of time steps (minus 1)
	for (int i = 0; i < mpc_params.T; i++)
	{
		
		quadrotor.delta_k.block(0,i,2,1) = mpc_params.A_u1*quadrotor.delta_k.block(0,i,2,1) + mpc_params.B_u1*(*eig_U)(0,i);
		quadrotor.u1_k(i,0) = quadrotor.delta_k(0,i); 
		quadrotor.u1_dot_k(i,0) = quadrotor.delta_k(1,i); 

	} // for (int i = 0; i < mpc_params.T-1; i++)

	cout << "<FMPC> compute_u1_u1dot complete" << endl;

} // void F_MPC_UNCUT::compute_u1_u1dot(Eigen::MatrixXf* eig_U)


bool closeEnough(const float& a, const float& b, const float& epsilon = std::numeric_limits<float>::epsilon()) 
{
    return (epsilon > std::abs(a - b));
}


void F_MPC_UNCUT::compute_Euler_angles(Eigen::Matrix3f* R, Eigen::Vector3f* Eul) 
{

    //check for gimbal lock
    if (closeEnough((*R)(0,2), -1.0f)) 
    {

        (*Eul)(0) = 0; //gimbal lock, value of x doesn't matter
        (*Eul)(1) = M_PI / 2;
        (*Eul)(2) = (*Eul)(0)+atan2((*R)(1,0), (*R)(2,0));
        return;

    } 
    else if (closeEnough((*R)(0,2), 1.0f)) 
    {

        (*Eul)(0) = 0;
        (*Eul)(1) = -M_PI / 2;
        (*Eul)(2) = -(*Eul)(0)+atan2(-(*R)(1,0), -(*R)(1,0));
        return;

    } 
    else 
    { 

    	//two solutions exist
        float x1 = -asin((*R)(0,2));
        float x2 = M_PI - x1;

        float y1 = atan2((*R)(1,2) / cos(x1), (*R)(2,2) / cos(x1));
        float y2 = atan2((*R)(1,2) / cos(x2), (*R)(2,2) / cos(x2));

        float z1 = atan2((*R)(0,1) / cos(x1), (*R)(0,0) / cos(x1));
        float z2 = atan2((*R)(0,1) / cos(x2), (*R)(0,0) / cos(x2));

        //choose one solution to return
        //for example the "shortest" rotation
        if ((std::abs(x1) + std::abs(y1) + std::abs(z1)) <= (std::abs(x2) + std::abs(y2) + std::abs(z2))) 
        {
        	(*Eul)(0) = x1;
        	(*Eul)(1) = y1;
        	(*Eul)(2) = z1;
            return;
        } 
        else 
        {
        	(*Eul)(0) = x2;
        	(*Eul)(1) = y2;
        	(*Eul)(2) = z2;
            return;
        }

    }

    cout << "<FMPC> compute_Euler_angles complete" << endl;

} // void F_MPC_UNCUT::compute_Euler_angles(const Eigen::Matrix3f& R) 


