// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 16th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file defines the MPC model used by F_MPC_UNCUT: it constructs the quadrotor's
// state-space model (A, B), constraint matrices, and MPC cost-function weights by reading
// Parameter_Files/System_params.txt, Parameter_Files/MPC_params.txt, and
// Parameter_Files/Line_Search_params.txt. This is the piece that most directly encodes the
// vehicle dynamics and will need to change to realize UGV dynamics.
// End File Decscription ################################################################################

#include <f_mpc_uncut.h>
#include <f_mpc_globals.h>

// Constructor
// Reads parameter files, sets up matrices for solver, sets up log files
F_MPC_UNCUT::F_MPC_UNCUT()
{

	// Open log files for messages, planner data, and runtimes
	log_f_mpc.openLogFile("TRAJECTORY_PLANNER");
	log_f_mpc_data.openLogFile("TRAJECTORY_PLANNER_DATA");
	log_f_mpc_policy_data.openLogFile("TRAJECTORY_PLANNER_POLICY_DATA");
	log_f_mpc_goal_data.openLogFile("TRAJECTORY_PLANNER_GOAL_DATA");
	log_f_mpc_runtime.openLogFile("TRAJECTORY_PLANNER_RUNTIME");

	// Additional weighting on the control penalty
	float _weight = 8.5;

	// Pointers to floats, where arrays can be defined
	float *A_in, *B_in, *Fu_hard_in, *fu_hard_in, *Fu_soft_in, *fu_soft_in, heading_soft_offset_in;

	// Integers storing integer parameters
	int n_in, m_in, num_fu_hard_in, num_fu_soft_in, noise_source_in;

	// Floats storing float parameters: maximum roll angle, maximum pitch angle, minimum thrust, maximum thrust
	float phi_max_in, theta_max_in, T_min_in, T_max_in;

	// Floats capturing various parameters
	float noise_w_in, z_hard_ceiling_in, z_soft_ceiling_in, z_hard_floor_in, z_soft_floor_in, centering_soft_constraints_in;
	float mu_20_in, mu_24_in, bc_epsilon_in, goal_tolerance_in;

	// Eigen::MatrixXf capturing the state-transition matrix and control effectiveness matrix
	Eigen::MatrixXf eig_A_temp, eig_B_temp;

	// String capturing the parameter file's name	
	string param_filename;

	// Set the parameter file's name
	param_filename = "Parameter_Files/System_params.txt";

	try
	{
		//open a parameter file to read in
		ifstream file(param_filename);
		
		// String to store file lines
		string file_line;
		stringstream ss;
		
		//read system physical dimensions
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> quadrotor.width;
		ss >> quadrotor.length;
		ss >> quadrotor.height;

		// read quadrotor mass
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> quadrotor.mass;

		// read quadrotor length of moment arm
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> quadrotor.moment_arm_l;

		// read quadrotor coefficient of drag (props)
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> quadrotor.cT;

		// Define the size of the inertia matrix and its inverse
		quadrotor.inertia_matrix = Eigen::MatrixXf::Zero(3,3);
		quadrotor.inertia_matrix_inv = Eigen::MatrixXf::Zero(3,3);

		// read the inertia matrix
		// Iterate over the number of rows
		for(unsigned short int i = 0; i < 3; ++i)
		{
			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);
			
			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				
				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')

			// Iterate over the number of columns
			for(unsigned short int j = 0; j < 3; ++j)
			{
				
				ss >> quadrotor.inertia_matrix(i,j);

			} // for(int j = 0; j < 3; ++j)

		} // for(int i = 0; i < 3; ++i)

		// Compute the inverse of the inertia matrix
		quadrotor.inertia_matrix_inv = quadrotor.inertia_matrix.inverse();

		// read the half-field-of-view angle (degrees)
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> quadrotor.half_camera_FOV;

		// convert the half-field-of-view angle (radians)
		quadrotor.half_camera_FOV = (quadrotor.half_camera_FOV*M_PI)/180;

		//read the number of states
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> n_in;

		//read the number of controls
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> m_in;

		// ************************************ //
		// read the A (state-transition) matrix //
		// ************************************ //

		// Define the size of the A matrix
		eig_A_temp = Eigen::MatrixXf::Zero(n_in,n_in);

		// Iterate over the number of states
		for(unsigned short int i = 0; i < n_in; ++i)
		{
			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);
			
			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				
				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')

			// Iterate over the number of states
			for(int j = 0; j < n_in; ++j)
			{
				
				ss >> eig_A_temp(i,j);

			} // for(int j = 0; j < n_in; ++j)

		} // for(int i = 0; i < n_in; ++i)

		// **************************************** //
		// read the B (control effectivness) matrix //
		// **************************************** //
		
		// Define the size of the B matrix
		eig_B_temp = Eigen::MatrixXf::Zero(n_in,m_in);

		// Iterate over the number of states
		for(unsigned short int i = 0; i < n_in; ++i)
		{
			
			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);
			
			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				
				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')

			// Iterate over the number of controls
			for(int j = 0; j < m_in; ++j)
			{
				
				ss >> eig_B_temp(i,j);

			} // for(int j = 0; j < m_in; ++j)

		} // for(int i = 0; i < n_in; ++i)

		//read noise element source
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> noise_source_in;

		//read noise element w_bar
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> noise_w_in;

		// Heading angle cone offset for heading soft constraints (in degrees): \mu_11 >= 0 (Equation 52)
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> mu_24_in; // heading_soft_offset_in
 	
 		// Convert to radians
 		mu_24_in = (mu_24_in*M_PI)/180;

		//read number of control constraints
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> num_fu_hard_in;

		// *************************************** //
		// read the hard control constraint matrix //
		// *************************************** //
		
		// Define a new float array
		Fu_hard_in = new float[num_fu_hard_in*m_in];

		// Iterate over the number of control constraints
		for(unsigned short int i = 0; i < num_fu_hard_in; ++i)
		{

			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);

			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				
				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')

			// Iterate over the number of controls
			for(int j = 0; j < m_in; ++j)
			{
				
				// Read the elements of the constraint matrix
				ss >> Fu_hard_in[i*m_in+j];

			} // for(int j = 0; j < m_in; ++j)

		} // for(int i = 0; i < num_fu_hard_in; ++i)

		// ******************************* //
		// read hard control bounds vector //
		// ******************************* //

		// Define a new float array
		fu_hard_in = new float[num_fu_hard_in];

		// Iterate over the number of control constraints
		for(unsigned short int i = 0; i < num_fu_hard_in; ++i)
		{
			
			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);
			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				
				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')
			
			// Read the rhs of the constraint
			ss >> fu_hard_in[i];

		} // for(int i = 0; i < num_fu_hard_in; ++i)


		// read number of soft control constraints
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> num_fu_soft_in;

		// **************************** //
		// read the soft control matrix //
		// **************************** //

		// Define a new float array 
		Fu_soft_in = new float[num_fu_soft_in*m_in];

		// Iterate over the number of soft control constraints
		for(unsigned short int i = 0; i < num_fu_soft_in; ++i)
		{
			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);
			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/'){
				--i;
				continue;
			}

			// Iterate over the number of controls
			for(int j = 0; j < m_in; ++j)
			{
				ss >> Fu_soft_in[i*m_in+j];
			}

		} // for(int i = 0; i < num_fu_soft_in; ++i)

		// ******************************* //
		// read soft control bounds vector //
		// ******************************* //
		
		// Define new float array
		fu_soft_in = new float[num_fu_soft_in];

		// Iterate over the number of soft control constraints
		for(unsigned short int i = 0; i < num_fu_soft_in; ++i)
		{
			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);
			
			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{

				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')

			// Read the RHS of the soft control constraint
			ss >> fu_soft_in[i];

		} // for(int i = 0; i < num_fu_soft_in; ++i)

		// Read the altitude constraint
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> z_hard_ceiling_in;

		// Read the altitude soft constraint
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> z_soft_ceiling_in;

		// Read the floor constraint
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> z_hard_floor_in;

		// Read the floor soft constraint
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> z_soft_floor_in;		

		// Read the roll angle constraint
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> phi_max_in;		

		// Read the pitch angle constraint
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> theta_max_in;

		// Read the pitch angle constraint
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> T_min_in;

		// Read the pitch angle constraint
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> T_max_in;				

		// Read the boundary condition tolerance constraint
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> bc_epsilon_in;				

		// Read the flag indicating whether or not to center soft constraints
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> centering_soft_constraints_in;

		// Read the goal tolerance
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> goal_tolerance_in;

		// Close the parameter file
		file.close();

	} // try
	catch (...)
	{

		cout << "Error during System_params.txt reading" << endl;
		time_t datetime = time(0);
		tm* now = std::localtime(&datetime);
		string time = "(" + to_string(now->tm_year + 1900) + "-" + to_string(now->tm_mon + 1) + "-" + to_string(now->tm_mday) + " " + to_string(now->tm_hour) + ":" + to_string(now->tm_min) + ":" + to_string(now->tm_sec) + "): ";
		ofstream error_file;
		error_file.open("MPC_error_log.txt");
		error_file << '\n' << time << "An error has occurred while trying to parse " + param_filename;
		cin.get();
		throw;

	} // catch (...)

	// Save parameters to System (quadrotor) object
	quadrotor.n = n_in;
	quadrotor.m = m_in;
	quadrotor.num_hard_u = num_fu_hard_in;
	quadrotor.num_soft_u = num_fu_soft_in;
	quadrotor.eig_z_hard_bounds(0) = z_hard_ceiling_in;
	quadrotor.eig_z_hard_bounds(1) = -z_hard_floor_in;
	quadrotor.eig_z_soft_bounds(0) = z_soft_ceiling_in;
	quadrotor.eig_z_soft_bounds(1) = -z_soft_floor_in;
	quadrotor.phi_max = phi_max_in;
	quadrotor.theta_max = theta_max_in;
	quadrotor.T_min = T_min_in;
	quadrotor.T_max = T_max_in;

	// Define float arrays for storing input data
	float *tilde_R_r_in = new float[n_in*n_in], *R_r_f_in = new float[n_in*n_in], *R_lambda_in = new float[m_in*m_in], *tilde_R_r_lambda_in = new float[n_in*m_in]; 
	float *tilde_q_r_in = new float[n_in-2], *tilde_q_lambda_in = new float[m_in], *q_psi_in = new float[1];

	// Declare floats storing scalar input data
	float delta_T_in, tol_in, alpha_in, beta_in, kappa_in, rho_in, gamma_in, rObs_saturation_in;
	float mu_10_in, mu_11_in, mu_12_in, mu_13_in;
	float collisionAvoidanceSoftConstraintOffset_in;
	float mu_14_in, mu_15_in, mu_16_in, mu_17_in, mu_18_in, mu_19_in, mu_21_in, mu_22_in, mu_23_in;

	// Declare ints for storing scalar input data
	int T_in, num_iterations_in, discount_form_in, nu_X_in;

	// string count_string = to_string(mpc_params.exp_count);
	//param_filename = "Parameter_Files/MPC_params"+ count_string + ".txt";

	// Set the parameter file's name
	param_filename = "Parameter_Files/MPC_params.txt";

	// try opening and reading the file
	try{

		//open a parameter file to read in
		ifstream file(param_filename);

		// Declare strings to store lines of the file
		string file_line;
		stringstream ss;
		
		//read the tilde_R_r matrix
		for(unsigned short int i = 0; i < n_in; ++i)
		{

			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);
			
			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				
				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')

			//parse the line for n-many floats for the Q matrix
			for(unsigned short int j = 0; j < n_in; ++j)
			{

				// Read in the \tilde_R_r matrix
				ss >> tilde_R_r_in[i*n_in+j];

			} // for(int j = 0; j < n_in; ++j)

		} // for(int i = 0; i < n_in; ++i){	

		//read the R_r,f matrix
		for(unsigned short int i = 0; i < n_in; ++i)
		{
			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);
			
			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				
				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')
			
			//parse the line for n-many floats for the Q matrix
			for(unsigned short int j = 0; j < n_in; ++j)
			{
				
				// Read in the \_R_r_f matrix
				ss >> R_r_f_in[i*n_in+j];

			} // for(int j = 0; j < n_in; ++j)

		} // for(int i = 0; i < n_in; ++i)

		//read the R_lambda matrix
		for(unsigned short int i = 0; i < m_in; ++i)
		{

			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);

			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				--i;
				continue;
			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')

			//parse the line for n-many floats for the Q matrix
			for(int j = 0; j < m_in; ++j)
			{
				
				ss >> R_lambda_in[i*m_in+j];

			} // for(int j = 0; j < m_in; ++j)

		} // for(int i = 0; i < m_in; ++i)

		//read the tilde_R_{r,\lambda} matrix
		for(unsigned short int i = 0; i < n_in; ++i)
		{

			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);

			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				
				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')

			//parse the line for n-many floats for the Q matrix
			for(unsigned short int j = 0; j < m_in; ++j)
			{
				
				ss >> tilde_R_r_lambda_in[i*m_in+j];

			} // for(int j = 0; j < m_in; ++j)

		} // for(int i = 0; i < n_in; ++i)

		//read the tilde_q_r
		for(unsigned short int i = 0; i < n_in-2; ++i)
		{

			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);

			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				
				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')

			ss >> tilde_q_r_in[i];

		} // for(int i = 0; i < n_in-2; ++i)

		// Read the weighting on heading angle in linear part of cost function
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> q_psi_in[0];		

		//read the tilde_q_lambda
		for(unsigned short int i = 0; i < m_in; ++i)
		{
			//grab the next line of the file
			ss.clear(); getline(file, file_line); ss.str(file_line);
			//ignore comment lines
			if(file_line.at(0) == '/' && file_line.at(1) == '/')
			{
				
				--i;
				continue;

			} // if(file_line.at(0) == '/' && file_line.at(1) == '/')

			ss >> tilde_q_lambda_in[i];

		} // for(int i = 0; i < m_in; ++i)

		//Read T, number of time steps
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> T_in;

		//Read delta_T, length of time step
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> delta_T_in;

		//Read rObs_saturation, distance threshold for obstacle influence
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> rObs_saturation_in;

		//Read num_interations, number of MPC iterations
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> num_iterations_in;

		//Read tol, residual tolerance
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> tol_in;

		// Read alpha, scaling parameter for MPC algorithm 
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> alpha_in;

		// Read beta, scaling parameter for \kappa (lagrange multiplier)
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> beta_in;

		// Read line_search_style, dictates which line search algorithm is used.
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> mpc_params.line_search_style;

		// read \mu_12, \kappa (lagrange multiplier for log barrier approximation of hard constraints)
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> mu_12_in;

		// read \mu_13, \rho (KS-approximation of soft constraints)
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> mu_13_in;

		// read \gamma, scaling parameter for \rho
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> gamma_in;

		// read \mu_10,\mu_11; \mu_10: weighing on goal or shelter priority in cost function; \mu_11: distance threshold for obstacle influence
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> mu_10_in;
		ss >> mu_11_in;

		// read \mu_14
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> mu_14_in;

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');		
		ss >> mu_15_in;

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');		
		ss >> mu_16_in;

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');		
		ss >> mu_17_in;

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');		
		ss >> mu_18_in;

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');		
		ss >> mu_19_in;

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');		
		ss >> discount_form_in;

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');		
		ss >> mu_21_in;

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');		
		ss >> mu_22_in;

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');		
		ss >> mu_23_in;	

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');		
		ss >> nu_X_in; // Path stride	

		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> collisionAvoidanceSoftConstraintOffset_in;	

		//close the parameter file
		file.close();
	}
	catch (...){
		time_t datetime = time(0);
		tm* now = std::localtime(&datetime);
		string time = "(" + to_string(now->tm_year + 1900) + "-" + to_string(now->tm_mon + 1) + "-" + to_string(now->tm_mday) + " " + to_string(now->tm_hour) + ":" + to_string(now->tm_min) + ":" + to_string(now->tm_sec) + "): ";
		ofstream error_file;
		error_file.open("MPC_error_log.txt");
		error_file << '\n' << time << "An error has occurred while trying to parse " + param_filename;
		throw;
	}


	// // Use Cayley-Hamilton Theorem to compute the matrix exponential
	quadrotor.eig_A = Eigen::MatrixXf::Zero(n_in,n_in);
	// quadrotor.eig_A_complex = Eigen::MatrixXcf::Zero(n_in,n_in);
	// Eigen::MatrixXf eig_A_temp_power;
	// // Step 1a: Find eigenvalues of the matrix
	// Eigen::VectorXcf eig_A_eigenvalues = eig_A_temp.eigenvalues();
	// Eigen::VectorXf eig_A_eigenvalues_real = Eigen::VectorXf::Zero(n_in);
	// Eigen::VectorXf eig_A_eigenvalues_imag = Eigen::VectorXf::Zero(n_in);
	// Eigen::VectorXi eig_A_eigenvalues_AM = Eigen::VectorXi::Ones(n_in);

	// Eigen::VectorXf temp_norm = Eigen::VectorXf::Zero(1); 

	// eig_A_eigenvalues_real = eig_A_eigenvalues.real();
	// eig_A_eigenvalues_imag = eig_A_eigenvalues.imag();

	// std::complex<float> c = 1i;

	// // Step 1b: Determine algebraic multiplicity
	// for (short int i = 0; i < n_in; i++)
	// {
	// 	for (short int j = 0; j < n_in; j++)
	// 	{
	// 		if (i != j)
	// 		{
	// 			temp_norm(0) = (eig_A_eigenvalues_real(j)-eig_A_eigenvalues_real(i))*(eig_A_eigenvalues_real(j)-eig_A_eigenvalues_real(i)) + (eig_A_eigenvalues_imag(j)-eig_A_eigenvalues_imag(i))*(eig_A_eigenvalues_imag(j)-eig_A_eigenvalues_imag(i));
	// 			if ( sqrt( temp_norm(0) ) < 1e-10)
	// 			{
	// 				eig_A_eigenvalues_AM(j)++;
	// 			}
	// 		}
	// 	}
	// }

	// // cout << "eig_A_eigenvalues:" << endl << eig_A_eigenvalues.transpose() << endl;
	// // cout << "Algebraic multiplicity: " << endl << eig_A_eigenvalues_AM.transpose() << endl;
	// // Step 2: Setup linear system to solve for coefficients of charateristic eqn
	// 		   // exp(\lambda_i*t) = sum_{k=0}^{n-1} \alpha_k * \lambda_i^k
	// 		   // where n is the dimension of A, t is a parameter (time for example) 
	// Eigen::VectorXcf lhs_coefficient_eqn = Eigen::VectorXcf::Zero(n_in);
	// for (short int i = 0; i < n_in; i++)
	// {
	// 	lhs_coefficient_eqn(i) = exp(eig_A_eigenvalues(i)*delta_T_in);
	// }

	// Eigen::MatrixXcf rhs_coefficient_eqn = Eigen::MatrixXcf::Zero(n_in,n_in);
	// Eigen::VectorXcf eig_A_product = Eigen::VectorXcf(1);

	// for (short int i = 0; i < n_in; i++)
	// {
	// 	eig_A_product(0) = 1.0;
	// 	for (short int j = 0; j < n_in; j++)
	// 	{
	// 		if (j > 0)
	// 		{
	// 			// if (eig_A_eigenvalues_imag(i) != 0)
	// 			// {
	// 			// 	eig_A_product(0) *= exp(eig_A_eigenvalues_real(i)*delta_T_in)*(cos(eig_A_eigenvalues_imag(i)*delta_T_in) + c*sin(eig_A_eigenvalues_imag(i)*delta_T_in));
	// 			// }
	// 			// else
	// 			// {
	// 				eig_A_product(0) *= eig_A_eigenvalues(i);
	// 			// }
	// 		}
			
	// 		rhs_coefficient_eqn(i,j) = eig_A_product(0);
	// 		// cout << "rhs_coefficient_eqn(" << i << "," << j <<"): " << endl << rhs_coefficient_eqn(i,j) << endl;
	// 	}
	// }


	// Step 3: Solve for the coefficients Lambda\alpha = exp(Lambda t)
	// Eigen::VectorXcf alpha_coefficients = rhs_coefficient_eqn.colPivHouseholderQr().solve(lhs_coefficient_eqn); 
	// assert(lhs_coefficient_eqn.isApprox(rhs_coefficient_eqn*alpha_coefficients));

	// Step 4: Compute the matrix exponential exp(A*t) = sum_{k=0}^{n-1} \alpha_k * A^k
	// eig_A_temp_power = Eigen::MatrixXf::Identity(n_in,n_in);
	// for (short int i = 0; i < n_in; i++)
	// {
	// 	if (i > 0)
	// 	{
	// 		eig_A_temp_power*=eig_A_temp;
	// 	}
	// 	// quadrotor.eig_A += (alpha_coefficients(i)*eig_A_temp_power);
	// 	quadrotor.eig_A_complex += (alpha_coefficients(i)*eig_A_temp_power);
	// }

	// quadrotor.eig_A = quadrotor.eig_A_complex.real();

	// cout << "eig_A_complex: " << endl << quadrotor.eig_A_complex << endl;
	// Eigen::VectorXcf discrete_eig_A_eigenvalues = quadrotor.eig_A.eigenvalues();

	// cout << "discrete_eig_A_eigenvalues: " << endl << discrete_eig_A_eigenvalues.transpose() << endl;

	quadrotor.eig_B = Eigen::MatrixXf::Zero(n_in,m_in);
	// quadrotor.eig_B = eig_A_temp.inverse()*(quadrotor.eig_A - Eigen::MatrixXf::Identity(n_in,n_in))*eig_B_temp / mpc_params.delta_t;

	quadrotor.eig_A = eig_A_temp;
	quadrotor.eig_B = eig_B_temp;

	// ****************** //
	// Set mpc parameters //
	// ****************** //

	mpc_params.T = T_in;
	mpc_params.num_iterations = num_iterations_in;
	mpc_params.tol_sq = tol_in*tol_in;
	mpc_params.alpha = alpha_in;
	mpc_params.beta = beta_in;
	mpc_params.mu_12 = mu_12_in;
	mpc_params.mu_12_initial = mu_12_in;
	mpc_params.mu_13 = mu_13_in;
	mpc_params.mu_13_initial = mu_13_in;
	mpc_params.gamma = gamma_in;
	mpc_params.delta_t = delta_T_in;
	mpc_params.rObs_saturation = rObs_saturation_in;
	mpc_params.mu_10 = mu_10_in;
	mpc_params.mu_11 = mu_11_in; 
	mpc_params.mu_14 = mu_14_in;
	mpc_params.mu_15 = mu_15_in;
	mpc_params.mu_16 = mu_16_in;
	mpc_params.mu_17 = mu_17_in;
	mpc_params.mu_18 = mu_18_in;
	mpc_params.mu_19 = mu_19_in;
	mpc_params.mu_20 = mu_20_in;
	mpc_params.discount_form = discount_form_in;
	mpc_params.mu_21 = mu_21_in;
	mpc_params.mu_22 = mu_22_in;
	mpc_params.mu_23 = mu_23_in;
	mpc_params.nu_X = nu_X_in;
	mpc_params.collisionAvoidanceSoftConstraintOffset = collisionAvoidanceSoftConstraintOffset_in;
	mpc_params.mu_24 = mu_24_in;
	mpc_params.bc_epsilon = bc_epsilon_in;
	mpc_params.centering_soft_constraints = centering_soft_constraints_in;
	mpc_params.goal_tolerance = goal_tolerance_in;

	quadrotor.noise_source = noise_source_in;
	quadrotor.noise_w = Eigen::MatrixXf::Ones(n_in, T_in);

	// Iterate over the number of states
	for (unsigned short int i = 0; i < n_in; i++)
	{
		
		// Iterate over the number of time steps
		for (unsigned short int j = 0; j < T_in; j++)
		{
			
			// Compute noise density
			quadrotor.noise_w(i,j) *= noise_w_in;	

		} // for (int j = 0; j < T_in; j++)

	} // for (int i = 0; i < n_in; i++)

	// Declare Eigen::MatrixXf to store Fpsi matrix (orientation of constraint on heading)
	Eigen::MatrixXf dummy_Fpsi;
	dummy_Fpsi = Eigen::MatrixXf::Zero(2,14);

	// Set the constraints to match Equation 33a and 33b of Marshall et. al. 2022
	dummy_Fpsi(0,12) = 1;
	dummy_Fpsi(1,12) = -1;

	// Declare Eigen::MatrixXf to store Fz matrix (orientation of constraint on altitude)
	Eigen::MatrixXf dummy_Fz_ceiling;
	dummy_Fz_ceiling = Eigen::MatrixXf::Zero(2,14);

	// Set the constraints to capture a floor and a ceiling
	dummy_Fz_ceiling(0,2) = 1;
	dummy_Fz_ceiling(1,2) = -1;

	// ******************************************************************************** //
	// The constraints below are used to capture the boundary conditions in Equation 12 // 
	// ******************************************************************************** //

	// Set the size of eig_Fx_hard_boundary_conditions 
	quadrotor.eig_Fx_hard_boundary_conditions = Eigen::MatrixXf::Zero(2*n_in,n_in);

	// Fill the matrix with 1s on diagonal 
	quadrotor.eig_Fx_hard_boundary_conditions.block(0,0,n_in,n_in) = Eigen::MatrixXf::Identity(n_in,n_in);  

	// Fill the matrix with -1s on diagonal 
	quadrotor.eig_Fx_hard_boundary_conditions.block(n_in,0,n_in,n_in) = -1*Eigen::MatrixXf::Identity(n_in,n_in);  
	
	// Set the size of eig_x_hard_boundary_conditions 
	quadrotor.eig_x_hard_boundary_conditions = Eigen::MatrixXf::Zero(2*n_in,mpc_params.T);

	// Set the size of eig_Fx_soft_boundary_conditions 
	quadrotor.eig_Fx_soft_boundary_conditions = Eigen::MatrixXf::Zero(2*n_in,n_in);

	// Fill the matrix with 1s on diagonal 
	quadrotor.eig_Fx_soft_boundary_conditions.block(0,0,n_in,n_in) = Eigen::MatrixXf::Identity(n_in,n_in);  

	// Fill the matrix with -1s on diagonal 
	quadrotor.eig_Fx_soft_boundary_conditions.block(n_in,0,n_in,n_in) = -1*Eigen::MatrixXf::Identity(n_in,n_in);  

	// Set the size of eig_x_hard_boundary_conditions 
	quadrotor.eig_x_soft_boundary_conditions = Eigen::MatrixXf::Zero(2*n_in,mpc_params.T);

	// Set the hard constraints on heading
	quadrotor.eig_Fpsi_hard = dummy_Fpsi;

	// Set the hard constraints on altitude
	quadrotor.eig_Fz_hard_ceiling = dummy_Fz_ceiling;

	// Set the hard constraints on the control input
	quadrotor.eig_Fu_hard = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (Fu_hard_in, quadrotor.num_hard_u, quadrotor.m);

	// Set the hard constraints RHS value
	quadrotor.eig_u_hard_bounds = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>> (fu_hard_in, quadrotor.num_hard_u);

	// Set the hard constraints on heading
	quadrotor.eig_Fpsi_soft = quadrotor.eig_Fpsi_hard;
	
	// Set the hard constraints on altitude
	quadrotor.eig_Fz_soft_ceiling = quadrotor.eig_Fz_hard_ceiling;
	
	// Set the hard constraints on the control input
	quadrotor.eig_Fu_soft = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (Fu_soft_in, quadrotor.num_soft_u, quadrotor.m);
	
	// Set the soft constraints RHS value
	quadrotor.eig_u_soft_bounds = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>> (fu_soft_in, quadrotor.num_soft_u);
	
	// Iterate over the size of the state^2
	for (unsigned short int i = 0; i < n_in*n_in; i++)
	{
		
		tilde_R_r_in[i] *= _weight;
		R_r_f_in[i] *= _weight;

	} // for (int i = 0; i < n_in*n_in; i++)

	// Set the R_r_tilde matrix, quadratic weighting on position error (set by user)
	mpc_params.eig_R_r_tilde = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (tilde_R_r_in, quadrotor.n, quadrotor.n);

	// Set the R_r_f matrix, quadratic weighting on final position error
	mpc_params.eig_R_rf = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(R_r_f_in, quadrotor.n, quadrotor.n);

	// Set the R_r_lambda matrix, quadratic weighting on coupling 
	mpc_params.eig_R_r_lambda_tilde =  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (tilde_R_r_lambda_in, quadrotor.n, quadrotor.m);
	
	// Set the q_r_\tilde vector, linear weighting on position error
	mpc_params.eig_q_r_tilde = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (tilde_q_r_in, quadrotor.n-2, 1);

	// Set the q_psi parameter, linear weighting on heading error
	mpc_params.q_psi = (*q_psi_in);

	// Set the R_lambda_tilde matrix, quadratic weighting on control input (set by user)
	mpc_params.eig_R_lambda_tilde = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (R_lambda_in, quadrotor.m, quadrotor.m);

	// Set the R_lambda matrix, quadratic weighting on control input (varies with time)
	mpc_params.eig_R_lambda = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (R_lambda_in, quadrotor.m, quadrotor.m);
	mpc_params.eig_R_lambda_init = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (R_lambda_in, quadrotor.m, quadrotor.m);
	
	// Set the q_lambda vector, linear weighting on control input
	mpc_params.eig_q_lambda_tilde = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (tilde_q_lambda_in, quadrotor.m, 1);

	// Define matrix to store the RHS of the Schur complement (condition for positive-definiteness of block matrix tilde{eig_R})
	Eigen::MatrixXf RHS_LMI = Eigen::MatrixXf::Zero(n_in,n_in);

	// Compute the condition
	RHS_LMI = mpc_params.eig_R_r_tilde - 2*mpc_params.eig_R_r_lambda_tilde*mpc_params.eig_R_lambda_tilde.inverse()*mpc_params.eig_R_r_lambda_tilde.transpose();

	// Compute the eigenvalues
	Eigen::VectorXcf discrete_LMI_eigenvalues = RHS_LMI.eigenvalues();

	// Iterate over the number of states
	for (unsigned short int i = 0; i < n_in; i++)
	{
		
		if (discrete_LMI_eigenvalues(i).real() <= 0)
		{
			
			cout << "Schur complement condition on the positive-definiteness of block matrix tilde{eig_R} not satisfied!" << endl;
			exit(0);

		} // if (discrete_LMI_eigenvalues(i).real() <= 0)

	} // for (short int i = 0; i < n_in; i++)

	// Set a parameter file's name
	param_filename = "Parameter_Files/Line_Search_params.txt";

	try
	{
		//open a parameter file to read in
		ifstream file(param_filename);

		// Declare string to store lines of a file
		string file_line;
		stringstream ss;

		// Read the initial left boundary of the interval of uncertainty (for dichotomous line search)
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> lsp.dichoto_a1;

		// Read the initial right boundary of the interval of uncertainty (for dichotomous line search)
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> lsp.dichoto_b1;

		// Read the minimum length of the interval of uncertainty (for dichotomous line search)
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> lsp.dichoto_l;

		// Read the distuishability constant (for dichotomous line search)
		do{ss.clear(); getline(file, file_line); ss.str(file_line);}
		while(file_line.at(0) == '/' && file_line.at(1) == '/');
		ss >> lsp.dichoto_epsilon;			

		//close the parameter file
		file.close();
	}
	catch (...){
		time_t datetime = time(0);
		tm* now = std::localtime(&datetime);
		string time = "(" + to_string(now->tm_year + 1900) + "-" + to_string(now->tm_mon + 1) + "-" + to_string(now->tm_mday) + " " + to_string(now->tm_hour) + ":" + to_string(now->tm_min) + ":" + to_string(now->tm_sec) + "): ";
		ofstream error_file;
		error_file.open("MPC_error_log.txt");
		error_file << '\n' << time << "An error has occurred while trying to parse " + param_filename;
		throw;
	}

	// Set the size of \Gamma, stores the 
	Gamma = Eigen::MatrixXf::Zero(3,3);

	// Set the size of \omega, stores the angular velocity of the body reference frame
	omega = Eigen::MatrixXf::Zero(3,1);

	// Set the size of dEuler, stores the time derivative of the Euler angles
	dEuler = Eigen::MatrixXf::Zero(3,1);

	// Set the size of the quadrotor's state vector (its 6dof state not the ofl state)
	quadrotor.X0 = Eigen::VectorXf::Zero(n_in);

	// Set the size of the matrix storing the previous planned path
	prev_plannedPath = Eigen::MatrixXf::Zero(69,3);

	// Set the size of the vector storing the coordinates of the closest obstacle
	r_obs = Eigen::MatrixXf::Zero(3,1);

	// Set the size of the state-transition matrix for the 
	// higher order dynamics of u1
	mpc_params.A_u1 = Eigen::MatrixXf::Zero(2,2);

	// Set the elements (this is the discrete-time matrix)
	mpc_params.A_u1(0,0) = 1.0;
	mpc_params.A_u1(0,1) = 1.0;
	mpc_params.A_u1(1,1) = 1.0;

	// Set the size of the control effectiveness matrix for the
	// higher dynamics of u1
	mpc_params.B_u1 = Eigen::MatrixXf::Zero(2,1);

	// Set the elements (this is the discrete-time matrix)
	mpc_params.B_u1(0,0) = 0.5*mpc_params.delta_t*mpc_params.delta_t;
	mpc_params.B_u1(1,0) = mpc_params.delta_t;

	// Set the size of \delta_k, the vector storing u1 and u1dot
	quadrotor.delta_k = Eigen::MatrixXf::Zero(2,mpc_params.T);

	// Set the size of u1_k storing the value of u1 over the time horizon T
	quadrotor.u1_k = Eigen::MatrixXf::Zero(mpc_params.T,1);

	// Set the size of u1_dot_k storing the value of u1_dot over the time horizon T
	quadrotor.u1_dot_k = Eigen::MatrixXf::Zero(mpc_params.T,1);

	// Set the size of T_k storing the value of the thrust over the time horizon T
	quadrotor.T_k = Eigen::MatrixXf::Zero(4,mpc_params.T);
	quadrotor.T_k_min = Eigen::MatrixXf::Zero(4,mpc_params.T);
	quadrotor.T_k_max = Eigen::MatrixXf::Zero(4,mpc_params.T);

	// Iterate over the number of time steps
	for (unsigned short int i = 0; i < mpc_params.T; i++)
	{
		
		quadrotor.u1_k(i,0) = -quadrotor.mass*9.81;
		
		// Iterate over the number of props
		for (unsigned short int j = 0; j < 4; j++)
		{
			
			quadrotor.T_k(j,i) = quadrotor.mass*9.81*0.25;

		} // for (int j = 0; j < 4; j++)

	} // for (int i = 0; i < mpc_params.T; i++)

	// Set the size of M, the mixer
	quadrotor.M = Eigen::MatrixXf::Zero(4,4);

	// Set mixer coefficients
	quadrotor.M(0,0) = 0.25;
	quadrotor.M(1,0) = 0.25;
	quadrotor.M(2,0) = 0.25;
	quadrotor.M(3,0) = 0.25;

	// quadrotor.M(0,1) = 0;
	// quadrotor.M(1,1) = -2*0.25/(quadrotor.moment_arm_l);
	// quadrotor.M(2,1) = 0;
	// quadrotor.M(3,1) = 2*0.25/(quadrotor.moment_arm_l);

	// quadrotor.M(0,2) = 2*0.25/(quadrotor.moment_arm_l);
	// quadrotor.M(1,2) = 0;
	// quadrotor.M(2,2) = -2*0.25/(quadrotor.moment_arm_l);
	// quadrotor.M(3,2) = 0;

	quadrotor.M(0,1) = -1 / ( quadrotor.moment_arm_l * 4 );
	quadrotor.M(1,1) = 1 / ( quadrotor.moment_arm_l * 4 );
	quadrotor.M(2,1) = 1 / ( quadrotor.moment_arm_l * 4 );
	quadrotor.M(3,1) = -1 / ( quadrotor.moment_arm_l * 4 );

	quadrotor.M(0,2) = 1 / ( quadrotor.moment_arm_l * 4 );
	quadrotor.M(1,2) = -1 / ( quadrotor.moment_arm_l * 4 );
	quadrotor.M(2,2) = 1 / ( quadrotor.moment_arm_l * 4 );
	quadrotor.M(3,2) = -1 / ( quadrotor.moment_arm_l * 4 );	

	quadrotor.M(0,3) = -1 / ( 4 * quadrotor.cT );
	quadrotor.M(1,3) = -1 / ( 4 * quadrotor.cT );
	quadrotor.M(2,3) = 1 / ( 4 * quadrotor.cT );
	quadrotor.M(3,3) = 1 / ( 4 * quadrotor.cT );

	// Set the size of g_barrier, capturing constraints on phi,theta,thrust
	g_barrier = Eigen::MatrixXf::Zero(mpc_params.T,1);

	// Set pointers to new Eigen::MatrixXfs, one for each time step
	mpc_params.eig_R_rk = new Eigen::MatrixXf[mpc_params.T];
	mpc_params.eig_R_r_lambda_k = new Eigen::MatrixXf[mpc_params.T];
	mpc_params.eig_q = new Eigen::MatrixXf[mpc_params.T];
	mpc_params.eig_q_rk = new Eigen::MatrixXf[mpc_params.T];
	mpc_params.eig_q_lambda_k = new Eigen::MatrixXf[mpc_params.T];

	// Set the size of the goal variable (xg,yg,zg,psi_g)
	goal = Eigen::MatrixXf::Zero(4,1);

	// Set the matrices storing the roll and pitch angles resulting from a given control policy
	quadrotor.eig_phi = Eigen::MatrixXf::Zero(mpc_params.T,1);
	quadrotor.eig_theta = Eigen::MatrixXf::Zero(mpc_params.T,1);

	// Initialize an object which stores many variables needed to compute a solution to the MPC problem
	Matrix_Set temp_bomt(quadrotor, mpc_params);

	// Set the FMPC object's bomt object
	bomt = temp_bomt;

	// Set the b vector
	bomt.b = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);

	// Allocate memory to Eigen::Matrices, one for each step in the path stride.
	full_trajectory = new Eigen::MatrixXf[mpc_params.nu_X];
	prevfull_trajectory = new Eigen::MatrixXf[mpc_params.nu_X];
	full_trajectory_interf = new Eigen::MatrixXf[mpc_params.nu_X];
	full_policy = new Eigen::MatrixXf[mpc_params.nu_X];
	full_attitude = new Eigen::MatrixXf[mpc_params.nu_X];
	prev_seg_eig_X = new Eigen::MatrixXf[mpc_params.nu_X];
	prev_seg_eig_U = new Eigen::MatrixXf[mpc_params.nu_X];

	// Set the local path to zero
	localPath = Eigen::MatrixXf::Zero(1,3);

	// Set the parent ellipsoid to zero
	parentEllipsoid = Eigen::MatrixXf::Zero(3,3);

	// Set the size of the objective function value
	Obj = Eigen::MatrixXf::Zero(1,1);

	u_k_nuX = new Eigen::MatrixXf[mpc_params.nu_X];
	v_k_nuX = new Eigen::MatrixXf[mpc_params.nu_X];
	du1_k_nuX = new Eigen::MatrixXf[mpc_params.nu_X];
	ddu1_k_nuX = new Eigen::MatrixXf[mpc_params.nu_X];
	lambda_k_nuX = new Eigen::MatrixXf[mpc_params.nu_X];
	zeta_k_nuX = new Eigen::MatrixXf[mpc_params.nu_X];
	T_k_nuX = new Eigen::MatrixXf[mpc_params.nu_X];

	u_k = Eigen::MatrixXf::Zero(4,mpc_params.T);
	v_k = Eigen::MatrixXf::Zero(4,mpc_params.T);
	du1_k = Eigen::MatrixXf::Zero(1,mpc_params.T);
	ddu1_k = Eigen::MatrixXf::Zero(1,mpc_params.T);
	lambda_k = Eigen::MatrixXf::Zero(4,mpc_params.T);
	zeta_k = Eigen::MatrixXf::Zero(4,mpc_params.T);

	for (int i = 0; i < mpc_params.nu_X; i++)
	{

		u_k_nuX[i] = Eigen::MatrixXf::Zero(4,mpc_params.T);
		v_k_nuX[i] = Eigen::MatrixXf::Zero(4,mpc_params.T);
		du1_k_nuX[i] = Eigen::MatrixXf::Zero(1,mpc_params.T);
		ddu1_k_nuX[i] = Eigen::MatrixXf::Zero(1,mpc_params.T);
		lambda_k_nuX[i] = Eigen::MatrixXf::Zero(4,mpc_params.T);
		zeta_k_nuX[i] = Eigen::MatrixXf::Zero(4,mpc_params.T);
		T_k_nuX[i] = Eigen::MatrixXf::Zero(4,mpc_params.T);

	}


} // F_MPC_UNCUT::F_MPC_UNCUT()


// Destructor
F_MPC_UNCUT::~F_MPC_UNCUT()
{
	
} // F_MPC_UNCUT::~F_MPC_UNCUT()
