// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 16th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file implements the trajectory-planning thread: the top-level start()/quit_handler()
// lifecycle, pose/waypoint/goal bookkeeping, obstacle lookup, and the main trajectory_planner_thread()
// loop that drives the MPC solver and feedback-linearization stages segment by segment to produce the
// final control request.
// End File Decscription ################################################################################

#include <f_mpc_uncut.h>
#include <f_mpc_globals.h>

// Member function of F_MPC_UNCUT
// quit_handler: this function handles program behavior when certain signals are raised
// INPUTS: integer representing the raised signal
// OUTPUTS: none
void F_MPC_UNCUT::quit_handler( int sig )
{

	// If the user pressed ctrl+c
  	if (sig == SIGINT)
	{
		
		printf("\n");
		printf("<QUIT HANDLER> TERMINATING AT USER REQUEST\n");
		string message = "<QUIT HANDLER> TERMINATING AT USER REQUEST\n";
		int writeStatus = log_f_mpc.writeToLog(message);		
		printf("\n");

	} // if (sig == SIGINT)

	// If the pogram was aborted (from an error)
  	if (sig == SIGABRT)
	{
		
		printf("\n");
		printf("<QUIT HANDLER> TERMINATING AFTER ABORT RAISED\n");
		string message = "<QUIT HANDLER> TERMINATING AFTER ABORT RAISED\n";
		int writeStatus = log_f_mpc.writeToLog(message);		
		printf("\n");

	} // if (sig == SIGABRT)

	// If a segmentation fault was thrown
  	if (sig == SIGSEGV)
	{
		
		printf("\n");
		printf("<QUIT HANDLER> TERMINATING AFTER SEGMENTATION FAULT\n");
		printf("<QUIT HANDLER> PRESS ENTER TO CONTINUE\n");
		string message = "<QUIT HANDLER> AFTER SEGMENTATION FAULT\n";
		int writeStatus = log_f_mpc.writeToLog(message);		
		message = "<QUIT HANDLER> PRESS ENTER TO CONTINUE\n";
		writeStatus = log_f_mpc.writeToLog(message);		
		printf("\n");
		cin.ignore();

	} // if (sig == SIGSEGV)

	time_to_exit = 1;
	usleep(2000000);
	exit(0);

} // void F_MPC_UNCUT::quit_handler( int sig )


// Member function of F_MPC_UNCUT object
// find_closest_obstacle: determines the closest obstacle to the UAVs position
// INPUTS: pointer to a matrix representing the goal point, integer representing the segment being planned
// OUTPUTS: none
void F_MPC_UNCUT::find_closest_obstacle(Eigen::MatrixXf* temp_goal_pos, int segment_number)
{

	// Declare a vector of pairs
	vector<pair<float,int>> d2;

	// Declare a float storing the distance
	float dist_norm;

	// Define matrices to store the orientation of the focal axis, the position, and a dot product
	Eigen::MatrixXf direction = Eigen::MatrixXf::Zero(3,1);
	Eigen::MatrixXf position = Eigen::MatrixXf::Zero(3,1);
	Eigen::MatrixXf dot_prod = Eigen::MatrixXf::Zero(1,1);

	direction(0,0) = cos(quadrotor.X0(12));
	direction(1,0) = sin(quadrotor.X0(12));

	position(0,0) = quadrotor.X0(0);
	position(1,0) = quadrotor.X0(1);

	// If there are obstacles
	if (local_obs.rows() > 0)
	{

		// Iterate over the number of obstacles
		for (int i = 0; i < local_obs.rows(); i++)
		{
			
			// Compute the norm of the distance
			dist_norm = ( local_obs.row(i) - (*temp_goal_pos) ).norm(); 

			// Compute the dot product
			dot_prod = ( (local_obs.row(i).transpose() - position).transpose()*direction).diagonal();
			
			// If the distance is less than some user-defined value and the obstacle is in the same direction the UAV is facing
			if ( ( dist_norm < 1/mpc_params.mu_11 ) && ( dot_prod(0,0) > 0 ) )
			{
				
				d2.push_back( make_pair( dist_norm , i ) );

			} // if ( ( dist_norm < 1/mpc_params.mu_11 ) && ( dot_prod(0,0) > 0 ) )

		} // for (int i = 0; i < local_obs.rows(); i++)

		// If there are obstacle in the same direction that the UAV is facing
		if (d2.size() > 0)
		{

			// Sort the vector storing the distance norms and labels
			sort(d2.begin(),d2.end());

			// Compute the nearest obstacle
			r_obs = local_obs.row(d2[0].second).transpose();

		} // if (d2.size() > 0)
		else if (d2.size() == 0 && localPath.rows() > 0 && segment_number < localPath.rows())
		{

			r_obs(0,0) = localPath(segment_number,0); 
			r_obs(1,0) = localPath(segment_number,1); 
			r_obs(2,0) = localPath(segment_number,2); 

		} // else if (d2.size() == 0 && localPath.rows() > 0 && traj_iterator < localPath.rows())
		else
		{

			r_obs(0,0) = quadrotor.X0(0); 
			r_obs(1,0) = quadrotor.X0(1); 
			r_obs(2,0) = quadrotor.X0(2); 

		} // if (d2.size() > 0)

	} // if (local_obs.rows() > 0)
	else
	{

		// If there are no obstacles, just use the planned path
		if (localPath.rows() > 0)
		{
			
			r_obs(0,0) = localPath(segment_number,0); 
			r_obs(1,0) = localPath(segment_number,1); 
			r_obs(2,0) = localPath(segment_number,2); 	

		} // if (localPath.rows() > 0)
		else
		{
			
			r_obs(0,0) = quadrotor.X0(0); 
			r_obs(1,0) = quadrotor.X0(1); 
			r_obs(2,0) = quadrotor.X0(2); 

		} // if (localPath.rows() > 0)

	} // if (local_obs.rows() > 0)

	cout << "<FMPC> find_closest_obstacle complete" << endl;

} // void F_MPC_UNCUT::find_closest_obstacle(Eigen::MatrixXf* temp_goal_pos, int segment_number)


void* start_trajectory_planning_thread(F_MPC_UNCUT* f_mpc_uncut)
{

	if( trajectory_status != 0 ){
		fprintf(stderr,"trajectory planning thread already running\n");
		return NULL;
	}
	else{
		f_mpc_uncut->trajectory_planner_thread();
		return NULL;
	}
	return NULL;

}


void* start_traj_interface_thread(void *args)
{

	F_MPC_UNCUT* f_mpc_uncut = (F_MPC_UNCUT* )args;

	start_trajectory_planning_thread(f_mpc_uncut);

	return NULL;

}


void F_MPC_UNCUT::start()
{

	signal(SIGPIPE, quit_handler);
	signal(SIGINT, quit_handler);
	signal(SIGSEGV, quit_handler);
	signal(SIGABRT, quit_handler);

	pthread_mutex_init(&pose_lock, NULL);
	pthread_mutex_init(&path_lock, NULL);
	pthread_mutex_init(&map_lock, NULL);
	pthread_mutex_init(&constraint_lock, NULL);
	pthread_mutex_init(&trajectory_lock, NULL);
	pthread_mutex_init(&ellipsoid_lock, NULL);

	int	result_traj = pthread_create( &traj_tid, NULL, &start_traj_interface_thread, this);	
	if ( result_traj )
	{
		cout << "<TRAJ-PLANNER-FMPC> Unable to start trajectory planner thread." << endl;
		log_f_mpc.writeToLog( "<TRAJ-PLANNER-FMPC> Unable to start trajectory planner thread.");
		exit(0);
	}

	int	result_comm = pthread_create( &comm_tid, NULL, &start_comm_interface_thread, this);	
	if ( result_comm )
	{
		cout << "<TRAJ-PLANNER-FMPC> Unable to start communication thread." << endl;
		log_f_mpc.writeToLog( "<TRAJ-PLANNER-FMPC> Unable to start communication thread.");
		exit(0);
	}

	while(!time_to_exit)
	{

	}

}


void F_MPC_UNCUT::update_quadrotor_pose()
{

	quadrotor.X0 *= 0;
	quadrotor.X0(0) = pose[1]; // x
	quadrotor.X0(1) = -pose[0]; // y
	quadrotor.X0(2) = -pose[2]; // z
	quadrotor.X0(3) = pose[4]; // dx
	quadrotor.X0(4) = -pose[3]; // dy
	quadrotor.X0(5) = -pose[5]; // dz
	quadrotor.X0(6) = pose[7]; // ddx
	quadrotor.X0(7) = -pose[6]; // ddy
	quadrotor.X0(8) = -pose[8]; // ddz
	quadrotor.X0(9) = pose[10]; // dddx
	quadrotor.X0(10) = -pose[9]; // dddy
	quadrotor.X0(11) = -pose[11]; // dddz
	quadrotor.X0(12) = pose[14]; // psi 	

	get_Omega();
	compute_Gamma();
	compute_dEuler();
	quadrotor.X0(13) = dEuler(2,0); // dpsi

	bomt.eX0 = Eigen::MatrixXf::Zero(18,1);
	for (int i = 0; i < quadrotor.n; i++)
	{
		bomt.eX0(i,0) = pose[i];
	}

	// cout << "<FMPC> update_quadrotor_pose complete" << endl;

}


void F_MPC_UNCUT::find_closest_waypoint(float* xdiff, float* ydiff, float* zdiff, int* closest_traj_iterator)
{
	float min_norm = 9999999;

	for(int i = 0; i < localPath.rows()-1; ++i)
	{

		(*xdiff) = quadrotor.X0(0) - localPath(i,0);
		(*ydiff) = quadrotor.X0(1) - localPath(i,1);
		(*zdiff) = quadrotor.X0(2) - localPath(i,2);

		if( (*xdiff)*(*xdiff) + (*ydiff)*(*ydiff) + (*zdiff)*(*zdiff) < min_norm )
		{

			min_norm = (*xdiff)*(*xdiff) + (*ydiff)*(*ydiff) + (*zdiff)*(*zdiff);
			traj_iterator = i;

		}
	}

	(*closest_traj_iterator) = traj_iterator;

	cout << "Closest point (supposedly): " << localPath.row(traj_iterator) << endl;
	cout << "<FMPC> find_closest_waypoint complete" << endl;

}


// Member function of F_MPC_UNCUT
// find_new_waypoint: this function is used to determine what the 
// next waypoint in the path should be
// INPUTS: pointer to an Eigen::MatrixXf capturing the goal,
// pointer to floats captuing the difference between the quadrotor's
// position and the goal
// OUTPUTS: none 
void F_MPC_UNCUT::find_new_waypoint(Eigen::MatrixXf* goal, float* xdiff, float* ydiff, float* zdiff)
{

	float yawdiff;
	int local_traj_iterator;
	local_traj_iterator = traj_iterator;
	Eigen::Vector3f quad_dir, yaw_des_dir;
	bool foundPoint = false;
	float norm_diff;

	// Integers capturing the voxel corresponding to the quadrotor's position
	int x,y,z,dx,dy,dz,nnwp, sx, sy, sz, exy, exz, ezy, ax, ay, az, bx, by, bz;

	// Compute a vector storing the orientation of the quadrotor's focal axis
	quad_dir(0) = cos(quadrotor.X0(12)); 
	quad_dir(1) = sin(quadrotor.X0(12)); 
	quad_dir(2) = 0.0; 

	(*xdiff) = quadrotor.X0(0) - localPath(local_traj_iterator,0);
	(*ydiff) = quadrotor.X0(1) - localPath(local_traj_iterator,1);
	(*zdiff) = quadrotor.X0(2) - localPath(local_traj_iterator,2);
	norm_diff = sqrt((*xdiff)*(*xdiff) + (*ydiff)*(*ydiff) + (*zdiff)*(*zdiff));

	// Search for the first point of the a* output that's 'goal_tolerance' away from the current position
	// Also, find the closest point of the A* output that is in the UAV's direction of motion
	// TODO:  We cannot assume that the path will never require the UAV to turn around
	// so make sure that if the closest waypoint is not in the UAV's direction of motion, that the next waypoint 
	// IN THE QUEUE is also not in the UAV's direction of motion AND not the previous waypoint
	for(; local_traj_iterator < localPath.rows() - 1; local_traj_iterator++)
	{

		(*xdiff) = quadrotor.X0(0) - localPath(local_traj_iterator,0);
		(*ydiff) = quadrotor.X0(1) - localPath(local_traj_iterator,1);
		(*zdiff) = quadrotor.X0(2) - localPath(local_traj_iterator,2);

		// yawdiff = atan2( plannedPath(local_traj_iterator,0) - quadrotor.X0(0), plannedPath(local_traj_iterator,1) - quadrotor.X0(1) );
		yawdiff = atan2( localPath(local_traj_iterator,0) - quadrotor.X0(0), localPath(local_traj_iterator,1) - quadrotor.X0(1) );

		yaw_des_dir(0) = cos(yawdiff);
		yaw_des_dir(1) = sin(yawdiff);
		yaw_des_dir(2) = 0.0;

		norm_diff = sqrt((*xdiff)*(*xdiff) + (*ydiff)*(*ydiff) + (*zdiff)*(*zdiff));

		// if(( norm_diff > mpc_params.goal_tolerance ) && (quad_dir.dot(yaw_des_dir)) >= 0.0 )
		if ( norm_diff > mpc_params.goal_tolerance )
		{

			x = floor(quadrotor.X0(0)*5.0 + 0.5);
			y = floor(quadrotor.X0(1)*5.0 + 0.5);
			z = floor(quadrotor.X0(2)*5.0 + 0.5);

			// Record the difference in position of the camera and sample point
			dx = (int) (floor(localPath(local_traj_iterator,0)*5.0 + 0.5) - x);
			dy = (int) (floor(localPath(local_traj_iterator,1)*5.0 + 0.5) - y);
			dz = (int) (floor(localPath(local_traj_iterator,2)*5.0 + 0.5) - z);			

			// Retrieve the sign of the change in each direction, store in sx/y/z
			get_sign(sx,dx); get_sign(sy,dy); get_sign(sz,dz);
			
			// magnitude of change in each direction
			ax = abs(dx); ay = abs(dy); az = abs(dz);
			
			bx = 2*ax; by = 2*ay; bz = 2*az;
			
			exy = ay-ax; exz = az-ax; ezy = ay-az; 
			
			nnwp = ax+ay+az;

			foundPoint = true;

			// If we have hit an obstacle, mark it explored, and exit the while loop
			while(pthread_mutex_trylock(&map_lock))
			{
				usleep(1);
			}
			pthread_mutex_unlock(&map_lock);
			pthread_mutex_lock(&map_lock);

				// Iterative voxel ray tracing See Ch V.3 of Graphics Gems 4
				while( nnwp-- && x < 100 && y < 30 && z < 100 && x >= 0 && y >= 0 && z >= 0)
				{
		
					if ( abs(x - floor(localPath(local_traj_iterator,0)*5+0.5)) <= 1 && abs(y - floor(localPath(local_traj_iterator,1)*5+0.5)) <= 1 && abs(z - floor(localPath(local_traj_iterator,2)*5+0.5) ) <= 1 )
					{
						cout << "ray trace got to goal!" << endl;
						break;
					}
					
					if (voxel_map[(y)][(z)][(x)] == 3)
					{

						cout << "ray trace did not get to goal!" << endl;
						local_traj_iterator-=1;
						local_traj_iterator = max(0,local_traj_iterator);
						foundPoint = true;
						break;

					} // if (voxel_map[(x)][(y)][(z)])
					// If ray tracing goes beyond the maps boundaries, stop
					else if (x + sx < 0 || y + sy < 0 || z + sz < 0 || x + sx >= 100 || y + sy >= 30 || z + sz >= 100)
					{
						break;

					} // if (x + sx < 0 || y + sy < 0 || z + sz < 0 || x + sx >= 100 || y + sy >= 30 || z + sz >= 100)
					// Otherwise, step along the ray
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

					} // if (voxel_map[(x)][(y)][(z)])


				} // while( n-- && x < 100 && y < 30 && z < 100)

			pthread_mutex_unlock(&map_lock);
			
			if (foundPoint)
			{
				break;
			}

		}	

	}

	// If we did not find a point nearby that the UAV is facing, search again only for nearby points
	// if (!foundPoint)
	// {
	// 	cout << "Did not find a point that the UAV is facing" << endl;
	// 	local_traj_iterator = 1;
	// }

	// Set the goal position
	(*goal)(0,0) = localPath(local_traj_iterator,0);
	(*goal)(1,0) = localPath(local_traj_iterator,1);
	(*goal)(2,0) = localPath(local_traj_iterator,2);

	// ************************************************************ //
	// This block determines if the goal coincides with an obstacle //
	// ************************************************************ //

	// If there are some obstacle points
	if (local_obs.rows() > 0)
	{

		// Iterate over the number of obstacle points
		for (int i = 0; i < local_obs.rows(); i++)
		{

			// If the goal and the obstacle point are close enough
			if ( (local_obs.row(i)-(*goal).block(0,0,3,1).transpose()).norm() < 0.2)
			{
				cout << "goal coincides with obstacle" << endl;
				// Step back one point in the path
				local_traj_iterator--;

				// Update the goal point 
				if (local_traj_iterator >= 0 && local_traj_iterator < localPath.rows())
				{
				
					(*goal)(0,0) = localPath(local_traj_iterator,0);
					(*goal)(1,0) = localPath(local_traj_iterator,1);
					(*goal)(2,0) = localPath(local_traj_iterator,2);		
					break;

				} // if (local_traj_iterator >= 0)
				else
				{
					
					(*goal)(0,0) = localPath(0,0);
					(*goal)(1,0) = localPath(0,1);
					(*goal)(2,0) = localPath(0,2);
					break;

				} // if (local_traj_iterator >= 0)

			} // if ( (local_obs.row(i)-(*goal).block(0,0,3,1).transpose()).norm() < 0.2)

		} // for (int i = 0; i < local_obs.rows(); i++)

	} //if (local_obs.rows() > 0)

	// **************************************** //
	// Project the goal onto the constraint set //
	// **************************************** //
	while(pthread_mutex_trylock(&constraint_lock))
	{
		usleep(1);
	}
	pthread_mutex_unlock(&constraint_lock);
	pthread_mutex_lock(&constraint_lock);
						
		Eigen::Vector3f n0, n1, n2, p_temp, proj_goal;
		Eigen::MatrixXf	p = Eigen::MatrixXf::Zero(3,collisionConstraints.rows());

		float d = 0.0, z1 = 0.0, plane_size = 2.0;

		Eigen::Vector3f v, proj_quad_on_plane_temp, goal_temp, difference_temp;
		Eigen::MatrixXf proj_quad_on_plane = Eigen::MatrixXf::Zero(3,collisionConstraints.rows());
		float dist, lam = 1.0;

		bool early_break, not_in_constraint_set = false;

		// cout << "(*goal): " << (*goal) << endl;
		goal_temp = Eigen::Map<Eigen::Vector3f>((*goal).data(),3);

		// Iterate over the number of constraints
		for (int j = 0; j < collisionConstraints.rows(); j++)
		{
			// find normal vector to the plane & offset
			n0(0) = collisionConstraints(j,0);
			n0(1) = collisionConstraints(j,1);
			n0(2) = collisionConstraints(j,2);
			d = collisionConstraints(j,3);

			// Find two vectors that form an orthonormal basis for the plane
			n1(0) = -n0(1);
			n1(1) = n0(0);
			n1(2) = z1;
			n1.normalize();

			n2 = n0.cross(n1); 

			// Find a point on the plane
			p_temp = quadrotor.X0.head(3) + n2*plane_size + n1*plane_size + n0*d;
			p.col(j) = Eigen::Map<Eigen::MatrixXf>(p_temp.data(),3,1);

		} // for (int j = 0; j < collisionConstraints.rows(); j++)


		// Iterate over the number of constraints
		for (int j = 0; j < collisionConstraints.rows(); j++)
		{
			// Project the quadrotor point onto each constraint
			v = quadrotor.X0.head(3) - Eigen::Map<Eigen::Vector3f>(p.col(j).data(),3);
			
			if (abs(collisionConstraints.block(j,0,1,3).norm()) < 10e-6)
			{
				continue;
			}

			n0(0) = collisionConstraints(j,0);
			n0(1) = collisionConstraints(j,1);
			n0(2) = collisionConstraints(j,2);
			dist = v.dot(-n0);

			proj_quad_on_plane_temp = quadrotor.X0.head(3) + dist*n0;
			proj_quad_on_plane.col(j) = Eigen::Map<Eigen::MatrixXf>(proj_quad_on_plane_temp.data(),3,1);

			// Find the vector connecting the goal to the quadrotor projected onto the constraint
			difference_temp = goal_temp-proj_quad_on_plane_temp;
			difference_temp.normalize();

			// If the vectors do not point in opposite directions, 
			// the goal is not in the constraint set
			// if (-n0.dot(difference_temp) >= 0)
			if (n0.dot(goal_temp) >= collisionConstraints(j,3))
			{
				// indicate that the goal is not in the constraint set
				not_in_constraint_set = true;

				cout << "p: " << endl << p << endl;
				cout << "goal_temp: " << goal_temp << endl;
				cout << "n0: " << n0.transpose() << endl;
				cout << "dist: " << dist << endl;
				cout << "proj_quad_on_plane_temp: " << proj_quad_on_plane_temp.transpose() << endl;
				cout << "difference_temp: " << difference_temp.transpose() << endl;
				cout << "n.dot(difference_temp): " << n0.dot(difference_temp) << endl;
				// cin.ignore();

				// stop searching
				break;

			} // if (-n.dot(difference_temp) >= 0)

		} // for (int j = 0; j < collisionConstraints.rows(); j++)

		// While the goal is not in the constraint set
		while(not_in_constraint_set)
		{

			lam*=0.9;
			// cout << "lam: " << lam << endl;

			// Find a provisional goal point as the convex combination
			// of the goal and quadrotor point
			proj_goal = lam*goal_temp+(1-lam)*quadrotor.X0.head(3);

			// Indicate that we have not stopped checking the provisional
			// goal for constraint adherence
			early_break = false;
			
			// Iterate over the number of constraints
			for (int j = 0; j < collisionConstraints.rows(); j++)
			{
				
				// Find vector pointing from the provisional goal
				// to the quadrotor projected onto the plane
				difference_temp = proj_goal-proj_quad_on_plane.col(j);
				difference_temp.normalize();

				// If the vector points in the same direction as the constraint,
				// stop checking
				if (-n0.dot(difference_temp) >= 0)
				{
					
					// Indicate that we have stopped early
					early_break = true;

					// Break out of for loop
					break;

				} // if (-n.dot(difference_temp) >= 0)

			} // for (int j = 0; j < collisionConstraints.rows(); j++)

			// If we never broke the for loop above
			if (early_break)
			{
				
				// The provisional goal satisfies the constraints
				(*goal).block(0,0,3,1) = Eigen::Map<Eigen::MatrixXf>(proj_goal.data(),3,1);				

				// Indicate that the goal is in the constraint set
				not_in_constraint_set = false;
				cout << "proj_goal: " << (*goal) << endl;

			} // if (!early_break)

			if (abs(lam) < 10e-4)
			{
				break;
			}

		} // while(not_in_constraint_set)

	pthread_mutex_unlock(&constraint_lock);

	traj_iterator = local_traj_iterator;

	cout << "<FMPC> find_new_waypoint complete" << endl;

} // void F_MPC_UNCUT::find_new_waypoint(Eigen::MatrixXf* goal, float* xdiff, float* ydiff, float* zdiff)


void F_MPC_UNCUT::get_trajectory_goal()
{

	float xdiff, ydiff, zdiff, lambda;

	if (localPath.rows() >= 1)
	{

		for (int i = 0; i < mpc_params.nu_X; i++)
		{
			prev_seg_eig_X[i] = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
			prev_seg_eig_U[i] = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
		}

		// find new goal waypoint
		traj_iterator = 0;
		int closest_traj_iterator = 0;
		find_closest_waypoint(&xdiff, &ydiff, &zdiff, &closest_traj_iterator);
		find_new_waypoint(&goal, &xdiff, &ydiff, &zdiff);

		// Eigen::MatrixXf temp = Eigen::MatrixXf::Zero(1,3);
		// temp = (localPath.row(traj_iterator)-localPath.row(closest_traj_iterator)) / (localPath.row(traj_iterator)-localPath.row(closest_traj_iterator)).norm();
		
		// Eigen::Vector3f hat_n_xk = Eigen::Map<Eigen::Vector3f>(temp.data(),3);
		// Eigen::Vector3f hat_n_yk;
		// hat_n_yk(0) = 0.0; 
		// hat_n_yk(1) = 0.05; 
		// hat_n_yk(2) = 9.81;
		// hat_n_yk = (hat_n_xk.cross(hat_n_yk)) / ((hat_n_xk.cross(hat_n_yk)).norm());
		// Eigen::Vector3f hat_n_zk = hat_n_xk.cross(hat_n_yk);

		// Eigen::Matrix3f Rfk;
		// Rfk.col(0) = hat_n_xk;
		// Rfk.col(1) = hat_n_yk;
		// Rfk.col(2) = hat_n_zk;

		// if (Rfk.determinant() <= 1+0.05 && Rfk.determinant() >= 1-0.05)
		// {	
		// 	Eigen::Vector3f euler;
		// 	compute_Euler_angles(&Rfk,&euler);
		// 	// cout << "Heading from rotm: " << euler(2) << endl;
		// }

		goal(3,0) = atan2(goal(0,0)-quadrotor.X0(0), goal(1,0)-quadrotor.X0(1));
		
		prev_plannedPath = Eigen::MatrixXf::Zero(localPath.rows(),localPath.cols());
		prev_plannedPath = localPath;


		if (sqrt(xdiff*xdiff + ydiff*ydiff) < 0.313)
		{
			goal(3,0) = quadrotor.X0(12);
		}

		remaining_segments = localPath.rows() - traj_iterator;
		cout << "remaining_segments: " << remaining_segments << endl;

	}
	else
	{
		goal(0,0) = quadrotor.X0(0);
		goal(1,0) = quadrotor.X0(1);
		goal(2,0) = quadrotor.X0(2);
		goal(3,0) = quadrotor.X0(12);
		remaining_segments = 1;
	}

	cout << "<FMPC> get_trajectory_goal complete" << endl;

}


void F_MPC_UNCUT::trajectory_planner_thread()
{

	trajectory_status = 1;

	string message;
	message = "<TRAJ-PLANNER-FMPC> Starting trajectory planner thread.";
	cout << message << endl;
	log_f_mpc.writeToLog(message);

	Eigen::MatrixXf seg_state = Eigen::MatrixXf::Zero(quadrotor.n,1);
	Eigen::MatrixXf seg_control = Eigen::MatrixXf::Zero(quadrotor.m,1);
	Eigen::MatrixXf temp_goal_pos = Eigen::MatrixXf::Zero(1,3);
	Eigen::MatrixXf eig_X = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
	Eigen::MatrixXf prev_eig_X = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
	Eigen::MatrixXf eig_U = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
	Eigen::MatrixXf prev_eig_U = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
	Eigen::VectorXf temporary_pose = Eigen::VectorXf::Zero(quadrotor.n);
	Eigen::MatrixXf prev_goal = Eigen::MatrixXf::Zero(4,1);
	// Eigen::MatrixXf segmentGoals = Eigen::MatrixXf::Zero(0,4*mpc_params.nu_X);
	segmentGoals = Eigen::MatrixXf::Zero(0,4*mpc_params.nu_X);
	
	auto current_time = std::chrono::high_resolution_clock::now();
	auto current_time1 = std::chrono::high_resolution_clock::now();
	auto pauseTime_start = std::chrono::high_resolution_clock::now();
	auto end_time = std::chrono::high_resolution_clock::now();
	auto end_time1 = std::chrono::high_resolution_clock::now();
	auto pauseTime_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed = end_time - current_time;
	std::chrono::duration<double> timeElapsed1 = end_time1 - current_time1;
	std::chrono::duration<double> pauseTime = pauseTime_end - pauseTime_start;
	double waitTime, sleepTime = 2500000;

	bool g_barrier_violation;
	int localPathSize, shifted_traj_iterator;
	bool badTraj = 0;
	int jjj = 0;

	for (int i = 0; i < mpc_params.nu_X; i++)
	{
		full_trajectory[i] = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
		prevfull_trajectory[i] = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
		full_trajectory_interf[i] = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
		full_policy[i] = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
		full_attitude[i] = Eigen::MatrixXf::Zero(2,mpc_params.T);
		prev_seg_eig_X[i] = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
		prev_seg_eig_U[i] = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
	}
	
	while(!firstPassComplete)
	{
		usleep(1000);
	}

	// usleep(100000000);

	message = "<TRAJ-PLANNER-FMPC> Entering trajectory planning loop.";
	cout << message << endl;
	log_f_mpc.writeToLog(message);	



	while(!time_to_exit)
	{

		if (firstPassComplete)
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

		current_time1 = std::chrono::high_resolution_clock::now();

		end_time = std::chrono::high_resolution_clock::now();
		timeElapsed = end_time - current_time;

		while(pthread_mutex_trylock(&pose_lock))
		{
			usleep(1);
		}
		pthread_mutex_unlock(&pose_lock);
		pthread_mutex_lock(&pose_lock);


			// Store the pose
			quadrotor.prevX0 = quadrotor.X0;
			update_quadrotor_pose();

			if (firstPassComplete)
			{

				pauseTime_start = std::chrono::high_resolution_clock::now();

				while ( 1 )
				{

					pauseTime_end = std::chrono::high_resolution_clock::now();
					// check if the pose has changed enough
					if ( (quadrotor.prevX0.head(3) - quadrotor.X0.head(3)).norm() > 0.6 )
					{
						// If so, allow to continue
						break;
					}	
					
					update_quadrotor_pose();
					pauseTime = pauseTime_end - pauseTime_start; 

					// Check if enough time has passed
					if (pauseTime.count() > mpc_params.delta_t*mpc_params.T)
					{
						// If so, allow continue
						update_quadrotor_pose();
						break;
					}

				}
				// allow to continue 
			}

		pthread_mutex_unlock(&pose_lock);

		temporary_pose = quadrotor.X0;
		prev_goal = goal;

		while(pthread_mutex_trylock(&path_lock))
		{
			usleep(1);
		}
		pthread_mutex_unlock(&path_lock);
		pthread_mutex_lock(&path_lock);

			int pathSize = plannedPath.size()/3;

			if (pathSize > 0)
			{
				localPath = Eigen::MatrixXf::Zero(pathSize,3);
				localPath = plannedPath;
			}
			else
			{
				localPath = Eigen::MatrixXf::Zero(1,3);
				localPath(0,0) = quadrotor.X0(0);
				localPath(0,1) = quadrotor.X0(1);
				localPath(0,2) = quadrotor.X0(2);
			}

		pthread_mutex_unlock(&path_lock);

		while(pthread_mutex_trylock(&map_lock))
		{
			usleep(1);
		}
		pthread_mutex_unlock(&map_lock);
		pthread_mutex_lock(&map_lock);	

			// Copy the matrix of obstacle points to a "local"
			// variable. (its not local to the function, but
			// acts as if it were)
			local_obs = Eigen::MatrixXf::Zero(obs.rows(),obs.cols());
			local_obs = obs;

		pthread_mutex_unlock(&map_lock);
		
		get_trajectory_goal();

		segmentGoals.conservativeResize(segmentGoals.rows()+1,4*mpc_params.nu_X);
		segmentGoals(numTrajectoryPlans-1,0) = goal(0,0);
		segmentGoals(numTrajectoryPlans-1,1) = goal(1,0);
		segmentGoals(numTrajectoryPlans-1,2) = goal(2,0);
		segmentGoals(numTrajectoryPlans-1,3) = goal(3,0);

		mpc_params.nu_sk = max(min(remaining_segments,mpc_params.nu_X),1);
		cout << "remaining_segments: " << remaining_segments << endl;
		// cout << "mpc_params.nu_X: " << mpc_params.nu_X << endl;

		localPathSize = localPath.rows(); 

		prev_plannedPath.conservativeResizeLike(localPath);
		prev_plannedPath = localPath;

		bomt.X_goal(0,0) = goal(0,0); 
		bomt.X_goal(1,0) = goal(1,0); 
		bomt.X_goal(2,0) = goal(2,0); 
		bomt.X_goal(12,0) = goal(3,0); 	

		delete [] mpc_params.P;
		delete [] mpc_params.h;
		delete [] mpc_params.h_tilde;

		mpc_params.P = new Eigen::MatrixXf[mpc_params.nu_X];
		mpc_params.h = new Eigen::MatrixXf[mpc_params.nu_X];
		mpc_params.h_tilde = new Eigen::MatrixXf[mpc_params.nu_X];


		cout << "New planning episode!" << endl;

		int ii = 0;
		int iii = 0;

		// if ((Eigen::Map<Eigen::MatrixXf>(quadrotor.X0.head(3).data(),1,3) ).norm() < 0.6)
		// if ( (Eigen::Map<Eigen::MatrixXf>(quadrotor.X0.head(3).data(),1,3) - localPath.row(localPath.rows()-1)).norm() < 0.6)
		// {

		while(pthread_mutex_trylock(&trajectory_lock))
		{
			usleep(1);
		}
		pthread_mutex_unlock(&trajectory_lock);
		pthread_mutex_lock(&trajectory_lock);

			while ( ii < mpc_params.nu_sk )
			{

				cout << "ii: " << ii << endl;

				if (ii == 0)
				{

					Obj(0,0) = 0.0;

					temp_goal_pos(0,0) = goal(0,0);	
					temp_goal_pos(0,1) = goal(1,0);	
					temp_goal_pos(0,2) = goal(2,0);	

					cout << "goal: " << goal.transpose() << endl;

					eig_X = Eigen::MatrixXf::Zero(quadrotor.n,mpc_params.T);
					eig_U = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);

					for( int i = 0; i < mpc_params.T; i++ )
					{
						eig_X.col(i) = quadrotor.X0;
						eig_X.block(3,i,9,1)*=0;
						eig_X.block(13,i,1,1)*=0;
						eig_U.col(i) = Eigen::MatrixXf::Zero(4,1);
					}

					while(pthread_mutex_trylock(&map_lock))
					{
						usleep(1);
					}
					pthread_mutex_unlock(&map_lock);
					pthread_mutex_lock(&map_lock);
					
						// Find the closest obstacle to the previous goal
						find_closest_obstacle(&temp_goal_pos,ii);
					
					pthread_mutex_unlock(&map_lock);
					if (!firstSuccess)
					{

						for (int j = 0; j < mpc_params.T; j++)
						{
							g_barrier(j,0) = (quadrotor.phi_max*quadrotor.phi_max)*(quadrotor.theta_max*quadrotor.theta_max)*quadrotor.mass*9.81*0.25;
						}

						update_weighting_matrices(ii, 1);
					}

					while(pthread_mutex_trylock(&constraint_lock))
					{
						usleep(1);
					}
					pthread_mutex_unlock(&constraint_lock);
					pthread_mutex_lock(&constraint_lock);
					
						// Update the constraints
						fmpc_hard_constraints(&eig_X,ii,traj_iterator,numFailures);
						fmpc_soft_constraints(&eig_X,ii,traj_iterator);
					
					pthread_mutex_unlock(&constraint_lock);

					// If the goal for the first segment is far away, set the goal to the closest point in the path
					if (( quadrotor.X0(0) - goal(0,0) )*( quadrotor.X0(0) - goal(0,0) ) + ( quadrotor.X0(1) - goal(1,0) )*( quadrotor.X0(1) - goal(1,0) ) + ( quadrotor.X0(2) - goal(2,0) )*( quadrotor.X0(2) - goal(2,0) ) > 2.0)
					{				
					
						float xdifftemp0,ydifftemp0,zdifftemp0;
						float min_norm_temp = 99999;
						int temp_traj_iterator = 0;
						for(int i = 0; i < localPath.rows()-1; ++i)
						{

							xdifftemp0 = goal(0,0) - localPath(i,0);
							ydifftemp0 = goal(1,0) - localPath(i,1);
							zdifftemp0 = goal(2,0) - localPath(i,2);

							if( xdifftemp0*xdifftemp0 + ydifftemp0*ydifftemp0 + zdifftemp0*zdifftemp0 < min_norm_temp )
							{

								min_norm_temp = xdifftemp0*xdifftemp0 + ydifftemp0*ydifftemp0 + zdifftemp0*zdifftemp0;
								temp_traj_iterator = i;

							}
						}
						goal.block(0,0,3,1) = localPath.row(temp_traj_iterator).transpose();

					}

					// cout << "segment: " << ii << endl;
					// cout << "position: " << quadrotor.X0(0) << ", " << quadrotor.X0(1)  << ", " << quadrotor.X0(2) << " goal: " << goal.transpose() << endl;

					// cout << "eig_X: " << endl;
					// cout << eig_X << endl;
					// cout << "eig_U: " << endl;
					// cout << eig_U << endl;
					// cout << "traj_iterator: " << traj_iterator <<endl;
					// cout << "goal: " << goal << endl;
					// cout << "robs: " << r_obs.transpose() << endl;
					// cout << "eig_Fx_hard: " << endl << quadrotor.eig_Fx_hard << endl;
					// cout << "eig_x_hard_bounds: " << endl << quadrotor.eig_x_hard_bounds << endl;
					// cout << "eig_Fu_hard: " << endl << quadrotor.eig_Fu_hard << endl;
					// cout << "eig_u_hard_bounds: " << endl << quadrotor.eig_u_hard_bounds << endl;
					// cout << "g_barrier: " << endl << g_barrier << endl;

					// perform fmpc, check if it was successful
					intermediate_success = fmpcsolve(&eig_X, &eig_U, ii);
					// for (int ji = 0; ji < quadrotor.eig_Fx_hard.rows(); ji++)
					// {
					// 	for (int ij = 0; ij < mpc_params.T; ij++)
					// 	{
					// 		if (quadrotor.eig_Fx_hard.row(ji)*eig_X.col(ij) >= quadrotor.eig_x_hard_bounds(ji))
					// 		{
					// 			cout << "Constraint " << ji << " step " << ij << ": " <<  quadrotor.eig_Fx_hard.row(ji)*eig_X.col(ij) << ">=" << quadrotor.eig_x_hard_bounds(ji) << endl;
					// 			cout << "quadrotor.eig_Fx_hard: " << quadrotor.eig_Fx_hard.block(ji,0,1,quadrotor.eig_Fx_hard.cols()) << endl;
					// 		}
					// 	}
					// }

					if (intermediate_success)
					{
						
						cout << "fMPC success, checking g_barrier" << endl;
						compute_u1_u1dot(&eig_U);

						// posterior computation of roll, pitch angles, and thrust for a particular control policy
						// and evaluate g_barrier

						g_barrier_violation = compute_roll_pitch_thrust_for_lambda_k_and_g_barrier(&eig_X, &eig_U);
						
						if ((eig_X.block(0,mpc_params.T-1,3,1) - goal.block(0,0,3,1)).norm() > 2.0)
						{
							cout << "fMPC failed: did not reach goal" << endl;
							// trajectory_plan_success = false;
						}
						
						if (g_barrier_violation)
						{
							cout << " g_barrier violated" << endl;
							// if violation, failure, adjust weighting matrices
							update_weighting_matrices(ii, 1);
							trajectory_plan_success = false;
						} 
						else
						{
							cout << " g_barrier ok" << endl;
							// if no violation, success, continue to next segment
							trajectory_plan_success = true;
							u_k_nuX[ii] = u_k;
							v_k_nuX[ii] = v_k; 
							du1_k_nuX[ii] = du1_k;
							ddu1_k_nuX[ii] = ddu1_k;
							lambda_k_nuX[ii] = lambda_k;
							zeta_k_nuX[ii] = zeta_k;
							T_k_nuX[ii] = quadrotor.T_k;
						}
					}
					else
					{
						cout << "fMPC failed" << endl;
						trajectory_plan_success = false;
					}

				}
				else
				{
					if (ii == 1)
					{
						firstSuccess = false;
						iii = 0;
					}

					if (trajectory_plan_success)
					{
						for (int j = 0; j < mpc_params.T; j++)
						{
							eig_X.block(0,j,14,1) = prev_eig_X.block(0,mpc_params.T-1,quadrotor.n,1);
							eig_U.col(j) = prev_eig_U.block(0,mpc_params.T-1,quadrotor.m,1);
							// eig_U.col(j) = Eigen::MatrixXf::Zero(4,1);
						}
						eig_X.block(3,0,9,mpc_params.T) *= 0;
						// eig_X.block(13,0,1,mpc_params.T) *= 0;
					}	

					if (trajectory_plan_success)
					{

						if (traj_iterator+ii+iii >= localPath.rows())
						{
							iii = localPath.rows()-traj_iterator-ii-1;
						}
						else
						{

							while ( !( ( localPath.block(traj_iterator+ii+iii,0,1,3).transpose() - eig_X.block(0,mpc_params.T-1,3,1) ).norm() > 0.4 ) )
							{
								iii++;
								// if (traj_iterator+ii+iii >= plannedPath.rows())
								if (traj_iterator+ii+iii >= localPath.rows())
								{
									iii--;
									break;
								}
							}

						}

						shifted_traj_iterator = traj_iterator+ii+iii;
						// cout << "shifted_traj_iterator: " << shifted_traj_iterator << endl;
						// cout << "ii: " << ii << endl;
						// cout << "iii: " << iii << endl;

						if (shifted_traj_iterator < localPath.rows())
						{
							goal.block(0,0,3,1) = localPath.row(shifted_traj_iterator).transpose();
						 	goal(3,0) = atan2(goal(0,0)- localPath(shifted_traj_iterator-1,0), goal(1,0) - localPath(shifted_traj_iterator-1,1)); 		
							
						}
						else
						{
							// Find waypoint closest to the goal of the previous segment.
							float xdifftemp,ydifftemp,zdifftemp;
							float min_norm_temp = 99999;
							for(int i = 0; i < localPath.rows()-1; ++i)
							{

								xdifftemp = goal(0,0) - localPath(i,0);
								ydifftemp = goal(1,0) - localPath(i,1);
								zdifftemp = goal(2,0) - localPath(i,2);

								if( xdifftemp*xdifftemp + ydifftemp*ydifftemp + zdifftemp*zdifftemp < min_norm_temp )
								{

									min_norm_temp = xdifftemp*xdifftemp + ydifftemp*ydifftemp + zdifftemp*zdifftemp;
									shifted_traj_iterator = i;

								}
							}
							goal.block(0,0,3,1) = localPath.row(shifted_traj_iterator).transpose();
						 	goal(3,0) = atan2(goal(0,0)- localPath(shifted_traj_iterator-1,0), goal(1,0) - localPath(shifted_traj_iterator-1,1)); 		
		
						}

						// If the goal point is an obstacle (perhaps the planned path is old) then pick one that isn't
						if (obs.rows() > 0)
						{

							for (int i = 0; i < obs.rows(); i++)
							{
								if ( (obs.row(i)-goal.block(0,0,3,1).transpose()).norm() <= 0.2)
								{
									shifted_traj_iterator--;
									if (shifted_traj_iterator >= 0)
									{			
										goal(0,0) = localPath(shifted_traj_iterator,0);
										goal(1,0) = localPath(shifted_traj_iterator,1);
										goal(2,0) = localPath(shifted_traj_iterator,2);			
										// goal(3,0) = atan2(goal(0,0)- quadrotor.X0(0), goal(1,0) - quadrotor.X0(1)); 	
										goal(3,0) = atan2( goal(1,0) - quadrotor.X0(1), goal(0,0)- quadrotor.X0(0)); 	
										break;
									}
									else
									{
										goal(0,0) = localPath(0,0);
										goal(1,0) = localPath(0,1);
										goal(2,0) = localPath(0,2);	
										// goal(3,0) = atan2(goal(0,0)- quadrotor.X0(0), goal(1,0) - quadrotor.X0(1)); 	
										goal(3,0) = atan2( goal(1,0) - quadrotor.X0(1), goal(0,0)- quadrotor.X0(0)); 	
										break;
									}
								}
							}

						}

						segmentGoals(numTrajectoryPlans-1,4*ii) = goal(0,0);
						segmentGoals(numTrajectoryPlans-1,4*ii+1) = goal(1,0);
						segmentGoals(numTrajectoryPlans-1,4*ii+2) = goal(2,0);
						segmentGoals(numTrajectoryPlans-1,4*ii+3) = goal(3,0);
					
						temp_goal_pos = goal.block(0,0,3,1).transpose();

						bomt.X_goal(0,0) = goal(0,0); 
						bomt.X_goal(1,0) = goal(1,0); 
						bomt.X_goal(2,0) = goal(2,0); 
						bomt.X_goal(12,0) = goal(3,0); 

						if (!firstSuccess)
						{
							for (int j = 0; j < mpc_params.T; j++)
							{
								g_barrier(j,0) = (quadrotor.phi_max*quadrotor.phi_max)*(quadrotor.theta_max*quadrotor.theta_max)*quadrotor.mass*9.81*0.25;
							}

							r_obs = temp_goal_pos.transpose();
						}


						while(pthread_mutex_trylock(&map_lock))
						{
							usleep(1);
						}
						pthread_mutex_unlock(&map_lock);
						pthread_mutex_lock(&map_lock);	

							// Find the closest obstacle to the previous goal
							find_closest_obstacle(&temp_goal_pos,shifted_traj_iterator);

						pthread_mutex_unlock(&map_lock);
						
						// Must update weighting matrices to account for new r_obs
						// and discount the cost by a factor depending on the 
						// segment number
						update_and_discount_weighting_matrices(ii, 1);

						while(pthread_mutex_trylock(&constraint_lock))
						{
							usleep(1);
						}
						pthread_mutex_unlock(&constraint_lock);
						pthread_mutex_lock(&constraint_lock);

							// Update the constraints
							fmpc_hard_constraints(&eig_X,ii,shifted_traj_iterator,numFailures);
							fmpc_soft_constraints(&eig_X,ii,shifted_traj_iterator);

							// cout << "collisionConstraints: " << endl << collisionConstraints << endl;

						pthread_mutex_unlock(&constraint_lock);
					
					}
					else
					{
						update_weighting_matrices(ii,0);
					}

					// cout << "eig_X: " << endl;
					// cout << eig_X << endl;
					// cout << "eig_U: " << endl;
					// cout << eig_U << endl;
					// cout << "traj_iterator: " << traj_iterator <<endl;
					// cout << "start: " << quadrotor.X0.head(3).transpose() << endl;					
					// cout << "goal: " << goal << endl;
					// cout << "robs: " << r_obs.transpose() << endl;
					// cout << "eig_Fx_hard: " << endl << quadrotor.eig_Fx_hard << endl;
					// cout << "eig_x_hard_bounds: " << endl << quadrotor.eig_x_hard_bounds << endl;
					// cout << "eig_Fu_hard: " << endl << quadrotor.eig_Fu_hard << endl;
					// cout << "eig_u_hard_bounds: " << endl << quadrotor.eig_u_hard_bounds << endl;
					// cout << "g_barrier: " << endl << g_barrier << endl;

					// perform fmpc, check if it was successful
					intermediate_success = fmpcsolve(&eig_X, &eig_U, ii);
					cout << "fmpc success: " << intermediate_success << endl;

					// cout << "Fx: " << endl << quadrotor.eig_Fx_hard.block(4,12,2,1) << endl;
					// cout << "eig_x_pos: " << endl << eig_X.block(12,0,1,mpc_params.T) << endl;
					// cout << "FxX: " << endl << quadrotor.eig_Fx_hard.block(4,12,2,1)*eig_X.block(12,0,1,mpc_params.T) << endl;
					// cout << "fx: " << endl << quadrotor.eig_x_hard_bounds.block(4,0,2,mpc_params.T) << endl;

					if (intermediate_success)
					{

						cout << "fMPC success, checking g barrier" << endl;
						// Use eig_U to determine u1 and u1_dot
						compute_u1_u1dot(&eig_U);

						// posterior computation of roll, pitch, and thrust for a particular control policy,
						// and evaluate g_barrier
						g_barrier_violation = compute_roll_pitch_thrust_for_lambda_k_and_g_barrier(&eig_X, &eig_U);
						
						if (g_barrier_violation)
						{
							// cout << "g_barrier < 0, external constraints violated." << endl;
							// if violation, failure, adjust weighting matrices
							cout << "g barrier violated" << endl;						
							update_weighting_matrices(ii, 1);
							trajectory_plan_success = false;
						} 
						else
						{
							// if no violation, success, continue to next segment
							trajectory_plan_success = true;
							u_k_nuX[ii] = u_k; // u_k is the same as zeta_k
							v_k_nuX[ii] = v_k; 
							du1_k_nuX[ii] = du1_k;
							ddu1_k_nuX[ii] = ddu1_k;
							lambda_k_nuX[ii] = lambda_k;
							zeta_k_nuX[ii] = zeta_k;
							T_k_nuX[ii] = quadrotor.T_k;
							cout << "g barrier ok" << endl;
						}

					}
					else
					{
						trajectory_plan_success = false;
					}

				}	

				if (trajectory_plan_success)
				{

					if ((eig_X.block(0,mpc_params.T-1,3,1) - goal.block(0,0,3,1)).norm() > 1.0)
					{
						message = "Plan #" + to_string(numTrajectoryPlans) + " suspicious.";
						cout << message << endl;
						log_f_mpc.writeToLog(message);
					}

					objective_function_value(&eig_X, &eig_U, ii);
					firstSuccess = true;
					prev_eig_X = eig_X;
					prev_eig_U = eig_U;
					prev_seg_eig_X[ii] = eig_X;
					prev_seg_eig_U[ii] = eig_U;
					full_trajectory[ii] = eig_X;
					full_policy[ii] = eig_U;
					ii++;
					numFailures = 0;						
				}
				else
				{
					numFailures++;
					if (numFailures == 10)
					{	

						message = "Plan #" + to_string(numTrajectoryPlans) + " failed at " + to_string(ii) + " segment.";
						cout << message << endl;
						log_f_mpc.writeToLog(message);
						prev_eig_X = eig_X;
						prev_eig_U = eig_U;

						if (ii == 0)
						{

							for (int j = 0; j < mpc_params.T; j++)
							{
								eig_X.col(j) = quadrotor.X0;
								eig_X.block(3,0,9,j)*=0;  
								eig_X.block(13,0,1,j)*=0;  
								eig_U.col(j) = Eigen::MatrixXf::Zero(4,1);
							}

							for (int jj = ii; jj < mpc_params.nu_X; jj++)
							{
								prev_seg_eig_X[jj] = eig_X;
								prev_seg_eig_U[jj] = eig_U;
								full_trajectory[jj] = eig_X;
								full_policy[jj] = eig_U;
								segmentGoals(numTrajectoryPlans-1,4*jj) = goal(0,0);
								segmentGoals(numTrajectoryPlans-1,4*jj+1) = goal(1,0);
								segmentGoals(numTrajectoryPlans-1,4*jj+2) = goal(2,0);
								segmentGoals(numTrajectoryPlans-1,4*jj+3) = goal(3,0);

							}
							// cout << "eig_X: " << eig_X << endl;
							// cout << "eig_U: " << eig_U << endl;
							firstSuccess = true;
							ii = mpc_params.nu_X;

						}
						else
						{

							for (int j = 0; j < mpc_params.T; j++)
							{
								eig_X.col(j) = prev_seg_eig_X[ii-1].col(mpc_params.T-1);
								// eig_U.col(j) = prev_seg_eig_U[ii-1].col(mpc_params.T-1);
								// eig_X.col(j) = quadrotor.X0;  
								eig_U.col(j) = Eigen::MatrixXf::Zero(4,1);
							}

							eig_X.block(3,0,9,mpc_params.T)*= 0; 
							eig_X.block(13,0,1,mpc_params.T)*=0;

							for (int jj = ii; jj < mpc_params.nu_X; jj++)
							{
								prev_seg_eig_X[jj] = eig_X;
								prev_seg_eig_U[jj] = eig_U;
								full_trajectory[jj] = eig_X;
								full_policy[jj] = eig_U;
								segmentGoals(numTrajectoryPlans-1,4*jj) = goal(0,0);
								segmentGoals(numTrajectoryPlans-1,4*jj+1) = goal(1,0);
								segmentGoals(numTrajectoryPlans-1,4*jj+2) = goal(2,0);
								segmentGoals(numTrajectoryPlans-1,4*jj+3) = goal(3,0);
							}

						}

						// eig_X.block(2,0,1,mpc_params.T) = Eigen::MatrixXf::Ones(1,mpc_params.T)*quadrotor.X0(2);
						badTraj = 1;
						numFailures = 0;
						firstSuccess = true;
						ii = mpc_params.nu_X;
						// ii++;
					}
					else
					{
						
						message = "Plan #" + to_string(numTrajectoryPlans) + " failed " + to_string(numFailures) + " times at " + to_string(ii) + " segment.";
						cout << message << endl;
						log_f_mpc.writeToLog(message);

						if (ii == 0)
						{

							for (int j = 0; j < mpc_params.T; j++)
							{
								eig_X.col(j) = quadrotor.X0;  
								eig_U.col(j) = Eigen::MatrixXf::Zero(4,1);
							}

						}
						else
						{
							for (int j = 0; j < mpc_params.T; j++)
							{
								// eig_X.col(j) = prev_seg_eig_X[ii-1].col(j);  
								eig_X.col(j) = prev_seg_eig_X[ii-1].col(mpc_params.T-1);  
								// eig_U.col(j) = prev_seg_eig_U[ii-1].col(mpc_params.T-1);
								eig_U.col(j) = Eigen::MatrixXf::Zero(4,1);
							}

						}

						eig_X.block(3,0,9,mpc_params.T)*= 0; 
						eig_X.block(13,0,1,mpc_params.T)*=0;

					}
					
				}

			}

			firstPlanningEpisode = true;
			if (mpc_params.nu_sk < mpc_params.nu_X)
			{
				for (int i = mpc_params.nu_sk; i < mpc_params.nu_X; i++)
				{
					for (unsigned int jj = 0; jj < mpc_params.T; jj++)
					{
		
						full_trajectory[i].block(0,jj,3,1) = full_trajectory[mpc_params.nu_sk-1].block(0,mpc_params.T-1,3,1);
						full_trajectory[i].block(12,jj,1,1) = full_trajectory[mpc_params.nu_sk-1].block(12,mpc_params.T-1,1,1);
						
					}
				
					full_trajectory[i].block(3,0,9,mpc_params.T) = Eigen::MatrixXf::Zero(9,mpc_params.T);
					full_trajectory[i].block(13,0,1,mpc_params.T) = Eigen::MatrixXf::Zero(1,mpc_params.T);
					full_policy[i].block(0,0,quadrotor.m,mpc_params.T) = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
					u_k_nuX[i].block(0,0,1,mpc_params.T) = 9.81*Eigen::MatrixXf::Ones(1,mpc_params.T);
					u_k_nuX[i].block(1,0,quadrotor.m-1,mpc_params.T) = Eigen::MatrixXf::Zero(quadrotor.m-1,mpc_params.T);
					v_k_nuX[i].block(0,0,quadrotor.m,mpc_params.T) = Eigen::MatrixXf::Zero(quadrotor.m,mpc_params.T);
					du1_k_nuX[i].block(0,0,1,mpc_params.T) = Eigen::MatrixXf::Zero(1,mpc_params.T);
					ddu1_k_nuX[i].block(0,0,1,mpc_params.T) = Eigen::MatrixXf::Zero(1,mpc_params.T);
					lambda_k_nuX[i] = full_policy[i];
					zeta_k_nuX[i] = u_k_nuX[i];
					T_k_nuX[i] = 9.81*Eigen::MatrixXf::Ones(4,mpc_params.T)/4;
					// cout << "full_policy[i]: " << endl << full_policy[i] << endl; 
					// cout << "u_k_nuX[i]: " << endl << u_k_nuX[i] << endl; 
					// cout << "v_k_nuX[i]: " << endl << v_k_nuX[i] << endl; 
					// cout << "du1_k_nuX[i]: " << endl << du1_k_nuX[i] << endl; 
					// cout << "ddu1_k_nuX[i]: " << endl << ddu1_k_nuX[i] << endl; 
					// cout << "lambda_k_nuX[i]: " << endl << lambda_k_nuX[i] << endl; 
					// cout << "zeta_k_nuX[i]: " << endl << zeta_k_nuX[i] << endl; 
					// cout << "T_k_nuX[i]: " << endl << T_k_nuX[i] << endl; 
				}
	
				if (trajectory_plan_success)
				{

					for (int jj = mpc_params.nu_sk; jj < mpc_params.nu_X; jj++)
					{
						segmentGoals(numTrajectoryPlans-1,4*jj) = goal(0,0);
						segmentGoals(numTrajectoryPlans-1,4*jj+1) = goal(1,0);
						segmentGoals(numTrajectoryPlans-1,4*jj+2) = goal(2,0);
						segmentGoals(numTrajectoryPlans-1,4*jj+3) = goal(3,0);						
					}

				}

			}


			// cout << "Plan #: " << numTrajectoryPlans << " Pos: " << quadrotor.X0.head(3).transpose() << endl;
			// for (int i = 0; i < mpc_params.nu_X; i++)
			// {
			// 	cout << "Traj " << i << " - Goal: " << segmentGoals.block(numTrajectoryPlans-1,4*i,1,4) << endl << full_trajectory[i].block(0,0,3,mpc_params.T) << endl;
			// }
			// for (int i = 0; i < mpc_params.nu_X; i++)
			// {
			// 	cout << "Policy " << i << ": " << endl << full_policy[i].block(0,0,4,mpc_params.T) << endl;
			// }
			for (int i = 0; i < mpc_params.nu_X; i++)
			{

				if ((full_trajectory[i].block(0,0,3,mpc_params.T).array() < 0.0).any() || (full_trajectory[i].block(0,0,3,mpc_params.T).array() > 20.0).any())
				{
					badTraj = 1;
				}
			}

			// cout << "here" << endl;

			if (!badTraj)
			{
				prev_traj_iterator = traj_iterator;

				for (int i = 0; i < mpc_params.nu_X; i++)
				{
					prevfull_trajectory[i] = full_trajectory[i];
				}

				// Log the trajectory 
				log_f_mpc_data.writeToLog("Plan " + to_string(jjj));
				log_f_mpc_policy_data.writeToLog("Plan " + to_string(jjj));
				log_f_mpc_goal_data.writeToLog(to_string(get_time_u()) + ",",0);

				for (int ll = 0; ll < mpc_params.nu_X; ll++)
				{
				
					for (int kk = 0; kk < mpc_params.T; kk++)
					{

						for (int mm = 0; mm < quadrotor.n-1; mm++)
						{

							log_f_mpc_data.writeToLog(to_string(full_trajectory[ll](mm,kk)) + ",",0);

						}

						log_f_mpc_data.writeToLog(to_string(full_trajectory[ll](quadrotor.n-1,kk)) + ",",1);

						log_f_mpc_policy_data.writeToLog(to_string(u_k_nuX[ll](0,kk)) + ",",0);
						log_f_mpc_policy_data.writeToLog(to_string(du1_k_nuX[ll](0,kk)) + ",",0);
											
						for (int mm = 0; mm < quadrotor.m; mm++)
						{						
						
							log_f_mpc_policy_data.writeToLog(to_string(full_policy[ll](mm,kk)) + ",",0);
						
						}

						for (int mm = 0; mm < quadrotor.m; mm++)
						{
							log_f_mpc_policy_data.writeToLog(to_string(v_k_nuX[ll](mm,kk)) + ",",0);
						}

						for (int mm = 0; mm < quadrotor.m; mm++)
						{
							log_f_mpc_policy_data.writeToLog(to_string(zeta_k_nuX[ll](mm,kk)) + ",",0);
						}

						for (int mm = 0; mm < quadrotor.m-1; mm++)
						{
							log_f_mpc_policy_data.writeToLog(to_string(abs(T_k_nuX[ll](mm,kk))) + ",",0);
						}
						log_f_mpc_policy_data.writeToLog(to_string(abs(T_k_nuX[ll](quadrotor.m-1,kk))),1);


					}
					if (ll < mpc_params.nu_X - 1)
					{
						log_f_mpc_goal_data.writeToLog(to_string(segmentGoals(numTrajectoryPlans-1,4*ll)) + "," + to_string(segmentGoals(numTrajectoryPlans-1,4*ll+1)) + "," + to_string(segmentGoals(numTrajectoryPlans-1,4*ll+2)) + "," + to_string(segmentGoals(numTrajectoryPlans-1,4*ll+3)) + ",", 0);
					}
					else
					{
						log_f_mpc_goal_data.writeToLog(to_string(segmentGoals(numTrajectoryPlans-1,4*ll)) + "," + to_string(segmentGoals(numTrajectoryPlans-1,4*ll+1)) + "," + to_string(segmentGoals(numTrajectoryPlans-1,4*ll+2)) + "," + to_string(segmentGoals(numTrajectoryPlans-1,4*ll+3)), 1);
					}
				
				}
				
				jjj++;

			}
			else
			{
				for (int i = 0; i < mpc_params.nu_X; i++)
				{
					full_trajectory[i] = prevfull_trajectory[i];
				}
				badTraj = 0;
			}

			numTrajectoryPlans++;
			cout << "numTrajectoryPlans: " << numTrajectoryPlans << endl;

		pthread_mutex_unlock(&trajectory_lock);

		// cin.ignore();
		// exit(0);
	// }
	// else
	// {
	// 	cout << "Got to end of path!" << endl;
	// 	usleep(sleepTime);
	// }

	}


}


void F_MPC_UNCUT::objective_function_value(Eigen::MatrixXf* X, Eigen::MatrixXf* U, int segment_number)
{

	if (mpc_params.P[segment_number].cols() != quadrotor.n+quadrotor.m || mpc_params.P[segment_number].rows() > 1000)
	{
		cout << "Issue with the size of P at segment " << segment_number << ": " << mpc_params.P[segment_number].rows() << "x" << mpc_params.P[segment_number].cols() << endl;
		return;
	}

	Eigen::MatrixXf* tildeR = new Eigen::MatrixXf[mpc_params.T];
	Eigen::MatrixXf* tildeq = new Eigen::MatrixXf[mpc_params.T];
	Eigen::MatrixXf* Zk = new Eigen::MatrixXf[mpc_params.T];
	Eigen::MatrixXf goal_n = Eigen::MatrixXf::Zero(quadrotor.n,1);
	Eigen::MatrixXf ZkTRZk;
	Eigen::MatrixXf ZfkT_Rf_Zfk;
	Eigen::MatrixXf qTZk;
	Eigen::MatrixXf qfT_Zfk;
	Eigen::MatrixXf ZkTRZk_qTZk;
	Eigen::MatrixXf ZfkT_Rf_Zfk__qfT_Zfk;

	bool constraint_violation;

	Eigen::MatrixXf Obj_vec = Eigen::MatrixXf::Zero(mpc_params.T,1);
	// Eigen::MatrixXf Obj = Eigen::MatrixXf::Zero(1,1);
	Eigen::MatrixXf PhiZ = Eigen::MatrixXf::Zero(1,1);
	Eigen::MatrixXf ThetaZ = Eigen::MatrixXf::Zero(1,1);
	Eigen::MatrixXf kappaPhiZ = Eigen::MatrixXf::Zero(1,1);
	Eigen::MatrixXf rhoInverseThetaZ = Eigen::MatrixXf::Zero(1,1);
	Eigen::MatrixXf temp;
	temp = Eigen::MatrixXf::Zero(1,1);

	goal_n(0,0) = goal(0,0);
	goal_n(1,0) = goal(1,0);
	goal_n(2,0) = goal(2,0);
	goal_n(12,0) = goal(3,0);

	for (int i = 0; i < mpc_params.T-1; i++)
	{

		tildeR[i] = Eigen::MatrixXf::Zero(quadrotor.n+quadrotor.m,quadrotor.n+quadrotor.m);
		tildeq[i] = Eigen::MatrixXf::Zero(quadrotor.n+quadrotor.m,1);
		Zk[i] =  Eigen::MatrixXf::Zero(quadrotor.n+quadrotor.m,1);
		
		tildeR[i].block(0,0,quadrotor.n,quadrotor.n) = mpc_params.eig_R_rk[i]; 
		tildeR[i].block(0,quadrotor.n,quadrotor.n,quadrotor.m) = mpc_params.eig_R_r_lambda_k[i]; 
		tildeR[i].block(quadrotor.n,0,quadrotor.m,quadrotor.n) = mpc_params.eig_R_r_lambda_k[i].transpose();
		tildeR[i].block(quadrotor.n,quadrotor.n,quadrotor.m,quadrotor.m) = mpc_params.eig_R_lambda; 

		tildeq[i].block(0,0,quadrotor.n-2,1) = mpc_params.eig_q_rk[i];
		tildeq[i](12,0) = mpc_params.q_psi;
		tildeq[i](13,0) = 0;
		tildeq[i].block(quadrotor.n,0,quadrotor.m,1) = mpc_params.eig_q_lambda_k[i];

		Zk[i].block(0,0,quadrotor.n,1) = (*X).col(i) - goal_n;
		Zk[i].block(quadrotor.n,0,quadrotor.m,1) = (*U).col(i);

		for (int ii = 0; ii < mpc_params.h[segment_number].rows(); ii++)
		{
			temp = mpc_params.P[segment_number].row(ii)*(Zk[i]);
			if (mpc_params.h[segment_number](ii,i) - temp(0,0) <= 0)
			{
				PhiZ(0,0) += 1000000;
			}
			else
			{
				PhiZ(0,0) += -1*log(mpc_params.h[segment_number](ii,i) - temp(0,0) );
			}
			ThetaZ(0,0) += log(1 + exp(mpc_params.mu_13 * ( -mpc_params.h_tilde[segment_number](ii,i) + temp(0,0) ) ) );
		}
		
		ZkTRZk = Zk[i].transpose()*tildeR[i]*Zk[i];
		qTZk = tildeq[i].transpose()*Zk[i];
		ZkTRZk_qTZk = ZkTRZk + qTZk;
		Obj_vec.block(i,0,1,1) = ZkTRZk_qTZk;

	}
	
	kappaPhiZ(0,0) = mpc_params.mu_12*PhiZ(0,0); 
	rhoInverseThetaZ(0,0) = (1/mpc_params.mu_13)*ThetaZ(0,0);

	ZfkT_Rf_Zfk = ((*X).col(mpc_params.T-1) - goal_n).transpose() * mpc_params.eig_R_rf * ((*X).col(mpc_params.T-1) - goal_n);

	// cout << "Obj_vec.sum: " << Obj_vec.sum() << " term: " << ZfkT_Rf_Zfk(0,0) << " kappaPhiZ: " <<kappaPhiZ(0,0) << " rhoInverseThetaZ: " << rhoInverseThetaZ(0,0) << endl;

	Obj(0,0) += Obj_vec.sum() + ZfkT_Rf_Zfk(0,0) + kappaPhiZ(0,0) + rhoInverseThetaZ(0,0);

	// cout << "obj: " << Obj(0,0) << endl;

	delete[] tildeR;
	delete[] tildeq;
	delete[] Zk;

} 


