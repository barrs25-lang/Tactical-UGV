// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 16th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file implements the communication thread: it owns the socket CLIENT connection to the
// simulator/mapping server and exchanges pose, map, path, constraint, and trajectory data with the
// trajectory-planning thread.
// End File Decscription ################################################################################

#include <f_mpc_uncut.h>
#include <f_mpc_globals.h>
#include <my_client.h>

void* start_communication_thread(F_MPC_UNCUT* f_mpc_uncut)
{

	if( communication_status != 0 ){
		fprintf(stderr,"communication thread already running\n");
		return NULL;
	}
	else{
		f_mpc_uncut->communication_thread();
		return NULL;
	}
	return NULL;

}


void* start_comm_interface_thread(void *args)
{

	F_MPC_UNCUT* f_mpc_uncut = (F_MPC_UNCUT* )args;

	start_communication_thread(f_mpc_uncut);

	return NULL;

}


void F_MPC_UNCUT::communication_thread()
{
	
	communication_status = 1;

    char comm_message[30] = "Hi, this is traj.\n";
	string message;
	message = "<TRAJ-PLANNER-FMPC> Starting communication thread.";
	cout << message << endl;
	log_f_mpc.writeToLog(message);

	CLIENT client("/home/julius/git/darpa_mapping_sim_server_testing/comm_server/build/socket_parameters3.txt", &time_to_exit);

	bool constraintPass = false, posePass = false, mapPass = false, pathPass = false;

	int prev_in_constraint_size = 0;
	int prev_in_path_size = 0;
	char map[300000];
	int turns;
	float sign_of_heading;
	Eigen::MatrixXf tempcollisionConstraints = Eigen::MatrixXf::Zero(1,3);

	vector<float> obs_x, obs_y, obs_z;

	float map_freq = 20; // Hz
	float pose_freq = 30; // Hz
	float goal_freq = 10; // Hz
	float path_freq = 2; // Hz
	float constraint_freq = 5; // Hz
	float trajectory_freq = 1; // Hz

	ifstream file("../../comm_server/build/communication_parameters.txt");
		
	// String to store file lines
	string file_line;
	stringstream ss;
	
	//read communication parameters
	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> map_freq;	
	cout << "map_freq: " << map_freq << endl;

	//read communication parameters
	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> pose_freq;	
	cout << "pose_freq: " << pose_freq << endl;

	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> goal_freq;	
	cout << "goal_freq: " << goal_freq << endl;

	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> path_freq;	
	cout << "path_freq: " << path_freq << endl;

	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> constraint_freq;	
	cout << "constraint_freq: " << constraint_freq << endl;

	do{ss.clear(); getline(file, file_line); ss.str(file_line);}
	while(file_line.at(0) == '/' && file_line.at(1) == '/');
	ss >> trajectory_freq;	
	cout << "trajectory_freq: " << trajectory_freq << endl;	

	float map_time_us = 1000000/map_freq; // Microseconds	
	float pose_time_us = 1000000/(pose_freq+6); // Microseconds	
	float goal_time_us = 1000000/goal_freq; // Microseconds	
	float path_time_us = 1000000/(2+path_freq); // Microseconds	
	float constraint_time_us = 1000000/(4+constraint_freq); // Microseconds	
	float trajectory_time_us = 1000000/trajectory_freq; // Microseconds	

	auto current_time_map = std::chrono::high_resolution_clock::now();
	auto current_time_pose = std::chrono::high_resolution_clock::now();
	auto current_time_goal = std::chrono::high_resolution_clock::now();
	auto current_time_path = std::chrono::high_resolution_clock::now();
	auto current_time_constraints = std::chrono::high_resolution_clock::now();
	auto current_time_trajectory = std::chrono::high_resolution_clock::now();
	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed_map = end_time - current_time_map;	
	std::chrono::duration<double> timeElapsed_pose = end_time - current_time_pose;	
	std::chrono::duration<double> timeElapsed_goal = end_time - current_time_goal;	
	std::chrono::duration<double> timeElapsed_path = end_time - current_time_path;	
	std::chrono::duration<double> timeElapsed_constraints = end_time - current_time_constraints;	
	std::chrono::duration<double> timeElapsed_trajectory = end_time - current_time_trajectory;		

	// Pose communication vars
	std::vector<string> pose_recv;
	string pose_recv_string;
	char* pose_recv_buffer;
	pose_recv_buffer = new char[1000];

	// Path communication vars
	std::vector<string> path_recv;
	string path_recv_string;
	char* path_recv_buffer = new char[5000];	

	std::vector<string> constraints_recv;
	string constraints_recv_string;
	char* constraints_recv_buffer = new char[5000];	

	string trajectory_send;
	char* trajectory_send_buffer = new char[6000];
	char* cstr_trajectory = new char[6000];		

	int bytes_read_map = 0, bytes_read_pose = 0, bytes_read_goal = 0, bytes_read_path = 0, bytes_read_constraints = 0, bytes_read_trajectory = 0;
	int counter = 0, _interface_loops = 0;

	while(!time_to_exit)
	{

		// Get the "end" time
		end_time = std::chrono::high_resolution_clock::now();		

		/////////////////////////
		// POSE COMMUNICATIONS //
		/////////////////////////

		// if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_pose).count() >= 1.1*pose_time_us)
		// {
			// ioctl(client.sockets[2]->sockfd,TCFLSH,2);
			// current_time_pose = std::chrono::high_resolution_clock::now();
		// }
		// else if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_pose).count() >= pose_time_us)
		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_pose).count() >= pose_time_us)
		{	

			// cout << "timeElapsed_pose: " << timeElapsed_pose.count() << endl;

			// RECEIVE THE POSE //
			if (client.socket_active[2])
			{

				// cout << "recv pose" << endl; 
				bytes_read_pose = client.sockets[2]->process_receiving(pose_recv_buffer,1000*sizeof(char),1);
				pose_recv_string.clear();
				// pose_recv_string.resize(1001);
				pose_recv_string = pose_recv_buffer;
				// cout << "pose_recv_string: " << pose_recv_string << endl;
				counter = 0;
				pose_recv.clear();

				if (pose_recv_string[0] == 'Q')
				{

					for (int i = 1; i < 1000; i++)
					{
						if (pose_recv_string[i] == '!')
						{
							break;
						}
						else if (pose_recv_string[i] == ',')
						{
							counter++;
							pose_recv.resize(counter);
						}
						else
						{
							pose_recv[counter-1] += pose_recv_string[i];
						}
					}
				
					while(pthread_mutex_trylock(&pose_lock))
					{
						usleep(1);
					}
					pthread_mutex_unlock(&pose_lock);
					pthread_mutex_lock(&pose_lock);

						if (counter > 0)
						{						

							for (int i = 0; i < 18; i++)
							{
								pose[i] = boost::lexical_cast<float>(pose_recv[i]);
							}

							get_sign(sign_of_heading,pose[14]);
							turns = floor(pose[14]/((sign_of_heading)*M_PI));
							if (turns > 0)
							{
								pose[14] = pose[14] - sign_of_heading*M_PI*turns;
							}

							if (!posePass)
							{
								for (int i = 0; i < 18; i++)
								{
									initialPose[i] = pose[i];
								}
							}
		
							posePass = true;
						}

					pthread_mutex_unlock(&pose_lock);
				}				

			}
			
			current_time_pose = std::chrono::high_resolution_clock::now();

		}
		else
		{
			ioctl(client.sockets[2]->sockfd,TCFLSH,2);
			// tcflush(client.sockets[2]->sockfd,TCIFLUSH);
		}

		////////////////////////
		// MAP COMMUNICATIONS //
		////////////////////////

		// If enough time has passed
		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_map).count() >= map_time_us)
		{		
			// cout << "receiving map" << endl;
			// cout << "timeElapsed_map: " << timeElapsed_map.count() << endl;

			// RECEIVE THE MAP //
			if (client.socket_active[0])
			{

				counter = 0;

				bytes_read_map = client.sockets[0]->process_receiving(map,sizeof(map),0);
				// cout << "bytes_read_map: " << bytes_read_map << endl;
				// cout << "map[0]: " << +map[0] << endl;
				// if (bytes_read_map == sizeof(map))
				// {
					while(pthread_mutex_trylock(&map_lock))
					{
						usleep(1);
					}
					pthread_mutex_unlock(&map_lock);
					pthread_mutex_lock(&map_lock);
						
						obs_x.clear();
						obs_y.clear();
						obs_z.clear();
						for (int k = 0; k < grid_z; k++) 
						{
							for (int j = 0; j < grid_y; j++) 
							{
								for (int i = 0; i < grid_x; i++) 
								{
									voxel_map[i][j][k] = +map[counter];
									if (voxel_map[i][j][k] == 3)
									{
										obs_x.push_back(0.2*i);
										obs_y.push_back(0.2*k);
										obs_z.push_back(0.2*j);
									}
									counter++;

								}
							}
						}

						if (obs_x.size() > 0)
						{				
							obs = Eigen::MatrixXf::Zero(obs_x.size(),3);
							for (int i = 0; i < obs_x.size(); i++)
							{
								obs(i,0) = obs_x[i];
								obs(i,1) = obs_y[i];
								obs(i,2) = obs_z[i];
							}
						}
						
						mapPass = true;
						counter = 0;

					pthread_mutex_unlock(&map_lock);	
				// }

			} // if (client.socket_active[0])

			current_time_map = std::chrono::high_resolution_clock::now();

		} // if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_map).count() >= map_time_us)

		/////////////////////////
		// PATH COMMUNICATIONS //
		/////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_path).count() >= path_time_us)
		{

			// RECEIEVE THE PATH // 
			if (client.socket_active[6])
			{
				counter = 0;

				// cout << "Receiving the path" << endl;
				client.sockets[6]->process_receiving(path_recv_buffer,5000*sizeof(char),1);
				
				path_recv_string.clear();
				path_recv_string.resize(5000);
				path_recv_string = path_recv_buffer;

				path_recv.clear();

				if (path_recv_string[0] == 'P' && path_recv_string[2] != '!')
				{

					for (int i = 1; i < 5000; i++)
					{
						if (path_recv_string[i] == '!')
						{
							break;
						}
						else if (path_recv_string[i] == ',')
						{
							counter++;
							path_recv.resize(counter);
						}
						else
						{
							path_recv[counter-1] += path_recv_string[i];
						}
					}

				}

				// cout << "path: "; 
				// for (int i = 0; i < counter; i++)
				// {
				// 	cout << boost::lexical_cast<float>(path_recv[i]) << ",";
				// }
				// cout << endl;	

				while(pthread_mutex_trylock(&path_lock))
				{
					usleep(1);
				}
				pthread_mutex_unlock(&path_lock);
				pthread_mutex_lock(&path_lock);

					if (counter > 0)
					{

					plannedPath = Eigen::MatrixXf::Zero(path_recv.size()/3,3);
					for (int i = 0; i < path_recv.size()/3; i++)
					{
						plannedPath(i,0) = boost::lexical_cast<float>(path_recv[3*i]);
						plannedPath(i,1) = boost::lexical_cast<float>(path_recv[3*i+1]);
						plannedPath(i,2) = boost::lexical_cast<float>(path_recv[3*i+2]);
					}

						prev_in_path_size = path_recv.size()/3;

						pathPass = true;
						if (plannedPath.rows() > 0)
						{
							localPath = Eigen::MatrixXf::Zero(plannedPath.rows(),3);
							localPath = plannedPath;
						}
						else
						{
							if (!firstPassComplete)
							{
								localPath = Eigen::MatrixXf::Zero(1,3);
								localPath(0,0) = boost::lexical_cast<float>(pose_recv[0]); 
								localPath(0,1) = boost::lexical_cast<float>(pose_recv[1]); 
								localPath(0,2) = boost::lexical_cast<float>(pose_recv[2]); 
								pathPass = false;
							}
						}

					}

				pthread_mutex_unlock(&path_lock);

				counter = 0;

			}
			
			current_time_path = std::chrono::high_resolution_clock::now();

		}

		////////////////////////////////
		// CONSTRAINTS COMMUNICATIONS //
		////////////////////////////////	
		
		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_constraints).count() >= constraint_time_us)
		{
			
			// RECEIEVE THE CONSTRAINTS // 
			if (client.socket_active[8])
			{
				counter = 0;
				constraints_recv.clear();

				// cout << "Receiving the constraints" << endl;
				client.sockets[8]->process_receiving(constraints_recv_buffer,5000*sizeof(char),1);
				constraints_recv_string = constraints_recv_buffer;
				// cout << "constraints_recv_string: " << endl << constraints_recv_string<<endl;

				if (constraints_recv_string[0] == 'C' && constraints_recv_string[2] != '!')
				{

					for (int i = 1; i < 5000; i++)
					{
						if (constraints_recv_string[i] == '!')
						{
							break;
						}
						else if (constraints_recv_string[i] == ',')
						{
							counter++;
							constraints_recv.resize(counter);
						}
						else
						{
							constraints_recv[counter-1] += constraints_recv_string[i];
						}
					}
	
					// cout << "constraints: "; 
					// for (int i = 0; i < counter; i++)
					// {
					// 	cout << boost::lexical_cast<float>(constraints_recv[i]) << ",";
					// }
					// cout << endl;	
			
					while(pthread_mutex_trylock(&constraint_lock))
					{
						usleep(1);
					} // while(pthread_mutex_trylock(&constraint_lock))
					pthread_mutex_unlock(&constraint_lock);
					pthread_mutex_lock(&constraint_lock);

						if (counter > 0)
						{

							Eigen::MatrixXf tempcollisionConstraints = Eigen::MatrixXf::Zero(constraints_recv.size()/4,4);
							collisionConstraints = Eigen::MatrixXf::Zero(constraints_recv.size()/4,4);
							// cout << "collisionConstraints.rows:"  << collisionConstraints.rows() << endl;
							// cout << "counter: " << counter << endl;
							// int j = -1;
							// Iterate over the number of constraints
							for (int i = 0; i < constraints_recv.size()/4; i++)
							{						
								// if (boost::lexical_cast<float>(constraints_recv[4*i+2]) < 0)
								// {
								// 	continue;
								// }
								// j++;
								// cout << "constraints_recv[" << 4*i << "]: " << constraints_recv[4*i] << endl;
								tempcollisionConstraints(i,0) = boost::lexical_cast<float>(constraints_recv[4*i]);
								// cout << "constraints_recv[" << 4*i+1 << "]: " << constraints_recv[4*i+1] << endl;
								tempcollisionConstraints(i,1) = boost::lexical_cast<float>(constraints_recv[4*i+1]);
								// cout << "constraints_recv[" << 4*i+2 << "]: " << constraints_recv[4*i+2] << endl;
								tempcollisionConstraints(i,2) = boost::lexical_cast<float>(constraints_recv[4*i+2]);
								tempcollisionConstraints(i,3) = 20+boost::lexical_cast<float>(constraints_recv[4*i+3]);
							}

							if (tempcollisionConstraints.norm() != 0)
							{
								collisionConstraints = tempcollisionConstraints;
							}


						}
						constraintPass = true;

					pthread_mutex_unlock(&constraint_lock);
			
					counter = 0;

				}


			}
			
			current_time_constraints = std::chrono::high_resolution_clock::now();

		}

		///////////////////////////////
		// TRAJECTORY COMMUNICATIONS //
		///////////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_trajectory).count() >= trajectory_time_us)
		{

			// SEND THE CONSTRAINTS // 
			if (client.socket_active[11])
			{
				
				// Make some test data and append to string buffer
				trajectory_send.clear();
				trajectory_send.append("T");
				trajectory_send.append(",");
				while(pthread_mutex_trylock(&trajectory_lock))
				{
					usleep(1);
				}
				pthread_mutex_unlock(&trajectory_lock);
				pthread_mutex_lock(&trajectory_lock);				
					for (int j = 0; j < mpc_params.nu_X; j++)
					{
						for (int i = 0; i < mpc_params.T; i++)
						{
							for (int k = 0; k < quadrotor.n; k++)
							{					
			
								float temp = full_trajectory[j](k,i);
								// full_trajectory_interf[j](k,i) = full_trajectory[j](k,i);
								trajectory_send.append( boost::lexical_cast<string>(temp) );
								if (j == mpc_params.nu_X-1 && i == mpc_params.T-1 && k == quadrotor.n-1)
								{
									trajectory_send.append("!");
								}
								else
								{
									trajectory_send.append(",");
								}
							}
						}
					}
				pthread_mutex_unlock(&trajectory_lock);

				// cout << "trajectory_send: " << trajectory_send << endl;

				// Copy the string to a char buffer
				// cstr_trajectory = new char[1000];
				strcpy(cstr_trajectory, trajectory_send.c_str());
				memcpy(trajectory_send_buffer, cstr_trajectory, strlen(cstr_trajectory)+1);

				// Send the path char buffer					
				client.sockets[11]->process_sending(trajectory_send_buffer,6000*sizeof(char));

			}
			
			current_time_trajectory = std::chrono::high_resolution_clock::now();

		}


		if (constraintPass && pathPass && mapPass && posePass)
		{
			firstPassComplete = true;
		}

		// Compute how much time has passed
		timeElapsed_map = end_time - current_time_map;
		timeElapsed_pose = end_time - current_time_pose;
		timeElapsed_path = end_time - current_time_path;
		timeElapsed_constraints = end_time - current_time_constraints;
		timeElapsed_trajectory = end_time - current_time_trajectory;

		// if (!(_interface_loops % 400))
		// {

		// 	cout << "\033[2J\033[1;1H";
		// 	cout << "Position: " << std::fixed << std::setprecision(5) << -pose[0] << ", " << -pose[1] << ", " << -pose[2] << " [m] Heading: " << pose[14] << " [rad]" << endl;
		// 	for (int i = 0; i < 6; i++)
		// 	{
		// 		// cout << "full_trajectory_interf " << i << " size: " << full_trajectory_interf[i].rows() << "x" << full_trajectory_interf[i].cols() << endl; 
		// 		if (segmentGoals.rows() > 0)
		// 		{
		// 			cout << "Traj " << std::fixed << std::setprecision(5) << i << " - Goal: " << segmentGoals.block(numTrajectoryPlans-1,4*i,1,4) << endl << full_trajectory_interf[i].block(0,0,3,5) << endl;
		// 		}
		// 		else
		// 		{
		// 			cout << "Traj " << std::fixed << std::setprecision(5) << i << endl << full_trajectory_interf[i].block(0,0,3,5) << endl;
		// 		}
		// 	}
		// 	cout << "Timers:   " << "  MAP  " << "  POS  " << "  PAT  " << "  CON  " << "  TRA  " << endl;
		// 	cout << "Time [s]: " << std::fixed << std::setprecision(5) << timeElapsed_map.count() << " " << timeElapsed_pose.count() << " " << timeElapsed_path.count() << " " << timeElapsed_constraints.count() << " " << timeElapsed_trajectory.count() << endl;

		// }


	}

}

