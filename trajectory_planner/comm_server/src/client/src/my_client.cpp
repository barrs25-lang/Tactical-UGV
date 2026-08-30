// ---------

// stuff

// ---------

#include <my_client.h>

bool time_to_exit = 0;

CLIENT* kill_client;

void quit_handler(int sig)
{

	if( sig == SIGINT )
	{
		cout << "User interrupt." << endl;
	}
	else if ( sig == SIGSEGV )
	{
		cout << "Segmentation fault." << endl;
	}
	else if ( sig == SIGABRT)
	{
		cout << "Aborted." << endl;
	}

	time_to_exit = 1;

	kill_client->~CLIENT();

	exit(0);

}

int main(int argc, char** argv)
{

	signal(SIGPIPE, quit_handler); // Ignore disconnected sockets
	signal(SIGINT, quit_handler); 
	signal(SIGSEGV, quit_handler);
	signal(SIGABRT, quit_handler);

	CLIENT client(argv[1]);
	kill_client = &client;
	
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

	char message[30];
	char map[300000];

	// Pose communication vars
	string pose_send;
	float pose[18];
	std::vector<string> pose_recv;
	string pose_recv_string;
	char* pose_recv_buffer;
	pose_recv_buffer = new char[1000];
	char* pose_send_buffer;
	pose_send_buffer = new char[1000];	
	char* cstr_pose;
	cstr_pose = new char[1000];

	// Goal communication vars
	string goal_send;
	float goal[3];
	std::vector<string> goal_recv;
	string goal_recv_string;
	char* goal_recv_buffer;
	goal_recv_buffer = new char[100];
	char* goal_send_buffer;
	goal_send_buffer = new char[100];	
	char* cstr_goal;
	cstr_goal = new char[100];

	// Path communication vars
	// Path does not have consistent size,
	// so we have to use a few more variables to
	// properly send the path.
	string path_send;
	std::vector<string> path_recv;
	string path_recv_string;
	char* path_recv_buffer;
	path_recv_buffer = new char[1000];
	char* path_send_buffer;
	path_send_buffer = new char[1000];
	char* cstr_path;
	cstr_path = new char[1000];

	// Constraint communication vars
	// Constraint set does not have consistent size,
	// so we have to use a few more variables to
	// properly send the Constraints.
	string constraints_send;
	std::vector<string> constraints_recv;
	string constraints_recv_string;
	char* constraints_recv_buffer;
	constraints_recv_buffer = new char[1000];
	char* constraints_send_buffer;
	constraints_send_buffer = new char[1000];
	char* cstr_constraints;
	cstr_constraints = new char[1000];

	// Trajectory communication vars
	float** trajectory;
	string trajectory_send;
	std::vector<string> trajectory_recv;
	string trajectory_recv_string;
	char* trajectory_recv_buffer;
	trajectory_recv_buffer = new char[1000];
	char* trajectory_send_buffer;
	trajectory_send_buffer = new char[1000];
	char* cstr_trajectory;
	cstr_trajectory = new char[1000];	

	for (int i = 0; i < 30; i++)
	{
		message[i] = 0;
	}

	for (int i = 0; i < 18; i++)
	{
		pose[i] = 0.0;
	}

	float map_freq = 20; // Hz
	float pose_freq = 30; // Hz
	float goal_freq = 10; // Hz
	float path_freq = 2; // Hz
	float constraint_freq = 5; // Hz
	float trajectory_freq = 1; // Hz

	float map_time_us = 1000000/map_freq; // Microseconds	
	float pose_time_us = 1000000/pose_freq; // Microseconds	
	float goal_time_us = 1000000/goal_freq; // Microseconds	
	float path_time_us = 1000000/path_freq; // Microseconds	
	float constraint_time_us = 1000000/constraint_freq; // Microseconds	
	float trajectory_time_us = 1000000/trajectory_freq; // Microseconds	

	int bytes_read_map = 0, bytes_read_pose = 0, bytes_read_goal = 0, bytes_read_path = 0, bytes_read_constraints = 0, bytes_read_trajectory = 0;
	int counter = 0;

	while(!time_to_exit)
	{	

		// Get the "end" time
		end_time = std::chrono::high_resolution_clock::now();

		////////////////////////
		// MAP COMMUNICATIONS //
		////////////////////////

		// If enough time has passed
		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_map).count() >= map_time_us)
		{		
			cout << "receiving map" << endl;
			cout << "timeElapsed_map: " << timeElapsed_map.count() << endl;
			map[0] = 0;
			map[299998] = 0;

			// RECEIVE THE MAP //
			if (client.socket_active[0])
			{
				bytes_read_map = client.sockets[0]->process_receiving(map,sizeof(map),0);
				cout << "bytes_read_map: " << bytes_read_map << endl;
			}

			current_time_map = std::chrono::high_resolution_clock::now();
			cout << "map: " << +map[0] << "," << +map[299998] << endl;
			map[0] = 100;
			map[299998] = 100;

			// SEND THE MAP //
			if (client.socket_active[1])
			{
				client.sockets[1]->process_sending(map,sizeof(map));
			}
		}

		/////////////////////////
		// POSE COMMUNICATIONS //
		/////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_pose).count() >= pose_time_us)
		{	

			// SEND THE POSE //
			if (client.socket_active[3])
			{
				pose[0] = 10.5;
				pose_send.clear();
				pose_send.append("Q");
				pose_send.append(",");
				for (int i = 0; i < 17; i++)
				{
					pose_send.append(boost::lexical_cast<string>(pose[i]));
					pose_send.append(",");
				}
				pose_send.append(boost::lexical_cast<string>(pose[17]));
				pose_send.append("!");

				cstr_pose = new char[1000];
				strcpy(cstr_pose, pose_send.c_str());
				memcpy(pose_send_buffer, cstr_pose, strlen(cstr_pose)+1);

				client.sockets[3]->process_sending(pose_send_buffer,1000*sizeof(char));
			}

			// RECEIVE THE POSE //
			if (client.socket_active[2])
			{

				bytes_read_pose = client.sockets[2]->process_receiving(pose_recv_buffer,1000*sizeof(char),1);
				if (bytes_read_pose == 1000*sizeof(char))
				{

					pose_recv_string = pose_recv_buffer;
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

						cout << "pose: "; 
						for (int i = 0; i < 18; i++)
						{
							cout <<  boost::lexical_cast<float>(pose_recv[i]) << ",";
						}
						cout << endl;

					}

				}

				counter = 0;

			}			

		}

		/////////////////////////
		// GOAL COMMUNICATIONS //
		/////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_goal).count() >= goal_time_us)
		{	

			// SEND THE GOAL //
			if (client.socket_active[5])
			{
				goal[0] = 1;
				goal_send.clear();
				goal_send.append("G");
				goal_send.append(",");
				for (int i = 0; i < 2; i++)
				{
					goal_send.append(boost::lexical_cast<string>(goal[i]));
					goal_send.append(",");
				}
				goal_send.append(boost::lexical_cast<string>(goal[2]));
				goal_send.append("!");

				cstr_goal = new char[100];
				strcpy(cstr_goal, goal_send.c_str());
				memcpy(goal_send_buffer, cstr_goal, strlen(cstr_goal)+1);

				string temp(goal_send_buffer);

				client.sockets[5]->process_sending(goal_send_buffer,100*sizeof(char));
			}

			// RECEIVE THE GOAL //
			if (client.socket_active[4])
			{
				// cout << "receiving the pose" << endl;
				bytes_read_goal = client.sockets[4]->process_receiving(goal_recv_buffer,100*sizeof(char),1);
				
				if (bytes_read_goal == 100*sizeof(char))
				{

					goal_recv_string = goal_recv_buffer;
					
					counter = 0;
					goal_recv.clear();

					if (goal_recv_string[0] == 'G')
					{

						for (int i = 1; i < 100; i++)
						{
							if (goal_recv_string[i] == '!')
							{
								break;
							}
							else if (goal_recv_string[i] == ',')
							{
								counter++;
								goal_recv.resize(counter);
							}
							else
							{
								goal_recv[counter-1] += goal_recv_string[i];
							}
						}

					}
	
					cout << "goal: "; 
					for (int i = 0; i < 3; i++)
					{
						cout << boost::lexical_cast<float>(goal_recv[i]) << ",";
					}
					cout << endl;
	
				}

				
			}
			
		}

		/////////////////////////
		// PATH COMMUNICATIONS //
		/////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_path).count() >= path_time_us)
		{
			

			// RECEIEVE THE PATH // 
			if (client.socket_active[6])
			{
				counter = 0;

				cout << "Receiving the path" << endl;
				client.sockets[6]->process_receiving(path_recv_buffer,1000*sizeof(char),1);
				path_recv_string = path_recv_buffer;
				if (path_recv_string[0] == 'P')
				{

					for (int i = 1; i < 1000; i++)
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

				cout << "path: "; 
				for (int i = 0; i < counter; i++)
				{
					cout << boost::lexical_cast<float>(path_recv[i]) << ",";
				}
				cout << endl;	

				counter = 0;

			}

			// SEND THE PATH // 
			if (client.socket_active[7])
			{
				
				// Make some test data and append to string buffer
				int pathsize = 4*3;
				path_send.clear();
				path_send.append("P");
				path_send.append(",");
				for (int i = 0; i < pathsize-1; i++)
				{
					float temp = 0.5*i;
					path_send.append( boost::lexical_cast<string>(temp) );
					path_send.append(",");
				}
				path_send.append( boost::lexical_cast<string>(pathsize-1*0.5) );
				path_send.append("!");

				cout << "path_send: " << path_send << endl;

				// Copy the string to a char buffer
				cstr_path = new char[1000];
				strcpy(cstr_path, path_send.c_str());
				memcpy(path_send_buffer, cstr_path, strlen(cstr_path)+1);

				// Send the path char buffer					
				client.sockets[7]->process_sending(path_send_buffer,1000*sizeof(char));

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

				cout << "Receiving the constraints" << endl;
				client.sockets[8]->process_receiving(constraints_recv_buffer,1000*sizeof(char),1);
				constraints_recv_string = constraints_recv_buffer;

				if (constraints_recv_string[0] == 'C')
				{

					for (int i = 1; i < 1000; i++)
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
	
					cout << "constraints: "; 
					for (int i = 0; i < counter; i++)
					{
						cout << boost::lexical_cast<float>(constraints_recv[i]) << ",";
					}
					cout << endl;	

					counter = 0;

				}


			}

			// SEND THE CONSTRAINTS // 
			if (client.socket_active[9])
			{
				
				// Make some test data and append to string buffer
				int constraintsize = 32*4;
				constraints_send.clear();
				constraints_send.append("C");
				constraints_send.append(",");
				for (int i = 0; i < constraintsize-1; i++)
				{
					float temp0 = 0.5*i;
					constraints_send.append( boost::lexical_cast<string>(temp0) );
					constraints_send.append(",");
				}
				constraints_send.append( boost::lexical_cast<string>(constraintsize-1*0.5) );
				constraints_send.append("!");

				cout << "constraints_send: " << constraints_send << endl;

				// Copy the string to a char buffer
				cstr_constraints = new char[1000];
				strcpy(cstr_constraints, constraints_send.c_str());
				memcpy(constraints_send_buffer, cstr_constraints, strlen(cstr_constraints)+1);

				// Send the path char buffer					
				client.sockets[9]->process_sending(constraints_send_buffer,1000*sizeof(char));

			}
			
			current_time_constraints = std::chrono::high_resolution_clock::now();

		}

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_trajectory).count() >= trajectory_time_us)
		{
			

			// RECEIEVE THE TRAJECTORY // 
			if (client.socket_active[10])
			{
				counter = 0;

				cout << "Receiving the trajectory" << endl;
				client.sockets[10]->process_receiving(trajectory_recv_buffer,1000*sizeof(char),1);
				trajectory_recv_string = trajectory_recv_buffer;

				if (trajectory_recv_string[0] == 'T')
				{

					for (int i = 1; i < 1000; i++)
					{
						if (trajectory_recv_string[i] == '!')
						{
							break;
						}
						else if (trajectory_recv_string[i] == ',')
						{
							counter++;
							trajectory_recv.resize(counter);
						}
						else
						{
							trajectory_recv[counter-1] += trajectory_recv_string[i];
						}
					}
	
					cout << "trajectory: "; 
					for (int i = 0; i < counter; i++)
					{
						cout << boost::lexical_cast<float>(trajectory_recv[i]) << ",";
					}
					cout << endl;	

					counter = 0;

				}


			}

			// SEND THE CONSTRAINTS // 
			if (client.socket_active[11])
			{
				
				// Make some test data and append to string buffer
				int trajectorysize = 14*5;
				trajectory_send.clear();
				trajectory_send.append("T");
				trajectory_send.append(",");
				for (int i = 0; i < trajectorysize-1; i++)
				{
					float temp1 = 0.5*i;
					trajectory_send.append( boost::lexical_cast<string>(temp1) );
					trajectory_send.append(",");
				}
				trajectory_send.append( boost::lexical_cast<string>(trajectorysize-1*0.5) );
				trajectory_send.append("!");

				cout << "trajectory_send: " << trajectory_send << endl;

				// Copy the string to a char buffer
				cstr_trajectory = new char[1000];
				strcpy(cstr_trajectory, trajectory_send.c_str());
				memcpy(trajectory_send_buffer, cstr_trajectory, strlen(cstr_trajectory)+1);

				// Send the path char buffer					
				client.sockets[11]->process_sending(trajectory_send_buffer,1000*sizeof(char));

			}
			
			current_time_trajectory = std::chrono::high_resolution_clock::now();

		}

		// Compute how much time has passed
		timeElapsed_map = end_time - current_time_map;
		timeElapsed_pose = end_time - current_time_pose;
		timeElapsed_goal = end_time - current_time_goal;
		timeElapsed_path = end_time - current_time_path;
		timeElapsed_constraints = end_time - current_time_constraints;
		timeElapsed_trajectory = end_time - current_time_trajectory;

	}

}