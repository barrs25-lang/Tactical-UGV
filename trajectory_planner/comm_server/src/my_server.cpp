// ----------

// stuff

// ----------


#include <my_server.h>

bool time_to_exit = 0;
// SERVER* kill_server;

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

	usleep(1000000);

	// kill_server->~SERVER();

	exit(0);

}

int main(int argc, char** argv)
{

	signal(SIGPIPE, quit_handler); // Ignore disconnected sockets
	signal(SIGINT, quit_handler); 
	signal(SIGSEGV, quit_handler);
	signal(SIGABRT, quit_handler);

	// Argument 0 is the name of the executable
	// Argument 1 is the number of clients
	// Argument 2 is the name of the communication parameter file
	if (argc < 3)
	{
		cout << "Usage: " << argv[0] << " <num_clients> <communication_parameter_file>" << endl;
		exit(1);
	}

	if (argv[1][0] < '0' || argv[1][0] > '9')
	{
		cout << "Error: <num_clients> must be a digit, got \"" << argv[1] << "\"" << endl;
		exit(1);
	}

	SERVER server( (*argv[1]) - '0' );
	// kill_server = &server;

	char message[30];
	char map[300000];

	// Pose communication vars
	string pose_send;
	float pose[18];
	string pose_recv_string;
	std::vector<string> pose_recv;
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

	// Path does not have consistent size,
	// so we have to use a few more variables to
	// properly send the path.
	std::vector<string> path_recv;
	string path_recv_string;
	char* path_recv_buffer;
	path_recv_buffer = new char[5000];
	char* path_send_buffer;
	path_send_buffer = new char[5000];
	char* cstr_path;
	cstr_path = new char[5000];

	// Constraint communication vars
	// Constraint set does not have consistent size,
	// so we have to use a few more variables to
	// properly send the Constraints.
	float** constraints;
	string constraints_send;
	std::vector<string> constraints_recv;
	string constraints_recv_string;
	char* constraints_recv_buffer;
	constraints_recv_buffer = new char[5000];
	char* constraints_send_buffer;
	constraints_send_buffer = new char[5000];
	char* cstr_constraints;
	cstr_constraints = new char[5000];	

	// Trajectory communication vars
	float** trajectory;
	string trajectory_send;
	std::vector<string> trajectory_recv;
	string trajectory_recv_string;
	char* trajectory_recv_buffer;
	trajectory_recv_buffer = new char[6000];
	char* trajectory_send_buffer;
	trajectory_send_buffer = new char[6000];
	char* cstr_trajectory;
	cstr_trajectory = new char[6000];	

	map[0] = 127;
	map[299998] = 127;
	int j = 0;

	float map_freq = 20; // Hz
	float pose_freq = 30; // Hz
	float goal_freq = 2; // Hz
	float path_freq = 10; // Hz
	float constraint_freq = 3; // Hz
	float trajectory_freq = 5; // Hz
	
	ifstream file(argv[2]);
		
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
	float pose_time_us = 1000000/pose_freq; // Microseconds
	float goal_time_us = 1000000/goal_freq; // Microseconds
	float path_time_us = 1000000/path_freq; // Microseconds
	float constraint_time_us = 1000000/constraint_freq; // Microseconds
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

	int bytes_recv_path = 0, bytes_recv_constraints = 0, bytes_recv_trajectory = 0;
	int counter = 0;

	while(!time_to_exit)
	{

		// Get the "end" time
		end_time = std::chrono::high_resolution_clock::now();
		
		// Compute how much time has passed
		timeElapsed_map = end_time - current_time_map;
		timeElapsed_pose = end_time - current_time_pose;
		timeElapsed_goal = end_time - current_time_goal;
		timeElapsed_path = end_time - current_time_path;
		timeElapsed_constraints = end_time - current_time_constraints;
		timeElapsed_trajectory = end_time - current_time_trajectory;

		////////////////////////
		// MAP COMMUNICATIONS //
		////////////////////////

		// If enough time has passed
		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_map).count() >= map_time_us)
		{
	
			// cout << "timeElapsed_map: " << timeElapsed_map.count() << endl;
			// cout << "Recv map" << endl;
			// RECEIEVE THE MAP // 
			for (int j = 0; j < server.num_clients; j++)
			{
				if (server.socket_active[j][1])
				{
					server.sockets[server.num_sockets*j + 1]->process_receiving(map,sizeof(map),0);
				}
			}

			// cout << "map[0]: " << +map[0] << endl;
			int obs = 0;
			for (int i = 0; i < 300000; i++)
			{
				if(+map[i])
				{
					obs++;
				}
			}
			// cout << "obs: " << obs << endl;


			// SEND THE MAP // 
			for (int j = 0; j < server.num_clients; j++)
			{
				if (server.socket_active[j][0])
				{
					server.sockets[server.num_sockets*j]->process_sending(map,300000*sizeof(char),1);
				}
			}
			// cout << "Sent the map" << endl;


			// Reset the current time
			current_time_map = std::chrono::high_resolution_clock::now();
			
		}

		/////////////////////////
		// POSE COMMUNICATIONS //
		/////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_pose).count() >= pose_time_us)
		{

			//cout << "timeElapsed_pose: " << timeElapsed_pose.count() << endl;			

			// RECEIEVE THE POSE // 
			for (int j = 0; j < server.num_clients; j++)
			{
				if (server.socket_active[j][3])
				{
					server.sockets[server.num_sockets*j + 3]->process_receiving(pose_recv_buffer,1000*sizeof(char),0);
					pose_recv_string.resize(1000);
					pose_recv_string = pose_recv_buffer;
					// cout << "Recv pose" << endl;
					// cout << pose_recv_string << endl;

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

					}

					// cout << "pose: "; 
					// for (int i = 0; i < 18; i++)
					// {
					// 	cout <<  boost::lexical_cast<float>(pose_recv[i]) << ",";
					// }
					// cout << endl;	

					counter = 0;

				}
			}

			// SEND THE POSE // 
			for (int j = 0; j < server.num_clients; j++)
			{
				if (server.socket_active[j][2])
				{

					// cstr_pose = new char[1000];
					strcpy(cstr_pose, pose_recv_string.c_str());
					memcpy(pose_send_buffer, cstr_pose, strlen(cstr_pose)+1);
					// cout << "pose_send_buffer: " << pose_send_buffer << endl;
					// cout << "Sending the pose" << endl;
					server.sockets[server.num_sockets*j + 2]->process_sending(pose_send_buffer,1000*sizeof(char),0);
				}
			}

			current_time_pose = std::chrono::high_resolution_clock::now();

		}
		
		/////////////////////////
		// GOAL COMMUNICATIONS //
		/////////////////////////		

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_goal).count() >= goal_time_us)
		{
			
			// cout << "timeElapsed_goal: " << timeElapsed_goal.count() << endl;
			
			// RECEIEVE THE GOAL // 
			for (int j = 0; j < server.num_clients; j++)
			{
				if (server.socket_active[j][5])
				{
					// cout << "Receiving the goal" << endl;
					server.sockets[server.num_sockets*j + 5]->process_receiving(goal_recv_buffer,100*sizeof(char),1);
					
					goal_recv_string.clear();
					goal_recv_string.resize(101);
					goal_recv_string = goal_recv_buffer;
					// cout << goal_recv_string << endl;
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

					// cout << "goal: "; 
					// for (int i = 0; i < 3; i++)
					// {
					// 	cout << boost::lexical_cast<float>(goal_recv[i]) << ",";
					// }
					// cout << endl;

					counter = 0;

				}
			}

			// SEND THE GOAL // 
			for (int j = 0; j < server.num_clients; j++)
			{
				if (server.socket_active[j][4])
				{					
					// cout << "Sending the goal" << endl;
					// cstr_goal = new char[100];
					strcpy(cstr_goal, goal_recv_string.c_str());
					memcpy(goal_send_buffer, cstr_goal, strlen(cstr_goal)+1);

					server.sockets[server.num_sockets*j + 4]->process_sending(goal_send_buffer,100*sizeof(char),0);
				}
			}

			current_time_goal = std::chrono::high_resolution_clock::now();

		}	

		/////////////////////////
		// PATH COMMUNICATIONS //
		/////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_path).count() >= path_time_us)
		{

			// cout << "timeElapsed_path: " << timeElapsed_path.count() << endl;
			// RECEIEVE THE PATH // 
			for (int j = 0; j < server.num_clients; j++)
			{

				if (server.socket_active[j][7])
				{
					// cout << "recv path" << endl;

					bytes_recv_path = server.sockets[server.num_sockets*j + 7]->process_receiving(path_recv_buffer,5000*sizeof(char),1);
					
					path_recv_string.clear();
					path_recv_string.resize(5000);
					path_recv_string = path_recv_buffer;
					// cout << path_recv_string << endl;
					
					counter = 0;
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

					// cout << "path_recv: " << path_recv[counter-1] << endl;

					// cout << "path: "; 
					// for (int i = 0; i < counter; i++)
					// {
					// 	cout << boost::lexical_cast<float>(path_recv[i]) << ",";
					// }
					// cout << endl;	

					counter = 0;

				}

			}

			// SEND THE PATH // 
			for (int j = 0; j < server.num_clients; j++)
			{
				if (server.socket_active[j][6])
				{
						
					// cout << "send path " << endl;
					// Copy path to a buffer
					// cstr_path = new char[5000];
					strcpy(cstr_path, path_recv_string.c_str());
					memcpy(path_send_buffer, cstr_path, strlen(cstr_path)+1);
					// free(cstr_path);

					server.sockets[server.num_sockets*j + 6]->process_sending(path_send_buffer,5000*sizeof(char),0);
				}
			}

			current_time_path = std::chrono::high_resolution_clock::now();

		}				

		////////////////////////////////
		// CONSTRAINTS COMMUNICATIONS //
		////////////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_constraints).count() >= constraint_time_us)
		{

			// cout << "timeElapsed_constraints: " << timeElapsed_constraints.count() << endl;

			// RECEIEVE THE CONSTRAINTS // 
			for (int j = 0; j < server.num_clients; j++)
			{

				if (server.socket_active[j][9])
				{
					// cout << "recv constraints" << endl;	
					bytes_recv_constraints = server.sockets[server.num_sockets*j + 9]->process_receiving(constraints_recv_buffer,5000*sizeof(char),1);
					
					constraints_recv_string.clear();
					constraints_recv_string.resize(5000);
					constraints_recv_string = constraints_recv_buffer;
					// cout << constraints_recv_string << endl;
					counter = 0;
					constraints_recv.clear();

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

					}

					// cout << "constraint:" << constraints_recv[counter-1] << ", " << boost::lexical_cast<float>(constraints_recv[counter-1]) << endl;

					// cout << "constraints: "; 
					// for (int i = 0; i < counter; i++)
					// {
					// 	cout << boost::lexical_cast<float>(constraints_recv[i]) << ",";
					// }
					// cout << endl;	

					counter = 0;

				}

			}

			// SEND THE CONSTRAINTS // 
			for (int j = 0; j < server.num_clients; j++)
			{
				if (server.socket_active[j][8])
				{
						// cout << "send constraints" << endl;
					// Copy path to a buffer
					// cstr_constraints = new char[5000];
					strcpy(cstr_constraints, constraints_recv_string.c_str());
					memcpy(constraints_send_buffer, cstr_constraints, strlen(cstr_constraints)+1);
					// free(cstr_constraints);
					constraints_send = constraints_send_buffer;
					// cout << "constraints_send: " << constraints_send << endl;5
					server.sockets[server.num_sockets*j + 8]->process_sending(constraints_send_buffer,5000*sizeof(char),0);
				}
			}

			current_time_constraints = std::chrono::high_resolution_clock::now();

		}				


		///////////////////////////////
		// TRAJECTORY COMMUNICATIONS //
		///////////////////////////////

		if (std::chrono::duration_cast<std::chrono::microseconds>(timeElapsed_trajectory).count() >= trajectory_time_us)
		{
			// cout << "timeElapsed_trajectory: " << timeElapsed_trajectory.count() << endl;			// RECEIEVE THE TRAJECTORY // 
			
			for (int j = 0; j < server.num_clients; j++)
			{

				if (server.socket_active[j][11])
				{

					// cout << "recv traj" << endl;
					bytes_recv_trajectory = server.sockets[server.num_sockets*j + 11]->process_receiving(trajectory_recv_buffer,6000*sizeof(char),1);
					
					trajectory_recv_string.clear();
					trajectory_recv_string.resize(6000);
					trajectory_recv_string = trajectory_recv_buffer;

					// cout << "trajectory_recv_string: " << trajectory_recv_string<< endl;
 
					counter = 0;
					trajectory_recv.clear();

					if (trajectory_recv_string[0] == 'C')
					{

						for (int i = 1; i < 6000; i++)
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

					}

					// cout << "trajectory: "; 
					// for (int i = 0; i < counter; i++)
					// {
					// 	cout << boost::lexical_cast<float>(trajectory_recv[i]) << ",";
					// }
					// cout << endl;	

					counter = 0;

				}

			}

			// SEND THE TRAJECTORY // 
			for (int j = 0; j < server.num_clients; j++)
			{
				if (server.socket_active[j][10])
				{
						// cout << "send trajectory" << endl;
					// Copy path to a buffer
					// cstr_trajectory = new char[1000];
					strcpy(cstr_trajectory, trajectory_recv_string.c_str());
					memcpy(trajectory_send_buffer, cstr_trajectory, strlen(cstr_trajectory)+1);
					server.sockets[server.num_sockets*j + 10]->process_sending(trajectory_send_buffer,6000*sizeof(char),0);
				}
			}

			current_time_trajectory = std::chrono::high_resolution_clock::now();

		}			


	}

}