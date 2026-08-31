// ----------
 
// stuff

// ----------

#ifndef MY_CLIENT_H_
#define MY_CLIENT_H_

#include <my_socket.h>
#include <signal.h>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string.h>
#include <boost/lexical_cast.hpp>

class CLIENT
{

public:

	SOCKET** sockets;

	int num_sockets = 12;	
	int* port_numbers;
	string* socket_names;
	bool* socket_active;
	bool active = false;

	/////////////////////////////////////////////
	// The first 12 sockets are ordered as such: //
	// map from server					   //
	// map to server					   //
	// pose from server					   //
	// pose to server					   //
	// goal from server					   //
	// goal to server					   //
	// path from server					   //
	// path to server					   //
	// constraint from server			   //
	// constraint to server				   //
	// trajectory from server			   //
	// trajectory to server				   //
	// Any sockets beyond the 12th are package-specific additions (e.g. index 12,
	// "control_to_server", for trajectory_planner's control-sequence output) --
	// see that package's own socket_parameters_*.txt for what they mean.
	/////////////////////////////////////////

	// Constructor
	// num_sockets_in: total number of (port, name, active) blocks to read from
	// socket_filename, defaulting to the original 12 so existing call sites are unaffected.
	CLIENT(const string& socket_filename, bool* gg_exit_thread, int num_sockets_in = 12)
	{

		num_sockets = num_sockets_in;

		port_numbers = new int[num_sockets];
		socket_names = new string[num_sockets];
		socket_active = new bool[num_sockets];
		
		sockets = new SOCKET*[num_sockets];
		for (int i = 0; i < num_sockets; i++)
		{
			sockets[i] = new SOCKET;
		}

		ifstream file(socket_filename);
		
		// String to store file lines
		string file_line;
		stringstream ss;
		
		//read port numbers
		for (int i = 0; i < num_sockets; i++)
		{	

			do{ss.clear(); getline(file, file_line); ss.str(file_line);}
			while(file_line.at(0) == '/' && file_line.at(1) == '/');
			ss >> port_numbers[i];	

			do{ss.clear(); getline(file, file_line); ss.str(file_line);}
			while(file_line.at(0) == '/' && file_line.at(1) == '/');
			ss >> socket_names[i];	

			do{ss.clear(); getline(file, file_line); ss.str(file_line);}
			while(file_line.at(0) == '/' && file_line.at(1) == '/');
			ss >> socket_active[i];	

			if (socket_active[i])
			{
				sockets[i]->init(port_numbers[i], socket_names[i], gg_exit_thread);
				active = true;
			}

			if ((*gg_exit_thread))
			{
				file.close();
				return;
			}	

		}

		file.close();

		if (!active)
		{
			cout << "Nothing to connect, exiting." << endl;
			exit(0);
		}

	}	

	// Destructor
	~CLIENT()
	{

		cout << "<CLIENT-CONTROL> Closing sockets..." << endl;
		for (int i = 0; i < num_sockets; i++)
		{
			sockets[i]->End();
		}
		cout << "<CLIENT-CONTROL> Sockets closed." << endl;

	}

private:

};

#endif
