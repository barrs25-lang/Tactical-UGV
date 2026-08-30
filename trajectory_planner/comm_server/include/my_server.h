// ----------
 
// stuff

// ----------

#include <my_socket.h>

#include <signal.h>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string.h>
#include <boost/lexical_cast.hpp>

class SERVER
{

public:

	SOCKET** sockets;

	int num_sockets = 12;
	int** port_numbers;
	string** socket_names;
	bool** socket_active;
	int num_clients = 0;

	/////////////////////////////////////////
	// The 12 sockets are ordered as such: //
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
	/////////////////////////////////////////	

	// Constructor
	SERVER(int _num_clients)
	{
		num_clients = _num_clients;

		port_numbers = new int*[num_clients];
		for (int j = 0; j < num_clients; j++)
		{
			port_numbers[j] = new int[num_sockets];
		}
		socket_active = new bool*[num_clients];
		for (int j = 0; j < num_clients; j++)
		{
			socket_active[j] = new bool[num_sockets];
		}
		socket_names = new string*[num_clients];
		for (int j = 0; j < num_clients; j++)
		{
			socket_names[j] = new string[num_sockets];
		}

		sockets = new SOCKET*[num_sockets*num_clients];
		for (int i = 0; i < num_clients*num_sockets; i++)
		{
			sockets[i] = new SOCKET;
		}
		
		// String to store file lines
		string file_line;
		stringstream ss;
		
		for (int j = 0; j < num_clients; j++)
		{
			// int temp = j+1;
			ifstream file("socket_parameters" + to_string(j) + ".txt");
			//read port numbers
			for (int i = 0; i < num_sockets; i++)
			{	

				do{ss.clear(); getline(file, file_line); ss.str(file_line);}
				while(file_line.at(0) == '/' && file_line.at(1) == '/');
				ss >> port_numbers[j][i];	

				do{ss.clear(); getline(file, file_line); ss.str(file_line);}
				while(file_line.at(0) == '/' && file_line.at(1) == '/');
				ss >> socket_names[j][i];	

				do{ss.clear(); getline(file, file_line); ss.str(file_line);}
				while(file_line.at(0) == '/' && file_line.at(1) == '/');
				ss >> socket_active[j][i];	

				if (socket_active[j][i])
				{
					sockets[num_sockets*j + i]->init(port_numbers[j][i],socket_names[j][i]);
				}

			}		

			file.close();

		}

	}

	// Destructor
	~SERVER()
	{

		for (int i = 0; i < num_sockets*num_clients; i++)
		{
			sockets[i]->End();
		}

	}

private:

};