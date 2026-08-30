#ifndef MY_SOCKET_H_
#define MY_SOCKET_H_

#include <sched.h>
#include <cstdio>
#include <iostream> 
#include <termios.h> 
#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

class SOCKET {
public:

	int sockfd = 0, Clientsockfd = 0, err;
	struct sockaddr_in serverInfo,clientInfo;
	unsigned int addrlen = sizeof(clientInfo);
	int ret = -1;
	string name;
	int port_num;
	bool socket_connected = 0;

	// Socket declared
	SOCKET() 
	{

	}

	int set_accept()
	{
		printf("wait for connection....\n");
		Clientsockfd = accept(sockfd,(struct sockaddr*)&serverInfo, &addrlen);
		if (Clientsockfd == -1)
		{
			perror("Count not accept gnc connection");
		}
		else
		{
			printf("Connection Accepted!\n");
		}
		return Clientsockfd;
	}
	void set_nonblocking()
	{
		//fcntl(Clientsockfd , F_SETFL, O_NONBLOCK);
		int flag = fcntl(sockfd, F_GETFL, 0);
		if (flag < 0) {
			perror("fcntl1 F_GETFL fail");
			//return 0;
		}
		if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
			perror("fcntl1 F_SETFL fail");
			//return 0;
		}
			printf("Non-blocking Activated!\n");
	}
	void process_sending(void * message, long unsigned int SIZE) 
	{

		//cout << "char size:" << sizeof(data) << endl;
		// if (flag == 0)
		// {
			// send(sockfd, message, SIZE, MSG_WAITALL);
			
		// }
		// else
		// {
			send(sockfd, message, SIZE, MSG_DONTWAIT);

		// }
		//send(sockfd, message, SIZE, MSG_DONTWAIT);
		// force cast
		//float(&pArray)[n_update][4] = *reinterpret_cast<float(*)[n_update][4]>(data);

	}

	void process_sending_initial_transmission() 
	{

		char message[30] = "Hi this is client 1.";

		cout << "Sending message to server through " << name << endl;
		send(sockfd, message, sizeof(message), MSG_DONTWAIT);
		cout << "Sent!" << endl;
		socket_connected = 1;

	}

	// data receive from server
	int process_receiving(void * recvbuf, int SIZE, int flag) 
	{

		if (flag == 0)
		{
			ret = recv(sockfd, recvbuf, SIZE, MSG_WAITALL);
		}
		else if (flag == 1)
		{
			ret = recv(sockfd, recvbuf, SIZE, MSG_DONTWAIT);
		}
		return ret;

	}

	void End() 
	{

		if (socket_connected)
		{
			int err = 1;
			socklen_t len = sizeof(err);
			if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char*)&err, &len) == -1)
			{
				cout << "Error in socket End()." << endl;
				// exit(0);
			}

			if(shutdown(sockfd, SHUT_RDWR) < 0)
			{
				if (err != ENOTCONN && err != EINVAL)
				{
					cout << "Error when trying to shutdown the socket." << endl;
				}
			}
			if (close(sockfd) < 0)
			{
				cout << "Error when trying to close the socket." << endl;
			}
			
			cout << "Closed " << name << " connection over: " << port_num << endl;
		}
	}

	//check if there some error
	void init(uint16_t port_num_in, const string& name_in, bool* gg_exit_thread) 
	{

		port_num = port_num_in;
		name = name_in;

		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if(sockfd == -1){
			printf("Fail to create a socket!\n");
		}

		bzero(&serverInfo,sizeof(serverInfo));
		serverInfo.sin_family = AF_INET;
		serverInfo.sin_addr.s_addr = inet_addr("127.0.0.1");	
		serverInfo.sin_port = htons(port_num);

		cout << "Waiting to connect " << name << " over port: " << port_num << endl;
		while(!(*gg_exit_thread)) 
		{
			err = connect(sockfd,(struct sockaddr *)&serverInfo,sizeof(serverInfo));
	    	if(err!=-1)
			{

				cout << "Connected " << name << " on port: " << port_num << endl;
				break;
			}
		}

		if (!(*gg_exit_thread))
		{
			process_sending_initial_transmission();
		}

	}

	
};

class SOCKETWrapper
{

public: 
	SOCKET* _in_pose_sim_socket;
	SOCKET* _in_map_sim_socket;
	SOCKET* _in_goal_generation_socket;
	SOCKET* _out_path_sim_socket;
	SOCKET* _out_path_mpc_socket;

	void socketWrapper(SOCKET* _in_pose_sim_socket, SOCKET* _in_map_sim_socket, SOCKET* _in_goal_generation_socket, SOCKET* _out_path_sim_socket, SOCKET* _out_path_mpc_socket)
	{

		this->_in_pose_sim_socket = _in_pose_sim_socket;
		this->_in_map_sim_socket = _in_map_sim_socket;
		this->_in_goal_generation_socket = _in_goal_generation_socket;
		this->_out_path_sim_socket = _out_path_sim_socket;
		this->_out_path_mpc_socket = _out_path_mpc_socket;

	}

	SOCKET* operator[](int i)
	{

		switch(i)
		{

			case 1: return _in_pose_sim_socket; break;
			case 2: return _in_map_sim_socket; break;
			case 3: return _in_goal_generation_socket; break;
			case 4: return _out_path_sim_socket; break;
			case 5: return _out_path_mpc_socket; break;

		}

	}

private:

};
	

#endif