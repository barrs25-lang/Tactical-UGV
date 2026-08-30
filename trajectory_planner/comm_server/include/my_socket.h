// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 6th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// Header file defining the SOCKET class.
// End File Decscription ################################################################################


#ifndef SOCKET_H
#define SOCKET_H

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

// Define socket class
class SOCKET 
{

// Publically available members
public:

	// Server socket file descriptor
	int sockfd = 0;

	// Client socket file descriptor
	int Clientsockfd = 0;

	// Declare server and client info structures
	struct sockaddr_in serverInfo,clientInfo;

	// Unsigned int representing the sizeof the structure storing the client info 
	unsigned int addrlen = sizeof(clientInfo);

	// Integer to store the number of bytes received or sent
	int ret = -1, sen = -1;

	string name;	

	int port_num;
	
	// Socket constructor
	SOCKET() 
	{

	} // SOCKET::SOCKET() 

	// set_accept: server side function responsible for accepting any incoming connections
	// INPUTS: none
	// OUTPUTS: integer representing the client socket file descriptor
	int set_accept()
	{
		
		cout << "Waiting for client to connect to " << name << " over port: " << port_num << endl;		
		
		// Accept the connection and set the client socket file descriptor
		Clientsockfd = accept(sockfd,(struct sockaddr*)&serverInfo, &addrlen);

		// Check if the fd was properly created
		if (Clientsockfd == -1)
		{
			
			cout << "Could not connect to " << name << " over port: " << port_num << endl;		

			// If the connection failed, keep trying
			while(Clientsockfd == -1)
			{

				Clientsockfd = accept(sockfd,(struct sockaddr*)&serverInfo, &addrlen);

			} // while(Clientsockfd == -1)	

		} // if (Clientsockfd == -1)
		else
		{

			cout << "Client connected to " << name << " over port: " << port_num << endl;		

		} // if (Clientsockfd == -1)

		// return the client socket file descriptor
		return Clientsockfd;

	} // int set_accept()


	// set_nonblocking: this function is responsible or setting the socket 
	// file control such that sends or receives never block the program waiting for 
	// information
	// INPUTS: none
	void set_nonblocking()
	{

		int flag = fcntl(sockfd, F_GETFL, 0);

		// checks if we got the file control successfully
		if (flag < 0) 
		{

			perror("fcntl1 F_GETFL fail");

		} // if (flag < 0)

		// Sets the socket to non blocking and simultaneously checks success
		if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) 
		{

			perror("fcntl1 F_SETFL fail");

		} // if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) 

		} // void set_nonblocking()


	// process_sending: this function sends the information at sendbuf, SIZE number of bytes
	// to the designated socket, according to flag
	// INPUTS: pointer to buffer to be sent (no data type because SIZE tells how many bytes to send)
	// integer representing the number of bytes to send, boolean representing how the send function should behave
	void process_sending(void * sendbuf, int SIZE, bool flag) 
	{

		if(!flag)
		{
			// Send the data to the client socket and wait (doesn't mean anything when sending)
			sen = send(Clientsockfd, sendbuf, SIZE, MSG_WAITALL);

		} // if(!flag)
		else if(flag)
		{

			// Send the data to the client socket and don't wait (doesn't mean anything when sending)
			sen = send(Clientsockfd, sendbuf, SIZE, MSG_DONTWAIT);

		} // else if(flag)

	} // void process_sending(void * sendbuf, int SIZE, bool flag)


	// process_receiving: this function receives data from a client socket
	// INPUTS: pointer to a buffer to receive data, integer indicating the number
	// of bytes to read from the socket and store in the buffer, integer representing
	// how the recv function should behave
	// OUTPUTS: integer indicating how many bytes were read from the socket
	int process_receiving(void* recvbuf, int SIZE, int flag) 
	{

		// Check the flag for how the recv function should behave
		if (flag == 0)
		{
			// Receive SIZE bytes from Client socket and wait until SIZE bytes have been read
			ret = recv(Clientsockfd, recvbuf, SIZE, MSG_WAITALL);

		} // if (flag == 0)
		else if (flag == 1)
		{

			// Receive SIZE bytes from Client socket and don't wait
			ret = recv(Clientsockfd, recvbuf, SIZE, MSG_DONTWAIT);

		} // else if (flag == 1)	
		else if (flag == 2)
		{

			// Receive SIZE bytes from Client socket and default recv behavior
			ret = recv(Clientsockfd, recvbuf, SIZE, 0);

		} // else if (flag == 2)

		// return an integer indicating how many bytes were read from the socket
		return ret;

	} // int process_receiving(void* recvbuf, int SIZE, int flag) 


	int process_receive_initial_transmission() 
	{

		char recv_buf[30];

		cout << "Waiting to receive initial transmission to " << name << endl;

		// Receive SIZE bytes from Client socket and wait until SIZE bytes have been read
		int ret = recv(Clientsockfd, recv_buf, sizeof(char)*30, MSG_WAITALL);

		cout << "Received " << ret << " bytes through " << name << endl;

		string s(recv_buf);
		cout << "Message: " << s << endl;

		// return an integer indicating how many bytes were read from the socket
		return ret;

	} // int process_receiving(void* recvbuf, int SIZE, int flag) 


	// End: this function shuts the socket down and closes the connection to the designated port
	void End() 
	{

		// Shutdown and close the socket if the socket successfully connected in the first place
		if (!name.empty())
		{
			int err = 1;
			socklen_t len = sizeof(err);
			if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char*)&err, &len) == -1)
			{
				cout << "Error in socket End()." << endl;
				exit(0);
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
	
	} // void End() 

	// init: Sets up file descriptor, fills server info
	// INPUTS: uint16_t describing the port of which the socket will connect to
	void init(uint16_t port_num_in, const string& name_in) 
	{

		name = name_in;
		port_num = port_num_in;

		// Setup a socket file descriptor, TCP type connection (SOCK_STREAM)
		sockfd = socket(AF_INET, SOCK_STREAM, 0);

		if(sockfd == 0)
		{
			
			printf("Fail to create a socket!\n");

		} // if(sockfd == 0)
	
		serverInfo.sin_family = AF_INET;
		// local host test
		serverInfo.sin_addr.s_addr = INADDR_ANY;	//"127.0.0.1" = INADDR_ANY
		serverInfo.sin_port = htons(port_num_in);
		bind(sockfd,(struct sockaddr *)&serverInfo,sizeof(serverInfo));
		listen(sockfd,SOMAXCONN); 

		set_accept();
		process_receive_initial_transmission();
				
	} // void init(uint16_t port_num) 	

}; //class SOCKET

#endif