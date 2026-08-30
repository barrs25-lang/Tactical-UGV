#ifndef DATASEND_H_
#define DATASEND_H_

#include <sys/socket.h>	//unix socket
#include <sys/types.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sched.h>
#include <termios.h> 

class DataSend {
public:

	int sockfd = 0, Clientsockfd = 0, err = 0;
	struct sockaddr_in serverInfo,clientInfo;
	unsigned int addrlen = sizeof(clientInfo);
	int ret = -1;
	char message[30] = "Hi,this is client.\n";
	// Socket declared
	DataSend() 
	{
		printf("Socket called!\n");
	}
	//check if there some error
	void init(uint16_t port_num) 
	{

		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if(sockfd == -1){
			printf("Fail to create a socket!\n");
		}
		//************************
		bzero(&serverInfo,sizeof(serverInfo));
		serverInfo.sin_family = AF_INET;
		// local host test
		serverInfo.sin_addr.s_addr = inet_addr("127.0.0.1");	//127.0.0.1		104.39.160.46	104.39.89.30
		serverInfo.sin_port = htons(port_num);

		std::cout << "Waiting to connect ..." << std::endl;

		while(1)
		{
			err = connect(sockfd,(struct sockaddr *)&serverInfo,sizeof(serverInfo));
	    	if(err==-1){

			}
			else{
				printf("Connected!\n");
				std::cout << ">> Port:" << port_num << std::endl;
				break;
			}
			usleep(100);
		}

	}
	void init_server(uint16_t port_num) 
	{

		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if(sockfd == 0)
		{
			//printf("Fail to create a socket!\n");
		}
		//************************
		//bzero(&serverInfo,sizeof(serverInfo));
		serverInfo.sin_family = AF_INET;
		// local host test
		serverInfo.sin_addr.s_addr = INADDR_ANY;	//"127.0.0.1" = INADDR_ANY
		serverInfo.sin_port = htons(port_num);
		bind(sockfd,(struct sockaddr *)&serverInfo,sizeof(serverInfo));
		listen(sockfd,SOMAXCONN); // JAM 12/5/2021: changed 2nd argument from 100 to SOMAXCONN
				
	}
	int set_accept()
	{
		printf("wait for connection....\n");
		Clientsockfd = accept(sockfd,(struct sockaddr*)&serverInfo, &addrlen);
		if (Clientsockfd == -1)
		{
			while(Clientsockfd == -1)
			{
				Clientsockfd = accept(sockfd,(struct sockaddr*)&serverInfo, &addrlen);
			}	
			//perror("Could not accept connection");
		}
		else{
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
			send(sockfd, message, SIZE, MSG_DONTWAIT);
	}

	void process_sending_server(void * message, long unsigned int SIZE) 
	{
			send(Clientsockfd, message, SIZE, MSG_DONTWAIT);
	}

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

		if (err != -1)
		{
			shutdown(sockfd, SHUT_RDWR);
			close(sockfd);
		}
		printf("Socket Closed!\n");

	}
};

class DataSendWrapper
{

public: 
	DataSend* _in_pose_socket;
	DataSend* _in_map_socket;
	DataSend* _out_goal_socket;
	DataSend* _out_bin_socket;

	void socketWrapper(DataSend* _in_pose_socket, DataSend* _in_map_socket, DataSend* _out_goal_socket, DataSend* _out_bin_socket)
	{

		this->_in_pose_socket = _in_pose_socket;
		this->_in_map_socket = _in_map_socket;
		this->_out_goal_socket = _out_goal_socket;
		this->_out_bin_socket = _out_bin_socket;

	}
	void socketWrapper(DataSend* _in_pose_socket, DataSend* _in_map_socket, DataSend* _out_goal_socket)
	{

		this->_in_pose_socket = _in_pose_socket;
		this->_in_map_socket = _in_map_socket;
		this->_out_goal_socket = _out_goal_socket;

	}

private:

};

#endif