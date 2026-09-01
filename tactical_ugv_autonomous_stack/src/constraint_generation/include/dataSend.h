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

	int sockfd = 0, Clientsockfd = 0, err;
	struct sockaddr_in serverInfo,clientInfo;
	unsigned int addrlen = sizeof(clientInfo);
	int ret = -1;
	char message[30] = "Hi,this is server.\n";
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
	    	if(err==-1)
	    	{

			}
			else
			{
				printf("Connected!\n");
				std::cout << ">> Port:" << port_num << std::endl;
				break;
			}
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

		//std::cout << "char size:" << sizeof(data) << std::endl;
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
	void process_sending_server(void * sendbuf, int SIZE, bool flag) {

		int sen;
//		std::cout << "sendbuf size:" << sizeof(sendbuf) << ", SIZE:" << SIZE << std::endl;
		if(flag == 0){
			sen = send(Clientsockfd, sendbuf, SIZE, MSG_WAITALL);//MSG_DONTWAIT);
		}
		else if(flag == 1){
			sen = send(Clientsockfd, sendbuf, SIZE, MSG_DONTWAIT);
		}
//		printf("sen:%i\n",sen);
		if(sen == -1){
			//std::cout << "\r" << "Sending error!!\n" << std::flush;
			//printf("Sending error!\n");
		}
		else if(sen == 0){
			//std::cout << "\r" << "Sending disconnected!\n" << std::flush;
			//printf("Sending disconnected!\n");
		}
		// force cast
		//float(&pArray)[n_update][4] = *reinterpret_cast<float(*)[n_update][4]>(data);

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
	int process_receiving_server(void * recvbuf, int SIZE, int flag) 
	{

		if (flag == 0)
		{
			ret = recv(Clientsockfd, recvbuf, SIZE, MSG_WAITALL);
		}
		else if (flag == 1)
		{
			ret = recv(Clientsockfd, recvbuf, SIZE, MSG_DONTWAIT);
		}		
		else if (flag == 2)
		{
			ret = recv(Clientsockfd, recvbuf, SIZE, 0);
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

#endif