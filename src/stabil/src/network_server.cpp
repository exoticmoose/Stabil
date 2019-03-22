#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include "stabil/ServoServer.h"
#include <cstdlib>
#include <iostream>

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "servo_client");
	ros::NodeHandle broadcaster;
	ros::Publisher chatter_pub = broadcaster.advertise<std_msgs::String>("remote_cmd_char", 1000);
	std_msgs::String cmd;
	
	 do {
	
		 int sockfd, newsockfd, portno;
		 socklen_t clilen;
		 char buffer[256];
		 struct sockaddr_in serv_addr, cli_addr;
		 int n;
		 if (argc < 2) {
			 fprintf(stderr,"ERROR, no port provided\n");
			 exit(1);
		 }
		 sockfd = socket(AF_INET, SOCK_STREAM, 0);
		 if (sockfd < 0) 
			error("ERROR opening socket");
		 bzero((char *) &serv_addr, sizeof(serv_addr));
		 portno = atoi(argv[1]);
		 serv_addr.sin_family = AF_INET;
		 serv_addr.sin_addr.s_addr = INADDR_ANY;
		 serv_addr.sin_port = htons(portno);
		 if (bind(sockfd, (struct sockaddr *) &serv_addr,
				  sizeof(serv_addr)) < 0) 
				  error("ERROR on binding");
		 listen(sockfd,5);
		 clilen = sizeof(cli_addr);
		 newsockfd = accept(sockfd, 
					 (struct sockaddr *) &cli_addr, 
					 &clilen);
		 if (newsockfd < 0) 
			  error("ERROR on accept");
		 bzero(buffer,256);
		 
		 int msg_count = 0;
		 
		char * pch;
		int nop = 1;
		do {
			bzero(buffer,256);
			n = read(newsockfd,buffer,255);
			if (n < 0) error("ERROR reading from socket");
			printf("Server Rx'd: %s\n",buffer);
			n = write(newsockfd,buffer,sizeof(buffer));
			if (n < 0) error("ERROR writing to socket");
			msg_count++;
			cmd.data = buffer;
			chatter_pub.publish(cmd);
				
		} while (nop);
		 
		 
		 
		 close(newsockfd);
		 close(sockfd);
	 } while (1);
     return 0; 
}
