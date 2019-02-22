#include "ros/ros.h"
#include "stabil/ServoServer.h"
#include <cstdlib>
#include <iostream>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<stabil::ServoServer>("servo_server");
  stabil::ServoServer srv;
  
  char input[1000];
  while(ros::ok()) {
	  
	  client.call(srv);
	  std::cin >> input;
	  srv.request.ping = atoi(input);
  }

  return 0;
}
