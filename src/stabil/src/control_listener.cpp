#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stabil/ServoServer.h"
#include <cstdlib>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: \"%s\" ... data = %d", msg->data.c_str(), atoi(msg->data.c_str()));




}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_listener");

	ros::NodeHandle listener;
	ros::Subscriber sub = listener.subscribe("remote_cmd_char", 1000, chatterCallback);

	ros::NodeHandle clientNode;
	ros::ServiceClient servo_control = clientNode.serviceClient<stabil::ServoServer>("servo_server");

	stabil::ServoServer srv;




	ros::Rate loop_rate(1);


  	while(ros::ok()) {
		  ROS_INFO("Spun once.");

		  srv.request.ping = 69; //TODO

		  if (servo_control.call(srv)) {
			  ROS_INFO("Success, got: %d", (int)srv.response.res);

		  }
		  else {
			  ROS_ERROR("Failed to call service servo_server");

		  }

		  ros::spinOnce();
		  loop_rate.sleep();
	  }

  return 0;
}
