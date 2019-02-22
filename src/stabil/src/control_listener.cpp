#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stabil/ServoServer.h"
#include "stabil/PCA9685.h"
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
	ros::ServiceClient servo_control = clientNode.serviceClient<stabil::PCA9685>("pca9685");

	stabil::PCA9685 srv;




	ros::Rate loop_rate(1);

	int curr_pwm = 0;
	bool dir = 0;
  	while(ros::ok()) {
		  ROS_INFO("Spun once.");

		  srv.request.address = 0;

		  if (curr_pwm > 2800 && dir) {
			  dir = 0;
		  }
		  if (curr_pwm < 500 && !dir) {
			  dir = 1;
		  }
		  if (dir) curr_pwm += 5;
		  else curr_pwm-= 5;

		  srv.request.request = curr_pwm;

		  if (servo_control.call(srv)) {
			  ROS_INFO("Success, got: %d", (int)srv.response.response);

		  }
		  else {
			  ROS_ERROR("Failed to call service servo_server");

		  }

		  ros::spinOnce();
		  loop_rate.sleep();
	  }

  return 0;
}
