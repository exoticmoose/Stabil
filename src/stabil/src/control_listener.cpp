#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "stabil/ServoServer.h"
#include "stabil/PCA9685.h"
#include <geometry_msgs/Twist.h>
#include <cstdlib>


void charCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: \"%s\" ... data = %d", msg->data.c_str(), atoi(msg->data.c_str()));
}

void twistCallback(const geometry_msgs::Twist &twist) {
	ROS_INFO("Got linear twist data... X: %.2d Y: %.2d Z: %.2d", (int)twist.linear.x, (int)twist.linear.y, (int)twist.linear.z);
	ROS_INFO("Got angular twist data... X: %.2d Y: %.2d Z: %.2d", (int)twist.angular.x, (int)twist.angular.y, (int)twist.angular.z);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_listener");

	ros::NodeHandle listener;
	ros::Subscriber char_sub_ = listener.subscribe("remote_cmd_char", 1000, charCallback);
	ros::Subscriber twist_sub_ = listener.subscribe("cmd_vel", 1000, twistCallback);




	ros::NodeHandle clientNode;
	ros::ServiceClient servo_control = clientNode.serviceClient<stabil::PCA9685>("pca9685");

	stabil::PCA9685 srv;




	ros::Rate loop_rate(50);

	int curr_pwm = 0;
	bool dir = 0;
  	while(ros::ok()) {
		  //ROS_INFO("Spun once.");

		  srv.request.address = 0;

		  if (curr_pwm > 2800 && dir) {
			  dir = 0;
		  }
		  if (curr_pwm < 500 && !dir) {
			  dir = 1;
		  }
		  if (dir) curr_pwm += 32;
		  else curr_pwm-= 32;

		  srv.request.request = curr_pwm;

		  if (servo_control.call(srv)) {
			  ROS_INFO("Success, sent %d : %d, got: %d", (int)srv.request.address, (int)srv.request.request, (int)srv.response.response);

		  }
		  else {
			  ROS_ERROR("Failed to call service servo_server");

		  }

		  ros::spinOnce();
		  loop_rate.sleep();
	  }

  return 0;
}
