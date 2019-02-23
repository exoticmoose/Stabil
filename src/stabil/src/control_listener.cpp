#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "stabil/ServoServer.h"
#include "stabil/PCA9685.h"
#include <geometry_msgs/Twist.h>
#include <cstdlib>


struct stick_cmd {
	float x;
	float y;
	float z;
} stick_cmd;

struct stick_cmd l_stick, r_stick;


void charCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: \"%s\" ... data = %d", msg->data.c_str(), atoi(msg->data.c_str()));
}

void twistCallback(const geometry_msgs::Twist &twist) {
	//ROS_INFO("Got linear  twist data... X: %+4.3f Y: %+4.3f Z: %+4.3f", (float)twist.linear.x, (float)twist.linear.y, (float)twist.linear.z);
	//ROS_INFO("Got angular twist data... X: %+4.3f Y: %+4.3f Z: %+4.3f", (float)twist.angular.x, (float)twist.angular.y, (float)twist.angular.z);
	//ROS_INFO("----------------------");

	l_stick.x = twist.linear.x;
	l_stick.y = twist.linear.y;
	l_stick.z = twist.linear.z;

	r_stick.x = twist.angular.x;
	r_stick.y = twist.angular.y;
	r_stick.z = twist.angular.z;

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




	ros::Rate loop_rate(10);

	int v_x = 0;
	int v_y = 0;
	bool dir = 0;


	int pwm_req[8];

  	while(ros::ok()) {
		  //ROS_INFO("Spun once.");

		  srv.request.address[0] = 0;
		  srv.request.address[1] = 2;
		  srv.request.address[2] = 4;
		  srv.request.address[3] = 6;
//
//		  if (curr_pwm > 2800 && dir) {
//			  dir = 0;
//		  }
//		  if (curr_pwm < 500 && !dir) {
//			  dir = 1;
//		  }
//		  if (dir) curr_pwm += 32;
//		  else curr_pwm-= 32;

		  v_x = 1200* l_stick.x;
		  v_y = 1200* l_stick.y;

		  srv.request.request[0] = 1600 + v_x;
		  srv.request.request[1] = 1600 - v_x;
		  srv.request.request[2] = 1600 + v_x;
		  srv.request.request[3] = 1600 - v_x;

		  if (servo_control.call(srv)) {
			  //ROS_INFO("Success, sent %d : %d, got: %d", (int)srv.request.address, (int)srv.request.request, (int)srv.response.response);

		  }
		  else {
			  ROS_ERROR("Failed to call service servo_server");

		  }

		  ros::spinOnce();
		  loop_rate.sleep();
	  }

  return 0;
}
