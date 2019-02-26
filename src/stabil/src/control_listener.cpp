#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "stabil/ServoServer.h"
#include "stabil/QuadFloat.h"
#include "stabil/PCA9685.h"
#include "stabil/AttitudeControl.h"
#include <geometry_msgs/Twist.h>
#include <cstdlib>

#include "servo.h"


struct stick_cmd {
	float x;
	float y;
	float z;
} stick_cmd;

struct stick_cmd l_stick, r_stick;

stabil::PCA9685 srv;
stabil::AttitudeControl attcon;

servo servo_fl = servo(0, 2500, 500, 90, 0);
servo servo_fr = servo(2, 2500, 500, 90, 0);
servo servo_rl = servo(4, 2500, 500, 90, 0);
servo servo_rr = servo(6, 2500, 500, 90, 0);

void charCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: \"%s\" ... data = %d", msg->data.c_str(), atoi(msg->data.c_str()));
}

void twistCallback(const geometry_msgs::Twist &twist) {
	ROS_INFO("Got linear  twist data... X: %+4.3f Y: %+4.3f Z: %+4.3f", (float)twist.linear.x, (float)twist.linear.y, (float)twist.linear.z);
	//ROS_INFO("Got angular twist data... X: %+4.3f Y: %+4.3f Z: %+4.3f", (float)twist.angular.x, (float)twist.angular.y, (float)twist.angular.z);
	ROS_INFO("----------------------");

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
	ros::ServiceClient limb_pose = clientNode.serviceClient<stabil::AttitudeControl>("limb_pose");


	srv.request.address[0] = 0;
	srv.request.address[1] = 2;
	srv.request.address[2] = 4;
	srv.request.address[3] = 6;


	geometry_msgs::Point offset;
	offset.x = -1;
	offset.y = 420;
	offset.z = 6;

	ros::Rate loop_rate(40);

	int v_x = 0;
	int v_y = 0;
	bool dir = 0;


	int pwm_req[8];

	ROS_INFO("Starting main loop");
  	while(ros::ok()) {
		  //ROS_INFO("Spun once.");


  		  attcon.request.jx = l_stick.x;
  		  attcon.request.jy = l_stick.y;
  		  attcon.request.offset = offset;

  		  attcon.request.ground.f0 = 0;
		  attcon.request.ground.f1 = 1;
		  attcon.request.ground.f2 = 2;
		  attcon.request.ground.f3 = 3;


  		  if (limb_pose.call(attcon)) {
  			  ROS_INFO("We heard back!");
  		  } else ROS_INFO(" DOH! ");

		  v_x = 400* l_stick.x;
		  v_y = 1200* l_stick.y;

		  servo_fl.setGoal((unsigned short) 1600 + v_x);
		  servo_fr.setGoal((unsigned short) 1600 - v_x);
		  servo_rl.setGoal((unsigned short) 1600 + v_x);
		  servo_rr.setGoal((unsigned short) 1600 - v_x);

		  servo_fl.calcNext();
		  servo_fr.calcNext();
		  servo_rl.calcNext();
		  servo_rr.calcNext();

		  servo_fl.sendNext();
		  servo_fr.sendNext();
		  servo_rl.sendNext();
		  servo_rr.sendNext();



//		  srv.request.request[0] = 1600 + v_x;
//		  srv.request.request[1] = 1600 - v_x;
//		  srv.request.request[2] = 1600 + v_x;
//		  srv.request.request[3] = 1600 - v_x;

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
