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

#define PI 3.14159265359

struct stick_cmd {
	float x;
	float y;
	float z;
} stick_cmd;

struct stick_cmd l_stick, r_stick;

stabil::PCA9685 srv;
stabil::AttitudeControl attcon;

servo servo_fl = servo(0, 2800, 500, 1.5 * PI, 0);
servo servo_fr = servo(2, 2800, 500, 1.5 * PI, 1);
servo servo_rl = servo(4, 2800, 500, 1.5 * PI, 0);
servo servo_rr = servo(6, 2800, 500, 1.5 * PI, 1);

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

float jxBuffer[3] = {0,0,0};
uint8_t jxBufferIdx = 0;

float jyBuffer[3] = {0,0,0};
uint8_t jyBufferIdx = 0;

float bufferAverage(float k, uint8_t &i, float buffer[]) {
	if (i > 3) i = 0;
	buffer[i] = k;
	i++;
	return (buffer[0] + buffer[1] + buffer[2]) / 3;

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
	offset.x = 0;
	offset.y = 0;
	offset.z = 4;

	ros::Rate loop_rate(50);

	int v_x = 0;
	int v_y = 0;
	bool dir = 0;
	int counter = 0;

	int pwm_req[8];

	ROS_INFO("Starting main loop");
  	while(ros::ok()) {
		  //ROS_INFO("Spun once.");


  		  attcon.request.jx = l_stick.x;//bufferAverage(l_stick.x, jxBufferIdx, jxBuffer);
  		  attcon.request.jy = l_stick.y;//bufferAverage(l_stick.y, jyBufferIdx, jyBuffer);
  		  //ROS_INFO("%f and %f", attcon.request.jx, attcon.request.jy);
  		  offset.y = 3 + 2 * r_stick.y;
  		  attcon.request.offset = offset;

  		  attcon.request.ground.f0 = 0;
		  attcon.request.ground.f1 = 0;
		  attcon.request.ground.f2 = 0;
		  attcon.request.ground.f3 = 0;


		  counter++;
		  if (!(counter % 2)) {
			  //every 4 actually do the calculations
			  if (limb_pose.call(attcon)) {
				  //ROS_INFO("We heard back!");
				  //ROS_INFO("Thetas: %f, %f, %f, %f", attcon.response.theta.f0, attcon.response.theta.f1, attcon.response.theta.f2, attcon.response.theta.f3);
				  for (int i = 0; i < 4; i++) {
					  //ROS_INFO("Contact %d: (%f, %f, %f)", i, attcon.response.contact.at(i).x, attcon.response.contact.at(i).y, attcon.response.contact.at(i).z);
				  }

			  }
			  else ROS_INFO(" DOH! ");
		  }



		  servo_fl.setGoalAngle(attcon.response.theta.f0);
		  servo_fr.setGoalAngle(attcon.response.theta.f1);
		  servo_rl.setGoalAngle(attcon.response.theta.f2);
		  servo_rr.setGoalAngle(attcon.response.theta.f3);

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
