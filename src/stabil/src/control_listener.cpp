#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Joy.h"
#include "stabil/ServoServer.h"
#include "stabil/QuadFloat.h"
#include "stabil/PCA9685.h"
#include "stabil/AttitudeControl.h"
#include "stabil/IMUEffort.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <cstdlib>

#include "servo.h"
#include "control_listener.h"

#define PI 3.14159265359

struct stick_cmd {
	float x;
	float y;
	float z;
} stick_cmd;

struct stick_cmd l_stick, r_stick;

stabil::PCA9685 srv;
stabil::AttitudeControl attcon;
stabil::IMUEffort effort;

servo servo_fl = servo(0, 2800, 500, 1.5 * PI, 1);
servo servo_fr = servo(2, 2800, 500, 1.5 * PI, 0);
servo servo_rl = servo(4, 2800, 500, 1.5 * PI, 1);
servo servo_rr = servo(6, 2800, 500, 1.5 * PI, 0);

class stabilBody {

	private:
		void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

		ros::NodeHandle nh_;
		ros::Subscriber imu_sub_;

	public:
		stabilBody();
		ros::ServiceClient limb_pose_;
		ros::ServiceClient ground_calc_;
};

stabilBody::stabilBody() {


	limb_pose_ = nh_.serviceClient<stabil::AttitudeControl>("limb_pose");
	ground_calc_ = nh_.serviceClient<stabil::IMUEffort>("ground_calc");
	imu_sub_ = nh_.subscribe("imu", 100, &stabilBody::imuCallback, this);

}


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

void stabilBody::imuCallback(const sensor_msgs::Imu::ConstPtr &imu) {
	//ROS_INFO("Control listener got IMU Data");
	effort.request.x = imu->orientation.x;
	effort.request.y = imu->orientation.y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_listener");

	stabilBody stabil;

	ros::NodeHandle n;
	ros::Subscriber char_sub_ = n.subscribe("remote_cmd_char", 1000, charCallback);
	ros::Subscriber twist_sub_ = n.subscribe("cmd_vel", 1000, twistCallback);


//	ros::Publisher body_latitude = n.advertise<std_msgs::Float64>("body_latitude", 100);
//	ros::Publisher body_longitude = n.advertise<std_msgs::Float64>("body_longitude", 100);


	ros::NodeHandle clientNode;
	ros::ServiceClient servo_control = clientNode.serviceClient<stabil::PCA9685>("pca9685");



	srv.request.address[0] = 0;
	srv.request.address[1] = 2;
	srv.request.address[2] = 4;
	srv.request.address[3] = 6;


	geometry_msgs::Point offset;
	offset.x = 0;
	offset.y = 0;
	offset.z = 10;

	ros::Rate loop_rate(100);

	int v_x = 0;
	int v_y = 0;
	bool dir = 0;
	int counter = 0;

	int pwm_req[8];

	ROS_INFO("Starting main loop");
  	while(ros::ok()) {
		  ROS_INFO("Spun once: Loop start.");


  		  attcon.request.jx = fabs(l_stick.x) < 0.1 ? 0 : l_stick.x;//bufferAverage(l_stick.x, jxBufferIdx, jxBuffer);
  		  attcon.request.jy = fabs(l_stick.y) < 0.1 ? 0 : l_stick.y;//bufferAverage(l_stick.y, jyBufferIdx, jyBuffer);
  		  //ROS_INFO("%f and %f", attcon.request.jx, attcon.request.jy);
  		  offset.z = 5 + 5 * r_stick.z;
  		  attcon.request.offset = offset;

//  		  attcon.request.ground.f0 = 0;
//		  attcon.request.ground.f1 = 0;
//		  attcon.request.ground.f2 = 0;
//		  attcon.request.ground.f3 = 0;


		  counter++;
		  if (!(counter % 2)) {
			  //every 2 actually do the calculations
	  		  ROS_INFO("Start calcs");
			  if (stabil.ground_calc_.call(effort)) {
	  			//ROS_INFO("Got efforts: %f \t %f \t %f \t %f", effort.response.w.f0, effort.response.w.f1, effort.response.w.f2, effort.response.w.f3);

	  			attcon.request.ground.f0 = effort.response.w.f0;
				attcon.request.ground.f1 = effort.response.w.f1;
				attcon.request.ground.f2 = effort.response.w.f2;
				attcon.request.ground.f3 = effort.response.w.f3;

	  		  }
	  		  else ROS_INFO("Error calling ground calc service");


			  if (stabil.limb_pose_.call(attcon)) {
				  //ROS_INFO("We heard back!");
				  //ROS_INFO("Thetas: %f, %f, %f, %f", attcon.response.theta.f0, attcon.response.theta.f1, attcon.response.theta.f2, attcon.response.theta.f3);
				  for (int i = 0; i < 4; i++) {
					  //ROS_INFO("Contact %d: (%f, %f, %f)", i, attcon.response.contact.at(i).x, attcon.response.contact.at(i).y, attcon.response.contact.at(i).z);
				  }

			  }
			  else ROS_INFO(" DOH! ");
			  ROS_INFO("End calcs");
		  }



		  servo_fl.setGoalAngle(attcon.response.theta.f0);
		  servo_fr.setGoalAngle(attcon.response.theta.f1);
		  servo_rl.setGoalAngle(attcon.response.theta.f2);
		  servo_rr.setGoalAngle(attcon.response.theta.f3);

//		  servo_fl.setGoalAngle(0);
//		  servo_fr.setGoalAngle(0);
//		  servo_rl.setGoalAngle(0);
//		  servo_rr.setGoalAngle(0);

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
		  ROS_INFO("Servos sent. Iteration: %d", counter);
		  ros::spinOnce();
		  loop_rate.sleep();
	  }

  return 0;
}
