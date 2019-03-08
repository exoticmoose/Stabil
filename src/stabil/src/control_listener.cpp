#include <cstdlib>
#include <stddef.h>
#include <stdlib.h>
#include "servo.h"
#include "control_listener.h"
#include "codegen.h"

#define PI 3.14159265359

struct xyz {
	float x;
	float y;
	float z;
} xyz;

struct xyz l_stick, r_stick;

stabil::PCA9685 srv;

servo servo_fl = servo(0, 2800, 550, 1.5 * PI, 1);
servo servo_fr = servo(2, 2850, 450, 1.5 * PI, 0);
servo servo_rl = servo(4, 2780, 440, 1.5 * PI, 1);
servo servo_rr = servo(6, 2800, 550, 1.5 * PI, 0);

class stabilBody {

private:
	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
	void pidCallbackX(const std_msgs::Float64 &val);
	void pidCallbackY(const std_msgs::Float64 &val);

	ros::NodeHandle nh_;
	ros::Subscriber imu_sub_;
	ros::Subscriber pid_x_;
	ros::Subscriber pid_y_;

public:
	stabilBody();

	ros::Publisher body_tilt_x_;
	ros::Publisher body_tilt_y_;
	ros::Publisher body_setpoint_x_;
	ros::Publisher body_setpoint_y_;

	std_msgs::Float64 body_tilt_x;
	std_msgs::Float64 body_tilt_y;

	double control_effort_x;
	double control_effort_y;

	double imu_x;
	double imu_y;

	double positions[12];
};

stabilBody::stabilBody() {


	body_setpoint_x_ = nh_.advertise<std_msgs::Float64>("body_setpoint_x", 100);
	body_setpoint_y_ = nh_.advertise<std_msgs::Float64>("body_setpoint_y", 100);

	body_tilt_x_ = nh_.advertise<std_msgs::Float64>("body_tilt_x", 1);
	body_tilt_y_ = nh_.advertise<std_msgs::Float64>("body_tilt_y", 1);

	pid_x_ = nh_.subscribe("/tilt_x/control_effort", 1,
			&stabilBody::pidCallbackX, this);
	pid_y_ = nh_.subscribe("/tilt_y/control_effort", 1,
			&stabilBody::pidCallbackY, this);
	imu_sub_ = nh_.subscribe("imu/data_raw", 100,
			&stabilBody::imuCallback, this);

	body_tilt_x.data = 0;
	body_tilt_y.data = 0;

	control_effort_x = 0;
	control_effort_y = 0;

	imu_x = 0;
	imu_y = 0;
	for (char i = 0; i < 12; i++) {
		positions[i] = 0.0;
	}

}

void charCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard a message: \"%s\" ... data = %d", msg->data.c_str(),
			atoi(msg->data.c_str()));
}

void twistCallback(const geometry_msgs::Twist &twist) {
//	ROS_INFO("Got linear  twist data... X: %+4.3f Y: %+4.3f Z: %+4.3f", (float)twist.linear.x, (float)twist.linear.y, (float)twist.linear.z);
//	ROS_INFO("Got angular twist data... X: %+4.3f Y: %+4.3f Z: %+4.3f", (float)twist.angular.x, (float)twist.angular.y, (float)twist.angular.z);
//	ROS_INFO("----------------------");

	l_stick.x = twist.linear.x;
	l_stick.y = twist.linear.y;
	l_stick.z = twist.linear.z;

	r_stick.x = twist.angular.x;
	r_stick.y = twist.angular.y;
	r_stick.z = twist.angular.z;
}

void stabilBody::imuCallback(const sensor_msgs::Imu::ConstPtr &imu) {
	ROS_INFO("Control listener got IMU Data");
	//ROS_INFO("IMU Callback");

	imu_x = imu->orientation.x;
	imu_y = imu->orientation.y;
	ROS_INFO("IMU: X Y = %f \t %f", imu_x, imu_y);

	body_tilt_x.data = atan(imu->orientation.x / imu->orientation.z);
	body_tilt_y.data = atan(imu->orientation.y / imu->orientation.z);
	body_tilt_x_.publish(body_tilt_x);
	body_tilt_y_.publish(body_tilt_y);
	ROS_INFO("Publishing tilt X Y: %f \t %f", body_tilt_x.data, body_tilt_y.data);

}

void stabilBody::pidCallbackX(const std_msgs::Float64 &val) {
	control_effort_x = val.data;
	ROS_INFO("PID effort X = %f", control_effort_x);
}

void stabilBody::pidCallbackY(const std_msgs::Float64 &val) {
	control_effort_y = val.data;
	ROS_INFO("PID effort X = %f", control_effort_y);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "control_listener");
	ROS_INFO("Starting node: control_listener");
	ROS_INFO("Testing Matlab calls...");

	ROS_INFO("Testing simpleLegAngle...");
	simpleLegAngle_initialize();
	test_simpleLegAngle();

	ROS_INFO("Testing tiltBalance...");
	test_tiltBalance();

	// --------------------

	stabilBody stabil;

	ros::NodeHandle n;
	ros::Subscriber char_sub_ = n.subscribe("remote_cmd_char", 1000,
			charCallback);
	ros::Subscriber twist_sub_ = n.subscribe("cmd_vel", 1000, twistCallback);

	ros::NodeHandle clientNode;
	ros::ServiceClient servo_control =
			clientNode.serviceClient<stabil::PCA9685>("pca9685");

	srv.request.address[0] = 0;
	srv.request.address[1] = 2;
	srv.request.address[2] = 4;
	srv.request.address[3] = 6;
	int pwm_req[8];

	double offset[3] = { 0.0, 0.0, 0.0 };
	double ground[4] = { 0.0, 0.0, 0.0 };
	double thetas[4] = { 0.0, 0.0, 0.0, 0.0 };
	double efforts[4] = { 0.0, 0.0, 0.0, 0.0 };

	double jsx;
	double jsy;
	double bias_jsx;
	double bias_jsy;

	std_msgs::Float64 spx;
	std_msgs::Float64 spy;

	// ---------------------------------------
	ROS_INFO("Starting main loop");

	ros::Rate loop_rate(50);
	int counter = 0;

	while (ros::ok()) {
		//ROS_INFO("Spun once: Loop start.");
		jsx = fabs(l_stick.x) < 0.05 ? 0 : l_stick.x;
		jsy = fabs(l_stick.y) < 0.05 ? 0 : l_stick.y;
		jsx = atan(jsx / 6.0 / 1.7320508075688772);
		jsy = atan(jsy / 6.0 / 1.7320508075688772);


		bias_jsx = -1 * stabil.control_effort_x;
		bias_jsy = -1 * stabil.control_effort_y;

		// Get "off the ground" metric acting as a dead-man switch
		offset[2] = 8 + (-8 * r_stick.z);

		// Limit calculation rates, but continue servo smoothing
		counter++;
		if (!((counter + 1) % 2)) {
			//ROS_INFO("Calculations entered");
			cg_calcEfforts(bias_jsx, bias_jsy, efforts);

			cg_calcPose(bias_jsx, bias_jsy,
					offset, ground, thetas, stabil.positions);


//			cg_calcEfforts(stabil.imu_x, stabil.imu_y, efforts);
//
//			cg_calcPose(stabil.control_effort_x, stabil.control_effort_x,
//					offset, ground, thetas, stabil.positions);

		}

//		ROS_INFO("Thetas: %f, %f, %f, %f",
//			thetas[0], thetas[1], thetas[2], thetas[3] );
//		for (int i = 0; i < 4; i++) {
//			ROS_INFO("Contact %d: (%f, %f, %f)", i,
//				stabil.positions[0], stabil.positions[1],
//				stabil.positions[2], stabil.positions[3]);
//		}


		if (!(counter % 2)) {
			//ROS_INFO("Servo operations entered");
			servo_fl.setGoalAngle(thetas[0]);
			servo_fr.setGoalAngle(thetas[1]);
			servo_rl.setGoalAngle(thetas[2]);
			servo_rr.setGoalAngle(thetas[3]);

			servo_fl.calcNext();
			servo_fr.calcNext();
			servo_rl.calcNext();
			servo_rr.calcNext();

			servo_fl.sendNext();
			servo_fr.sendNext();
			servo_rl.sendNext();
			servo_rr.sendNext();


			if (servo_control.call(srv)) {
				//ROS_INFO("Success, sent %d : %d, got: %d", (int)srv.request.address, (int)srv.request.request, (int)srv.response.response);
			} else {
				//ROS_ERROR("Failed to call service servo_control");
				// TODO PUT BACK IN!

			}
		}

		//ROS_INFO("Servos sent. Iteration: %d", counter);

		spx.data = 0;			  //atan(jsx / 6.0 / 1.7320508075688772);
		spy.data = 0;			  //atan(jsy / 6.0 / 1.7320508075688772);

		stabil.body_setpoint_x_.publish(spx);
		stabil.body_setpoint_y_.publish(spy);

		ros::spinOnce();
		loop_rate.sleep();
	}

	simpleLegAngle_terminate();
	return 0;
}
