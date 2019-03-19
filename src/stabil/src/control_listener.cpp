#include <cstdlib>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "servo.h"
#include "control_listener.h"
#include "control_structures.h"
#include "codegen.h"

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

bool proximity;
float proximity_last = 999999999999999999;



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
	remoteController rc;


	ros::NodeHandle n;
	ros::Subscriber twist_sub_ = n.subscribe("cmd_vel", 1000, twistCallback);

	ros::NodeHandle clientNode;
	ros::ServiceClient servo_control =
			clientNode.serviceClient<stabil::PCA9685>("pca9685");

	srv.request.address[0] = 0;
	srv.request.address[1] = 2;
	srv.request.address[2] = 4;
	srv.request.address[3] = 6;

	srv.request.address[4] = 8;
	srv.request.address[5] = 10;
	srv.request.address[6] = 12;
	srv.request.address[7] = 14;

	srv.request.throttle = 0;
	int pwm_req[8];

	double offset[3] = { 0.0, 0.0, 0.0 };
	double ground[4] = { 0.0, 0.0, 0.0 };
	double thetas[4] = { 0.0, 0.0, 0.0, 0.0 };
	double efforts[4] = { 0.0, 0.0, 0.0, 0.0 };

	double jsx;
	double jsy;
	double bias_jsx;
	double bias_jsy;

	double throttle;
	int diff_left;
	int diff_right;
	
	ros::WallTime time;

	std_msgs::Float64 spx;
	std_msgs::Float64 spy;

	// ---------------------------------------
	ROS_INFO("Starting main loop");

	ros::Rate loop_rate(40);
	int counter = 0;
	int lastObj = 0;

	while (ros::ok()) {
		
		

		// Limit calculation rates, but continue servo smoothing
		counter++;
		if (proximity) lastObj++;
		else lastObj = 0;
		if (!((counter + 1) % 1)) {
			//ROS_INFO("Calculations entered");
			if (rc.remoteActive) {
				//ROS_INFO("Remote control activate...");
				jsx = 0;
				jsy = 0;
				bias_jsx = 0 - stabil.control_effort_x;
				bias_jsy = 0 - stabil.control_effort_y;
				offset[2] = 18;
				throttle = rc.getNextThrottle();
				//ROS_INFO("Got throttle!");
				
			}
			else {
				jsx = fabs(l_stick.x) < 0.1 ? 0 : (l_stick.x > 0 ? l_stick.x - 0.1 : l_stick.x + 0.1) * 0.50;
				jsy = fabs(l_stick.y) < 0.1 ? 0 : (l_stick.y > 0 ? l_stick.y - 0.1 : l_stick.y + 0.1) * 0.50;
				
				bias_jsx = jsx - stabil.control_effort_x;
				bias_jsy = jsy - stabil.control_effort_y;	
				
				throttle = fabs(r_stick.y) < 0.05 ? 0 : (r_stick.y > 0 ? r_stick.y - 0.05 : r_stick.y + 0.05);	

				// Get "off the ground" metric acting as a dead-man switch
				offset[2] = 12 + (-12 * r_stick.z);		
				
			}
			
			
			proximity_last++;
			if (proximity && proximity_last > 100) {
				proximity = false;
				ROS_INFO("last timeout");
				//ROS_INFO("Timer prximity timout: %f versus %f", time.toSec(), proximity_last);
			}
			
			
			
			
			if (proximity) {
				ROS_INFO("Limiting throttle due to proximity");
				if (throttle > 0) throttle = 0;
			} //else ROS_INFO("Not limiting throttle!")
			
			
			int tmpThrottle = fabs(throttle) * 4000;
			bias_jsx = atan(bias_jsx / 2);
			bias_jsy = atan(bias_jsy / 2);
			
			if (jsx > 0) {
				diff_left = tmpThrottle  - 7000 * jsx;
				diff_right = tmpThrottle;
			}
			else {
				diff_right = tmpThrottle + 7000 * jsx;
				diff_left = tmpThrottle;
			}
			
			
			cg_calcEfforts(bias_jsx, bias_jsy, efforts);

			cg_calcPose(bias_jsx, bias_jsy,
					offset, ground, thetas, stabil.positions);


//			cg_calcEfforts(stabil.imu_x, stabil.imu_y, efforts);
//
//			cg_calcPose(stabil.control_effort_x, stabil.control_effort_x,
//					offset, ground, thetas, stabil.positions);

		}


		if (!(counter % 1)) {
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

			srv.request.throttle = throttle * -1;
			srv.request.request[4] = diff_left;
			srv.request.request[5] = diff_left;
			srv.request.request[6] = diff_right;
			srv.request.request[7] = diff_right;


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
		//ROS_INFO("Publishing setpoints: %f \t %f", spx.data, spy.data);

		ros::spinOnce();
		loop_rate.sleep();
	}

	simpleLegAngle_terminate();
	return 0;
}
