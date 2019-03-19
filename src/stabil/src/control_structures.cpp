#include <cstdlib>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <vector>
#include "ros/ros.h"
#include "servo.h"
#include "control_listener.h"
#include "control_structures.h"
#include "codegen.h"

extern float proximity_last;
extern bool proximity;


remoteController::remoteController() {
	fault = false;
	remoteActive = false;
	controlActive = false;

	
	char_sub_ = nh_.subscribe("remote_cmd_char", 10,
		&remoteController::charCallback, this);
}

void remoteController::parseCommand(char* cmd_string) {
	if (cmd_string) {
		ROS_INFO("Processing your command string: %s", cmd_string);
		char cmd[100] = {0};
		strcpy(cmd, cmd_string);
		ROS_INFO("Processing c string: %s", cmd);
		
		char* arg1 = strchr(cmd, ' ');
		char* arg2 = NULL;
		char* arg3 = NULL;
		if (arg1) {
			*arg1 = 0;
			arg1 += 1;
			ROS_INFO("Arg1 and beyond: %s" , arg1);
			arg2 = strchr(arg1, ' ');
			if (arg2) {
				*arg2 = 0;
				arg2 += 1;
				ROS_INFO("Arg2 and beyond: %s" , arg2);
				arg3 = strchr(arg2, ' ');
				if (arg3) {
					*arg3 = 0;
					arg3 += 1;
					ROS_INFO("Arg3 and beyond: %s" , arg3);
				}
			}
		}
		if (arg1) ROS_INFO("arg1 = %s", arg1);
		if (arg2) ROS_INFO("arg2 = %s", arg2);
		if (arg3) ROS_INFO("arg3 = %s", arg3);
		if (!strcmp(cmd, "activate")) {
			// If activate is called...
			ROS_INFO("Got activate!!!!");
			this->remoteActive = true;
		}
		else if (!strcmp(cmd, "release")) {
			// If release is called..
			ROS_INFO("Deactivating...");
			this->remoteActive = false;
		}
		else if (!strcmp(cmd, "move")) {
			// If release is called..
			float tmpTime = atof(arg1);
			ROS_INFO("Got time: %f", tmpTime);
			remoteInstruction tmpInst = NO_OP; 
			if (tmpTime > 0) {
				tmpInst = FORWARD_DRIVE;
				this->addInstruction(tmpInst, tmpTime);
			}
			else if (tmpTime < 0) {
				tmpInst = REVERSE_DRIVE;
				this->addInstruction(tmpInst, fabs(tmpTime));
			}
			else ROS_INFO("No valid command");
			
		}
		
	} else ROS_INFO("CMD string must be provided? Something is wrong here...");
}

float remoteController::getNextThrottle() {
	//ROS_INFO("Getting throttle");
	float throttle = 0;
	remoteInstruction nextInst = NO_OP; 
	float timing = 0;
	
	if (cmdInstruction.empty()) {
		ROS_INFO("Command queue empty");
	} else {
		nextInst = cmdInstruction.front();
		//ROS_INFO("Got Inst");
		if (cmdTiming.empty()) {
			ROS_INFO("Timing queue empty");
		} else {
			timing = cmdTiming.front();
			//ROS_INFO("Got timing");
			if (timing < 0.6) {
				// End of command
				//ROS_INFO("Erasing");
				cmdTiming.erase(cmdTiming.begin()); // TODO gross!!!
				cmdInstruction.erase(cmdInstruction.begin()); // TODO gross!!!
				//ROS_INFO("Finished rasing");
			}
			 else cmdTiming.at(0) = timing - 0.5;
			
			switch (nextInst) {
				case FORWARD_DRIVE:
					throttle = 0.5;
					break;
					
				case REVERSE_DRIVE:
					throttle = -0.5;
					break;
					
				case NO_OP:
					throttle = 0;
					ROS_INFO("Reached end of remote inst. queue");
					
				default:
					throttle = 0;
					ROS_INFO("Should not occur!");
			}	
		}
	}
	

	
	
	
	
	
	return throttle;
}



void remoteController::addInstruction(remoteInstruction inst, float time) {
	cmdTiming.push_back(time);
	cmdInstruction.push_back(inst);
	ROS_INFO("Pushed back instructions");	
}

void remoteController::charCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard a message: \"%s\" ... data = %d", msg->data.c_str(),
			atoi(msg->data.c_str()));
	char tmp[100];
	strcpy(tmp, msg->data.c_str());
	this->parseCommand(tmp);
}

stabilBody::stabilBody() {

	body_setpoint_x_ = nh_.advertise<std_msgs::Float64>("body_setpoint_x", 100);
	body_setpoint_y_ = nh_.advertise<std_msgs::Float64>("body_setpoint_y", 100);

	body_tilt_x_ = nh_.advertise<std_msgs::Float64>("body_tilt_x", 1);
	body_tilt_y_ = nh_.advertise<std_msgs::Float64>("body_tilt_y", 1);

	pid_x_ = nh_.subscribe("/tilt_x/control_effort", 1,
			&stabilBody::pidCallbackX, this);
	pid_y_ = nh_.subscribe("/tilt_y/control_effort", 1,
			&stabilBody::pidCallbackY, this);
	imu_sub_ = nh_.subscribe("imu/data_raw", 1,
			&stabilBody::imuCallback, this);
	cv_sub_ = nh_.subscribe("/rosObjectHolder", 1,
			&stabilBody::cvCallback, this);


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




void stabilBody::cvCallback(const stabil::rosObjectHolder::ConstPtr &msg) {
	ros::WallTime tmp;
	
	ROS_INFO("Heard from CV");
	
	for (int i = 0; i < msg->data.size(); i++) {
		const stabil::rosObject &tmp_cv = msg->data[i];
		//ROS_INFO("Got an object...");
		ROS_INFO("Distance = %f", tmp_cv.distance);
		
		if (tmp_cv.classifier == "person") {
			if (tmp_cv.distance < 3.0 || tmp_cv.critical) {
				this->proximityCounter = 0;
				ROS_INFO("Critical object detected");
				proximity = true;
				proximity_last = 0;

			} else {
				proximityCounter++;
				if (proximityCounter > 20) {
					proximity = false;
					ROS_INFO("Proximity timeout: CV sucks");
				}
			} 
		}
		
		
	}
	

}

void stabilBody::imuCallback(const sensor_msgs::Imu::ConstPtr &imu) {
	//ROS_INFO("Control listener got IMU Data");
	//ROS_INFO("IMU Callback");

	double w_sign = std::signbit(imu->orientation.w) ? -1.0f : 1.0f;
	imu_x = imu->orientation.x;
	imu_y = imu->orientation.y;

	ROS_INFO("IMU: X Y = %f \t %f", imu_x, imu_y);

	body_tilt_x.data = asin(imu->orientation.x * w_sign) * 10;
	body_tilt_y.data = asin(imu->orientation.y * w_sign) * 10;
	body_tilt_x_.publish(body_tilt_x); // scaling for PID
	body_tilt_y_.publish(body_tilt_y);
	ROS_INFO("Publishing tilt X Y: %f \t %f", body_tilt_x.data, body_tilt_y.data);

}

void stabilBody::pidCallbackX(const std_msgs::Float64 &val) {
	control_effort_x = val.data;
	ROS_INFO("PID effort X = %f", control_effort_x);
}

void stabilBody::pidCallbackY(const std_msgs::Float64 &val) {
	control_effort_y = val.data;
	ROS_INFO("PID effort Y = %f", control_effort_y);
}
