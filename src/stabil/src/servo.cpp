#define BIG_STEP 60
#define SMALL_STEP 10

#include "ros/ros.h"


#include "servo.h"

#include "stabil/PCA9685.h"

extern stabil::PCA9685 srv;

void servo::heartbeat() {
	//Serial.println("I'm Here!");
	ROS_INFO("I'm alive");
}



void servo::calcNext() {

			if (curr_pw != goal_pw) {
				// If undershooting goal, move closer by BIG_STEP until starting to move by 10 when close
				if (curr_pw < goal_pw) {
					if (curr_pw + BIG_STEP < goal_pw) next_pw = curr_pw + BIG_STEP;
					else if (curr_pw + SMALL_STEP < goal_pw) next_pw = curr_pw + SMALL_STEP;
					else next_pw = goal_pw;
				}
				else if (curr_pw > goal_pw) {
					if (curr_pw - BIG_STEP > goal_pw) next_pw = curr_pw - BIG_STEP;
					else if (curr_pw - SMALL_STEP > goal_pw) next_pw = curr_pw - SMALL_STEP;
					else next_pw = goal_pw;
				}
			}
		}

void servo::sendNext() {
			
			if (curr_pw != next_pw) {
				srv.request.request[my_id / 2] = next_pw; //TODO address <--> id mapping
				//ROS_INFO("Sending next=%d for %d", next_pw, my_id);
				curr_pw = next_pw;
			}
	}
//
unsigned short servo::getGoal() {return goal_pw;}
unsigned short servo::getNext() {return next_pw;}
unsigned short servo::getCurr() {return curr_pw;}

void servo::setGoalAngle(float angle) {
	goal_angle = angle;
	setGoal(my_direction ? my_max - (angle * angular_pw) : my_min + (angle * angular_pw));

}

void servo::setGoal(unsigned short new_goal) {
	if (new_goal > my_max) goal_pw = my_max;
	else if (new_goal < my_min) goal_pw = my_min;
	else goal_pw = new_goal;
}
