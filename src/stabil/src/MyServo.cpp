#include <proj_parms.h>
#include <MyServo.h>

void MyServo::heartbeat() {
	Serial.println("I'm Here!");
}



void MyServo::calcNext() {
			//Serial.print("calcNext Called - curr_pw vs. goal_pw: ");
			//Serial.print(curr_pw); Serial.print("\t"); Serial.println(goal_pw);
			


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
				else {
					// - - - - - - -
					Serial.print("Error - curr_pw vs. goal_pw: ");
					Serial.print(curr_pw); Serial.print("\t"); Serial.println(goal_pw);
					// - - - - - - -
				}
			}
			// else, we are already where we want to be.
		}

void MyServo::sendNext() {
			
			if (curr_pw != next_pw) {
				Serial.print("Setting PWM on \t"); Serial.print(my_id); 
				Serial.print("\t to: \t"); Serial.println(next_pw);
				servo_pwm.setPWM(my_id, 0, next_pw);
				curr_pw = next_pw;
			}
			
			
	}
//
unsigned short MyServo::getGoal() {return goal_pw;}
unsigned short MyServo::getNext() {return next_pw;}
unsigned short MyServo::getCurr() {return curr_pw;}

void MyServo::setGoalAngle(float angle) {
	goal_angle = angle;
	setGoal(my_direction ? my_max - (angle * degree_pw) : my_min + (angle * degree_pw));

}

void MyServo::setGoal(unsigned short new_goal) {
	if (new_goal > my_max) goal_pw = my_max;
	else if (new_goal < my_min) goal_pw = my_min;
	else goal_pw = new_goal;
}