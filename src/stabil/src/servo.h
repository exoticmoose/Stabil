#ifndef MYSERVO_H
#define MYSERVO_H
// ------------------------------------------- MYSERVO_H



class servo {
	public:

		servo(unsigned char id, unsigned short max, unsigned short min, float arc, bool direction) {
			my_id = id;
			my_max = max;
			my_min = min;
			my_arc = arc;
			my_direction = direction;
			degree_pw = (my_max - my_min) / my_arc;
			goal_pw = 0;
			next_pw = 0;
			curr_pw = 0;
			goal_angle = 0;
			//ROS_INFO("Servo created.");
		}

		void heartbeat();

		void setGoal(unsigned short);
		void setGoalAngle(float);
		void calcNext();
		void sendNext();

		unsigned short getGoal();
		unsigned short getNext();
		unsigned short getCurr();


	private:
		unsigned short my_id;
		unsigned short my_max;
		unsigned short my_min;
		float my_arc; // total movement range in degrees
		bool my_direction;
		
		float degree_pw;

		float goal_angle;
		unsigned short goal_pw;
		unsigned short next_pw;
		unsigned short curr_pw;
};

// ------------------------------------------ \MYSERVO_H
#endif
