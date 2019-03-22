#ifndef CONTROL_STRUCTURES_H
#define CONTROL_STRUCTURES_H
#include <vector>

enum remoteInstruction {
	NO_OP,
	FORWARD_DRIVE,
	REVERSE_DRIVE
};

class stabilBody {

	private:
		void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
		void pidCallbackX(const std_msgs::Float64 &val);
		void pidCallbackY(const std_msgs::Float64 &val);
		void cvCallback(const stabil::rosObjectHolder::ConstPtr  &data);
		

		ros::NodeHandle nh_;
		ros::Subscriber imu_sub_;
		ros::Subscriber pid_x_;
		ros::Subscriber pid_y_;
		ros::Subscriber cv_sub_;
		int proximityCounter;

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

		bool flagCriticalDistance;
		
		double positions[12];
};

class remoteController {
	public:
		remoteController();
		bool controlActive;
		bool remoteActive;
		bool proximity;
		bool fault;
		
		
		
		
		void parseCommand(char* cmd_string);
		
		float getNextThrottle();
		
		
		
		std::vector<float> cmdTiming;
		std::vector<remoteInstruction> cmdInstruction;
		
	private: 
		void addInstruction(remoteInstruction inst, float time);
		
		
		
		ros::NodeHandle nh_;
		void charCallback(const std_msgs::String::ConstPtr &msg);
		ros::Subscriber char_sub_;
};



#endif