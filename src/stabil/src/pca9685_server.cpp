#include "ros/ros.h"
#include "stabil/PCA9685.h"

bool setServo(stabil::PCA9685::Request &req, stabil::PCA9685::Response &res) {
	ROS_INFO("Got request to set servo %d to %d", 69, 240);

	return true;



}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pca_9685_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("pca_9685", setServo);
	ROS_INFO("Ready to roll!");
	ros::spin();

	return 0;
}
