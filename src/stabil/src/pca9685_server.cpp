#include "ros/ros.h"
#include "stabil/PCA9685.h"
#include "JHPWMPCA9685.h"

PCA9685 *pca9685;




bool setServo(stabil::PCA9685::Request &req, stabil::PCA9685::Response &res) {
	ROS_INFO("Got request to set servo %d to %d", req.address[0], req.request[0]);
	ROS_INFO("Got request to set servo %d to %d", req.address[1], req.request[1]);
	ROS_INFO("Got request to set servo %d to %d", req.address[2], req.request[2]);
	ROS_INFO("Got request to set servo %d to %d", req.address[3], req.request[3]);
	ROS_INFO("--------------------");
	pca9685->setPWM((int)req.address[0], 0, (int)req.request[0]);
	pca9685->setPWM((int)req.address[1], 0, (int)req.request[1]);
	pca9685->setPWM((int)req.address[2], 0, (int)req.request[2]);
	pca9685->setPWM((int)req.address[3], 0, (int)req.request[3]);


	return true;

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pca9685_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("pca9685", setServo);

	pca9685 = new PCA9685();
	int err = pca9685->openPCA9685();
	if (err < 0) {
		ROS_ERROR("Unable to open PCA9685 Connection");
	}
	else {
		  pca9685->setAllPWM(0,0);
		  pca9685->reset();
		  pca9685->setPWMFrequency(250);
		  ROS_INFO("PCA9685 Connection Successful.");
	}

	ROS_INFO("Ready to roll!");
	ros::spin();

	return 0;
}
