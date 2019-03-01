#include "rt_nonfinite.h"
#include "simpleLegAngle.h"
#include "simpleLegAngle_terminate.h"
#include "simpleLegAngle_initialize.h"
#include "tiltBalance.h"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "stabil/QuadFloat.h"
#include "stabil/AttitudeControl.h"
#include "stabil/IMUEffort.h"

#define LEG_LENGTH 11.5
#define WHEEL_RADIUS 0.01
#define BODY_DIM_X 24.3
#define BODY_DIM_Y 20.3



void main_simpleLegAngle()
{
  double body_offset[3] = {0.0, 0.0, 7.0};
  double ground_z[4] = {0.0, 0.0, 0.0, 0.0};
  double leg = LEG_LENGTH;
  double radius = WHEEL_RADIUS;
  double theta[4];
  double pos[12];

  /* Initialize function 'simpleLegAngle' input arguments. */
  /* Initialize function input argument 'offset_z'. */
  /* Initialize function input argument 'ground_z'. */
  /* Call the entry-point 'simpleLegAngle'. */

  simpleLegAngle(0.0, 0.0, body_offset, ground_z, leg, radius, theta, pos);

  ROS_INFO("Basic Thetas = %f, %f, %f, %f", theta[0], theta[1], theta[2], theta[3]);
}

void main_tiltBalance() {
	double x = 0;
	double y = 1;
	double z = 0;
	double Dx = BODY_DIM_X;
	double Dy = BODY_DIM_Y;

	double w0;
	double w1;
	double w2;
	double w3;

	tiltBalance(x, y, z, Dx, Dy, &w0, &w1, &w2, &w3);
	ROS_INFO("Basic efforts = %f, %f, %f, %f", w0, w1, w2, w3);

}

bool calcPose(stabil::AttitudeControl::Request &req, stabil::AttitudeControl::Response &res) {
	//ROS_INFO("Got data");
	double offset[3] = {req.offset.x, req.offset.y, req.offset.z};
	double ground[4] = {req.ground.f0, req.ground.f1, req.ground.f2, req.ground.f3};
	double leg = 8;
	double radius = 1;
	double theta[4];
	double pos[12];

	simpleLegAngle(req.jx, req.jy, offset, ground, leg, radius, theta, pos);
	ROS_INFO("Thetas = %f, %f, %f, %f", theta[0], theta[1], theta[2], theta[3]);

	res.theta.f0 = theta[0];
	res.theta.f1 = theta[1];
	res.theta.f2 = theta[2];
	res.theta.f3 = theta[3];

	geometry_msgs::Point tmp;
	for (uint8_t i = 0; i < 4; i++) {
		tmp.x = pos[i];
		tmp.y = pos[i + 4];
		tmp.z = pos[i + 8];
		//ROS_INFO("Corner: %d (x: %f, y:%f, z:%f)", i, tmp.x, tmp.y, tmp.z);
		res.contact[i] = tmp;
	}

	return true;
}

bool calcGround(stabil::IMUEffort::Request &req, stabil::IMUEffort::Response &res) {

	tiltBalance(req.x, req.y, 0, BODY_DIM_X, BODY_DIM_Y, &res.w.f0, &res.w.f1, &res.w.f2, &res.w.f3 );

	ROS_INFO("(%f, %f) = Efforts: %f \t %f \t %f \t %f", req.x, req.y, res.w.f0, res.w.f1, res.w.f2, res.w.f3);
	return true;
}


int main(int argc, char** argv)
{
  simpleLegAngle_initialize();


  ros::init(argc, argv, "limb_pose");

  ros::NodeHandle n;
  ros::ServiceServer limb_pose = n.advertiseService("limb_pose", calcPose);
  ros::ServiceServer ground_calc = n.advertiseService("ground_calc", calcGround);
  //ros::Subscriber imuProcess



  main_simpleLegAngle();
  main_tiltBalance();

  ros::spin();

  simpleLegAngle_terminate();
  return 0;
}
