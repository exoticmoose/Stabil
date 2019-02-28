#include "rt_nonfinite.h"
#include "limbPose.h"
#include "simpleLegAngle.h"
#include "simpleLegAngle_terminate.h"
#include "simpleLegAngle_initialize.h"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "stabil/QuadFloat.h"
#include "stabil/AttitudeControl.h"

/* Function Declarations */
static double argInit_real_T();
static void main_simpleLegAngle();

/* Function Definitions */


static void main_simpleLegAngle()
{
  double dv0[3] = {0.0, 0.0, 7.0};
  double dv1[4] = {0.0, 0.0, 0.0, 0.0};
  double leg = 7.0;
  double radius = 1.0;
  double theta[4];
  double pos[12];

  /* Initialize function 'simpleLegAngle' input arguments. */
  /* Initialize function input argument 'offset_z'. */
  /* Initialize function input argument 'ground_z'. */
  /* Call the entry-point 'simpleLegAngle'. */

  simpleLegAngle(0.0, 0.0, dv0, dv1, leg, radius, theta, pos);

  ROS_INFO("Thetas = %f, %f, %f, %f", theta[0], theta[1], theta[2], theta[3]);
}

bool calcPose(stabil::AttitudeControl::Request &req, stabil::AttitudeControl::Response &res) {
	ROS_INFO("Got data");
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
		ROS_INFO("Corner: %d (x: %f, y:%f, z:%f)", i, tmp.x, tmp.y, tmp.z);
		res.contact[i] = tmp;
	}

	return true;
}
int main(int argc, char** argv)
{
  simpleLegAngle_initialize();


  ros::init(argc, argv, "limb_pose");

  ros::NodeHandle n;
  ros::ServiceServer serve = n.advertiseService("limb_pose", calcPose);


  main_simpleLegAngle();

  ros::spin();

  simpleLegAngle_terminate();
  return 0;
}
