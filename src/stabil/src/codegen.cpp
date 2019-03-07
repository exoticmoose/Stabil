#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "stabil/QuadFloat.h"

#include "rt_nonfinite.h"
#include "simpleLegAngle.h"
#include "simpleLegAngle_terminate.h"
#include "simpleLegAngle_initialize.h"
#include "tiltBalance.h"
//#include "stabil/AttitudeControl.h"
//#include "stabil/IMUEffort.h"

#include "codegen.h"



void test_simpleLegAngle()
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

  ROS_INFO("Test thetas = %f, %f, %f, %f", theta[0], theta[1], theta[2], theta[3]);
}

void test_tiltBalance() {
	double x = 0;
	double y = 1;
	double z = 0;
	double Dx = BODY_DIM_X;
	double Dy = BODY_DIM_Y;

	double w[4];

	tiltBalance(x, y, z, Dx, Dy, w);
	ROS_INFO("Test efforts = %f, %f, %f, %f", w[0], w[1], w[2], w[3]);

}

void cg_calcPose(const double x, const double y, const double offset[], const double ground[], double theta[], double position[]) {
	//ROS_INFO("Called calcPose");

	simpleLegAngle(x, y, offset, ground, LEG_LENGTH, WHEEL_RADIUS, theta, position);
	//ROS_INFO("Thetas = %f, %f, %f, %f", theta[0], theta[1], theta[2], theta[3]);

	geometry_msgs::Point tmp;
	for (uint8_t i = 0; i < 4; i++) {
		tmp.x = position[i];
		tmp.y = position[i + 4];
		tmp.z = position[i + 8];
		//ROS_INFO("Corner: %d (x: %f, y:%f, z:%f)", i, tmp.x, tmp.y, tmp.z);
	}

	return;
}

void cg_calcEfforts(const double x, const double y, double res[]) {

	tiltBalance(x, y, 0, BODY_DIM_X, BODY_DIM_Y, res);

	//ROS_INFO("(%f, %f) = Efforts: %f \t %f \t %f \t %f", x, y, res[0], res[1], res[2], res[3]);

	return;
}
