/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.cpp
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include files */
#include "rt_nonfinite.h"
#include "simpleLegAngle.h"
#include "limbPose.h"
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

static double argInit_real_T()
{
  return 0.0;
}

static void main_simpleLegAngle()
{
  double dv0[3] = {0, 0, 0};
  double dv1[4] = {0, 0, 0, 0};
  double theta[4];
  double pos[12];

  /* Initialize function 'simpleLegAngle' input arguments. */
  /* Initialize function input argument 'offset_z'. */
  /* Initialize function input argument 'ground_z'. */
  /* Call the entry-point 'simpleLegAngle'. */

  simpleLegAngle(argInit_real_T(), argInit_real_T(), dv0, dv1, theta, pos);
  
  ROS_INFO("Thetas = %f, %f, %f, %f", theta[0], theta[1], theta[2], theta[3]);
}

bool calcPose(stabil::AttitudeControl::Request &req, stabil::AttitudeControl::Response &res) {
	ROS_INFO("Got data");
	double dv0[3] = {req.offset.x, req.offset.y, req.offset.z};
	double dv1[4] = {req.ground.f0, req.ground.f1, req.ground.f2, req.ground.f3};
	double theta[4];
	double pos[12];

	res.theta.f0 = theta[0];
	res.theta.f1 = theta[1];
	res.theta.f2 = theta[2];
	res.theta.f3 = theta[3];

	geometry_msgs::Point tmp;

	for (uint8_t i = 0; i < 10; i = i + 3) {
		tmp.x = pos[i];
		tmp.y = pos[i + 1];
		tmp.z = pos[i + 2];
		ROS_INFO("Corner %d: (x: %f, y:%f, z:%f)", i, tmp.x, tmp.y, tmp.z);
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

/* End of code generation (main.cpp) */
