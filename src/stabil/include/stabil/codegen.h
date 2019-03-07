#ifndef CODEGEN_H
#define CODEGEN_H

#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "simpleLegAngle.h"
#include "simpleLegAngle_initialize.h"
#include "simpleLegAngle_terminate.h"

#include "tiltBalance.h"

#include "geometry_params.h"

void test_simpleLegAngle();
void test_tiltBalance();

void cg_calcPose(const double x, const double y, const double offset[], const double ground[], double theta[], double position[]);
void cg_calcEfforts(const double x, const double y, double res[]);


#endif
