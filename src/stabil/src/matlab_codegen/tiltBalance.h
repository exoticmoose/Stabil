/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * tiltBalance.h
 *
 * Code generation for function 'tiltBalance'
 *
 */

#ifndef TILTBALANCE_H
#define TILTBALANCE_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "tiltBalance_types.h"

/* Function Declarations */
extern void tiltBalance(double x, double y, double z, double Dx, double Dy,
  double reaction[]);
extern void tiltBalance_initialize();
extern void tiltBalance_terminate();

#endif

/* End of code generation (tiltBalance.h) */
