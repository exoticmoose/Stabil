/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * simpleLegAngle.cpp
 *
 * Code generation for function 'simpleLegAngle'
 *
 */

/* Include files */
#include <cmath>
#include <math.h>
#include "rt_nonfinite.h"
#include "simpleLegAngle.h"

/* Function Definitions */
void simpleLegAngle(double jsx, double jsy, const double offset_z[3], const
                    double ground_z[4], double theta[4], double pos[12])
{
  double body_angle_y;
  double body_angle_x;
  double Rx[9];
  double Ry[9];
  int i0;
  static const signed char iv0[3] = { 1, 0, 0 };

  static const signed char iv1[3] = { 0, 1, 0 };

  int idx;
  double b_Rx[9];
  double bod_coords[12];
  int k;
  bool exitg1;
  double max_z;
  static const double b[12] = { 12.5, 10.25, 0.0, -12.5, 10.25, 0.0, -12.5,
    -10.25, 0.0, 12.5, -10.25, 0.0 };

  double N[3];
  double V_0[3];
  double U_0[3];
  double c_Rx[9];
  static const signed char b_b[3] = { 1, 0, 0 };

  static const signed char c_b[3] = { 0, 0, 1 };

  /* --------------------------------------------------- start setup */
  /* ------ start physical parms */
  /*  divide by 2 for center offset */
  /* ----- end physical parms */
  /* ----- declared variables */
  /* ----- end variables */
  /* --------------------------------------------------- end setup */
  /*  get desired body plane orientation */
  body_angle_y = std::atan(jsx / 2.0 / 1.7320508075688772);
  body_angle_x = std::atan(jsy / 2.0 / 1.7320508075688772);

  /* x and y angles  */
  Rx[1] = 0.0;
  Rx[4] = std::cos(body_angle_x);
  Rx[7] = -std::sin(body_angle_x);
  Rx[2] = 0.0;
  Rx[5] = std::sin(body_angle_x);
  Rx[8] = std::cos(body_angle_x);
  Ry[0] = std::cos(body_angle_y);
  Ry[3] = 0.0;
  Ry[6] = std::sin(body_angle_y);
  for (i0 = 0; i0 < 3; i0++) {
    Rx[3 * i0] = iv0[i0];
    Ry[1 + 3 * i0] = iv1[i0];
  }

  Ry[2] = -std::sin(body_angle_y);
  Ry[5] = 0.0;
  Ry[8] = std::cos(body_angle_y);
  for (i0 = 0; i0 < 3; i0++) {
    for (idx = 0; idx < 3; idx++) {
      b_Rx[i0 + 3 * idx] = 0.0;
      for (k = 0; k < 3; k++) {
        b_Rx[i0 + 3 * idx] += Rx[i0 + 3 * k] * Ry[k + 3 * idx];
      }
    }

    for (idx = 0; idx < 4; idx++) {
      bod_coords[i0 + 3 * idx] = 0.0;
      for (k = 0; k < 3; k++) {
        bod_coords[i0 + 3 * idx] += b_Rx[i0 + 3 * k] * b[k + 3 * idx];
      }
    }
  }

  /*  bod_coords = Ry * bod_coords */
  for (i0 = 0; i0 < 4; i0++) {
    bod_coords[2 + 3 * i0] += offset_z[2];
  }

  if (!rtIsNaN(bod_coords[2])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!rtIsNaN(bod_coords[2 + 3 * (k - 1)])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    max_z = bod_coords[2];
  } else {
    max_z = bod_coords[2 + 3 * (idx - 1)];
    while (idx + 1 < 5) {
      if (max_z < bod_coords[2 + 3 * idx]) {
        max_z = bod_coords[2 + 3 * idx];
      }

      idx++;
    }
  }

  body_angle_x = 8.0 * std::cos(body_angle_y) + 3.5;
  if (max_z > body_angle_x) {
    max_z -= body_angle_x;
    for (i0 = 0; i0 < 4; i0++) {
      bod_coords[2 + 3 * i0] -= max_z;
    }
  }

  /*  leg position calculations */
  /*  vector Normal to circle's plane */
  for (i0 = 0; i0 < 3; i0++) {
    N[i0] = 0.0;
    U_0[i0] = 0.0;
    for (idx = 0; idx < 3; idx++) {
      b_Rx[i0 + 3 * idx] = 0.0;
      for (k = 0; k < 3; k++) {
        b_Rx[i0 + 3 * idx] += Rx[i0 + 3 * k] * Ry[k + 3 * idx];
      }

      c_Rx[i0 + 3 * idx] = 0.0;
      for (k = 0; k < 3; k++) {
        c_Rx[i0 + 3 * idx] += Rx[i0 + 3 * k] * Ry[k + 3 * idx];
      }

      N[i0] += b_Rx[i0 + 3 * idx] * (double)b_b[idx];
      U_0[i0] += c_Rx[i0 + 3 * idx] * (double)c_b[idx];
    }
  }

  /*  circle initial radius unitvector, i.e. theta=0  */
  V_0[0] = N[1] * U_0[2] - N[2] * U_0[1];
  V_0[1] = N[2] * U_0[0] - N[0] * U_0[2];
  V_0[2] = N[0] * U_0[1] - N[1] * U_0[0];

  /*  vector orthogonal to get second vector from circle center to point on circle */
  for (k = 0; k < 4; k++) {
    if (1 + k < 3) {
      for (idx = 0; idx < 3; idx++) {
        N[idx] = -V_0[idx];
      }
    } else {
      for (idx = 0; idx < 3; idx++) {
        N[idx] = V_0[idx];
      }
    }

    body_angle_x = std::acos(((3.5 + ground_z[k]) - bod_coords[2 + 3 * k]) / 8.0
      / std::sqrt(U_0[2] * U_0[2] + -N[2] * -N[2])) + -std::atan(-N[2] / U_0[2]);
    body_angle_y = 8.0 * std::cos(body_angle_x);
    max_z = 8.0 * std::sin(body_angle_x);
    for (i0 = 0; i0 < 3; i0++) {
      pos[k + (i0 << 2)] = (bod_coords[i0 + 3 * k] + body_angle_y * U_0[i0]) +
        max_z * N[i0];
    }

    theta[k] = body_angle_x;
  }

  /*  end leg calculations */
  /* --------------------------------------------------- end calculation */
  /*  Start pose viz */
  /*  zero_plane = [25 25 0; -25 25 0; -25 -25 0; 25 -25 0]'; */
  /*  max_plane = [20 25 L+r; -20 25 L+r; -20 -25 L+r; 20 -25 L+r]'; */
  /*  wheel_plane = [K(1, 1) K(1, 2) r; K(2, 1) K(2, 2) r; K(3, 1) K(3, 2) r; K(4, 1) K(4, 2) r]'; */
  /*  wheel_plane(3, :) = wheel_plane(3, :) + ground_z */
  /*   */
  /*  clf */
  /*  fill3(bod_coords(1, :), bod_coords(2, :), bod_coords(3, :), 'r') */
  /*  hold on */
  /*   */
  /*  floor_limit = fill3(zero_plane(1, :), zero_plane(2, :), zero_plane(3, :), 'b'); */
  /*  z_limit = fill3(max_plane(1, :), max_plane(2, :), max_plane(3, :), 'g'); */
  /*  alpha(z_limit, 0.2); */
  /*  wheel_limit = fill3(wheel_plane(1, :), wheel_plane(2, :), wheel_plane(3, :), 'g'); */
  /*  alpha(wheel_limit, 0.2); */
  /*   */
  /*  P(P==0)=NaN */
  /*  for i = 1:4 */
  /*      tmp = squeeze(P(i,:, :)); */
  /*      plot3(tmp(1,:), tmp(2,:), tmp(3,:)); */
  /*  end */
  /*  scatter3(K(:, 1), K(:, 2), K(:,3)) */
  /*   */
  /*  axis([-25, 25, -25, 25, 0, 50]) */
  /*  axis vis3d */
  /*  rotate3d on */
  /*   */
  /*  theta; */
  /*  pos = K; */
  /*   */
  /*  end */
}

/* End of code generation (simpleLegAngle.cpp) */
