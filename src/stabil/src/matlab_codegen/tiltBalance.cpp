/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * tiltBalance.cpp
 *
 * Code generation for function 'tiltBalance'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "tiltBalance.h"

/* Function Declarations */
static void linsolve(const double A[9], const double B[3], double C[3]);
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */

/*
 *
 */
static void linsolve(const double A[9], const double B[3], double C[3])
{
  double b_A[9];
  int r1;
  int r2;
  int r3;
  double maxval;
  double a21;
  int rtemp;
  memcpy(&b_A[0], &A[0], 9U * sizeof(double));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(A[0]);
  a21 = std::abs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[3 + r2] -= b_A[r2] * b_A[3 + r1];
  b_A[3 + r3] -= b_A[r3] * b_A[3 + r1];
  b_A[6 + r2] -= b_A[r2] * b_A[6 + r1];
  b_A[6 + r3] -= b_A[r3] * b_A[6 + r1];
  if (std::abs(b_A[3 + r3]) > std::abs(b_A[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[3 + r3] /= b_A[3 + r2];
  b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + r2];
  C[1] = B[r2] - B[r1] * b_A[r2];
  C[2] = (B[r3] - B[r1] * b_A[r3]) - C[1] * b_A[3 + r3];
  C[2] /= b_A[6 + r3];
  C[0] = B[r1] - C[2] * b_A[6 + r1];
  C[1] -= C[2] * b_A[6 + r2];
  C[1] /= b_A[3 + r2];
  C[0] -= C[1] * b_A[3 + r1];
  C[0] /= b_A[r1];
}

static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * function [w0, w1, w2, w3] = tiltBalance(x, y, z, Dx, Dy)
 */
void tiltBalance(double x, double y, double z, double Dx, double Dy, double *w0,
                 double *w1, double *w2, double *w3)
{
  double W[9];
  int i;
  static const signed char iv0[3] = { 0, 0, 1 };

  double Auw[9];
  static const double dv0[3] = { 1.0, 0.0, 0.0 };

  static const double dv1[3] = { 0.0, 1.0, 0.0 };

  static const double dv2[3] = { 0.0, 0.0, 1.0 };

  double b_x[3];
  double Vw[3];
  double alpha_w;
  int i0;
  double reaction[4];

  /*   */
  /*  x = -1; */
  /*  y = 1; */
  /*  z = 0; */
  /*  Get change of basis matrices */
  /* tic */
  /* 'tiltBalance:9' W = [ Dx -Dx 0; */
  /* 'tiltBalance:10'       Dy Dy 0; */
  /* 'tiltBalance:11'       0 0 1]; */
  W[0] = Dx;
  W[3] = -Dx;
  W[6] = 0.0;
  W[1] = Dy;
  W[4] = Dy;
  W[7] = 0.0;
  for (i = 0; i < 3; i++) {
    W[2 + 3 * i] = iv0[i];
  }

  /* 'tiltBalance:12' U = eye(3); */
  /* 'tiltBalance:13' Auw = zeros(3); */
  memset(&Auw[0], 0, 9U * sizeof(double));

  /* 'tiltBalance:14' Auw(:,1) = linsolve(W, U(:,1)); */
  linsolve(W, dv0, *(double (*)[3])&Auw[0]);

  /* 'tiltBalance:15' Auw(:,2) = linsolve(W, U(:,2)); */
  linsolve(W, dv1, *(double (*)[3])&Auw[3]);

  /* 'tiltBalance:16' Auw(:,3) = linsolve(W, U(:,3)); */
  linsolve(W, dv2, *(double (*)[3])&Auw[6]);

  /* 'tiltBalance:17' Awu = inv(Auw); */
  /* 'tiltBalance:19' Vu = [x y z]'; */
  /* 'tiltBalance:20' Vw = Auw * Vu; */
  b_x[0] = x;
  b_x[1] = y;
  b_x[2] = z;
  for (i = 0; i < 3; i++) {
    Vw[i] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      Vw[i] += Auw[i + 3 * i0] * b_x[i0];
    }
  }

  /* 'tiltBalance:22' alpha_w = atan2(Vw(2), Vw(1)); */
  alpha_w = rt_atan2d_snf(Vw[1], Vw[0]);

  /* if alpha_w < 0; alpha_w = 2*pi + alpha_w; end */
  /* 'tiltBalance:25' reaction = zeros(4,1); */
  /* 'tiltBalance:26' reaction(1) = (cos(alpha_w)); */
  reaction[0] = std::cos(alpha_w);

  /* 'tiltBalance:27' reaction(2) = (sin(alpha_w)); */
  reaction[1] = std::sin(alpha_w);

  /* 'tiltBalance:28' reaction(3) = -(cos(alpha_w)); */
  reaction[2] = -std::cos(alpha_w);

  /* 'tiltBalance:29' reaction(4) = -(sin(alpha_w)); */
  reaction[3] = -std::sin(alpha_w);

  /* 'tiltBalance:31' reaction(reaction < 0) = 0; */
  for (i = 0; i < 4; i++) {
    alpha_w = reaction[i];
    if (reaction[i] < 0.0) {
      alpha_w = 0.0;
    }

    reaction[i] = alpha_w;
  }

  /* 'tiltBalance:33' F0 = zeros(1,2); */
  /* 'tiltBalance:34' F1 = zeros(1,2); */
  /* 'tiltBalance:35' F2 = zeros(1,2); */
  /* 'tiltBalance:36' F3 = zeros(1,2); */
  /* 'tiltBalance:38' [F0(1), F0(2)] = pol2cart(0, reaction(1)); */
  /* 'tiltBalance:39' [F1(1), F1(2)] = pol2cart(pi/2, reaction(2)); */
  /* 'tiltBalance:40' [F2(1), F2(2)] = pol2cart(pi, reaction(3)); */
  /* 'tiltBalance:41' [F3(1), F3(2)] = pol2cart(3*pi/2, reaction(4)); */
  /*  [F0; F1; F2; F3] */
  /* 'tiltBalance:44' R0 = Awu * [F0 0]'; */
  /* 'tiltBalance:45' R1 = Awu * [F1 0]'; */
  /* 'tiltBalance:46' R2 = Awu * [F2 0]'; */
  /* 'tiltBalance:47' R3 = Awu * [F3 0]'; */
  /*  [R0 R1 R2 R3] */
  /* 'tiltBalance:50' w0 = reaction(1); */
  *w0 = reaction[0];

  /* 'tiltBalance:51' w1 = reaction(2); */
  *w1 = reaction[1];

  /* 'tiltBalance:52' w2 = reaction(3); */
  *w2 = reaction[2];

  /* 'tiltBalance:53' w3 = reaction(4); */
  *w3 = reaction[3];

  /* toc */
  /*  ---------- visualizing only below this line */
  /*  clf */
  /*  hold on; */
  /*   */
  /*  K0 = [ Dx    Dy   0]; */
  /*  K1 = [-Dx    Dy   0]; */
  /*  K2 = [-Dx   -Dy   0]; */
  /*  K3 = [ Dx   -Dy   0]; */
  /*   */
  /*  scatter3(K0(1), K0(2), K0(3)) */
  /*  scatter3(K1(1), K1(2), K1(3)) */
  /*  scatter3(K2(1), K2(2), K2(3)) */
  /*  scatter3(K3(1), K3(2), K3(3)) */
  /*   */
  /*   */
  /*  plot3([K0(1) K0(1)], [K0(2), K0(2)], [K0(3) K0(3)+reaction(1)]) */
  /*  plot3([K1(1) K1(1)], [K1(2), K1(2)], [K1(3) K1(3)+reaction(2)]) */
  /*  plot3([K2(1) K2(1)], [K2(2), K2(2)], [K2(3) K2(3)+reaction(3)]) */
  /*  plot3([K3(1) K3(1)], [K3(2), K3(2)], [K3(3) K3(3)+reaction(4)]) */
  /*   */
  /*  ZZZ = 10; */
  /*  axis([-ZZZ ZZZ -ZZZ ZZZ -ZZZ ZZZ]) */
  /*  axis vis3d */
  /*  xlabel('X') */
  /*  ylabel('Y') */
  /*  rotate3d on */
  /*  view([135 45]) */
  /*  if (x > 0); px = x; else; px = 0; end */
  /*  if (x < 0); nx = x; else; nx = 0; end */
  /*  if (y > 0); py = y; else; py = 0; end */
  /*  if (y < 0); ny = y; else; ny = 0; end */
  /*  c0 = dot(w0, K0); */
  /*  c1 = dot(w1, K1); */
  /*  c2 = dot(w2, K2); */
  /*  c3 = dot(w3, K3); */
  /*  scatter3(w0(1), w0(2), w0(3)) */
  /*  scatter3(w1(1), w1(2), w1(3)) */
  /*  scatter3(w2(1), w2(2), w2(3)) */
  /*  scatter3(w3(1), w3(2), w3(3)) */
}

void tiltBalance_initialize()
{
  rt_InitInfAndNaN(8U);
}

void tiltBalance_terminate()
{
  /* (no terminate code required) */
}

/* End of code generation (tiltBalance.cpp) */
