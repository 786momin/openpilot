#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4586029367640539685) {
   out_4586029367640539685[0] = delta_x[0] + nom_x[0];
   out_4586029367640539685[1] = delta_x[1] + nom_x[1];
   out_4586029367640539685[2] = delta_x[2] + nom_x[2];
   out_4586029367640539685[3] = delta_x[3] + nom_x[3];
   out_4586029367640539685[4] = delta_x[4] + nom_x[4];
   out_4586029367640539685[5] = delta_x[5] + nom_x[5];
   out_4586029367640539685[6] = delta_x[6] + nom_x[6];
   out_4586029367640539685[7] = delta_x[7] + nom_x[7];
   out_4586029367640539685[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5378054743951964721) {
   out_5378054743951964721[0] = -nom_x[0] + true_x[0];
   out_5378054743951964721[1] = -nom_x[1] + true_x[1];
   out_5378054743951964721[2] = -nom_x[2] + true_x[2];
   out_5378054743951964721[3] = -nom_x[3] + true_x[3];
   out_5378054743951964721[4] = -nom_x[4] + true_x[4];
   out_5378054743951964721[5] = -nom_x[5] + true_x[5];
   out_5378054743951964721[6] = -nom_x[6] + true_x[6];
   out_5378054743951964721[7] = -nom_x[7] + true_x[7];
   out_5378054743951964721[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8109768655304479947) {
   out_8109768655304479947[0] = 1.0;
   out_8109768655304479947[1] = 0.0;
   out_8109768655304479947[2] = 0.0;
   out_8109768655304479947[3] = 0.0;
   out_8109768655304479947[4] = 0.0;
   out_8109768655304479947[5] = 0.0;
   out_8109768655304479947[6] = 0.0;
   out_8109768655304479947[7] = 0.0;
   out_8109768655304479947[8] = 0.0;
   out_8109768655304479947[9] = 0.0;
   out_8109768655304479947[10] = 1.0;
   out_8109768655304479947[11] = 0.0;
   out_8109768655304479947[12] = 0.0;
   out_8109768655304479947[13] = 0.0;
   out_8109768655304479947[14] = 0.0;
   out_8109768655304479947[15] = 0.0;
   out_8109768655304479947[16] = 0.0;
   out_8109768655304479947[17] = 0.0;
   out_8109768655304479947[18] = 0.0;
   out_8109768655304479947[19] = 0.0;
   out_8109768655304479947[20] = 1.0;
   out_8109768655304479947[21] = 0.0;
   out_8109768655304479947[22] = 0.0;
   out_8109768655304479947[23] = 0.0;
   out_8109768655304479947[24] = 0.0;
   out_8109768655304479947[25] = 0.0;
   out_8109768655304479947[26] = 0.0;
   out_8109768655304479947[27] = 0.0;
   out_8109768655304479947[28] = 0.0;
   out_8109768655304479947[29] = 0.0;
   out_8109768655304479947[30] = 1.0;
   out_8109768655304479947[31] = 0.0;
   out_8109768655304479947[32] = 0.0;
   out_8109768655304479947[33] = 0.0;
   out_8109768655304479947[34] = 0.0;
   out_8109768655304479947[35] = 0.0;
   out_8109768655304479947[36] = 0.0;
   out_8109768655304479947[37] = 0.0;
   out_8109768655304479947[38] = 0.0;
   out_8109768655304479947[39] = 0.0;
   out_8109768655304479947[40] = 1.0;
   out_8109768655304479947[41] = 0.0;
   out_8109768655304479947[42] = 0.0;
   out_8109768655304479947[43] = 0.0;
   out_8109768655304479947[44] = 0.0;
   out_8109768655304479947[45] = 0.0;
   out_8109768655304479947[46] = 0.0;
   out_8109768655304479947[47] = 0.0;
   out_8109768655304479947[48] = 0.0;
   out_8109768655304479947[49] = 0.0;
   out_8109768655304479947[50] = 1.0;
   out_8109768655304479947[51] = 0.0;
   out_8109768655304479947[52] = 0.0;
   out_8109768655304479947[53] = 0.0;
   out_8109768655304479947[54] = 0.0;
   out_8109768655304479947[55] = 0.0;
   out_8109768655304479947[56] = 0.0;
   out_8109768655304479947[57] = 0.0;
   out_8109768655304479947[58] = 0.0;
   out_8109768655304479947[59] = 0.0;
   out_8109768655304479947[60] = 1.0;
   out_8109768655304479947[61] = 0.0;
   out_8109768655304479947[62] = 0.0;
   out_8109768655304479947[63] = 0.0;
   out_8109768655304479947[64] = 0.0;
   out_8109768655304479947[65] = 0.0;
   out_8109768655304479947[66] = 0.0;
   out_8109768655304479947[67] = 0.0;
   out_8109768655304479947[68] = 0.0;
   out_8109768655304479947[69] = 0.0;
   out_8109768655304479947[70] = 1.0;
   out_8109768655304479947[71] = 0.0;
   out_8109768655304479947[72] = 0.0;
   out_8109768655304479947[73] = 0.0;
   out_8109768655304479947[74] = 0.0;
   out_8109768655304479947[75] = 0.0;
   out_8109768655304479947[76] = 0.0;
   out_8109768655304479947[77] = 0.0;
   out_8109768655304479947[78] = 0.0;
   out_8109768655304479947[79] = 0.0;
   out_8109768655304479947[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8436141603802697156) {
   out_8436141603802697156[0] = state[0];
   out_8436141603802697156[1] = state[1];
   out_8436141603802697156[2] = state[2];
   out_8436141603802697156[3] = state[3];
   out_8436141603802697156[4] = state[4];
   out_8436141603802697156[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8436141603802697156[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8436141603802697156[7] = state[7];
   out_8436141603802697156[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2266778645478433972) {
   out_2266778645478433972[0] = 1;
   out_2266778645478433972[1] = 0;
   out_2266778645478433972[2] = 0;
   out_2266778645478433972[3] = 0;
   out_2266778645478433972[4] = 0;
   out_2266778645478433972[5] = 0;
   out_2266778645478433972[6] = 0;
   out_2266778645478433972[7] = 0;
   out_2266778645478433972[8] = 0;
   out_2266778645478433972[9] = 0;
   out_2266778645478433972[10] = 1;
   out_2266778645478433972[11] = 0;
   out_2266778645478433972[12] = 0;
   out_2266778645478433972[13] = 0;
   out_2266778645478433972[14] = 0;
   out_2266778645478433972[15] = 0;
   out_2266778645478433972[16] = 0;
   out_2266778645478433972[17] = 0;
   out_2266778645478433972[18] = 0;
   out_2266778645478433972[19] = 0;
   out_2266778645478433972[20] = 1;
   out_2266778645478433972[21] = 0;
   out_2266778645478433972[22] = 0;
   out_2266778645478433972[23] = 0;
   out_2266778645478433972[24] = 0;
   out_2266778645478433972[25] = 0;
   out_2266778645478433972[26] = 0;
   out_2266778645478433972[27] = 0;
   out_2266778645478433972[28] = 0;
   out_2266778645478433972[29] = 0;
   out_2266778645478433972[30] = 1;
   out_2266778645478433972[31] = 0;
   out_2266778645478433972[32] = 0;
   out_2266778645478433972[33] = 0;
   out_2266778645478433972[34] = 0;
   out_2266778645478433972[35] = 0;
   out_2266778645478433972[36] = 0;
   out_2266778645478433972[37] = 0;
   out_2266778645478433972[38] = 0;
   out_2266778645478433972[39] = 0;
   out_2266778645478433972[40] = 1;
   out_2266778645478433972[41] = 0;
   out_2266778645478433972[42] = 0;
   out_2266778645478433972[43] = 0;
   out_2266778645478433972[44] = 0;
   out_2266778645478433972[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2266778645478433972[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2266778645478433972[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2266778645478433972[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2266778645478433972[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2266778645478433972[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2266778645478433972[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2266778645478433972[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2266778645478433972[53] = -9.8000000000000007*dt;
   out_2266778645478433972[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2266778645478433972[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2266778645478433972[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2266778645478433972[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2266778645478433972[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2266778645478433972[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2266778645478433972[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2266778645478433972[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2266778645478433972[62] = 0;
   out_2266778645478433972[63] = 0;
   out_2266778645478433972[64] = 0;
   out_2266778645478433972[65] = 0;
   out_2266778645478433972[66] = 0;
   out_2266778645478433972[67] = 0;
   out_2266778645478433972[68] = 0;
   out_2266778645478433972[69] = 0;
   out_2266778645478433972[70] = 1;
   out_2266778645478433972[71] = 0;
   out_2266778645478433972[72] = 0;
   out_2266778645478433972[73] = 0;
   out_2266778645478433972[74] = 0;
   out_2266778645478433972[75] = 0;
   out_2266778645478433972[76] = 0;
   out_2266778645478433972[77] = 0;
   out_2266778645478433972[78] = 0;
   out_2266778645478433972[79] = 0;
   out_2266778645478433972[80] = 1;
}
void h_25(double *state, double *unused, double *out_6676757261096667859) {
   out_6676757261096667859[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1143787192492834432) {
   out_1143787192492834432[0] = 0;
   out_1143787192492834432[1] = 0;
   out_1143787192492834432[2] = 0;
   out_1143787192492834432[3] = 0;
   out_1143787192492834432[4] = 0;
   out_1143787192492834432[5] = 0;
   out_1143787192492834432[6] = 1;
   out_1143787192492834432[7] = 0;
   out_1143787192492834432[8] = 0;
}
void h_24(double *state, double *unused, double *out_547609794561793546) {
   out_547609794561793546[0] = state[4];
   out_547609794561793546[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7797992650601020189) {
   out_7797992650601020189[0] = 0;
   out_7797992650601020189[1] = 0;
   out_7797992650601020189[2] = 0;
   out_7797992650601020189[3] = 0;
   out_7797992650601020189[4] = 1;
   out_7797992650601020189[5] = 0;
   out_7797992650601020189[6] = 0;
   out_7797992650601020189[7] = 0;
   out_7797992650601020189[8] = 0;
   out_7797992650601020189[9] = 0;
   out_7797992650601020189[10] = 0;
   out_7797992650601020189[11] = 0;
   out_7797992650601020189[12] = 0;
   out_7797992650601020189[13] = 0;
   out_7797992650601020189[14] = 1;
   out_7797992650601020189[15] = 0;
   out_7797992650601020189[16] = 0;
   out_7797992650601020189[17] = 0;
}
void h_30(double *state, double *unused, double *out_7908303336184382105) {
   out_7908303336184382105[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1014448245349594362) {
   out_1014448245349594362[0] = 0;
   out_1014448245349594362[1] = 0;
   out_1014448245349594362[2] = 0;
   out_1014448245349594362[3] = 0;
   out_1014448245349594362[4] = 1;
   out_1014448245349594362[5] = 0;
   out_1014448245349594362[6] = 0;
   out_1014448245349594362[7] = 0;
   out_1014448245349594362[8] = 0;
}
void h_26(double *state, double *unused, double *out_5312533521689844564) {
   out_5312533521689844564[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1800641256603146336) {
   out_1800641256603146336[0] = 0;
   out_1800641256603146336[1] = 0;
   out_1800641256603146336[2] = 0;
   out_1800641256603146336[3] = 0;
   out_1800641256603146336[4] = 0;
   out_1800641256603146336[5] = 0;
   out_1800641256603146336[6] = 0;
   out_1800641256603146336[7] = 1;
   out_1800641256603146336[8] = 0;
}
void h_27(double *state, double *unused, double *out_3052149892744191431) {
   out_3052149892744191431[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1160315066450830549) {
   out_1160315066450830549[0] = 0;
   out_1160315066450830549[1] = 0;
   out_1160315066450830549[2] = 0;
   out_1160315066450830549[3] = 1;
   out_1160315066450830549[4] = 0;
   out_1160315066450830549[5] = 0;
   out_1160315066450830549[6] = 0;
   out_1160315066450830549[7] = 0;
   out_1160315066450830549[8] = 0;
}
void h_29(double *state, double *unused, double *out_2906630068523536460) {
   out_2906630068523536460[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1524679589663986546) {
   out_1524679589663986546[0] = 0;
   out_1524679589663986546[1] = 1;
   out_1524679589663986546[2] = 0;
   out_1524679589663986546[3] = 0;
   out_1524679589663986546[4] = 0;
   out_1524679589663986546[5] = 0;
   out_1524679589663986546[6] = 0;
   out_1524679589663986546[7] = 0;
   out_1524679589663986546[8] = 0;
}
void h_28(double *state, double *unused, double *out_2696461356750672739) {
   out_2696461356750672739[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3488309861229312797) {
   out_3488309861229312797[0] = 1;
   out_3488309861229312797[1] = 0;
   out_3488309861229312797[2] = 0;
   out_3488309861229312797[3] = 0;
   out_3488309861229312797[4] = 0;
   out_3488309861229312797[5] = 0;
   out_3488309861229312797[6] = 0;
   out_3488309861229312797[7] = 0;
   out_3488309861229312797[8] = 0;
}
void h_31(double *state, double *unused, double *out_4107799479400003493) {
   out_4107799479400003493[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1174433154369794860) {
   out_1174433154369794860[0] = 0;
   out_1174433154369794860[1] = 0;
   out_1174433154369794860[2] = 0;
   out_1174433154369794860[3] = 0;
   out_1174433154369794860[4] = 0;
   out_1174433154369794860[5] = 0;
   out_1174433154369794860[6] = 0;
   out_1174433154369794860[7] = 0;
   out_1174433154369794860[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4586029367640539685) {
  err_fun(nom_x, delta_x, out_4586029367640539685);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5378054743951964721) {
  inv_err_fun(nom_x, true_x, out_5378054743951964721);
}
void car_H_mod_fun(double *state, double *out_8109768655304479947) {
  H_mod_fun(state, out_8109768655304479947);
}
void car_f_fun(double *state, double dt, double *out_8436141603802697156) {
  f_fun(state,  dt, out_8436141603802697156);
}
void car_F_fun(double *state, double dt, double *out_2266778645478433972) {
  F_fun(state,  dt, out_2266778645478433972);
}
void car_h_25(double *state, double *unused, double *out_6676757261096667859) {
  h_25(state, unused, out_6676757261096667859);
}
void car_H_25(double *state, double *unused, double *out_1143787192492834432) {
  H_25(state, unused, out_1143787192492834432);
}
void car_h_24(double *state, double *unused, double *out_547609794561793546) {
  h_24(state, unused, out_547609794561793546);
}
void car_H_24(double *state, double *unused, double *out_7797992650601020189) {
  H_24(state, unused, out_7797992650601020189);
}
void car_h_30(double *state, double *unused, double *out_7908303336184382105) {
  h_30(state, unused, out_7908303336184382105);
}
void car_H_30(double *state, double *unused, double *out_1014448245349594362) {
  H_30(state, unused, out_1014448245349594362);
}
void car_h_26(double *state, double *unused, double *out_5312533521689844564) {
  h_26(state, unused, out_5312533521689844564);
}
void car_H_26(double *state, double *unused, double *out_1800641256603146336) {
  H_26(state, unused, out_1800641256603146336);
}
void car_h_27(double *state, double *unused, double *out_3052149892744191431) {
  h_27(state, unused, out_3052149892744191431);
}
void car_H_27(double *state, double *unused, double *out_1160315066450830549) {
  H_27(state, unused, out_1160315066450830549);
}
void car_h_29(double *state, double *unused, double *out_2906630068523536460) {
  h_29(state, unused, out_2906630068523536460);
}
void car_H_29(double *state, double *unused, double *out_1524679589663986546) {
  H_29(state, unused, out_1524679589663986546);
}
void car_h_28(double *state, double *unused, double *out_2696461356750672739) {
  h_28(state, unused, out_2696461356750672739);
}
void car_H_28(double *state, double *unused, double *out_3488309861229312797) {
  H_28(state, unused, out_3488309861229312797);
}
void car_h_31(double *state, double *unused, double *out_4107799479400003493) {
  h_31(state, unused, out_4107799479400003493);
}
void car_H_31(double *state, double *unused, double *out_1174433154369794860) {
  H_31(state, unused, out_1174433154369794860);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
