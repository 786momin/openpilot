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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8098387024111253733) {
   out_8098387024111253733[0] = delta_x[0] + nom_x[0];
   out_8098387024111253733[1] = delta_x[1] + nom_x[1];
   out_8098387024111253733[2] = delta_x[2] + nom_x[2];
   out_8098387024111253733[3] = delta_x[3] + nom_x[3];
   out_8098387024111253733[4] = delta_x[4] + nom_x[4];
   out_8098387024111253733[5] = delta_x[5] + nom_x[5];
   out_8098387024111253733[6] = delta_x[6] + nom_x[6];
   out_8098387024111253733[7] = delta_x[7] + nom_x[7];
   out_8098387024111253733[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8197590140558714354) {
   out_8197590140558714354[0] = -nom_x[0] + true_x[0];
   out_8197590140558714354[1] = -nom_x[1] + true_x[1];
   out_8197590140558714354[2] = -nom_x[2] + true_x[2];
   out_8197590140558714354[3] = -nom_x[3] + true_x[3];
   out_8197590140558714354[4] = -nom_x[4] + true_x[4];
   out_8197590140558714354[5] = -nom_x[5] + true_x[5];
   out_8197590140558714354[6] = -nom_x[6] + true_x[6];
   out_8197590140558714354[7] = -nom_x[7] + true_x[7];
   out_8197590140558714354[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7985016218246684398) {
   out_7985016218246684398[0] = 1.0;
   out_7985016218246684398[1] = 0.0;
   out_7985016218246684398[2] = 0.0;
   out_7985016218246684398[3] = 0.0;
   out_7985016218246684398[4] = 0.0;
   out_7985016218246684398[5] = 0.0;
   out_7985016218246684398[6] = 0.0;
   out_7985016218246684398[7] = 0.0;
   out_7985016218246684398[8] = 0.0;
   out_7985016218246684398[9] = 0.0;
   out_7985016218246684398[10] = 1.0;
   out_7985016218246684398[11] = 0.0;
   out_7985016218246684398[12] = 0.0;
   out_7985016218246684398[13] = 0.0;
   out_7985016218246684398[14] = 0.0;
   out_7985016218246684398[15] = 0.0;
   out_7985016218246684398[16] = 0.0;
   out_7985016218246684398[17] = 0.0;
   out_7985016218246684398[18] = 0.0;
   out_7985016218246684398[19] = 0.0;
   out_7985016218246684398[20] = 1.0;
   out_7985016218246684398[21] = 0.0;
   out_7985016218246684398[22] = 0.0;
   out_7985016218246684398[23] = 0.0;
   out_7985016218246684398[24] = 0.0;
   out_7985016218246684398[25] = 0.0;
   out_7985016218246684398[26] = 0.0;
   out_7985016218246684398[27] = 0.0;
   out_7985016218246684398[28] = 0.0;
   out_7985016218246684398[29] = 0.0;
   out_7985016218246684398[30] = 1.0;
   out_7985016218246684398[31] = 0.0;
   out_7985016218246684398[32] = 0.0;
   out_7985016218246684398[33] = 0.0;
   out_7985016218246684398[34] = 0.0;
   out_7985016218246684398[35] = 0.0;
   out_7985016218246684398[36] = 0.0;
   out_7985016218246684398[37] = 0.0;
   out_7985016218246684398[38] = 0.0;
   out_7985016218246684398[39] = 0.0;
   out_7985016218246684398[40] = 1.0;
   out_7985016218246684398[41] = 0.0;
   out_7985016218246684398[42] = 0.0;
   out_7985016218246684398[43] = 0.0;
   out_7985016218246684398[44] = 0.0;
   out_7985016218246684398[45] = 0.0;
   out_7985016218246684398[46] = 0.0;
   out_7985016218246684398[47] = 0.0;
   out_7985016218246684398[48] = 0.0;
   out_7985016218246684398[49] = 0.0;
   out_7985016218246684398[50] = 1.0;
   out_7985016218246684398[51] = 0.0;
   out_7985016218246684398[52] = 0.0;
   out_7985016218246684398[53] = 0.0;
   out_7985016218246684398[54] = 0.0;
   out_7985016218246684398[55] = 0.0;
   out_7985016218246684398[56] = 0.0;
   out_7985016218246684398[57] = 0.0;
   out_7985016218246684398[58] = 0.0;
   out_7985016218246684398[59] = 0.0;
   out_7985016218246684398[60] = 1.0;
   out_7985016218246684398[61] = 0.0;
   out_7985016218246684398[62] = 0.0;
   out_7985016218246684398[63] = 0.0;
   out_7985016218246684398[64] = 0.0;
   out_7985016218246684398[65] = 0.0;
   out_7985016218246684398[66] = 0.0;
   out_7985016218246684398[67] = 0.0;
   out_7985016218246684398[68] = 0.0;
   out_7985016218246684398[69] = 0.0;
   out_7985016218246684398[70] = 1.0;
   out_7985016218246684398[71] = 0.0;
   out_7985016218246684398[72] = 0.0;
   out_7985016218246684398[73] = 0.0;
   out_7985016218246684398[74] = 0.0;
   out_7985016218246684398[75] = 0.0;
   out_7985016218246684398[76] = 0.0;
   out_7985016218246684398[77] = 0.0;
   out_7985016218246684398[78] = 0.0;
   out_7985016218246684398[79] = 0.0;
   out_7985016218246684398[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2546879206310887771) {
   out_2546879206310887771[0] = state[0];
   out_2546879206310887771[1] = state[1];
   out_2546879206310887771[2] = state[2];
   out_2546879206310887771[3] = state[3];
   out_2546879206310887771[4] = state[4];
   out_2546879206310887771[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2546879206310887771[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2546879206310887771[7] = state[7];
   out_2546879206310887771[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6865742471245780773) {
   out_6865742471245780773[0] = 1;
   out_6865742471245780773[1] = 0;
   out_6865742471245780773[2] = 0;
   out_6865742471245780773[3] = 0;
   out_6865742471245780773[4] = 0;
   out_6865742471245780773[5] = 0;
   out_6865742471245780773[6] = 0;
   out_6865742471245780773[7] = 0;
   out_6865742471245780773[8] = 0;
   out_6865742471245780773[9] = 0;
   out_6865742471245780773[10] = 1;
   out_6865742471245780773[11] = 0;
   out_6865742471245780773[12] = 0;
   out_6865742471245780773[13] = 0;
   out_6865742471245780773[14] = 0;
   out_6865742471245780773[15] = 0;
   out_6865742471245780773[16] = 0;
   out_6865742471245780773[17] = 0;
   out_6865742471245780773[18] = 0;
   out_6865742471245780773[19] = 0;
   out_6865742471245780773[20] = 1;
   out_6865742471245780773[21] = 0;
   out_6865742471245780773[22] = 0;
   out_6865742471245780773[23] = 0;
   out_6865742471245780773[24] = 0;
   out_6865742471245780773[25] = 0;
   out_6865742471245780773[26] = 0;
   out_6865742471245780773[27] = 0;
   out_6865742471245780773[28] = 0;
   out_6865742471245780773[29] = 0;
   out_6865742471245780773[30] = 1;
   out_6865742471245780773[31] = 0;
   out_6865742471245780773[32] = 0;
   out_6865742471245780773[33] = 0;
   out_6865742471245780773[34] = 0;
   out_6865742471245780773[35] = 0;
   out_6865742471245780773[36] = 0;
   out_6865742471245780773[37] = 0;
   out_6865742471245780773[38] = 0;
   out_6865742471245780773[39] = 0;
   out_6865742471245780773[40] = 1;
   out_6865742471245780773[41] = 0;
   out_6865742471245780773[42] = 0;
   out_6865742471245780773[43] = 0;
   out_6865742471245780773[44] = 0;
   out_6865742471245780773[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6865742471245780773[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6865742471245780773[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6865742471245780773[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6865742471245780773[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6865742471245780773[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6865742471245780773[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6865742471245780773[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6865742471245780773[53] = -9.8000000000000007*dt;
   out_6865742471245780773[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6865742471245780773[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6865742471245780773[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6865742471245780773[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6865742471245780773[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6865742471245780773[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6865742471245780773[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6865742471245780773[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6865742471245780773[62] = 0;
   out_6865742471245780773[63] = 0;
   out_6865742471245780773[64] = 0;
   out_6865742471245780773[65] = 0;
   out_6865742471245780773[66] = 0;
   out_6865742471245780773[67] = 0;
   out_6865742471245780773[68] = 0;
   out_6865742471245780773[69] = 0;
   out_6865742471245780773[70] = 1;
   out_6865742471245780773[71] = 0;
   out_6865742471245780773[72] = 0;
   out_6865742471245780773[73] = 0;
   out_6865742471245780773[74] = 0;
   out_6865742471245780773[75] = 0;
   out_6865742471245780773[76] = 0;
   out_6865742471245780773[77] = 0;
   out_6865742471245780773[78] = 0;
   out_6865742471245780773[79] = 0;
   out_6865742471245780773[80] = 1;
}
void h_25(double *state, double *unused, double *out_409840628699075303) {
   out_409840628699075303[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6938747205473375541) {
   out_6938747205473375541[0] = 0;
   out_6938747205473375541[1] = 0;
   out_6938747205473375541[2] = 0;
   out_6938747205473375541[3] = 0;
   out_6938747205473375541[4] = 0;
   out_6938747205473375541[5] = 0;
   out_6938747205473375541[6] = 1;
   out_6938747205473375541[7] = 0;
   out_6938747205473375541[8] = 0;
}
void h_24(double *state, double *unused, double *out_8835857786613779772) {
   out_8835857786613779772[0] = state[4];
   out_8835857786613779772[1] = state[5];
}
void H_24(double *state, double *unused, double *out_9111396804478875107) {
   out_9111396804478875107[0] = 0;
   out_9111396804478875107[1] = 0;
   out_9111396804478875107[2] = 0;
   out_9111396804478875107[3] = 0;
   out_9111396804478875107[4] = 1;
   out_9111396804478875107[5] = 0;
   out_9111396804478875107[6] = 0;
   out_9111396804478875107[7] = 0;
   out_9111396804478875107[8] = 0;
   out_9111396804478875107[9] = 0;
   out_9111396804478875107[10] = 0;
   out_9111396804478875107[11] = 0;
   out_9111396804478875107[12] = 0;
   out_9111396804478875107[13] = 0;
   out_9111396804478875107[14] = 1;
   out_9111396804478875107[15] = 0;
   out_9111396804478875107[16] = 0;
   out_9111396804478875107[17] = 0;
}
void h_30(double *state, double *unused, double *out_8173105885519747744) {
   out_8173105885519747744[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4420414246966126914) {
   out_4420414246966126914[0] = 0;
   out_4420414246966126914[1] = 0;
   out_4420414246966126914[2] = 0;
   out_4420414246966126914[3] = 0;
   out_4420414246966126914[4] = 1;
   out_4420414246966126914[5] = 0;
   out_4420414246966126914[6] = 0;
   out_4420414246966126914[7] = 0;
   out_4420414246966126914[8] = 0;
}
void h_26(double *state, double *unused, double *out_5568700564711050093) {
   out_5568700564711050093[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7766493549362119851) {
   out_7766493549362119851[0] = 0;
   out_7766493549362119851[1] = 0;
   out_7766493549362119851[2] = 0;
   out_7766493549362119851[3] = 0;
   out_7766493549362119851[4] = 0;
   out_7766493549362119851[5] = 0;
   out_7766493549362119851[6] = 0;
   out_7766493549362119851[7] = 1;
   out_7766493549362119851[8] = 0;
}
void h_27(double *state, double *unused, double *out_9054934273003635548) {
   out_9054934273003635548[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2196820175782183697) {
   out_2196820175782183697[0] = 0;
   out_2196820175782183697[1] = 0;
   out_2196820175782183697[2] = 0;
   out_2196820175782183697[3] = 1;
   out_2196820175782183697[4] = 0;
   out_2196820175782183697[5] = 0;
   out_2196820175782183697[6] = 0;
   out_2196820175782183697[7] = 0;
   out_2196820175782183697[8] = 0;
}
void h_29(double *state, double *unused, double *out_6822852019009251702) {
   out_6822852019009251702[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3910182902651734730) {
   out_3910182902651734730[0] = 0;
   out_3910182902651734730[1] = 1;
   out_3910182902651734730[2] = 0;
   out_3910182902651734730[3] = 0;
   out_3910182902651734730[4] = 0;
   out_3910182902651734730[5] = 0;
   out_3910182902651734730[6] = 0;
   out_3910182902651734730[7] = 0;
   out_3910182902651734730[8] = 0;
}
void h_28(double *state, double *unused, double *out_8788493916029438551) {
   out_8788493916029438551[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8992581919721265304) {
   out_8992581919721265304[0] = 1;
   out_8992581919721265304[1] = 0;
   out_8992581919721265304[2] = 0;
   out_8992581919721265304[3] = 0;
   out_8992581919721265304[4] = 0;
   out_8992581919721265304[5] = 0;
   out_8992581919721265304[6] = 0;
   out_8992581919721265304[7] = 0;
   out_8992581919721265304[8] = 0;
}
void h_31(double *state, double *unused, double *out_191577528914288251) {
   out_191577528914288251[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6908101243596415113) {
   out_6908101243596415113[0] = 0;
   out_6908101243596415113[1] = 0;
   out_6908101243596415113[2] = 0;
   out_6908101243596415113[3] = 0;
   out_6908101243596415113[4] = 0;
   out_6908101243596415113[5] = 0;
   out_6908101243596415113[6] = 0;
   out_6908101243596415113[7] = 0;
   out_6908101243596415113[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8098387024111253733) {
  err_fun(nom_x, delta_x, out_8098387024111253733);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8197590140558714354) {
  inv_err_fun(nom_x, true_x, out_8197590140558714354);
}
void car_H_mod_fun(double *state, double *out_7985016218246684398) {
  H_mod_fun(state, out_7985016218246684398);
}
void car_f_fun(double *state, double dt, double *out_2546879206310887771) {
  f_fun(state,  dt, out_2546879206310887771);
}
void car_F_fun(double *state, double dt, double *out_6865742471245780773) {
  F_fun(state,  dt, out_6865742471245780773);
}
void car_h_25(double *state, double *unused, double *out_409840628699075303) {
  h_25(state, unused, out_409840628699075303);
}
void car_H_25(double *state, double *unused, double *out_6938747205473375541) {
  H_25(state, unused, out_6938747205473375541);
}
void car_h_24(double *state, double *unused, double *out_8835857786613779772) {
  h_24(state, unused, out_8835857786613779772);
}
void car_H_24(double *state, double *unused, double *out_9111396804478875107) {
  H_24(state, unused, out_9111396804478875107);
}
void car_h_30(double *state, double *unused, double *out_8173105885519747744) {
  h_30(state, unused, out_8173105885519747744);
}
void car_H_30(double *state, double *unused, double *out_4420414246966126914) {
  H_30(state, unused, out_4420414246966126914);
}
void car_h_26(double *state, double *unused, double *out_5568700564711050093) {
  h_26(state, unused, out_5568700564711050093);
}
void car_H_26(double *state, double *unused, double *out_7766493549362119851) {
  H_26(state, unused, out_7766493549362119851);
}
void car_h_27(double *state, double *unused, double *out_9054934273003635548) {
  h_27(state, unused, out_9054934273003635548);
}
void car_H_27(double *state, double *unused, double *out_2196820175782183697) {
  H_27(state, unused, out_2196820175782183697);
}
void car_h_29(double *state, double *unused, double *out_6822852019009251702) {
  h_29(state, unused, out_6822852019009251702);
}
void car_H_29(double *state, double *unused, double *out_3910182902651734730) {
  H_29(state, unused, out_3910182902651734730);
}
void car_h_28(double *state, double *unused, double *out_8788493916029438551) {
  h_28(state, unused, out_8788493916029438551);
}
void car_H_28(double *state, double *unused, double *out_8992581919721265304) {
  H_28(state, unused, out_8992581919721265304);
}
void car_h_31(double *state, double *unused, double *out_191577528914288251) {
  h_31(state, unused, out_191577528914288251);
}
void car_H_31(double *state, double *unused, double *out_6908101243596415113) {
  H_31(state, unused, out_6908101243596415113);
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
