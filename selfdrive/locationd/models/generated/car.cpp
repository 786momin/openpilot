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
void err_fun(double *nom_x, double *delta_x, double *out_6145962361718420193) {
   out_6145962361718420193[0] = delta_x[0] + nom_x[0];
   out_6145962361718420193[1] = delta_x[1] + nom_x[1];
   out_6145962361718420193[2] = delta_x[2] + nom_x[2];
   out_6145962361718420193[3] = delta_x[3] + nom_x[3];
   out_6145962361718420193[4] = delta_x[4] + nom_x[4];
   out_6145962361718420193[5] = delta_x[5] + nom_x[5];
   out_6145962361718420193[6] = delta_x[6] + nom_x[6];
   out_6145962361718420193[7] = delta_x[7] + nom_x[7];
   out_6145962361718420193[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8230558101007659953) {
   out_8230558101007659953[0] = -nom_x[0] + true_x[0];
   out_8230558101007659953[1] = -nom_x[1] + true_x[1];
   out_8230558101007659953[2] = -nom_x[2] + true_x[2];
   out_8230558101007659953[3] = -nom_x[3] + true_x[3];
   out_8230558101007659953[4] = -nom_x[4] + true_x[4];
   out_8230558101007659953[5] = -nom_x[5] + true_x[5];
   out_8230558101007659953[6] = -nom_x[6] + true_x[6];
   out_8230558101007659953[7] = -nom_x[7] + true_x[7];
   out_8230558101007659953[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7703293425562739781) {
   out_7703293425562739781[0] = 1.0;
   out_7703293425562739781[1] = 0.0;
   out_7703293425562739781[2] = 0.0;
   out_7703293425562739781[3] = 0.0;
   out_7703293425562739781[4] = 0.0;
   out_7703293425562739781[5] = 0.0;
   out_7703293425562739781[6] = 0.0;
   out_7703293425562739781[7] = 0.0;
   out_7703293425562739781[8] = 0.0;
   out_7703293425562739781[9] = 0.0;
   out_7703293425562739781[10] = 1.0;
   out_7703293425562739781[11] = 0.0;
   out_7703293425562739781[12] = 0.0;
   out_7703293425562739781[13] = 0.0;
   out_7703293425562739781[14] = 0.0;
   out_7703293425562739781[15] = 0.0;
   out_7703293425562739781[16] = 0.0;
   out_7703293425562739781[17] = 0.0;
   out_7703293425562739781[18] = 0.0;
   out_7703293425562739781[19] = 0.0;
   out_7703293425562739781[20] = 1.0;
   out_7703293425562739781[21] = 0.0;
   out_7703293425562739781[22] = 0.0;
   out_7703293425562739781[23] = 0.0;
   out_7703293425562739781[24] = 0.0;
   out_7703293425562739781[25] = 0.0;
   out_7703293425562739781[26] = 0.0;
   out_7703293425562739781[27] = 0.0;
   out_7703293425562739781[28] = 0.0;
   out_7703293425562739781[29] = 0.0;
   out_7703293425562739781[30] = 1.0;
   out_7703293425562739781[31] = 0.0;
   out_7703293425562739781[32] = 0.0;
   out_7703293425562739781[33] = 0.0;
   out_7703293425562739781[34] = 0.0;
   out_7703293425562739781[35] = 0.0;
   out_7703293425562739781[36] = 0.0;
   out_7703293425562739781[37] = 0.0;
   out_7703293425562739781[38] = 0.0;
   out_7703293425562739781[39] = 0.0;
   out_7703293425562739781[40] = 1.0;
   out_7703293425562739781[41] = 0.0;
   out_7703293425562739781[42] = 0.0;
   out_7703293425562739781[43] = 0.0;
   out_7703293425562739781[44] = 0.0;
   out_7703293425562739781[45] = 0.0;
   out_7703293425562739781[46] = 0.0;
   out_7703293425562739781[47] = 0.0;
   out_7703293425562739781[48] = 0.0;
   out_7703293425562739781[49] = 0.0;
   out_7703293425562739781[50] = 1.0;
   out_7703293425562739781[51] = 0.0;
   out_7703293425562739781[52] = 0.0;
   out_7703293425562739781[53] = 0.0;
   out_7703293425562739781[54] = 0.0;
   out_7703293425562739781[55] = 0.0;
   out_7703293425562739781[56] = 0.0;
   out_7703293425562739781[57] = 0.0;
   out_7703293425562739781[58] = 0.0;
   out_7703293425562739781[59] = 0.0;
   out_7703293425562739781[60] = 1.0;
   out_7703293425562739781[61] = 0.0;
   out_7703293425562739781[62] = 0.0;
   out_7703293425562739781[63] = 0.0;
   out_7703293425562739781[64] = 0.0;
   out_7703293425562739781[65] = 0.0;
   out_7703293425562739781[66] = 0.0;
   out_7703293425562739781[67] = 0.0;
   out_7703293425562739781[68] = 0.0;
   out_7703293425562739781[69] = 0.0;
   out_7703293425562739781[70] = 1.0;
   out_7703293425562739781[71] = 0.0;
   out_7703293425562739781[72] = 0.0;
   out_7703293425562739781[73] = 0.0;
   out_7703293425562739781[74] = 0.0;
   out_7703293425562739781[75] = 0.0;
   out_7703293425562739781[76] = 0.0;
   out_7703293425562739781[77] = 0.0;
   out_7703293425562739781[78] = 0.0;
   out_7703293425562739781[79] = 0.0;
   out_7703293425562739781[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5449189766295560519) {
   out_5449189766295560519[0] = state[0];
   out_5449189766295560519[1] = state[1];
   out_5449189766295560519[2] = state[2];
   out_5449189766295560519[3] = state[3];
   out_5449189766295560519[4] = state[4];
   out_5449189766295560519[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5449189766295560519[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5449189766295560519[7] = state[7];
   out_5449189766295560519[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1552542908753030021) {
   out_1552542908753030021[0] = 1;
   out_1552542908753030021[1] = 0;
   out_1552542908753030021[2] = 0;
   out_1552542908753030021[3] = 0;
   out_1552542908753030021[4] = 0;
   out_1552542908753030021[5] = 0;
   out_1552542908753030021[6] = 0;
   out_1552542908753030021[7] = 0;
   out_1552542908753030021[8] = 0;
   out_1552542908753030021[9] = 0;
   out_1552542908753030021[10] = 1;
   out_1552542908753030021[11] = 0;
   out_1552542908753030021[12] = 0;
   out_1552542908753030021[13] = 0;
   out_1552542908753030021[14] = 0;
   out_1552542908753030021[15] = 0;
   out_1552542908753030021[16] = 0;
   out_1552542908753030021[17] = 0;
   out_1552542908753030021[18] = 0;
   out_1552542908753030021[19] = 0;
   out_1552542908753030021[20] = 1;
   out_1552542908753030021[21] = 0;
   out_1552542908753030021[22] = 0;
   out_1552542908753030021[23] = 0;
   out_1552542908753030021[24] = 0;
   out_1552542908753030021[25] = 0;
   out_1552542908753030021[26] = 0;
   out_1552542908753030021[27] = 0;
   out_1552542908753030021[28] = 0;
   out_1552542908753030021[29] = 0;
   out_1552542908753030021[30] = 1;
   out_1552542908753030021[31] = 0;
   out_1552542908753030021[32] = 0;
   out_1552542908753030021[33] = 0;
   out_1552542908753030021[34] = 0;
   out_1552542908753030021[35] = 0;
   out_1552542908753030021[36] = 0;
   out_1552542908753030021[37] = 0;
   out_1552542908753030021[38] = 0;
   out_1552542908753030021[39] = 0;
   out_1552542908753030021[40] = 1;
   out_1552542908753030021[41] = 0;
   out_1552542908753030021[42] = 0;
   out_1552542908753030021[43] = 0;
   out_1552542908753030021[44] = 0;
   out_1552542908753030021[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1552542908753030021[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1552542908753030021[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1552542908753030021[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1552542908753030021[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1552542908753030021[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1552542908753030021[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1552542908753030021[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1552542908753030021[53] = -9.8100000000000005*dt;
   out_1552542908753030021[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1552542908753030021[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1552542908753030021[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1552542908753030021[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1552542908753030021[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1552542908753030021[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1552542908753030021[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1552542908753030021[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1552542908753030021[62] = 0;
   out_1552542908753030021[63] = 0;
   out_1552542908753030021[64] = 0;
   out_1552542908753030021[65] = 0;
   out_1552542908753030021[66] = 0;
   out_1552542908753030021[67] = 0;
   out_1552542908753030021[68] = 0;
   out_1552542908753030021[69] = 0;
   out_1552542908753030021[70] = 1;
   out_1552542908753030021[71] = 0;
   out_1552542908753030021[72] = 0;
   out_1552542908753030021[73] = 0;
   out_1552542908753030021[74] = 0;
   out_1552542908753030021[75] = 0;
   out_1552542908753030021[76] = 0;
   out_1552542908753030021[77] = 0;
   out_1552542908753030021[78] = 0;
   out_1552542908753030021[79] = 0;
   out_1552542908753030021[80] = 1;
}
void h_25(double *state, double *unused, double *out_7146718993336217190) {
   out_7146718993336217190[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2799601600378074941) {
   out_2799601600378074941[0] = 0;
   out_2799601600378074941[1] = 0;
   out_2799601600378074941[2] = 0;
   out_2799601600378074941[3] = 0;
   out_2799601600378074941[4] = 0;
   out_2799601600378074941[5] = 0;
   out_2799601600378074941[6] = 1;
   out_2799601600378074941[7] = 0;
   out_2799601600378074941[8] = 0;
}
void h_24(double *state, double *unused, double *out_4016802530754852498) {
   out_4016802530754852498[0] = state[4];
   out_4016802530754852498[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1954604991594379542) {
   out_1954604991594379542[0] = 0;
   out_1954604991594379542[1] = 0;
   out_1954604991594379542[2] = 0;
   out_1954604991594379542[3] = 0;
   out_1954604991594379542[4] = 1;
   out_1954604991594379542[5] = 0;
   out_1954604991594379542[6] = 0;
   out_1954604991594379542[7] = 0;
   out_1954604991594379542[8] = 0;
   out_1954604991594379542[9] = 0;
   out_1954604991594379542[10] = 0;
   out_1954604991594379542[11] = 0;
   out_1954604991594379542[12] = 0;
   out_1954604991594379542[13] = 0;
   out_1954604991594379542[14] = 1;
   out_1954604991594379542[15] = 0;
   out_1954604991594379542[16] = 0;
   out_1954604991594379542[17] = 0;
}
void h_30(double *state, double *unused, double *out_4198993002980732229) {
   out_4198993002980732229[0] = state[4];
}
void H_30(double *state, double *unused, double *out_281268641870826314) {
   out_281268641870826314[0] = 0;
   out_281268641870826314[1] = 0;
   out_281268641870826314[2] = 0;
   out_281268641870826314[3] = 0;
   out_281268641870826314[4] = 1;
   out_281268641870826314[5] = 0;
   out_281268641870826314[6] = 0;
   out_281268641870826314[7] = 0;
   out_281268641870826314[8] = 0;
}
void h_26(double *state, double *unused, double *out_1162206951536399544) {
   out_1162206951536399544[0] = state[7];
}
void H_26(double *state, double *unused, double *out_504924369382725660) {
   out_504924369382725660[0] = 0;
   out_504924369382725660[1] = 0;
   out_504924369382725660[2] = 0;
   out_504924369382725660[3] = 0;
   out_504924369382725660[4] = 0;
   out_504924369382725660[5] = 0;
   out_504924369382725660[6] = 0;
   out_504924369382725660[7] = 1;
   out_504924369382725660[8] = 0;
}
void h_27(double *state, double *unused, double *out_8695619759564636775) {
   out_8695619759564636775[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1942325429313116903) {
   out_1942325429313116903[0] = 0;
   out_1942325429313116903[1] = 0;
   out_1942325429313116903[2] = 0;
   out_1942325429313116903[3] = 1;
   out_1942325429313116903[4] = 0;
   out_1942325429313116903[5] = 0;
   out_1942325429313116903[6] = 0;
   out_1942325429313116903[7] = 0;
   out_1942325429313116903[8] = 0;
}
void h_29(double *state, double *unused, double *out_802680264680113416) {
   out_802680264680113416[0] = state[1];
}
void H_29(double *state, double *unused, double *out_228962702443565870) {
   out_228962702443565870[0] = 0;
   out_228962702443565870[1] = 1;
   out_228962702443565870[2] = 0;
   out_228962702443565870[3] = 0;
   out_228962702443565870[4] = 0;
   out_228962702443565870[5] = 0;
   out_228962702443565870[6] = 0;
   out_228962702443565870[7] = 0;
   out_228962702443565870[8] = 0;
}
void h_28(double *state, double *unused, double *out_286349412442203990) {
   out_286349412442203990[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4853436314625964704) {
   out_4853436314625964704[0] = 1;
   out_4853436314625964704[1] = 0;
   out_4853436314625964704[2] = 0;
   out_4853436314625964704[3] = 0;
   out_4853436314625964704[4] = 0;
   out_4853436314625964704[5] = 0;
   out_4853436314625964704[6] = 0;
   out_4853436314625964704[7] = 0;
   out_4853436314625964704[8] = 0;
}
void h_31(double *state, double *unused, double *out_3418752429619285241) {
   out_3418752429619285241[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4277073650133742312) {
   out_4277073650133742312[0] = 0;
   out_4277073650133742312[1] = 0;
   out_4277073650133742312[2] = 0;
   out_4277073650133742312[3] = 0;
   out_4277073650133742312[4] = 0;
   out_4277073650133742312[5] = 0;
   out_4277073650133742312[6] = 0;
   out_4277073650133742312[7] = 0;
   out_4277073650133742312[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6145962361718420193) {
  err_fun(nom_x, delta_x, out_6145962361718420193);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8230558101007659953) {
  inv_err_fun(nom_x, true_x, out_8230558101007659953);
}
void car_H_mod_fun(double *state, double *out_7703293425562739781) {
  H_mod_fun(state, out_7703293425562739781);
}
void car_f_fun(double *state, double dt, double *out_5449189766295560519) {
  f_fun(state,  dt, out_5449189766295560519);
}
void car_F_fun(double *state, double dt, double *out_1552542908753030021) {
  F_fun(state,  dt, out_1552542908753030021);
}
void car_h_25(double *state, double *unused, double *out_7146718993336217190) {
  h_25(state, unused, out_7146718993336217190);
}
void car_H_25(double *state, double *unused, double *out_2799601600378074941) {
  H_25(state, unused, out_2799601600378074941);
}
void car_h_24(double *state, double *unused, double *out_4016802530754852498) {
  h_24(state, unused, out_4016802530754852498);
}
void car_H_24(double *state, double *unused, double *out_1954604991594379542) {
  H_24(state, unused, out_1954604991594379542);
}
void car_h_30(double *state, double *unused, double *out_4198993002980732229) {
  h_30(state, unused, out_4198993002980732229);
}
void car_H_30(double *state, double *unused, double *out_281268641870826314) {
  H_30(state, unused, out_281268641870826314);
}
void car_h_26(double *state, double *unused, double *out_1162206951536399544) {
  h_26(state, unused, out_1162206951536399544);
}
void car_H_26(double *state, double *unused, double *out_504924369382725660) {
  H_26(state, unused, out_504924369382725660);
}
void car_h_27(double *state, double *unused, double *out_8695619759564636775) {
  h_27(state, unused, out_8695619759564636775);
}
void car_H_27(double *state, double *unused, double *out_1942325429313116903) {
  H_27(state, unused, out_1942325429313116903);
}
void car_h_29(double *state, double *unused, double *out_802680264680113416) {
  h_29(state, unused, out_802680264680113416);
}
void car_H_29(double *state, double *unused, double *out_228962702443565870) {
  H_29(state, unused, out_228962702443565870);
}
void car_h_28(double *state, double *unused, double *out_286349412442203990) {
  h_28(state, unused, out_286349412442203990);
}
void car_H_28(double *state, double *unused, double *out_4853436314625964704) {
  H_28(state, unused, out_4853436314625964704);
}
void car_h_31(double *state, double *unused, double *out_3418752429619285241) {
  h_31(state, unused, out_3418752429619285241);
}
void car_H_31(double *state, double *unused, double *out_4277073650133742312) {
  H_31(state, unused, out_4277073650133742312);
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
