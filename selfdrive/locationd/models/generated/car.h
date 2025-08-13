#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6145962361718420193);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8230558101007659953);
void car_H_mod_fun(double *state, double *out_7703293425562739781);
void car_f_fun(double *state, double dt, double *out_5449189766295560519);
void car_F_fun(double *state, double dt, double *out_1552542908753030021);
void car_h_25(double *state, double *unused, double *out_7146718993336217190);
void car_H_25(double *state, double *unused, double *out_2799601600378074941);
void car_h_24(double *state, double *unused, double *out_4016802530754852498);
void car_H_24(double *state, double *unused, double *out_1954604991594379542);
void car_h_30(double *state, double *unused, double *out_4198993002980732229);
void car_H_30(double *state, double *unused, double *out_281268641870826314);
void car_h_26(double *state, double *unused, double *out_1162206951536399544);
void car_H_26(double *state, double *unused, double *out_504924369382725660);
void car_h_27(double *state, double *unused, double *out_8695619759564636775);
void car_H_27(double *state, double *unused, double *out_1942325429313116903);
void car_h_29(double *state, double *unused, double *out_802680264680113416);
void car_H_29(double *state, double *unused, double *out_228962702443565870);
void car_h_28(double *state, double *unused, double *out_286349412442203990);
void car_H_28(double *state, double *unused, double *out_4853436314625964704);
void car_h_31(double *state, double *unused, double *out_3418752429619285241);
void car_H_31(double *state, double *unused, double *out_4277073650133742312);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}