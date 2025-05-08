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
void car_err_fun(double *nom_x, double *delta_x, double *out_2816734334840272051);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3197403674388065693);
void car_H_mod_fun(double *state, double *out_3258567857202627240);
void car_f_fun(double *state, double dt, double *out_4531498887012494036);
void car_F_fun(double *state, double dt, double *out_2630502094007921965);
void car_h_25(double *state, double *unused, double *out_7971611860832540066);
void car_H_25(double *state, double *unused, double *out_5994987990594687139);
void car_h_24(double *state, double *unused, double *out_3750464184291796320);
void car_H_24(double *state, double *unused, double *out_5797550625006678720);
void car_h_30(double *state, double *unused, double *out_6837682684622989200);
void car_H_30(double *state, double *unused, double *out_5865649043451447069);
void car_h_26(double *state, double *unused, double *out_2144286487383851565);
void car_H_26(double *state, double *unused, double *out_2253484671720630915);
void car_h_27(double *state, double *unused, double *out_968849157039474287);
void car_H_27(double *state, double *unused, double *out_3690885731651022158);
void car_h_29(double *state, double *unused, double *out_6010132851772032381);
void car_H_29(double *state, double *unused, double *out_6375880387765839253);
void car_h_28(double *state, double *unused, double *out_8889732234865235921);
void car_H_28(double *state, double *unused, double *out_8339510659331165504);
void car_h_31(double *state, double *unused, double *out_3730441146199301083);
void car_H_31(double *state, double *unused, double *out_6025633952471647567);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}