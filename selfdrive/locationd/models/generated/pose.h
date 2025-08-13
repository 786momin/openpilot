#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3275079632708504218);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7534695222353310290);
void pose_H_mod_fun(double *state, double *out_4371184608249583105);
void pose_f_fun(double *state, double dt, double *out_93371807215507034);
void pose_F_fun(double *state, double dt, double *out_2962017685485010087);
void pose_h_4(double *state, double *unused, double *out_1054160117623376617);
void pose_H_4(double *state, double *unused, double *out_6949843265327654372);
void pose_h_10(double *state, double *unused, double *out_7057271041764588651);
void pose_H_10(double *state, double *unused, double *out_5778434028158511681);
void pose_h_13(double *state, double *unused, double *out_2281372985271087306);
void pose_H_13(double *state, double *unused, double *out_8284626983049564443);
void pose_h_14(double *state, double *unused, double *out_4701097407416680047);
void pose_H_14(double *state, double *unused, double *out_7533659952042412715);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}