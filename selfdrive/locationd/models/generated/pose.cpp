#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3275079632708504218) {
   out_3275079632708504218[0] = delta_x[0] + nom_x[0];
   out_3275079632708504218[1] = delta_x[1] + nom_x[1];
   out_3275079632708504218[2] = delta_x[2] + nom_x[2];
   out_3275079632708504218[3] = delta_x[3] + nom_x[3];
   out_3275079632708504218[4] = delta_x[4] + nom_x[4];
   out_3275079632708504218[5] = delta_x[5] + nom_x[5];
   out_3275079632708504218[6] = delta_x[6] + nom_x[6];
   out_3275079632708504218[7] = delta_x[7] + nom_x[7];
   out_3275079632708504218[8] = delta_x[8] + nom_x[8];
   out_3275079632708504218[9] = delta_x[9] + nom_x[9];
   out_3275079632708504218[10] = delta_x[10] + nom_x[10];
   out_3275079632708504218[11] = delta_x[11] + nom_x[11];
   out_3275079632708504218[12] = delta_x[12] + nom_x[12];
   out_3275079632708504218[13] = delta_x[13] + nom_x[13];
   out_3275079632708504218[14] = delta_x[14] + nom_x[14];
   out_3275079632708504218[15] = delta_x[15] + nom_x[15];
   out_3275079632708504218[16] = delta_x[16] + nom_x[16];
   out_3275079632708504218[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7534695222353310290) {
   out_7534695222353310290[0] = -nom_x[0] + true_x[0];
   out_7534695222353310290[1] = -nom_x[1] + true_x[1];
   out_7534695222353310290[2] = -nom_x[2] + true_x[2];
   out_7534695222353310290[3] = -nom_x[3] + true_x[3];
   out_7534695222353310290[4] = -nom_x[4] + true_x[4];
   out_7534695222353310290[5] = -nom_x[5] + true_x[5];
   out_7534695222353310290[6] = -nom_x[6] + true_x[6];
   out_7534695222353310290[7] = -nom_x[7] + true_x[7];
   out_7534695222353310290[8] = -nom_x[8] + true_x[8];
   out_7534695222353310290[9] = -nom_x[9] + true_x[9];
   out_7534695222353310290[10] = -nom_x[10] + true_x[10];
   out_7534695222353310290[11] = -nom_x[11] + true_x[11];
   out_7534695222353310290[12] = -nom_x[12] + true_x[12];
   out_7534695222353310290[13] = -nom_x[13] + true_x[13];
   out_7534695222353310290[14] = -nom_x[14] + true_x[14];
   out_7534695222353310290[15] = -nom_x[15] + true_x[15];
   out_7534695222353310290[16] = -nom_x[16] + true_x[16];
   out_7534695222353310290[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4371184608249583105) {
   out_4371184608249583105[0] = 1.0;
   out_4371184608249583105[1] = 0.0;
   out_4371184608249583105[2] = 0.0;
   out_4371184608249583105[3] = 0.0;
   out_4371184608249583105[4] = 0.0;
   out_4371184608249583105[5] = 0.0;
   out_4371184608249583105[6] = 0.0;
   out_4371184608249583105[7] = 0.0;
   out_4371184608249583105[8] = 0.0;
   out_4371184608249583105[9] = 0.0;
   out_4371184608249583105[10] = 0.0;
   out_4371184608249583105[11] = 0.0;
   out_4371184608249583105[12] = 0.0;
   out_4371184608249583105[13] = 0.0;
   out_4371184608249583105[14] = 0.0;
   out_4371184608249583105[15] = 0.0;
   out_4371184608249583105[16] = 0.0;
   out_4371184608249583105[17] = 0.0;
   out_4371184608249583105[18] = 0.0;
   out_4371184608249583105[19] = 1.0;
   out_4371184608249583105[20] = 0.0;
   out_4371184608249583105[21] = 0.0;
   out_4371184608249583105[22] = 0.0;
   out_4371184608249583105[23] = 0.0;
   out_4371184608249583105[24] = 0.0;
   out_4371184608249583105[25] = 0.0;
   out_4371184608249583105[26] = 0.0;
   out_4371184608249583105[27] = 0.0;
   out_4371184608249583105[28] = 0.0;
   out_4371184608249583105[29] = 0.0;
   out_4371184608249583105[30] = 0.0;
   out_4371184608249583105[31] = 0.0;
   out_4371184608249583105[32] = 0.0;
   out_4371184608249583105[33] = 0.0;
   out_4371184608249583105[34] = 0.0;
   out_4371184608249583105[35] = 0.0;
   out_4371184608249583105[36] = 0.0;
   out_4371184608249583105[37] = 0.0;
   out_4371184608249583105[38] = 1.0;
   out_4371184608249583105[39] = 0.0;
   out_4371184608249583105[40] = 0.0;
   out_4371184608249583105[41] = 0.0;
   out_4371184608249583105[42] = 0.0;
   out_4371184608249583105[43] = 0.0;
   out_4371184608249583105[44] = 0.0;
   out_4371184608249583105[45] = 0.0;
   out_4371184608249583105[46] = 0.0;
   out_4371184608249583105[47] = 0.0;
   out_4371184608249583105[48] = 0.0;
   out_4371184608249583105[49] = 0.0;
   out_4371184608249583105[50] = 0.0;
   out_4371184608249583105[51] = 0.0;
   out_4371184608249583105[52] = 0.0;
   out_4371184608249583105[53] = 0.0;
   out_4371184608249583105[54] = 0.0;
   out_4371184608249583105[55] = 0.0;
   out_4371184608249583105[56] = 0.0;
   out_4371184608249583105[57] = 1.0;
   out_4371184608249583105[58] = 0.0;
   out_4371184608249583105[59] = 0.0;
   out_4371184608249583105[60] = 0.0;
   out_4371184608249583105[61] = 0.0;
   out_4371184608249583105[62] = 0.0;
   out_4371184608249583105[63] = 0.0;
   out_4371184608249583105[64] = 0.0;
   out_4371184608249583105[65] = 0.0;
   out_4371184608249583105[66] = 0.0;
   out_4371184608249583105[67] = 0.0;
   out_4371184608249583105[68] = 0.0;
   out_4371184608249583105[69] = 0.0;
   out_4371184608249583105[70] = 0.0;
   out_4371184608249583105[71] = 0.0;
   out_4371184608249583105[72] = 0.0;
   out_4371184608249583105[73] = 0.0;
   out_4371184608249583105[74] = 0.0;
   out_4371184608249583105[75] = 0.0;
   out_4371184608249583105[76] = 1.0;
   out_4371184608249583105[77] = 0.0;
   out_4371184608249583105[78] = 0.0;
   out_4371184608249583105[79] = 0.0;
   out_4371184608249583105[80] = 0.0;
   out_4371184608249583105[81] = 0.0;
   out_4371184608249583105[82] = 0.0;
   out_4371184608249583105[83] = 0.0;
   out_4371184608249583105[84] = 0.0;
   out_4371184608249583105[85] = 0.0;
   out_4371184608249583105[86] = 0.0;
   out_4371184608249583105[87] = 0.0;
   out_4371184608249583105[88] = 0.0;
   out_4371184608249583105[89] = 0.0;
   out_4371184608249583105[90] = 0.0;
   out_4371184608249583105[91] = 0.0;
   out_4371184608249583105[92] = 0.0;
   out_4371184608249583105[93] = 0.0;
   out_4371184608249583105[94] = 0.0;
   out_4371184608249583105[95] = 1.0;
   out_4371184608249583105[96] = 0.0;
   out_4371184608249583105[97] = 0.0;
   out_4371184608249583105[98] = 0.0;
   out_4371184608249583105[99] = 0.0;
   out_4371184608249583105[100] = 0.0;
   out_4371184608249583105[101] = 0.0;
   out_4371184608249583105[102] = 0.0;
   out_4371184608249583105[103] = 0.0;
   out_4371184608249583105[104] = 0.0;
   out_4371184608249583105[105] = 0.0;
   out_4371184608249583105[106] = 0.0;
   out_4371184608249583105[107] = 0.0;
   out_4371184608249583105[108] = 0.0;
   out_4371184608249583105[109] = 0.0;
   out_4371184608249583105[110] = 0.0;
   out_4371184608249583105[111] = 0.0;
   out_4371184608249583105[112] = 0.0;
   out_4371184608249583105[113] = 0.0;
   out_4371184608249583105[114] = 1.0;
   out_4371184608249583105[115] = 0.0;
   out_4371184608249583105[116] = 0.0;
   out_4371184608249583105[117] = 0.0;
   out_4371184608249583105[118] = 0.0;
   out_4371184608249583105[119] = 0.0;
   out_4371184608249583105[120] = 0.0;
   out_4371184608249583105[121] = 0.0;
   out_4371184608249583105[122] = 0.0;
   out_4371184608249583105[123] = 0.0;
   out_4371184608249583105[124] = 0.0;
   out_4371184608249583105[125] = 0.0;
   out_4371184608249583105[126] = 0.0;
   out_4371184608249583105[127] = 0.0;
   out_4371184608249583105[128] = 0.0;
   out_4371184608249583105[129] = 0.0;
   out_4371184608249583105[130] = 0.0;
   out_4371184608249583105[131] = 0.0;
   out_4371184608249583105[132] = 0.0;
   out_4371184608249583105[133] = 1.0;
   out_4371184608249583105[134] = 0.0;
   out_4371184608249583105[135] = 0.0;
   out_4371184608249583105[136] = 0.0;
   out_4371184608249583105[137] = 0.0;
   out_4371184608249583105[138] = 0.0;
   out_4371184608249583105[139] = 0.0;
   out_4371184608249583105[140] = 0.0;
   out_4371184608249583105[141] = 0.0;
   out_4371184608249583105[142] = 0.0;
   out_4371184608249583105[143] = 0.0;
   out_4371184608249583105[144] = 0.0;
   out_4371184608249583105[145] = 0.0;
   out_4371184608249583105[146] = 0.0;
   out_4371184608249583105[147] = 0.0;
   out_4371184608249583105[148] = 0.0;
   out_4371184608249583105[149] = 0.0;
   out_4371184608249583105[150] = 0.0;
   out_4371184608249583105[151] = 0.0;
   out_4371184608249583105[152] = 1.0;
   out_4371184608249583105[153] = 0.0;
   out_4371184608249583105[154] = 0.0;
   out_4371184608249583105[155] = 0.0;
   out_4371184608249583105[156] = 0.0;
   out_4371184608249583105[157] = 0.0;
   out_4371184608249583105[158] = 0.0;
   out_4371184608249583105[159] = 0.0;
   out_4371184608249583105[160] = 0.0;
   out_4371184608249583105[161] = 0.0;
   out_4371184608249583105[162] = 0.0;
   out_4371184608249583105[163] = 0.0;
   out_4371184608249583105[164] = 0.0;
   out_4371184608249583105[165] = 0.0;
   out_4371184608249583105[166] = 0.0;
   out_4371184608249583105[167] = 0.0;
   out_4371184608249583105[168] = 0.0;
   out_4371184608249583105[169] = 0.0;
   out_4371184608249583105[170] = 0.0;
   out_4371184608249583105[171] = 1.0;
   out_4371184608249583105[172] = 0.0;
   out_4371184608249583105[173] = 0.0;
   out_4371184608249583105[174] = 0.0;
   out_4371184608249583105[175] = 0.0;
   out_4371184608249583105[176] = 0.0;
   out_4371184608249583105[177] = 0.0;
   out_4371184608249583105[178] = 0.0;
   out_4371184608249583105[179] = 0.0;
   out_4371184608249583105[180] = 0.0;
   out_4371184608249583105[181] = 0.0;
   out_4371184608249583105[182] = 0.0;
   out_4371184608249583105[183] = 0.0;
   out_4371184608249583105[184] = 0.0;
   out_4371184608249583105[185] = 0.0;
   out_4371184608249583105[186] = 0.0;
   out_4371184608249583105[187] = 0.0;
   out_4371184608249583105[188] = 0.0;
   out_4371184608249583105[189] = 0.0;
   out_4371184608249583105[190] = 1.0;
   out_4371184608249583105[191] = 0.0;
   out_4371184608249583105[192] = 0.0;
   out_4371184608249583105[193] = 0.0;
   out_4371184608249583105[194] = 0.0;
   out_4371184608249583105[195] = 0.0;
   out_4371184608249583105[196] = 0.0;
   out_4371184608249583105[197] = 0.0;
   out_4371184608249583105[198] = 0.0;
   out_4371184608249583105[199] = 0.0;
   out_4371184608249583105[200] = 0.0;
   out_4371184608249583105[201] = 0.0;
   out_4371184608249583105[202] = 0.0;
   out_4371184608249583105[203] = 0.0;
   out_4371184608249583105[204] = 0.0;
   out_4371184608249583105[205] = 0.0;
   out_4371184608249583105[206] = 0.0;
   out_4371184608249583105[207] = 0.0;
   out_4371184608249583105[208] = 0.0;
   out_4371184608249583105[209] = 1.0;
   out_4371184608249583105[210] = 0.0;
   out_4371184608249583105[211] = 0.0;
   out_4371184608249583105[212] = 0.0;
   out_4371184608249583105[213] = 0.0;
   out_4371184608249583105[214] = 0.0;
   out_4371184608249583105[215] = 0.0;
   out_4371184608249583105[216] = 0.0;
   out_4371184608249583105[217] = 0.0;
   out_4371184608249583105[218] = 0.0;
   out_4371184608249583105[219] = 0.0;
   out_4371184608249583105[220] = 0.0;
   out_4371184608249583105[221] = 0.0;
   out_4371184608249583105[222] = 0.0;
   out_4371184608249583105[223] = 0.0;
   out_4371184608249583105[224] = 0.0;
   out_4371184608249583105[225] = 0.0;
   out_4371184608249583105[226] = 0.0;
   out_4371184608249583105[227] = 0.0;
   out_4371184608249583105[228] = 1.0;
   out_4371184608249583105[229] = 0.0;
   out_4371184608249583105[230] = 0.0;
   out_4371184608249583105[231] = 0.0;
   out_4371184608249583105[232] = 0.0;
   out_4371184608249583105[233] = 0.0;
   out_4371184608249583105[234] = 0.0;
   out_4371184608249583105[235] = 0.0;
   out_4371184608249583105[236] = 0.0;
   out_4371184608249583105[237] = 0.0;
   out_4371184608249583105[238] = 0.0;
   out_4371184608249583105[239] = 0.0;
   out_4371184608249583105[240] = 0.0;
   out_4371184608249583105[241] = 0.0;
   out_4371184608249583105[242] = 0.0;
   out_4371184608249583105[243] = 0.0;
   out_4371184608249583105[244] = 0.0;
   out_4371184608249583105[245] = 0.0;
   out_4371184608249583105[246] = 0.0;
   out_4371184608249583105[247] = 1.0;
   out_4371184608249583105[248] = 0.0;
   out_4371184608249583105[249] = 0.0;
   out_4371184608249583105[250] = 0.0;
   out_4371184608249583105[251] = 0.0;
   out_4371184608249583105[252] = 0.0;
   out_4371184608249583105[253] = 0.0;
   out_4371184608249583105[254] = 0.0;
   out_4371184608249583105[255] = 0.0;
   out_4371184608249583105[256] = 0.0;
   out_4371184608249583105[257] = 0.0;
   out_4371184608249583105[258] = 0.0;
   out_4371184608249583105[259] = 0.0;
   out_4371184608249583105[260] = 0.0;
   out_4371184608249583105[261] = 0.0;
   out_4371184608249583105[262] = 0.0;
   out_4371184608249583105[263] = 0.0;
   out_4371184608249583105[264] = 0.0;
   out_4371184608249583105[265] = 0.0;
   out_4371184608249583105[266] = 1.0;
   out_4371184608249583105[267] = 0.0;
   out_4371184608249583105[268] = 0.0;
   out_4371184608249583105[269] = 0.0;
   out_4371184608249583105[270] = 0.0;
   out_4371184608249583105[271] = 0.0;
   out_4371184608249583105[272] = 0.0;
   out_4371184608249583105[273] = 0.0;
   out_4371184608249583105[274] = 0.0;
   out_4371184608249583105[275] = 0.0;
   out_4371184608249583105[276] = 0.0;
   out_4371184608249583105[277] = 0.0;
   out_4371184608249583105[278] = 0.0;
   out_4371184608249583105[279] = 0.0;
   out_4371184608249583105[280] = 0.0;
   out_4371184608249583105[281] = 0.0;
   out_4371184608249583105[282] = 0.0;
   out_4371184608249583105[283] = 0.0;
   out_4371184608249583105[284] = 0.0;
   out_4371184608249583105[285] = 1.0;
   out_4371184608249583105[286] = 0.0;
   out_4371184608249583105[287] = 0.0;
   out_4371184608249583105[288] = 0.0;
   out_4371184608249583105[289] = 0.0;
   out_4371184608249583105[290] = 0.0;
   out_4371184608249583105[291] = 0.0;
   out_4371184608249583105[292] = 0.0;
   out_4371184608249583105[293] = 0.0;
   out_4371184608249583105[294] = 0.0;
   out_4371184608249583105[295] = 0.0;
   out_4371184608249583105[296] = 0.0;
   out_4371184608249583105[297] = 0.0;
   out_4371184608249583105[298] = 0.0;
   out_4371184608249583105[299] = 0.0;
   out_4371184608249583105[300] = 0.0;
   out_4371184608249583105[301] = 0.0;
   out_4371184608249583105[302] = 0.0;
   out_4371184608249583105[303] = 0.0;
   out_4371184608249583105[304] = 1.0;
   out_4371184608249583105[305] = 0.0;
   out_4371184608249583105[306] = 0.0;
   out_4371184608249583105[307] = 0.0;
   out_4371184608249583105[308] = 0.0;
   out_4371184608249583105[309] = 0.0;
   out_4371184608249583105[310] = 0.0;
   out_4371184608249583105[311] = 0.0;
   out_4371184608249583105[312] = 0.0;
   out_4371184608249583105[313] = 0.0;
   out_4371184608249583105[314] = 0.0;
   out_4371184608249583105[315] = 0.0;
   out_4371184608249583105[316] = 0.0;
   out_4371184608249583105[317] = 0.0;
   out_4371184608249583105[318] = 0.0;
   out_4371184608249583105[319] = 0.0;
   out_4371184608249583105[320] = 0.0;
   out_4371184608249583105[321] = 0.0;
   out_4371184608249583105[322] = 0.0;
   out_4371184608249583105[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_93371807215507034) {
   out_93371807215507034[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_93371807215507034[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_93371807215507034[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_93371807215507034[3] = dt*state[12] + state[3];
   out_93371807215507034[4] = dt*state[13] + state[4];
   out_93371807215507034[5] = dt*state[14] + state[5];
   out_93371807215507034[6] = state[6];
   out_93371807215507034[7] = state[7];
   out_93371807215507034[8] = state[8];
   out_93371807215507034[9] = state[9];
   out_93371807215507034[10] = state[10];
   out_93371807215507034[11] = state[11];
   out_93371807215507034[12] = state[12];
   out_93371807215507034[13] = state[13];
   out_93371807215507034[14] = state[14];
   out_93371807215507034[15] = state[15];
   out_93371807215507034[16] = state[16];
   out_93371807215507034[17] = state[17];
}
void F_fun(double *state, double dt, double *out_2962017685485010087) {
   out_2962017685485010087[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2962017685485010087[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2962017685485010087[2] = 0;
   out_2962017685485010087[3] = 0;
   out_2962017685485010087[4] = 0;
   out_2962017685485010087[5] = 0;
   out_2962017685485010087[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2962017685485010087[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2962017685485010087[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2962017685485010087[9] = 0;
   out_2962017685485010087[10] = 0;
   out_2962017685485010087[11] = 0;
   out_2962017685485010087[12] = 0;
   out_2962017685485010087[13] = 0;
   out_2962017685485010087[14] = 0;
   out_2962017685485010087[15] = 0;
   out_2962017685485010087[16] = 0;
   out_2962017685485010087[17] = 0;
   out_2962017685485010087[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2962017685485010087[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2962017685485010087[20] = 0;
   out_2962017685485010087[21] = 0;
   out_2962017685485010087[22] = 0;
   out_2962017685485010087[23] = 0;
   out_2962017685485010087[24] = 0;
   out_2962017685485010087[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2962017685485010087[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2962017685485010087[27] = 0;
   out_2962017685485010087[28] = 0;
   out_2962017685485010087[29] = 0;
   out_2962017685485010087[30] = 0;
   out_2962017685485010087[31] = 0;
   out_2962017685485010087[32] = 0;
   out_2962017685485010087[33] = 0;
   out_2962017685485010087[34] = 0;
   out_2962017685485010087[35] = 0;
   out_2962017685485010087[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2962017685485010087[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2962017685485010087[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2962017685485010087[39] = 0;
   out_2962017685485010087[40] = 0;
   out_2962017685485010087[41] = 0;
   out_2962017685485010087[42] = 0;
   out_2962017685485010087[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2962017685485010087[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2962017685485010087[45] = 0;
   out_2962017685485010087[46] = 0;
   out_2962017685485010087[47] = 0;
   out_2962017685485010087[48] = 0;
   out_2962017685485010087[49] = 0;
   out_2962017685485010087[50] = 0;
   out_2962017685485010087[51] = 0;
   out_2962017685485010087[52] = 0;
   out_2962017685485010087[53] = 0;
   out_2962017685485010087[54] = 0;
   out_2962017685485010087[55] = 0;
   out_2962017685485010087[56] = 0;
   out_2962017685485010087[57] = 1;
   out_2962017685485010087[58] = 0;
   out_2962017685485010087[59] = 0;
   out_2962017685485010087[60] = 0;
   out_2962017685485010087[61] = 0;
   out_2962017685485010087[62] = 0;
   out_2962017685485010087[63] = 0;
   out_2962017685485010087[64] = 0;
   out_2962017685485010087[65] = 0;
   out_2962017685485010087[66] = dt;
   out_2962017685485010087[67] = 0;
   out_2962017685485010087[68] = 0;
   out_2962017685485010087[69] = 0;
   out_2962017685485010087[70] = 0;
   out_2962017685485010087[71] = 0;
   out_2962017685485010087[72] = 0;
   out_2962017685485010087[73] = 0;
   out_2962017685485010087[74] = 0;
   out_2962017685485010087[75] = 0;
   out_2962017685485010087[76] = 1;
   out_2962017685485010087[77] = 0;
   out_2962017685485010087[78] = 0;
   out_2962017685485010087[79] = 0;
   out_2962017685485010087[80] = 0;
   out_2962017685485010087[81] = 0;
   out_2962017685485010087[82] = 0;
   out_2962017685485010087[83] = 0;
   out_2962017685485010087[84] = 0;
   out_2962017685485010087[85] = dt;
   out_2962017685485010087[86] = 0;
   out_2962017685485010087[87] = 0;
   out_2962017685485010087[88] = 0;
   out_2962017685485010087[89] = 0;
   out_2962017685485010087[90] = 0;
   out_2962017685485010087[91] = 0;
   out_2962017685485010087[92] = 0;
   out_2962017685485010087[93] = 0;
   out_2962017685485010087[94] = 0;
   out_2962017685485010087[95] = 1;
   out_2962017685485010087[96] = 0;
   out_2962017685485010087[97] = 0;
   out_2962017685485010087[98] = 0;
   out_2962017685485010087[99] = 0;
   out_2962017685485010087[100] = 0;
   out_2962017685485010087[101] = 0;
   out_2962017685485010087[102] = 0;
   out_2962017685485010087[103] = 0;
   out_2962017685485010087[104] = dt;
   out_2962017685485010087[105] = 0;
   out_2962017685485010087[106] = 0;
   out_2962017685485010087[107] = 0;
   out_2962017685485010087[108] = 0;
   out_2962017685485010087[109] = 0;
   out_2962017685485010087[110] = 0;
   out_2962017685485010087[111] = 0;
   out_2962017685485010087[112] = 0;
   out_2962017685485010087[113] = 0;
   out_2962017685485010087[114] = 1;
   out_2962017685485010087[115] = 0;
   out_2962017685485010087[116] = 0;
   out_2962017685485010087[117] = 0;
   out_2962017685485010087[118] = 0;
   out_2962017685485010087[119] = 0;
   out_2962017685485010087[120] = 0;
   out_2962017685485010087[121] = 0;
   out_2962017685485010087[122] = 0;
   out_2962017685485010087[123] = 0;
   out_2962017685485010087[124] = 0;
   out_2962017685485010087[125] = 0;
   out_2962017685485010087[126] = 0;
   out_2962017685485010087[127] = 0;
   out_2962017685485010087[128] = 0;
   out_2962017685485010087[129] = 0;
   out_2962017685485010087[130] = 0;
   out_2962017685485010087[131] = 0;
   out_2962017685485010087[132] = 0;
   out_2962017685485010087[133] = 1;
   out_2962017685485010087[134] = 0;
   out_2962017685485010087[135] = 0;
   out_2962017685485010087[136] = 0;
   out_2962017685485010087[137] = 0;
   out_2962017685485010087[138] = 0;
   out_2962017685485010087[139] = 0;
   out_2962017685485010087[140] = 0;
   out_2962017685485010087[141] = 0;
   out_2962017685485010087[142] = 0;
   out_2962017685485010087[143] = 0;
   out_2962017685485010087[144] = 0;
   out_2962017685485010087[145] = 0;
   out_2962017685485010087[146] = 0;
   out_2962017685485010087[147] = 0;
   out_2962017685485010087[148] = 0;
   out_2962017685485010087[149] = 0;
   out_2962017685485010087[150] = 0;
   out_2962017685485010087[151] = 0;
   out_2962017685485010087[152] = 1;
   out_2962017685485010087[153] = 0;
   out_2962017685485010087[154] = 0;
   out_2962017685485010087[155] = 0;
   out_2962017685485010087[156] = 0;
   out_2962017685485010087[157] = 0;
   out_2962017685485010087[158] = 0;
   out_2962017685485010087[159] = 0;
   out_2962017685485010087[160] = 0;
   out_2962017685485010087[161] = 0;
   out_2962017685485010087[162] = 0;
   out_2962017685485010087[163] = 0;
   out_2962017685485010087[164] = 0;
   out_2962017685485010087[165] = 0;
   out_2962017685485010087[166] = 0;
   out_2962017685485010087[167] = 0;
   out_2962017685485010087[168] = 0;
   out_2962017685485010087[169] = 0;
   out_2962017685485010087[170] = 0;
   out_2962017685485010087[171] = 1;
   out_2962017685485010087[172] = 0;
   out_2962017685485010087[173] = 0;
   out_2962017685485010087[174] = 0;
   out_2962017685485010087[175] = 0;
   out_2962017685485010087[176] = 0;
   out_2962017685485010087[177] = 0;
   out_2962017685485010087[178] = 0;
   out_2962017685485010087[179] = 0;
   out_2962017685485010087[180] = 0;
   out_2962017685485010087[181] = 0;
   out_2962017685485010087[182] = 0;
   out_2962017685485010087[183] = 0;
   out_2962017685485010087[184] = 0;
   out_2962017685485010087[185] = 0;
   out_2962017685485010087[186] = 0;
   out_2962017685485010087[187] = 0;
   out_2962017685485010087[188] = 0;
   out_2962017685485010087[189] = 0;
   out_2962017685485010087[190] = 1;
   out_2962017685485010087[191] = 0;
   out_2962017685485010087[192] = 0;
   out_2962017685485010087[193] = 0;
   out_2962017685485010087[194] = 0;
   out_2962017685485010087[195] = 0;
   out_2962017685485010087[196] = 0;
   out_2962017685485010087[197] = 0;
   out_2962017685485010087[198] = 0;
   out_2962017685485010087[199] = 0;
   out_2962017685485010087[200] = 0;
   out_2962017685485010087[201] = 0;
   out_2962017685485010087[202] = 0;
   out_2962017685485010087[203] = 0;
   out_2962017685485010087[204] = 0;
   out_2962017685485010087[205] = 0;
   out_2962017685485010087[206] = 0;
   out_2962017685485010087[207] = 0;
   out_2962017685485010087[208] = 0;
   out_2962017685485010087[209] = 1;
   out_2962017685485010087[210] = 0;
   out_2962017685485010087[211] = 0;
   out_2962017685485010087[212] = 0;
   out_2962017685485010087[213] = 0;
   out_2962017685485010087[214] = 0;
   out_2962017685485010087[215] = 0;
   out_2962017685485010087[216] = 0;
   out_2962017685485010087[217] = 0;
   out_2962017685485010087[218] = 0;
   out_2962017685485010087[219] = 0;
   out_2962017685485010087[220] = 0;
   out_2962017685485010087[221] = 0;
   out_2962017685485010087[222] = 0;
   out_2962017685485010087[223] = 0;
   out_2962017685485010087[224] = 0;
   out_2962017685485010087[225] = 0;
   out_2962017685485010087[226] = 0;
   out_2962017685485010087[227] = 0;
   out_2962017685485010087[228] = 1;
   out_2962017685485010087[229] = 0;
   out_2962017685485010087[230] = 0;
   out_2962017685485010087[231] = 0;
   out_2962017685485010087[232] = 0;
   out_2962017685485010087[233] = 0;
   out_2962017685485010087[234] = 0;
   out_2962017685485010087[235] = 0;
   out_2962017685485010087[236] = 0;
   out_2962017685485010087[237] = 0;
   out_2962017685485010087[238] = 0;
   out_2962017685485010087[239] = 0;
   out_2962017685485010087[240] = 0;
   out_2962017685485010087[241] = 0;
   out_2962017685485010087[242] = 0;
   out_2962017685485010087[243] = 0;
   out_2962017685485010087[244] = 0;
   out_2962017685485010087[245] = 0;
   out_2962017685485010087[246] = 0;
   out_2962017685485010087[247] = 1;
   out_2962017685485010087[248] = 0;
   out_2962017685485010087[249] = 0;
   out_2962017685485010087[250] = 0;
   out_2962017685485010087[251] = 0;
   out_2962017685485010087[252] = 0;
   out_2962017685485010087[253] = 0;
   out_2962017685485010087[254] = 0;
   out_2962017685485010087[255] = 0;
   out_2962017685485010087[256] = 0;
   out_2962017685485010087[257] = 0;
   out_2962017685485010087[258] = 0;
   out_2962017685485010087[259] = 0;
   out_2962017685485010087[260] = 0;
   out_2962017685485010087[261] = 0;
   out_2962017685485010087[262] = 0;
   out_2962017685485010087[263] = 0;
   out_2962017685485010087[264] = 0;
   out_2962017685485010087[265] = 0;
   out_2962017685485010087[266] = 1;
   out_2962017685485010087[267] = 0;
   out_2962017685485010087[268] = 0;
   out_2962017685485010087[269] = 0;
   out_2962017685485010087[270] = 0;
   out_2962017685485010087[271] = 0;
   out_2962017685485010087[272] = 0;
   out_2962017685485010087[273] = 0;
   out_2962017685485010087[274] = 0;
   out_2962017685485010087[275] = 0;
   out_2962017685485010087[276] = 0;
   out_2962017685485010087[277] = 0;
   out_2962017685485010087[278] = 0;
   out_2962017685485010087[279] = 0;
   out_2962017685485010087[280] = 0;
   out_2962017685485010087[281] = 0;
   out_2962017685485010087[282] = 0;
   out_2962017685485010087[283] = 0;
   out_2962017685485010087[284] = 0;
   out_2962017685485010087[285] = 1;
   out_2962017685485010087[286] = 0;
   out_2962017685485010087[287] = 0;
   out_2962017685485010087[288] = 0;
   out_2962017685485010087[289] = 0;
   out_2962017685485010087[290] = 0;
   out_2962017685485010087[291] = 0;
   out_2962017685485010087[292] = 0;
   out_2962017685485010087[293] = 0;
   out_2962017685485010087[294] = 0;
   out_2962017685485010087[295] = 0;
   out_2962017685485010087[296] = 0;
   out_2962017685485010087[297] = 0;
   out_2962017685485010087[298] = 0;
   out_2962017685485010087[299] = 0;
   out_2962017685485010087[300] = 0;
   out_2962017685485010087[301] = 0;
   out_2962017685485010087[302] = 0;
   out_2962017685485010087[303] = 0;
   out_2962017685485010087[304] = 1;
   out_2962017685485010087[305] = 0;
   out_2962017685485010087[306] = 0;
   out_2962017685485010087[307] = 0;
   out_2962017685485010087[308] = 0;
   out_2962017685485010087[309] = 0;
   out_2962017685485010087[310] = 0;
   out_2962017685485010087[311] = 0;
   out_2962017685485010087[312] = 0;
   out_2962017685485010087[313] = 0;
   out_2962017685485010087[314] = 0;
   out_2962017685485010087[315] = 0;
   out_2962017685485010087[316] = 0;
   out_2962017685485010087[317] = 0;
   out_2962017685485010087[318] = 0;
   out_2962017685485010087[319] = 0;
   out_2962017685485010087[320] = 0;
   out_2962017685485010087[321] = 0;
   out_2962017685485010087[322] = 0;
   out_2962017685485010087[323] = 1;
}
void h_4(double *state, double *unused, double *out_1054160117623376617) {
   out_1054160117623376617[0] = state[6] + state[9];
   out_1054160117623376617[1] = state[7] + state[10];
   out_1054160117623376617[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6949843265327654372) {
   out_6949843265327654372[0] = 0;
   out_6949843265327654372[1] = 0;
   out_6949843265327654372[2] = 0;
   out_6949843265327654372[3] = 0;
   out_6949843265327654372[4] = 0;
   out_6949843265327654372[5] = 0;
   out_6949843265327654372[6] = 1;
   out_6949843265327654372[7] = 0;
   out_6949843265327654372[8] = 0;
   out_6949843265327654372[9] = 1;
   out_6949843265327654372[10] = 0;
   out_6949843265327654372[11] = 0;
   out_6949843265327654372[12] = 0;
   out_6949843265327654372[13] = 0;
   out_6949843265327654372[14] = 0;
   out_6949843265327654372[15] = 0;
   out_6949843265327654372[16] = 0;
   out_6949843265327654372[17] = 0;
   out_6949843265327654372[18] = 0;
   out_6949843265327654372[19] = 0;
   out_6949843265327654372[20] = 0;
   out_6949843265327654372[21] = 0;
   out_6949843265327654372[22] = 0;
   out_6949843265327654372[23] = 0;
   out_6949843265327654372[24] = 0;
   out_6949843265327654372[25] = 1;
   out_6949843265327654372[26] = 0;
   out_6949843265327654372[27] = 0;
   out_6949843265327654372[28] = 1;
   out_6949843265327654372[29] = 0;
   out_6949843265327654372[30] = 0;
   out_6949843265327654372[31] = 0;
   out_6949843265327654372[32] = 0;
   out_6949843265327654372[33] = 0;
   out_6949843265327654372[34] = 0;
   out_6949843265327654372[35] = 0;
   out_6949843265327654372[36] = 0;
   out_6949843265327654372[37] = 0;
   out_6949843265327654372[38] = 0;
   out_6949843265327654372[39] = 0;
   out_6949843265327654372[40] = 0;
   out_6949843265327654372[41] = 0;
   out_6949843265327654372[42] = 0;
   out_6949843265327654372[43] = 0;
   out_6949843265327654372[44] = 1;
   out_6949843265327654372[45] = 0;
   out_6949843265327654372[46] = 0;
   out_6949843265327654372[47] = 1;
   out_6949843265327654372[48] = 0;
   out_6949843265327654372[49] = 0;
   out_6949843265327654372[50] = 0;
   out_6949843265327654372[51] = 0;
   out_6949843265327654372[52] = 0;
   out_6949843265327654372[53] = 0;
}
void h_10(double *state, double *unused, double *out_7057271041764588651) {
   out_7057271041764588651[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7057271041764588651[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7057271041764588651[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5778434028158511681) {
   out_5778434028158511681[0] = 0;
   out_5778434028158511681[1] = 9.8100000000000005*cos(state[1]);
   out_5778434028158511681[2] = 0;
   out_5778434028158511681[3] = 0;
   out_5778434028158511681[4] = -state[8];
   out_5778434028158511681[5] = state[7];
   out_5778434028158511681[6] = 0;
   out_5778434028158511681[7] = state[5];
   out_5778434028158511681[8] = -state[4];
   out_5778434028158511681[9] = 0;
   out_5778434028158511681[10] = 0;
   out_5778434028158511681[11] = 0;
   out_5778434028158511681[12] = 1;
   out_5778434028158511681[13] = 0;
   out_5778434028158511681[14] = 0;
   out_5778434028158511681[15] = 1;
   out_5778434028158511681[16] = 0;
   out_5778434028158511681[17] = 0;
   out_5778434028158511681[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5778434028158511681[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5778434028158511681[20] = 0;
   out_5778434028158511681[21] = state[8];
   out_5778434028158511681[22] = 0;
   out_5778434028158511681[23] = -state[6];
   out_5778434028158511681[24] = -state[5];
   out_5778434028158511681[25] = 0;
   out_5778434028158511681[26] = state[3];
   out_5778434028158511681[27] = 0;
   out_5778434028158511681[28] = 0;
   out_5778434028158511681[29] = 0;
   out_5778434028158511681[30] = 0;
   out_5778434028158511681[31] = 1;
   out_5778434028158511681[32] = 0;
   out_5778434028158511681[33] = 0;
   out_5778434028158511681[34] = 1;
   out_5778434028158511681[35] = 0;
   out_5778434028158511681[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5778434028158511681[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5778434028158511681[38] = 0;
   out_5778434028158511681[39] = -state[7];
   out_5778434028158511681[40] = state[6];
   out_5778434028158511681[41] = 0;
   out_5778434028158511681[42] = state[4];
   out_5778434028158511681[43] = -state[3];
   out_5778434028158511681[44] = 0;
   out_5778434028158511681[45] = 0;
   out_5778434028158511681[46] = 0;
   out_5778434028158511681[47] = 0;
   out_5778434028158511681[48] = 0;
   out_5778434028158511681[49] = 0;
   out_5778434028158511681[50] = 1;
   out_5778434028158511681[51] = 0;
   out_5778434028158511681[52] = 0;
   out_5778434028158511681[53] = 1;
}
void h_13(double *state, double *unused, double *out_2281372985271087306) {
   out_2281372985271087306[0] = state[3];
   out_2281372985271087306[1] = state[4];
   out_2281372985271087306[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8284626983049564443) {
   out_8284626983049564443[0] = 0;
   out_8284626983049564443[1] = 0;
   out_8284626983049564443[2] = 0;
   out_8284626983049564443[3] = 1;
   out_8284626983049564443[4] = 0;
   out_8284626983049564443[5] = 0;
   out_8284626983049564443[6] = 0;
   out_8284626983049564443[7] = 0;
   out_8284626983049564443[8] = 0;
   out_8284626983049564443[9] = 0;
   out_8284626983049564443[10] = 0;
   out_8284626983049564443[11] = 0;
   out_8284626983049564443[12] = 0;
   out_8284626983049564443[13] = 0;
   out_8284626983049564443[14] = 0;
   out_8284626983049564443[15] = 0;
   out_8284626983049564443[16] = 0;
   out_8284626983049564443[17] = 0;
   out_8284626983049564443[18] = 0;
   out_8284626983049564443[19] = 0;
   out_8284626983049564443[20] = 0;
   out_8284626983049564443[21] = 0;
   out_8284626983049564443[22] = 1;
   out_8284626983049564443[23] = 0;
   out_8284626983049564443[24] = 0;
   out_8284626983049564443[25] = 0;
   out_8284626983049564443[26] = 0;
   out_8284626983049564443[27] = 0;
   out_8284626983049564443[28] = 0;
   out_8284626983049564443[29] = 0;
   out_8284626983049564443[30] = 0;
   out_8284626983049564443[31] = 0;
   out_8284626983049564443[32] = 0;
   out_8284626983049564443[33] = 0;
   out_8284626983049564443[34] = 0;
   out_8284626983049564443[35] = 0;
   out_8284626983049564443[36] = 0;
   out_8284626983049564443[37] = 0;
   out_8284626983049564443[38] = 0;
   out_8284626983049564443[39] = 0;
   out_8284626983049564443[40] = 0;
   out_8284626983049564443[41] = 1;
   out_8284626983049564443[42] = 0;
   out_8284626983049564443[43] = 0;
   out_8284626983049564443[44] = 0;
   out_8284626983049564443[45] = 0;
   out_8284626983049564443[46] = 0;
   out_8284626983049564443[47] = 0;
   out_8284626983049564443[48] = 0;
   out_8284626983049564443[49] = 0;
   out_8284626983049564443[50] = 0;
   out_8284626983049564443[51] = 0;
   out_8284626983049564443[52] = 0;
   out_8284626983049564443[53] = 0;
}
void h_14(double *state, double *unused, double *out_4701097407416680047) {
   out_4701097407416680047[0] = state[6];
   out_4701097407416680047[1] = state[7];
   out_4701097407416680047[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7533659952042412715) {
   out_7533659952042412715[0] = 0;
   out_7533659952042412715[1] = 0;
   out_7533659952042412715[2] = 0;
   out_7533659952042412715[3] = 0;
   out_7533659952042412715[4] = 0;
   out_7533659952042412715[5] = 0;
   out_7533659952042412715[6] = 1;
   out_7533659952042412715[7] = 0;
   out_7533659952042412715[8] = 0;
   out_7533659952042412715[9] = 0;
   out_7533659952042412715[10] = 0;
   out_7533659952042412715[11] = 0;
   out_7533659952042412715[12] = 0;
   out_7533659952042412715[13] = 0;
   out_7533659952042412715[14] = 0;
   out_7533659952042412715[15] = 0;
   out_7533659952042412715[16] = 0;
   out_7533659952042412715[17] = 0;
   out_7533659952042412715[18] = 0;
   out_7533659952042412715[19] = 0;
   out_7533659952042412715[20] = 0;
   out_7533659952042412715[21] = 0;
   out_7533659952042412715[22] = 0;
   out_7533659952042412715[23] = 0;
   out_7533659952042412715[24] = 0;
   out_7533659952042412715[25] = 1;
   out_7533659952042412715[26] = 0;
   out_7533659952042412715[27] = 0;
   out_7533659952042412715[28] = 0;
   out_7533659952042412715[29] = 0;
   out_7533659952042412715[30] = 0;
   out_7533659952042412715[31] = 0;
   out_7533659952042412715[32] = 0;
   out_7533659952042412715[33] = 0;
   out_7533659952042412715[34] = 0;
   out_7533659952042412715[35] = 0;
   out_7533659952042412715[36] = 0;
   out_7533659952042412715[37] = 0;
   out_7533659952042412715[38] = 0;
   out_7533659952042412715[39] = 0;
   out_7533659952042412715[40] = 0;
   out_7533659952042412715[41] = 0;
   out_7533659952042412715[42] = 0;
   out_7533659952042412715[43] = 0;
   out_7533659952042412715[44] = 1;
   out_7533659952042412715[45] = 0;
   out_7533659952042412715[46] = 0;
   out_7533659952042412715[47] = 0;
   out_7533659952042412715[48] = 0;
   out_7533659952042412715[49] = 0;
   out_7533659952042412715[50] = 0;
   out_7533659952042412715[51] = 0;
   out_7533659952042412715[52] = 0;
   out_7533659952042412715[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_3275079632708504218) {
  err_fun(nom_x, delta_x, out_3275079632708504218);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7534695222353310290) {
  inv_err_fun(nom_x, true_x, out_7534695222353310290);
}
void pose_H_mod_fun(double *state, double *out_4371184608249583105) {
  H_mod_fun(state, out_4371184608249583105);
}
void pose_f_fun(double *state, double dt, double *out_93371807215507034) {
  f_fun(state,  dt, out_93371807215507034);
}
void pose_F_fun(double *state, double dt, double *out_2962017685485010087) {
  F_fun(state,  dt, out_2962017685485010087);
}
void pose_h_4(double *state, double *unused, double *out_1054160117623376617) {
  h_4(state, unused, out_1054160117623376617);
}
void pose_H_4(double *state, double *unused, double *out_6949843265327654372) {
  H_4(state, unused, out_6949843265327654372);
}
void pose_h_10(double *state, double *unused, double *out_7057271041764588651) {
  h_10(state, unused, out_7057271041764588651);
}
void pose_H_10(double *state, double *unused, double *out_5778434028158511681) {
  H_10(state, unused, out_5778434028158511681);
}
void pose_h_13(double *state, double *unused, double *out_2281372985271087306) {
  h_13(state, unused, out_2281372985271087306);
}
void pose_H_13(double *state, double *unused, double *out_8284626983049564443) {
  H_13(state, unused, out_8284626983049564443);
}
void pose_h_14(double *state, double *unused, double *out_4701097407416680047) {
  h_14(state, unused, out_4701097407416680047);
}
void pose_H_14(double *state, double *unused, double *out_7533659952042412715) {
  H_14(state, unused, out_7533659952042412715);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
