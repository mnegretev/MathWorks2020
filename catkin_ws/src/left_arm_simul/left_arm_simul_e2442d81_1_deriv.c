/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'left_arm_simul/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "left_arm_simul_e2442d81_1_geometries.h"

PmfMessageId left_arm_simul_e2442d81_1_deriv(const RuntimeDerivedValuesBundle
  *rtdv, const int *eqnEnableFlags, const double *state, const int *modeVector,
  const double *input, const double *inputDot, const double *inputDdot, const
  double *discreteState, double *deriv, double *errorResult,
  NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[209];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 1.5;
  xx[1] = 1.570795;
  xx[2] = 0.0;
  xx[3] = state[0] + xx[1];
  if (xx[2] < xx[3])
    xx[3] = xx[2];
  xx[4] = 1.74532925199433e-3;
  xx[5] = 1.0;
  xx[6] = - (xx[3] / xx[4]);
  if (xx[5] < xx[6])
    xx[6] = xx[5];
  xx[7] = 3.0;
  xx[8] = 2.0;
  xx[9] = 572.9577951308232;
  xx[10] = xx[9] * state[1];
  xx[11] = 5.729577951308233e5;
  xx[12] = xx[6] * xx[6] * (xx[7] - xx[8] * xx[6]) * ((- xx[3] == xx[2] ? xx[2] :
    - xx[10]) - xx[11] * xx[3]);
  if (xx[2] > xx[12])
    xx[12] = xx[2];
  xx[3] = state[0] - xx[1];
  if (xx[2] > xx[3])
    xx[3] = xx[2];
  xx[6] = xx[3] / xx[4];
  if (xx[5] < xx[6])
    xx[6] = xx[5];
  xx[13] = xx[6] * xx[6] * (xx[7] - xx[8] * xx[6]) * (xx[11] * xx[3] + (xx[3] ==
    xx[2] ? xx[2] : xx[10]));
  if (xx[2] > xx[13])
    xx[13] = xx[2];
  xx[3] = 0.7071054825112363;
  xx[6] = 0.5;
  xx[10] = xx[6] * state[2];
  xx[14] = cos(xx[10]);
  xx[15] = xx[3] * xx[14];
  xx[16] = 0.7071080798594735;
  xx[17] = xx[16] * xx[14];
  xx[14] = - xx[17];
  xx[18] = sin(xx[10]);
  xx[10] = xx[16] * xx[18];
  xx[19] = xx[3] * xx[18];
  xx[18] = - xx[19];
  xx[20] = - xx[15];
  xx[21] = xx[14];
  xx[22] = xx[10];
  xx[23] = xx[18];
  xx[24] = xx[10] * state[1];
  xx[25] = xx[17] * state[1];
  xx[26] = xx[8] * (xx[15] * xx[24] + xx[19] * xx[25]);
  xx[27] = xx[8] * (xx[15] * xx[25] - xx[19] * xx[24]);
  xx[28] = state[1] - xx[8] * (xx[17] * xx[25] + xx[10] * xx[24]);
  xx[24] = xx[28] + state[3];
  xx[29] = xx[26];
  xx[30] = xx[27];
  xx[31] = xx[24];
  xx[25] = 0.3;
  xx[32] = xx[25] * xx[26];
  xx[33] = xx[25] * xx[27];
  xx[34] = xx[25] * xx[24];
  pm_math_Vector3_cross_ra(xx + 29, xx + 32, xx + 35);
  xx[32] = 0.4999999999966269;
  xx[33] = xx[6] * state[4];
  xx[34] = sin(xx[33]);
  xx[38] = xx[32] * xx[34];
  xx[39] = 0.4999981633974483;
  xx[40] = cos(xx[33]);
  xx[33] = xx[38] - xx[39] * xx[40];
  xx[41] = xx[32] * xx[40];
  xx[42] = 0.5000018366025516;
  xx[43] = xx[41] + xx[42] * xx[34];
  xx[44] = xx[38] - xx[42] * xx[40];
  xx[38] = xx[39] * xx[34] + xx[41];
  xx[45] = xx[33];
  xx[46] = - xx[43];
  xx[47] = xx[44];
  xx[48] = - xx[38];
  pm_math_Quaternion_inverseXform_ra(xx + 45, xx + 29, xx + 49);
  xx[34] = xx[51] + state[5];
  xx[51] = xx[49];
  xx[52] = xx[50];
  xx[53] = xx[34];
  xx[54] = xx[25] * xx[49];
  xx[55] = xx[25] * xx[50];
  xx[56] = xx[25] * xx[34];
  pm_math_Vector3_cross_ra(xx + 51, xx + 54, xx + 57);
  xx[34] = xx[6] * state[6];
  xx[40] = cos(xx[34]);
  xx[41] = sin(xx[34]);
  xx[34] = xx[32] * xx[41];
  xx[54] = xx[39] * xx[40] + xx[34];
  xx[55] = xx[32] * xx[40];
  xx[32] = xx[55] - xx[42] * xx[41];
  xx[56] = xx[42] * xx[40] + xx[34];
  xx[34] = xx[55] - xx[39] * xx[41];
  xx[39] = - xx[54];
  xx[40] = xx[32];
  xx[41] = - xx[56];
  xx[42] = xx[34];
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 51, xx + 60);
  xx[55] = xx[62] + state[7];
  xx[63] = xx[60];
  xx[64] = xx[61];
  xx[65] = xx[55];
  xx[66] = xx[25] * xx[60];
  xx[67] = xx[25] * xx[61];
  xx[68] = xx[25] * xx[55];
  pm_math_Vector3_cross_ra(xx + 63, xx + 66, xx + 69);
  xx[66] = xx[6] * state[8];
  xx[67] = cos(xx[66]);
  xx[68] = xx[3] * xx[67];
  xx[72] = xx[16] * xx[67];
  xx[67] = sin(xx[66]);
  xx[66] = xx[16] * xx[67];
  xx[73] = xx[3] * xx[67];
  xx[74] = - xx[68];
  xx[75] = - xx[72];
  xx[76] = xx[66];
  xx[77] = - xx[73];
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 63, xx + 78);
  xx[67] = xx[80] + state[9];
  xx[80] = xx[78];
  xx[81] = xx[79];
  xx[82] = xx[67];
  xx[83] = xx[25] * xx[78];
  xx[84] = xx[25] * xx[79];
  xx[85] = xx[25] * xx[67];
  pm_math_Vector3_cross_ra(xx + 80, xx + 83, xx + 86);
  xx[67] = xx[6] * state[10];
  xx[83] = cos(xx[67]);
  xx[84] = xx[3] * xx[83];
  xx[85] = xx[16] * xx[83];
  xx[83] = sin(xx[67]);
  xx[67] = xx[16] * xx[83];
  xx[89] = xx[3] * xx[83];
  xx[90] = - xx[84];
  xx[91] = xx[85];
  xx[92] = - xx[67];
  xx[93] = - xx[89];
  pm_math_Quaternion_inverseXform_ra(xx + 90, xx + 80, xx + 94);
  xx[83] = xx[96] + state[11];
  xx[97] = xx[94];
  xx[98] = xx[95];
  xx[99] = xx[83];
  xx[100] = xx[25] * xx[94];
  xx[101] = xx[25] * xx[95];
  xx[102] = xx[25] * xx[83];
  pm_math_Vector3_cross_ra(xx + 97, xx + 100, xx + 103);
  xx[100] = xx[6] * state[12];
  xx[101] = cos(xx[100]);
  xx[102] = xx[3] * xx[101];
  xx[106] = xx[16] * xx[101];
  xx[101] = sin(xx[100]);
  xx[100] = xx[16] * xx[101];
  xx[107] = xx[3] * xx[101];
  xx[108] = - xx[102];
  xx[109] = - xx[106];
  xx[110] = xx[100];
  xx[111] = - xx[107];
  pm_math_Quaternion_inverseXform_ra(xx + 108, xx + 97, xx + 112);
  xx[101] = xx[114] + state[13];
  xx[114] = xx[112];
  xx[115] = xx[113];
  xx[116] = xx[101];
  xx[117] = xx[25] * xx[112];
  xx[118] = xx[25] * xx[113];
  xx[119] = xx[25] * xx[101];
  pm_math_Vector3_cross_ra(xx + 114, xx + 117, xx + 120);
  xx[117] = 1.084202172485504e-19;
  xx[118] = xx[16] * xx[112];
  xx[119] = xx[3] * xx[118];
  xx[123] = xx[16] * xx[101];
  xx[124] = xx[16] * xx[123];
  xx[125] = xx[101] - xx[8] * (xx[119] + xx[124]);
  xx[126] = xx[117] * xx[125];
  xx[127] = xx[117] * xx[113];
  xx[128] = xx[126] * xx[113] - xx[127] * xx[125];
  xx[125] = xx[3] * xx[123];
  xx[123] = xx[16] * xx[118];
  xx[118] = xx[112] + xx[8] * (xx[125] - xx[123]);
  xx[129] = xx[127] * xx[118];
  xx[130] = xx[16] * xx[129];
  xx[131] = xx[16] * xx[128];
  xx[132] = 2.387583317164932e-7;
  xx[133] = 0.04;
  xx[134] = 0.06499999999956149;
  xx[135] = - xx[132];
  xx[136] = xx[133];
  xx[137] = xx[134];
  xx[138] = 0.15;
  pm_math_Vector3_cross_ra(xx + 114, xx + 135, xx + 139);
  pm_math_Vector3_cross_ra(xx + 114, xx + 139, xx + 142);
  xx[139] = xx[16] * xx[144];
  xx[140] = xx[16] * xx[142];
  xx[141] = xx[138] * (xx[142] + xx[8] * (xx[3] * xx[139] - xx[16] * xx[140]));
  xx[145] = xx[138] * (xx[144] - xx[8] * (xx[3] * xx[140] + xx[16] * xx[139]));
  xx[139] = xx[16] * xx[145];
  xx[140] = xx[16] * xx[141];
  xx[146] = xx[141] - xx[8] * (xx[3] * xx[139] + xx[16] * xx[140]);
  xx[141] = xx[138] * xx[143];
  xx[142] = xx[145] - xx[8] * (xx[16] * xx[139] - xx[3] * xx[140]);
  xx[143] = xx[146];
  xx[144] = xx[141];
  xx[145] = xx[142];
  pm_math_Vector3_cross_ra(xx + 135, xx + 143, xx + 147);
  xx[135] = xx[101] - xx[8] * (xx[124] - xx[119]);
  xx[119] = xx[117] * xx[135];
  xx[117] = xx[119] * xx[113] - xx[127] * xx[135];
  xx[124] = xx[112] - xx[8] * (xx[125] + xx[123]);
  xx[123] = xx[127] * xx[124];
  xx[125] = xx[16] * xx[123];
  xx[127] = xx[16] * xx[117];
  xx[135] = xx[132];
  xx[136] = - xx[133];
  xx[137] = xx[134];
  pm_math_Vector3_cross_ra(xx + 114, xx + 135, xx + 132);
  pm_math_Vector3_cross_ra(xx + 114, xx + 132, xx + 143);
  xx[114] = xx[16] * xx[145];
  xx[115] = xx[16] * xx[143];
  xx[116] = xx[138] * (xx[143] - xx[8] * (xx[3] * xx[114] + xx[16] * xx[115]));
  xx[132] = xx[138] * (xx[145] - xx[8] * (xx[16] * xx[114] - xx[3] * xx[115]));
  xx[114] = xx[16] * xx[132];
  xx[115] = xx[16] * xx[116];
  xx[133] = xx[116] + xx[8] * (xx[3] * xx[114] - xx[16] * xx[115]);
  xx[116] = xx[138] * xx[144];
  xx[134] = xx[132] - xx[8] * (xx[3] * xx[115] + xx[16] * xx[114]);
  xx[138] = xx[133];
  xx[139] = xx[116];
  xx[140] = xx[134];
  pm_math_Vector3_cross_ra(xx + 135, xx + 138, xx + 143);
  xx[114] = 0.03;
  xx[115] = xx[114] * xx[113];
  xx[132] = xx[115] * xx[101];
  xx[135] = xx[141] + xx[116] + 0.2999999999999999 * xx[132];
  xx[116] = 0.303187499999975;
  xx[136] = state[13] * xx[113];
  xx[137] = 2.865099980597918e-9;
  xx[138] = state[13] * xx[112];
  xx[139] = 0.02849999999986844;
  xx[140] = 0.1;
  xx[141] = xx[140] * xx[100];
  xx[150] = xx[140] * xx[106];
  xx[151] = 0.05;
  xx[152] = - (xx[8] * (xx[141] * xx[102] - xx[150] * xx[107]));
  xx[153] = xx[151] - xx[8] * (xx[150] * xx[102] + xx[141] * xx[107]);
  xx[154] = - (xx[8] * (xx[150] * xx[106] + xx[141] * xx[100]) - xx[140]);
  pm_math_Vector3_cross_ra(xx + 97, xx + 152, xx + 155);
  pm_math_Vector3_cross_ra(xx + 97, xx + 155, xx + 158);
  pm_math_Quaternion_inverseXform_ra(xx + 108, xx + 158, xx + 97);
  xx[140] = xx[114] * xx[112];
  xx[141] = xx[140] * xx[101];
  xx[101] = 0.2999999999999999;
  xx[150] = xx[146] + xx[133] + xx[101] * xx[141];
  xx[133] = 2.865099980597917e-9;
  xx[146] = 0.3027074999999921;
  xx[155] = 0.02849999999986844;
  xx[156] = xx[122] + xx[129] - xx[8] * (xx[16] * xx[130] - xx[3] * xx[131]) +
    xx[149] + xx[123] - xx[8] * (xx[3] * xx[127] + xx[16] * xx[125]) + xx[145];
  xx[122] = 0.3004800000000171;
  xx[123] = state[12] + xx[1];
  if (xx[2] < xx[123])
    xx[123] = xx[2];
  xx[129] = - (xx[123] / xx[4]);
  if (xx[5] < xx[129])
    xx[129] = xx[5];
  xx[145] = xx[9] * state[13];
  xx[149] = xx[129] * xx[129] * (xx[7] - xx[8] * xx[129]) * ((- xx[123] == xx[2]
    ? xx[2] : - xx[145]) - xx[11] * xx[123]);
  if (xx[2] > xx[149])
    xx[149] = xx[2];
  xx[123] = state[12] - xx[1];
  if (xx[2] > xx[123])
    xx[123] = xx[2];
  xx[129] = xx[123] / xx[4];
  if (xx[5] < xx[129])
    xx[129] = xx[5];
  xx[157] = xx[129] * xx[129] * (xx[7] - xx[8] * xx[129]) * (xx[11] * xx[123] +
    (xx[123] == xx[2] ? xx[2] : xx[145]));
  if (xx[2] > xx[157])
    xx[157] = xx[2];
  memcpy(xx + 123, xx + 122, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 123, 1, xx + 129);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_7_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[129] = (input[6] - xx[0] * state[13] + xx[149] - xx[157] - xx[156]) / xx
    [123];
  xx[157] = xx[120] + xx[128] - xx[8] * (xx[3] * xx[130] + xx[16] * xx[131]) +
    xx[147] + xx[117] + xx[8] * (xx[3] * xx[125] - xx[16] * xx[127]) + xx[143] -
    0.01949999999986844 * xx[132] - xx[114] * xx[135] + xx[116] * xx[136] - xx
    [137] * xx[138] - xx[139] * xx[98];
  xx[158] = xx[121] + xx[148] - xx[126] * xx[118] + xx[144] - xx[119] * xx[124]
    + 0.01949999999986844 * xx[141] + xx[114] * xx[150] + xx[133] * xx[136] -
    xx[146] * xx[138] + xx[155] * xx[97];
  xx[159] = xx[156] + xx[122] * xx[129];
  pm_math_Quaternion_xform_ra(xx + 108, xx + 157, xx + 117);
  xx[114] = 0.5999999999999999;
  xx[124] = xx[150] + xx[114] * xx[97] - xx[155] * xx[138];
  xx[125] = xx[135] + xx[114] * xx[98] - xx[139] * xx[136];
  xx[126] = xx[142] + xx[134] - xx[101] * (xx[140] * xx[112] + xx[115] * xx[113])
    + xx[114] * xx[99];
  pm_math_Quaternion_xform_ra(xx + 108, xx + 124, xx + 97);
  pm_math_Vector3_cross_ra(xx + 152, xx + 97, xx + 124);
  xx[101] = state[11] * xx[95];
  xx[112] = xx[102] * xx[102];
  xx[113] = xx[8] * (xx[112] + xx[106] * xx[106]) - xx[5];
  xx[115] = xx[106] * xx[100];
  xx[120] = xx[102] * xx[107];
  xx[121] = xx[8] * (xx[115] + xx[120]);
  xx[127] = xx[107] * xx[106];
  xx[128] = xx[102] * xx[100];
  xx[130] = xx[8] * (xx[127] - xx[128]);
  xx[131] = xx[8] * (xx[120] - xx[115]);
  xx[115] = xx[8] * (xx[112] + xx[100] * xx[100]) - xx[5];
  xx[120] = xx[107] * xx[100];
  xx[100] = xx[102] * xx[106];
  xx[102] = xx[8] * (xx[120] + xx[100]);
  xx[106] = xx[8] * (xx[127] + xx[128]);
  xx[127] = xx[8] * (xx[100] - xx[120]);
  xx[100] = xx[8] * (xx[112] + xx[107] * xx[107]) - xx[5];
  xx[156] = xx[113];
  xx[157] = - xx[121];
  xx[158] = xx[130];
  xx[159] = xx[131];
  xx[160] = xx[115];
  xx[161] = - xx[102];
  xx[162] = xx[106];
  xx[163] = xx[127];
  xx[164] = xx[100];
  xx[107] = xx[122] / xx[123];
  xx[112] = xx[122] - xx[122] * xx[107];
  xx[165] = xx[116] * xx[113] - xx[137] * xx[121];
  xx[166] = xx[116] * xx[131] + xx[137] * xx[115];
  xx[167] = xx[116] * xx[106] + xx[137] * xx[127];
  xx[168] = xx[133] * xx[113] - xx[146] * xx[121];
  xx[169] = xx[133] * xx[131] + xx[146] * xx[115];
  xx[170] = xx[133] * xx[106] + xx[146] * xx[127];
  xx[171] = xx[130] * xx[112];
  xx[172] = - (xx[102] * xx[112]);
  xx[173] = xx[112] * xx[100];
  pm_math_Matrix3x3_compose_ra(xx + 156, xx + 165, xx + 140);
  xx[112] = xx[155] * xx[113];
  xx[116] = xx[139] * xx[121];
  xx[120] = xx[112] * xx[121] - xx[116] * xx[113];
  xx[122] = xx[139] * xx[115];
  xx[123] = xx[155] * xx[131];
  xx[128] = xx[122] * xx[113] + xx[123] * xx[121];
  xx[132] = xx[139] * xx[127];
  xx[133] = xx[155] * xx[106];
  xx[134] = xx[132] * xx[113] + xx[133] * xx[121];
  xx[135] = xx[112] * xx[115] + xx[116] * xx[131];
  xx[136] = xx[123] * xx[115] - xx[122] * xx[131];
  xx[137] = xx[133] * xx[115] - xx[132] * xx[131];
  xx[138] = xx[112] * xx[127] + xx[116] * xx[106];
  xx[112] = xx[123] * xx[127] - xx[122] * xx[106];
  xx[116] = xx[133] * xx[127] - xx[132] * xx[106];
  xx[165] = - xx[120];
  xx[166] = - xx[128];
  xx[167] = - xx[134];
  xx[168] = xx[135];
  xx[169] = xx[136];
  xx[170] = xx[137];
  xx[171] = xx[138];
  xx[172] = xx[112];
  xx[173] = xx[116];
  pm_math_Matrix3x3_postCross_ra(xx + 165, xx + 152, xx + 174);
  xx[165] = xx[114] * xx[113];
  xx[166] = xx[114] * xx[131];
  xx[167] = xx[114] * xx[106];
  xx[168] = - (xx[114] * xx[121]);
  xx[169] = xx[114] * xx[115];
  xx[170] = xx[114] * xx[127];
  xx[171] = xx[114] * xx[130];
  xx[172] = - (xx[114] * xx[102]);
  xx[173] = xx[114] * xx[100];
  pm_math_Matrix3x3_compose_ra(xx + 156, xx + 165, xx + 183);
  pm_math_Matrix3x3_postCross_ra(xx + 183, xx + 152, xx + 155);
  pm_math_Matrix3x3_preCross_ra(xx + 155, xx + 152, xx + 164);
  xx[100] = xx[25] + xx[140] - xx[174] - xx[174] - xx[164];
  xx[102] = state[11] * xx[94];
  xx[94] = xx[141] - xx[175] - xx[177] - xx[165];
  xx[106] = xx[120] + xx[155];
  xx[113] = xx[128] + xx[158];
  xx[114] = xx[134] + xx[161];
  xx[115] = xx[135] - xx[156];
  xx[120] = xx[136] - xx[159];
  xx[121] = xx[137] - xx[162];
  xx[122] = xx[138] - xx[157];
  xx[123] = xx[112] - xx[160];
  xx[112] = xx[116] - xx[163];
  xx[130] = - xx[106];
  xx[131] = - xx[113];
  xx[132] = - xx[114];
  xx[133] = xx[115];
  xx[134] = xx[120];
  xx[135] = xx[121];
  xx[136] = xx[122];
  xx[137] = xx[123];
  xx[138] = xx[112];
  xx[116] = xx[151] * xx[89];
  xx[127] = xx[151] * xx[85];
  xx[152] = xx[8] * (xx[116] * xx[84] + xx[127] * xx[67]);
  xx[153] = - (xx[151] - xx[8] * (xx[116] * xx[89] + xx[127] * xx[85]));
  xx[154] = 0.03260000000000002 - xx[8] * (xx[116] * xx[67] - xx[127] * xx[84]);
  pm_math_Vector3_cross_ra(xx + 80, xx + 152, xx + 155);
  pm_math_Vector3_cross_ra(xx + 80, xx + 155, xx + 158);
  pm_math_Quaternion_inverseXform_ra(xx + 90, xx + 158, xx + 80);
  xx[116] = xx[151] * state[11];
  xx[155] = xx[80];
  xx[156] = xx[116] * (xx[96] + xx[83]) + xx[81];
  xx[157] = xx[82] - xx[116] * (xx[95] + xx[95]);
  pm_math_Matrix3x3_xform_ra(xx + 130, xx + 155, xx + 80);
  xx[83] = xx[142] - xx[176] - xx[180] - xx[166];
  xx[95] = xx[83] - xx[151] * xx[106];
  xx[96] = state[10] + xx[1];
  if (xx[2] < xx[96])
    xx[96] = xx[2];
  xx[116] = - (xx[96] / xx[4]);
  if (xx[5] < xx[116])
    xx[116] = xx[5];
  xx[127] = xx[9] * state[11];
  xx[128] = xx[116] * xx[116] * (xx[7] - xx[8] * xx[116]) * ((- xx[96] == xx[2] ?
    xx[2] : - xx[127]) - xx[11] * xx[96]);
  if (xx[2] > xx[128])
    xx[128] = xx[2];
  xx[96] = state[10] - xx[1];
  if (xx[2] > xx[96])
    xx[96] = xx[2];
  xx[116] = xx[96] / xx[4];
  if (xx[5] < xx[116])
    xx[116] = xx[5];
  xx[130] = xx[116] * xx[116] * (xx[7] - xx[8] * xx[116]) * (xx[11] * xx[96] +
    (xx[96] == xx[2] ? xx[2] : xx[127]));
  if (xx[2] > xx[130])
    xx[130] = xx[2];
  xx[96] = xx[146] - xx[180] - xx[176] - xx[170];
  xx[116] = xx[147] - xx[181] - xx[179] - xx[171];
  xx[127] = xx[105] + xx[119] + xx[126] + xx[101] * xx[96] - xx[102] * xx[116] +
    xx[82];
  xx[131] = xx[25] + xx[183];
  xx[192] = xx[131];
  xx[193] = xx[184];
  xx[194] = xx[185];
  xx[195] = xx[186];
  xx[196] = xx[25] + xx[187];
  xx[197] = xx[188];
  xx[198] = xx[189];
  xx[199] = xx[190];
  xx[200] = xx[25] + xx[191];
  pm_math_Matrix3x3_xform_ra(xx + 192, xx + 155, xx + 132);
  xx[135] = xx[97] + xx[132] - (xx[101] * xx[106] + xx[102] * xx[115]);
  xx[97] = xx[25] + xx[148] - xx[182] - xx[182] - xx[172];
  xx[136] = xx[97] + xx[151] * xx[122];
  xx[137] = xx[122] + xx[151] * xx[131];
  xx[138] = xx[136] + xx[151] * xx[137];
  ii[0] = factorSymmetricPosDef(xx + 138, 1, xx + 139);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_6_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[139] = (input[5] - xx[0] * state[11] + xx[128] - xx[130] - (xx[127] + xx
              [151] * xx[135])) / xx[138];
  xx[105] = xx[143] - xx[177] - xx[175] - xx[167];
  xx[119] = xx[25] + xx[144] - xx[178] - xx[178] - xx[168];
  xx[82] = xx[145] - xx[179] - xx[181] - xx[169];
  xx[126] = xx[82] + xx[151] * xx[115];
  xx[140] = xx[103] + xx[117] + xx[124] + xx[101] * xx[100] - xx[102] * xx[94] +
    xx[80] + xx[95] * xx[139];
  xx[141] = xx[104] + xx[118] + xx[125] + xx[101] * xx[105] - xx[102] * xx[119]
    + xx[81] + xx[126] * xx[139];
  xx[142] = xx[127] + xx[136] * xx[139];
  pm_math_Quaternion_xform_ra(xx + 90, xx + 140, xx + 143);
  xx[80] = xx[123] + xx[151] * xx[186];
  xx[81] = xx[112] + xx[151] * xx[189];
  xx[140] = xx[135] + xx[137] * xx[139];
  xx[141] = xx[98] + xx[133] - (xx[101] * xx[113] + xx[102] * xx[120]) + xx[80] *
    xx[139];
  xx[142] = xx[99] + xx[134] - (xx[101] * xx[114] + xx[102] * xx[121]) + xx[81] *
    xx[139];
  pm_math_Quaternion_xform_ra(xx + 90, xx + 140, xx + 132);
  pm_math_Vector3_cross_ra(xx + 152, xx + 132, xx + 140);
  xx[98] = state[9] * xx[79];
  xx[79] = xx[84] * xx[84];
  xx[99] = xx[85] * xx[67];
  xx[103] = xx[84] * xx[89];
  xx[104] = xx[84] * xx[67];
  xx[117] = xx[89] * xx[85];
  xx[118] = xx[89] * xx[67];
  xx[124] = xx[84] * xx[85];
  xx[155] = xx[8] * (xx[79] + xx[85] * xx[85]) - xx[5];
  xx[156] = - (xx[8] * (xx[99] + xx[103]));
  xx[157] = xx[8] * (xx[104] - xx[117]);
  xx[158] = xx[8] * (xx[103] - xx[99]);
  xx[159] = xx[8] * (xx[79] + xx[67] * xx[67]) - xx[5];
  xx[160] = xx[8] * (xx[118] + xx[124]);
  xx[161] = - (xx[8] * (xx[117] + xx[104]));
  xx[162] = xx[8] * (xx[118] - xx[124]);
  xx[163] = xx[8] * (xx[79] + xx[89] * xx[89]) - xx[5];
  xx[67] = xx[95] / xx[138];
  xx[79] = xx[126] * xx[67];
  xx[84] = xx[136] * xx[67];
  xx[85] = xx[126] / xx[138];
  xx[89] = xx[136] * xx[85];
  xx[99] = xx[136] / xx[138];
  xx[164] = xx[100] - xx[95] * xx[67];
  xx[165] = xx[94] - xx[79];
  xx[166] = xx[83] - xx[84];
  xx[167] = xx[105] - xx[79];
  xx[168] = xx[119] - xx[126] * xx[85];
  xx[169] = xx[82] - xx[89];
  xx[170] = xx[96] - xx[84];
  xx[171] = xx[116] - xx[89];
  xx[172] = xx[97] - xx[136] * xx[99];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 164, xx + 155, xx + 173);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 173, xx + 164);
  xx[79] = xx[137] / xx[138];
  xx[82] = xx[80] / xx[138];
  xx[83] = xx[81] / xx[138];
  xx[173] = - (xx[106] + xx[95] * xx[79]);
  xx[174] = - (xx[113] + xx[95] * xx[82]);
  xx[175] = - (xx[114] + xx[95] * xx[83]);
  xx[176] = xx[115] - xx[126] * xx[79];
  xx[177] = xx[120] - xx[126] * xx[82];
  xx[178] = xx[121] - xx[126] * xx[83];
  xx[179] = xx[122] - xx[136] * xx[79];
  xx[180] = xx[123] - xx[136] * xx[82];
  xx[181] = xx[112] - xx[136] * xx[83];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 173, xx + 155, xx + 112);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 112, xx + 173);
  pm_math_Matrix3x3_postCross_ra(xx + 173, xx + 152, xx + 112);
  xx[84] = xx[80] * xx[79];
  xx[89] = xx[81] * xx[79];
  xx[94] = xx[81] * xx[82];
  xx[192] = xx[131] - xx[137] * xx[79];
  xx[193] = xx[184] - xx[84];
  xx[194] = xx[185] - xx[89];
  xx[195] = xx[186] - xx[84];
  xx[196] = xx[187] - xx[80] * xx[82] + xx[25];
  xx[197] = xx[188] - xx[94];
  xx[198] = xx[189] - xx[89];
  xx[199] = xx[190] - xx[94];
  xx[200] = xx[191] - xx[81] * xx[83] + xx[25];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 192, xx + 155, xx + 182);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 182, xx + 191);
  pm_math_Matrix3x3_postCross_ra(xx + 191, xx + 152, xx + 155);
  pm_math_Matrix3x3_preCross_ra(xx + 155, xx + 152, xx + 182);
  xx[80] = xx[25] + xx[164] - xx[112] - xx[112] - xx[182];
  xx[81] = state[9] * xx[78];
  xx[78] = xx[165] - xx[113] - xx[115] - xx[183];
  xx[84] = xx[173] - xx[155];
  xx[89] = xx[174] - xx[158];
  xx[94] = xx[175] - xx[161];
  xx[95] = xx[176] - xx[156];
  xx[96] = xx[177] - xx[159];
  xx[97] = xx[178] - xx[162];
  xx[100] = xx[179] - xx[157];
  xx[103] = xx[180] - xx[160];
  xx[104] = xx[181] - xx[163];
  xx[155] = xx[84];
  xx[156] = xx[89];
  xx[157] = xx[94];
  xx[158] = xx[95];
  xx[159] = xx[96];
  xx[160] = xx[97];
  xx[161] = xx[100];
  xx[162] = xx[103];
  xx[163] = xx[104];
  xx[105] = 0.18;
  xx[106] = xx[105] * xx[66];
  xx[121] = xx[105] * xx[72];
  xx[122] = 0.09;
  xx[123] = - (xx[8] * (xx[106] * xx[68] - xx[121] * xx[73]));
  xx[124] = xx[122] - xx[8] * (xx[121] * xx[68] + xx[106] * xx[73]);
  xx[125] = - (xx[8] * (xx[121] * xx[72] + xx[106] * xx[66]) - xx[105]);
  pm_math_Vector3_cross_ra(xx + 63, xx + 123, xx + 126);
  pm_math_Vector3_cross_ra(xx + 63, xx + 126, xx + 135);
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 135, xx + 63);
  pm_math_Matrix3x3_xform_ra(xx + 155, xx + 63, xx + 126);
  xx[105] = xx[166] - xx[114] - xx[118] - xx[184];
  xx[106] = state[8] + xx[1];
  if (xx[2] < xx[106])
    xx[106] = xx[2];
  xx[121] = - (xx[106] / xx[4]);
  if (xx[5] < xx[121])
    xx[121] = xx[5];
  xx[130] = xx[9] * state[9];
  xx[131] = xx[121] * xx[121] * (xx[7] - xx[8] * xx[121]) * ((- xx[106] == xx[2]
    ? xx[2] : - xx[130]) - xx[11] * xx[106]);
  if (xx[2] > xx[131])
    xx[131] = xx[2];
  xx[106] = state[8] - xx[1];
  if (xx[2] > xx[106])
    xx[106] = xx[2];
  xx[121] = xx[106] / xx[4];
  if (xx[5] < xx[121])
    xx[121] = xx[5];
  xx[135] = xx[121] * xx[121] * (xx[7] - xx[8] * xx[121]) * (xx[11] * xx[106] +
    (xx[106] == xx[2] ? xx[2] : xx[130]));
  if (xx[2] > xx[135])
    xx[135] = xx[2];
  xx[106] = xx[170] - xx[118] - xx[114] - xx[188];
  xx[121] = xx[171] - xx[119] - xx[117] - xx[189];
  xx[130] = xx[88] + xx[145] + xx[142] + xx[98] * xx[106] - xx[81] * xx[121] +
    xx[128];
  xx[136] = xx[25] + xx[172] - xx[120] - xx[120] - xx[190];
  memcpy(xx + 137, xx + 136, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 137, 1, xx + 138);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_5_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[138] = (input[4] - xx[0] * state[9] + xx[131] - xx[135] - xx[130]) / xx[137];
  xx[88] = xx[167] - xx[115] - xx[113] - xx[185];
  xx[112] = xx[25] + xx[168] - xx[116] - xx[116] - xx[186];
  xx[113] = xx[169] - xx[117] - xx[119] - xx[187];
  xx[114] = xx[86] + xx[143] + xx[140] + xx[98] * xx[80] - xx[81] * xx[78] + xx
    [126] + xx[105] * xx[138];
  xx[115] = xx[87] + xx[144] + xx[141] + xx[98] * xx[88] - xx[81] * xx[112] +
    xx[127] + xx[113] * xx[138];
  xx[116] = xx[130] + xx[136] * xx[138];
  pm_math_Quaternion_xform_ra(xx + 74, xx + 114, xx + 117);
  xx[140] = xx[25] + xx[191];
  xx[141] = xx[192];
  xx[142] = xx[193];
  xx[143] = xx[194];
  xx[144] = xx[25] + xx[195];
  xx[145] = xx[196];
  xx[146] = xx[197];
  xx[147] = xx[198];
  xx[148] = xx[25] + xx[199];
  pm_math_Matrix3x3_xform_ra(xx + 140, xx + 63, xx + 114);
  xx[126] = xx[132] + xx[98] * xx[84] - xx[81] * xx[95] + xx[114] + xx[100] *
    xx[138];
  xx[127] = xx[133] + xx[98] * xx[89] - xx[81] * xx[96] + xx[115] + xx[103] *
    xx[138];
  xx[128] = xx[134] + xx[98] * xx[94] - xx[81] * xx[97] + xx[116] + xx[104] *
    xx[138];
  pm_math_Quaternion_xform_ra(xx + 74, xx + 126, xx + 114);
  pm_math_Vector3_cross_ra(xx + 123, xx + 114, xx + 126);
  xx[86] = state[7] * xx[61];
  xx[87] = xx[68] * xx[68];
  xx[120] = xx[72] * xx[66];
  xx[130] = xx[68] * xx[73];
  xx[131] = xx[73] * xx[72];
  xx[132] = xx[68] * xx[66];
  xx[133] = xx[73] * xx[66];
  xx[134] = xx[68] * xx[72];
  xx[140] = xx[8] * (xx[87] + xx[72] * xx[72]) - xx[5];
  xx[141] = - (xx[8] * (xx[120] + xx[130]));
  xx[142] = xx[8] * (xx[131] - xx[132]);
  xx[143] = xx[8] * (xx[130] - xx[120]);
  xx[144] = xx[8] * (xx[87] + xx[66] * xx[66]) - xx[5];
  xx[145] = - (xx[8] * (xx[133] + xx[134]));
  xx[146] = xx[8] * (xx[131] + xx[132]);
  xx[147] = xx[8] * (xx[134] - xx[133]);
  xx[148] = xx[8] * (xx[87] + xx[73] * xx[73]) - xx[5];
  xx[66] = xx[105] / xx[137];
  xx[68] = xx[113] * xx[66];
  xx[72] = xx[136] * xx[66];
  xx[73] = xx[113] / xx[137];
  xx[87] = xx[136] * xx[73];
  xx[120] = xx[136] / xx[137];
  xx[155] = xx[80] - xx[105] * xx[66];
  xx[156] = xx[78] - xx[68];
  xx[157] = xx[105] - xx[72];
  xx[158] = xx[88] - xx[68];
  xx[159] = xx[112] - xx[113] * xx[73];
  xx[160] = xx[113] - xx[87];
  xx[161] = xx[106] - xx[72];
  xx[162] = xx[121] - xx[87];
  xx[163] = xx[136] - xx[136] * xx[120];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 155, xx + 140, xx + 164);
  pm_math_Matrix3x3_compose_ra(xx + 140, xx + 164, xx + 155);
  xx[68] = xx[100] / xx[137];
  xx[72] = xx[103] / xx[137];
  xx[78] = xx[104] / xx[137];
  xx[164] = xx[84] - xx[105] * xx[68];
  xx[165] = xx[89] - xx[105] * xx[72];
  xx[166] = xx[94] - xx[105] * xx[78];
  xx[167] = xx[95] - xx[113] * xx[68];
  xx[168] = xx[96] - xx[113] * xx[72];
  xx[169] = xx[97] - xx[113] * xx[78];
  xx[170] = xx[100] - xx[136] * xx[68];
  xx[171] = xx[103] - xx[136] * xx[72];
  xx[172] = xx[104] - xx[136] * xx[78];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 164, xx + 140, xx + 173);
  pm_math_Matrix3x3_compose_ra(xx + 140, xx + 173, xx + 164);
  pm_math_Matrix3x3_postCross_ra(xx + 164, xx + 123, xx + 173);
  xx[80] = xx[103] * xx[68];
  xx[84] = xx[104] * xx[68];
  xx[87] = xx[104] * xx[72];
  xx[182] = xx[191] - xx[100] * xx[68] + xx[25];
  xx[183] = xx[192] - xx[80];
  xx[184] = xx[193] - xx[84];
  xx[185] = xx[194] - xx[80];
  xx[186] = xx[195] - xx[103] * xx[72] + xx[25];
  xx[187] = xx[196] - xx[87];
  xx[188] = xx[197] - xx[84];
  xx[189] = xx[198] - xx[87];
  xx[190] = xx[199] - xx[104] * xx[78] + xx[25];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 182, xx + 140, xx + 191);
  pm_math_Matrix3x3_compose_ra(xx + 140, xx + 191, xx + 182);
  pm_math_Matrix3x3_postCross_ra(xx + 182, xx + 123, xx + 140);
  pm_math_Matrix3x3_preCross_ra(xx + 140, xx + 123, xx + 191);
  xx[80] = xx[25] + xx[155] - xx[173] - xx[173] - xx[191];
  xx[84] = state[7] * xx[60];
  xx[60] = xx[156] - xx[174] - xx[176] - xx[192];
  xx[87] = xx[164] - xx[140];
  xx[88] = xx[165] - xx[143];
  xx[89] = xx[166] - xx[146];
  xx[94] = xx[167] - xx[141];
  xx[95] = xx[168] - xx[144];
  xx[96] = xx[169] - xx[147];
  xx[97] = xx[170] - xx[142];
  xx[100] = xx[171] - xx[145];
  xx[103] = xx[172] - xx[148];
  xx[140] = xx[87];
  xx[141] = xx[88];
  xx[142] = xx[89];
  xx[143] = xx[94];
  xx[144] = xx[95];
  xx[145] = xx[96];
  xx[146] = xx[97];
  xx[147] = xx[100];
  xx[148] = xx[103];
  xx[104] = xx[122] * xx[34];
  xx[105] = xx[122] * xx[32];
  xx[130] = - (xx[8] * (xx[104] * xx[54] - xx[105] * xx[56]));
  xx[131] = - (xx[122] - xx[8] * (xx[104] * xx[34] + xx[105] * xx[32]));
  xx[132] = 0.06 + xx[8] * (xx[105] * xx[54] + xx[104] * xx[56]);
  pm_math_Vector3_cross_ra(xx + 51, xx + 130, xx + 104);
  pm_math_Vector3_cross_ra(xx + 51, xx + 104, xx + 133);
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 133, xx + 51);
  xx[104] = xx[122] * state[7];
  xx[105] = xx[104] * (xx[62] + xx[55]) + xx[52];
  xx[52] = xx[53] - xx[104] * (xx[61] + xx[61]);
  xx[133] = xx[51];
  xx[134] = xx[105];
  xx[135] = xx[52];
  pm_math_Matrix3x3_xform_ra(xx + 140, xx + 133, xx + 149);
  xx[53] = xx[157] - xx[175] - xx[179] - xx[193];
  xx[55] = xx[53] + xx[122] * xx[87];
  xx[61] = state[6] + xx[1];
  if (xx[2] < xx[61])
    xx[61] = xx[2];
  xx[62] = - (xx[61] / xx[4]);
  if (xx[5] < xx[62])
    xx[62] = xx[5];
  xx[104] = xx[9] * state[7];
  xx[106] = xx[62] * xx[62] * (xx[7] - xx[8] * xx[62]) * ((- xx[61] == xx[2] ?
    xx[2] : - xx[104]) - xx[11] * xx[61]);
  if (xx[2] > xx[106])
    xx[106] = xx[2];
  xx[61] = state[6] - xx[1];
  if (xx[2] > xx[61])
    xx[61] = xx[2];
  xx[62] = xx[61] / xx[4];
  if (xx[5] < xx[62])
    xx[62] = xx[5];
  xx[112] = xx[62] * xx[62] * (xx[7] - xx[8] * xx[62]) * (xx[11] * xx[61] + (xx
    [61] == xx[2] ? xx[2] : xx[104]));
  if (xx[2] > xx[112])
    xx[112] = xx[2];
  xx[61] = xx[161] - xx[179] - xx[175] - xx[197];
  xx[62] = xx[162] - xx[180] - xx[178] - xx[198];
  xx[104] = xx[71] + xx[119] + xx[128] + xx[86] * xx[61] - xx[84] * xx[62] + xx
    [151];
  xx[113] = xx[25] + xx[182];
  xx[140] = xx[113];
  xx[141] = xx[183];
  xx[142] = xx[184];
  xx[143] = xx[185];
  xx[144] = xx[25] + xx[186];
  xx[145] = xx[187];
  xx[146] = xx[188];
  xx[147] = xx[189];
  xx[148] = xx[25] + xx[190];
  pm_math_Matrix3x3_xform_ra(xx + 140, xx + 133, xx + 164);
  xx[121] = xx[114] + xx[86] * xx[87] - xx[84] * xx[94] + xx[164];
  xx[114] = xx[25] + xx[163] - xx[181] - xx[181] - xx[199];
  xx[133] = xx[114] + xx[122] * xx[97];
  xx[134] = xx[97] + xx[122] * xx[113];
  xx[135] = xx[133] + xx[122] * xx[134];
  ii[0] = factorSymmetricPosDef(xx + 135, 1, xx + 136);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_4_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[136] = (input[3] - xx[0] * state[7] + xx[106] - xx[112] - (xx[104] + xx[122]
              * xx[121])) / xx[135];
  xx[71] = xx[158] - xx[176] - xx[174] - xx[194];
  xx[106] = xx[25] + xx[159] - xx[177] - xx[177] - xx[195];
  xx[112] = xx[160] - xx[178] - xx[180] - xx[196];
  xx[119] = xx[112] + xx[122] * xx[94];
  xx[140] = xx[69] + xx[117] + xx[126] + xx[86] * xx[80] - xx[84] * xx[60] + xx
    [149] + xx[55] * xx[136];
  xx[141] = xx[70] + xx[118] + xx[127] + xx[86] * xx[71] - xx[84] * xx[106] +
    xx[150] + xx[119] * xx[136];
  xx[142] = xx[104] + xx[133] * xx[136];
  pm_math_Quaternion_xform_ra(xx + 39, xx + 140, xx + 126);
  xx[69] = xx[100] + xx[122] * xx[185];
  xx[70] = xx[103] + xx[122] * xx[188];
  xx[140] = xx[121] + xx[134] * xx[136];
  xx[141] = xx[115] + xx[86] * xx[88] - xx[84] * xx[95] + xx[165] + xx[69] * xx
    [136];
  xx[142] = xx[116] + xx[86] * xx[89] - xx[84] * xx[96] + xx[166] + xx[70] * xx
    [136];
  pm_math_Quaternion_xform_ra(xx + 39, xx + 140, xx + 115);
  pm_math_Vector3_cross_ra(xx + 130, xx + 115, xx + 140);
  xx[104] = state[5] * xx[50];
  xx[50] = xx[54] * xx[54];
  xx[118] = xx[54] * xx[34];
  xx[121] = xx[56] * xx[32];
  xx[137] = xx[32] * xx[34];
  xx[143] = xx[54] * xx[56];
  xx[144] = xx[54] * xx[32];
  xx[54] = xx[56] * xx[34];
  xx[155] = xx[8] * (xx[50] + xx[32] * xx[32]) - xx[5];
  xx[156] = xx[8] * (xx[118] - xx[121]);
  xx[157] = xx[8] * (xx[137] + xx[143]);
  xx[158] = - (xx[8] * (xx[121] + xx[118]));
  xx[159] = xx[8] * (xx[50] + xx[56] * xx[56]) - xx[5];
  xx[160] = xx[8] * (xx[144] - xx[54]);
  xx[161] = xx[8] * (xx[137] - xx[143]);
  xx[162] = - (xx[8] * (xx[54] + xx[144]));
  xx[163] = xx[8] * (xx[50] + xx[34] * xx[34]) - xx[5];
  xx[32] = xx[55] / xx[135];
  xx[34] = xx[119] * xx[32];
  xx[50] = xx[133] * xx[32];
  xx[54] = xx[119] / xx[135];
  xx[56] = xx[133] * xx[54];
  xx[118] = xx[133] / xx[135];
  xx[143] = xx[80] - xx[55] * xx[32];
  xx[144] = xx[60] - xx[34];
  xx[145] = xx[53] - xx[50];
  xx[146] = xx[71] - xx[34];
  xx[147] = xx[106] - xx[119] * xx[54];
  xx[148] = xx[112] - xx[56];
  xx[149] = xx[61] - xx[50];
  xx[150] = xx[62] - xx[56];
  xx[151] = xx[114] - xx[133] * xx[118];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 143, xx + 155, xx + 164);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 164, xx + 143);
  xx[34] = xx[134] / xx[135];
  xx[50] = xx[69] / xx[135];
  xx[53] = xx[70] / xx[135];
  xx[164] = xx[87] - xx[55] * xx[34];
  xx[165] = xx[88] - xx[55] * xx[50];
  xx[166] = xx[89] - xx[55] * xx[53];
  xx[167] = xx[94] - xx[119] * xx[34];
  xx[168] = xx[95] - xx[119] * xx[50];
  xx[169] = xx[96] - xx[119] * xx[53];
  xx[170] = xx[97] - xx[133] * xx[34];
  xx[171] = xx[100] - xx[133] * xx[50];
  xx[172] = xx[103] - xx[133] * xx[53];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 164, xx + 155, xx + 173);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 173, xx + 164);
  pm_math_Matrix3x3_postCross_ra(xx + 164, xx + 130, xx + 173);
  xx[55] = xx[69] * xx[34];
  xx[56] = xx[70] * xx[34];
  xx[60] = xx[70] * xx[50];
  xx[191] = xx[113] - xx[134] * xx[34];
  xx[192] = xx[183] - xx[55];
  xx[193] = xx[184] - xx[56];
  xx[194] = xx[185] - xx[55];
  xx[195] = xx[186] - xx[69] * xx[50] + xx[25];
  xx[196] = xx[187] - xx[60];
  xx[197] = xx[188] - xx[56];
  xx[198] = xx[189] - xx[60];
  xx[199] = xx[190] - xx[70] * xx[53] + xx[25];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 191, xx + 155, xx + 182);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 182, xx + 191);
  pm_math_Matrix3x3_postCross_ra(xx + 191, xx + 130, xx + 155);
  pm_math_Matrix3x3_preCross_ra(xx + 155, xx + 130, xx + 182);
  xx[55] = xx[25] + xx[143] - xx[173] - xx[173] - xx[182];
  xx[56] = state[5] * xx[49];
  xx[49] = xx[144] - xx[174] - xx[176] - xx[183];
  xx[60] = xx[164] - xx[155];
  xx[61] = xx[165] - xx[158];
  xx[62] = xx[166] - xx[161];
  xx[69] = xx[167] - xx[156];
  xx[70] = xx[168] - xx[159];
  xx[71] = xx[169] - xx[162];
  xx[80] = xx[170] - xx[157];
  xx[87] = xx[171] - xx[160];
  xx[88] = xx[172] - xx[163];
  xx[155] = xx[60];
  xx[156] = xx[61];
  xx[157] = xx[62];
  xx[158] = xx[69];
  xx[159] = xx[70];
  xx[160] = xx[71];
  xx[161] = xx[80];
  xx[162] = xx[87];
  xx[163] = xx[88];
  xx[89] = 0.21;
  xx[94] = xx[89] * xx[44];
  xx[95] = xx[89] * xx[43];
  xx[96] = 0.105;
  xx[112] = xx[8] * (xx[94] * xx[33] + xx[95] * xx[38]) - xx[96];
  xx[113] = - (xx[8] * (xx[94] * xx[38] - xx[95] * xx[33]));
  xx[114] = - (xx[8] * (xx[95] * xx[43] + xx[94] * xx[44]) - xx[89]);
  pm_math_Vector3_cross_ra(xx + 29, xx + 112, xx + 133);
  pm_math_Vector3_cross_ra(xx + 29, xx + 133, xx + 164);
  pm_math_Quaternion_inverseXform_ra(xx + 45, xx + 164, xx + 29);
  pm_math_Matrix3x3_xform_ra(xx + 155, xx + 29, xx + 133);
  xx[89] = xx[145] - xx[175] - xx[179] - xx[184];
  xx[94] = state[4] + xx[1];
  if (xx[2] < xx[94])
    xx[94] = xx[2];
  xx[95] = - (xx[94] / xx[4]);
  if (xx[5] < xx[95])
    xx[95] = xx[5];
  xx[97] = xx[9] * state[5];
  xx[100] = xx[95] * xx[95] * (xx[7] - xx[8] * xx[95]) * ((- xx[94] == xx[2] ?
    xx[2] : - xx[97]) - xx[11] * xx[94]);
  if (xx[2] > xx[100])
    xx[100] = xx[2];
  xx[94] = state[4] - xx[1];
  if (xx[2] > xx[94])
    xx[94] = xx[2];
  xx[95] = xx[94] / xx[4];
  if (xx[5] < xx[95])
    xx[95] = xx[5];
  xx[103] = xx[95] * xx[95] * (xx[7] - xx[8] * xx[95]) * (xx[11] * xx[94] + (xx
    [94] == xx[2] ? xx[2] : xx[97]));
  if (xx[2] > xx[103])
    xx[103] = xx[2];
  xx[94] = xx[149] - xx[179] - xx[175] - xx[188];
  xx[95] = xx[150] - xx[180] - xx[178] - xx[189];
  xx[97] = xx[59] + xx[128] + xx[142] + xx[104] * xx[94] - xx[56] * xx[95] + xx
    [135];
  xx[106] = xx[25] + xx[151] - xx[181] - xx[181] - xx[190];
  memcpy(xx + 119, xx + 106, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 119, 1, xx + 121);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_3_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[121] = (input[2] - xx[0] * state[5] + xx[100] - xx[103] - xx[97]) / xx[119];
  xx[59] = xx[146] - xx[176] - xx[174] - xx[185];
  xx[100] = xx[25] + xx[147] - xx[177] - xx[177] - xx[186];
  xx[103] = xx[148] - xx[178] - xx[180] - xx[187];
  xx[142] = xx[57] + xx[126] + xx[140] + xx[104] * xx[55] - xx[56] * xx[49] +
    xx[133] + xx[89] * xx[121];
  xx[143] = xx[58] + xx[127] + xx[141] + xx[104] * xx[59] - xx[56] * xx[100] +
    xx[134] + xx[103] * xx[121];
  xx[144] = xx[97] + xx[106] * xx[121];
  pm_math_Quaternion_xform_ra(xx + 45, xx + 142, xx + 126);
  xx[140] = xx[25] + xx[191];
  xx[141] = xx[192];
  xx[142] = xx[193];
  xx[143] = xx[194];
  xx[144] = xx[25] + xx[195];
  xx[145] = xx[196];
  xx[146] = xx[197];
  xx[147] = xx[198];
  xx[148] = xx[25] + xx[199];
  pm_math_Matrix3x3_xform_ra(xx + 140, xx + 29, xx + 133);
  xx[140] = xx[115] + xx[104] * xx[60] - xx[56] * xx[69] + xx[133] + xx[80] *
    xx[121];
  xx[141] = xx[116] + xx[104] * xx[61] - xx[56] * xx[70] + xx[134] + xx[87] *
    xx[121];
  xx[142] = xx[117] + xx[104] * xx[62] - xx[56] * xx[71] + xx[135] + xx[88] *
    xx[121];
  pm_math_Quaternion_xform_ra(xx + 45, xx + 140, xx + 115);
  pm_math_Vector3_cross_ra(xx + 112, xx + 115, xx + 133);
  xx[57] = xx[27] * state[3];
  xx[27] = xx[33] * xx[33];
  xx[58] = xx[33] * xx[38];
  xx[97] = xx[44] * xx[43];
  xx[137] = xx[38] * xx[43];
  xx[140] = xx[33] * xx[44];
  xx[141] = xx[33] * xx[43];
  xx[33] = xx[44] * xx[38];
  xx[142] = xx[8] * (xx[27] + xx[43] * xx[43]) - xx[5];
  xx[143] = xx[8] * (xx[58] - xx[97]);
  xx[144] = xx[8] * (xx[137] + xx[140]);
  xx[145] = - (xx[8] * (xx[97] + xx[58]));
  xx[146] = xx[8] * (xx[27] + xx[44] * xx[44]) - xx[5];
  xx[147] = xx[8] * (xx[141] - xx[33]);
  xx[148] = xx[8] * (xx[137] - xx[140]);
  xx[149] = - (xx[8] * (xx[33] + xx[141]));
  xx[150] = xx[8] * (xx[27] + xx[38] * xx[38]) - xx[5];
  xx[27] = xx[89] / xx[119];
  xx[33] = xx[103] * xx[27];
  xx[38] = xx[106] * xx[27];
  xx[43] = xx[103] / xx[119];
  xx[44] = xx[106] * xx[43];
  xx[58] = xx[106] / xx[119];
  xx[155] = xx[55] - xx[89] * xx[27];
  xx[156] = xx[49] - xx[33];
  xx[157] = xx[89] - xx[38];
  xx[158] = xx[59] - xx[33];
  xx[159] = xx[100] - xx[103] * xx[43];
  xx[160] = xx[103] - xx[44];
  xx[161] = xx[94] - xx[38];
  xx[162] = xx[95] - xx[44];
  xx[163] = xx[106] - xx[106] * xx[58];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 155, xx + 142, xx + 164);
  pm_math_Matrix3x3_compose_ra(xx + 142, xx + 164, xx + 155);
  xx[33] = xx[80] / xx[119];
  xx[38] = xx[87] / xx[119];
  xx[44] = xx[88] / xx[119];
  xx[164] = xx[60] - xx[89] * xx[33];
  xx[165] = xx[61] - xx[89] * xx[38];
  xx[166] = xx[62] - xx[89] * xx[44];
  xx[167] = xx[69] - xx[103] * xx[33];
  xx[168] = xx[70] - xx[103] * xx[38];
  xx[169] = xx[71] - xx[103] * xx[44];
  xx[170] = xx[80] - xx[106] * xx[33];
  xx[171] = xx[87] - xx[106] * xx[38];
  xx[172] = xx[88] - xx[106] * xx[44];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 164, xx + 142, xx + 173);
  pm_math_Matrix3x3_compose_ra(xx + 142, xx + 173, xx + 164);
  pm_math_Matrix3x3_postCross_ra(xx + 164, xx + 112, xx + 173);
  xx[49] = xx[87] * xx[33];
  xx[55] = xx[88] * xx[33];
  xx[59] = xx[88] * xx[38];
  xx[182] = xx[191] - xx[80] * xx[33] + xx[25];
  xx[183] = xx[192] - xx[49];
  xx[184] = xx[193] - xx[55];
  xx[185] = xx[194] - xx[49];
  xx[186] = xx[195] - xx[87] * xx[38] + xx[25];
  xx[187] = xx[196] - xx[59];
  xx[188] = xx[197] - xx[55];
  xx[189] = xx[198] - xx[59];
  xx[190] = xx[199] - xx[88] * xx[44] + xx[25];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 182, xx + 142, xx + 191);
  pm_math_Matrix3x3_compose_ra(xx + 142, xx + 191, xx + 182);
  pm_math_Matrix3x3_postCross_ra(xx + 182, xx + 112, xx + 140);
  pm_math_Matrix3x3_preCross_ra(xx + 140, xx + 112, xx + 191);
  xx[49] = xx[25] + xx[155] - xx[173] - xx[173] - xx[191];
  xx[55] = xx[26] * state[3];
  xx[59] = xx[156] - xx[174] - xx[176] - xx[192];
  xx[60] = xx[164] - xx[140];
  xx[61] = xx[165] - xx[143];
  xx[62] = xx[166] - xx[146];
  xx[69] = xx[167] - xx[141];
  xx[70] = xx[168] - xx[144];
  xx[71] = xx[169] - xx[147];
  xx[80] = xx[170] - xx[142];
  xx[87] = xx[171] - xx[145];
  xx[88] = xx[172] - xx[148];
  xx[140] = xx[60];
  xx[141] = xx[61];
  xx[142] = xx[62];
  xx[143] = xx[69];
  xx[144] = xx[70];
  xx[145] = xx[71];
  xx[146] = xx[80];
  xx[147] = xx[87];
  xx[148] = xx[88];
  xx[89] = xx[96] * xx[10];
  xx[94] = xx[96] * xx[19];
  xx[95] = xx[8] * (xx[89] * xx[17] - xx[94] * xx[15]);
  xx[97] = xx[95] * state[1] * state[1];
  xx[100] = xx[19] * xx[97];
  xx[149] = xx[14];
  xx[150] = xx[10];
  xx[151] = xx[18];
  xx[14] = 0.02 - (xx[8] * (xx[89] * xx[10] + xx[94] * xx[19]) - xx[96]);
  xx[18] = xx[14] * state[1] * state[1];
  xx[103] = xx[19] * xx[18];
  xx[106] = xx[10] * xx[18] - xx[17] * xx[97];
  xx[164] = xx[100];
  xx[165] = xx[103];
  xx[166] = xx[106];
  pm_math_Vector3_cross_ra(xx + 149, xx + 164, xx + 167);
  xx[119] = xx[96] * state[3];
  xx[137] = xx[8] * (xx[15] * xx[100] + xx[167]) - xx[18] - xx[119] * (xx[28] +
    xx[24]);
  xx[18] = xx[97] + xx[8] * (xx[15] * xx[103] + xx[168]);
  xx[24] = xx[119] * (xx[26] + xx[26]) + xx[8] * (xx[169] + xx[15] * xx[106]);
  xx[149] = xx[137];
  xx[150] = xx[18];
  xx[151] = xx[24];
  pm_math_Matrix3x3_xform_ra(xx + 140, xx + 149, xx + 164);
  xx[26] = xx[157] - xx[175] - xx[179] - xx[193];
  xx[28] = xx[26] + xx[96] * xx[61];
  xx[97] = state[2] + xx[1];
  if (xx[2] < xx[97])
    xx[97] = xx[2];
  xx[100] = - (xx[97] / xx[4]);
  if (xx[5] < xx[100])
    xx[100] = xx[5];
  xx[103] = xx[9] * state[3];
  xx[9] = xx[100] * xx[100] * (xx[7] - xx[8] * xx[100]) * ((- xx[97] == xx[2] ?
    xx[2] : - xx[103]) - xx[11] * xx[97]);
  if (xx[2] > xx[9])
    xx[9] = xx[2];
  xx[97] = state[2] - xx[1];
  if (xx[2] > xx[97])
    xx[97] = xx[2];
  xx[1] = xx[97] / xx[4];
  if (xx[5] < xx[1])
    xx[1] = xx[5];
  xx[4] = xx[1] * xx[1] * (xx[7] - xx[8] * xx[1]) * (xx[11] * xx[97] + (xx[97] ==
    xx[2] ? xx[2] : xx[103]));
  if (xx[2] > xx[4])
    xx[4] = xx[2];
  xx[1] = xx[161] - xx[179] - xx[175] - xx[197];
  xx[7] = xx[162] - xx[180] - xx[178] - xx[198];
  xx[11] = xx[37] + xx[128] + xx[135] + xx[57] * xx[1] - xx[55] * xx[7] + xx[166];
  xx[97] = xx[25] + xx[186];
  xx[140] = xx[25] + xx[182];
  xx[141] = xx[183];
  xx[142] = xx[184];
  xx[143] = xx[185];
  xx[144] = xx[97];
  xx[145] = xx[187];
  xx[146] = xx[188];
  xx[147] = xx[189];
  xx[148] = xx[25] + xx[190];
  pm_math_Matrix3x3_xform_ra(xx + 140, xx + 149, xx + 167);
  xx[100] = xx[116] + xx[57] * xx[61] - xx[55] * xx[70] + xx[168];
  xx[103] = xx[25] + xx[163] - xx[181] - xx[181] - xx[199];
  xx[106] = xx[103] + xx[96] * xx[87];
  xx[116] = xx[87] + xx[96] * xx[97];
  xx[119] = xx[106] + xx[96] * xx[116];
  ii[0] = factorSymmetricPosDef(xx + 119, 1, xx + 140);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_2_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[140] = (input[1] - xx[0] * state[3] + xx[9] - xx[4] - (xx[11] + xx[96] *
              xx[100])) / xx[119];
  xx[4] = xx[158] - xx[176] - xx[174] - xx[194];
  xx[9] = xx[25] + xx[159] - xx[177] - xx[177] - xx[195];
  xx[37] = xx[160] - xx[178] - xx[180] - xx[196];
  xx[128] = xx[37] + xx[96] * xx[70];
  xx[141] = xx[35] + xx[126] + xx[133] + xx[57] * xx[49] - xx[55] * xx[59] + xx
    [164] + xx[28] * xx[140];
  xx[142] = xx[36] + xx[127] + xx[134] + xx[57] * xx[4] - xx[55] * xx[9] + xx
    [165] + xx[128] * xx[140];
  xx[143] = xx[11] + xx[106] * xx[140];
  pm_math_Quaternion_xform_ra(xx + 20, xx + 141, xx + 133);
  xx[141] = xx[14];
  xx[142] = - xx[95];
  xx[143] = xx[8] * (xx[89] * xx[15] + xx[94] * xx[17]) - 0.015;
  xx[11] = xx[80] + xx[96] * xx[183];
  xx[35] = xx[88] + xx[96] * xx[189];
  xx[144] = xx[115] + xx[57] * xx[60] - xx[55] * xx[69] + xx[167] + xx[11] * xx
    [140];
  xx[145] = xx[100] + xx[116] * xx[140];
  xx[146] = xx[117] + xx[57] * xx[62] - xx[55] * xx[71] + xx[169] + xx[35] * xx
    [140];
  pm_math_Quaternion_xform_ra(xx + 20, xx + 144, xx + 147);
  pm_math_Vector3_cross_ra(xx + 141, xx + 147, xx + 144);
  xx[36] = 0.0403;
  xx[89] = xx[36] * state[1] * state[1];
  xx[94] = xx[15] * xx[15];
  xx[100] = xx[17] * xx[10];
  xx[115] = xx[15] * xx[19];
  xx[117] = xx[19] * xx[17];
  xx[126] = xx[15] * xx[10];
  xx[127] = xx[19] * xx[10];
  xx[133] = xx[15] * xx[17];
  xx[155] = xx[8] * (xx[94] + xx[17] * xx[17]) - xx[5];
  xx[156] = - (xx[8] * (xx[100] + xx[115]));
  xx[157] = xx[8] * (xx[117] - xx[126]);
  xx[158] = xx[8] * (xx[115] - xx[100]);
  xx[159] = xx[8] * (xx[94] + xx[10] * xx[10]) - xx[5];
  xx[160] = - (xx[8] * (xx[127] + xx[133]));
  xx[161] = xx[8] * (xx[117] + xx[126]);
  xx[162] = xx[8] * (xx[133] - xx[127]);
  xx[163] = xx[8] * (xx[94] + xx[19] * xx[19]) - xx[5];
  xx[5] = xx[11] / xx[119];
  xx[94] = xx[116] / xx[119];
  xx[100] = xx[35] / xx[119];
  xx[164] = xx[60] - xx[28] * xx[5];
  xx[165] = xx[61] - xx[28] * xx[94];
  xx[166] = xx[62] - xx[28] * xx[100];
  xx[167] = xx[69] - xx[128] * xx[5];
  xx[168] = xx[70] - xx[128] * xx[94];
  xx[169] = xx[71] - xx[128] * xx[100];
  xx[170] = xx[80] - xx[106] * xx[5];
  xx[171] = xx[87] - xx[106] * xx[94];
  xx[172] = xx[88] - xx[106] * xx[100];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 164, xx + 155, xx + 173);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 173, xx + 164);
  xx[60] = xx[116] * xx[5];
  xx[61] = xx[35] * xx[5];
  xx[62] = xx[35] * xx[94];
  xx[173] = xx[182] - xx[11] * xx[5] + xx[25];
  xx[174] = xx[183] - xx[60];
  xx[175] = xx[184] - xx[61];
  xx[176] = xx[185] - xx[60];
  xx[177] = xx[97] - xx[116] * xx[94];
  xx[178] = xx[187] - xx[62];
  xx[179] = xx[188] - xx[61];
  xx[180] = xx[189] - xx[62];
  xx[181] = xx[190] - xx[35] * xx[100] + xx[25];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 173, xx + 155, xx + 182);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 182, xx + 173);
  pm_math_Matrix3x3_postCross_ra(xx + 173, xx + 141, xx + 182);
  xx[11] = xx[170] - xx[184];
  xx[35] = xx[28] / xx[119];
  xx[60] = xx[128] * xx[35];
  xx[61] = xx[106] * xx[35];
  xx[62] = xx[128] / xx[119];
  xx[69] = xx[106] * xx[62];
  xx[70] = xx[106] / xx[119];
  xx[191] = xx[49] - xx[28] * xx[35];
  xx[192] = xx[59] - xx[60];
  xx[193] = xx[26] - xx[61];
  xx[194] = xx[4] - xx[60];
  xx[195] = xx[9] - xx[128] * xx[62];
  xx[196] = xx[37] - xx[69];
  xx[197] = xx[1] - xx[61];
  xx[198] = xx[7] - xx[69];
  xx[199] = xx[103] - xx[106] * xx[70];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 191, xx + 155, xx + 200);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 200, xx + 191);
  pm_math_Matrix3x3_postCross_ra(xx + 164, xx + 141, xx + 155);
  pm_math_Matrix3x3_preCross_ra(xx + 182, xx + 141, xx + 200);
  xx[1] = xx[171] - xx[187];
  xx[4] = xx[1] + xx[36] * (xx[25] + xx[177]);
  xx[7] = xx[199] - xx[163] - xx[163] - xx[208] + xx[36] * xx[1] + xx[36] * xx[4]
    + xx[25];
  ii[0] = factorSymmetricPosDef(xx + 7, 1, xx + 1);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_1_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[59] = (xx[11] + xx[36] * xx[174]) / xx[7];
  xx[60] = xx[4] / xx[7];
  xx[61] = (xx[172] - xx[190] + xx[36] * xx[180]) / xx[7];
  xx[1] = xx[6] * state[0];
  xx[4] = sin(xx[1]);
  xx[6] = xx[16] * xx[4];
  xx[9] = 9.806649999999999;
  xx[25] = xx[9] * xx[6];
  xx[26] = xx[3] * xx[4];
  xx[4] = xx[9] * xx[26];
  xx[28] = xx[8] * (xx[6] * xx[25] + xx[26] * xx[4]) - xx[9];
  xx[6] = cos(xx[1]);
  xx[1] = xx[3] * xx[6];
  xx[3] = xx[16] * xx[6];
  xx[6] = xx[8] * (xx[1] * xx[4] + xx[3] * xx[25]);
  xx[9] = xx[8] * (xx[1] * xx[25] - xx[3] * xx[4]);
  xx[115] = xx[28];
  xx[116] = xx[6];
  xx[117] = xx[9];
  xx[1] = (input[0] - xx[0] * state[1] + xx[12] - xx[13] - (xx[135] + xx[146] -
            xx[89] * xx[11] + xx[36] * (xx[148] - xx[89] * xx[176]))) / xx[7] -
    pm_math_Vector3_dot_ra(xx + 59, xx + 115);
  xx[11] = xx[35];
  xx[12] = xx[62];
  xx[13] = xx[70];
  xx[0] = xx[10] * xx[1];
  xx[3] = xx[17] * xx[1];
  xx[4] = xx[8] * (xx[15] * xx[0] + xx[19] * xx[3]);
  xx[7] = xx[8] * (xx[15] * xx[3] - xx[19] * xx[0]);
  xx[15] = xx[1] - xx[8] * (xx[17] * xx[3] + xx[10] * xx[0]);
  xx[59] = xx[4];
  xx[60] = xx[7];
  xx[61] = xx[15];
  xx[69] = xx[5];
  xx[70] = xx[94];
  xx[71] = xx[100];
  xx[115] = xx[28] - xx[89] + xx[95] * xx[1];
  xx[116] = xx[36] * xx[1] + xx[6] + xx[14] * xx[1];
  xx[117] = xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 115, xx + 8);
  xx[0] = xx[140] - (pm_math_Vector3_dot_ra(xx + 11, xx + 59) +
                     pm_math_Vector3_dot_ra(xx + 69, xx + 8));
  xx[11] = xx[27];
  xx[12] = xx[43];
  xx[13] = xx[58];
  xx[19] = xx[4] + xx[57];
  xx[20] = xx[7] - xx[55];
  xx[21] = xx[15] + xx[0];
  pm_math_Quaternion_inverseXform_ra(xx + 45, xx + 19, xx + 3);
  xx[14] = xx[33];
  xx[15] = xx[38];
  xx[16] = xx[44];
  pm_math_Vector3_cross_ra(xx + 19, xx + 112, xx + 25);
  xx[19] = xx[8] + xx[137] + xx[25];
  xx[20] = xx[9] + xx[96] * xx[0] + xx[18] + xx[26];
  xx[21] = xx[10] + xx[24] + xx[27];
  pm_math_Quaternion_inverseXform_ra(xx + 45, xx + 19, xx + 6);
  xx[9] = xx[121] - (pm_math_Vector3_dot_ra(xx + 11, xx + 3) +
                     pm_math_Vector3_dot_ra(xx + 14, xx + 6));
  xx[10] = xx[32];
  xx[11] = xx[54];
  xx[12] = xx[118];
  xx[13] = xx[3] + xx[104];
  xx[14] = xx[4] - xx[56];
  xx[15] = xx[5] + xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 13, xx + 3);
  xx[16] = xx[34];
  xx[17] = xx[50];
  xx[18] = xx[53];
  pm_math_Vector3_cross_ra(xx + 13, xx + 130, xx + 19);
  xx[13] = xx[6] + xx[29] + xx[19];
  xx[14] = xx[7] + xx[30] + xx[20];
  xx[15] = xx[8] + xx[31] + xx[21];
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 13, xx + 6);
  xx[13] = xx[136] - (pm_math_Vector3_dot_ra(xx + 10, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 16, xx + 6));
  xx[10] = xx[66];
  xx[11] = xx[73];
  xx[12] = xx[120];
  xx[14] = xx[3] + xx[86];
  xx[15] = xx[4] - xx[84];
  xx[16] = xx[5] + xx[13];
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 14, xx + 3);
  xx[17] = xx[68];
  xx[18] = xx[72];
  xx[19] = xx[78];
  pm_math_Vector3_cross_ra(xx + 14, xx + 123, xx + 20);
  xx[14] = xx[6] + xx[122] * xx[13] + xx[51] + xx[20];
  xx[15] = xx[7] + xx[105] + xx[21];
  xx[16] = xx[8] + xx[52] + xx[22];
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 14, xx + 6);
  xx[14] = xx[138] - (pm_math_Vector3_dot_ra(xx + 10, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 17, xx + 6));
  xx[10] = xx[67];
  xx[11] = xx[85];
  xx[12] = xx[99];
  xx[15] = xx[3] + xx[98];
  xx[16] = xx[4] - xx[81];
  xx[17] = xx[5] + xx[14];
  pm_math_Quaternion_inverseXform_ra(xx + 90, xx + 15, xx + 3);
  xx[18] = xx[79];
  xx[19] = xx[82];
  xx[20] = xx[83];
  pm_math_Vector3_cross_ra(xx + 15, xx + 152, xx + 21);
  xx[15] = xx[6] + xx[63] + xx[21];
  xx[16] = xx[7] + xx[64] + xx[22];
  xx[17] = xx[8] + xx[65] + xx[23];
  pm_math_Quaternion_inverseXform_ra(xx + 90, xx + 15, xx + 6);
  xx[15] = xx[139] - (pm_math_Vector3_dot_ra(xx + 10, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 18, xx + 6));
  xx[6] = xx[3] + xx[101];
  xx[7] = xx[4] - xx[102];
  xx[8] = xx[5] + xx[15];
  pm_math_Quaternion_inverseXform_ra(xx + 108, xx + 6, xx + 3);
  deriv[0] = state[1];
  deriv[1] = xx[1];
  deriv[2] = state[3];
  deriv[3] = xx[0];
  deriv[4] = state[5];
  deriv[5] = xx[9];
  deriv[6] = state[7];
  deriv[7] = xx[13];
  deriv[8] = state[9];
  deriv[9] = xx[14];
  deriv[10] = state[11];
  deriv[11] = xx[15];
  deriv[12] = state[13];
  deriv[13] = xx[129] - xx[107] * xx[5];
  errorResult[0] = xx[2];
  return NULL;
}
