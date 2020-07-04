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
  double xx[218];
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
  xx[117] = 0.07000000000000001;
  if (0 != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ZeroLengthDivision",
      "The Length parameter of the 'left_arm_simul/Variable Brick Solid' block is zero. For the calculation of solid dimensions from mass, the block requires a nonzero value.",
      neDiagMgr);
  }

  memcpy(xx + 118, xx + 117, 1 * sizeof(double));
  if (0 != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ZeroWidthDivision",
      "The Width parameter of the 'left_arm_simul/Variable Brick Solid' block is zero. For the calculation of solid dimensions from mass, the block requires a nonzero value.",
      neDiagMgr);
  }

  xx[119] = 1.0e4 * xx[118] * xx[117];
  xx[119] = xx[119] == 0.0 ? 0.0 : input[0] / xx[119];
  xx[117] = fabs(xx[119]);
  xx[118] = xx[6] * xx[117];
  xx[119] = input[0] / 12.0;
  xx[123] = (4.900000000000001e-3 + xx[117] * xx[117]) * xx[119];
  xx[124] = input[0];
  xx[125] = xx[2];
  xx[126] = xx[2];
  xx[127] = xx[118];
  xx[128] = xx[123];
  xx[129] = xx[123];
  xx[130] = 9.800000000000001e-3 * xx[119];
  xx[131] = xx[2];
  xx[132] = xx[2];
  xx[133] = xx[2];
  if (!(input[0] != xx[2] || xx[118] == xx[2]) ? 1 : 0 != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:InvalidZeroMass",
      "The zero mass of 'left_arm_simul/Variable Brick Solid' is incompatible with the nonzero center of mass.",
      neDiagMgr);
  }

  xx[117] = xx[124] * xx[125];
  xx[118] = xx[124] * xx[127];
  xx[119] = xx[16] * xx[118];
  xx[123] = xx[16] * xx[117];
  xx[134] = xx[117] + xx[8] * (xx[3] * xx[119] - xx[16] * xx[123]);
  xx[117] = xx[124] * xx[126];
  xx[135] = xx[118] - xx[8] * (xx[3] * xx[123] + xx[16] * xx[119]);
  xx[118] = 0.075;
  xx[119] = xx[135] + xx[118] * xx[124];
  xx[123] = 0.9999999999932537;
  xx[136] = xx[6] * (xx[128] + xx[129] - xx[130]) + xx[127] * xx[127] * xx[124];
  xx[137] = 3.673205103416066e-6;
  xx[138] = xx[125] * xx[127] * xx[124] - xx[132];
  xx[139] = xx[137] * xx[138];
  xx[140] = xx[123] * xx[136] - xx[139];
  xx[141] = xx[123] * xx[138];
  xx[138] = xx[6] * (xx[129] + xx[130] - xx[128]) + xx[125] * xx[125] * xx[124];
  xx[142] = xx[141] - xx[137] * xx[138];
  xx[143] = xx[123] * xx[138] + xx[139];
  xx[138] = xx[141] + xx[137] * xx[136];
  xx[136] = xx[118] * xx[135];
  xx[135] = xx[118] * xx[117];
  xx[139] = 0.9999999999999998;
  xx[141] = xx[125] * xx[126] * xx[124] - xx[133];
  xx[144] = xx[139] * xx[141];
  xx[125] = xx[126] * xx[127] * xx[124] - xx[131];
  xx[127] = xx[139] * xx[125];
  xx[131] = xx[118] * xx[134];
  xx[145] = xx[124];
  xx[146] = xx[134];
  xx[147] = xx[117];
  xx[148] = xx[119];
  xx[149] = xx[123] * xx[140] - xx[137] * xx[142];
  xx[150] = 0.9999999999999996 * (xx[6] * (xx[130] + xx[128] - xx[129]) + xx[126]
    * xx[126] * xx[124]);
  xx[151] = xx[123] * xx[143] + xx[137] * xx[138] + xx[136] + xx[136] + 5.625e-3
    * xx[124];
  xx[152] = xx[6] * (xx[135] - (xx[123] * xx[144] + xx[137] * xx[127]) + xx[135]
                     - xx[139] * (xx[123] * xx[141] + xx[137] * xx[125]));
  xx[153] = xx[6] * (xx[137] * xx[143] - xx[123] * xx[138] + xx[131] + xx[131] -
                     (xx[123] * xx[142] + xx[137] * xx[140]));
  xx[154] = xx[6] * (xx[139] * (xx[123] * xx[125] - xx[137] * xx[141]) + xx[123]
                     * xx[127] - xx[137] * xx[144]);
  if (xx[124] != xx[2] || (xx[134] == xx[2] && xx[117] == xx[2] && xx[119] ==
       xx[2]) ? 0 : 1 != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:InvalidMassDistribution",
      "The rigid component containing 'left_arm_simul/Variable Brick Solid' has invalid zero total mass and nonzero first moment of mass.",
      neDiagMgr);
  }

  xx[117] = xx[150] + xx[151];
  xx[118] = - xx[154];
  xx[119] = - xx[153];
  xx[123] = xx[151] + xx[149];
  xx[124] = - xx[152];
  xx[125] = xx[149] + xx[150];
  xx[126] = xx[117];
  xx[127] = xx[118];
  xx[128] = xx[119];
  xx[129] = xx[118];
  xx[130] = xx[123];
  xx[131] = xx[124];
  xx[132] = xx[119];
  xx[133] = xx[124];
  xx[134] = xx[125];
  pm_math_Matrix3x3_xform_ra(xx + 126, xx + 114, xx + 135);
  pm_math_Vector3_cross_ra(xx + 114, xx + 135, xx + 126);
  xx[118] = 0.03;
  xx[119] = xx[118] * xx[113];
  xx[124] = xx[119] * xx[101];
  xx[129] = xx[145] == 0.0 ? 0.0 : xx[148] / xx[145];
  xx[130] = xx[129] * xx[145];
  xx[131] = xx[145] == 0.0 ? 0.0 : xx[147] / xx[145];
  xx[132] = xx[131] * xx[145];
  xx[133] = xx[118] * xx[112];
  xx[134] = xx[133] * xx[112] + xx[119] * xx[113];
  xx[119] = xx[145] == 0.0 ? 0.0 : xx[146] / xx[145];
  xx[135] = xx[119];
  xx[136] = xx[131];
  xx[137] = xx[129];
  pm_math_Vector3_cross_ra(xx + 114, xx + 135, xx + 138);
  pm_math_Vector3_cross_ra(xx + 114, xx + 138, xx + 135);
  xx[114] = xx[145] * xx[136] + xx[124] * xx[145];
  xx[115] = state[13] * xx[113];
  xx[113] = xx[118] * xx[130];
  xx[116] = xx[118] * xx[145];
  xx[129] = xx[118] * xx[116];
  xx[131] = xx[25] + xx[117] + xx[113] + xx[113] + xx[129];
  xx[117] = state[13] * xx[112];
  xx[112] = 0.1;
  xx[138] = xx[112] * xx[100];
  xx[139] = xx[112] * xx[106];
  xx[140] = 0.05;
  xx[141] = - (xx[8] * (xx[138] * xx[102] - xx[139] * xx[107]));
  xx[142] = xx[140] - xx[8] * (xx[139] * xx[102] + xx[138] * xx[107]);
  xx[143] = - (xx[8] * (xx[139] * xx[106] + xx[138] * xx[100]) - xx[112]);
  pm_math_Vector3_cross_ra(xx + 97, xx + 141, xx + 146);
  pm_math_Vector3_cross_ra(xx + 97, xx + 146, xx + 149);
  pm_math_Quaternion_inverseXform_ra(xx + 108, xx + 149, xx + 97);
  xx[112] = xx[130] + xx[116];
  xx[116] = xx[119] * xx[145];
  xx[119] = xx[153] + xx[118] * xx[116];
  xx[138] = state[12] + xx[1];
  if (xx[2] < xx[138])
    xx[138] = xx[2];
  xx[139] = - (xx[138] / xx[4]);
  if (xx[5] < xx[139])
    xx[139] = xx[5];
  xx[144] = xx[9] * state[13];
  xx[146] = xx[139] * xx[139] * (xx[7] - xx[8] * xx[139]) * ((- xx[138] == xx[2]
    ? xx[2] : - xx[144]) - xx[11] * xx[138]);
  if (xx[2] > xx[146])
    xx[146] = xx[2];
  xx[138] = state[12] - xx[1];
  if (xx[2] > xx[138])
    xx[138] = xx[2];
  xx[139] = xx[138] / xx[4];
  if (xx[5] < xx[139])
    xx[139] = xx[5];
  xx[147] = xx[139] * xx[139] * (xx[7] - xx[8] * xx[139]) * (xx[11] * xx[138] +
    (xx[138] == xx[2] ? xx[2] : xx[144]));
  if (xx[2] > xx[147])
    xx[147] = xx[2];
  xx[138] = xx[133] * xx[101];
  xx[101] = xx[152] + xx[118] * xx[132];
  xx[133] = xx[122] + xx[128] + xx[124] * xx[116] - xx[138] * xx[132] + xx[117] *
    xx[101] - xx[115] * xx[119] + xx[116] * xx[98] - xx[132] * xx[97];
  xx[139] = xx[25] + xx[125];
  memcpy(xx + 125, xx + 139, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 125, 1, xx + 144);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_7_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[144] = (input[7] - xx[0] * state[13] + xx[146] - xx[147] - xx[133]) / xx
    [125];
  xx[122] = xx[145] * xx[135] + xx[138] * xx[145];
  xx[128] = xx[25] + xx[123] + xx[113] + xx[113] + xx[129];
  xx[146] = xx[120] + xx[126] - (xx[124] * xx[130] + xx[132] * xx[134]) - xx[118]
    * xx[114] + xx[115] * xx[131] + xx[117] * xx[154] + xx[132] * xx[99] - xx
    [112] * xx[98] - xx[119] * xx[144];
  xx[147] = xx[121] + xx[127] + xx[138] * xx[130] + xx[116] * xx[134] + xx[118] *
    xx[122] + xx[112] * xx[97] - xx[116] * xx[99] - (xx[115] * xx[154] + xx[117]
    * xx[128]) - xx[101] * xx[144];
  xx[148] = xx[133] + xx[139] * xx[144];
  pm_math_Quaternion_xform_ra(xx + 108, xx + 146, xx + 149);
  xx[113] = xx[25] + xx[145];
  xx[146] = xx[122] + xx[113] * xx[97] - xx[117] * xx[112] - xx[132] * xx[144];
  xx[147] = xx[114] + xx[113] * xx[98] - xx[115] * xx[112] + xx[116] * xx[144];
  xx[148] = xx[145] * xx[137] - xx[134] * xx[145] + xx[115] * xx[132] + xx[117] *
    xx[116] + xx[113] * xx[99];
  pm_math_Quaternion_xform_ra(xx + 108, xx + 146, xx + 97);
  pm_math_Vector3_cross_ra(xx + 141, xx + 97, xx + 120);
  xx[114] = state[11] * xx[95];
  xx[115] = xx[102] * xx[102];
  xx[117] = xx[8] * (xx[115] + xx[106] * xx[106]) - xx[5];
  xx[118] = xx[106] * xx[100];
  xx[123] = xx[102] * xx[107];
  xx[124] = xx[8] * (xx[118] + xx[123]);
  xx[126] = xx[107] * xx[106];
  xx[127] = xx[102] * xx[100];
  xx[129] = xx[8] * (xx[126] - xx[127]);
  xx[130] = xx[8] * (xx[123] - xx[118]);
  xx[118] = xx[8] * (xx[115] + xx[100] * xx[100]) - xx[5];
  xx[123] = xx[107] * xx[100];
  xx[100] = xx[102] * xx[106];
  xx[102] = xx[8] * (xx[123] + xx[100]);
  xx[106] = xx[8] * (xx[126] + xx[127]);
  xx[126] = xx[8] * (xx[100] - xx[123]);
  xx[100] = xx[8] * (xx[115] + xx[107] * xx[107]) - xx[5];
  xx[155] = xx[117];
  xx[156] = - xx[124];
  xx[157] = xx[129];
  xx[158] = xx[130];
  xx[159] = xx[118];
  xx[160] = - xx[102];
  xx[161] = xx[106];
  xx[162] = xx[126];
  xx[163] = xx[100];
  xx[107] = xx[119] / xx[125];
  xx[115] = - (xx[154] + xx[101] * xx[107]);
  xx[123] = xx[139] * xx[107] - xx[119];
  xx[127] = xx[101] / xx[125];
  xx[133] = xx[139] * xx[127] - xx[101];
  xx[134] = xx[139] / xx[125];
  xx[164] = xx[131] - xx[119] * xx[107];
  xx[165] = xx[115];
  xx[166] = xx[123];
  xx[167] = xx[115];
  xx[168] = xx[128] - xx[101] * xx[127];
  xx[169] = xx[133];
  xx[170] = xx[123];
  xx[171] = xx[133];
  xx[172] = xx[139] - xx[139] * xx[134];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 164, xx + 155, xx + 173);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 173, xx + 164);
  xx[115] = xx[132] / xx[125];
  xx[123] = xx[119] * xx[115];
  xx[128] = xx[116] / xx[125];
  xx[125] = xx[119] * xx[128] - xx[112];
  xx[119] = xx[112] - xx[101] * xx[115];
  xx[112] = xx[101] * xx[128];
  xx[101] = xx[139] * xx[115] - xx[132];
  xx[131] = xx[116] - xx[139] * xx[128];
  xx[173] = xx[129] * xx[132] - (xx[123] * xx[117] + xx[124] * xx[125]);
  xx[174] = xx[118] * xx[125] - xx[130] * xx[123] - xx[102] * xx[132];
  xx[175] = xx[126] * xx[125] - xx[106] * xx[123] + xx[132] * xx[100];
  xx[176] = xx[117] * xx[119] - xx[124] * xx[112] - xx[129] * xx[116];
  xx[177] = xx[130] * xx[119] + xx[112] * xx[118] + xx[102] * xx[116];
  xx[178] = xx[106] * xx[119] + xx[126] * xx[112] - xx[116] * xx[100];
  xx[179] = xx[117] * xx[101] - xx[124] * xx[131];
  xx[180] = xx[130] * xx[101] + xx[118] * xx[131];
  xx[181] = xx[106] * xx[101] + xx[126] * xx[131];
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 173, xx + 182);
  pm_math_Matrix3x3_postCross_ra(xx + 182, xx + 141, xx + 173);
  xx[101] = xx[113] - xx[132] * xx[115];
  xx[112] = xx[116] * xx[115];
  xx[119] = xx[113] - xx[116] * xx[128];
  xx[191] = xx[117] * xx[101] - xx[124] * xx[112];
  xx[192] = xx[130] * xx[101] + xx[112] * xx[118];
  xx[193] = xx[106] * xx[101] + xx[126] * xx[112];
  xx[194] = xx[112] * xx[117] - xx[124] * xx[119];
  xx[195] = xx[130] * xx[112] + xx[118] * xx[119];
  xx[196] = xx[106] * xx[112] + xx[126] * xx[119];
  xx[197] = xx[129] * xx[113];
  xx[198] = - (xx[102] * xx[113]);
  xx[199] = xx[100] * xx[113];
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 191, xx + 200);
  pm_math_Matrix3x3_postCross_ra(xx + 200, xx + 141, xx + 152);
  pm_math_Matrix3x3_preCross_ra(xx + 152, xx + 141, xx + 191);
  xx[100] = xx[25] + xx[164] - xx[173] - xx[173] - xx[191];
  xx[101] = state[11] * xx[94];
  xx[94] = xx[165] - xx[174] - xx[176] - xx[192];
  xx[102] = xx[182] - xx[152];
  xx[106] = xx[183] - xx[155];
  xx[112] = xx[184] - xx[158];
  xx[113] = xx[185] - xx[153];
  xx[116] = xx[186] - xx[156];
  xx[117] = xx[187] - xx[159];
  xx[118] = xx[188] - xx[154];
  xx[119] = xx[189] - xx[157];
  xx[123] = xx[190] - xx[160];
  xx[152] = xx[102];
  xx[153] = xx[106];
  xx[154] = xx[112];
  xx[155] = xx[113];
  xx[156] = xx[116];
  xx[157] = xx[117];
  xx[158] = xx[118];
  xx[159] = xx[119];
  xx[160] = xx[123];
  xx[124] = xx[140] * xx[89];
  xx[125] = xx[140] * xx[85];
  xx[129] = xx[8] * (xx[124] * xx[84] + xx[125] * xx[67]);
  xx[130] = - (xx[140] - xx[8] * (xx[124] * xx[89] + xx[125] * xx[85]));
  xx[131] = 0.03260000000000002 - xx[8] * (xx[124] * xx[67] - xx[125] * xx[84]);
  pm_math_Vector3_cross_ra(xx + 80, xx + 129, xx + 124);
  pm_math_Vector3_cross_ra(xx + 80, xx + 124, xx + 135);
  pm_math_Quaternion_inverseXform_ra(xx + 90, xx + 135, xx + 80);
  xx[124] = xx[140] * state[11];
  xx[125] = xx[124] * (xx[96] + xx[83]) + xx[81];
  xx[81] = xx[82] - xx[124] * (xx[95] + xx[95]);
  xx[135] = xx[80];
  xx[136] = xx[125];
  xx[137] = xx[81];
  pm_math_Matrix3x3_xform_ra(xx + 152, xx + 135, xx + 145);
  xx[82] = xx[166] - xx[175] - xx[179] - xx[193];
  xx[83] = xx[82] + xx[140] * xx[102];
  xx[95] = state[10] + xx[1];
  if (xx[2] < xx[95])
    xx[95] = xx[2];
  xx[96] = - (xx[95] / xx[4]);
  if (xx[5] < xx[96])
    xx[96] = xx[5];
  xx[124] = xx[9] * state[11];
  xx[126] = xx[96] * xx[96] * (xx[7] - xx[8] * xx[96]) * ((- xx[95] == xx[2] ?
    xx[2] : - xx[124]) - xx[11] * xx[95]);
  if (xx[2] > xx[126])
    xx[126] = xx[2];
  xx[95] = state[10] - xx[1];
  if (xx[2] > xx[95])
    xx[95] = xx[2];
  xx[96] = xx[95] / xx[4];
  if (xx[5] < xx[96])
    xx[96] = xx[5];
  xx[132] = xx[96] * xx[96] * (xx[7] - xx[8] * xx[96]) * (xx[11] * xx[95] + (xx
    [95] == xx[2] ? xx[2] : xx[124]));
  if (xx[2] > xx[132])
    xx[132] = xx[2];
  xx[95] = xx[170] - xx[179] - xx[175] - xx[197];
  xx[96] = xx[171] - xx[180] - xx[178] - xx[198];
  xx[124] = xx[105] + xx[151] + xx[122] + xx[114] * xx[95] - xx[101] * xx[96] +
    xx[147];
  xx[133] = xx[25] + xx[200];
  xx[152] = xx[133];
  xx[153] = xx[201];
  xx[154] = xx[202];
  xx[155] = xx[203];
  xx[156] = xx[25] + xx[204];
  xx[157] = xx[205];
  xx[158] = xx[206];
  xx[159] = xx[207];
  xx[160] = xx[25] + xx[208];
  pm_math_Matrix3x3_xform_ra(xx + 152, xx + 135, xx + 161);
  xx[135] = xx[97] + xx[114] * xx[102] - xx[101] * xx[113] + xx[161];
  xx[97] = xx[25] + xx[172] - xx[181] - xx[181] - xx[199];
  xx[136] = xx[97] + xx[140] * xx[118];
  xx[137] = xx[118] + xx[140] * xx[133];
  xx[138] = xx[136] + xx[140] * xx[137];
  ii[0] = factorSymmetricPosDef(xx + 138, 1, xx + 139);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_6_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[139] = (input[6] - xx[0] * state[11] + xx[126] - xx[132] - (xx[124] + xx
              [140] * xx[135])) / xx[138];
  xx[105] = xx[167] - xx[176] - xx[174] - xx[194];
  xx[122] = xx[25] + xx[168] - xx[177] - xx[177] - xx[195];
  xx[126] = xx[169] - xx[178] - xx[180] - xx[196];
  xx[132] = xx[126] + xx[140] * xx[113];
  xx[151] = xx[103] + xx[149] + xx[120] + xx[114] * xx[100] - xx[101] * xx[94] +
    xx[145] + xx[83] * xx[139];
  xx[152] = xx[104] + xx[150] + xx[121] + xx[114] * xx[105] - xx[101] * xx[122]
    + xx[146] + xx[132] * xx[139];
  xx[153] = xx[124] + xx[136] * xx[139];
  pm_math_Quaternion_xform_ra(xx + 90, xx + 151, xx + 145);
  xx[103] = xx[119] + xx[140] * xx[203];
  xx[104] = xx[123] + xx[140] * xx[206];
  xx[148] = xx[135] + xx[137] * xx[139];
  xx[149] = xx[98] + xx[114] * xx[106] - xx[101] * xx[116] + xx[162] + xx[103] *
    xx[139];
  xx[150] = xx[99] + xx[114] * xx[112] - xx[101] * xx[117] + xx[163] + xx[104] *
    xx[139];
  pm_math_Quaternion_xform_ra(xx + 90, xx + 148, xx + 151);
  pm_math_Vector3_cross_ra(xx + 129, xx + 151, xx + 148);
  xx[98] = state[9] * xx[79];
  xx[79] = xx[84] * xx[84];
  xx[99] = xx[85] * xx[67];
  xx[120] = xx[84] * xx[89];
  xx[121] = xx[84] * xx[67];
  xx[124] = xx[89] * xx[85];
  xx[135] = xx[89] * xx[67];
  xx[154] = xx[84] * xx[85];
  xx[155] = xx[8] * (xx[79] + xx[85] * xx[85]) - xx[5];
  xx[156] = - (xx[8] * (xx[99] + xx[120]));
  xx[157] = xx[8] * (xx[121] - xx[124]);
  xx[158] = xx[8] * (xx[120] - xx[99]);
  xx[159] = xx[8] * (xx[79] + xx[67] * xx[67]) - xx[5];
  xx[160] = xx[8] * (xx[135] + xx[154]);
  xx[161] = - (xx[8] * (xx[124] + xx[121]));
  xx[162] = xx[8] * (xx[135] - xx[154]);
  xx[163] = xx[8] * (xx[79] + xx[89] * xx[89]) - xx[5];
  xx[67] = xx[83] / xx[138];
  xx[79] = xx[132] * xx[67];
  xx[84] = xx[136] * xx[67];
  xx[85] = xx[132] / xx[138];
  xx[89] = xx[136] * xx[85];
  xx[99] = xx[136] / xx[138];
  xx[164] = xx[100] - xx[83] * xx[67];
  xx[165] = xx[94] - xx[79];
  xx[166] = xx[82] - xx[84];
  xx[167] = xx[105] - xx[79];
  xx[168] = xx[122] - xx[132] * xx[85];
  xx[169] = xx[126] - xx[89];
  xx[170] = xx[95] - xx[84];
  xx[171] = xx[96] - xx[89];
  xx[172] = xx[97] - xx[136] * xx[99];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 164, xx + 155, xx + 173);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 173, xx + 164);
  xx[79] = xx[137] / xx[138];
  xx[82] = xx[103] / xx[138];
  xx[84] = xx[104] / xx[138];
  xx[173] = xx[102] - xx[83] * xx[79];
  xx[174] = xx[106] - xx[83] * xx[82];
  xx[175] = xx[112] - xx[83] * xx[84];
  xx[176] = xx[113] - xx[132] * xx[79];
  xx[177] = xx[116] - xx[132] * xx[82];
  xx[178] = xx[117] - xx[132] * xx[84];
  xx[179] = xx[118] - xx[136] * xx[79];
  xx[180] = xx[119] - xx[136] * xx[82];
  xx[181] = xx[123] - xx[136] * xx[84];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 173, xx + 155, xx + 116);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 116, xx + 173);
  pm_math_Matrix3x3_postCross_ra(xx + 173, xx + 129, xx + 116);
  xx[83] = xx[103] * xx[79];
  xx[89] = xx[104] * xx[79];
  xx[94] = xx[104] * xx[82];
  xx[182] = xx[133] - xx[137] * xx[79];
  xx[183] = xx[201] - xx[83];
  xx[184] = xx[202] - xx[89];
  xx[185] = xx[203] - xx[83];
  xx[186] = xx[204] - xx[103] * xx[82] + xx[25];
  xx[187] = xx[205] - xx[94];
  xx[188] = xx[206] - xx[89];
  xx[189] = xx[207] - xx[94];
  xx[190] = xx[208] - xx[104] * xx[84] + xx[25];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 182, xx + 155, xx + 191);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 191, xx + 182);
  pm_math_Matrix3x3_postCross_ra(xx + 182, xx + 129, xx + 154);
  pm_math_Matrix3x3_preCross_ra(xx + 154, xx + 129, xx + 191);
  xx[83] = xx[25] + xx[164] - xx[116] - xx[116] - xx[191];
  xx[89] = state[9] * xx[78];
  xx[78] = xx[165] - xx[117] - xx[119] - xx[192];
  xx[94] = xx[173] - xx[154];
  xx[95] = xx[174] - xx[157];
  xx[96] = xx[175] - xx[160];
  xx[97] = xx[176] - xx[155];
  xx[100] = xx[177] - xx[158];
  xx[102] = xx[178] - xx[161];
  xx[103] = xx[179] - xx[156];
  xx[104] = xx[180] - xx[159];
  xx[105] = xx[181] - xx[162];
  xx[154] = xx[94];
  xx[155] = xx[95];
  xx[156] = xx[96];
  xx[157] = xx[97];
  xx[158] = xx[100];
  xx[159] = xx[102];
  xx[160] = xx[103];
  xx[161] = xx[104];
  xx[162] = xx[105];
  xx[106] = 0.18;
  xx[112] = xx[106] * xx[66];
  xx[113] = xx[106] * xx[72];
  xx[126] = 0.09;
  xx[135] = - (xx[8] * (xx[112] * xx[68] - xx[113] * xx[73]));
  xx[136] = xx[126] - xx[8] * (xx[113] * xx[68] + xx[112] * xx[73]);
  xx[137] = - (xx[8] * (xx[113] * xx[72] + xx[112] * xx[66]) - xx[106]);
  pm_math_Vector3_cross_ra(xx + 63, xx + 135, xx + 173);
  pm_math_Vector3_cross_ra(xx + 63, xx + 173, xx + 176);
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 176, xx + 63);
  pm_math_Matrix3x3_xform_ra(xx + 154, xx + 63, xx + 173);
  xx[106] = xx[166] - xx[118] - xx[122] - xx[193];
  xx[112] = state[8] + xx[1];
  if (xx[2] < xx[112])
    xx[112] = xx[2];
  xx[113] = - (xx[112] / xx[4]);
  if (xx[5] < xx[113])
    xx[113] = xx[5];
  xx[132] = xx[9] * state[9];
  xx[133] = xx[113] * xx[113] * (xx[7] - xx[8] * xx[113]) * ((- xx[112] == xx[2]
    ? xx[2] : - xx[132]) - xx[11] * xx[112]);
  if (xx[2] > xx[133])
    xx[133] = xx[2];
  xx[112] = state[8] - xx[1];
  if (xx[2] > xx[112])
    xx[112] = xx[2];
  xx[113] = xx[112] / xx[4];
  if (xx[5] < xx[113])
    xx[113] = xx[5];
  xx[138] = xx[113] * xx[113] * (xx[7] - xx[8] * xx[113]) * (xx[11] * xx[112] +
    (xx[112] == xx[2] ? xx[2] : xx[132]));
  if (xx[2] > xx[138])
    xx[138] = xx[2];
  xx[112] = xx[170] - xx[122] - xx[118] - xx[197];
  xx[113] = xx[171] - xx[123] - xx[121] - xx[198];
  xx[132] = xx[88] + xx[147] + xx[150] + xx[98] * xx[112] - xx[89] * xx[113] +
    xx[175];
  xx[154] = xx[25] + xx[172] - xx[124] - xx[124] - xx[199];
  memcpy(xx + 155, xx + 154, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 155, 1, xx + 156);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_5_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[156] = (input[5] - xx[0] * state[9] + xx[133] - xx[138] - xx[132]) / xx[155];
  xx[88] = xx[167] - xx[119] - xx[117] - xx[194];
  xx[116] = xx[25] + xx[168] - xx[120] - xx[120] - xx[195];
  xx[117] = xx[169] - xx[121] - xx[123] - xx[196];
  xx[118] = xx[86] + xx[145] + xx[148] + xx[98] * xx[83] - xx[89] * xx[78] + xx
    [173] + xx[106] * xx[156];
  xx[119] = xx[87] + xx[146] + xx[149] + xx[98] * xx[88] - xx[89] * xx[116] +
    xx[174] + xx[117] * xx[156];
  xx[120] = xx[132] + xx[154] * xx[156];
  pm_math_Quaternion_xform_ra(xx + 74, xx + 118, xx + 121);
  xx[157] = xx[25] + xx[182];
  xx[158] = xx[183];
  xx[159] = xx[184];
  xx[160] = xx[185];
  xx[161] = xx[25] + xx[186];
  xx[162] = xx[187];
  xx[163] = xx[188];
  xx[164] = xx[189];
  xx[165] = xx[25] + xx[190];
  pm_math_Matrix3x3_xform_ra(xx + 157, xx + 63, xx + 118);
  xx[145] = xx[151] + xx[98] * xx[94] - xx[89] * xx[97] + xx[118] + xx[103] *
    xx[156];
  xx[146] = xx[152] + xx[98] * xx[95] - xx[89] * xx[100] + xx[119] + xx[104] *
    xx[156];
  xx[147] = xx[153] + xx[98] * xx[96] - xx[89] * xx[102] + xx[120] + xx[105] *
    xx[156];
  pm_math_Quaternion_xform_ra(xx + 74, xx + 145, xx + 118);
  pm_math_Vector3_cross_ra(xx + 135, xx + 118, xx + 145);
  xx[86] = state[7] * xx[61];
  xx[87] = xx[68] * xx[68];
  xx[124] = xx[72] * xx[66];
  xx[132] = xx[68] * xx[73];
  xx[133] = xx[73] * xx[72];
  xx[138] = xx[68] * xx[66];
  xx[148] = xx[73] * xx[66];
  xx[149] = xx[68] * xx[72];
  xx[157] = xx[8] * (xx[87] + xx[72] * xx[72]) - xx[5];
  xx[158] = - (xx[8] * (xx[124] + xx[132]));
  xx[159] = xx[8] * (xx[133] - xx[138]);
  xx[160] = xx[8] * (xx[132] - xx[124]);
  xx[161] = xx[8] * (xx[87] + xx[66] * xx[66]) - xx[5];
  xx[162] = - (xx[8] * (xx[148] + xx[149]));
  xx[163] = xx[8] * (xx[133] + xx[138]);
  xx[164] = xx[8] * (xx[149] - xx[148]);
  xx[165] = xx[8] * (xx[87] + xx[73] * xx[73]) - xx[5];
  xx[66] = xx[106] / xx[155];
  xx[68] = xx[117] * xx[66];
  xx[72] = xx[154] * xx[66];
  xx[73] = xx[117] / xx[155];
  xx[87] = xx[154] * xx[73];
  xx[124] = xx[154] / xx[155];
  xx[166] = xx[83] - xx[106] * xx[66];
  xx[167] = xx[78] - xx[68];
  xx[168] = xx[106] - xx[72];
  xx[169] = xx[88] - xx[68];
  xx[170] = xx[116] - xx[117] * xx[73];
  xx[171] = xx[117] - xx[87];
  xx[172] = xx[112] - xx[72];
  xx[173] = xx[113] - xx[87];
  xx[174] = xx[154] - xx[154] * xx[124];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 166, xx + 157, xx + 191);
  pm_math_Matrix3x3_compose_ra(xx + 157, xx + 191, xx + 166);
  xx[68] = xx[103] / xx[155];
  xx[72] = xx[104] / xx[155];
  xx[78] = xx[105] / xx[155];
  xx[191] = xx[94] - xx[106] * xx[68];
  xx[192] = xx[95] - xx[106] * xx[72];
  xx[193] = xx[96] - xx[106] * xx[78];
  xx[194] = xx[97] - xx[117] * xx[68];
  xx[195] = xx[100] - xx[117] * xx[72];
  xx[196] = xx[102] - xx[117] * xx[78];
  xx[197] = xx[103] - xx[154] * xx[68];
  xx[198] = xx[104] - xx[154] * xx[72];
  xx[199] = xx[105] - xx[154] * xx[78];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 191, xx + 157, xx + 200);
  pm_math_Matrix3x3_compose_ra(xx + 157, xx + 200, xx + 191);
  pm_math_Matrix3x3_postCross_ra(xx + 191, xx + 135, xx + 200);
  xx[83] = xx[104] * xx[68];
  xx[87] = xx[105] * xx[68];
  xx[88] = xx[105] * xx[72];
  xx[209] = xx[182] - xx[103] * xx[68] + xx[25];
  xx[210] = xx[183] - xx[83];
  xx[211] = xx[184] - xx[87];
  xx[212] = xx[185] - xx[83];
  xx[213] = xx[186] - xx[104] * xx[72] + xx[25];
  xx[214] = xx[187] - xx[88];
  xx[215] = xx[188] - xx[87];
  xx[216] = xx[189] - xx[88];
  xx[217] = xx[190] - xx[105] * xx[78] + xx[25];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 209, xx + 157, xx + 175);
  pm_math_Matrix3x3_compose_ra(xx + 157, xx + 175, xx + 209);
  pm_math_Matrix3x3_postCross_ra(xx + 209, xx + 135, xx + 157);
  pm_math_Matrix3x3_preCross_ra(xx + 157, xx + 135, xx + 175);
  xx[83] = xx[25] + xx[166] - xx[200] - xx[200] - xx[175];
  xx[87] = state[7] * xx[60];
  xx[60] = xx[167] - xx[201] - xx[203] - xx[176];
  xx[88] = xx[191] - xx[157];
  xx[94] = xx[192] - xx[160];
  xx[95] = xx[193] - xx[163];
  xx[96] = xx[194] - xx[158];
  xx[97] = xx[195] - xx[161];
  xx[100] = xx[196] - xx[164];
  xx[102] = xx[197] - xx[159];
  xx[103] = xx[198] - xx[162];
  xx[104] = xx[199] - xx[165];
  xx[157] = xx[88];
  xx[158] = xx[94];
  xx[159] = xx[95];
  xx[160] = xx[96];
  xx[161] = xx[97];
  xx[162] = xx[100];
  xx[163] = xx[102];
  xx[164] = xx[103];
  xx[165] = xx[104];
  xx[105] = xx[126] * xx[34];
  xx[106] = xx[126] * xx[32];
  xx[148] = - (xx[8] * (xx[105] * xx[54] - xx[106] * xx[56]));
  xx[149] = - (xx[126] - xx[8] * (xx[105] * xx[34] + xx[106] * xx[32]));
  xx[150] = 0.06 + xx[8] * (xx[106] * xx[54] + xx[105] * xx[56]);
  pm_math_Vector3_cross_ra(xx + 51, xx + 148, xx + 151);
  pm_math_Vector3_cross_ra(xx + 51, xx + 151, xx + 184);
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 184, xx + 51);
  xx[105] = xx[126] * state[7];
  xx[106] = xx[105] * (xx[62] + xx[55]) + xx[52];
  xx[52] = xx[53] - xx[105] * (xx[61] + xx[61]);
  xx[151] = xx[51];
  xx[152] = xx[106];
  xx[153] = xx[52];
  pm_math_Matrix3x3_xform_ra(xx + 157, xx + 151, xx + 184);
  xx[53] = xx[168] - xx[202] - xx[206] - xx[177];
  xx[55] = xx[53] + xx[126] * xx[88];
  xx[61] = state[6] + xx[1];
  if (xx[2] < xx[61])
    xx[61] = xx[2];
  xx[62] = - (xx[61] / xx[4]);
  if (xx[5] < xx[62])
    xx[62] = xx[5];
  xx[105] = xx[9] * state[7];
  xx[112] = xx[62] * xx[62] * (xx[7] - xx[8] * xx[62]) * ((- xx[61] == xx[2] ?
    xx[2] : - xx[105]) - xx[11] * xx[61]);
  if (xx[2] > xx[112])
    xx[112] = xx[2];
  xx[61] = state[6] - xx[1];
  if (xx[2] > xx[61])
    xx[61] = xx[2];
  xx[62] = xx[61] / xx[4];
  if (xx[5] < xx[62])
    xx[62] = xx[5];
  xx[113] = xx[62] * xx[62] * (xx[7] - xx[8] * xx[62]) * (xx[11] * xx[61] + (xx
    [61] == xx[2] ? xx[2] : xx[105]));
  if (xx[2] > xx[113])
    xx[113] = xx[2];
  xx[61] = xx[172] - xx[206] - xx[202] - xx[181];
  xx[62] = xx[173] - xx[207] - xx[205] - xx[182];
  xx[105] = xx[71] + xx[123] + xx[147] + xx[86] * xx[61] - xx[87] * xx[62] + xx
    [186];
  xx[116] = xx[25] + xx[209];
  xx[157] = xx[116];
  xx[158] = xx[210];
  xx[159] = xx[211];
  xx[160] = xx[212];
  xx[161] = xx[25] + xx[213];
  xx[162] = xx[214];
  xx[163] = xx[215];
  xx[164] = xx[216];
  xx[165] = xx[25] + xx[217];
  pm_math_Matrix3x3_xform_ra(xx + 157, xx + 151, xx + 187);
  xx[117] = xx[118] + xx[86] * xx[88] - xx[87] * xx[96] + xx[187];
  xx[118] = xx[25] + xx[174] - xx[208] - xx[208] - xx[183];
  xx[132] = xx[118] + xx[126] * xx[102];
  xx[133] = xx[102] + xx[126] * xx[116];
  xx[138] = xx[132] + xx[126] * xx[133];
  ii[0] = factorSymmetricPosDef(xx + 138, 1, xx + 151);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_4_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[151] = (input[4] - xx[0] * state[7] + xx[112] - xx[113] - (xx[105] + xx[126]
              * xx[117])) / xx[138];
  xx[71] = xx[169] - xx[203] - xx[201] - xx[178];
  xx[112] = xx[25] + xx[170] - xx[204] - xx[204] - xx[179];
  xx[113] = xx[171] - xx[205] - xx[207] - xx[180];
  xx[123] = xx[113] + xx[126] * xx[96];
  xx[152] = xx[69] + xx[121] + xx[145] + xx[86] * xx[83] - xx[87] * xx[60] + xx
    [184] + xx[55] * xx[151];
  xx[153] = xx[70] + xx[122] + xx[146] + xx[86] * xx[71] - xx[87] * xx[112] +
    xx[185] + xx[123] * xx[151];
  xx[154] = xx[105] + xx[132] * xx[151];
  pm_math_Quaternion_xform_ra(xx + 39, xx + 152, xx + 145);
  xx[69] = xx[103] + xx[126] * xx[212];
  xx[70] = xx[104] + xx[126] * xx[215];
  xx[152] = xx[117] + xx[133] * xx[151];
  xx[153] = xx[119] + xx[86] * xx[94] - xx[87] * xx[97] + xx[188] + xx[69] * xx
    [151];
  xx[154] = xx[120] + xx[86] * xx[95] - xx[87] * xx[100] + xx[189] + xx[70] *
    xx[151];
  pm_math_Quaternion_xform_ra(xx + 39, xx + 152, xx + 119);
  pm_math_Vector3_cross_ra(xx + 148, xx + 119, xx + 152);
  xx[105] = state[5] * xx[50];
  xx[50] = xx[54] * xx[54];
  xx[117] = xx[54] * xx[34];
  xx[122] = xx[56] * xx[32];
  xx[155] = xx[32] * xx[34];
  xx[157] = xx[54] * xx[56];
  xx[158] = xx[54] * xx[32];
  xx[54] = xx[56] * xx[34];
  xx[159] = xx[8] * (xx[50] + xx[32] * xx[32]) - xx[5];
  xx[160] = xx[8] * (xx[117] - xx[122]);
  xx[161] = xx[8] * (xx[155] + xx[157]);
  xx[162] = - (xx[8] * (xx[122] + xx[117]));
  xx[163] = xx[8] * (xx[50] + xx[56] * xx[56]) - xx[5];
  xx[164] = xx[8] * (xx[158] - xx[54]);
  xx[165] = xx[8] * (xx[155] - xx[157]);
  xx[166] = - (xx[8] * (xx[54] + xx[158]));
  xx[167] = xx[8] * (xx[50] + xx[34] * xx[34]) - xx[5];
  xx[32] = xx[55] / xx[138];
  xx[34] = xx[123] * xx[32];
  xx[50] = xx[132] * xx[32];
  xx[54] = xx[123] / xx[138];
  xx[56] = xx[132] * xx[54];
  xx[117] = xx[132] / xx[138];
  xx[168] = xx[83] - xx[55] * xx[32];
  xx[169] = xx[60] - xx[34];
  xx[170] = xx[53] - xx[50];
  xx[171] = xx[71] - xx[34];
  xx[172] = xx[112] - xx[123] * xx[54];
  xx[173] = xx[113] - xx[56];
  xx[174] = xx[61] - xx[50];
  xx[175] = xx[62] - xx[56];
  xx[176] = xx[118] - xx[132] * xx[117];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 168, xx + 159, xx + 177);
  pm_math_Matrix3x3_compose_ra(xx + 159, xx + 177, xx + 168);
  xx[34] = xx[133] / xx[138];
  xx[50] = xx[69] / xx[138];
  xx[53] = xx[70] / xx[138];
  xx[177] = xx[88] - xx[55] * xx[34];
  xx[178] = xx[94] - xx[55] * xx[50];
  xx[179] = xx[95] - xx[55] * xx[53];
  xx[180] = xx[96] - xx[123] * xx[34];
  xx[181] = xx[97] - xx[123] * xx[50];
  xx[182] = xx[100] - xx[123] * xx[53];
  xx[183] = xx[102] - xx[132] * xx[34];
  xx[184] = xx[103] - xx[132] * xx[50];
  xx[185] = xx[104] - xx[132] * xx[53];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 177, xx + 159, xx + 186);
  pm_math_Matrix3x3_compose_ra(xx + 159, xx + 186, xx + 177);
  pm_math_Matrix3x3_postCross_ra(xx + 177, xx + 148, xx + 186);
  xx[55] = xx[69] * xx[34];
  xx[56] = xx[70] * xx[34];
  xx[60] = xx[70] * xx[50];
  xx[195] = xx[116] - xx[133] * xx[34];
  xx[196] = xx[210] - xx[55];
  xx[197] = xx[211] - xx[56];
  xx[198] = xx[212] - xx[55];
  xx[199] = xx[213] - xx[69] * xx[50] + xx[25];
  xx[200] = xx[214] - xx[60];
  xx[201] = xx[215] - xx[56];
  xx[202] = xx[216] - xx[60];
  xx[203] = xx[217] - xx[70] * xx[53] + xx[25];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 195, xx + 159, xx + 204);
  pm_math_Matrix3x3_compose_ra(xx + 159, xx + 204, xx + 195);
  pm_math_Matrix3x3_postCross_ra(xx + 195, xx + 148, xx + 157);
  pm_math_Matrix3x3_preCross_ra(xx + 157, xx + 148, xx + 204);
  xx[55] = xx[25] + xx[168] - xx[186] - xx[186] - xx[204];
  xx[56] = state[5] * xx[49];
  xx[49] = xx[169] - xx[187] - xx[189] - xx[205];
  xx[60] = xx[177] - xx[157];
  xx[61] = xx[178] - xx[160];
  xx[62] = xx[179] - xx[163];
  xx[69] = xx[180] - xx[158];
  xx[70] = xx[181] - xx[161];
  xx[71] = xx[182] - xx[164];
  xx[83] = xx[183] - xx[159];
  xx[88] = xx[184] - xx[162];
  xx[94] = xx[185] - xx[165];
  xx[157] = xx[60];
  xx[158] = xx[61];
  xx[159] = xx[62];
  xx[160] = xx[69];
  xx[161] = xx[70];
  xx[162] = xx[71];
  xx[163] = xx[83];
  xx[164] = xx[88];
  xx[165] = xx[94];
  xx[95] = 0.21;
  xx[96] = xx[95] * xx[44];
  xx[97] = xx[95] * xx[43];
  xx[100] = 0.105;
  xx[102] = xx[8] * (xx[96] * xx[33] + xx[97] * xx[38]) - xx[100];
  xx[103] = - (xx[8] * (xx[96] * xx[38] - xx[97] * xx[33]));
  xx[104] = - (xx[8] * (xx[97] * xx[43] + xx[96] * xx[44]) - xx[95]);
  pm_math_Vector3_cross_ra(xx + 29, xx + 102, xx + 95);
  pm_math_Vector3_cross_ra(xx + 29, xx + 95, xx + 177);
  pm_math_Quaternion_inverseXform_ra(xx + 45, xx + 177, xx + 29);
  pm_math_Matrix3x3_xform_ra(xx + 157, xx + 29, xx + 95);
  xx[112] = xx[170] - xx[188] - xx[192] - xx[206];
  xx[113] = state[4] + xx[1];
  if (xx[2] < xx[113])
    xx[113] = xx[2];
  xx[116] = - (xx[113] / xx[4]);
  if (xx[5] < xx[116])
    xx[116] = xx[5];
  xx[118] = xx[9] * state[5];
  xx[122] = xx[116] * xx[116] * (xx[7] - xx[8] * xx[116]) * ((- xx[113] == xx[2]
    ? xx[2] : - xx[118]) - xx[11] * xx[113]);
  if (xx[2] > xx[122])
    xx[122] = xx[2];
  xx[113] = state[4] - xx[1];
  if (xx[2] > xx[113])
    xx[113] = xx[2];
  xx[116] = xx[113] / xx[4];
  if (xx[5] < xx[116])
    xx[116] = xx[5];
  xx[123] = xx[116] * xx[116] * (xx[7] - xx[8] * xx[116]) * (xx[11] * xx[113] +
    (xx[113] == xx[2] ? xx[2] : xx[118]));
  if (xx[2] > xx[123])
    xx[123] = xx[2];
  xx[113] = xx[174] - xx[192] - xx[188] - xx[210];
  xx[116] = xx[175] - xx[193] - xx[191] - xx[211];
  xx[118] = xx[59] + xx[147] + xx[154] + xx[105] * xx[113] - xx[56] * xx[116] +
    xx[97];
  xx[132] = xx[25] + xx[176] - xx[194] - xx[194] - xx[212];
  memcpy(xx + 133, xx + 132, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 133, 1, xx + 138);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_3_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[138] = (input[3] - xx[0] * state[5] + xx[122] - xx[123] - xx[118]) / xx[133];
  xx[59] = xx[171] - xx[189] - xx[187] - xx[207];
  xx[122] = xx[25] + xx[172] - xx[190] - xx[190] - xx[208];
  xx[97] = xx[173] - xx[191] - xx[193] - xx[209];
  xx[157] = xx[57] + xx[145] + xx[152] + xx[105] * xx[55] - xx[56] * xx[49] +
    xx[95] + xx[112] * xx[138];
  xx[158] = xx[58] + xx[146] + xx[153] + xx[105] * xx[59] - xx[56] * xx[122] +
    xx[96] + xx[97] * xx[138];
  xx[159] = xx[118] + xx[132] * xx[138];
  pm_math_Quaternion_xform_ra(xx + 45, xx + 157, xx + 145);
  xx[157] = xx[25] + xx[195];
  xx[158] = xx[196];
  xx[159] = xx[197];
  xx[160] = xx[198];
  xx[161] = xx[25] + xx[199];
  xx[162] = xx[200];
  xx[163] = xx[201];
  xx[164] = xx[202];
  xx[165] = xx[25] + xx[203];
  pm_math_Matrix3x3_xform_ra(xx + 157, xx + 29, xx + 152);
  xx[157] = xx[119] + xx[105] * xx[60] - xx[56] * xx[69] + xx[152] + xx[83] *
    xx[138];
  xx[158] = xx[120] + xx[105] * xx[61] - xx[56] * xx[70] + xx[153] + xx[88] *
    xx[138];
  xx[159] = xx[121] + xx[105] * xx[62] - xx[56] * xx[71] + xx[154] + xx[94] *
    xx[138];
  pm_math_Quaternion_xform_ra(xx + 45, xx + 157, xx + 118);
  pm_math_Vector3_cross_ra(xx + 102, xx + 118, xx + 152);
  xx[57] = xx[27] * state[3];
  xx[27] = xx[33] * xx[33];
  xx[58] = xx[33] * xx[38];
  xx[95] = xx[44] * xx[43];
  xx[96] = xx[38] * xx[43];
  xx[121] = xx[33] * xx[44];
  xx[123] = xx[33] * xx[43];
  xx[33] = xx[44] * xx[38];
  xx[157] = xx[8] * (xx[27] + xx[43] * xx[43]) - xx[5];
  xx[158] = xx[8] * (xx[58] - xx[95]);
  xx[159] = xx[8] * (xx[96] + xx[121]);
  xx[160] = - (xx[8] * (xx[95] + xx[58]));
  xx[161] = xx[8] * (xx[27] + xx[44] * xx[44]) - xx[5];
  xx[162] = xx[8] * (xx[123] - xx[33]);
  xx[163] = xx[8] * (xx[96] - xx[121]);
  xx[164] = - (xx[8] * (xx[33] + xx[123]));
  xx[165] = xx[8] * (xx[27] + xx[38] * xx[38]) - xx[5];
  xx[27] = xx[112] / xx[133];
  xx[33] = xx[97] * xx[27];
  xx[38] = xx[132] * xx[27];
  xx[43] = xx[97] / xx[133];
  xx[44] = xx[132] * xx[43];
  xx[58] = xx[132] / xx[133];
  xx[166] = xx[55] - xx[112] * xx[27];
  xx[167] = xx[49] - xx[33];
  xx[168] = xx[112] - xx[38];
  xx[169] = xx[59] - xx[33];
  xx[170] = xx[122] - xx[97] * xx[43];
  xx[171] = xx[97] - xx[44];
  xx[172] = xx[113] - xx[38];
  xx[173] = xx[116] - xx[44];
  xx[174] = xx[132] - xx[132] * xx[58];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 166, xx + 157, xx + 175);
  pm_math_Matrix3x3_compose_ra(xx + 157, xx + 175, xx + 166);
  xx[33] = xx[83] / xx[133];
  xx[38] = xx[88] / xx[133];
  xx[44] = xx[94] / xx[133];
  xx[175] = xx[60] - xx[112] * xx[33];
  xx[176] = xx[61] - xx[112] * xx[38];
  xx[177] = xx[62] - xx[112] * xx[44];
  xx[178] = xx[69] - xx[97] * xx[33];
  xx[179] = xx[70] - xx[97] * xx[38];
  xx[180] = xx[71] - xx[97] * xx[44];
  xx[181] = xx[83] - xx[132] * xx[33];
  xx[182] = xx[88] - xx[132] * xx[38];
  xx[183] = xx[94] - xx[132] * xx[44];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 175, xx + 157, xx + 184);
  pm_math_Matrix3x3_compose_ra(xx + 157, xx + 184, xx + 175);
  pm_math_Matrix3x3_postCross_ra(xx + 175, xx + 102, xx + 184);
  xx[49] = xx[88] * xx[33];
  xx[55] = xx[94] * xx[33];
  xx[59] = xx[94] * xx[38];
  xx[204] = xx[195] - xx[83] * xx[33] + xx[25];
  xx[205] = xx[196] - xx[49];
  xx[206] = xx[197] - xx[55];
  xx[207] = xx[198] - xx[49];
  xx[208] = xx[199] - xx[88] * xx[38] + xx[25];
  xx[209] = xx[200] - xx[59];
  xx[210] = xx[201] - xx[55];
  xx[211] = xx[202] - xx[59];
  xx[212] = xx[203] - xx[94] * xx[44] + xx[25];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 204, xx + 157, xx + 193);
  pm_math_Matrix3x3_compose_ra(xx + 157, xx + 193, xx + 202);
  pm_math_Matrix3x3_postCross_ra(xx + 202, xx + 102, xx + 157);
  pm_math_Matrix3x3_preCross_ra(xx + 157, xx + 102, xx + 193);
  xx[49] = xx[25] + xx[166] - xx[184] - xx[184] - xx[193];
  xx[55] = xx[26] * state[3];
  xx[59] = xx[167] - xx[185] - xx[187] - xx[194];
  xx[60] = xx[175] - xx[157];
  xx[61] = xx[176] - xx[160];
  xx[62] = xx[177] - xx[163];
  xx[69] = xx[178] - xx[158];
  xx[70] = xx[179] - xx[161];
  xx[71] = xx[180] - xx[164];
  xx[83] = xx[181] - xx[159];
  xx[88] = xx[182] - xx[162];
  xx[94] = xx[183] - xx[165];
  xx[157] = xx[60];
  xx[158] = xx[61];
  xx[159] = xx[62];
  xx[160] = xx[69];
  xx[161] = xx[70];
  xx[162] = xx[71];
  xx[163] = xx[83];
  xx[164] = xx[88];
  xx[165] = xx[94];
  xx[95] = xx[100] * xx[10];
  xx[96] = xx[100] * xx[19];
  xx[97] = xx[8] * (xx[95] * xx[17] - xx[96] * xx[15]);
  xx[112] = xx[97] * state[1] * state[1];
  xx[113] = xx[19] * xx[112];
  xx[121] = xx[14];
  xx[122] = xx[10];
  xx[123] = xx[18];
  xx[14] = 0.02 - (xx[8] * (xx[95] * xx[10] + xx[96] * xx[19]) - xx[100]);
  xx[18] = xx[14] * state[1] * state[1];
  xx[116] = xx[19] * xx[18];
  xx[132] = xx[10] * xx[18] - xx[17] * xx[112];
  xx[175] = xx[113];
  xx[176] = xx[116];
  xx[177] = xx[132];
  pm_math_Vector3_cross_ra(xx + 121, xx + 175, xx + 178);
  xx[121] = xx[100] * state[3];
  xx[122] = xx[8] * (xx[15] * xx[113] + xx[178]) - xx[18] - xx[121] * (xx[28] +
    xx[24]);
  xx[18] = xx[112] + xx[8] * (xx[15] * xx[116] + xx[179]);
  xx[24] = xx[121] * (xx[26] + xx[26]) + xx[8] * (xx[180] + xx[15] * xx[132]);
  xx[175] = xx[122];
  xx[176] = xx[18];
  xx[177] = xx[24];
  pm_math_Matrix3x3_xform_ra(xx + 157, xx + 175, xx + 178);
  xx[26] = xx[168] - xx[186] - xx[190] - xx[195];
  xx[28] = xx[26] + xx[100] * xx[61];
  xx[112] = state[2] + xx[1];
  if (xx[2] < xx[112])
    xx[112] = xx[2];
  xx[113] = - (xx[112] / xx[4]);
  if (xx[5] < xx[113])
    xx[113] = xx[5];
  xx[116] = xx[9] * state[3];
  xx[9] = xx[113] * xx[113] * (xx[7] - xx[8] * xx[113]) * ((- xx[112] == xx[2] ?
    xx[2] : - xx[116]) - xx[11] * xx[112]);
  if (xx[2] > xx[9])
    xx[9] = xx[2];
  xx[112] = state[2] - xx[1];
  if (xx[2] > xx[112])
    xx[112] = xx[2];
  xx[1] = xx[112] / xx[4];
  if (xx[5] < xx[1])
    xx[1] = xx[5];
  xx[4] = xx[1] * xx[1] * (xx[7] - xx[8] * xx[1]) * (xx[11] * xx[112] + (xx[112]
    == xx[2] ? xx[2] : xx[116]));
  if (xx[2] > xx[4])
    xx[4] = xx[2];
  xx[1] = xx[172] - xx[190] - xx[186] - xx[199];
  xx[7] = xx[173] - xx[191] - xx[189] - xx[200];
  xx[11] = xx[37] + xx[147] + xx[154] + xx[57] * xx[1] - xx[55] * xx[7] + xx[180];
  xx[112] = xx[25] + xx[206];
  xx[157] = xx[25] + xx[202];
  xx[158] = xx[203];
  xx[159] = xx[204];
  xx[160] = xx[205];
  xx[161] = xx[112];
  xx[162] = xx[207];
  xx[163] = xx[208];
  xx[164] = xx[209];
  xx[165] = xx[25] + xx[210];
  pm_math_Matrix3x3_xform_ra(xx + 157, xx + 175, xx + 181);
  xx[113] = xx[119] + xx[57] * xx[61] - xx[55] * xx[70] + xx[182];
  xx[116] = xx[25] + xx[174] - xx[192] - xx[192] - xx[201];
  xx[119] = xx[116] + xx[100] * xx[88];
  xx[121] = xx[88] + xx[100] * xx[112];
  xx[123] = xx[119] + xx[100] * xx[121];
  ii[0] = factorSymmetricPosDef(xx + 123, 1, xx + 132);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_2_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[132] = (input[2] - xx[0] * state[3] + xx[9] - xx[4] - (xx[11] + xx[100] *
              xx[113])) / xx[123];
  xx[4] = xx[169] - xx[187] - xx[185] - xx[196];
  xx[9] = xx[25] + xx[170] - xx[188] - xx[188] - xx[197];
  xx[37] = xx[171] - xx[189] - xx[191] - xx[198];
  xx[133] = xx[37] + xx[100] * xx[70];
  xx[157] = xx[35] + xx[145] + xx[152] + xx[57] * xx[49] - xx[55] * xx[59] + xx
    [178] + xx[28] * xx[132];
  xx[158] = xx[36] + xx[146] + xx[153] + xx[57] * xx[4] - xx[55] * xx[9] + xx
    [179] + xx[133] * xx[132];
  xx[159] = xx[11] + xx[119] * xx[132];
  pm_math_Quaternion_xform_ra(xx + 20, xx + 157, xx + 145);
  xx[152] = xx[14];
  xx[153] = - xx[97];
  xx[154] = xx[8] * (xx[95] * xx[15] + xx[96] * xx[17]) - 0.015;
  xx[11] = xx[83] + xx[100] * xx[203];
  xx[35] = xx[94] + xx[100] * xx[209];
  xx[157] = xx[118] + xx[57] * xx[60] - xx[55] * xx[69] + xx[181] + xx[11] * xx
    [132];
  xx[158] = xx[113] + xx[121] * xx[132];
  xx[159] = xx[120] + xx[57] * xx[62] - xx[55] * xx[71] + xx[183] + xx[35] * xx
    [132];
  pm_math_Quaternion_xform_ra(xx + 20, xx + 157, xx + 160);
  pm_math_Vector3_cross_ra(xx + 152, xx + 160, xx + 157);
  xx[36] = 0.0403;
  xx[95] = xx[36] * state[1] * state[1];
  xx[96] = xx[15] * xx[15];
  xx[113] = xx[17] * xx[10];
  xx[118] = xx[15] * xx[19];
  xx[120] = xx[19] * xx[17];
  xx[145] = xx[15] * xx[10];
  xx[146] = xx[19] * xx[10];
  xx[155] = xx[15] * xx[17];
  xx[162] = xx[8] * (xx[96] + xx[17] * xx[17]) - xx[5];
  xx[163] = - (xx[8] * (xx[113] + xx[118]));
  xx[164] = xx[8] * (xx[120] - xx[145]);
  xx[165] = xx[8] * (xx[118] - xx[113]);
  xx[166] = xx[8] * (xx[96] + xx[10] * xx[10]) - xx[5];
  xx[167] = - (xx[8] * (xx[146] + xx[155]));
  xx[168] = xx[8] * (xx[120] + xx[145]);
  xx[169] = xx[8] * (xx[155] - xx[146]);
  xx[170] = xx[8] * (xx[96] + xx[19] * xx[19]) - xx[5];
  xx[5] = xx[11] / xx[123];
  xx[96] = xx[121] / xx[123];
  xx[113] = xx[35] / xx[123];
  xx[171] = xx[60] - xx[28] * xx[5];
  xx[172] = xx[61] - xx[28] * xx[96];
  xx[173] = xx[62] - xx[28] * xx[113];
  xx[174] = xx[69] - xx[133] * xx[5];
  xx[175] = xx[70] - xx[133] * xx[96];
  xx[176] = xx[71] - xx[133] * xx[113];
  xx[177] = xx[83] - xx[119] * xx[5];
  xx[178] = xx[88] - xx[119] * xx[96];
  xx[179] = xx[94] - xx[119] * xx[113];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 171, xx + 162, xx + 180);
  pm_math_Matrix3x3_compose_ra(xx + 162, xx + 180, xx + 171);
  xx[60] = xx[121] * xx[5];
  xx[61] = xx[35] * xx[5];
  xx[62] = xx[35] * xx[96];
  xx[180] = xx[202] - xx[11] * xx[5] + xx[25];
  xx[181] = xx[203] - xx[60];
  xx[182] = xx[204] - xx[61];
  xx[183] = xx[205] - xx[60];
  xx[184] = xx[112] - xx[121] * xx[96];
  xx[185] = xx[207] - xx[62];
  xx[186] = xx[208] - xx[61];
  xx[187] = xx[209] - xx[62];
  xx[188] = xx[210] - xx[35] * xx[113] + xx[25];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 180, xx + 162, xx + 189);
  pm_math_Matrix3x3_compose_ra(xx + 162, xx + 189, xx + 180);
  pm_math_Matrix3x3_postCross_ra(xx + 180, xx + 152, xx + 189);
  xx[11] = xx[177] - xx[191];
  xx[35] = xx[28] / xx[123];
  xx[60] = xx[133] * xx[35];
  xx[61] = xx[119] * xx[35];
  xx[62] = xx[133] / xx[123];
  xx[69] = xx[119] * xx[62];
  xx[70] = xx[119] / xx[123];
  xx[198] = xx[49] - xx[28] * xx[35];
  xx[199] = xx[59] - xx[60];
  xx[200] = xx[26] - xx[61];
  xx[201] = xx[4] - xx[60];
  xx[202] = xx[9] - xx[133] * xx[62];
  xx[203] = xx[37] - xx[69];
  xx[204] = xx[1] - xx[61];
  xx[205] = xx[7] - xx[69];
  xx[206] = xx[116] - xx[119] * xx[70];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 198, xx + 162, xx + 207);
  pm_math_Matrix3x3_compose_ra(xx + 162, xx + 207, xx + 198);
  pm_math_Matrix3x3_postCross_ra(xx + 171, xx + 152, xx + 162);
  pm_math_Matrix3x3_preCross_ra(xx + 189, xx + 152, xx + 207);
  xx[1] = xx[178] - xx[194];
  xx[4] = xx[1] + xx[36] * (xx[25] + xx[184]);
  xx[7] = xx[206] - xx[170] - xx[170] - xx[215] + xx[36] * xx[1] + xx[36] * xx[4]
    + xx[25];
  ii[0] = factorSymmetricPosDef(xx + 7, 1, xx + 1);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/la_1_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[59] = (xx[11] + xx[36] * xx[181]) / xx[7];
  xx[60] = xx[4] / xx[7];
  xx[61] = (xx[179] - xx[197] + xx[36] * xx[187]) / xx[7];
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
  xx[118] = xx[28];
  xx[119] = xx[6];
  xx[120] = xx[9];
  xx[1] = (input[1] - xx[0] * state[1] + xx[12] - xx[13] - (xx[147] + xx[159] -
            xx[95] * xx[11] + xx[36] * (xx[161] - xx[95] * xx[183]))) / xx[7] -
    pm_math_Vector3_dot_ra(xx + 59, xx + 118);
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
  xx[70] = xx[96];
  xx[71] = xx[113];
  xx[118] = xx[28] - xx[95] + xx[97] * xx[1];
  xx[119] = xx[36] * xx[1] + xx[6] + xx[14] * xx[1];
  xx[120] = xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 118, xx + 8);
  xx[0] = xx[132] - (pm_math_Vector3_dot_ra(xx + 11, xx + 59) +
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
  pm_math_Vector3_cross_ra(xx + 19, xx + 102, xx + 25);
  xx[19] = xx[8] + xx[122] + xx[25];
  xx[20] = xx[9] + xx[100] * xx[0] + xx[18] + xx[26];
  xx[21] = xx[10] + xx[24] + xx[27];
  pm_math_Quaternion_inverseXform_ra(xx + 45, xx + 19, xx + 6);
  xx[9] = xx[138] - (pm_math_Vector3_dot_ra(xx + 11, xx + 3) +
                     pm_math_Vector3_dot_ra(xx + 14, xx + 6));
  xx[10] = xx[32];
  xx[11] = xx[54];
  xx[12] = xx[117];
  xx[13] = xx[3] + xx[105];
  xx[14] = xx[4] - xx[56];
  xx[15] = xx[5] + xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 13, xx + 3);
  xx[16] = xx[34];
  xx[17] = xx[50];
  xx[18] = xx[53];
  pm_math_Vector3_cross_ra(xx + 13, xx + 148, xx + 19);
  xx[13] = xx[6] + xx[29] + xx[19];
  xx[14] = xx[7] + xx[30] + xx[20];
  xx[15] = xx[8] + xx[31] + xx[21];
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 13, xx + 6);
  xx[13] = xx[151] - (pm_math_Vector3_dot_ra(xx + 10, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 16, xx + 6));
  xx[10] = xx[66];
  xx[11] = xx[73];
  xx[12] = xx[124];
  xx[14] = xx[3] + xx[86];
  xx[15] = xx[4] - xx[87];
  xx[16] = xx[5] + xx[13];
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 14, xx + 3);
  xx[17] = xx[68];
  xx[18] = xx[72];
  xx[19] = xx[78];
  pm_math_Vector3_cross_ra(xx + 14, xx + 135, xx + 20);
  xx[14] = xx[6] + xx[126] * xx[13] + xx[51] + xx[20];
  xx[15] = xx[7] + xx[106] + xx[21];
  xx[16] = xx[8] + xx[52] + xx[22];
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 14, xx + 6);
  xx[14] = xx[156] - (pm_math_Vector3_dot_ra(xx + 10, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 17, xx + 6));
  xx[10] = xx[67];
  xx[11] = xx[85];
  xx[12] = xx[99];
  xx[15] = xx[3] + xx[98];
  xx[16] = xx[4] - xx[89];
  xx[17] = xx[5] + xx[14];
  pm_math_Quaternion_inverseXform_ra(xx + 90, xx + 15, xx + 3);
  xx[18] = xx[79];
  xx[19] = xx[82];
  xx[20] = xx[84];
  pm_math_Vector3_cross_ra(xx + 15, xx + 129, xx + 21);
  xx[15] = xx[6] + xx[63] + xx[21];
  xx[16] = xx[7] + xx[64] + xx[22];
  xx[17] = xx[8] + xx[65] + xx[23];
  pm_math_Quaternion_inverseXform_ra(xx + 90, xx + 15, xx + 6);
  xx[15] = xx[139] - (pm_math_Vector3_dot_ra(xx + 10, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 18, xx + 6));
  xx[10] = - xx[107];
  xx[11] = - xx[127];
  xx[12] = xx[134];
  xx[16] = xx[3] + xx[114];
  xx[17] = xx[4] - xx[101];
  xx[18] = xx[5] + xx[15];
  pm_math_Quaternion_inverseXform_ra(xx + 108, xx + 16, xx + 3);
  pm_math_Vector3_cross_ra(xx + 16, xx + 141, xx + 19);
  xx[16] = xx[6] + xx[140] * xx[15] + xx[80] + xx[19];
  xx[17] = xx[7] + xx[125] + xx[20];
  xx[18] = xx[8] + xx[81] + xx[21];
  pm_math_Quaternion_inverseXform_ra(xx + 108, xx + 16, xx + 6);
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
  deriv[13] = xx[144] - (pm_math_Vector3_dot_ra(xx + 10, xx + 3) + xx[128] * xx
    [7] - xx[115] * xx[6]);
  errorResult[0] = xx[2];
  return NULL;
}
