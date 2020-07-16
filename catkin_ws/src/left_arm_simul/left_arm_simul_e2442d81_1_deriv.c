/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'left_arm_simul/Manipulator/Solver Configuration'.
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
  double xx[219];
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
      "The Length parameter of the 'left_arm_simul/Manipulator/Variable Brick Solid' block is zero. For the calculation of solid dimensions from mass, the block requires a nonzero value.",
      neDiagMgr);
  }

  memcpy(xx + 118, xx + 117, 1 * sizeof(double));
  if (0 != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:ZeroWidthDivision",
      "The Width parameter of the 'left_arm_simul/Manipulator/Variable Brick Solid' block is zero. For the calculation of solid dimensions from mass, the block requires a nonzero value.",
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
      "The zero mass of 'left_arm_simul/Manipulator/Variable Brick Solid' is incompatible with the nonzero center of mass.",
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
      "The rigid component containing 'left_arm_simul/Manipulator/Variable Brick Solid' has invalid zero total mass and nonzero first moment of mass.",
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
      "'left_arm_simul/Manipulator/la_7_joint' has a degenerate mass distribution on its follower side.",
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
  xx[113] = 0.15;
  xx[118] = xx[113] + xx[145];
  xx[146] = xx[122] + xx[118] * xx[97] - xx[117] * xx[112] - xx[132] * xx[144];
  xx[147] = xx[114] + xx[118] * xx[98] - xx[115] * xx[112] + xx[116] * xx[144];
  xx[148] = xx[145] * xx[137] - xx[134] * xx[145] + xx[115] * xx[132] + xx[117] *
    xx[116] + xx[118] * xx[99];
  pm_math_Quaternion_xform_ra(xx + 108, xx + 146, xx + 97);
  pm_math_Vector3_cross_ra(xx + 141, xx + 97, xx + 120);
  xx[114] = state[11] * xx[95];
  xx[115] = xx[102] * xx[102];
  xx[117] = xx[8] * (xx[115] + xx[106] * xx[106]) - xx[5];
  xx[123] = xx[106] * xx[100];
  xx[124] = xx[102] * xx[107];
  xx[126] = xx[8] * (xx[123] + xx[124]);
  xx[127] = xx[107] * xx[106];
  xx[129] = xx[102] * xx[100];
  xx[130] = xx[8] * (xx[127] - xx[129]);
  xx[133] = xx[8] * (xx[124] - xx[123]);
  xx[123] = xx[8] * (xx[115] + xx[100] * xx[100]) - xx[5];
  xx[124] = xx[107] * xx[100];
  xx[100] = xx[102] * xx[106];
  xx[102] = xx[8] * (xx[124] + xx[100]);
  xx[106] = xx[8] * (xx[127] + xx[129]);
  xx[127] = xx[8] * (xx[100] - xx[124]);
  xx[100] = xx[8] * (xx[115] + xx[107] * xx[107]) - xx[5];
  xx[155] = xx[117];
  xx[156] = - xx[126];
  xx[157] = xx[130];
  xx[158] = xx[133];
  xx[159] = xx[123];
  xx[160] = - xx[102];
  xx[161] = xx[106];
  xx[162] = xx[127];
  xx[163] = xx[100];
  xx[107] = xx[119] / xx[125];
  xx[115] = - (xx[154] + xx[101] * xx[107]);
  xx[124] = xx[139] * xx[107] - xx[119];
  xx[129] = xx[101] / xx[125];
  xx[134] = xx[139] * xx[129] - xx[101];
  xx[135] = xx[139] / xx[125];
  xx[164] = xx[131] - xx[119] * xx[107];
  xx[165] = xx[115];
  xx[166] = xx[124];
  xx[167] = xx[115];
  xx[168] = xx[128] - xx[101] * xx[129];
  xx[169] = xx[134];
  xx[170] = xx[124];
  xx[171] = xx[134];
  xx[172] = xx[139] - xx[139] * xx[135];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 164, xx + 155, xx + 173);
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 173, xx + 164);
  xx[115] = xx[132] / xx[125];
  xx[124] = xx[119] * xx[115];
  xx[128] = xx[116] / xx[125];
  xx[125] = xx[119] * xx[128] - xx[112];
  xx[119] = xx[112] - xx[101] * xx[115];
  xx[112] = xx[101] * xx[128];
  xx[101] = xx[139] * xx[115] - xx[132];
  xx[131] = xx[116] - xx[139] * xx[128];
  xx[173] = xx[130] * xx[132] - (xx[124] * xx[117] + xx[126] * xx[125]);
  xx[174] = xx[123] * xx[125] - xx[133] * xx[124] - xx[102] * xx[132];
  xx[175] = xx[127] * xx[125] - xx[106] * xx[124] + xx[132] * xx[100];
  xx[176] = xx[117] * xx[119] - xx[126] * xx[112] - xx[130] * xx[116];
  xx[177] = xx[133] * xx[119] + xx[112] * xx[123] + xx[102] * xx[116];
  xx[178] = xx[106] * xx[119] + xx[127] * xx[112] - xx[116] * xx[100];
  xx[179] = xx[117] * xx[101] - xx[126] * xx[131];
  xx[180] = xx[133] * xx[101] + xx[123] * xx[131];
  xx[181] = xx[106] * xx[101] + xx[127] * xx[131];
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 173, xx + 182);
  pm_math_Matrix3x3_postCross_ra(xx + 182, xx + 141, xx + 173);
  xx[101] = xx[118] - xx[132] * xx[115];
  xx[112] = xx[116] * xx[115];
  xx[119] = xx[118] - xx[116] * xx[128];
  xx[191] = xx[117] * xx[101] - xx[126] * xx[112];
  xx[192] = xx[133] * xx[101] + xx[112] * xx[123];
  xx[193] = xx[106] * xx[101] + xx[127] * xx[112];
  xx[194] = xx[112] * xx[117] - xx[126] * xx[119];
  xx[195] = xx[133] * xx[112] + xx[123] * xx[119];
  xx[196] = xx[106] * xx[112] + xx[127] * xx[119];
  xx[197] = xx[130] * xx[118];
  xx[198] = - (xx[102] * xx[118]);
  xx[199] = xx[100] * xx[118];
  pm_math_Matrix3x3_compose_ra(xx + 155, xx + 191, xx + 200);
  pm_math_Matrix3x3_postCross_ra(xx + 200, xx + 141, xx + 152);
  pm_math_Matrix3x3_preCross_ra(xx + 152, xx + 141, xx + 191);
  xx[100] = xx[25] + xx[164] - xx[173] - xx[173] - xx[191];
  xx[101] = state[11] * xx[94];
  xx[94] = xx[165] - xx[174] - xx[176] - xx[192];
  xx[102] = xx[182] - xx[152];
  xx[106] = xx[183] - xx[155];
  xx[112] = xx[184] - xx[158];
  xx[116] = xx[185] - xx[153];
  xx[117] = xx[186] - xx[156];
  xx[118] = xx[187] - xx[159];
  xx[119] = xx[188] - xx[154];
  xx[123] = xx[189] - xx[157];
  xx[124] = xx[190] - xx[160];
  xx[152] = xx[102];
  xx[153] = xx[106];
  xx[154] = xx[112];
  xx[155] = xx[116];
  xx[156] = xx[117];
  xx[157] = xx[118];
  xx[158] = xx[119];
  xx[159] = xx[123];
  xx[160] = xx[124];
  xx[125] = xx[140] * xx[89];
  xx[126] = xx[140] * xx[85];
  xx[130] = xx[8] * (xx[125] * xx[84] + xx[126] * xx[67]);
  xx[131] = - (xx[140] - xx[8] * (xx[125] * xx[89] + xx[126] * xx[85]));
  xx[132] = 0.03260000000000002 - xx[8] * (xx[125] * xx[67] - xx[126] * xx[84]);
  pm_math_Vector3_cross_ra(xx + 80, xx + 130, xx + 125);
  pm_math_Vector3_cross_ra(xx + 80, xx + 125, xx + 136);
  pm_math_Quaternion_inverseXform_ra(xx + 90, xx + 136, xx + 80);
  xx[125] = xx[140] * state[11];
  xx[126] = xx[125] * (xx[96] + xx[83]) + xx[81];
  xx[81] = xx[82] - xx[125] * (xx[95] + xx[95]);
  xx[136] = xx[80];
  xx[137] = xx[126];
  xx[138] = xx[81];
  pm_math_Matrix3x3_xform_ra(xx + 152, xx + 136, xx + 145);
  xx[82] = xx[166] - xx[175] - xx[179] - xx[193];
  xx[83] = xx[82] + xx[140] * xx[102];
  xx[95] = state[10] + xx[1];
  if (xx[2] < xx[95])
    xx[95] = xx[2];
  xx[96] = - (xx[95] / xx[4]);
  if (xx[5] < xx[96])
    xx[96] = xx[5];
  xx[125] = xx[9] * state[11];
  xx[127] = xx[96] * xx[96] * (xx[7] - xx[8] * xx[96]) * ((- xx[95] == xx[2] ?
    xx[2] : - xx[125]) - xx[11] * xx[95]);
  if (xx[2] > xx[127])
    xx[127] = xx[2];
  xx[95] = state[10] - xx[1];
  if (xx[2] > xx[95])
    xx[95] = xx[2];
  xx[96] = xx[95] / xx[4];
  if (xx[5] < xx[96])
    xx[96] = xx[5];
  xx[133] = xx[96] * xx[96] * (xx[7] - xx[8] * xx[96]) * (xx[11] * xx[95] + (xx
    [95] == xx[2] ? xx[2] : xx[125]));
  if (xx[2] > xx[133])
    xx[133] = xx[2];
  xx[95] = xx[170] - xx[179] - xx[175] - xx[197];
  xx[96] = xx[171] - xx[180] - xx[178] - xx[198];
  xx[125] = xx[105] + xx[151] + xx[122] + xx[114] * xx[95] - xx[101] * xx[96] +
    xx[147];
  xx[134] = xx[113] + xx[200];
  xx[152] = xx[134];
  xx[153] = xx[201];
  xx[154] = xx[202];
  xx[155] = xx[203];
  xx[156] = xx[113] + xx[204];
  xx[157] = xx[205];
  xx[158] = xx[206];
  xx[159] = xx[207];
  xx[160] = xx[113] + xx[208];
  pm_math_Matrix3x3_xform_ra(xx + 152, xx + 136, xx + 161);
  xx[136] = xx[97] + xx[114] * xx[102] - xx[101] * xx[116] + xx[161];
  xx[97] = xx[25] + xx[172] - xx[181] - xx[181] - xx[199];
  xx[137] = xx[97] + xx[140] * xx[119];
  xx[138] = xx[119] + xx[140] * xx[134];
  xx[139] = xx[137] + xx[140] * xx[138];
  ii[0] = factorSymmetricPosDef(xx + 139, 1, xx + 148);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/Manipulator/la_6_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[148] = (input[6] - xx[0] * state[11] + xx[127] - xx[133] - (xx[125] + xx
              [140] * xx[136])) / xx[139];
  xx[105] = xx[167] - xx[176] - xx[174] - xx[194];
  xx[122] = xx[25] + xx[168] - xx[177] - xx[177] - xx[195];
  xx[127] = xx[169] - xx[178] - xx[180] - xx[196];
  xx[133] = xx[127] + xx[140] * xx[116];
  xx[151] = xx[103] + xx[149] + xx[120] + xx[114] * xx[100] - xx[101] * xx[94] +
    xx[145] + xx[83] * xx[148];
  xx[152] = xx[104] + xx[150] + xx[121] + xx[114] * xx[105] - xx[101] * xx[122]
    + xx[146] + xx[133] * xx[148];
  xx[153] = xx[125] + xx[137] * xx[148];
  pm_math_Quaternion_xform_ra(xx + 90, xx + 151, xx + 145);
  xx[103] = xx[123] + xx[140] * xx[203];
  xx[104] = xx[124] + xx[140] * xx[206];
  xx[149] = xx[136] + xx[138] * xx[148];
  xx[150] = xx[98] + xx[114] * xx[106] - xx[101] * xx[117] + xx[162] + xx[103] *
    xx[148];
  xx[151] = xx[99] + xx[114] * xx[112] - xx[101] * xx[118] + xx[163] + xx[104] *
    xx[148];
  pm_math_Quaternion_xform_ra(xx + 90, xx + 149, xx + 152);
  pm_math_Vector3_cross_ra(xx + 130, xx + 152, xx + 149);
  xx[98] = state[9] * xx[79];
  xx[79] = xx[84] * xx[84];
  xx[99] = xx[85] * xx[67];
  xx[120] = xx[84] * xx[89];
  xx[121] = xx[84] * xx[67];
  xx[125] = xx[89] * xx[85];
  xx[136] = xx[89] * xx[67];
  xx[155] = xx[84] * xx[85];
  xx[156] = xx[8] * (xx[79] + xx[85] * xx[85]) - xx[5];
  xx[157] = - (xx[8] * (xx[99] + xx[120]));
  xx[158] = xx[8] * (xx[121] - xx[125]);
  xx[159] = xx[8] * (xx[120] - xx[99]);
  xx[160] = xx[8] * (xx[79] + xx[67] * xx[67]) - xx[5];
  xx[161] = xx[8] * (xx[136] + xx[155]);
  xx[162] = - (xx[8] * (xx[125] + xx[121]));
  xx[163] = xx[8] * (xx[136] - xx[155]);
  xx[164] = xx[8] * (xx[79] + xx[89] * xx[89]) - xx[5];
  xx[67] = xx[83] / xx[139];
  xx[79] = xx[133] * xx[67];
  xx[84] = xx[137] * xx[67];
  xx[85] = xx[133] / xx[139];
  xx[89] = xx[137] * xx[85];
  xx[99] = xx[137] / xx[139];
  xx[165] = xx[100] - xx[83] * xx[67];
  xx[166] = xx[94] - xx[79];
  xx[167] = xx[82] - xx[84];
  xx[168] = xx[105] - xx[79];
  xx[169] = xx[122] - xx[133] * xx[85];
  xx[170] = xx[127] - xx[89];
  xx[171] = xx[95] - xx[84];
  xx[172] = xx[96] - xx[89];
  xx[173] = xx[97] - xx[137] * xx[99];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 165, xx + 156, xx + 174);
  pm_math_Matrix3x3_compose_ra(xx + 156, xx + 174, xx + 165);
  xx[79] = xx[138] / xx[139];
  xx[82] = xx[103] / xx[139];
  xx[84] = xx[104] / xx[139];
  xx[174] = xx[102] - xx[83] * xx[79];
  xx[175] = xx[106] - xx[83] * xx[82];
  xx[176] = xx[112] - xx[83] * xx[84];
  xx[177] = xx[116] - xx[133] * xx[79];
  xx[178] = xx[117] - xx[133] * xx[82];
  xx[179] = xx[118] - xx[133] * xx[84];
  xx[180] = xx[119] - xx[137] * xx[79];
  xx[181] = xx[123] - xx[137] * xx[82];
  xx[182] = xx[124] - xx[137] * xx[84];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 174, xx + 156, xx + 116);
  pm_math_Matrix3x3_compose_ra(xx + 156, xx + 116, xx + 174);
  pm_math_Matrix3x3_postCross_ra(xx + 174, xx + 130, xx + 116);
  xx[83] = xx[103] * xx[79];
  xx[89] = xx[104] * xx[79];
  xx[94] = xx[104] * xx[82];
  xx[183] = xx[134] - xx[138] * xx[79];
  xx[184] = xx[201] - xx[83];
  xx[185] = xx[202] - xx[89];
  xx[186] = xx[203] - xx[83];
  xx[187] = xx[204] - xx[103] * xx[82] + xx[113];
  xx[188] = xx[205] - xx[94];
  xx[189] = xx[206] - xx[89];
  xx[190] = xx[207] - xx[94];
  xx[191] = xx[208] - xx[104] * xx[84] + xx[113];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 183, xx + 156, xx + 192);
  pm_math_Matrix3x3_compose_ra(xx + 156, xx + 192, xx + 183);
  pm_math_Matrix3x3_postCross_ra(xx + 183, xx + 130, xx + 155);
  pm_math_Matrix3x3_preCross_ra(xx + 155, xx + 130, xx + 192);
  xx[83] = xx[25] + xx[165] - xx[116] - xx[116] - xx[192];
  xx[89] = state[9] * xx[78];
  xx[78] = xx[166] - xx[117] - xx[119] - xx[193];
  xx[94] = xx[174] - xx[155];
  xx[95] = xx[175] - xx[158];
  xx[96] = xx[176] - xx[161];
  xx[97] = xx[177] - xx[156];
  xx[100] = xx[178] - xx[159];
  xx[102] = xx[179] - xx[162];
  xx[103] = xx[180] - xx[157];
  xx[104] = xx[181] - xx[160];
  xx[105] = xx[182] - xx[163];
  xx[155] = xx[94];
  xx[156] = xx[95];
  xx[157] = xx[96];
  xx[158] = xx[97];
  xx[159] = xx[100];
  xx[160] = xx[102];
  xx[161] = xx[103];
  xx[162] = xx[104];
  xx[163] = xx[105];
  xx[106] = 0.18;
  xx[112] = xx[106] * xx[66];
  xx[113] = xx[106] * xx[72];
  xx[125] = 0.09;
  xx[136] = - (xx[8] * (xx[112] * xx[68] - xx[113] * xx[73]));
  xx[137] = xx[125] - xx[8] * (xx[113] * xx[68] + xx[112] * xx[73]);
  xx[138] = - (xx[8] * (xx[113] * xx[72] + xx[112] * xx[66]) - xx[106]);
  pm_math_Vector3_cross_ra(xx + 63, xx + 136, xx + 174);
  pm_math_Vector3_cross_ra(xx + 63, xx + 174, xx + 177);
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 177, xx + 63);
  pm_math_Matrix3x3_xform_ra(xx + 155, xx + 63, xx + 174);
  xx[112] = xx[167] - xx[118] - xx[122] - xx[194];
  xx[113] = state[8] + xx[1];
  if (xx[2] < xx[113])
    xx[113] = xx[2];
  xx[127] = - (xx[113] / xx[4]);
  if (xx[5] < xx[127])
    xx[127] = xx[5];
  xx[133] = xx[9] * state[9];
  xx[134] = xx[127] * xx[127] * (xx[7] - xx[8] * xx[127]) * ((- xx[113] == xx[2]
    ? xx[2] : - xx[133]) - xx[11] * xx[113]);
  if (xx[2] > xx[134])
    xx[134] = xx[2];
  xx[113] = state[8] - xx[1];
  if (xx[2] > xx[113])
    xx[113] = xx[2];
  xx[127] = xx[113] / xx[4];
  if (xx[5] < xx[127])
    xx[127] = xx[5];
  xx[139] = xx[127] * xx[127] * (xx[7] - xx[8] * xx[127]) * (xx[11] * xx[113] +
    (xx[113] == xx[2] ? xx[2] : xx[133]));
  if (xx[2] > xx[139])
    xx[139] = xx[2];
  xx[113] = xx[171] - xx[122] - xx[118] - xx[198];
  xx[127] = xx[172] - xx[123] - xx[121] - xx[199];
  xx[133] = xx[88] + xx[147] + xx[151] + xx[98] * xx[113] - xx[89] * xx[127] +
    xx[176];
  xx[155] = xx[25] + xx[173] - xx[124] - xx[124] - xx[200];
  memcpy(xx + 156, xx + 155, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 156, 1, xx + 157);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/Manipulator/la_5_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[157] = (input[5] - xx[0] * state[9] + xx[134] - xx[139] - xx[133]) / xx[156];
  xx[88] = xx[168] - xx[119] - xx[117] - xx[195];
  xx[116] = xx[25] + xx[169] - xx[120] - xx[120] - xx[196];
  xx[117] = xx[170] - xx[121] - xx[123] - xx[197];
  xx[118] = xx[86] + xx[145] + xx[149] + xx[98] * xx[83] - xx[89] * xx[78] + xx
    [174] + xx[112] * xx[157];
  xx[119] = xx[87] + xx[146] + xx[150] + xx[98] * xx[88] - xx[89] * xx[116] +
    xx[175] + xx[117] * xx[157];
  xx[120] = xx[133] + xx[155] * xx[157];
  pm_math_Quaternion_xform_ra(xx + 74, xx + 118, xx + 121);
  xx[158] = xx[106] + xx[183];
  xx[159] = xx[184];
  xx[160] = xx[185];
  xx[161] = xx[186];
  xx[162] = xx[106] + xx[187];
  xx[163] = xx[188];
  xx[164] = xx[189];
  xx[165] = xx[190];
  xx[166] = xx[106] + xx[191];
  pm_math_Matrix3x3_xform_ra(xx + 158, xx + 63, xx + 118);
  xx[145] = xx[152] + xx[98] * xx[94] - xx[89] * xx[97] + xx[118] + xx[103] *
    xx[157];
  xx[146] = xx[153] + xx[98] * xx[95] - xx[89] * xx[100] + xx[119] + xx[104] *
    xx[157];
  xx[147] = xx[154] + xx[98] * xx[96] - xx[89] * xx[102] + xx[120] + xx[105] *
    xx[157];
  pm_math_Quaternion_xform_ra(xx + 74, xx + 145, xx + 118);
  pm_math_Vector3_cross_ra(xx + 136, xx + 118, xx + 145);
  xx[86] = state[7] * xx[61];
  xx[87] = xx[68] * xx[68];
  xx[124] = xx[72] * xx[66];
  xx[133] = xx[68] * xx[73];
  xx[134] = xx[73] * xx[72];
  xx[139] = xx[68] * xx[66];
  xx[149] = xx[73] * xx[66];
  xx[150] = xx[68] * xx[72];
  xx[158] = xx[8] * (xx[87] + xx[72] * xx[72]) - xx[5];
  xx[159] = - (xx[8] * (xx[124] + xx[133]));
  xx[160] = xx[8] * (xx[134] - xx[139]);
  xx[161] = xx[8] * (xx[133] - xx[124]);
  xx[162] = xx[8] * (xx[87] + xx[66] * xx[66]) - xx[5];
  xx[163] = - (xx[8] * (xx[149] + xx[150]));
  xx[164] = xx[8] * (xx[134] + xx[139]);
  xx[165] = xx[8] * (xx[150] - xx[149]);
  xx[166] = xx[8] * (xx[87] + xx[73] * xx[73]) - xx[5];
  xx[66] = xx[112] / xx[156];
  xx[68] = xx[117] * xx[66];
  xx[72] = xx[155] * xx[66];
  xx[73] = xx[117] / xx[156];
  xx[87] = xx[155] * xx[73];
  xx[124] = xx[155] / xx[156];
  xx[167] = xx[83] - xx[112] * xx[66];
  xx[168] = xx[78] - xx[68];
  xx[169] = xx[112] - xx[72];
  xx[170] = xx[88] - xx[68];
  xx[171] = xx[116] - xx[117] * xx[73];
  xx[172] = xx[117] - xx[87];
  xx[173] = xx[113] - xx[72];
  xx[174] = xx[127] - xx[87];
  xx[175] = xx[155] - xx[155] * xx[124];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 167, xx + 158, xx + 192);
  pm_math_Matrix3x3_compose_ra(xx + 158, xx + 192, xx + 167);
  xx[68] = xx[103] / xx[156];
  xx[72] = xx[104] / xx[156];
  xx[78] = xx[105] / xx[156];
  xx[192] = xx[94] - xx[112] * xx[68];
  xx[193] = xx[95] - xx[112] * xx[72];
  xx[194] = xx[96] - xx[112] * xx[78];
  xx[195] = xx[97] - xx[117] * xx[68];
  xx[196] = xx[100] - xx[117] * xx[72];
  xx[197] = xx[102] - xx[117] * xx[78];
  xx[198] = xx[103] - xx[155] * xx[68];
  xx[199] = xx[104] - xx[155] * xx[72];
  xx[200] = xx[105] - xx[155] * xx[78];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 192, xx + 158, xx + 201);
  pm_math_Matrix3x3_compose_ra(xx + 158, xx + 201, xx + 192);
  pm_math_Matrix3x3_postCross_ra(xx + 192, xx + 136, xx + 201);
  xx[83] = xx[104] * xx[68];
  xx[87] = xx[105] * xx[68];
  xx[88] = xx[105] * xx[72];
  xx[210] = xx[183] - xx[103] * xx[68] + xx[106];
  xx[211] = xx[184] - xx[83];
  xx[212] = xx[185] - xx[87];
  xx[213] = xx[186] - xx[83];
  xx[214] = xx[187] - xx[104] * xx[72] + xx[106];
  xx[215] = xx[188] - xx[88];
  xx[216] = xx[189] - xx[87];
  xx[217] = xx[190] - xx[88];
  xx[218] = xx[191] - xx[105] * xx[78] + xx[106];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 210, xx + 158, xx + 176);
  pm_math_Matrix3x3_compose_ra(xx + 158, xx + 176, xx + 210);
  pm_math_Matrix3x3_postCross_ra(xx + 210, xx + 136, xx + 158);
  pm_math_Matrix3x3_preCross_ra(xx + 158, xx + 136, xx + 176);
  xx[83] = xx[25] + xx[167] - xx[201] - xx[201] - xx[176];
  xx[87] = state[7] * xx[60];
  xx[60] = xx[168] - xx[202] - xx[204] - xx[177];
  xx[88] = xx[192] - xx[158];
  xx[94] = xx[193] - xx[161];
  xx[95] = xx[194] - xx[164];
  xx[96] = xx[195] - xx[159];
  xx[97] = xx[196] - xx[162];
  xx[100] = xx[197] - xx[165];
  xx[102] = xx[198] - xx[160];
  xx[103] = xx[199] - xx[163];
  xx[104] = xx[200] - xx[166];
  xx[158] = xx[88];
  xx[159] = xx[94];
  xx[160] = xx[95];
  xx[161] = xx[96];
  xx[162] = xx[97];
  xx[163] = xx[100];
  xx[164] = xx[102];
  xx[165] = xx[103];
  xx[166] = xx[104];
  xx[105] = xx[125] * xx[34];
  xx[112] = xx[125] * xx[32];
  xx[149] = - (xx[8] * (xx[105] * xx[54] - xx[112] * xx[56]));
  xx[150] = - (xx[125] - xx[8] * (xx[105] * xx[34] + xx[112] * xx[32]));
  xx[151] = 0.06 + xx[8] * (xx[112] * xx[54] + xx[105] * xx[56]);
  pm_math_Vector3_cross_ra(xx + 51, xx + 149, xx + 152);
  pm_math_Vector3_cross_ra(xx + 51, xx + 152, xx + 185);
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 185, xx + 51);
  xx[105] = xx[125] * state[7];
  xx[112] = xx[105] * (xx[62] + xx[55]) + xx[52];
  xx[52] = xx[53] - xx[105] * (xx[61] + xx[61]);
  xx[152] = xx[51];
  xx[153] = xx[112];
  xx[154] = xx[52];
  pm_math_Matrix3x3_xform_ra(xx + 158, xx + 152, xx + 185);
  xx[53] = xx[169] - xx[203] - xx[207] - xx[178];
  xx[55] = xx[53] + xx[125] * xx[88];
  xx[61] = state[6] + xx[1];
  if (xx[2] < xx[61])
    xx[61] = xx[2];
  xx[62] = - (xx[61] / xx[4]);
  if (xx[5] < xx[62])
    xx[62] = xx[5];
  xx[105] = xx[9] * state[7];
  xx[113] = xx[62] * xx[62] * (xx[7] - xx[8] * xx[62]) * ((- xx[61] == xx[2] ?
    xx[2] : - xx[105]) - xx[11] * xx[61]);
  if (xx[2] > xx[113])
    xx[113] = xx[2];
  xx[61] = state[6] - xx[1];
  if (xx[2] > xx[61])
    xx[61] = xx[2];
  xx[62] = xx[61] / xx[4];
  if (xx[5] < xx[62])
    xx[62] = xx[5];
  xx[116] = xx[62] * xx[62] * (xx[7] - xx[8] * xx[62]) * (xx[11] * xx[61] + (xx
    [61] == xx[2] ? xx[2] : xx[105]));
  if (xx[2] > xx[116])
    xx[116] = xx[2];
  xx[61] = xx[173] - xx[207] - xx[203] - xx[182];
  xx[62] = xx[174] - xx[208] - xx[206] - xx[183];
  xx[105] = xx[71] + xx[123] + xx[147] + xx[86] * xx[61] - xx[87] * xx[62] + xx
    [187];
  xx[117] = xx[106] + xx[210];
  xx[158] = xx[117];
  xx[159] = xx[211];
  xx[160] = xx[212];
  xx[161] = xx[213];
  xx[162] = xx[106] + xx[214];
  xx[163] = xx[215];
  xx[164] = xx[216];
  xx[165] = xx[217];
  xx[166] = xx[106] + xx[218];
  pm_math_Matrix3x3_xform_ra(xx + 158, xx + 152, xx + 188);
  xx[127] = xx[118] + xx[86] * xx[88] - xx[87] * xx[96] + xx[188];
  xx[118] = xx[25] + xx[175] - xx[209] - xx[209] - xx[184];
  xx[133] = xx[118] + xx[125] * xx[102];
  xx[134] = xx[102] + xx[125] * xx[117];
  xx[139] = xx[133] + xx[125] * xx[134];
  ii[0] = factorSymmetricPosDef(xx + 139, 1, xx + 152);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/Manipulator/la_4_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[152] = (input[4] - xx[0] * state[7] + xx[113] - xx[116] - (xx[105] + xx[125]
              * xx[127])) / xx[139];
  xx[71] = xx[170] - xx[204] - xx[202] - xx[179];
  xx[113] = xx[25] + xx[171] - xx[205] - xx[205] - xx[180];
  xx[116] = xx[172] - xx[206] - xx[208] - xx[181];
  xx[123] = xx[116] + xx[125] * xx[96];
  xx[153] = xx[69] + xx[121] + xx[145] + xx[86] * xx[83] - xx[87] * xx[60] + xx
    [185] + xx[55] * xx[152];
  xx[154] = xx[70] + xx[122] + xx[146] + xx[86] * xx[71] - xx[87] * xx[113] +
    xx[186] + xx[123] * xx[152];
  xx[155] = xx[105] + xx[133] * xx[152];
  pm_math_Quaternion_xform_ra(xx + 39, xx + 153, xx + 145);
  xx[69] = xx[103] + xx[125] * xx[213];
  xx[70] = xx[104] + xx[125] * xx[216];
  xx[153] = xx[127] + xx[134] * xx[152];
  xx[154] = xx[119] + xx[86] * xx[94] - xx[87] * xx[97] + xx[189] + xx[69] * xx
    [152];
  xx[155] = xx[120] + xx[86] * xx[95] - xx[87] * xx[100] + xx[190] + xx[70] *
    xx[152];
  pm_math_Quaternion_xform_ra(xx + 39, xx + 153, xx + 119);
  pm_math_Vector3_cross_ra(xx + 149, xx + 119, xx + 153);
  xx[105] = state[5] * xx[50];
  xx[50] = xx[54] * xx[54];
  xx[122] = xx[54] * xx[34];
  xx[127] = xx[56] * xx[32];
  xx[156] = xx[32] * xx[34];
  xx[158] = xx[54] * xx[56];
  xx[159] = xx[54] * xx[32];
  xx[54] = xx[56] * xx[34];
  xx[160] = xx[8] * (xx[50] + xx[32] * xx[32]) - xx[5];
  xx[161] = xx[8] * (xx[122] - xx[127]);
  xx[162] = xx[8] * (xx[156] + xx[158]);
  xx[163] = - (xx[8] * (xx[127] + xx[122]));
  xx[164] = xx[8] * (xx[50] + xx[56] * xx[56]) - xx[5];
  xx[165] = xx[8] * (xx[159] - xx[54]);
  xx[166] = xx[8] * (xx[156] - xx[158]);
  xx[167] = - (xx[8] * (xx[54] + xx[159]));
  xx[168] = xx[8] * (xx[50] + xx[34] * xx[34]) - xx[5];
  xx[32] = xx[55] / xx[139];
  xx[34] = xx[123] * xx[32];
  xx[50] = xx[133] * xx[32];
  xx[54] = xx[123] / xx[139];
  xx[56] = xx[133] * xx[54];
  xx[122] = xx[133] / xx[139];
  xx[169] = xx[83] - xx[55] * xx[32];
  xx[170] = xx[60] - xx[34];
  xx[171] = xx[53] - xx[50];
  xx[172] = xx[71] - xx[34];
  xx[173] = xx[113] - xx[123] * xx[54];
  xx[174] = xx[116] - xx[56];
  xx[175] = xx[61] - xx[50];
  xx[176] = xx[62] - xx[56];
  xx[177] = xx[118] - xx[133] * xx[122];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 169, xx + 160, xx + 178);
  pm_math_Matrix3x3_compose_ra(xx + 160, xx + 178, xx + 169);
  xx[34] = xx[134] / xx[139];
  xx[50] = xx[69] / xx[139];
  xx[53] = xx[70] / xx[139];
  xx[178] = xx[88] - xx[55] * xx[34];
  xx[179] = xx[94] - xx[55] * xx[50];
  xx[180] = xx[95] - xx[55] * xx[53];
  xx[181] = xx[96] - xx[123] * xx[34];
  xx[182] = xx[97] - xx[123] * xx[50];
  xx[183] = xx[100] - xx[123] * xx[53];
  xx[184] = xx[102] - xx[133] * xx[34];
  xx[185] = xx[103] - xx[133] * xx[50];
  xx[186] = xx[104] - xx[133] * xx[53];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 178, xx + 160, xx + 187);
  pm_math_Matrix3x3_compose_ra(xx + 160, xx + 187, xx + 178);
  pm_math_Matrix3x3_postCross_ra(xx + 178, xx + 149, xx + 187);
  xx[55] = xx[69] * xx[34];
  xx[56] = xx[70] * xx[34];
  xx[60] = xx[70] * xx[50];
  xx[196] = xx[117] - xx[134] * xx[34];
  xx[197] = xx[211] - xx[55];
  xx[198] = xx[212] - xx[56];
  xx[199] = xx[213] - xx[55];
  xx[200] = xx[214] - xx[69] * xx[50] + xx[106];
  xx[201] = xx[215] - xx[60];
  xx[202] = xx[216] - xx[56];
  xx[203] = xx[217] - xx[60];
  xx[204] = xx[218] - xx[70] * xx[53] + xx[106];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 196, xx + 160, xx + 205);
  pm_math_Matrix3x3_compose_ra(xx + 160, xx + 205, xx + 196);
  pm_math_Matrix3x3_postCross_ra(xx + 196, xx + 149, xx + 158);
  pm_math_Matrix3x3_preCross_ra(xx + 158, xx + 149, xx + 205);
  xx[55] = xx[25] + xx[169] - xx[187] - xx[187] - xx[205];
  xx[56] = state[5] * xx[49];
  xx[49] = xx[170] - xx[188] - xx[190] - xx[206];
  xx[60] = xx[178] - xx[158];
  xx[61] = xx[179] - xx[161];
  xx[62] = xx[180] - xx[164];
  xx[69] = xx[181] - xx[159];
  xx[70] = xx[182] - xx[162];
  xx[71] = xx[183] - xx[165];
  xx[83] = xx[184] - xx[160];
  xx[88] = xx[185] - xx[163];
  xx[94] = xx[186] - xx[166];
  xx[158] = xx[60];
  xx[159] = xx[61];
  xx[160] = xx[62];
  xx[161] = xx[69];
  xx[162] = xx[70];
  xx[163] = xx[71];
  xx[164] = xx[83];
  xx[165] = xx[88];
  xx[166] = xx[94];
  xx[95] = 0.21;
  xx[96] = xx[95] * xx[44];
  xx[97] = xx[95] * xx[43];
  xx[100] = 0.105;
  xx[102] = xx[8] * (xx[96] * xx[33] + xx[97] * xx[38]) - xx[100];
  xx[103] = - (xx[8] * (xx[96] * xx[38] - xx[97] * xx[33]));
  xx[104] = - (xx[8] * (xx[97] * xx[43] + xx[96] * xx[44]) - xx[95]);
  pm_math_Vector3_cross_ra(xx + 29, xx + 102, xx + 95);
  pm_math_Vector3_cross_ra(xx + 29, xx + 95, xx + 116);
  pm_math_Quaternion_inverseXform_ra(xx + 45, xx + 116, xx + 29);
  pm_math_Matrix3x3_xform_ra(xx + 158, xx + 29, xx + 95);
  xx[113] = xx[171] - xx[189] - xx[193] - xx[207];
  xx[116] = state[4] + xx[1];
  if (xx[2] < xx[116])
    xx[116] = xx[2];
  xx[117] = - (xx[116] / xx[4]);
  if (xx[5] < xx[117])
    xx[117] = xx[5];
  xx[118] = xx[9] * state[5];
  xx[123] = xx[117] * xx[117] * (xx[7] - xx[8] * xx[117]) * ((- xx[116] == xx[2]
    ? xx[2] : - xx[118]) - xx[11] * xx[116]);
  if (xx[2] > xx[123])
    xx[123] = xx[2];
  xx[116] = state[4] - xx[1];
  if (xx[2] > xx[116])
    xx[116] = xx[2];
  xx[117] = xx[116] / xx[4];
  if (xx[5] < xx[117])
    xx[117] = xx[5];
  xx[127] = xx[117] * xx[117] * (xx[7] - xx[8] * xx[117]) * (xx[11] * xx[116] +
    (xx[116] == xx[2] ? xx[2] : xx[118]));
  if (xx[2] > xx[127])
    xx[127] = xx[2];
  xx[116] = xx[175] - xx[193] - xx[189] - xx[211];
  xx[117] = xx[176] - xx[194] - xx[192] - xx[212];
  xx[118] = xx[59] + xx[147] + xx[155] + xx[105] * xx[116] - xx[56] * xx[117] +
    xx[97];
  xx[133] = xx[25] + xx[177] - xx[195] - xx[195] - xx[213];
  memcpy(xx + 134, xx + 133, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 134, 1, xx + 139);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/Manipulator/la_3_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[139] = (input[3] - xx[0] * state[5] + xx[123] - xx[127] - xx[118]) / xx[134];
  xx[59] = xx[172] - xx[190] - xx[188] - xx[208];
  xx[123] = xx[25] + xx[173] - xx[191] - xx[191] - xx[209];
  xx[97] = xx[174] - xx[192] - xx[194] - xx[210];
  xx[158] = xx[57] + xx[145] + xx[153] + xx[105] * xx[55] - xx[56] * xx[49] +
    xx[95] + xx[113] * xx[139];
  xx[159] = xx[58] + xx[146] + xx[154] + xx[105] * xx[59] - xx[56] * xx[123] +
    xx[96] + xx[97] * xx[139];
  xx[160] = xx[118] + xx[133] * xx[139];
  pm_math_Quaternion_xform_ra(xx + 45, xx + 158, xx + 145);
  xx[158] = xx[106] + xx[196];
  xx[159] = xx[197];
  xx[160] = xx[198];
  xx[161] = xx[199];
  xx[162] = xx[106] + xx[200];
  xx[163] = xx[201];
  xx[164] = xx[202];
  xx[165] = xx[203];
  xx[166] = xx[106] + xx[204];
  pm_math_Matrix3x3_xform_ra(xx + 158, xx + 29, xx + 153);
  xx[158] = xx[119] + xx[105] * xx[60] - xx[56] * xx[69] + xx[153] + xx[83] *
    xx[139];
  xx[159] = xx[120] + xx[105] * xx[61] - xx[56] * xx[70] + xx[154] + xx[88] *
    xx[139];
  xx[160] = xx[121] + xx[105] * xx[62] - xx[56] * xx[71] + xx[155] + xx[94] *
    xx[139];
  pm_math_Quaternion_xform_ra(xx + 45, xx + 158, xx + 118);
  pm_math_Vector3_cross_ra(xx + 102, xx + 118, xx + 153);
  xx[57] = xx[27] * state[3];
  xx[27] = xx[33] * xx[33];
  xx[58] = xx[33] * xx[38];
  xx[95] = xx[44] * xx[43];
  xx[96] = xx[38] * xx[43];
  xx[121] = xx[33] * xx[44];
  xx[127] = xx[33] * xx[43];
  xx[33] = xx[44] * xx[38];
  xx[158] = xx[8] * (xx[27] + xx[43] * xx[43]) - xx[5];
  xx[159] = xx[8] * (xx[58] - xx[95]);
  xx[160] = xx[8] * (xx[96] + xx[121]);
  xx[161] = - (xx[8] * (xx[95] + xx[58]));
  xx[162] = xx[8] * (xx[27] + xx[44] * xx[44]) - xx[5];
  xx[163] = xx[8] * (xx[127] - xx[33]);
  xx[164] = xx[8] * (xx[96] - xx[121]);
  xx[165] = - (xx[8] * (xx[33] + xx[127]));
  xx[166] = xx[8] * (xx[27] + xx[38] * xx[38]) - xx[5];
  xx[27] = xx[113] / xx[134];
  xx[33] = xx[97] * xx[27];
  xx[38] = xx[133] * xx[27];
  xx[43] = xx[97] / xx[134];
  xx[44] = xx[133] * xx[43];
  xx[58] = xx[133] / xx[134];
  xx[167] = xx[55] - xx[113] * xx[27];
  xx[168] = xx[49] - xx[33];
  xx[169] = xx[113] - xx[38];
  xx[170] = xx[59] - xx[33];
  xx[171] = xx[123] - xx[97] * xx[43];
  xx[172] = xx[97] - xx[44];
  xx[173] = xx[116] - xx[38];
  xx[174] = xx[117] - xx[44];
  xx[175] = xx[133] - xx[133] * xx[58];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 167, xx + 158, xx + 176);
  pm_math_Matrix3x3_compose_ra(xx + 158, xx + 176, xx + 167);
  xx[33] = xx[83] / xx[134];
  xx[38] = xx[88] / xx[134];
  xx[44] = xx[94] / xx[134];
  xx[176] = xx[60] - xx[113] * xx[33];
  xx[177] = xx[61] - xx[113] * xx[38];
  xx[178] = xx[62] - xx[113] * xx[44];
  xx[179] = xx[69] - xx[97] * xx[33];
  xx[180] = xx[70] - xx[97] * xx[38];
  xx[181] = xx[71] - xx[97] * xx[44];
  xx[182] = xx[83] - xx[133] * xx[33];
  xx[183] = xx[88] - xx[133] * xx[38];
  xx[184] = xx[94] - xx[133] * xx[44];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 176, xx + 158, xx + 185);
  pm_math_Matrix3x3_compose_ra(xx + 158, xx + 185, xx + 176);
  pm_math_Matrix3x3_postCross_ra(xx + 176, xx + 102, xx + 185);
  xx[49] = xx[88] * xx[33];
  xx[55] = xx[94] * xx[33];
  xx[59] = xx[94] * xx[38];
  xx[205] = xx[196] - xx[83] * xx[33] + xx[106];
  xx[206] = xx[197] - xx[49];
  xx[207] = xx[198] - xx[55];
  xx[208] = xx[199] - xx[49];
  xx[209] = xx[200] - xx[88] * xx[38] + xx[106];
  xx[210] = xx[201] - xx[59];
  xx[211] = xx[202] - xx[55];
  xx[212] = xx[203] - xx[59];
  xx[213] = xx[204] - xx[94] * xx[44] + xx[106];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 205, xx + 158, xx + 194);
  pm_math_Matrix3x3_compose_ra(xx + 158, xx + 194, xx + 203);
  pm_math_Matrix3x3_postCross_ra(xx + 203, xx + 102, xx + 158);
  pm_math_Matrix3x3_preCross_ra(xx + 158, xx + 102, xx + 194);
  xx[49] = xx[25] + xx[167] - xx[185] - xx[185] - xx[194];
  xx[55] = xx[26] * state[3];
  xx[59] = xx[168] - xx[186] - xx[188] - xx[195];
  xx[60] = xx[176] - xx[158];
  xx[61] = xx[177] - xx[161];
  xx[62] = xx[178] - xx[164];
  xx[69] = xx[179] - xx[159];
  xx[70] = xx[180] - xx[162];
  xx[71] = xx[181] - xx[165];
  xx[83] = xx[182] - xx[160];
  xx[88] = xx[183] - xx[163];
  xx[94] = xx[184] - xx[166];
  xx[158] = xx[60];
  xx[159] = xx[61];
  xx[160] = xx[62];
  xx[161] = xx[69];
  xx[162] = xx[70];
  xx[163] = xx[71];
  xx[164] = xx[83];
  xx[165] = xx[88];
  xx[166] = xx[94];
  xx[95] = xx[100] * xx[10];
  xx[96] = xx[100] * xx[19];
  xx[97] = xx[8] * (xx[95] * xx[17] - xx[96] * xx[15]);
  xx[106] = xx[97] * state[1] * state[1];
  xx[113] = xx[19] * xx[106];
  xx[176] = xx[14];
  xx[177] = xx[10];
  xx[178] = xx[18];
  xx[14] = 0.02 - (xx[8] * (xx[95] * xx[10] + xx[96] * xx[19]) - xx[100]);
  xx[18] = xx[14] * state[1] * state[1];
  xx[116] = xx[19] * xx[18];
  xx[117] = xx[10] * xx[18] - xx[17] * xx[106];
  xx[179] = xx[113];
  xx[180] = xx[116];
  xx[181] = xx[117];
  pm_math_Vector3_cross_ra(xx + 176, xx + 179, xx + 182);
  xx[121] = xx[100] * state[3];
  xx[123] = xx[8] * (xx[15] * xx[113] + xx[182]) - xx[18] - xx[121] * (xx[28] +
    xx[24]);
  xx[18] = xx[106] + xx[8] * (xx[15] * xx[116] + xx[183]);
  xx[24] = xx[121] * (xx[26] + xx[26]) + xx[8] * (xx[184] + xx[15] * xx[117]);
  xx[176] = xx[123];
  xx[177] = xx[18];
  xx[178] = xx[24];
  pm_math_Matrix3x3_xform_ra(xx + 158, xx + 176, xx + 179);
  xx[26] = xx[169] - xx[187] - xx[191] - xx[196];
  xx[28] = xx[26] + xx[100] * xx[61];
  xx[106] = state[2] + xx[1];
  if (xx[2] < xx[106])
    xx[106] = xx[2];
  xx[113] = - (xx[106] / xx[4]);
  if (xx[5] < xx[113])
    xx[113] = xx[5];
  xx[116] = xx[9] * state[3];
  xx[9] = xx[113] * xx[113] * (xx[7] - xx[8] * xx[113]) * ((- xx[106] == xx[2] ?
    xx[2] : - xx[116]) - xx[11] * xx[106]);
  if (xx[2] > xx[9])
    xx[9] = xx[2];
  xx[106] = state[2] - xx[1];
  if (xx[2] > xx[106])
    xx[106] = xx[2];
  xx[1] = xx[106] / xx[4];
  if (xx[5] < xx[1])
    xx[1] = xx[5];
  xx[4] = xx[1] * xx[1] * (xx[7] - xx[8] * xx[1]) * (xx[11] * xx[106] + (xx[106]
    == xx[2] ? xx[2] : xx[116]));
  if (xx[2] > xx[4])
    xx[4] = xx[2];
  xx[1] = xx[173] - xx[191] - xx[187] - xx[200];
  xx[7] = xx[174] - xx[192] - xx[190] - xx[201];
  xx[11] = xx[37] + xx[147] + xx[155] + xx[57] * xx[1] - xx[55] * xx[7] + xx[181];
  xx[106] = 0.2;
  xx[113] = xx[106] + xx[207];
  xx[158] = xx[106] + xx[203];
  xx[159] = xx[204];
  xx[160] = xx[205];
  xx[161] = xx[206];
  xx[162] = xx[113];
  xx[163] = xx[208];
  xx[164] = xx[209];
  xx[165] = xx[210];
  xx[166] = xx[106] + xx[211];
  pm_math_Matrix3x3_xform_ra(xx + 158, xx + 176, xx + 182);
  xx[116] = xx[119] + xx[57] * xx[61] - xx[55] * xx[70] + xx[183];
  xx[117] = xx[25] + xx[175] - xx[193] - xx[193] - xx[202];
  xx[119] = xx[117] + xx[100] * xx[88];
  xx[121] = xx[88] + xx[100] * xx[113];
  xx[127] = xx[119] + xx[100] * xx[121];
  ii[0] = factorSymmetricPosDef(xx + 127, 1, xx + 133);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/Manipulator/la_2_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[133] = (input[2] - xx[0] * state[3] + xx[9] - xx[4] - (xx[11] + xx[100] *
              xx[116])) / xx[127];
  xx[4] = xx[170] - xx[188] - xx[186] - xx[197];
  xx[9] = xx[25] + xx[171] - xx[189] - xx[189] - xx[198];
  xx[37] = xx[172] - xx[190] - xx[192] - xx[199];
  xx[134] = xx[37] + xx[100] * xx[70];
  xx[158] = xx[35] + xx[145] + xx[153] + xx[57] * xx[49] - xx[55] * xx[59] + xx
    [179] + xx[28] * xx[133];
  xx[159] = xx[36] + xx[146] + xx[154] + xx[57] * xx[4] - xx[55] * xx[9] + xx
    [180] + xx[134] * xx[133];
  xx[160] = xx[11] + xx[119] * xx[133];
  pm_math_Quaternion_xform_ra(xx + 20, xx + 158, xx + 145);
  xx[153] = xx[14];
  xx[154] = - xx[97];
  xx[155] = xx[8] * (xx[95] * xx[15] + xx[96] * xx[17]) - 0.015;
  xx[11] = xx[83] + xx[100] * xx[204];
  xx[35] = xx[94] + xx[100] * xx[210];
  xx[158] = xx[118] + xx[57] * xx[60] - xx[55] * xx[69] + xx[182] + xx[11] * xx
    [133];
  xx[159] = xx[116] + xx[121] * xx[133];
  xx[160] = xx[120] + xx[57] * xx[62] - xx[55] * xx[71] + xx[184] + xx[35] * xx
    [133];
  pm_math_Quaternion_xform_ra(xx + 20, xx + 158, xx + 161);
  pm_math_Vector3_cross_ra(xx + 153, xx + 161, xx + 158);
  xx[36] = 0.0403;
  xx[95] = xx[36] * state[1] * state[1];
  xx[96] = xx[15] * xx[15];
  xx[116] = xx[17] * xx[10];
  xx[118] = xx[15] * xx[19];
  xx[120] = xx[19] * xx[17];
  xx[145] = xx[15] * xx[10];
  xx[146] = xx[19] * xx[10];
  xx[156] = xx[15] * xx[17];
  xx[163] = xx[8] * (xx[96] + xx[17] * xx[17]) - xx[5];
  xx[164] = - (xx[8] * (xx[116] + xx[118]));
  xx[165] = xx[8] * (xx[120] - xx[145]);
  xx[166] = xx[8] * (xx[118] - xx[116]);
  xx[167] = xx[8] * (xx[96] + xx[10] * xx[10]) - xx[5];
  xx[168] = - (xx[8] * (xx[146] + xx[156]));
  xx[169] = xx[8] * (xx[120] + xx[145]);
  xx[170] = xx[8] * (xx[156] - xx[146]);
  xx[171] = xx[8] * (xx[96] + xx[19] * xx[19]) - xx[5];
  xx[5] = xx[11] / xx[127];
  xx[96] = xx[121] / xx[127];
  xx[116] = xx[35] / xx[127];
  xx[172] = xx[60] - xx[28] * xx[5];
  xx[173] = xx[61] - xx[28] * xx[96];
  xx[174] = xx[62] - xx[28] * xx[116];
  xx[175] = xx[69] - xx[134] * xx[5];
  xx[176] = xx[70] - xx[134] * xx[96];
  xx[177] = xx[71] - xx[134] * xx[116];
  xx[178] = xx[83] - xx[119] * xx[5];
  xx[179] = xx[88] - xx[119] * xx[96];
  xx[180] = xx[94] - xx[119] * xx[116];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 172, xx + 163, xx + 181);
  pm_math_Matrix3x3_compose_ra(xx + 163, xx + 181, xx + 172);
  xx[60] = xx[121] * xx[5];
  xx[61] = xx[35] * xx[5];
  xx[62] = xx[35] * xx[96];
  xx[181] = xx[203] - xx[11] * xx[5] + xx[106];
  xx[182] = xx[204] - xx[60];
  xx[183] = xx[205] - xx[61];
  xx[184] = xx[206] - xx[60];
  xx[185] = xx[113] - xx[121] * xx[96];
  xx[186] = xx[208] - xx[62];
  xx[187] = xx[209] - xx[61];
  xx[188] = xx[210] - xx[62];
  xx[189] = xx[211] - xx[35] * xx[116] + xx[106];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 181, xx + 163, xx + 190);
  pm_math_Matrix3x3_compose_ra(xx + 163, xx + 190, xx + 181);
  pm_math_Matrix3x3_postCross_ra(xx + 181, xx + 153, xx + 190);
  xx[11] = xx[178] - xx[192];
  xx[35] = xx[28] / xx[127];
  xx[60] = xx[134] * xx[35];
  xx[61] = xx[119] * xx[35];
  xx[62] = xx[134] / xx[127];
  xx[69] = xx[119] * xx[62];
  xx[70] = xx[119] / xx[127];
  xx[199] = xx[49] - xx[28] * xx[35];
  xx[200] = xx[59] - xx[60];
  xx[201] = xx[26] - xx[61];
  xx[202] = xx[4] - xx[60];
  xx[203] = xx[9] - xx[134] * xx[62];
  xx[204] = xx[37] - xx[69];
  xx[205] = xx[1] - xx[61];
  xx[206] = xx[7] - xx[69];
  xx[207] = xx[117] - xx[119] * xx[70];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 199, xx + 163, xx + 208);
  pm_math_Matrix3x3_compose_ra(xx + 163, xx + 208, xx + 199);
  pm_math_Matrix3x3_postCross_ra(xx + 172, xx + 153, xx + 163);
  pm_math_Matrix3x3_preCross_ra(xx + 190, xx + 153, xx + 208);
  xx[1] = xx[179] - xx[195];
  xx[4] = xx[1] + xx[36] * (0.35 + xx[185]);
  xx[7] = xx[207] - xx[171] - xx[171] - xx[216] + xx[36] * xx[1] + xx[36] * xx[4]
    + xx[25];
  ii[0] = factorSymmetricPosDef(xx + 7, 1, xx + 1);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'left_arm_simul/Manipulator/la_1_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[59] = (xx[11] + xx[36] * xx[182]) / xx[7];
  xx[60] = xx[4] / xx[7];
  xx[61] = (xx[180] - xx[198] + xx[36] * xx[188]) / xx[7];
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
  xx[117] = xx[28];
  xx[118] = xx[6];
  xx[119] = xx[9];
  xx[1] = (input[1] - xx[0] * state[1] + xx[12] - xx[13] - (xx[147] + xx[160] -
            xx[95] * xx[11] + xx[36] * (xx[162] - xx[95] * xx[184]))) / xx[7] -
    pm_math_Vector3_dot_ra(xx + 59, xx + 117);
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
  xx[71] = xx[116];
  xx[116] = xx[28] - xx[95] + xx[97] * xx[1];
  xx[117] = xx[36] * xx[1] + xx[6] + xx[14] * xx[1];
  xx[118] = xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 20, xx + 116, xx + 8);
  xx[0] = xx[133] - (pm_math_Vector3_dot_ra(xx + 11, xx + 59) +
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
  xx[19] = xx[8] + xx[123] + xx[25];
  xx[20] = xx[9] + xx[100] * xx[0] + xx[18] + xx[26];
  xx[21] = xx[10] + xx[24] + xx[27];
  pm_math_Quaternion_inverseXform_ra(xx + 45, xx + 19, xx + 6);
  xx[9] = xx[139] - (pm_math_Vector3_dot_ra(xx + 11, xx + 3) +
                     pm_math_Vector3_dot_ra(xx + 14, xx + 6));
  xx[10] = xx[32];
  xx[11] = xx[54];
  xx[12] = xx[122];
  xx[13] = xx[3] + xx[105];
  xx[14] = xx[4] - xx[56];
  xx[15] = xx[5] + xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 13, xx + 3);
  xx[16] = xx[34];
  xx[17] = xx[50];
  xx[18] = xx[53];
  pm_math_Vector3_cross_ra(xx + 13, xx + 149, xx + 19);
  xx[13] = xx[6] + xx[29] + xx[19];
  xx[14] = xx[7] + xx[30] + xx[20];
  xx[15] = xx[8] + xx[31] + xx[21];
  pm_math_Quaternion_inverseXform_ra(xx + 39, xx + 13, xx + 6);
  xx[13] = xx[152] - (pm_math_Vector3_dot_ra(xx + 10, xx + 3) +
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
  pm_math_Vector3_cross_ra(xx + 14, xx + 136, xx + 20);
  xx[14] = xx[6] + xx[125] * xx[13] + xx[51] + xx[20];
  xx[15] = xx[7] + xx[112] + xx[21];
  xx[16] = xx[8] + xx[52] + xx[22];
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 14, xx + 6);
  xx[14] = xx[157] - (pm_math_Vector3_dot_ra(xx + 10, xx + 3) +
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
  pm_math_Vector3_cross_ra(xx + 15, xx + 130, xx + 21);
  xx[15] = xx[6] + xx[63] + xx[21];
  xx[16] = xx[7] + xx[64] + xx[22];
  xx[17] = xx[8] + xx[65] + xx[23];
  pm_math_Quaternion_inverseXform_ra(xx + 90, xx + 15, xx + 6);
  xx[15] = xx[148] - (pm_math_Vector3_dot_ra(xx + 10, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 18, xx + 6));
  xx[10] = - xx[107];
  xx[11] = - xx[129];
  xx[12] = xx[135];
  xx[16] = xx[3] + xx[114];
  xx[17] = xx[4] - xx[101];
  xx[18] = xx[5] + xx[15];
  pm_math_Quaternion_inverseXform_ra(xx + 108, xx + 16, xx + 3);
  pm_math_Vector3_cross_ra(xx + 16, xx + 141, xx + 19);
  xx[16] = xx[6] + xx[140] * xx[15] + xx[80] + xx[19];
  xx[17] = xx[7] + xx[126] + xx[20];
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
