/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'la_2dof_simul/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "la_2dof_simul_e38647b6_1_geometries.h"

PmfMessageId la_2dof_simul_e38647b6_1_deriv(const RuntimeDerivedValuesBundle
  *rtdv, const int *eqnEnableFlags, const double *state, const int *modeVector,
  const double *input, const double *inputDot, const double *inputDdot, const
  double *discreteState, double *deriv, double *errorResult,
  NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[157];
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
  xx[1] = xx[3] / xx[4];
  if (xx[5] < xx[1])
    xx[1] = xx[5];
  xx[6] = xx[1] * xx[1] * (xx[7] - xx[8] * xx[1]) * (xx[11] * xx[3] + (xx[3] ==
    xx[2] ? xx[2] : xx[10]));
  if (xx[2] > xx[6])
    xx[6] = xx[2];
  xx[1] = 0.4999981633974483;
  xx[3] = 0.4999999999966269;
  xx[10] = - xx[3];
  xx[13] = 0.5000018366025516;
  xx[14] = - xx[13];
  xx[15] = - xx[1];
  xx[16] = xx[10];
  xx[17] = xx[14];
  xx[18] = xx[10];
  xx[19] = 1.414210965022473;
  xx[20] = 0.7071080798594735;
  xx[21] = xx[20] * state[1];
  xx[22] = xx[19] * xx[21];
  xx[23] = xx[3] * xx[22];
  xx[24] = 1.414216159718947;
  xx[25] = state[1] - xx[24] * xx[21];
  xx[21] = xx[23] - xx[13] * xx[25];
  xx[26] = xx[10];
  xx[27] = xx[14];
  xx[28] = xx[10];
  xx[10] = xx[3] * xx[25];
  xx[29] = xx[21];
  xx[30] = xx[10];
  xx[31] = - xx[23];
  pm_math_Vector3_cross_ra(xx + 26, xx + 29, xx + 32);
  xx[14] = xx[1] * xx[21] + xx[32];
  xx[21] = xx[8] * (xx[33] + xx[1] * xx[10]) + xx[22];
  xx[10] = 0.2499990816970376;
  xx[23] = xx[25] + xx[8] * (xx[34] - xx[10] * xx[22]);
  xx[29] = xx[8] * xx[14];
  xx[30] = xx[21];
  xx[31] = xx[23];
  xx[32] = 0.3;
  xx[33] = 0.6 * xx[14];
  xx[34] = xx[32] * xx[21];
  xx[35] = xx[32] * xx[23];
  pm_math_Vector3_cross_ra(xx + 29, xx + 33, xx + 36);
  xx[14] = 0.5;
  xx[21] = xx[14] * state[2];
  xx[23] = cos(xx[21]);
  xx[33] = sin(xx[21]);
  xx[21] = xx[3] * xx[33];
  xx[34] = xx[1] * xx[23] + xx[21];
  xx[35] = xx[3] * xx[23];
  xx[39] = xx[35] - xx[13] * xx[33];
  xx[40] = xx[13] * xx[23] + xx[21];
  xx[21] = xx[35] - xx[1] * xx[33];
  xx[41] = - xx[34];
  xx[42] = xx[39];
  xx[43] = - xx[40];
  xx[44] = xx[21];
  pm_math_Quaternion_inverseXform_ra(xx + 41, xx + 29, xx + 45);
  xx[23] = xx[47] + state[3];
  xx[48] = xx[45];
  xx[49] = xx[46];
  xx[50] = xx[23];
  xx[33] = xx[32] * xx[45];
  xx[51] = xx[33];
  xx[52] = xx[32] * xx[46];
  xx[53] = xx[32] * xx[23];
  pm_math_Vector3_cross_ra(xx + 48, xx + 51, xx + 54);
  xx[35] = xx[20] * xx[46];
  xx[51] = 0.7071054825112363;
  xx[52] = xx[20] * xx[23];
  xx[53] = xx[46] - xx[8] * (xx[20] * xx[35] - xx[51] * xx[52]);
  xx[57] = xx[23] - xx[8] * (xx[51] * xx[35] + xx[20] * xx[52]);
  xx[58] = xx[45];
  xx[59] = xx[53];
  xx[60] = xx[57];
  xx[61] = xx[33];
  xx[62] = xx[32] * xx[53];
  xx[63] = xx[32] * xx[57];
  pm_math_Vector3_cross_ra(xx + 58, xx + 61, xx + 64);
  xx[35] = xx[20] * xx[57];
  xx[52] = xx[20] * xx[53];
  xx[61] = xx[53] - xx[8] * (xx[51] * xx[35] + xx[20] * xx[52]);
  xx[62] = xx[57] + xx[8] * (xx[51] * xx[52] - xx[20] * xx[35]);
  xx[67] = xx[45];
  xx[68] = xx[61];
  xx[69] = xx[62];
  xx[70] = xx[33];
  xx[71] = xx[32] * xx[61];
  xx[72] = xx[32] * xx[62];
  pm_math_Vector3_cross_ra(xx + 67, xx + 70, xx + 73);
  xx[35] = xx[20] * xx[61];
  xx[52] = xx[20] * xx[62];
  xx[63] = xx[61] - xx[8] * (xx[20] * xx[35] - xx[51] * xx[52]);
  xx[70] = xx[62] - xx[8] * (xx[51] * xx[35] + xx[20] * xx[52]);
  xx[76] = xx[45];
  xx[77] = xx[63];
  xx[78] = xx[70];
  xx[79] = xx[33];
  xx[80] = xx[32] * xx[63];
  xx[81] = xx[32] * xx[70];
  pm_math_Vector3_cross_ra(xx + 76, xx + 79, xx + 82);
  xx[33] = 2.387583317164932e-7;
  xx[35] = 0.04;
  xx[52] = 0.06499999999956149;
  xx[79] = - xx[33];
  xx[80] = xx[35];
  xx[81] = xx[52];
  xx[71] = 0.25;
  pm_math_Vector3_cross_ra(xx + 76, xx + 79, xx + 85);
  pm_math_Vector3_cross_ra(xx + 76, xx + 85, xx + 88);
  xx[72] = xx[20] * xx[90];
  xx[85] = xx[20] * xx[88];
  xx[86] = xx[71] * (xx[88] + xx[8] * (xx[51] * xx[72] - xx[20] * xx[85]));
  xx[87] = xx[71] * (xx[90] - xx[8] * (xx[51] * xx[85] + xx[20] * xx[72]));
  xx[72] = xx[20] * xx[87];
  xx[85] = xx[20] * xx[86];
  xx[91] = xx[86] - xx[8] * (xx[51] * xx[72] + xx[20] * xx[85]);
  xx[86] = xx[71] * xx[89];
  xx[88] = xx[87] - xx[8] * (xx[20] * xx[72] - xx[51] * xx[85]);
  xx[92] = xx[91];
  xx[93] = xx[86];
  xx[94] = xx[88];
  pm_math_Vector3_cross_ra(xx + 79, xx + 92, xx + 95);
  xx[79] = xx[33];
  xx[80] = - xx[35];
  xx[81] = xx[52];
  pm_math_Vector3_cross_ra(xx + 76, xx + 79, xx + 92);
  pm_math_Vector3_cross_ra(xx + 76, xx + 92, xx + 98);
  xx[33] = xx[20] * xx[100];
  xx[35] = xx[20] * xx[98];
  xx[52] = xx[71] * (xx[98] - xx[8] * (xx[51] * xx[33] + xx[20] * xx[35]));
  xx[72] = xx[71] * (xx[100] - xx[8] * (xx[20] * xx[33] - xx[51] * xx[35]));
  xx[33] = xx[20] * xx[72];
  xx[35] = xx[20] * xx[52];
  xx[76] = xx[52] + xx[8] * (xx[51] * xx[33] - xx[20] * xx[35]);
  xx[52] = xx[71] * xx[99];
  xx[71] = xx[72] - xx[8] * (xx[51] * xx[35] + xx[20] * xx[33]);
  xx[92] = xx[76];
  xx[93] = xx[52];
  xx[94] = xx[71];
  pm_math_Vector3_cross_ra(xx + 79, xx + 92, xx + 98);
  xx[33] = 0.03;
  xx[35] = xx[33] * xx[63];
  xx[72] = xx[35] * xx[70];
  xx[77] = xx[86] + xx[52] + 0.4999999999999998 * xx[72];
  xx[52] = 0.04999999999932536;
  xx[78] = 3.67320510313851e-7;
  xx[79] = xx[52] * xx[62] - xx[78] * xx[61];
  xx[80] = xx[78] * xx[45];
  xx[81] = - (xx[52] * xx[45]);
  pm_math_Vector3_cross_ra(xx + 67, xx + 79, xx + 85);
  xx[61] = xx[20] * xx[86];
  xx[62] = xx[20] * xx[87];
  xx[67] = xx[86] - xx[8] * (xx[20] * xx[61] - xx[51] * xx[62]);
  xx[68] = xx[77] + 0.7999999999999998 * xx[67];
  xx[69] = 0.4999999999999999;
  xx[79] = xx[33] * xx[45];
  xx[80] = 0.7999999999999999;
  xx[81] = xx[88] + xx[71] - xx[69] * (xx[79] * xx[45] + xx[35] * xx[63]) + xx
    [80] * (xx[87] - xx[8] * (xx[51] * xx[61] + xx[20] * xx[62]));
  xx[35] = xx[20] * xx[81];
  xx[61] = xx[20] * xx[68];
  xx[62] = xx[68] - xx[8] * (xx[51] * xx[35] + xx[20] * xx[61]);
  xx[63] = xx[81] + xx[8] * (xx[51] * xx[61] - xx[20] * xx[35]);
  xx[35] = 0.08259999999966269;
  xx[61] = 1.836602551569255e-7;
  xx[88] = xx[35] * xx[53] - xx[61] * xx[57];
  xx[89] = - (xx[35] * xx[45]);
  xx[90] = xx[61] * xx[45];
  pm_math_Vector3_cross_ra(xx + 58, xx + 88, xx + 92);
  xx[53] = xx[20] * xx[94];
  xx[57] = xx[20] * xx[93];
  xx[58] = xx[93] - xx[8] * (xx[51] * xx[53] + xx[20] * xx[57]);
  xx[59] = xx[94] + xx[8] * (xx[51] * xx[57] - xx[20] * xx[53]);
  xx[53] = 8.470329472543003e-22;
  xx[57] = xx[63] + xx[53] * xx[58] + 1.1 * xx[59];
  xx[60] = xx[62] + 1.1 * xx[58];
  xx[68] = xx[20] * xx[60];
  xx[71] = xx[20] * xx[57];
  xx[81] = xx[57] - xx[8] * (xx[51] * xx[68] + xx[20] * xx[71]);
  xx[57] = xx[60] - xx[8] * (xx[20] * xx[68] - xx[51] * xx[71]);
  xx[60] = 0.08999999999878563;
  xx[68] = 6.611769185760341e-7;
  xx[88] = xx[60] * xx[23] - xx[68] * xx[46];
  xx[89] = xx[68] * xx[45];
  xx[90] = - (xx[60] * xx[45]);
  pm_math_Vector3_cross_ra(xx + 48, xx + 88, xx + 101);
  xx[48] = xx[20] * xx[102];
  xx[49] = xx[20] * xx[103];
  xx[50] = xx[103] - xx[8] * (xx[51] * xx[48] + xx[20] * xx[49]);
  xx[71] = xx[102] - xx[8] * (xx[20] * xx[48] - xx[51] * xx[49]);
  xx[48] = xx[57] + 1.399999999999999 * xx[71];
  xx[49] = xx[81] + xx[53] * xx[71] + 1.4 * xx[50];
  xx[88] = xx[20] * xx[49];
  xx[89] = xx[20] * xx[48];
  xx[90] = xx[48] - xx[8] * (xx[51] * xx[88] + xx[20] * xx[89]);
  xx[48] = xx[49] + xx[8] * (xx[51] * xx[89] - xx[20] * xx[88]);
  xx[49] = 1.277467336000321;
  xx[88] = state[3] * xx[46];
  xx[89] = 1.754016645088257e-14;
  xx[104] = state[3] * xx[45];
  xx[45] = 1.525702471692752e-6;
  xx[105] = 0.09;
  xx[106] = xx[105] * state[3];
  xx[107] = xx[105] * xx[21];
  xx[108] = xx[105] * xx[39];
  xx[109] = - (xx[8] * (xx[107] * xx[34] - xx[108] * xx[40]));
  xx[110] = - (xx[105] - xx[8] * (xx[107] * xx[21] + xx[108] * xx[39]));
  xx[111] = 0.06 + xx[8] * (xx[108] * xx[34] + xx[107] * xx[40]);
  pm_math_Vector3_cross_ra(xx + 29, xx + 109, xx + 112);
  pm_math_Vector3_cross_ra(xx + 29, xx + 112, xx + 115);
  pm_math_Quaternion_inverseXform_ra(xx + 41, xx + 115, xx + 29);
  xx[107] = xx[106] * (xx[47] + xx[23]) + xx[30];
  xx[23] = 0.3043599999969784;
  xx[47] = xx[31] - xx[106] * (xx[46] + xx[46]);
  xx[46] = 4.775166634297645e-9;
  xx[106] = state[2] + xx[8];
  if (xx[2] < xx[106])
    xx[106] = xx[2];
  xx[108] = - (xx[106] / xx[4]);
  if (xx[5] < xx[108])
    xx[108] = xx[5];
  xx[112] = xx[9] * state[3];
  xx[9] = xx[108] * xx[108] * (xx[7] - xx[8] * xx[108]) * ((- xx[106] == xx[2] ?
    xx[2] : - xx[112]) - xx[11] * xx[106]);
  if (xx[2] > xx[9])
    xx[9] = xx[2];
  xx[106] = state[2] - xx[8];
  if (xx[2] > xx[106])
    xx[106] = xx[2];
  xx[108] = xx[106] / xx[4];
  if (xx[5] < xx[108])
    xx[108] = xx[5];
  xx[4] = xx[108] * xx[108] * (xx[7] - xx[8] * xx[108]) * (xx[11] * xx[106] +
    (xx[106] == xx[2] ? xx[2] : xx[112]));
  if (xx[2] > xx[4])
    xx[4] = xx[2];
  xx[7] = xx[84] + xx[97] + xx[100];
  xx[11] = xx[79] * xx[70];
  xx[70] = xx[91] + xx[76] + xx[69] * xx[11];
  xx[69] = xx[83] + xx[96] + xx[99] + 0.03249999999978074 * xx[11] + xx[33] *
    xx[70] + 0.04749999999978074 * xx[85];
  xx[11] = xx[20] * xx[69];
  xx[76] = xx[20] * xx[7];
  xx[79] = xx[70] + xx[80] * xx[85];
  xx[70] = xx[75] + xx[7] + xx[8] * (xx[51] * xx[11] - xx[20] * xx[76]) + xx[52]
    * xx[79] + 0.08749999999892055 * xx[92];
  xx[7] = xx[74] + xx[69] - xx[8] * (xx[51] * xx[76] + xx[20] * xx[11]) - xx[78]
    * xx[79] - 4.683336506625383e-7 * xx[92];
  xx[11] = xx[20] * xx[7];
  xx[69] = xx[20] * xx[70];
  xx[74] = xx[79] + 1.099999999999999 * xx[92];
  xx[75] = xx[66] + xx[70] - xx[8] * (xx[51] * xx[11] + xx[20] * xx[69]) - xx[61]
    * xx[74] - 5.509807655817988e-8 * xx[101];
  xx[66] = xx[65] + xx[7] - xx[8] * (xx[20] * xx[11] - xx[51] * xx[69]) + xx[35]
    * xx[74] + 0.1783599999996794 * xx[101];
  xx[7] = xx[20] * xx[66];
  xx[11] = xx[20] * xx[75];
  xx[65] = xx[74] + 1.399999999999999 * xx[101];
  xx[69] = 3.697842612216864e-7;
  xx[70] = 0.3043599999969783;
  xx[30] = xx[56] + xx[75] + xx[8] * (xx[51] * xx[7] - xx[20] * xx[11]) + xx[60]
    * xx[65] + xx[46] * xx[88] + xx[69] * xx[104] + xx[70] * xx[29];
  xx[31] = 1.699999999999999;
  xx[74] = 1.525702471692751e-6;
  xx[75] = xx[65] + xx[31] * xx[29] + xx[74] * xx[104];
  xx[76] = 1.345222135997987;
  ii[0] = factorSymmetricPosDef(xx + 76, 1, xx + 79);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'la_2dof_simul/la_4_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[79] = (input[1] - xx[0] * state[3] + xx[9] - xx[4] - (xx[30] + xx[105] *
             xx[75])) / xx[76];
  xx[4] = 1.200800000001848;
  xx[9] = 5.070974836740335e-7;
  xx[56] = 1.304059735998259;
  xx[83] = xx[54] + xx[64] + xx[73] + xx[82] + xx[95] + xx[98] -
    0.03249999999978073 * xx[72] - xx[33] * xx[77] - 0.04749999999978072 * xx[67]
    + xx[78] * xx[62] - xx[52] * xx[63] + 4.683336506625385e-7 * xx[58] -
    0.08749999999892054 * xx[59] + xx[61] * xx[81] - xx[35] * xx[57] +
    5.509807655817967e-8 * xx[50] - 0.1783599999996794 * xx[71] + xx[68] * xx[90]
    - xx[60] * xx[48] + xx[49] * xx[88] + xx[89] * xx[104] + xx[45] * xx[107] -
    xx[23] * xx[47] + xx[46] * xx[79];
  xx[84] = xx[55] + xx[66] - xx[8] * (xx[51] * xx[11] + xx[20] * xx[7]) - xx[68]
    * xx[65] - (xx[89] * xx[88] + xx[4] * xx[104] + xx[74] * xx[29]) - xx[9] *
    xx[79];
  xx[85] = xx[30] + xx[56] * xx[79];
  pm_math_Quaternion_xform_ra(xx + 41, xx + 83, xx + 57);
  xx[7] = 0.4573599999969781;
  xx[11] = 1.7;
  xx[29] = 1.7;
  xx[60] = xx[75] + xx[7] * xx[79];
  xx[61] = xx[90] + xx[45] * xx[88] + xx[11] * xx[107];
  xx[62] = xx[48] + xx[53] * xx[107] + xx[29] * xx[47] - xx[23] * xx[88];
  pm_math_Quaternion_xform_ra(xx + 41, xx + 60, xx + 63);
  pm_math_Vector3_cross_ra(xx + 109, xx + 63, xx + 60);
  xx[30] = xx[34] * xx[34];
  xx[33] = xx[8] * (xx[30] + xx[39] * xx[39]) - xx[5];
  xx[35] = xx[34] * xx[21];
  xx[47] = xx[40] * xx[39];
  xx[48] = xx[8] * (xx[35] - xx[47]);
  xx[50] = xx[39] * xx[21];
  xx[52] = xx[34] * xx[40];
  xx[54] = xx[8] * (xx[50] + xx[52]);
  xx[55] = xx[8] * (xx[47] + xx[35]);
  xx[35] = xx[8] * (xx[30] + xx[40] * xx[40]) - xx[5];
  xx[47] = xx[34] * xx[39];
  xx[34] = xx[40] * xx[21];
  xx[39] = xx[8] * (xx[47] - xx[34]);
  xx[40] = xx[8] * (xx[50] - xx[52]);
  xx[50] = xx[8] * (xx[34] + xx[47]);
  xx[34] = xx[8] * (xx[30] + xx[21] * xx[21]) - xx[5];
  xx[80] = xx[33];
  xx[81] = xx[48];
  xx[82] = xx[54];
  xx[83] = - xx[55];
  xx[84] = xx[35];
  xx[85] = xx[39];
  xx[86] = xx[40];
  xx[87] = - xx[50];
  xx[88] = xx[34];
  xx[5] = xx[7] / xx[76];
  xx[21] = xx[46] * xx[5];
  xx[30] = xx[9] * xx[5] - xx[74];
  xx[47] = xx[70] - xx[56] * xx[5];
  xx[90] = xx[45] * xx[48] - xx[21] * xx[33] - xx[23] * xx[54];
  xx[91] = xx[45] * xx[35] + xx[21] * xx[55] - xx[23] * xx[39];
  xx[92] = - (xx[45] * xx[50] + xx[21] * xx[40] + xx[23] * xx[34]);
  xx[93] = xx[33] * xx[30];
  xx[94] = - (xx[55] * xx[30]);
  xx[95] = xx[40] * xx[30];
  xx[96] = xx[47] * xx[33];
  xx[97] = - (xx[55] * xx[47]);
  xx[98] = xx[40] * xx[47];
  pm_math_Matrix3x3_compose_ra(xx + 80, xx + 90, xx + 99);
  xx[21] = xx[31] - xx[7] * xx[5];
  xx[90] = xx[21] * xx[33];
  xx[91] = - (xx[55] * xx[21]);
  xx[92] = xx[40] * xx[21];
  xx[93] = xx[11] * xx[48];
  xx[94] = xx[11] * xx[35];
  xx[95] = - (xx[11] * xx[50]);
  xx[96] = xx[53] * xx[48] + xx[29] * xx[54];
  xx[97] = xx[53] * xx[35] + xx[29] * xx[39];
  xx[98] = xx[29] * xx[34] - xx[53] * xx[50];
  pm_math_Matrix3x3_compose_ra(xx + 80, xx + 90, xx + 112);
  pm_math_Matrix3x3_postCross_ra(xx + 112, xx + 109, xx + 90);
  xx[121] = xx[99] - xx[90];
  xx[122] = xx[100] - xx[93];
  xx[123] = xx[101] - xx[96];
  xx[124] = xx[102] - xx[91];
  xx[125] = xx[103] - xx[94];
  xx[126] = xx[104] - xx[97];
  xx[127] = xx[105] - xx[92];
  xx[128] = xx[106] - xx[95];
  xx[129] = xx[107] - xx[98];
  xx[7] = 0.1049999999971666;
  xx[11] = 7.713730716396583e-7;
  xx[21] = 7.713730716951694e-7;
  xx[23] = xx[11] * xx[22] + xx[21] * xx[25];
  xx[29] = - (xx[7] * xx[22] * xx[22] + xx[7] * xx[25] * xx[25]);
  xx[30] = - (xx[25] * xx[23]);
  xx[31] = xx[22] * xx[23];
  pm_math_Quaternion_inverseXform_ra(xx + 15, xx + 29, xx + 33);
  pm_math_Matrix3x3_xform_ra(xx + 121, xx + 33, xx + 29);
  xx[52] = xx[36] + xx[57] + xx[60] + xx[29];
  xx[53] = xx[37] + xx[58] + xx[61] + xx[30];
  xx[54] = xx[38] + xx[59] + xx[62] + xx[31];
  pm_math_Quaternion_xform_ra(xx + 15, xx + 52, xx + 29);
  xx[36] = xx[7];
  xx[37] = xx[21];
  xx[38] = - xx[11];
  xx[130] = xx[32] + xx[112];
  xx[131] = xx[113];
  xx[132] = xx[114];
  xx[133] = xx[115];
  xx[134] = xx[32] + xx[116];
  xx[135] = xx[117];
  xx[136] = xx[118];
  xx[137] = xx[119];
  xx[138] = xx[32] + xx[120];
  pm_math_Matrix3x3_xform_ra(xx + 130, xx + 33, xx + 52);
  xx[57] = xx[63] + xx[52];
  xx[58] = xx[64] + xx[53];
  xx[59] = xx[65] + xx[54];
  pm_math_Quaternion_xform_ra(xx + 15, xx + 57, xx + 52);
  pm_math_Vector3_cross_ra(xx + 36, xx + 52, xx + 57);
  xx[22] = 0.125;
  xx[23] = state[1] * state[1];
  xx[25] = xx[22] * xx[23];
  xx[39] = 3.673205103416066e-6;
  xx[40] = - xx[39];
  xx[45] = 3.673205103249533e-6;
  xx[47] = 0.9999999999932536;
  xx[60] = xx[40];
  xx[61] = xx[45];
  xx[62] = 0.9999999999865075;
  xx[63] = xx[47];
  xx[64] = 1.34923183736646e-11;
  xx[65] = xx[45];
  xx[66] = 5.551115123125783e-17;
  xx[67] = xx[47];
  xx[68] = xx[40];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 121, xx + 60, xx + 112);
  pm_math_Matrix3x3_compose_ra(xx + 60, xx + 112, xx + 121);
  pm_math_Matrix3x3_composeTranspose_ra(xx + 130, xx + 60, xx + 112);
  pm_math_Matrix3x3_compose_ra(xx + 60, xx + 112, xx + 130);
  pm_math_Matrix3x3_postCross_ra(xx + 130, xx + 36, xx + 112);
  xx[40] = xx[127] - xx[114];
  xx[45] = xx[31] + xx[59] - xx[25] * xx[40];
  xx[29] = xx[124] - xx[113];
  xx[31] = xx[53] - xx[25] * xx[133];
  xx[47] = xx[31] - xx[8] * (xx[3] * (xx[54] - xx[25] * xx[136]) + xx[13] * xx
    [31]);
  xx[31] = 0.0403;
  xx[48] = xx[31] * xx[23];
  xx[23] = 0.9999999999932535;
  xx[50] = 3.673205103416065e-6;
  xx[52] = 0.9999999999932537;
  xx[53] = 0.9999999999999998 * (xx[39] * xx[131] + xx[52] * xx[132]);
  xx[54] = xx[23] * xx[29] - xx[50] * xx[40] - xx[22] * xx[53];
  xx[40] = xx[46] / xx[76];
  xx[55] = xx[9] * xx[40] - xx[89];
  xx[57] = xx[46] - xx[56] * xx[40];
  xx[59] = xx[9] / xx[76];
  xx[70] = xx[56] * xx[59];
  xx[139] = xx[49] - xx[46] * xx[40];
  xx[140] = xx[55];
  xx[141] = xx[57];
  xx[142] = xx[55];
  xx[143] = xx[4] - xx[9] * xx[59];
  xx[144] = xx[70] - 3.69784261221686e-7;
  xx[145] = xx[57];
  xx[146] = xx[70] - xx[69];
  xx[147] = 1.276667335998531 - 1.700571795051848 / xx[76];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 139, xx + 80, xx + 148);
  pm_math_Matrix3x3_compose_ra(xx + 80, xx + 148, xx + 139);
  pm_math_Matrix3x3_postCross_ra(xx + 99, xx + 109, xx + 80);
  pm_math_Matrix3x3_preCross_ra(xx + 90, xx + 109, xx + 99);
  xx[89] = xx[32] + xx[139] - xx[80] - xx[80] - xx[99];
  xx[90] = xx[140] - xx[81] - xx[83] - xx[100];
  xx[91] = xx[141] - xx[82] - xx[86] - xx[101];
  xx[92] = xx[142] - xx[83] - xx[81] - xx[102];
  xx[93] = xx[32] + xx[143] - xx[84] - xx[84] - xx[103];
  xx[94] = xx[144] - xx[85] - xx[87] - xx[104];
  xx[95] = xx[145] - xx[86] - xx[82] - xx[105];
  xx[96] = xx[146] - xx[87] - xx[85] - xx[106];
  xx[97] = xx[32] + xx[147] - xx[88] - xx[88] - xx[107];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 89, xx + 60, xx + 80);
  pm_math_Matrix3x3_compose_ra(xx + 60, xx + 80, xx + 89);
  pm_math_Matrix3x3_postCross_ra(xx + 121, xx + 36, xx + 60);
  pm_math_Matrix3x3_preCross_ra(xx + 112, xx + 36, xx + 80);
  xx[4] = xx[128] - xx[117];
  xx[9] = xx[129] - xx[120];
  xx[36] = xx[125] - xx[116];
  xx[37] = xx[126] - xx[119];
  xx[38] = xx[39] * (xx[39] * xx[4] + xx[52] * xx[9]) - xx[52] * (xx[39] * xx[36]
    + xx[52] * xx[37]);
  xx[46] = xx[22] * xx[38];
  xx[49] = xx[39] * (xx[32] + xx[134]) + xx[52] * xx[135];
  xx[55] = xx[39] * xx[137] + xx[52] * (xx[32] + xx[138]);
  xx[57] = xx[39] * xx[49] + xx[52] * xx[55];
  xx[60] = xx[22] * xx[57];
  xx[61] = xx[38] + xx[60];
  xx[38] = xx[61] + xx[31] * (xx[32] + xx[57]);
  xx[57] = xx[52] * (xx[52] * (xx[32] + xx[93] - xx[64] - xx[64] - xx[84]) - xx
                     [39] * (xx[94] - xx[65] - xx[67] - xx[85])) - xx[39] * (xx
    [52] * (xx[96] - xx[67] - xx[65] - xx[87]) - xx[39] * (xx[32] + xx[97] - xx
    [68] - xx[68] - xx[88])) + xx[46] + xx[46] + xx[22] * xx[60] + xx[31] * xx
    [61] + xx[31] * xx[38] + xx[32];
  ii[0] = factorSymmetricPosDef(xx + 57, 1, xx + 32);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'la_2dof_simul/la_1_joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[32] = xx[39] * xx[55] - xx[52] * xx[49];
  xx[60] = (xx[54] - xx[31] * xx[53]) / xx[57];
  xx[61] = xx[38] / xx[57];
  xx[62] = (xx[52] * (xx[52] * xx[36] - xx[39] * xx[37]) - xx[39] * (xx[52] *
             xx[4] - xx[39] * xx[9]) + xx[22] * xx[32] + xx[31] * xx[32]) / xx
    [57];
  xx[4] = xx[14] * state[0];
  xx[9] = sin(xx[4]);
  xx[14] = xx[20] * xx[9];
  xx[32] = 9.806649999999999;
  xx[36] = xx[32] * xx[14];
  xx[37] = xx[51] * xx[9];
  xx[9] = xx[32] * xx[37];
  xx[38] = xx[8] * (xx[14] * xx[36] + xx[37] * xx[9]) - xx[32];
  xx[14] = cos(xx[4]);
  xx[4] = xx[51] * xx[14];
  xx[32] = xx[20] * xx[14];
  xx[14] = xx[8] * (xx[4] * xx[9] + xx[32] * xx[36]);
  xx[37] = xx[8] * (xx[4] * xx[36] - xx[32] * xx[9]);
  xx[63] = xx[38];
  xx[64] = xx[14];
  xx[65] = xx[37];
  xx[4] = (input[0] - xx[0] * state[1] + xx[12] - xx[6] - (xx[45] + xx[8] * (xx
             [3] * (xx[30] + xx[58] - xx[25] * xx[29]) - xx[13] * xx[45]) + xx
            [22] * xx[47] - xx[48] * xx[54] + xx[31] * (xx[47] + xx[48] * (xx[50]
              * xx[133] + xx[23] * xx[136])))) / xx[57] - pm_math_Vector3_dot_ra
    (xx + 60, xx + 63);
  xx[45] = xx[40];
  xx[46] = - xx[59];
  xx[47] = xx[56] / xx[76];
  xx[0] = xx[20] * xx[4];
  xx[6] = xx[19] * xx[0];
  xx[9] = xx[3] * xx[6];
  xx[12] = xx[4] - xx[24] * xx[0];
  xx[0] = xx[9] - xx[13] * xx[12];
  xx[13] = xx[3] * xx[12];
  xx[52] = xx[0];
  xx[53] = xx[13];
  xx[54] = - xx[9];
  pm_math_Vector3_cross_ra(xx + 26, xx + 52, xx + 55);
  xx[26] = xx[8] * (xx[1] * xx[0] + xx[55]);
  xx[27] = xx[8] * (xx[56] + xx[1] * xx[13]) + xx[6];
  xx[28] = xx[12] + xx[8] * (xx[57] - xx[10] * xx[6]);
  pm_math_Quaternion_inverseXform_ra(xx + 41, xx + 26, xx + 52);
  xx[0] = xx[31] * xx[4] + xx[14] + xx[22] * xx[4];
  xx[1] = xx[20] * xx[0];
  xx[3] = xx[20] * xx[37];
  xx[22] = xx[38] - xx[48] - xx[25] - (xx[11] * xx[6] + xx[21] * xx[12]);
  xx[23] = xx[0] - xx[8] * (xx[20] * xx[1] - xx[51] * xx[3]) + xx[7] * xx[12];
  xx[24] = xx[37] - xx[8] * (xx[51] * xx[1] + xx[20] * xx[3]) - xx[7] * xx[6];
  pm_math_Quaternion_inverseXform_ra(xx + 15, xx + 22, xx + 6);
  pm_math_Vector3_cross_ra(xx + 26, xx + 109, xx + 9);
  xx[12] = xx[6] + xx[33] + xx[9];
  xx[13] = xx[7] + xx[34] + xx[10];
  xx[14] = xx[8] + xx[35] + xx[11];
  pm_math_Quaternion_inverseXform_ra(xx + 41, xx + 12, xx + 6);
  deriv[0] = state[1];
  deriv[1] = xx[4];
  deriv[2] = state[3];
  deriv[3] = xx[79] - (pm_math_Vector3_dot_ra(xx + 45, xx + 52) + xx[5] * xx[6]);
  errorResult[0] = xx[2];
  return NULL;
}
