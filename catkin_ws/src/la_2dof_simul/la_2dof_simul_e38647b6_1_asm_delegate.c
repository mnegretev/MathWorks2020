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
#include "sm_CTarget.h"

void la_2dof_simul_e38647b6_1_setTargets(const RuntimeDerivedValuesBundle *rtdv,
  CTarget *targets)
{
  (void) rtdv;
  (void) targets;
}

void la_2dof_simul_e38647b6_1_resetAsmStateVector(const void *mech, double
  *state)
{
  double xx[1];
  (void) mech;
  xx[0] = 0.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[0];
}

void la_2dof_simul_e38647b6_1_initializeTrackedAngleState(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const int *modeVector, const double
  *motionData, double *state, void *neDiagMgr0)
{
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  (void) state;
  (void) neDiagMgr;
}

void la_2dof_simul_e38647b6_1_computeDiscreteState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
}

void la_2dof_simul_e38647b6_1_adjustPosition(const void *mech, const double
  *dofDeltas, double *state)
{
  (void) mech;
  state[0] = state[0] + dofDeltas[0];
  state[2] = state[2] + dofDeltas[1];
}

static void perturbAsmJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[1] = state[1] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_3_0(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_3_0v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[3] = state[3] - 0.875 * mag;
}

void la_2dof_simul_e38647b6_1_perturbAsmJointPrimitiveState(const void *mech,
  size_t stageIdx, size_t primIdx, double mag, boolean_T doPerturbVelocity,
  double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbAsmJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbAsmJointPrimitiveState_0_0v(mag, state);
    break;

   case 36:
    perturbAsmJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbAsmJointPrimitiveState_3_0v(mag, state);
    break;
  }
}

void la_2dof_simul_e38647b6_1_computeDofBlendMatrix(const void *mech, size_t
  stageIdx, size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void la_2dof_simul_e38647b6_1_projectPartiallyTargetedPos(const void *mech,
  size_t stageIdx, size_t primIdx, const double *origState, int partialType,
  double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void la_2dof_simul_e38647b6_1_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[97];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  xx[0] = 0.7071054825112363;
  xx[1] = 0.5;
  xx[2] = xx[1] * state[0];
  xx[3] = cos(xx[2]);
  xx[4] = xx[0] * xx[3];
  xx[5] = 0.7071080798594735;
  xx[6] = - (xx[5] * xx[3]);
  xx[7] = sin(xx[2]);
  xx[2] = xx[5] * xx[7];
  xx[8] = - (xx[0] * xx[7]);
  xx[9] = 2.0;
  xx[10] = xx[6];
  xx[11] = xx[2];
  xx[12] = xx[8];
  xx[13] = 0.015;
  xx[14] = xx[13] * xx[2];
  xx[15] = 0.02849635094520282 * xx[7] - 0.0106066211978921 * xx[3];
  xx[3] = 0.0403;
  xx[7] = xx[3] * xx[2];
  xx[16] = - xx[14];
  xx[17] = xx[15];
  xx[18] = xx[7];
  pm_math_Vector3_cross_ra(xx + 10, xx + 16, xx + 19);
  xx[10] = 0.09;
  xx[11] = - xx[0];
  xx[12] = - xx[5];
  xx[16] = 0.0;
  xx[17] = 0.125;
  xx[18] = 0.4999981633974483;
  xx[22] = - xx[18];
  xx[23] = 0.4999999999966269;
  xx[24] = - xx[23];
  xx[25] = 0.5000018366025516;
  xx[26] = - xx[25];
  xx[27] = 0.1049999999971666;
  xx[28] = 7.713730716951694e-7;
  xx[29] = 7.713730716396583e-7;
  xx[30] = xx[1] * state[2];
  xx[1] = cos(xx[30]);
  xx[31] = sin(xx[30]);
  xx[30] = xx[23] * xx[31];
  xx[32] = xx[18] * xx[1] + xx[30];
  xx[33] = - xx[32];
  xx[34] = xx[23] * xx[1];
  xx[35] = xx[34] - xx[25] * xx[31];
  xx[36] = xx[25] * xx[1] + xx[30];
  xx[1] = - xx[36];
  xx[30] = xx[34] - xx[18] * xx[31];
  xx[31] = xx[10] * xx[30];
  xx[34] = xx[10] * xx[35];
  xx[37] = - (xx[9] * (xx[31] * xx[32] - xx[34] * xx[36]));
  xx[38] = - (xx[10] - xx[9] * (xx[31] * xx[30] + xx[34] * xx[35]));
  xx[39] = 0.06 + xx[9] * (xx[34] * xx[32] + xx[31] * xx[36]);
  xx[31] = 0.08999999999878563;
  xx[32] = 6.611769185760341e-7;
  xx[34] = 1.836602551569255e-7;
  xx[36] = 0.08259999999966269;
  xx[40] = 0.04999999999932536;
  xx[41] = 3.67320510313851e-7;
  xx[42] = 0.03;
  xx[43] = 2.387583317164932e-7;
  xx[44] = 0.04;
  xx[45] = - xx[44];
  xx[46] = 0.06499999999956149;
  xx[47] = - xx[43];
  xx[48] = xx[3] * state[1];
  xx[3] = xx[5] * state[1];
  xx[49] = xx[9] * xx[0] * xx[3];
  xx[50] = 1.414216159718947;
  xx[51] = state[1] - xx[50] * xx[3];
  xx[3] = xx[17] * state[1] + xx[48];
  xx[52] = xx[5] * xx[3];
  xx[53] = xx[3] - xx[50] * xx[52];
  xx[3] = xx[9] * xx[0] * xx[52];
  xx[50] = xx[23] * xx[49];
  xx[52] = xx[50] - xx[25] * xx[51];
  xx[54] = xx[24];
  xx[55] = xx[26];
  xx[56] = xx[24];
  xx[25] = xx[23] * xx[51];
  xx[57] = xx[52];
  xx[58] = xx[25];
  xx[59] = - xx[50];
  pm_math_Vector3_cross_ra(xx + 54, xx + 57, xx + 60);
  xx[23] = xx[9] * (xx[18] * xx[52] + xx[60]);
  xx[50] = xx[9] * (xx[61] + xx[18] * xx[25]) + xx[49];
  xx[18] = xx[51] + xx[9] * (xx[62] - 0.2499990816970376 * xx[49]);
  xx[54] = xx[22];
  xx[55] = xx[24];
  xx[56] = xx[26];
  xx[57] = xx[24];
  xx[58] = - (xx[29] * xx[49] + xx[28] * xx[51]);
  xx[59] = xx[27] * xx[51] + xx[53];
  xx[60] = - (xx[27] * xx[49] + xx[3]);
  pm_math_Quaternion_inverseXform_ra(xx + 54, xx + 58, xx + 61);
  xx[54] = xx[33];
  xx[55] = xx[35];
  xx[56] = xx[1];
  xx[57] = xx[30];
  xx[58] = xx[23];
  xx[59] = xx[50];
  xx[60] = xx[18];
  pm_math_Quaternion_inverseXform_ra(xx + 54, xx + 58, xx + 64);
  xx[25] = xx[66] + state[3];
  pm_math_Vector3_cross_ra(xx + 58, xx + 37, xx + 66);
  xx[58] = xx[66] + xx[61];
  xx[59] = xx[67] + xx[62];
  xx[60] = xx[68] + xx[63];
  pm_math_Quaternion_inverseXform_ra(xx + 54, xx + 58, xx + 66);
  xx[52] = xx[66] + xx[10] * state[3];
  xx[54] = xx[5] * xx[65];
  xx[55] = xx[5] * xx[25];
  xx[56] = xx[65] - xx[9] * (xx[5] * xx[54] - xx[0] * xx[55]);
  xx[57] = xx[25] - xx[9] * (xx[0] * xx[54] + xx[5] * xx[55]);
  xx[54] = xx[31] * xx[25] - xx[32] * xx[65] + xx[52];
  xx[55] = xx[67] + xx[32] * xx[64];
  xx[58] = xx[5] * xx[55];
  xx[59] = xx[68] - xx[31] * xx[64];
  xx[60] = xx[5] * xx[59];
  xx[66] = xx[55] - xx[9] * (xx[5] * xx[58] - xx[0] * xx[60]);
  xx[55] = xx[59] - xx[9] * (xx[0] * xx[58] + xx[5] * xx[60]);
  xx[58] = xx[5] * xx[57];
  xx[59] = xx[5] * xx[56];
  xx[60] = xx[56] - xx[9] * (xx[0] * xx[58] + xx[5] * xx[59]);
  xx[69] = xx[57] + xx[9] * (xx[0] * xx[59] - xx[5] * xx[58]);
  xx[58] = xx[36] * xx[56] - xx[34] * xx[57] + xx[54];
  xx[59] = xx[66] - xx[36] * xx[64];
  xx[70] = xx[34] * xx[64] + xx[55];
  xx[71] = xx[5] * xx[70];
  xx[72] = xx[5] * xx[59];
  xx[73] = xx[59] - xx[9] * (xx[0] * xx[71] + xx[5] * xx[72]);
  xx[59] = xx[70] + xx[9] * (xx[0] * xx[72] - xx[5] * xx[71]);
  xx[70] = xx[5] * xx[60];
  xx[71] = xx[5] * xx[69];
  xx[72] = xx[60] - xx[9] * (xx[5] * xx[70] - xx[0] * xx[71]);
  xx[74] = xx[69] - xx[9] * (xx[0] * xx[70] + xx[5] * xx[71]);
  xx[70] = xx[40] * xx[69] - xx[41] * xx[60] + xx[58];
  xx[71] = xx[73] + xx[41] * xx[64];
  xx[75] = xx[5] * xx[71];
  xx[76] = xx[59] - xx[40] * xx[64];
  xx[77] = xx[5] * xx[76];
  xx[78] = xx[71] - xx[9] * (xx[5] * xx[75] - xx[0] * xx[77]);
  xx[71] = xx[76] - xx[9] * (xx[0] * xx[75] + xx[5] * xx[77]);
  xx[75] = xx[42] * xx[72] + xx[70];
  xx[76] = xx[78] - xx[42] * xx[64];
  xx[77] = xx[5] * xx[74];
  xx[79] = xx[0] * xx[77];
  xx[80] = xx[5] * xx[64];
  xx[81] = xx[5] * xx[80];
  xx[82] = xx[5] * xx[77];
  xx[77] = xx[0] * xx[80];
  xx[83] = xx[64];
  xx[84] = xx[72];
  xx[85] = xx[74];
  xx[86] = xx[43];
  xx[87] = xx[45];
  xx[88] = xx[46];
  pm_math_Vector3_cross_ra(xx + 83, xx + 86, xx + 89);
  xx[80] = xx[89] + xx[75];
  xx[86] = xx[91] + xx[71];
  xx[87] = xx[5] * xx[86];
  xx[88] = xx[5] * xx[80];
  xx[91] = xx[47];
  xx[92] = xx[44];
  xx[93] = xx[46];
  pm_math_Vector3_cross_ra(xx + 83, xx + 91, xx + 94);
  xx[83] = xx[94] + xx[75];
  xx[84] = xx[96] + xx[71];
  xx[85] = xx[5] * xx[84];
  xx[89] = xx[5] * xx[83];
  motionData[0] = - xx[4];
  motionData[1] = xx[6];
  motionData[2] = xx[2];
  motionData[3] = xx[8];
  motionData[4] = 0.0953 - xx[9] * (xx[19] + xx[14] * xx[4]);
  motionData[5] = xx[10] - xx[9] * (xx[20] - xx[4] * xx[15]);
  motionData[6] = - (xx[9] * (xx[21] - xx[7] * xx[4]) - xx[13]);
  motionData[7] = xx[11];
  motionData[8] = xx[12];
  motionData[9] = xx[16];
  motionData[10] = xx[16];
  motionData[11] = xx[17];
  motionData[12] = xx[16];
  motionData[13] = - xx[13];
  motionData[14] = xx[22];
  motionData[15] = xx[24];
  motionData[16] = xx[26];
  motionData[17] = xx[24];
  motionData[18] = xx[27];
  motionData[19] = xx[28];
  motionData[20] = - xx[29];
  motionData[21] = xx[33];
  motionData[22] = xx[35];
  motionData[23] = xx[1];
  motionData[24] = xx[30];
  motionData[25] = xx[37];
  motionData[26] = xx[38];
  motionData[27] = xx[39];
  motionData[28] = xx[11];
  motionData[29] = xx[12];
  motionData[30] = xx[16];
  motionData[31] = xx[16];
  motionData[32] = xx[16];
  motionData[33] = - xx[31];
  motionData[34] = - xx[32];
  motionData[35] = xx[11];
  motionData[36] = xx[5];
  motionData[37] = xx[16];
  motionData[38] = xx[16];
  motionData[39] = xx[16];
  motionData[40] = xx[34];
  motionData[41] = xx[36];
  motionData[42] = xx[11];
  motionData[43] = xx[12];
  motionData[44] = xx[16];
  motionData[45] = xx[16];
  motionData[46] = xx[16];
  motionData[47] = - xx[40];
  motionData[48] = - xx[41];
  motionData[49] = - 1.0;
  motionData[50] = xx[16];
  motionData[51] = xx[16];
  motionData[52] = xx[16];
  motionData[53] = xx[16];
  motionData[54] = xx[16];
  motionData[55] = xx[42];
  motionData[56] = xx[11];
  motionData[57] = xx[16];
  motionData[58] = xx[12];
  motionData[59] = xx[16];
  motionData[60] = xx[43];
  motionData[61] = xx[45];
  motionData[62] = xx[46];
  motionData[63] = xx[11];
  motionData[64] = xx[16];
  motionData[65] = xx[5];
  motionData[66] = xx[16];
  motionData[67] = xx[47];
  motionData[68] = xx[44];
  motionData[69] = xx[46];
  motionData[70] = xx[16];
  motionData[71] = xx[16];
  motionData[72] = state[1];
  motionData[73] = xx[16];
  motionData[74] = xx[48];
  motionData[75] = xx[16];
  motionData[76] = xx[16];
  motionData[77] = xx[49];
  motionData[78] = xx[51];
  motionData[79] = xx[16];
  motionData[80] = xx[53];
  motionData[81] = - xx[3];
  motionData[82] = xx[23];
  motionData[83] = xx[50];
  motionData[84] = xx[18];
  motionData[85] = xx[61];
  motionData[86] = xx[62];
  motionData[87] = xx[63];
  motionData[88] = xx[64];
  motionData[89] = xx[65];
  motionData[90] = xx[25];
  motionData[91] = xx[52];
  motionData[92] = xx[67];
  motionData[93] = xx[68];
  motionData[94] = xx[64];
  motionData[95] = xx[56];
  motionData[96] = xx[57];
  motionData[97] = xx[54];
  motionData[98] = xx[66];
  motionData[99] = xx[55];
  motionData[100] = xx[64];
  motionData[101] = xx[60];
  motionData[102] = xx[69];
  motionData[103] = xx[58];
  motionData[104] = xx[73];
  motionData[105] = xx[59];
  motionData[106] = xx[64];
  motionData[107] = xx[72];
  motionData[108] = xx[74];
  motionData[109] = xx[70];
  motionData[110] = xx[78];
  motionData[111] = xx[71];
  motionData[112] = xx[64];
  motionData[113] = xx[72];
  motionData[114] = xx[74];
  motionData[115] = xx[75];
  motionData[116] = xx[76];
  motionData[117] = xx[71];
  motionData[118] = xx[64] - xx[9] * (xx[79] + xx[81]);
  motionData[119] = xx[72];
  motionData[120] = xx[74] - xx[9] * (xx[82] - xx[77]);
  motionData[121] = xx[80] - xx[9] * (xx[0] * xx[87] + xx[5] * xx[88]);
  motionData[122] = xx[90] + xx[76];
  motionData[123] = xx[86] - xx[9] * (xx[5] * xx[87] - xx[0] * xx[88]);
  motionData[124] = xx[64] + xx[9] * (xx[79] - xx[81]);
  motionData[125] = xx[72];
  motionData[126] = xx[74] - xx[9] * (xx[77] + xx[82]);
  motionData[127] = xx[83] + xx[9] * (xx[0] * xx[85] - xx[5] * xx[89]);
  motionData[128] = xx[95] + xx[76];
  motionData[129] = xx[84] - xx[9] * (xx[0] * xx[89] + xx[5] * xx[85]);
}

size_t la_2dof_simul_e38647b6_1_computeAssemblyError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const double *state,
  const int *modeVector, const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t la_2dof_simul_e38647b6_1_computeAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, boolean_T
  forVelocitySatisfaction, const double *state, const int *modeVector, const
  double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t la_2dof_simul_e38647b6_1_computeFullAssemblyJacobian(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const double *state, const int
  *modeVector, const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) J;
  return 0;
}

int la_2dof_simul_e38647b6_1_isInKinematicSingularity(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData)
{
  (void) mech;
  (void) rtdv
    ;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
  }

  return 0;
}

PmfMessageId la_2dof_simul_e38647b6_1_convertStateVector(const void *asmMech,
  const RuntimeDerivedValuesBundle *rtdv, const void *simMech, const double
  *asmState, const int *asmModeVector, const int *simModeVector, double
  *simState, void *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  (void) neDiagMgr;
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  return NULL;
}
