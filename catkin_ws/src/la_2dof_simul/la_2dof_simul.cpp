//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: la_2dof_simul.cpp
//
// Code generated for Simulink model 'la_2dof_simul'.
//
// Model version                  : 1.82
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Sun Jun 14 22:54:11 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "la_2dof_simul.h"
#include "la_2dof_simul_private.h"

// Block signals (default storage)
B_la_2dof_simul_T la_2dof_simul_B;

// Continuous states
X_la_2dof_simul_T la_2dof_simul_X;

// Block states (default storage)
DW_la_2dof_simul_T la_2dof_simul_DW;

// Real-time model
RT_MODEL_la_2dof_simul_T la_2dof_simul_M_ = RT_MODEL_la_2dof_simul_T();
RT_MODEL_la_2dof_simul_T *const la_2dof_simul_M = &la_2dof_simul_M_;

// Forward declaration for local functions
static void la_2dof_simul_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[2], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_la_2dof_simul_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void matlabCodegenHandle_matlabCod_d(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);

// Projection for root system: '<Root>'
void la_2dof_simul_projection(void)
{
  NeslSimulationData *simulationData;
  real_T time;
  boolean_T tmp;
  int_T tmp_0[3];
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  int32_T tmp_1;
  char *msg;

  // Projection for SimscapeExecutionBlock: '<S24>/STATE_1'
  simulationData = (NeslSimulationData *)la_2dof_simul_DW.STATE_1_SimData;
  time = la_2dof_simul_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 4;
  simulationData->mData->mContStates.mX =
    &la_2dof_simul_X.la_2dof_simulla_1_jointRzq[0];
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &la_2dof_simul_DW.STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX = &la_2dof_simul_DW.STATE_1_Modes;
  tmp = false;
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep(la_2dof_simul_M);
  tmp = false;
  simulationData->mData->mIsSolverAssertCheck = tmp;
  simulationData->mData->mIsSolverCheckingCIC = false;
  tmp = rtsiIsSolverComputingJacobian(&la_2dof_simul_M->solverInfo);
  simulationData->mData->mIsComputingJacobian = tmp;
  simulationData->mData->mIsEvaluatingF0 = false;
  simulationData->mData->mIsSolverRequestingReset = false;
  tmp_0[0] = 0;
  la_2dof_simul_B.dv3[0] = la_2dof_simul_B.INPUT_1_1_1[0];
  la_2dof_simul_B.dv3[1] = la_2dof_simul_B.INPUT_1_1_1[1];
  la_2dof_simul_B.dv3[2] = la_2dof_simul_B.INPUT_1_1_1[2];
  la_2dof_simul_B.dv3[3] = la_2dof_simul_B.INPUT_1_1_1[3];
  tmp_0[1] = 4;
  la_2dof_simul_B.dv3[4] = la_2dof_simul_B.INPUT_2_1_1[0];
  la_2dof_simul_B.dv3[5] = la_2dof_simul_B.INPUT_2_1_1[1];
  la_2dof_simul_B.dv3[6] = la_2dof_simul_B.INPUT_2_1_1[2];
  la_2dof_simul_B.dv3[7] = la_2dof_simul_B.INPUT_2_1_1[3];
  tmp_0[2] = 8;
  simulationData->mData->mInputValues.mN = 8;
  simulationData->mData->mInputValues.mX = &la_2dof_simul_B.dv3[0];
  simulationData->mData->mInputOffsets.mN = 3;
  simulationData->mData->mInputOffsets.mX = &tmp_0[0];
  diagnosticManager = (NeuDiagnosticManager *)la_2dof_simul_DW.STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_1 = ne_simulator_method((NeslSimulator *)
    la_2dof_simul_DW.STATE_1_Simulator, NESL_SIM_PROJECTION, simulationData,
    diagnosticManager);
  if (tmp_1 != 0) {
    tmp = error_buffer_is_empty(rtmGetErrorStatus(la_2dof_simul_M));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      rtmSetErrorStatus(la_2dof_simul_M, msg);
    }
  }

  // End of Projection for SimscapeExecutionBlock: '<S24>/STATE_1'
}

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  la_2dof_simul_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  la_2dof_simul_step();
  la_2dof_simul_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  la_2dof_simul_step();
  la_2dof_simul_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  la_2dof_simul_step();
  la_2dof_simul_projection();
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void la_2dof_simul_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[2], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_la_2dof_simul_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_la_2dof_simul_193.getLatestMessage
    (&la_2dof_simul_B.b_varargout_2);
  varargout_2_Data[0] = la_2dof_simul_B.b_varargout_2.Data[0];
  varargout_2_Data[1] = la_2dof_simul_B.b_varargout_2.Data[1];
  *varargout_2_Data_SL_Info_Curren =
    la_2dof_simul_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    la_2dof_simul_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    la_2dof_simul_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0], &la_2dof_simul_B.b_varargout_2.Layout.Dim[0],
         sizeof(SL_Bus_la_2dof_simul_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    la_2dof_simul_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    la_2dof_simul_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void matlabCodegenHandle_matlabCod_d(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void la_2dof_simul_step(void)
{
  if (rtmIsMajorTimeStep(la_2dof_simul_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&la_2dof_simul_M->solverInfo,
                          ((la_2dof_simul_M->Timing.clockTick0+1)*
      la_2dof_simul_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(la_2dof_simul_M)) {
    la_2dof_simul_M->Timing.t[0] = rtsiGetT(&la_2dof_simul_M->solverInfo);
  }

  {
    NeslSimulationData *simulationData;
    boolean_T tmp;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    int32_T tmp_0;
    char *msg;
    uint32_T b_varargout_2_Data_SL_Info_Curr;
    uint32_T b_varargout_2_Data_SL_Info_Rece;
    uint32_T b_varargout_2_Layout_DataOffset;
    uint32_T b_varargout_2_Layout_Dim_SL_Inf;
    uint32_T b_varargout_2_Layout_Dim_SL_I_0;
    boolean_T b_varargout_1;

    // SimscapeExecutionBlock: '<S24>/STATE_1' incorporates:
    //   SimscapeExecutionBlock: '<S24>/OUTPUT_1_0'

    simulationData = (NeslSimulationData *)la_2dof_simul_DW.STATE_1_SimData;
    la_2dof_simul_B.time_tmp = la_2dof_simul_M->Timing.t[0];
    la_2dof_simul_B.time = la_2dof_simul_B.time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &la_2dof_simul_B.time;
    simulationData->mData->mContStates.mN = 4;
    simulationData->mData->mContStates.mX =
      &la_2dof_simul_X.la_2dof_simulla_1_jointRzq[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &la_2dof_simul_DW.STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &la_2dof_simul_DW.STATE_1_Modes;
    tmp = false;
    simulationData->mData->mFoundZcEvents = tmp;
    tmp = rtmIsMajorTimeStep(la_2dof_simul_M);
    simulationData->mData->mIsMajorTimeStep = tmp;
    b_varargout_1 = false;
    simulationData->mData->mIsSolverAssertCheck = b_varargout_1;
    simulationData->mData->mIsSolverCheckingCIC = false;
    b_varargout_1 = rtsiIsSolverComputingJacobian(&la_2dof_simul_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = b_varargout_1;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    la_2dof_simul_B.iv1[0] = 0;
    la_2dof_simul_B.dv1[0] = la_2dof_simul_B.INPUT_1_1_1[0];
    la_2dof_simul_B.dv1[1] = la_2dof_simul_B.INPUT_1_1_1[1];
    la_2dof_simul_B.dv1[2] = la_2dof_simul_B.INPUT_1_1_1[2];
    la_2dof_simul_B.dv1[3] = la_2dof_simul_B.INPUT_1_1_1[3];
    la_2dof_simul_B.iv1[1] = 4;
    la_2dof_simul_B.dv1[4] = la_2dof_simul_B.INPUT_2_1_1[0];
    la_2dof_simul_B.dv1[5] = la_2dof_simul_B.INPUT_2_1_1[1];
    la_2dof_simul_B.dv1[6] = la_2dof_simul_B.INPUT_2_1_1[2];
    la_2dof_simul_B.dv1[7] = la_2dof_simul_B.INPUT_2_1_1[3];
    la_2dof_simul_B.iv1[2] = 8;
    simulationData->mData->mInputValues.mN = 8;
    simulationData->mData->mInputValues.mX = &la_2dof_simul_B.dv1[0];
    simulationData->mData->mInputOffsets.mN = 3;
    simulationData->mData->mInputOffsets.mX = &la_2dof_simul_B.iv1[0];
    simulationData->mData->mOutputs.mN = 4;
    simulationData->mData->mOutputs.mX = &la_2dof_simul_B.STATE_1[0];
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    diagnosticManager = (NeuDiagnosticManager *)la_2dof_simul_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_0 = ne_simulator_method((NeslSimulator *)
      la_2dof_simul_DW.STATE_1_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diagnosticManager);
    if (tmp_0 != 0) {
      b_varargout_1 = error_buffer_is_empty(rtmGetErrorStatus(la_2dof_simul_M));
      if (b_varargout_1) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(la_2dof_simul_M, msg);
      }
    }

    // End of SimscapeExecutionBlock: '<S24>/STATE_1'
    if (rtmIsMajorTimeStep(la_2dof_simul_M)) {
      // Outputs for Atomic SubSystem: '<Root>/Subscribe'
      // MATLABSystem: '<S8>/SourceBlock' incorporates:
      //   Inport: '<S25>/In1'

      la_2dof_simul_SystemCore_step(&b_varargout_1,
        la_2dof_simul_B.b_varargout_2_Data, &b_varargout_2_Data_SL_Info_Curr,
        &b_varargout_2_Data_SL_Info_Rece, &b_varargout_2_Layout_DataOffset,
        la_2dof_simul_B.b_varargout_2_Layout_Dim,
        &b_varargout_2_Layout_Dim_SL_Inf, &b_varargout_2_Layout_Dim_SL_I_0);

      // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S25>/Enable'

      if (b_varargout_1) {
        la_2dof_simul_B.In1.Data[0] = la_2dof_simul_B.b_varargout_2_Data[0];
        la_2dof_simul_B.In1.Data[1] = la_2dof_simul_B.b_varargout_2_Data[1];
        la_2dof_simul_B.In1.Data_SL_Info.CurrentLength =
          b_varargout_2_Data_SL_Info_Curr;
        la_2dof_simul_B.In1.Data_SL_Info.ReceivedLength =
          b_varargout_2_Data_SL_Info_Rece;
        la_2dof_simul_B.In1.Layout.DataOffset = b_varargout_2_Layout_DataOffset;
        memcpy(&la_2dof_simul_B.In1.Layout.Dim[0],
               &la_2dof_simul_B.b_varargout_2_Layout_Dim[0], sizeof
               (SL_Bus_la_2dof_simul_std_msgs_MultiArrayDimension) << 4U);
        la_2dof_simul_B.In1.Layout.Dim_SL_Info.CurrentLength =
          b_varargout_2_Layout_Dim_SL_Inf;
        la_2dof_simul_B.In1.Layout.Dim_SL_Info.ReceivedLength =
          b_varargout_2_Layout_Dim_SL_I_0;
      }

      // End of MATLABSystem: '<S8>/SourceBlock'
      // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<Root>/Subscribe'
    }

    // SimscapeInputBlock: '<S24>/INPUT_1_1_1'
    la_2dof_simul_B.INPUT_1_1_1[0] = la_2dof_simul_B.In1.Data[0];
    la_2dof_simul_B.INPUT_1_1_1[1] = 0.0;
    la_2dof_simul_B.INPUT_1_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(la_2dof_simul_M)) {
      la_2dof_simul_DW.INPUT_1_1_1_Discrete[0] = !(la_2dof_simul_B.INPUT_1_1_1[0]
        == la_2dof_simul_DW.INPUT_1_1_1_Discrete[1]);
      la_2dof_simul_DW.INPUT_1_1_1_Discrete[1] = la_2dof_simul_B.INPUT_1_1_1[0];
    }

    la_2dof_simul_B.INPUT_1_1_1[0] = la_2dof_simul_DW.INPUT_1_1_1_Discrete[1];
    la_2dof_simul_B.INPUT_1_1_1[3] = la_2dof_simul_DW.INPUT_1_1_1_Discrete[0];

    // End of SimscapeInputBlock: '<S24>/INPUT_1_1_1'

    // SimscapeInputBlock: '<S24>/INPUT_2_1_1'
    la_2dof_simul_B.INPUT_2_1_1[0] = la_2dof_simul_B.In1.Data[1];
    la_2dof_simul_B.INPUT_2_1_1[1] = 0.0;
    la_2dof_simul_B.INPUT_2_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(la_2dof_simul_M)) {
      la_2dof_simul_DW.INPUT_2_1_1_Discrete[0] = !(la_2dof_simul_B.INPUT_2_1_1[0]
        == la_2dof_simul_DW.INPUT_2_1_1_Discrete[1]);
      la_2dof_simul_DW.INPUT_2_1_1_Discrete[1] = la_2dof_simul_B.INPUT_2_1_1[0];
    }

    la_2dof_simul_B.INPUT_2_1_1[0] = la_2dof_simul_DW.INPUT_2_1_1_Discrete[1];
    la_2dof_simul_B.INPUT_2_1_1[3] = la_2dof_simul_DW.INPUT_2_1_1_Discrete[0];

    // End of SimscapeInputBlock: '<S24>/INPUT_2_1_1'

    // SimscapeExecutionBlock: '<S24>/OUTPUT_1_0'
    simulationData = (NeslSimulationData *)la_2dof_simul_DW.OUTPUT_1_0_SimData;
    la_2dof_simul_B.time_c = la_2dof_simul_B.time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &la_2dof_simul_B.time_c;
    simulationData->mData->mContStates.mN = 0;
    simulationData->mData->mContStates.mX = NULL;
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX =
      &la_2dof_simul_DW.OUTPUT_1_0_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &la_2dof_simul_DW.OUTPUT_1_0_Modes;
    b_varargout_1 = false;
    simulationData->mData->mFoundZcEvents = b_varargout_1;
    simulationData->mData->mIsMajorTimeStep = tmp;
    tmp = false;
    simulationData->mData->mIsSolverAssertCheck = tmp;
    simulationData->mData->mIsSolverCheckingCIC = false;
    simulationData->mData->mIsComputingJacobian = false;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    la_2dof_simul_B.iv[0] = 0;
    la_2dof_simul_B.dv[0] = la_2dof_simul_B.INPUT_1_1_1[0];
    la_2dof_simul_B.dv[1] = la_2dof_simul_B.INPUT_1_1_1[1];
    la_2dof_simul_B.dv[2] = la_2dof_simul_B.INPUT_1_1_1[2];
    la_2dof_simul_B.dv[3] = la_2dof_simul_B.INPUT_1_1_1[3];
    la_2dof_simul_B.iv[1] = 4;
    la_2dof_simul_B.dv[4] = la_2dof_simul_B.INPUT_2_1_1[0];
    la_2dof_simul_B.dv[5] = la_2dof_simul_B.INPUT_2_1_1[1];
    la_2dof_simul_B.dv[6] = la_2dof_simul_B.INPUT_2_1_1[2];
    la_2dof_simul_B.dv[7] = la_2dof_simul_B.INPUT_2_1_1[3];
    la_2dof_simul_B.iv[2] = 8;
    la_2dof_simul_B.dv[8] = la_2dof_simul_B.STATE_1[0];
    la_2dof_simul_B.dv[9] = la_2dof_simul_B.STATE_1[1];
    la_2dof_simul_B.dv[10] = la_2dof_simul_B.STATE_1[2];
    la_2dof_simul_B.dv[11] = la_2dof_simul_B.STATE_1[3];
    la_2dof_simul_B.iv[3] = 12;
    simulationData->mData->mInputValues.mN = 12;
    simulationData->mData->mInputValues.mX = &la_2dof_simul_B.dv[0];
    simulationData->mData->mInputOffsets.mN = 4;
    simulationData->mData->mInputOffsets.mX = &la_2dof_simul_B.iv[0];
    simulationData->mData->mOutputs.mN = 2;
    simulationData->mData->mOutputs.mX = &la_2dof_simul_B.OUTPUT_1_0[0];
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    diagnosticManager = (NeuDiagnosticManager *)
      la_2dof_simul_DW.OUTPUT_1_0_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_0 = ne_simulator_method((NeslSimulator *)
      la_2dof_simul_DW.OUTPUT_1_0_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diagnosticManager);
    if (tmp_0 != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus(la_2dof_simul_M));
      if (tmp) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(la_2dof_simul_M, msg);
      }
    }

    if (rtmIsMajorTimeStep(la_2dof_simul_M)) {
      // BusAssignment: '<Root>/Bus Assignment1' incorporates:
      //   Constant: '<Root>/Constant'
      //   Constant: '<S1>/Constant'

      la_2dof_simul_B.BusAssignment1 = la_2dof_simul_P.Constant_Value_b;
      la_2dof_simul_B.BusAssignment1.Data[0] = la_2dof_simul_B.OUTPUT_1_0[0];
      la_2dof_simul_B.BusAssignment1.Data[1] = la_2dof_simul_B.OUTPUT_1_0[1];
      la_2dof_simul_B.BusAssignment1.Data_SL_Info.CurrentLength =
        la_2dof_simul_P.Constant_Value_d;
      la_2dof_simul_B.BusAssignment1.Data_SL_Info.ReceivedLength =
        la_2dof_simul_P.Constant_Value_d;

      // Outputs for Atomic SubSystem: '<Root>/Publish1'
      // MATLABSystem: '<S4>/SinkBlock'
      Pub_la_2dof_simul_198.publish(&la_2dof_simul_B.BusAssignment1);

      // End of Outputs for SubSystem: '<Root>/Publish1'
    }
  }

  if (rtmIsMajorTimeStep(la_2dof_simul_M)) {
    NeslSimulationData *simulationData;
    real_T time;
    boolean_T tmp;
    int_T tmp_0[3];
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    int32_T tmp_1;
    char *msg;

    // Update for SimscapeExecutionBlock: '<S24>/STATE_1'
    simulationData = (NeslSimulationData *)la_2dof_simul_DW.STATE_1_SimData;
    time = la_2dof_simul_M->Timing.t[0];
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 4;
    simulationData->mData->mContStates.mX =
      &la_2dof_simul_X.la_2dof_simulla_1_jointRzq[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &la_2dof_simul_DW.STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &la_2dof_simul_DW.STATE_1_Modes;
    tmp = false;
    simulationData->mData->mFoundZcEvents = tmp;
    simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep(la_2dof_simul_M);
    tmp = false;
    simulationData->mData->mIsSolverAssertCheck = tmp;
    simulationData->mData->mIsSolverCheckingCIC = false;
    tmp = rtsiIsSolverComputingJacobian(&la_2dof_simul_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = tmp;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    tmp_0[0] = 0;
    la_2dof_simul_B.dv5[0] = la_2dof_simul_B.INPUT_1_1_1[0];
    la_2dof_simul_B.dv5[1] = la_2dof_simul_B.INPUT_1_1_1[1];
    la_2dof_simul_B.dv5[2] = la_2dof_simul_B.INPUT_1_1_1[2];
    la_2dof_simul_B.dv5[3] = la_2dof_simul_B.INPUT_1_1_1[3];
    tmp_0[1] = 4;
    la_2dof_simul_B.dv5[4] = la_2dof_simul_B.INPUT_2_1_1[0];
    la_2dof_simul_B.dv5[5] = la_2dof_simul_B.INPUT_2_1_1[1];
    la_2dof_simul_B.dv5[6] = la_2dof_simul_B.INPUT_2_1_1[2];
    la_2dof_simul_B.dv5[7] = la_2dof_simul_B.INPUT_2_1_1[3];
    tmp_0[2] = 8;
    simulationData->mData->mInputValues.mN = 8;
    simulationData->mData->mInputValues.mX = &la_2dof_simul_B.dv5[0];
    simulationData->mData->mInputOffsets.mN = 3;
    simulationData->mData->mInputOffsets.mX = &tmp_0[0];
    diagnosticManager = (NeuDiagnosticManager *)la_2dof_simul_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_1 = ne_simulator_method((NeslSimulator *)
      la_2dof_simul_DW.STATE_1_Simulator, NESL_SIM_UPDATE, simulationData,
      diagnosticManager);
    if (tmp_1 != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus(la_2dof_simul_M));
      if (tmp) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(la_2dof_simul_M, msg);
      }
    }

    // End of Update for SimscapeExecutionBlock: '<S24>/STATE_1'
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(la_2dof_simul_M)) {
    rt_ertODEUpdateContinuousStates(&la_2dof_simul_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++la_2dof_simul_M->Timing.clockTick0;
    la_2dof_simul_M->Timing.t[0] = rtsiGetSolverStopTime
      (&la_2dof_simul_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.001s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.001, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      la_2dof_simul_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void la_2dof_simul_derivatives(void)
{
  NeslSimulationData *simulationData;
  real_T time;
  boolean_T tmp;
  int_T tmp_0[3];
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  int32_T tmp_1;
  char *msg;
  XDot_la_2dof_simul_T *_rtXdot;
  _rtXdot = ((XDot_la_2dof_simul_T *) la_2dof_simul_M->derivs);

  // Derivatives for SimscapeExecutionBlock: '<S24>/STATE_1'
  simulationData = (NeslSimulationData *)la_2dof_simul_DW.STATE_1_SimData;
  time = la_2dof_simul_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 4;
  simulationData->mData->mContStates.mX =
    &la_2dof_simul_X.la_2dof_simulla_1_jointRzq[0];
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &la_2dof_simul_DW.STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX = &la_2dof_simul_DW.STATE_1_Modes;
  tmp = false;
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep(la_2dof_simul_M);
  tmp = false;
  simulationData->mData->mIsSolverAssertCheck = tmp;
  simulationData->mData->mIsSolverCheckingCIC = false;
  tmp = rtsiIsSolverComputingJacobian(&la_2dof_simul_M->solverInfo);
  simulationData->mData->mIsComputingJacobian = tmp;
  simulationData->mData->mIsEvaluatingF0 = false;
  simulationData->mData->mIsSolverRequestingReset = false;
  tmp_0[0] = 0;
  la_2dof_simul_B.dv4[0] = la_2dof_simul_B.INPUT_1_1_1[0];
  la_2dof_simul_B.dv4[1] = la_2dof_simul_B.INPUT_1_1_1[1];
  la_2dof_simul_B.dv4[2] = la_2dof_simul_B.INPUT_1_1_1[2];
  la_2dof_simul_B.dv4[3] = la_2dof_simul_B.INPUT_1_1_1[3];
  tmp_0[1] = 4;
  la_2dof_simul_B.dv4[4] = la_2dof_simul_B.INPUT_2_1_1[0];
  la_2dof_simul_B.dv4[5] = la_2dof_simul_B.INPUT_2_1_1[1];
  la_2dof_simul_B.dv4[6] = la_2dof_simul_B.INPUT_2_1_1[2];
  la_2dof_simul_B.dv4[7] = la_2dof_simul_B.INPUT_2_1_1[3];
  tmp_0[2] = 8;
  simulationData->mData->mInputValues.mN = 8;
  simulationData->mData->mInputValues.mX = &la_2dof_simul_B.dv4[0];
  simulationData->mData->mInputOffsets.mN = 3;
  simulationData->mData->mInputOffsets.mX = &tmp_0[0];
  simulationData->mData->mDx.mN = 4;
  simulationData->mData->mDx.mX = &_rtXdot->la_2dof_simulla_1_jointRzq[0];
  diagnosticManager = (NeuDiagnosticManager *)la_2dof_simul_DW.STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_1 = ne_simulator_method((NeslSimulator *)
    la_2dof_simul_DW.STATE_1_Simulator, NESL_SIM_DERIVATIVES, simulationData,
    diagnosticManager);
  if (tmp_1 != 0) {
    tmp = error_buffer_is_empty(rtmGetErrorStatus(la_2dof_simul_M));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      rtmSetErrorStatus(la_2dof_simul_M, msg);
    }
  }

  // End of Derivatives for SimscapeExecutionBlock: '<S24>/STATE_1'
}

// Model initialize function
void la_2dof_simul_initialize(void)
{
  // Registration code
  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&la_2dof_simul_M->solverInfo,
                          &la_2dof_simul_M->Timing.simTimeStep);
    rtsiSetTPtr(&la_2dof_simul_M->solverInfo, &rtmGetTPtr(la_2dof_simul_M));
    rtsiSetStepSizePtr(&la_2dof_simul_M->solverInfo,
                       &la_2dof_simul_M->Timing.stepSize0);
    rtsiSetdXPtr(&la_2dof_simul_M->solverInfo, &la_2dof_simul_M->derivs);
    rtsiSetContStatesPtr(&la_2dof_simul_M->solverInfo, (real_T **)
                         &la_2dof_simul_M->contStates);
    rtsiSetNumContStatesPtr(&la_2dof_simul_M->solverInfo,
      &la_2dof_simul_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&la_2dof_simul_M->solverInfo,
      &la_2dof_simul_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&la_2dof_simul_M->solverInfo,
      &la_2dof_simul_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&la_2dof_simul_M->solverInfo,
      &la_2dof_simul_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&la_2dof_simul_M->solverInfo, (&rtmGetErrorStatus
      (la_2dof_simul_M)));
    rtsiSetRTModelPtr(&la_2dof_simul_M->solverInfo, la_2dof_simul_M);
  }

  rtsiSetSimTimeStep(&la_2dof_simul_M->solverInfo, MAJOR_TIME_STEP);
  la_2dof_simul_M->intgData.y = la_2dof_simul_M->odeY;
  la_2dof_simul_M->intgData.f[0] = la_2dof_simul_M->odeF[0];
  la_2dof_simul_M->intgData.f[1] = la_2dof_simul_M->odeF[1];
  la_2dof_simul_M->intgData.f[2] = la_2dof_simul_M->odeF[2];
  la_2dof_simul_M->contStates = ((X_la_2dof_simul_T *) &la_2dof_simul_X);
  rtsiSetSolverData(&la_2dof_simul_M->solverInfo, static_cast<void *>
                    (&la_2dof_simul_M->intgData));
  rtsiSetSolverName(&la_2dof_simul_M->solverInfo,"ode3");
  rtmSetTPtr(la_2dof_simul_M, &la_2dof_simul_M->Timing.tArray[0]);
  la_2dof_simul_M->Timing.stepSize0 = 0.001;

  {
    NeslSimulator *tmp;
    boolean_T tmp_0;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    int32_T i;
    char *msg;
    NeslSimulationData *simulationData;
    boolean_T tmp_1;
    real_T time;
    real_T time_tmp;
    static const char_T tmp_2[19] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 't', 'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_3[25] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'c', 'u', 'r', 'r', 'e', 'n', 't', '_', 'p', 'o',
      's', 'e' };

    // Start for SimscapeExecutionBlock: '<S24>/STATE_1' incorporates:
    //   SimscapeExecutionBlock: '<S24>/OUTPUT_1_0'

    tmp = nesl_lease_simulator("la_2dof_simul/Solver Configuration_1", 0, 0);
    la_2dof_simul_DW.STATE_1_Simulator = (void *)tmp;
    tmp_0 = pointer_is_null(la_2dof_simul_DW.STATE_1_Simulator);
    if (tmp_0) {
      la_2dof_simul_e38647b6_1_gateway();
      tmp = nesl_lease_simulator("la_2dof_simul/Solver Configuration_1", 0, 0);
      la_2dof_simul_DW.STATE_1_Simulator = (void *)tmp;
    }

    simulationData = nesl_create_simulation_data();
    la_2dof_simul_DW.STATE_1_SimData = (void *)simulationData;
    diagnosticManager = rtw_create_diagnostics();
    la_2dof_simul_DW.STATE_1_DiagMgr = (void *)diagnosticManager;
    la_2dof_simul_B.modelParameters.mSolverType = NE_SOLVER_TYPE_DAE;
    la_2dof_simul_B.modelParameters.mSolverTolerance = 0.001;
    la_2dof_simul_B.modelParameters.mVariableStepSolver = false;
    la_2dof_simul_B.modelParameters.mFixedStepSize = 0.001;
    la_2dof_simul_B.modelParameters.mStartTime = 0.0;
    la_2dof_simul_B.modelParameters.mLoadInitialState = false;
    la_2dof_simul_B.modelParameters.mUseSimState = false;
    la_2dof_simul_B.modelParameters.mLinTrimCompile = false;
    la_2dof_simul_B.modelParameters.mLoggingMode = SSC_LOGGING_NONE;
    la_2dof_simul_B.modelParameters.mRTWModifiedTimeStamp = 5.14076005E+8;
    la_2dof_simul_B.d = 0.001;
    la_2dof_simul_B.modelParameters.mSolverTolerance = la_2dof_simul_B.d;
    la_2dof_simul_B.d = 0.001;
    la_2dof_simul_B.modelParameters.mFixedStepSize = la_2dof_simul_B.d;
    tmp_0 = false;
    la_2dof_simul_B.modelParameters.mVariableStepSolver = tmp_0;
    diagnosticManager = (NeuDiagnosticManager *)la_2dof_simul_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = nesl_initialize_simulator((NeslSimulator *)
      la_2dof_simul_DW.STATE_1_Simulator, &la_2dof_simul_B.modelParameters,
      diagnosticManager);
    if (i != 0) {
      tmp_0 = error_buffer_is_empty(rtmGetErrorStatus(la_2dof_simul_M));
      if (tmp_0) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(la_2dof_simul_M, msg);
      }
    }

    la_2dof_simul_B.expl_temp.mRealParameters.mN = 0;
    la_2dof_simul_B.expl_temp.mRealParameters.mX = NULL;
    la_2dof_simul_B.expl_temp.mLogicalParameters.mN = 0;
    la_2dof_simul_B.expl_temp.mLogicalParameters.mX = NULL;
    la_2dof_simul_B.expl_temp.mIntegerParameters.mN = 0;
    la_2dof_simul_B.expl_temp.mIntegerParameters.mX = NULL;
    la_2dof_simul_B.expl_temp.mIndexParameters.mN = 0;
    la_2dof_simul_B.expl_temp.mIndexParameters.mX = NULL;
    nesl_simulator_set_rtps((NeslSimulator *)la_2dof_simul_DW.STATE_1_Simulator,
      la_2dof_simul_B.expl_temp);
    simulationData = (NeslSimulationData *)la_2dof_simul_DW.STATE_1_SimData;
    time_tmp = la_2dof_simul_M->Timing.t[0];
    la_2dof_simul_B.time_k = time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &la_2dof_simul_B.time_k;
    simulationData->mData->mContStates.mN = 4;
    simulationData->mData->mContStates.mX =
      &la_2dof_simul_X.la_2dof_simulla_1_jointRzq[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &la_2dof_simul_DW.STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &la_2dof_simul_DW.STATE_1_Modes;
    tmp_0 = false;
    simulationData->mData->mFoundZcEvents = tmp_0;
    tmp_0 = rtmIsMajorTimeStep(la_2dof_simul_M);
    simulationData->mData->mIsMajorTimeStep = tmp_0;
    tmp_1 = false;
    simulationData->mData->mIsSolverAssertCheck = tmp_1;
    simulationData->mData->mIsSolverCheckingCIC = false;
    tmp_1 = rtsiIsSolverComputingJacobian(&la_2dof_simul_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = tmp_1;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    diagnosticManager = (NeuDiagnosticManager *)la_2dof_simul_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = ne_simulator_method((NeslSimulator *)la_2dof_simul_DW.STATE_1_Simulator,
      NESL_SIM_INITIALIZEONCE, simulationData, diagnosticManager);
    if (i != 0) {
      tmp_1 = error_buffer_is_empty(rtmGetErrorStatus(la_2dof_simul_M));
      if (tmp_1) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(la_2dof_simul_M, msg);
      }
    }

    // End of Start for SimscapeExecutionBlock: '<S24>/STATE_1'

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S8>/SourceBlock'
    la_2dof_simul_DW.obj_a.matlabCodegenIsDeleted = true;
    la_2dof_simul_DW.obj_a.isInitialized = 0;
    la_2dof_simul_DW.obj_a.matlabCodegenIsDeleted = false;
    la_2dof_simul_DW.obj_a.isSetupComplete = false;
    la_2dof_simul_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      la_2dof_simul_B.cv1[i] = tmp_2[i];
    }

    la_2dof_simul_B.cv1[19] = '\x00';
    Sub_la_2dof_simul_193.createSubscriber(la_2dof_simul_B.cv1, 1);
    la_2dof_simul_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for SimscapeExecutionBlock: '<S24>/OUTPUT_1_0'
    tmp = nesl_lease_simulator("la_2dof_simul/Solver Configuration_1", 1, 0);
    la_2dof_simul_DW.OUTPUT_1_0_Simulator = (void *)tmp;
    tmp_1 = pointer_is_null(la_2dof_simul_DW.OUTPUT_1_0_Simulator);
    if (tmp_1) {
      la_2dof_simul_e38647b6_1_gateway();
      tmp = nesl_lease_simulator("la_2dof_simul/Solver Configuration_1", 1, 0);
      la_2dof_simul_DW.OUTPUT_1_0_Simulator = (void *)tmp;
    }

    simulationData = nesl_create_simulation_data();
    la_2dof_simul_DW.OUTPUT_1_0_SimData = (void *)simulationData;
    diagnosticManager = rtw_create_diagnostics();
    la_2dof_simul_DW.OUTPUT_1_0_DiagMgr = (void *)diagnosticManager;
    la_2dof_simul_B.modelParameters_m.mSolverType = NE_SOLVER_TYPE_DAE;
    la_2dof_simul_B.modelParameters_m.mSolverTolerance = 0.001;
    la_2dof_simul_B.modelParameters_m.mVariableStepSolver = false;
    la_2dof_simul_B.modelParameters_m.mFixedStepSize = 0.001;
    la_2dof_simul_B.modelParameters_m.mStartTime = 0.0;
    la_2dof_simul_B.modelParameters_m.mLoadInitialState = false;
    la_2dof_simul_B.modelParameters_m.mUseSimState = false;
    la_2dof_simul_B.modelParameters_m.mLinTrimCompile = false;
    la_2dof_simul_B.modelParameters_m.mLoggingMode = SSC_LOGGING_NONE;
    la_2dof_simul_B.modelParameters_m.mRTWModifiedTimeStamp = 5.14076005E+8;
    la_2dof_simul_B.d = 0.001;
    la_2dof_simul_B.modelParameters_m.mSolverTolerance = la_2dof_simul_B.d;
    la_2dof_simul_B.d = 0.001;
    la_2dof_simul_B.modelParameters_m.mFixedStepSize = la_2dof_simul_B.d;
    tmp_1 = false;
    la_2dof_simul_B.modelParameters_m.mVariableStepSolver = tmp_1;
    diagnosticManager = (NeuDiagnosticManager *)
      la_2dof_simul_DW.OUTPUT_1_0_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = nesl_initialize_simulator((NeslSimulator *)
      la_2dof_simul_DW.OUTPUT_1_0_Simulator, &la_2dof_simul_B.modelParameters_m,
      diagnosticManager);
    if (i != 0) {
      tmp_1 = error_buffer_is_empty(rtmGetErrorStatus(la_2dof_simul_M));
      if (tmp_1) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(la_2dof_simul_M, msg);
      }
    }

    simulationData = (NeslSimulationData *)la_2dof_simul_DW.OUTPUT_1_0_SimData;
    time = time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 0;
    simulationData->mData->mContStates.mX = NULL;
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX =
      &la_2dof_simul_DW.OUTPUT_1_0_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &la_2dof_simul_DW.OUTPUT_1_0_Modes;
    tmp_1 = false;
    simulationData->mData->mFoundZcEvents = tmp_1;
    simulationData->mData->mIsMajorTimeStep = tmp_0;
    tmp_0 = false;
    simulationData->mData->mIsSolverAssertCheck = tmp_0;
    simulationData->mData->mIsSolverCheckingCIC = false;
    simulationData->mData->mIsComputingJacobian = false;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    diagnosticManager = (NeuDiagnosticManager *)
      la_2dof_simul_DW.OUTPUT_1_0_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = ne_simulator_method((NeslSimulator *)
      la_2dof_simul_DW.OUTPUT_1_0_Simulator, NESL_SIM_INITIALIZEONCE,
      simulationData, diagnosticManager);
    if (i != 0) {
      tmp_0 = error_buffer_is_empty(rtmGetErrorStatus(la_2dof_simul_M));
      if (tmp_0) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(la_2dof_simul_M, msg);
      }
    }

    // Start for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    la_2dof_simul_DW.obj.matlabCodegenIsDeleted = true;
    la_2dof_simul_DW.obj.isInitialized = 0;
    la_2dof_simul_DW.obj.matlabCodegenIsDeleted = false;
    la_2dof_simul_DW.obj.isSetupComplete = false;
    la_2dof_simul_DW.obj.isInitialized = 1;
    for (i = 0; i < 25; i++) {
      la_2dof_simul_B.cv[i] = tmp_3[i];
    }

    la_2dof_simul_B.cv[25] = '\x00';
    Pub_la_2dof_simul_198.createPublisher(la_2dof_simul_B.cv, 1);
    la_2dof_simul_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish1'

    // InitializeConditions for SimscapeExecutionBlock: '<S24>/STATE_1'
    tmp_0 = false;
    if (tmp_0) {
      i = strcmp("ode3", rtsiGetSolverName(&la_2dof_simul_M->solverInfo));
      if (i != 0) {
        msg = solver_mismatch_message("ode3", rtsiGetSolverName
          (&la_2dof_simul_M->solverInfo));
        rtmSetErrorStatus(la_2dof_simul_M, msg);
      }
    }

    // End of InitializeConditions for SimscapeExecutionBlock: '<S24>/STATE_1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S25>/Out1'
    la_2dof_simul_B.In1 = la_2dof_simul_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'
  }
}

// Model terminate function
void la_2dof_simul_terminate(void)
{
  // Terminate for SimscapeExecutionBlock: '<S24>/STATE_1'
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    la_2dof_simul_DW.STATE_1_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    la_2dof_simul_DW.STATE_1_SimData);
  nesl_erase_simulator("la_2dof_simul/Solver Configuration_1");

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S8>/SourceBlock'
  matlabCodegenHandle_matlabCod_d(&la_2dof_simul_DW.obj_a);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for SimscapeExecutionBlock: '<S24>/OUTPUT_1_0'
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    la_2dof_simul_DW.OUTPUT_1_0_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    la_2dof_simul_DW.OUTPUT_1_0_SimData);
  nesl_erase_simulator("la_2dof_simul/Solver Configuration_1");

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S4>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&la_2dof_simul_DW.obj);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
