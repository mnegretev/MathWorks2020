//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_simul.cpp
//
// Code generated for Simulink model 'left_arm_simul'.
//
// Model version                  : 1.92
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Mon Jun 15 12:53:03 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "left_arm_simul.h"
#include "left_arm_simul_private.h"

// Block signals (default storage)
B_left_arm_simul_T left_arm_simul_B;

// Continuous states
X_left_arm_simul_T left_arm_simul_X;

// Block states (default storage)
DW_left_arm_simul_T left_arm_simul_DW;

// Real-time model
RT_MODEL_left_arm_simul_T left_arm_simul_M_ = RT_MODEL_left_arm_simul_T();
RT_MODEL_left_arm_simul_T *const left_arm_simul_M = &left_arm_simul_M_;

// Forward declaration for local functions
static void left_arm_simul_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_simul_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void matlabCodegenHandle_matlabC_k10(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Curr_T *obj);
static void matlabCodegenHandle_matlabCod_k(ros_slros_internal_block_Publ_T *obj);
static void rate_scheduler(void);

//
//   This function updates active task flag for each subrate.
// The function is called at model base rate, hence the
// generated code self-manages all its subrates.
//
static void rate_scheduler(void)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (left_arm_simul_M->Timing.TaskCounters.TID[2])++;
  if ((left_arm_simul_M->Timing.TaskCounters.TID[2]) > 49) {// Sample time: [0.05s, 0.0s] 
    left_arm_simul_M->Timing.TaskCounters.TID[2] = 0;
  }
}

// Projection for root system: '<Root>'
void left_arm_simul_projection(void)
{
  NeslSimulationData *simulationData;
  real_T time;
  boolean_T tmp;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  int32_T tmp_0;
  char *msg;

  // Projection for SimscapeExecutionBlock: '<S48>/STATE_1'
  simulationData = (NeslSimulationData *)left_arm_simul_DW.STATE_1_SimData;
  time = left_arm_simul_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 14;
  simulationData->mData->mContStates.mX =
    &left_arm_simul_X.left_arm_simulla_1_jointRzq[0];
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &left_arm_simul_DW.STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX = &left_arm_simul_DW.STATE_1_Modes;
  tmp = false;
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep(left_arm_simul_M);
  tmp = false;
  simulationData->mData->mIsSolverAssertCheck = tmp;
  simulationData->mData->mIsSolverCheckingCIC = false;
  tmp = rtsiIsSolverComputingJacobian(&left_arm_simul_M->solverInfo);
  simulationData->mData->mIsComputingJacobian = tmp;
  simulationData->mData->mIsEvaluatingF0 = false;
  simulationData->mData->mIsSolverRequestingReset = false;
  left_arm_simul_B.iv3[0] = 0;
  left_arm_simul_B.dv3[0] = left_arm_simul_B.INPUT_1_1_1[0];
  left_arm_simul_B.dv3[1] = left_arm_simul_B.INPUT_1_1_1[1];
  left_arm_simul_B.dv3[2] = left_arm_simul_B.INPUT_1_1_1[2];
  left_arm_simul_B.dv3[3] = left_arm_simul_B.INPUT_1_1_1[3];
  left_arm_simul_B.iv3[1] = 4;
  left_arm_simul_B.dv3[4] = left_arm_simul_B.INPUT_2_1_1[0];
  left_arm_simul_B.dv3[5] = left_arm_simul_B.INPUT_2_1_1[1];
  left_arm_simul_B.dv3[6] = left_arm_simul_B.INPUT_2_1_1[2];
  left_arm_simul_B.dv3[7] = left_arm_simul_B.INPUT_2_1_1[3];
  left_arm_simul_B.iv3[2] = 8;
  left_arm_simul_B.dv3[8] = left_arm_simul_B.INPUT_3_1_1[0];
  left_arm_simul_B.dv3[9] = left_arm_simul_B.INPUT_3_1_1[1];
  left_arm_simul_B.dv3[10] = left_arm_simul_B.INPUT_3_1_1[2];
  left_arm_simul_B.dv3[11] = left_arm_simul_B.INPUT_3_1_1[3];
  left_arm_simul_B.iv3[3] = 12;
  left_arm_simul_B.dv3[12] = left_arm_simul_B.INPUT_4_1_1[0];
  left_arm_simul_B.dv3[13] = left_arm_simul_B.INPUT_4_1_1[1];
  left_arm_simul_B.dv3[14] = left_arm_simul_B.INPUT_4_1_1[2];
  left_arm_simul_B.dv3[15] = left_arm_simul_B.INPUT_4_1_1[3];
  left_arm_simul_B.iv3[4] = 16;
  left_arm_simul_B.dv3[16] = left_arm_simul_B.INPUT_5_1_1[0];
  left_arm_simul_B.dv3[17] = left_arm_simul_B.INPUT_5_1_1[1];
  left_arm_simul_B.dv3[18] = left_arm_simul_B.INPUT_5_1_1[2];
  left_arm_simul_B.dv3[19] = left_arm_simul_B.INPUT_5_1_1[3];
  left_arm_simul_B.iv3[5] = 20;
  left_arm_simul_B.dv3[20] = left_arm_simul_B.INPUT_6_1_1[0];
  left_arm_simul_B.dv3[21] = left_arm_simul_B.INPUT_6_1_1[1];
  left_arm_simul_B.dv3[22] = left_arm_simul_B.INPUT_6_1_1[2];
  left_arm_simul_B.dv3[23] = left_arm_simul_B.INPUT_6_1_1[3];
  left_arm_simul_B.iv3[6] = 24;
  left_arm_simul_B.dv3[24] = left_arm_simul_B.INPUT_7_1_1[0];
  left_arm_simul_B.dv3[25] = left_arm_simul_B.INPUT_7_1_1[1];
  left_arm_simul_B.dv3[26] = left_arm_simul_B.INPUT_7_1_1[2];
  left_arm_simul_B.dv3[27] = left_arm_simul_B.INPUT_7_1_1[3];
  left_arm_simul_B.iv3[7] = 28;
  simulationData->mData->mInputValues.mN = 28;
  simulationData->mData->mInputValues.mX = &left_arm_simul_B.dv3[0];
  simulationData->mData->mInputOffsets.mN = 8;
  simulationData->mData->mInputOffsets.mX = &left_arm_simul_B.iv3[0];
  diagnosticManager = (NeuDiagnosticManager *)left_arm_simul_DW.STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_0 = ne_simulator_method((NeslSimulator *)
    left_arm_simul_DW.STATE_1_Simulator, NESL_SIM_PROJECTION, simulationData,
    diagnosticManager);
  if (tmp_0 != 0) {
    tmp = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      rtmSetErrorStatus(left_arm_simul_M, msg);
    }
  }

  // End of Projection for SimscapeExecutionBlock: '<S48>/STATE_1'
}

//
// This function updates continuous states using the ODE4 fixed-step
// solver algorithm
//
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = static_cast<ODE4_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 14;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  left_arm_simul_derivatives();

  // f1 = f(t + (h/2), y + (h/2)*f0)
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  left_arm_simul_step();
  left_arm_simul_derivatives();

  // f2 = f(t + (h/2), y + (h/2)*f1)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  left_arm_simul_step();
  left_arm_simul_derivatives();

  // f3 = f(t + h, y + h*f2)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  left_arm_simul_step();
  left_arm_simul_derivatives();

  // tnew = t + h
  // ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3)
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  left_arm_simul_step();
  left_arm_simul_projection();
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void left_arm_simul_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_simul_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  int32_T i;
  *varargout_1 = Sub_left_arm_simul_160.getLatestMessage
    (&left_arm_simul_B.b_varargout_2);
  for (i = 0; i < 7; i++) {
    varargout_2_Data[i] = left_arm_simul_B.b_varargout_2.Data[i];
  }

  *varargout_2_Data_SL_Info_Curren =
    left_arm_simul_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    left_arm_simul_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    left_arm_simul_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0], &left_arm_simul_B.b_varargout_2.Layout.Dim
         [0], sizeof(SL_Bus_left_arm_simul_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    left_arm_simul_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    left_arm_simul_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void matlabCodegenHandle_matlabC_k10(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Curr_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCod_k(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

// Model step function
void left_arm_simul_step(void)
{
  if (rtmIsMajorTimeStep(left_arm_simul_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&left_arm_simul_M->solverInfo,
                          ((left_arm_simul_M->Timing.clockTick0+1)*
      left_arm_simul_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(left_arm_simul_M)) {
    left_arm_simul_M->Timing.t[0] = rtsiGetT(&left_arm_simul_M->solverInfo);
  }

  {
    NeslSimulationData *simulationData;
    boolean_T tmp;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    char *msg;
    uint32_T b_varargout_2_Data_SL_Info_Curr;
    uint32_T b_varargout_2_Data_SL_Info_Rece;
    uint32_T b_varargout_2_Layout_DataOffset;
    uint32_T b_varargout_2_Layout_Dim_SL_Inf;
    uint32_T b_varargout_2_Layout_Dim_SL_I_0;
    boolean_T b_varargout_1;
    int32_T i;
    static const uint8_T b[10] = { 108U, 97U, 95U, 49U, 95U, 106U, 111U, 105U,
      110U, 116U };

    static const uint8_T c[10] = { 108U, 97U, 95U, 50U, 95U, 106U, 111U, 105U,
      110U, 116U };

    static const uint8_T d[10] = { 108U, 97U, 95U, 51U, 95U, 106U, 111U, 105U,
      110U, 116U };

    static const uint8_T e[10] = { 108U, 97U, 95U, 52U, 95U, 106U, 111U, 105U,
      110U, 116U };

    static const uint8_T f[10] = { 108U, 97U, 95U, 53U, 95U, 106U, 111U, 105U,
      110U, 116U };

    static const uint8_T g[10] = { 108U, 97U, 95U, 54U, 95U, 106U, 111U, 105U,
      110U, 116U };

    static const uint8_T h[10] = { 108U, 97U, 95U, 55U, 95U, 106U, 111U, 105U,
      110U, 116U };

    // SimscapeExecutionBlock: '<S48>/STATE_1' incorporates:
    //   SimscapeExecutionBlock: '<S48>/OUTPUT_1_0'

    simulationData = (NeslSimulationData *)left_arm_simul_DW.STATE_1_SimData;
    left_arm_simul_B.time_tmp = left_arm_simul_M->Timing.t[0];
    left_arm_simul_B.time = left_arm_simul_B.time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &left_arm_simul_B.time;
    simulationData->mData->mContStates.mN = 14;
    simulationData->mData->mContStates.mX =
      &left_arm_simul_X.left_arm_simulla_1_jointRzq[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &left_arm_simul_DW.STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &left_arm_simul_DW.STATE_1_Modes;
    tmp = false;
    simulationData->mData->mFoundZcEvents = tmp;
    tmp = rtmIsMajorTimeStep(left_arm_simul_M);
    simulationData->mData->mIsMajorTimeStep = tmp;
    b_varargout_1 = false;
    simulationData->mData->mIsSolverAssertCheck = b_varargout_1;
    simulationData->mData->mIsSolverCheckingCIC = false;
    b_varargout_1 = rtsiIsSolverComputingJacobian(&left_arm_simul_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = b_varargout_1;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    left_arm_simul_B.iv1[0] = 0;
    left_arm_simul_B.dv1[0] = left_arm_simul_B.INPUT_1_1_1[0];
    left_arm_simul_B.dv1[1] = left_arm_simul_B.INPUT_1_1_1[1];
    left_arm_simul_B.dv1[2] = left_arm_simul_B.INPUT_1_1_1[2];
    left_arm_simul_B.dv1[3] = left_arm_simul_B.INPUT_1_1_1[3];
    left_arm_simul_B.iv1[1] = 4;
    left_arm_simul_B.dv1[4] = left_arm_simul_B.INPUT_2_1_1[0];
    left_arm_simul_B.dv1[5] = left_arm_simul_B.INPUT_2_1_1[1];
    left_arm_simul_B.dv1[6] = left_arm_simul_B.INPUT_2_1_1[2];
    left_arm_simul_B.dv1[7] = left_arm_simul_B.INPUT_2_1_1[3];
    left_arm_simul_B.iv1[2] = 8;
    left_arm_simul_B.dv1[8] = left_arm_simul_B.INPUT_3_1_1[0];
    left_arm_simul_B.dv1[9] = left_arm_simul_B.INPUT_3_1_1[1];
    left_arm_simul_B.dv1[10] = left_arm_simul_B.INPUT_3_1_1[2];
    left_arm_simul_B.dv1[11] = left_arm_simul_B.INPUT_3_1_1[3];
    left_arm_simul_B.iv1[3] = 12;
    left_arm_simul_B.dv1[12] = left_arm_simul_B.INPUT_4_1_1[0];
    left_arm_simul_B.dv1[13] = left_arm_simul_B.INPUT_4_1_1[1];
    left_arm_simul_B.dv1[14] = left_arm_simul_B.INPUT_4_1_1[2];
    left_arm_simul_B.dv1[15] = left_arm_simul_B.INPUT_4_1_1[3];
    left_arm_simul_B.iv1[4] = 16;
    left_arm_simul_B.dv1[16] = left_arm_simul_B.INPUT_5_1_1[0];
    left_arm_simul_B.dv1[17] = left_arm_simul_B.INPUT_5_1_1[1];
    left_arm_simul_B.dv1[18] = left_arm_simul_B.INPUT_5_1_1[2];
    left_arm_simul_B.dv1[19] = left_arm_simul_B.INPUT_5_1_1[3];
    left_arm_simul_B.iv1[5] = 20;
    left_arm_simul_B.dv1[20] = left_arm_simul_B.INPUT_6_1_1[0];
    left_arm_simul_B.dv1[21] = left_arm_simul_B.INPUT_6_1_1[1];
    left_arm_simul_B.dv1[22] = left_arm_simul_B.INPUT_6_1_1[2];
    left_arm_simul_B.dv1[23] = left_arm_simul_B.INPUT_6_1_1[3];
    left_arm_simul_B.iv1[6] = 24;
    left_arm_simul_B.dv1[24] = left_arm_simul_B.INPUT_7_1_1[0];
    left_arm_simul_B.dv1[25] = left_arm_simul_B.INPUT_7_1_1[1];
    left_arm_simul_B.dv1[26] = left_arm_simul_B.INPUT_7_1_1[2];
    left_arm_simul_B.dv1[27] = left_arm_simul_B.INPUT_7_1_1[3];
    left_arm_simul_B.iv1[7] = 28;
    simulationData->mData->mInputValues.mN = 28;
    simulationData->mData->mInputValues.mX = &left_arm_simul_B.dv1[0];
    simulationData->mData->mInputOffsets.mN = 8;
    simulationData->mData->mInputOffsets.mX = &left_arm_simul_B.iv1[0];
    simulationData->mData->mOutputs.mN = 14;
    simulationData->mData->mOutputs.mX = &left_arm_simul_B.STATE_1[0];
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    diagnosticManager = (NeuDiagnosticManager *)
      left_arm_simul_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = ne_simulator_method((NeslSimulator *)left_arm_simul_DW.STATE_1_Simulator,
      NESL_SIM_OUTPUTS, simulationData, diagnosticManager);
    if (i != 0) {
      b_varargout_1 = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
      if (b_varargout_1) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    // End of SimscapeExecutionBlock: '<S48>/STATE_1'
    if (rtmIsMajorTimeStep(left_arm_simul_M) &&
        left_arm_simul_M->Timing.TaskCounters.TID[1] == 0) {
      // Outputs for Atomic SubSystem: '<Root>/Subscribe'
      // MATLABSystem: '<S22>/SourceBlock' incorporates:
      //   Inport: '<S49>/In1'

      left_arm_simul_SystemCore_step(&b_varargout_1,
        left_arm_simul_B.b_varargout_2_Data, &b_varargout_2_Data_SL_Info_Curr,
        &b_varargout_2_Data_SL_Info_Rece, &b_varargout_2_Layout_DataOffset,
        left_arm_simul_B.b_varargout_2_Layout_Dim,
        &b_varargout_2_Layout_Dim_SL_Inf, &b_varargout_2_Layout_Dim_SL_I_0);

      // Outputs for Enabled SubSystem: '<S22>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S49>/Enable'

      if (b_varargout_1) {
        for (i = 0; i < 7; i++) {
          left_arm_simul_B.In1.Data[i] = left_arm_simul_B.b_varargout_2_Data[i];
        }

        left_arm_simul_B.In1.Data_SL_Info.CurrentLength =
          b_varargout_2_Data_SL_Info_Curr;
        left_arm_simul_B.In1.Data_SL_Info.ReceivedLength =
          b_varargout_2_Data_SL_Info_Rece;
        left_arm_simul_B.In1.Layout.DataOffset = b_varargout_2_Layout_DataOffset;
        memcpy(&left_arm_simul_B.In1.Layout.Dim[0],
               &left_arm_simul_B.b_varargout_2_Layout_Dim[0], sizeof
               (SL_Bus_left_arm_simul_std_msgs_MultiArrayDimension) << 4U);
        left_arm_simul_B.In1.Layout.Dim_SL_Info.CurrentLength =
          b_varargout_2_Layout_Dim_SL_Inf;
        left_arm_simul_B.In1.Layout.Dim_SL_Info.ReceivedLength =
          b_varargout_2_Layout_Dim_SL_I_0;
      }

      // End of MATLABSystem: '<S22>/SourceBlock'
      // End of Outputs for SubSystem: '<S22>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<Root>/Subscribe'
    }

    // SimscapeInputBlock: '<S48>/INPUT_1_1_1'
    left_arm_simul_B.INPUT_1_1_1[0] = left_arm_simul_B.In1.Data[0];
    left_arm_simul_B.INPUT_1_1_1[1] = 0.0;
    left_arm_simul_B.INPUT_1_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(left_arm_simul_M)) {
      left_arm_simul_DW.INPUT_1_1_1_Discrete[0] =
        !(left_arm_simul_B.INPUT_1_1_1[0] ==
          left_arm_simul_DW.INPUT_1_1_1_Discrete[1]);
      left_arm_simul_DW.INPUT_1_1_1_Discrete[1] = left_arm_simul_B.INPUT_1_1_1[0];
    }

    left_arm_simul_B.INPUT_1_1_1[0] = left_arm_simul_DW.INPUT_1_1_1_Discrete[1];
    left_arm_simul_B.INPUT_1_1_1[3] = left_arm_simul_DW.INPUT_1_1_1_Discrete[0];

    // End of SimscapeInputBlock: '<S48>/INPUT_1_1_1'

    // SimscapeInputBlock: '<S48>/INPUT_2_1_1'
    left_arm_simul_B.INPUT_2_1_1[0] = left_arm_simul_B.In1.Data[1];
    left_arm_simul_B.INPUT_2_1_1[1] = 0.0;
    left_arm_simul_B.INPUT_2_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(left_arm_simul_M)) {
      left_arm_simul_DW.INPUT_2_1_1_Discrete[0] =
        !(left_arm_simul_B.INPUT_2_1_1[0] ==
          left_arm_simul_DW.INPUT_2_1_1_Discrete[1]);
      left_arm_simul_DW.INPUT_2_1_1_Discrete[1] = left_arm_simul_B.INPUT_2_1_1[0];
    }

    left_arm_simul_B.INPUT_2_1_1[0] = left_arm_simul_DW.INPUT_2_1_1_Discrete[1];
    left_arm_simul_B.INPUT_2_1_1[3] = left_arm_simul_DW.INPUT_2_1_1_Discrete[0];

    // End of SimscapeInputBlock: '<S48>/INPUT_2_1_1'

    // SimscapeInputBlock: '<S48>/INPUT_3_1_1'
    left_arm_simul_B.INPUT_3_1_1[0] = left_arm_simul_B.In1.Data[2];
    left_arm_simul_B.INPUT_3_1_1[1] = 0.0;
    left_arm_simul_B.INPUT_3_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(left_arm_simul_M)) {
      left_arm_simul_DW.INPUT_3_1_1_Discrete[0] =
        !(left_arm_simul_B.INPUT_3_1_1[0] ==
          left_arm_simul_DW.INPUT_3_1_1_Discrete[1]);
      left_arm_simul_DW.INPUT_3_1_1_Discrete[1] = left_arm_simul_B.INPUT_3_1_1[0];
    }

    left_arm_simul_B.INPUT_3_1_1[0] = left_arm_simul_DW.INPUT_3_1_1_Discrete[1];
    left_arm_simul_B.INPUT_3_1_1[3] = left_arm_simul_DW.INPUT_3_1_1_Discrete[0];

    // End of SimscapeInputBlock: '<S48>/INPUT_3_1_1'

    // SimscapeInputBlock: '<S48>/INPUT_4_1_1'
    left_arm_simul_B.INPUT_4_1_1[0] = left_arm_simul_B.In1.Data[3];
    left_arm_simul_B.INPUT_4_1_1[1] = 0.0;
    left_arm_simul_B.INPUT_4_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(left_arm_simul_M)) {
      left_arm_simul_DW.INPUT_4_1_1_Discrete[0] =
        !(left_arm_simul_B.INPUT_4_1_1[0] ==
          left_arm_simul_DW.INPUT_4_1_1_Discrete[1]);
      left_arm_simul_DW.INPUT_4_1_1_Discrete[1] = left_arm_simul_B.INPUT_4_1_1[0];
    }

    left_arm_simul_B.INPUT_4_1_1[0] = left_arm_simul_DW.INPUT_4_1_1_Discrete[1];
    left_arm_simul_B.INPUT_4_1_1[3] = left_arm_simul_DW.INPUT_4_1_1_Discrete[0];

    // End of SimscapeInputBlock: '<S48>/INPUT_4_1_1'

    // SimscapeInputBlock: '<S48>/INPUT_5_1_1'
    left_arm_simul_B.INPUT_5_1_1[0] = left_arm_simul_B.In1.Data[4];
    left_arm_simul_B.INPUT_5_1_1[1] = 0.0;
    left_arm_simul_B.INPUT_5_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(left_arm_simul_M)) {
      left_arm_simul_DW.INPUT_5_1_1_Discrete[0] =
        !(left_arm_simul_B.INPUT_5_1_1[0] ==
          left_arm_simul_DW.INPUT_5_1_1_Discrete[1]);
      left_arm_simul_DW.INPUT_5_1_1_Discrete[1] = left_arm_simul_B.INPUT_5_1_1[0];
    }

    left_arm_simul_B.INPUT_5_1_1[0] = left_arm_simul_DW.INPUT_5_1_1_Discrete[1];
    left_arm_simul_B.INPUT_5_1_1[3] = left_arm_simul_DW.INPUT_5_1_1_Discrete[0];

    // End of SimscapeInputBlock: '<S48>/INPUT_5_1_1'

    // SimscapeInputBlock: '<S48>/INPUT_6_1_1'
    left_arm_simul_B.INPUT_6_1_1[0] = left_arm_simul_B.In1.Data[5];
    left_arm_simul_B.INPUT_6_1_1[1] = 0.0;
    left_arm_simul_B.INPUT_6_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(left_arm_simul_M)) {
      left_arm_simul_DW.INPUT_6_1_1_Discrete[0] =
        !(left_arm_simul_B.INPUT_6_1_1[0] ==
          left_arm_simul_DW.INPUT_6_1_1_Discrete[1]);
      left_arm_simul_DW.INPUT_6_1_1_Discrete[1] = left_arm_simul_B.INPUT_6_1_1[0];
    }

    left_arm_simul_B.INPUT_6_1_1[0] = left_arm_simul_DW.INPUT_6_1_1_Discrete[1];
    left_arm_simul_B.INPUT_6_1_1[3] = left_arm_simul_DW.INPUT_6_1_1_Discrete[0];

    // End of SimscapeInputBlock: '<S48>/INPUT_6_1_1'

    // SimscapeInputBlock: '<S48>/INPUT_7_1_1'
    left_arm_simul_B.INPUT_7_1_1[0] = left_arm_simul_B.In1.Data[6];
    left_arm_simul_B.INPUT_7_1_1[1] = 0.0;
    left_arm_simul_B.INPUT_7_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(left_arm_simul_M)) {
      left_arm_simul_DW.INPUT_7_1_1_Discrete[0] =
        !(left_arm_simul_B.INPUT_7_1_1[0] ==
          left_arm_simul_DW.INPUT_7_1_1_Discrete[1]);
      left_arm_simul_DW.INPUT_7_1_1_Discrete[1] = left_arm_simul_B.INPUT_7_1_1[0];
    }

    left_arm_simul_B.INPUT_7_1_1[0] = left_arm_simul_DW.INPUT_7_1_1_Discrete[1];
    left_arm_simul_B.INPUT_7_1_1[3] = left_arm_simul_DW.INPUT_7_1_1_Discrete[0];

    // End of SimscapeInputBlock: '<S48>/INPUT_7_1_1'

    // SimscapeExecutionBlock: '<S48>/OUTPUT_1_0'
    simulationData = (NeslSimulationData *)left_arm_simul_DW.OUTPUT_1_0_SimData;
    left_arm_simul_B.time_k = left_arm_simul_B.time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &left_arm_simul_B.time_k;
    simulationData->mData->mContStates.mN = 0;
    simulationData->mData->mContStates.mX = NULL;
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX =
      &left_arm_simul_DW.OUTPUT_1_0_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &left_arm_simul_DW.OUTPUT_1_0_Modes;
    b_varargout_1 = false;
    simulationData->mData->mFoundZcEvents = b_varargout_1;
    simulationData->mData->mIsMajorTimeStep = tmp;
    tmp = false;
    simulationData->mData->mIsSolverAssertCheck = tmp;
    simulationData->mData->mIsSolverCheckingCIC = false;
    simulationData->mData->mIsComputingJacobian = false;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    left_arm_simul_B.iv[0] = 0;
    left_arm_simul_B.dv[0] = left_arm_simul_B.INPUT_1_1_1[0];
    left_arm_simul_B.dv[1] = left_arm_simul_B.INPUT_1_1_1[1];
    left_arm_simul_B.dv[2] = left_arm_simul_B.INPUT_1_1_1[2];
    left_arm_simul_B.dv[3] = left_arm_simul_B.INPUT_1_1_1[3];
    left_arm_simul_B.iv[1] = 4;
    left_arm_simul_B.dv[4] = left_arm_simul_B.INPUT_2_1_1[0];
    left_arm_simul_B.dv[5] = left_arm_simul_B.INPUT_2_1_1[1];
    left_arm_simul_B.dv[6] = left_arm_simul_B.INPUT_2_1_1[2];
    left_arm_simul_B.dv[7] = left_arm_simul_B.INPUT_2_1_1[3];
    left_arm_simul_B.iv[2] = 8;
    left_arm_simul_B.dv[8] = left_arm_simul_B.INPUT_3_1_1[0];
    left_arm_simul_B.dv[9] = left_arm_simul_B.INPUT_3_1_1[1];
    left_arm_simul_B.dv[10] = left_arm_simul_B.INPUT_3_1_1[2];
    left_arm_simul_B.dv[11] = left_arm_simul_B.INPUT_3_1_1[3];
    left_arm_simul_B.iv[3] = 12;
    left_arm_simul_B.dv[12] = left_arm_simul_B.INPUT_4_1_1[0];
    left_arm_simul_B.dv[13] = left_arm_simul_B.INPUT_4_1_1[1];
    left_arm_simul_B.dv[14] = left_arm_simul_B.INPUT_4_1_1[2];
    left_arm_simul_B.dv[15] = left_arm_simul_B.INPUT_4_1_1[3];
    left_arm_simul_B.iv[4] = 16;
    left_arm_simul_B.dv[16] = left_arm_simul_B.INPUT_5_1_1[0];
    left_arm_simul_B.dv[17] = left_arm_simul_B.INPUT_5_1_1[1];
    left_arm_simul_B.dv[18] = left_arm_simul_B.INPUT_5_1_1[2];
    left_arm_simul_B.dv[19] = left_arm_simul_B.INPUT_5_1_1[3];
    left_arm_simul_B.iv[5] = 20;
    left_arm_simul_B.dv[20] = left_arm_simul_B.INPUT_6_1_1[0];
    left_arm_simul_B.dv[21] = left_arm_simul_B.INPUT_6_1_1[1];
    left_arm_simul_B.dv[22] = left_arm_simul_B.INPUT_6_1_1[2];
    left_arm_simul_B.dv[23] = left_arm_simul_B.INPUT_6_1_1[3];
    left_arm_simul_B.iv[6] = 24;
    left_arm_simul_B.dv[24] = left_arm_simul_B.INPUT_7_1_1[0];
    left_arm_simul_B.dv[25] = left_arm_simul_B.INPUT_7_1_1[1];
    left_arm_simul_B.dv[26] = left_arm_simul_B.INPUT_7_1_1[2];
    left_arm_simul_B.dv[27] = left_arm_simul_B.INPUT_7_1_1[3];
    left_arm_simul_B.iv[7] = 28;
    memcpy(&left_arm_simul_B.dv[28], &left_arm_simul_B.STATE_1[0], 14U * sizeof
           (real_T));
    left_arm_simul_B.iv[8] = 42;
    simulationData->mData->mInputValues.mN = 42;
    simulationData->mData->mInputValues.mX = &left_arm_simul_B.dv[0];
    simulationData->mData->mInputOffsets.mN = 9;
    simulationData->mData->mInputOffsets.mX = &left_arm_simul_B.iv[0];
    simulationData->mData->mOutputs.mN = 7;
    simulationData->mData->mOutputs.mX = &left_arm_simul_B.OUTPUT_1_0[0];
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    diagnosticManager = (NeuDiagnosticManager *)
      left_arm_simul_DW.OUTPUT_1_0_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = ne_simulator_method((NeslSimulator *)
      left_arm_simul_DW.OUTPUT_1_0_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diagnosticManager);
    if (i != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
      if (tmp) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    // RateTransition: '<Root>/Rate Transition1'
    if ((rtmIsMajorTimeStep(left_arm_simul_M) &&
         left_arm_simul_M->Timing.TaskCounters.TID[1] == 0) &&
        (rtmIsMajorTimeStep(left_arm_simul_M) &&
         left_arm_simul_M->Timing.TaskCounters.TID[2] == 0)) {
      for (i = 0; i < 7; i++) {
        left_arm_simul_DW.RateTransition1_Buffer[i] =
          left_arm_simul_B.OUTPUT_1_0[i];
      }
    }

    if (rtmIsMajorTimeStep(left_arm_simul_M) &&
        left_arm_simul_M->Timing.TaskCounters.TID[2] == 0) {
      // MATLABSystem: '<Root>/Current Time'
      currentROSTimeBus(&left_arm_simul_B.rtb_CurrentTime_c);

      // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
      //   Constant: '<S3>/Constant'
      //   MATLABSystem: '<Root>/Current Time'

      left_arm_simul_B.js = left_arm_simul_P.Constant_Value_b;
      left_arm_simul_B.js.Name[0].Data_SL_Info.CurrentLength = 10U;
      left_arm_simul_B.js.Name[0].Data_SL_Info.ReceivedLength = 10U;
      left_arm_simul_B.js.Name[1].Data_SL_Info.CurrentLength = 10U;
      left_arm_simul_B.js.Name[1].Data_SL_Info.ReceivedLength = 10U;
      left_arm_simul_B.js.Name[2].Data_SL_Info.CurrentLength = 10U;
      left_arm_simul_B.js.Name[2].Data_SL_Info.ReceivedLength = 10U;
      left_arm_simul_B.js.Name[3].Data_SL_Info.CurrentLength = 10U;
      left_arm_simul_B.js.Name[3].Data_SL_Info.ReceivedLength = 10U;
      left_arm_simul_B.js.Name[4].Data_SL_Info.CurrentLength = 10U;
      left_arm_simul_B.js.Name[4].Data_SL_Info.ReceivedLength = 10U;
      left_arm_simul_B.js.Name[5].Data_SL_Info.CurrentLength = 10U;
      left_arm_simul_B.js.Name[5].Data_SL_Info.ReceivedLength = 10U;
      left_arm_simul_B.js.Name[6].Data_SL_Info.CurrentLength = 10U;
      left_arm_simul_B.js.Name[6].Data_SL_Info.ReceivedLength = 10U;
      for (i = 0; i < 10; i++) {
        left_arm_simul_B.js.Name[0].Data[i] = b[i];
        left_arm_simul_B.js.Name[1].Data[i] = c[i];
        left_arm_simul_B.js.Name[2].Data[i] = d[i];
        left_arm_simul_B.js.Name[3].Data[i] = e[i];
        left_arm_simul_B.js.Name[4].Data[i] = f[i];
        left_arm_simul_B.js.Name[5].Data[i] = g[i];
        left_arm_simul_B.js.Name[6].Data[i] = h[i];
      }

      left_arm_simul_B.js.Name_SL_Info.CurrentLength = 7U;
      left_arm_simul_B.js.Name_SL_Info.ReceivedLength = 7U;
      left_arm_simul_B.js.Header.Stamp = left_arm_simul_B.rtb_CurrentTime_c;
      left_arm_simul_B.js.Position_SL_Info.CurrentLength = 7U;
      left_arm_simul_B.js.Position_SL_Info.ReceivedLength = 7U;
      left_arm_simul_B.js.Position[0] =
        left_arm_simul_DW.RateTransition1_Buffer[0];
      left_arm_simul_B.js.Position[1] =
        left_arm_simul_DW.RateTransition1_Buffer[1];
      left_arm_simul_B.js.Position[2] =
        left_arm_simul_DW.RateTransition1_Buffer[2];
      left_arm_simul_B.js.Position[3] =
        left_arm_simul_DW.RateTransition1_Buffer[3];
      left_arm_simul_B.js.Position[4] =
        left_arm_simul_DW.RateTransition1_Buffer[4];
      left_arm_simul_B.js.Position[5] =
        left_arm_simul_DW.RateTransition1_Buffer[5];
      left_arm_simul_B.js.Position[6] =
        left_arm_simul_DW.RateTransition1_Buffer[6];

      // End of MATLAB Function: '<Root>/MATLAB Function1'

      // Outputs for Atomic SubSystem: '<Root>/Publish2'
      // MATLABSystem: '<S13>/SinkBlock'
      Pub_left_arm_simul_191.publish(&left_arm_simul_B.js);

      // End of Outputs for SubSystem: '<Root>/Publish2'
    }

    // End of RateTransition: '<Root>/Rate Transition1'
    if (rtmIsMajorTimeStep(left_arm_simul_M) &&
        left_arm_simul_M->Timing.TaskCounters.TID[1] == 0) {
      // BusAssignment: '<Root>/Bus Assignment1' incorporates:
      //   Constant: '<Root>/Constant'
      //   Constant: '<S1>/Constant'

      left_arm_simul_B.BusAssignment1 = left_arm_simul_P.Constant_Value_m;
      for (i = 0; i < 7; i++) {
        left_arm_simul_B.BusAssignment1.Data[i] = left_arm_simul_B.OUTPUT_1_0[i];
      }

      left_arm_simul_B.BusAssignment1.Data_SL_Info.CurrentLength =
        left_arm_simul_P.Constant_Value_mx;
      left_arm_simul_B.BusAssignment1.Data_SL_Info.ReceivedLength =
        left_arm_simul_P.Constant_Value_mx;

      // End of BusAssignment: '<Root>/Bus Assignment1'

      // Outputs for Atomic SubSystem: '<Root>/Publish1'
      // MATLABSystem: '<S12>/SinkBlock'
      Pub_left_arm_simul_163.publish(&left_arm_simul_B.BusAssignment1);

      // End of Outputs for SubSystem: '<Root>/Publish1'
    }
  }

  if (rtmIsMajorTimeStep(left_arm_simul_M)) {
    NeslSimulationData *simulationData;
    real_T time;
    boolean_T tmp;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    int32_T tmp_0;
    char *msg;

    // Update for SimscapeExecutionBlock: '<S48>/STATE_1'
    simulationData = (NeslSimulationData *)left_arm_simul_DW.STATE_1_SimData;
    time = left_arm_simul_M->Timing.t[0];
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 14;
    simulationData->mData->mContStates.mX =
      &left_arm_simul_X.left_arm_simulla_1_jointRzq[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &left_arm_simul_DW.STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &left_arm_simul_DW.STATE_1_Modes;
    tmp = false;
    simulationData->mData->mFoundZcEvents = tmp;
    simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep
      (left_arm_simul_M);
    tmp = false;
    simulationData->mData->mIsSolverAssertCheck = tmp;
    simulationData->mData->mIsSolverCheckingCIC = false;
    tmp = rtsiIsSolverComputingJacobian(&left_arm_simul_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = tmp;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    left_arm_simul_B.iv5[0] = 0;
    left_arm_simul_B.dv5[0] = left_arm_simul_B.INPUT_1_1_1[0];
    left_arm_simul_B.dv5[1] = left_arm_simul_B.INPUT_1_1_1[1];
    left_arm_simul_B.dv5[2] = left_arm_simul_B.INPUT_1_1_1[2];
    left_arm_simul_B.dv5[3] = left_arm_simul_B.INPUT_1_1_1[3];
    left_arm_simul_B.iv5[1] = 4;
    left_arm_simul_B.dv5[4] = left_arm_simul_B.INPUT_2_1_1[0];
    left_arm_simul_B.dv5[5] = left_arm_simul_B.INPUT_2_1_1[1];
    left_arm_simul_B.dv5[6] = left_arm_simul_B.INPUT_2_1_1[2];
    left_arm_simul_B.dv5[7] = left_arm_simul_B.INPUT_2_1_1[3];
    left_arm_simul_B.iv5[2] = 8;
    left_arm_simul_B.dv5[8] = left_arm_simul_B.INPUT_3_1_1[0];
    left_arm_simul_B.dv5[9] = left_arm_simul_B.INPUT_3_1_1[1];
    left_arm_simul_B.dv5[10] = left_arm_simul_B.INPUT_3_1_1[2];
    left_arm_simul_B.dv5[11] = left_arm_simul_B.INPUT_3_1_1[3];
    left_arm_simul_B.iv5[3] = 12;
    left_arm_simul_B.dv5[12] = left_arm_simul_B.INPUT_4_1_1[0];
    left_arm_simul_B.dv5[13] = left_arm_simul_B.INPUT_4_1_1[1];
    left_arm_simul_B.dv5[14] = left_arm_simul_B.INPUT_4_1_1[2];
    left_arm_simul_B.dv5[15] = left_arm_simul_B.INPUT_4_1_1[3];
    left_arm_simul_B.iv5[4] = 16;
    left_arm_simul_B.dv5[16] = left_arm_simul_B.INPUT_5_1_1[0];
    left_arm_simul_B.dv5[17] = left_arm_simul_B.INPUT_5_1_1[1];
    left_arm_simul_B.dv5[18] = left_arm_simul_B.INPUT_5_1_1[2];
    left_arm_simul_B.dv5[19] = left_arm_simul_B.INPUT_5_1_1[3];
    left_arm_simul_B.iv5[5] = 20;
    left_arm_simul_B.dv5[20] = left_arm_simul_B.INPUT_6_1_1[0];
    left_arm_simul_B.dv5[21] = left_arm_simul_B.INPUT_6_1_1[1];
    left_arm_simul_B.dv5[22] = left_arm_simul_B.INPUT_6_1_1[2];
    left_arm_simul_B.dv5[23] = left_arm_simul_B.INPUT_6_1_1[3];
    left_arm_simul_B.iv5[6] = 24;
    left_arm_simul_B.dv5[24] = left_arm_simul_B.INPUT_7_1_1[0];
    left_arm_simul_B.dv5[25] = left_arm_simul_B.INPUT_7_1_1[1];
    left_arm_simul_B.dv5[26] = left_arm_simul_B.INPUT_7_1_1[2];
    left_arm_simul_B.dv5[27] = left_arm_simul_B.INPUT_7_1_1[3];
    left_arm_simul_B.iv5[7] = 28;
    simulationData->mData->mInputValues.mN = 28;
    simulationData->mData->mInputValues.mX = &left_arm_simul_B.dv5[0];
    simulationData->mData->mInputOffsets.mN = 8;
    simulationData->mData->mInputOffsets.mX = &left_arm_simul_B.iv5[0];
    diagnosticManager = (NeuDiagnosticManager *)
      left_arm_simul_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_0 = ne_simulator_method((NeslSimulator *)
      left_arm_simul_DW.STATE_1_Simulator, NESL_SIM_UPDATE, simulationData,
      diagnosticManager);
    if (tmp_0 != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
      if (tmp) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    // End of Update for SimscapeExecutionBlock: '<S48>/STATE_1'
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(left_arm_simul_M)) {
    rt_ertODEUpdateContinuousStates(&left_arm_simul_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++left_arm_simul_M->Timing.clockTick0;
    left_arm_simul_M->Timing.t[0] = rtsiGetSolverStopTime
      (&left_arm_simul_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.001s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.001, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      left_arm_simul_M->Timing.clockTick1++;
    }

    rate_scheduler();
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void left_arm_simul_derivatives(void)
{
  NeslSimulationData *simulationData;
  real_T time;
  boolean_T tmp;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  int32_T tmp_0;
  char *msg;
  XDot_left_arm_simul_T *_rtXdot;
  _rtXdot = ((XDot_left_arm_simul_T *) left_arm_simul_M->derivs);

  // Derivatives for SimscapeExecutionBlock: '<S48>/STATE_1'
  simulationData = (NeslSimulationData *)left_arm_simul_DW.STATE_1_SimData;
  time = left_arm_simul_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 14;
  simulationData->mData->mContStates.mX =
    &left_arm_simul_X.left_arm_simulla_1_jointRzq[0];
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &left_arm_simul_DW.STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX = &left_arm_simul_DW.STATE_1_Modes;
  tmp = false;
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep(left_arm_simul_M);
  tmp = false;
  simulationData->mData->mIsSolverAssertCheck = tmp;
  simulationData->mData->mIsSolverCheckingCIC = false;
  tmp = rtsiIsSolverComputingJacobian(&left_arm_simul_M->solverInfo);
  simulationData->mData->mIsComputingJacobian = tmp;
  simulationData->mData->mIsEvaluatingF0 = false;
  simulationData->mData->mIsSolverRequestingReset = false;
  left_arm_simul_B.iv4[0] = 0;
  left_arm_simul_B.dv4[0] = left_arm_simul_B.INPUT_1_1_1[0];
  left_arm_simul_B.dv4[1] = left_arm_simul_B.INPUT_1_1_1[1];
  left_arm_simul_B.dv4[2] = left_arm_simul_B.INPUT_1_1_1[2];
  left_arm_simul_B.dv4[3] = left_arm_simul_B.INPUT_1_1_1[3];
  left_arm_simul_B.iv4[1] = 4;
  left_arm_simul_B.dv4[4] = left_arm_simul_B.INPUT_2_1_1[0];
  left_arm_simul_B.dv4[5] = left_arm_simul_B.INPUT_2_1_1[1];
  left_arm_simul_B.dv4[6] = left_arm_simul_B.INPUT_2_1_1[2];
  left_arm_simul_B.dv4[7] = left_arm_simul_B.INPUT_2_1_1[3];
  left_arm_simul_B.iv4[2] = 8;
  left_arm_simul_B.dv4[8] = left_arm_simul_B.INPUT_3_1_1[0];
  left_arm_simul_B.dv4[9] = left_arm_simul_B.INPUT_3_1_1[1];
  left_arm_simul_B.dv4[10] = left_arm_simul_B.INPUT_3_1_1[2];
  left_arm_simul_B.dv4[11] = left_arm_simul_B.INPUT_3_1_1[3];
  left_arm_simul_B.iv4[3] = 12;
  left_arm_simul_B.dv4[12] = left_arm_simul_B.INPUT_4_1_1[0];
  left_arm_simul_B.dv4[13] = left_arm_simul_B.INPUT_4_1_1[1];
  left_arm_simul_B.dv4[14] = left_arm_simul_B.INPUT_4_1_1[2];
  left_arm_simul_B.dv4[15] = left_arm_simul_B.INPUT_4_1_1[3];
  left_arm_simul_B.iv4[4] = 16;
  left_arm_simul_B.dv4[16] = left_arm_simul_B.INPUT_5_1_1[0];
  left_arm_simul_B.dv4[17] = left_arm_simul_B.INPUT_5_1_1[1];
  left_arm_simul_B.dv4[18] = left_arm_simul_B.INPUT_5_1_1[2];
  left_arm_simul_B.dv4[19] = left_arm_simul_B.INPUT_5_1_1[3];
  left_arm_simul_B.iv4[5] = 20;
  left_arm_simul_B.dv4[20] = left_arm_simul_B.INPUT_6_1_1[0];
  left_arm_simul_B.dv4[21] = left_arm_simul_B.INPUT_6_1_1[1];
  left_arm_simul_B.dv4[22] = left_arm_simul_B.INPUT_6_1_1[2];
  left_arm_simul_B.dv4[23] = left_arm_simul_B.INPUT_6_1_1[3];
  left_arm_simul_B.iv4[6] = 24;
  left_arm_simul_B.dv4[24] = left_arm_simul_B.INPUT_7_1_1[0];
  left_arm_simul_B.dv4[25] = left_arm_simul_B.INPUT_7_1_1[1];
  left_arm_simul_B.dv4[26] = left_arm_simul_B.INPUT_7_1_1[2];
  left_arm_simul_B.dv4[27] = left_arm_simul_B.INPUT_7_1_1[3];
  left_arm_simul_B.iv4[7] = 28;
  simulationData->mData->mInputValues.mN = 28;
  simulationData->mData->mInputValues.mX = &left_arm_simul_B.dv4[0];
  simulationData->mData->mInputOffsets.mN = 8;
  simulationData->mData->mInputOffsets.mX = &left_arm_simul_B.iv4[0];
  simulationData->mData->mDx.mN = 14;
  simulationData->mData->mDx.mX = &_rtXdot->left_arm_simulla_1_jointRzq[0];
  diagnosticManager = (NeuDiagnosticManager *)left_arm_simul_DW.STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_0 = ne_simulator_method((NeslSimulator *)
    left_arm_simul_DW.STATE_1_Simulator, NESL_SIM_DERIVATIVES, simulationData,
    diagnosticManager);
  if (tmp_0 != 0) {
    tmp = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      rtmSetErrorStatus(left_arm_simul_M, msg);
    }
  }

  // End of Derivatives for SimscapeExecutionBlock: '<S48>/STATE_1'
}

// Model initialize function
void left_arm_simul_initialize(void)
{
  // Registration code
  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&left_arm_simul_M->solverInfo,
                          &left_arm_simul_M->Timing.simTimeStep);
    rtsiSetTPtr(&left_arm_simul_M->solverInfo, &rtmGetTPtr(left_arm_simul_M));
    rtsiSetStepSizePtr(&left_arm_simul_M->solverInfo,
                       &left_arm_simul_M->Timing.stepSize0);
    rtsiSetdXPtr(&left_arm_simul_M->solverInfo, &left_arm_simul_M->derivs);
    rtsiSetContStatesPtr(&left_arm_simul_M->solverInfo, (real_T **)
                         &left_arm_simul_M->contStates);
    rtsiSetNumContStatesPtr(&left_arm_simul_M->solverInfo,
      &left_arm_simul_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&left_arm_simul_M->solverInfo,
      &left_arm_simul_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&left_arm_simul_M->solverInfo,
      &left_arm_simul_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&left_arm_simul_M->solverInfo,
      &left_arm_simul_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&left_arm_simul_M->solverInfo, (&rtmGetErrorStatus
      (left_arm_simul_M)));
    rtsiSetRTModelPtr(&left_arm_simul_M->solverInfo, left_arm_simul_M);
  }

  rtsiSetSimTimeStep(&left_arm_simul_M->solverInfo, MAJOR_TIME_STEP);
  left_arm_simul_M->intgData.y = left_arm_simul_M->odeY;
  left_arm_simul_M->intgData.f[0] = left_arm_simul_M->odeF[0];
  left_arm_simul_M->intgData.f[1] = left_arm_simul_M->odeF[1];
  left_arm_simul_M->intgData.f[2] = left_arm_simul_M->odeF[2];
  left_arm_simul_M->intgData.f[3] = left_arm_simul_M->odeF[3];
  left_arm_simul_M->contStates = ((X_left_arm_simul_T *) &left_arm_simul_X);
  rtsiSetSolverData(&left_arm_simul_M->solverInfo, static_cast<void *>
                    (&left_arm_simul_M->intgData));
  rtsiSetSolverName(&left_arm_simul_M->solverInfo,"ode4");
  rtmSetTPtr(left_arm_simul_M, &left_arm_simul_M->Timing.tArray[0]);
  left_arm_simul_M->Timing.stepSize0 = 0.001;

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

    static const char_T tmp_3[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_4[25] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'c', 'u', 'r', 'r', 'e', 'n', 't', '_', 'p', 'o',
      's', 'e' };

    // Start for SimscapeExecutionBlock: '<S48>/STATE_1' incorporates:
    //   SimscapeExecutionBlock: '<S48>/OUTPUT_1_0'

    tmp = nesl_lease_simulator("left_arm_simul/Solver Configuration_1", 0, 0);
    left_arm_simul_DW.STATE_1_Simulator = (void *)tmp;
    tmp_0 = pointer_is_null(left_arm_simul_DW.STATE_1_Simulator);
    if (tmp_0) {
      left_arm_simul_e2442d81_1_gateway();
      tmp = nesl_lease_simulator("left_arm_simul/Solver Configuration_1", 0, 0);
      left_arm_simul_DW.STATE_1_Simulator = (void *)tmp;
    }

    simulationData = nesl_create_simulation_data();
    left_arm_simul_DW.STATE_1_SimData = (void *)simulationData;
    diagnosticManager = rtw_create_diagnostics();
    left_arm_simul_DW.STATE_1_DiagMgr = (void *)diagnosticManager;
    left_arm_simul_B.modelParameters.mSolverType = NE_SOLVER_TYPE_ODE;
    left_arm_simul_B.modelParameters.mSolverTolerance = 0.001;
    left_arm_simul_B.modelParameters.mVariableStepSolver = false;
    left_arm_simul_B.modelParameters.mFixedStepSize = 0.001;
    left_arm_simul_B.modelParameters.mStartTime = 0.0;
    left_arm_simul_B.modelParameters.mLoadInitialState = false;
    left_arm_simul_B.modelParameters.mUseSimState = false;
    left_arm_simul_B.modelParameters.mLinTrimCompile = false;
    left_arm_simul_B.modelParameters.mLoggingMode = SSC_LOGGING_NONE;
    left_arm_simul_B.modelParameters.mRTWModifiedTimeStamp = 5.14126304E+8;
    left_arm_simul_B.d = 0.001;
    left_arm_simul_B.modelParameters.mSolverTolerance = left_arm_simul_B.d;
    left_arm_simul_B.d = 0.001;
    left_arm_simul_B.modelParameters.mFixedStepSize = left_arm_simul_B.d;
    tmp_0 = false;
    left_arm_simul_B.modelParameters.mVariableStepSolver = tmp_0;
    diagnosticManager = (NeuDiagnosticManager *)
      left_arm_simul_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = nesl_initialize_simulator((NeslSimulator *)
      left_arm_simul_DW.STATE_1_Simulator, &left_arm_simul_B.modelParameters,
      diagnosticManager);
    if (i != 0) {
      tmp_0 = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
      if (tmp_0) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    left_arm_simul_B.expl_temp.mRealParameters.mN = 0;
    left_arm_simul_B.expl_temp.mRealParameters.mX = NULL;
    left_arm_simul_B.expl_temp.mLogicalParameters.mN = 0;
    left_arm_simul_B.expl_temp.mLogicalParameters.mX = NULL;
    left_arm_simul_B.expl_temp.mIntegerParameters.mN = 0;
    left_arm_simul_B.expl_temp.mIntegerParameters.mX = NULL;
    left_arm_simul_B.expl_temp.mIndexParameters.mN = 0;
    left_arm_simul_B.expl_temp.mIndexParameters.mX = NULL;
    nesl_simulator_set_rtps((NeslSimulator *)left_arm_simul_DW.STATE_1_Simulator,
      left_arm_simul_B.expl_temp);
    simulationData = (NeslSimulationData *)left_arm_simul_DW.STATE_1_SimData;
    time_tmp = left_arm_simul_M->Timing.t[0];
    left_arm_simul_B.time_c = time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &left_arm_simul_B.time_c;
    simulationData->mData->mContStates.mN = 14;
    simulationData->mData->mContStates.mX =
      &left_arm_simul_X.left_arm_simulla_1_jointRzq[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &left_arm_simul_DW.STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &left_arm_simul_DW.STATE_1_Modes;
    tmp_0 = false;
    simulationData->mData->mFoundZcEvents = tmp_0;
    tmp_0 = rtmIsMajorTimeStep(left_arm_simul_M);
    simulationData->mData->mIsMajorTimeStep = tmp_0;
    tmp_1 = false;
    simulationData->mData->mIsSolverAssertCheck = tmp_1;
    simulationData->mData->mIsSolverCheckingCIC = false;
    tmp_1 = rtsiIsSolverComputingJacobian(&left_arm_simul_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = tmp_1;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    diagnosticManager = (NeuDiagnosticManager *)
      left_arm_simul_DW.STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = ne_simulator_method((NeslSimulator *)left_arm_simul_DW.STATE_1_Simulator,
      NESL_SIM_INITIALIZEONCE, simulationData, diagnosticManager);
    if (i != 0) {
      tmp_1 = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
      if (tmp_1) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    // End of Start for SimscapeExecutionBlock: '<S48>/STATE_1'

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S22>/SourceBlock'
    left_arm_simul_DW.obj_f.matlabCodegenIsDeleted = true;
    left_arm_simul_DW.obj_f.isInitialized = 0;
    left_arm_simul_DW.obj_f.matlabCodegenIsDeleted = false;
    left_arm_simul_DW.obj_f.isSetupComplete = false;
    left_arm_simul_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      left_arm_simul_B.cv1[i] = tmp_2[i];
    }

    left_arm_simul_B.cv1[19] = '\x00';
    Sub_left_arm_simul_160.createSubscriber(left_arm_simul_B.cv1, 1);
    left_arm_simul_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S22>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for SimscapeExecutionBlock: '<S48>/OUTPUT_1_0'
    tmp = nesl_lease_simulator("left_arm_simul/Solver Configuration_1", 1, 0);
    left_arm_simul_DW.OUTPUT_1_0_Simulator = (void *)tmp;
    tmp_1 = pointer_is_null(left_arm_simul_DW.OUTPUT_1_0_Simulator);
    if (tmp_1) {
      left_arm_simul_e2442d81_1_gateway();
      tmp = nesl_lease_simulator("left_arm_simul/Solver Configuration_1", 1, 0);
      left_arm_simul_DW.OUTPUT_1_0_Simulator = (void *)tmp;
    }

    simulationData = nesl_create_simulation_data();
    left_arm_simul_DW.OUTPUT_1_0_SimData = (void *)simulationData;
    diagnosticManager = rtw_create_diagnostics();
    left_arm_simul_DW.OUTPUT_1_0_DiagMgr = (void *)diagnosticManager;
    left_arm_simul_B.modelParameters_m.mSolverType = NE_SOLVER_TYPE_ODE;
    left_arm_simul_B.modelParameters_m.mSolverTolerance = 0.001;
    left_arm_simul_B.modelParameters_m.mVariableStepSolver = false;
    left_arm_simul_B.modelParameters_m.mFixedStepSize = 0.001;
    left_arm_simul_B.modelParameters_m.mStartTime = 0.0;
    left_arm_simul_B.modelParameters_m.mLoadInitialState = false;
    left_arm_simul_B.modelParameters_m.mUseSimState = false;
    left_arm_simul_B.modelParameters_m.mLinTrimCompile = false;
    left_arm_simul_B.modelParameters_m.mLoggingMode = SSC_LOGGING_NONE;
    left_arm_simul_B.modelParameters_m.mRTWModifiedTimeStamp = 5.14126304E+8;
    left_arm_simul_B.d = 0.001;
    left_arm_simul_B.modelParameters_m.mSolverTolerance = left_arm_simul_B.d;
    left_arm_simul_B.d = 0.001;
    left_arm_simul_B.modelParameters_m.mFixedStepSize = left_arm_simul_B.d;
    tmp_1 = false;
    left_arm_simul_B.modelParameters_m.mVariableStepSolver = tmp_1;
    diagnosticManager = (NeuDiagnosticManager *)
      left_arm_simul_DW.OUTPUT_1_0_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = nesl_initialize_simulator((NeslSimulator *)
      left_arm_simul_DW.OUTPUT_1_0_Simulator,
      &left_arm_simul_B.modelParameters_m, diagnosticManager);
    if (i != 0) {
      tmp_1 = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
      if (tmp_1) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    simulationData = (NeslSimulationData *)left_arm_simul_DW.OUTPUT_1_0_SimData;
    time = time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 0;
    simulationData->mData->mContStates.mX = NULL;
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX =
      &left_arm_simul_DW.OUTPUT_1_0_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &left_arm_simul_DW.OUTPUT_1_0_Modes;
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
      left_arm_simul_DW.OUTPUT_1_0_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    i = ne_simulator_method((NeslSimulator *)
      left_arm_simul_DW.OUTPUT_1_0_Simulator, NESL_SIM_INITIALIZEONCE,
      simulationData, diagnosticManager);
    if (i != 0) {
      tmp_0 = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
      if (tmp_0) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    // Start for MATLABSystem: '<Root>/Current Time'
    left_arm_simul_DW.obj.matlabCodegenIsDeleted = true;
    left_arm_simul_DW.obj.isInitialized = 0;
    left_arm_simul_DW.obj.matlabCodegenIsDeleted = false;
    left_arm_simul_DW.obj.isSetupComplete = false;
    left_arm_simul_DW.obj.isInitialized = 1;
    left_arm_simul_DW.obj.isSetupComplete = true;

    // Start for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S13>/SinkBlock'
    left_arm_simul_DW.obj_a.matlabCodegenIsDeleted = true;
    left_arm_simul_DW.obj_a.isInitialized = 0;
    left_arm_simul_DW.obj_a.matlabCodegenIsDeleted = false;
    left_arm_simul_DW.obj_a.isSetupComplete = false;
    left_arm_simul_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      left_arm_simul_B.cv2[i] = tmp_3[i];
    }

    left_arm_simul_B.cv2[13] = '\x00';
    Pub_left_arm_simul_191.createPublisher(left_arm_simul_B.cv2, 1);
    left_arm_simul_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish2'

    // Start for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S12>/SinkBlock'
    left_arm_simul_DW.obj_ag.matlabCodegenIsDeleted = true;
    left_arm_simul_DW.obj_ag.isInitialized = 0;
    left_arm_simul_DW.obj_ag.matlabCodegenIsDeleted = false;
    left_arm_simul_DW.obj_ag.isSetupComplete = false;
    left_arm_simul_DW.obj_ag.isInitialized = 1;
    for (i = 0; i < 25; i++) {
      left_arm_simul_B.cv[i] = tmp_4[i];
    }

    left_arm_simul_B.cv[25] = '\x00';
    Pub_left_arm_simul_163.createPublisher(left_arm_simul_B.cv, 1);
    left_arm_simul_DW.obj_ag.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish1'

    // InitializeConditions for SimscapeExecutionBlock: '<S48>/STATE_1'
    tmp_0 = false;
    if (tmp_0) {
      i = strcmp("ode4", rtsiGetSolverName(&left_arm_simul_M->solverInfo));
      if (i != 0) {
        msg = solver_mismatch_message("ode4", rtsiGetSolverName
          (&left_arm_simul_M->solverInfo));
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    // End of InitializeConditions for SimscapeExecutionBlock: '<S48>/STATE_1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S22>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S49>/Out1'
    left_arm_simul_B.In1 = left_arm_simul_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S22>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'
  }
}

// Model terminate function
void left_arm_simul_terminate(void)
{
  // Terminate for SimscapeExecutionBlock: '<S48>/STATE_1'
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    left_arm_simul_DW.STATE_1_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    left_arm_simul_DW.STATE_1_SimData);
  nesl_erase_simulator("left_arm_simul/Solver Configuration_1");

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S22>/SourceBlock'
  matlabCodegenHandle_matlabC_k10(&left_arm_simul_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for SimscapeExecutionBlock: '<S48>/OUTPUT_1_0'
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    left_arm_simul_DW.OUTPUT_1_0_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    left_arm_simul_DW.OUTPUT_1_0_SimData);
  nesl_erase_simulator("left_arm_simul/Solver Configuration_1");

  // Terminate for MATLABSystem: '<Root>/Current Time'
  matlabCodegenHandle_matlabCodeg(&left_arm_simul_DW.obj);

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S13>/SinkBlock'
  matlabCodegenHandle_matlabCod_k(&left_arm_simul_DW.obj_a);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S12>/SinkBlock'
  matlabCodegenHandle_matlabCod_k(&left_arm_simul_DW.obj_ag);

  // End of Terminate for SubSystem: '<Root>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
