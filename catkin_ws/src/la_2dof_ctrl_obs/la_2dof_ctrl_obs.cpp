//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: la_2dof_ctrl_obs.cpp
//
// Code generated for Simulink model 'la_2dof_ctrl_obs'.
//
// Model version                  : 1.86
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Sun Jun 14 23:27:58 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "la_2dof_ctrl_obs.h"
#include "la_2dof_ctrl_obs_private.h"

// Block signals (default storage)
B_la_2dof_ctrl_obs_T la_2dof_ctrl_obs_B;

// Continuous states
X_la_2dof_ctrl_obs_T la_2dof_ctrl_obs_X;

// Block states (default storage)
DW_la_2dof_ctrl_obs_T la_2dof_ctrl_obs_DW;

// Real-time model
RT_MODEL_la_2dof_ctrl_obs_T la_2dof_ctrl_obs_M_ = RT_MODEL_la_2dof_ctrl_obs_T();
RT_MODEL_la_2dof_ctrl_obs_T *const la_2dof_ctrl_obs_M = &la_2dof_ctrl_obs_M_;

// Forward declaration for local functions
static void la_2dof_ctrl_ob_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[2], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_la_2dof_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void la_2dof_ctrl__SystemCore_step_o(boolean_T *varargout_1, real_T
  varargout_2_Data[2], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_la_2dof_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void matlabCodegenHandle_matlabCo_ot(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);

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
  int_T nXc = 14;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  la_2dof_ctrl_obs_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  la_2dof_ctrl_obs_step();
  la_2dof_ctrl_obs_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  la_2dof_ctrl_obs_step();
  la_2dof_ctrl_obs_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void la_2dof_ctrl_ob_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[2], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_la_2dof_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_la_2dof_ctrl_obs_193.getLatestMessage
    (&la_2dof_ctrl_obs_B.b_varargout_2_m);
  varargout_2_Data[0] = la_2dof_ctrl_obs_B.b_varargout_2_m.Data[0];
  varargout_2_Data[1] = la_2dof_ctrl_obs_B.b_varargout_2_m.Data[1];
  *varargout_2_Data_SL_Info_Curren =
    la_2dof_ctrl_obs_B.b_varargout_2_m.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    la_2dof_ctrl_obs_B.b_varargout_2_m.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    la_2dof_ctrl_obs_B.b_varargout_2_m.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &la_2dof_ctrl_obs_B.b_varargout_2_m.Layout.Dim[0], sizeof
         (SL_Bus_la_2dof_ctrl_obs_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    la_2dof_ctrl_obs_B.b_varargout_2_m.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    la_2dof_ctrl_obs_B.b_varargout_2_m.Layout.Dim_SL_Info.ReceivedLength;
}

static void la_2dof_ctrl__SystemCore_step_o(boolean_T *varargout_1, real_T
  varargout_2_Data[2], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_la_2dof_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_la_2dof_ctrl_obs_195.getLatestMessage
    (&la_2dof_ctrl_obs_B.b_varargout_2);
  varargout_2_Data[0] = la_2dof_ctrl_obs_B.b_varargout_2.Data[0];
  varargout_2_Data[1] = la_2dof_ctrl_obs_B.b_varargout_2.Data[1];
  *varargout_2_Data_SL_Info_Curren =
    la_2dof_ctrl_obs_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    la_2dof_ctrl_obs_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    la_2dof_ctrl_obs_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &la_2dof_ctrl_obs_B.b_varargout_2.Layout.Dim[0], sizeof
         (SL_Bus_la_2dof_ctrl_obs_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    la_2dof_ctrl_obs_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    la_2dof_ctrl_obs_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void matlabCodegenHandle_matlabCo_ot(ros_slros_internal_block_Subs_T *obj)
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
void la_2dof_ctrl_obs_step(void)
{
  int_T ci;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_0;
  boolean_T b_varargout_1;
  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64 rtb_BusAssignment2;
  real_T z22_tmp;
  real_T qe_idx_1;
  real_T u_tmp;
  if (rtmIsMajorTimeStep(la_2dof_ctrl_obs_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&la_2dof_ctrl_obs_M->solverInfo,
                          ((la_2dof_ctrl_obs_M->Timing.clockTick0+1)*
      la_2dof_ctrl_obs_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(la_2dof_ctrl_obs_M)) {
    la_2dof_ctrl_obs_M->Timing.t[0] = rtsiGetT(&la_2dof_ctrl_obs_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(la_2dof_ctrl_obs_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S7>/SourceBlock' incorporates:
    //   Inport: '<S10>/In1'

    la_2dof_ctrl_ob_SystemCore_step(&b_varargout_1, la_2dof_ctrl_obs_B.torque,
      &b_varargout_2_Data_SL_Info_Curr, &b_varargout_2_Data_SL_Info_Rece,
      &b_varargout_2_Layout_DataOffset,
      la_2dof_ctrl_obs_B.b_varargout_2_Layout_Dim,
      &b_varargout_2_Layout_Dim_SL_Inf, &b_varargout_2_Layout_Dim_SL_I_0);

    // Outputs for Enabled SubSystem: '<S7>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S10>/Enable'

    if (b_varargout_1) {
      la_2dof_ctrl_obs_B.In1_d.Data[0] = la_2dof_ctrl_obs_B.torque[0];
      la_2dof_ctrl_obs_B.In1_d.Data[1] = la_2dof_ctrl_obs_B.torque[1];
      la_2dof_ctrl_obs_B.In1_d.Data_SL_Info.CurrentLength =
        b_varargout_2_Data_SL_Info_Curr;
      la_2dof_ctrl_obs_B.In1_d.Data_SL_Info.ReceivedLength =
        b_varargout_2_Data_SL_Info_Rece;
      la_2dof_ctrl_obs_B.In1_d.Layout.DataOffset =
        b_varargout_2_Layout_DataOffset;
      memcpy(&la_2dof_ctrl_obs_B.In1_d.Layout.Dim[0],
             &la_2dof_ctrl_obs_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_la_2dof_ctrl_obs_std_msgs_MultiArrayDimension) << 4U);
      la_2dof_ctrl_obs_B.In1_d.Layout.Dim_SL_Info.CurrentLength =
        b_varargout_2_Layout_Dim_SL_Inf;
      la_2dof_ctrl_obs_B.In1_d.Layout.Dim_SL_Info.ReceivedLength =
        b_varargout_2_Layout_Dim_SL_I_0;
    }

    // End of MATLABSystem: '<S7>/SourceBlock'
    // End of Outputs for SubSystem: '<S7>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'

    // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
    // MATLABSystem: '<S8>/SourceBlock' incorporates:
    //   Inport: '<S11>/In1'

    la_2dof_ctrl__SystemCore_step_o(&b_varargout_1, la_2dof_ctrl_obs_B.torque,
      &b_varargout_2_Data_SL_Info_Curr, &b_varargout_2_Data_SL_Info_Rece,
      &b_varargout_2_Layout_DataOffset,
      la_2dof_ctrl_obs_B.b_varargout_2_Layout_Dim,
      &b_varargout_2_Layout_Dim_SL_Inf, &b_varargout_2_Layout_Dim_SL_I_0);

    // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S11>/Enable'

    if (b_varargout_1) {
      la_2dof_ctrl_obs_B.In1.Data[0] = la_2dof_ctrl_obs_B.torque[0];
      la_2dof_ctrl_obs_B.In1.Data[1] = la_2dof_ctrl_obs_B.torque[1];
      la_2dof_ctrl_obs_B.In1.Data_SL_Info.CurrentLength =
        b_varargout_2_Data_SL_Info_Curr;
      la_2dof_ctrl_obs_B.In1.Data_SL_Info.ReceivedLength =
        b_varargout_2_Data_SL_Info_Rece;
      la_2dof_ctrl_obs_B.In1.Layout.DataOffset = b_varargout_2_Layout_DataOffset;
      memcpy(&la_2dof_ctrl_obs_B.In1.Layout.Dim[0],
             &la_2dof_ctrl_obs_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_la_2dof_ctrl_obs_std_msgs_MultiArrayDimension) << 4U);
      la_2dof_ctrl_obs_B.In1.Layout.Dim_SL_Info.CurrentLength =
        b_varargout_2_Layout_Dim_SL_Inf;
      la_2dof_ctrl_obs_B.In1.Layout.Dim_SL_Info.ReceivedLength =
        b_varargout_2_Layout_Dim_SL_I_0;
    }

    // End of MATLABSystem: '<S8>/SourceBlock'
    // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe1'
  }

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   Integrator: '<Root>/Integrator'

  la_2dof_ctrl_obs_B.m12 = sin(la_2dof_ctrl_obs_B.In1.Data[0] +
    la_2dof_ctrl_obs_B.In1.Data[1]) * 2.0165436;
  la_2dof_ctrl_obs_B.torque[0] = ((la_2dof_ctrl_obs_B.m12 + 5.3450766000000005 *
    sin(la_2dof_ctrl_obs_B.In1.Data[0])) + (la_2dof_ctrl_obs_B.In1.Data[0] -
    la_2dof_ctrl_obs_B.In1_d.Data[0]) * 7.5) - 3.5 *
    la_2dof_ctrl_obs_X.Integrator_CSTATE[2];
  la_2dof_ctrl_obs_B.torque[1] = (la_2dof_ctrl_obs_B.m12 +
    (la_2dof_ctrl_obs_B.In1.Data[1] - la_2dof_ctrl_obs_B.In1_d.Data[1]) * 4.5) -
    3.5 * la_2dof_ctrl_obs_X.Integrator_CSTATE[3];

  // MATLAB Function: '<Root>/Observer' incorporates:
  //   Integrator: '<Root>/Integrator'

  la_2dof_ctrl_obs_B.m12 = 0.083750867999999992 * cos
    (la_2dof_ctrl_obs_B.In1_d.Data[1]) + 1.253577228;
  la_2dof_ctrl_obs_B.A[0] = -(0.16750173599999998 * cos
    (la_2dof_ctrl_obs_B.In1_d.Data[1]) + 2.3443905528168);
  qe_idx_1 = sin(la_2dof_ctrl_obs_B.In1_d.Data[1]);
  la_2dof_ctrl_obs_B.G[0] = (((-0.083750867999999992 * qe_idx_1 *
    la_2dof_ctrl_obs_X.Integrator_CSTATE[3] + 1.5) *
    la_2dof_ctrl_obs_X.Integrator_CSTATE[2] + -0.083750867999999992 * sin
    (la_2dof_ctrl_obs_B.In1_d.Data[1]) * (la_2dof_ctrl_obs_X.Integrator_CSTATE[2]
    + la_2dof_ctrl_obs_X.Integrator_CSTATE[3]) *
    la_2dof_ctrl_obs_X.Integrator_CSTATE[3]) + (sin
    (la_2dof_ctrl_obs_B.In1_d.Data[0] + la_2dof_ctrl_obs_B.In1_d.Data[1]) *
    2.4874235999999996 + 5.3450766000000005 * sin(la_2dof_ctrl_obs_B.In1_d.Data
    [0]))) - la_2dof_ctrl_obs_B.torque[0];
  la_2dof_ctrl_obs_B.a21 = -la_2dof_ctrl_obs_B.m12 / la_2dof_ctrl_obs_B.A[0];
  qe_idx_1 = ((((0.083750867999999992 * qe_idx_1 *
                 la_2dof_ctrl_obs_X.Integrator_CSTATE[2] *
                 la_2dof_ctrl_obs_X.Integrator_CSTATE[2] + 1.5 *
                 la_2dof_ctrl_obs_X.Integrator_CSTATE[3]) + sin
                (la_2dof_ctrl_obs_B.In1_d.Data[0] +
                 la_2dof_ctrl_obs_B.In1_d.Data[1]) * 2.4874235999999996) -
               la_2dof_ctrl_obs_B.torque[1]) - la_2dof_ctrl_obs_B.G[0] *
              la_2dof_ctrl_obs_B.a21) / (-1.253577228 - la_2dof_ctrl_obs_B.a21 *
    -la_2dof_ctrl_obs_B.m12);
  la_2dof_ctrl_obs_B.a21 = la_2dof_ctrl_obs_B.In1_d.Data[0] -
    la_2dof_ctrl_obs_X.Integrator_CSTATE[0];
  if (la_2dof_ctrl_obs_B.a21 < 0.0) {
    la_2dof_ctrl_obs_B.a21 = -1.0;
  } else if (la_2dof_ctrl_obs_B.a21 > 0.0) {
    la_2dof_ctrl_obs_B.a21 = 1.0;
  } else if (la_2dof_ctrl_obs_B.a21 == 0.0) {
    la_2dof_ctrl_obs_B.a21 = 0.0;
  } else {
    la_2dof_ctrl_obs_B.a21 = (rtNaN);
  }

  la_2dof_ctrl_obs_B.z21 = 5.0 * la_2dof_ctrl_obs_B.a21;
  u_tmp = la_2dof_ctrl_obs_B.In1_d.Data[1] -
    la_2dof_ctrl_obs_X.Integrator_CSTATE[1];
  if (u_tmp < 0.0) {
    z22_tmp = -1.0;
  } else if (u_tmp > 0.0) {
    z22_tmp = 1.0;
  } else if (u_tmp == 0.0) {
    z22_tmp = 0.0;
  } else {
    z22_tmp = (rtNaN);
  }

  la_2dof_ctrl_obs_B.z22 = 5.0 * z22_tmp;
  la_2dof_ctrl_obs_B.xp_est[0] = sqrt(fabs(la_2dof_ctrl_obs_B.In1_d.Data[0] -
    la_2dof_ctrl_obs_X.Integrator_CSTATE[0])) * 5.0 * la_2dof_ctrl_obs_B.a21 +
    la_2dof_ctrl_obs_X.Integrator_CSTATE[2];
  la_2dof_ctrl_obs_B.xp_est[1] = sqrt(fabs(u_tmp)) * 5.0 * z22_tmp +
    la_2dof_ctrl_obs_X.Integrator_CSTATE[3];
  la_2dof_ctrl_obs_B.xp_est[2] = (la_2dof_ctrl_obs_B.G[0] - qe_idx_1 *
    -la_2dof_ctrl_obs_B.m12) / la_2dof_ctrl_obs_B.A[0] + la_2dof_ctrl_obs_B.z21;
  la_2dof_ctrl_obs_B.xp_est[3] = qe_idx_1 + la_2dof_ctrl_obs_B.z22;
  la_2dof_ctrl_obs_B.xp_est[4] = la_2dof_ctrl_obs_B.z21;
  la_2dof_ctrl_obs_B.xp_est[5] = la_2dof_ctrl_obs_B.z22;

  // End of MATLAB Function: '<Root>/Observer'
  if (rtmIsMajorTimeStep(la_2dof_ctrl_obs_M)) {
    // BusAssignment: '<Root>/Bus Assignment1' incorporates:
    //   Constant: '<Root>/Constant'
    //   Constant: '<S1>/Constant'

    la_2dof_ctrl_obs_B.BusAssignment1 = la_2dof_ctrl_obs_P.Constant_Value_j;
    la_2dof_ctrl_obs_B.BusAssignment1.Data[0] = la_2dof_ctrl_obs_B.torque[0];
    la_2dof_ctrl_obs_B.BusAssignment1.Data[1] = la_2dof_ctrl_obs_B.torque[1];
    la_2dof_ctrl_obs_B.BusAssignment1.Data_SL_Info.CurrentLength =
      la_2dof_ctrl_obs_P.Constant_Value_l;
    la_2dof_ctrl_obs_B.BusAssignment1.Data_SL_Info.ReceivedLength =
      la_2dof_ctrl_obs_P.Constant_Value_l;

    // Outputs for Atomic SubSystem: '<Root>/Publish1'
    // MATLABSystem: '<S5>/SinkBlock'
    Pub_la_2dof_ctrl_obs_201.publish(&la_2dof_ctrl_obs_B.BusAssignment1);

    // End of Outputs for SubSystem: '<Root>/Publish1'
  }

  // TransferFcn: '<Root>/Low Pass (z1)'
  la_2dof_ctrl_obs_B.m12 = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)'
  qe_idx_1 = 0.0;
  for (ci = 0; ci < 5; ci++) {
    // TransferFcn: '<Root>/Low Pass (z1)'
    la_2dof_ctrl_obs_B.m12 += la_2dof_ctrl_obs_P.LowPassz1_C[ci] *
      la_2dof_ctrl_obs_X.LowPassz1_CSTATE[ci];

    // TransferFcn: '<Root>/Low Pass (z2)'
    qe_idx_1 += la_2dof_ctrl_obs_P.LowPassz2_C[ci] *
      la_2dof_ctrl_obs_X.LowPassz2_CSTATE[ci];
  }

  // MATLAB Function: '<Root>/mass estimator' incorporates:
  //   SignalConversion generated from: '<S9>/ SFunction '

  la_2dof_ctrl_obs_B.a21 = sin(la_2dof_ctrl_obs_B.In1_d.Data[0] +
    la_2dof_ctrl_obs_B.In1_d.Data[1]);
  if (fabs(la_2dof_ctrl_obs_B.a21) > 0.05) {
    la_2dof_ctrl_obs_B.m12 = -((0.083750867999999992 * cos
      (la_2dof_ctrl_obs_B.In1_d.Data[1]) + 1.253577228) * la_2dof_ctrl_obs_B.m12
      + 1.253577228 * qe_idx_1) / (3.9495060000000004 * la_2dof_ctrl_obs_B.a21);
  } else {
    la_2dof_ctrl_obs_B.m12 = 0.0;
  }

  // End of MATLAB Function: '<Root>/mass estimator'
  if (rtmIsMajorTimeStep(la_2dof_ctrl_obs_M)) {
    // BusAssignment: '<Root>/Bus Assignment2'
    rtb_BusAssignment2.Data = la_2dof_ctrl_obs_B.m12;

    // Outputs for Atomic SubSystem: '<Root>/Publish2'
    // MATLABSystem: '<S6>/SinkBlock'
    Pub_la_2dof_ctrl_obs_206.publish(&rtb_BusAssignment2);

    // End of Outputs for SubSystem: '<Root>/Publish2'
  }

  if (rtmIsMajorTimeStep(la_2dof_ctrl_obs_M)) {
    rt_ertODEUpdateContinuousStates(&la_2dof_ctrl_obs_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++la_2dof_ctrl_obs_M->Timing.clockTick0;
    la_2dof_ctrl_obs_M->Timing.t[0] = rtsiGetSolverStopTime
      (&la_2dof_ctrl_obs_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.001s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.001, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      la_2dof_ctrl_obs_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void la_2dof_ctrl_obs_derivatives(void)
{
  int_T is;
  XDot_la_2dof_ctrl_obs_T *_rtXdot;
  _rtXdot = ((XDot_la_2dof_ctrl_obs_T *) la_2dof_ctrl_obs_M->derivs);

  // Derivatives for Integrator: '<Root>/Integrator'
  _rtXdot->Integrator_CSTATE[0] = la_2dof_ctrl_obs_B.xp_est[0];
  _rtXdot->Integrator_CSTATE[1] = la_2dof_ctrl_obs_B.xp_est[1];
  _rtXdot->Integrator_CSTATE[2] = la_2dof_ctrl_obs_B.xp_est[2];
  _rtXdot->Integrator_CSTATE[3] = la_2dof_ctrl_obs_B.xp_est[3];
  for (is = 0; is < 5; is++) {
    // Derivatives for TransferFcn: '<Root>/Low Pass (z1)'
    _rtXdot->LowPassz1_CSTATE[is] = 0.0;
    _rtXdot->LowPassz1_CSTATE[0] += la_2dof_ctrl_obs_P.LowPassz1_A[is] *
      la_2dof_ctrl_obs_X.LowPassz1_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)'
    _rtXdot->LowPassz2_CSTATE[is] = 0.0;
    _rtXdot->LowPassz2_CSTATE[0] += la_2dof_ctrl_obs_P.LowPassz2_A[is] *
      la_2dof_ctrl_obs_X.LowPassz2_CSTATE[is];
  }

  // Derivatives for TransferFcn: '<Root>/Low Pass (z1)'
  _rtXdot->LowPassz1_CSTATE[1] += la_2dof_ctrl_obs_X.LowPassz1_CSTATE[0];
  _rtXdot->LowPassz1_CSTATE[2] += la_2dof_ctrl_obs_X.LowPassz1_CSTATE[1];
  _rtXdot->LowPassz1_CSTATE[3] += la_2dof_ctrl_obs_X.LowPassz1_CSTATE[2];
  _rtXdot->LowPassz1_CSTATE[4] += la_2dof_ctrl_obs_X.LowPassz1_CSTATE[3];
  _rtXdot->LowPassz1_CSTATE[0] += la_2dof_ctrl_obs_B.xp_est[4];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)'
  _rtXdot->LowPassz2_CSTATE[1] += la_2dof_ctrl_obs_X.LowPassz2_CSTATE[0];
  _rtXdot->LowPassz2_CSTATE[2] += la_2dof_ctrl_obs_X.LowPassz2_CSTATE[1];
  _rtXdot->LowPassz2_CSTATE[3] += la_2dof_ctrl_obs_X.LowPassz2_CSTATE[2];
  _rtXdot->LowPassz2_CSTATE[4] += la_2dof_ctrl_obs_X.LowPassz2_CSTATE[3];
  _rtXdot->LowPassz2_CSTATE[0] += la_2dof_ctrl_obs_B.xp_est[5];
}

// Model initialize function
void la_2dof_ctrl_obs_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&la_2dof_ctrl_obs_M->solverInfo,
                          &la_2dof_ctrl_obs_M->Timing.simTimeStep);
    rtsiSetTPtr(&la_2dof_ctrl_obs_M->solverInfo, &rtmGetTPtr(la_2dof_ctrl_obs_M));
    rtsiSetStepSizePtr(&la_2dof_ctrl_obs_M->solverInfo,
                       &la_2dof_ctrl_obs_M->Timing.stepSize0);
    rtsiSetdXPtr(&la_2dof_ctrl_obs_M->solverInfo, &la_2dof_ctrl_obs_M->derivs);
    rtsiSetContStatesPtr(&la_2dof_ctrl_obs_M->solverInfo, (real_T **)
                         &la_2dof_ctrl_obs_M->contStates);
    rtsiSetNumContStatesPtr(&la_2dof_ctrl_obs_M->solverInfo,
      &la_2dof_ctrl_obs_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&la_2dof_ctrl_obs_M->solverInfo,
      &la_2dof_ctrl_obs_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&la_2dof_ctrl_obs_M->solverInfo,
      &la_2dof_ctrl_obs_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&la_2dof_ctrl_obs_M->solverInfo,
      &la_2dof_ctrl_obs_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&la_2dof_ctrl_obs_M->solverInfo, (&rtmGetErrorStatus
      (la_2dof_ctrl_obs_M)));
    rtsiSetRTModelPtr(&la_2dof_ctrl_obs_M->solverInfo, la_2dof_ctrl_obs_M);
  }

  rtsiSetSimTimeStep(&la_2dof_ctrl_obs_M->solverInfo, MAJOR_TIME_STEP);
  la_2dof_ctrl_obs_M->intgData.y = la_2dof_ctrl_obs_M->odeY;
  la_2dof_ctrl_obs_M->intgData.f[0] = la_2dof_ctrl_obs_M->odeF[0];
  la_2dof_ctrl_obs_M->intgData.f[1] = la_2dof_ctrl_obs_M->odeF[1];
  la_2dof_ctrl_obs_M->intgData.f[2] = la_2dof_ctrl_obs_M->odeF[2];
  la_2dof_ctrl_obs_M->contStates = ((X_la_2dof_ctrl_obs_T *) &la_2dof_ctrl_obs_X);
  rtsiSetSolverData(&la_2dof_ctrl_obs_M->solverInfo, static_cast<void *>
                    (&la_2dof_ctrl_obs_M->intgData));
  rtsiSetSolverName(&la_2dof_ctrl_obs_M->solverInfo,"ode3");
  rtmSetTPtr(la_2dof_ctrl_obs_M, &la_2dof_ctrl_obs_M->Timing.tArray[0]);
  la_2dof_ctrl_obs_M->Timing.stepSize0 = 0.001;

  {
    int_T is;
    char_T tmp[20];
    char_T tmp_0[23];
    static const char_T tmp_1[25] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'c', 'u', 'r', 'r', 'e', 'n', 't', '_', 'p', 'o',
      's', 'e' };

    static const char_T tmp_2[22] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'g', 'o', 'a', 'l', '_', 'p', 'o', 's', 'e' };

    static const char_T tmp_3[19] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 't', 'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_4[27] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'e', 's', 't', 'i', 'm', 'a', 't', 'e', 'd', '_',
      'm', 'a', 's', 's' };

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S7>/SourceBlock'
    la_2dof_ctrl_obs_DW.obj_a.matlabCodegenIsDeleted = false;
    la_2dof_ctrl_obs_DW.obj_a.isInitialized = 1;
    for (is = 0; is < 25; is++) {
      la_2dof_ctrl_obs_B.cv1[is] = tmp_1[is];
    }

    la_2dof_ctrl_obs_B.cv1[25] = '\x00';
    Sub_la_2dof_ctrl_obs_193.createSubscriber(la_2dof_ctrl_obs_B.cv1, 1);
    la_2dof_ctrl_obs_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for Atomic SubSystem: '<Root>/Subscribe1'
    // Start for MATLABSystem: '<S8>/SourceBlock'
    la_2dof_ctrl_obs_DW.obj_e.matlabCodegenIsDeleted = false;
    la_2dof_ctrl_obs_DW.obj_e.isInitialized = 1;
    for (is = 0; is < 22; is++) {
      tmp_0[is] = tmp_2[is];
    }

    tmp_0[22] = '\x00';
    Sub_la_2dof_ctrl_obs_195.createSubscriber(tmp_0, 1);
    la_2dof_ctrl_obs_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe1'

    // Start for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S5>/SinkBlock'
    la_2dof_ctrl_obs_DW.obj_d.matlabCodegenIsDeleted = false;
    la_2dof_ctrl_obs_DW.obj_d.isInitialized = 1;
    for (is = 0; is < 19; is++) {
      tmp[is] = tmp_3[is];
    }

    tmp[19] = '\x00';
    Pub_la_2dof_ctrl_obs_201.createPublisher(tmp, 1);
    la_2dof_ctrl_obs_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish1'

    // Start for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S6>/SinkBlock'
    la_2dof_ctrl_obs_DW.obj.matlabCodegenIsDeleted = false;
    la_2dof_ctrl_obs_DW.obj.isInitialized = 1;
    for (is = 0; is < 27; is++) {
      la_2dof_ctrl_obs_B.cv[is] = tmp_4[is];
    }

    la_2dof_ctrl_obs_B.cv[27] = '\x00';
    Pub_la_2dof_ctrl_obs_206.createPublisher(la_2dof_ctrl_obs_B.cv, 1);
    la_2dof_ctrl_obs_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish2'

    // InitializeConditions for Integrator: '<Root>/Integrator'
    la_2dof_ctrl_obs_X.Integrator_CSTATE[0] = la_2dof_ctrl_obs_P.Integrator_IC[0];
    la_2dof_ctrl_obs_X.Integrator_CSTATE[1] = la_2dof_ctrl_obs_P.Integrator_IC[1];
    la_2dof_ctrl_obs_X.Integrator_CSTATE[2] = la_2dof_ctrl_obs_P.Integrator_IC[2];
    la_2dof_ctrl_obs_X.Integrator_CSTATE[3] = la_2dof_ctrl_obs_P.Integrator_IC[3];
    for (is = 0; is < 5; is++) {
      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z1)'
      la_2dof_ctrl_obs_X.LowPassz1_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)'
      la_2dof_ctrl_obs_X.LowPassz2_CSTATE[is] = 0.0;
    }

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S7>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S10>/Out1'
    la_2dof_ctrl_obs_B.In1_d = la_2dof_ctrl_obs_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S7>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S11>/Out1'
    la_2dof_ctrl_obs_B.In1 = la_2dof_ctrl_obs_P.Out1_Y0_o;

    // End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'
  }
}

// Model terminate function
void la_2dof_ctrl_obs_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S7>/SourceBlock'
  matlabCodegenHandle_matlabCo_ot(&la_2dof_ctrl_obs_DW.obj_a);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S8>/SourceBlock'
  matlabCodegenHandle_matlabCo_ot(&la_2dof_ctrl_obs_DW.obj_e);

  // End of Terminate for SubSystem: '<Root>/Subscribe1'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S5>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&la_2dof_ctrl_obs_DW.obj_d);

  // End of Terminate for SubSystem: '<Root>/Publish1'

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S6>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&la_2dof_ctrl_obs_DW.obj);

  // End of Terminate for SubSystem: '<Root>/Publish2'
}

//
// File trailer for generated code.
//
// [EOF]
//
