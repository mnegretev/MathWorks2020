//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_simul.cpp
//
// Code generated for Simulink model 'left_arm_simul'.
//
// Model version                  : 1.128
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Tue Jul 21 20:46:03 2020
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
static real_T left_arm_sim_eml_rand_mt19937ar(uint32_T state[625]);
static real_T left_arm_simul_rand(void);
static void matlabCodegenHandle_matlab_k10o(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHandle_matlabCod_k(ros_slros_internal_block_Curr_T *obj);
static void matlabCodegenHandle_matlabCo_k1(ros_slros_internal_block_Publ_T *obj);
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
  if ((left_arm_simul_M->Timing.TaskCounters.TID[2]) > 9) {// Sample time: [0.04s, 0.0s] 
    left_arm_simul_M->Timing.TaskCounters.TID[2] = 0;
  }

  (left_arm_simul_M->Timing.TaskCounters.TID[3])++;
  if ((left_arm_simul_M->Timing.TaskCounters.TID[3]) > 249) {// Sample time: [1.0s, 0.0s] 
    left_arm_simul_M->Timing.TaskCounters.TID[3] = 0;
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

  // Projection for SimscapeExecutionBlock: '<S49>/STATE_1'
  simulationData = (NeslSimulationData *)left_arm_simul_DW.STATE_1_SimData;
  time = left_arm_simul_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 14;
  simulationData->mData->mContStates.mX =
    &left_arm_simul_X.left_arm_simulManipulatorla_1_j[0];
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
  left_arm_simul_B.dv3[0] = left_arm_simul_B.INPUT_8_1_1[0];
  left_arm_simul_B.dv3[1] = left_arm_simul_B.INPUT_8_1_1[1];
  left_arm_simul_B.dv3[2] = left_arm_simul_B.INPUT_8_1_1[2];
  left_arm_simul_B.dv3[3] = left_arm_simul_B.INPUT_8_1_1[3];
  left_arm_simul_B.iv3[1] = 4;
  left_arm_simul_B.dv3[4] = left_arm_simul_B.INPUT_1_1_1[0];
  left_arm_simul_B.dv3[5] = left_arm_simul_B.INPUT_1_1_1[1];
  left_arm_simul_B.dv3[6] = left_arm_simul_B.INPUT_1_1_1[2];
  left_arm_simul_B.dv3[7] = left_arm_simul_B.INPUT_1_1_1[3];
  left_arm_simul_B.iv3[2] = 8;
  left_arm_simul_B.dv3[8] = left_arm_simul_B.INPUT_2_1_1[0];
  left_arm_simul_B.dv3[9] = left_arm_simul_B.INPUT_2_1_1[1];
  left_arm_simul_B.dv3[10] = left_arm_simul_B.INPUT_2_1_1[2];
  left_arm_simul_B.dv3[11] = left_arm_simul_B.INPUT_2_1_1[3];
  left_arm_simul_B.iv3[3] = 12;
  left_arm_simul_B.dv3[12] = left_arm_simul_B.INPUT_3_1_1[0];
  left_arm_simul_B.dv3[13] = left_arm_simul_B.INPUT_3_1_1[1];
  left_arm_simul_B.dv3[14] = left_arm_simul_B.INPUT_3_1_1[2];
  left_arm_simul_B.dv3[15] = left_arm_simul_B.INPUT_3_1_1[3];
  left_arm_simul_B.iv3[4] = 16;
  left_arm_simul_B.dv3[16] = left_arm_simul_B.INPUT_4_1_1[0];
  left_arm_simul_B.dv3[17] = left_arm_simul_B.INPUT_4_1_1[1];
  left_arm_simul_B.dv3[18] = left_arm_simul_B.INPUT_4_1_1[2];
  left_arm_simul_B.dv3[19] = left_arm_simul_B.INPUT_4_1_1[3];
  left_arm_simul_B.iv3[5] = 20;
  left_arm_simul_B.dv3[20] = left_arm_simul_B.INPUT_5_1_1[0];
  left_arm_simul_B.dv3[21] = left_arm_simul_B.INPUT_5_1_1[1];
  left_arm_simul_B.dv3[22] = left_arm_simul_B.INPUT_5_1_1[2];
  left_arm_simul_B.dv3[23] = left_arm_simul_B.INPUT_5_1_1[3];
  left_arm_simul_B.iv3[6] = 24;
  left_arm_simul_B.dv3[24] = left_arm_simul_B.INPUT_6_1_1[0];
  left_arm_simul_B.dv3[25] = left_arm_simul_B.INPUT_6_1_1[1];
  left_arm_simul_B.dv3[26] = left_arm_simul_B.INPUT_6_1_1[2];
  left_arm_simul_B.dv3[27] = left_arm_simul_B.INPUT_6_1_1[3];
  left_arm_simul_B.iv3[7] = 28;
  left_arm_simul_B.dv3[28] = left_arm_simul_B.INPUT_7_1_1[0];
  left_arm_simul_B.dv3[29] = left_arm_simul_B.INPUT_7_1_1[1];
  left_arm_simul_B.dv3[30] = left_arm_simul_B.INPUT_7_1_1[2];
  left_arm_simul_B.dv3[31] = left_arm_simul_B.INPUT_7_1_1[3];
  left_arm_simul_B.iv3[8] = 32;
  simulationData->mData->mInputValues.mN = 32;
  simulationData->mData->mInputValues.mX = &left_arm_simul_B.dv3[0];
  simulationData->mData->mInputOffsets.mN = 9;
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

  // End of Projection for SimscapeExecutionBlock: '<S49>/STATE_1'
}

//
// This function updates continuous states using the ODE5 fixed-step
// solver algorithm
//
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE5_A[6] = {
    1.0/5.0, 3.0/10.0, 4.0/5.0, 8.0/9.0, 1.0, 1.0
  };

  static const real_T rt_ODE5_B[6][6] = {
    { 1.0/5.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

    { 3.0/40.0, 9.0/40.0, 0.0, 0.0, 0.0, 0.0 },

    { 44.0/45.0, -56.0/15.0, 32.0/9.0, 0.0, 0.0, 0.0 },

    { 19372.0/6561.0, -25360.0/2187.0, 64448.0/6561.0, -212.0/729.0, 0.0, 0.0 },

    { 9017.0/3168.0, -355.0/33.0, 46732.0/5247.0, 49.0/176.0, -5103.0/18656.0,
      0.0 },

    { 35.0/384.0, 0.0, 500.0/1113.0, 125.0/192.0, -2187.0/6784.0, 11.0/84.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE5_IntgData *id = static_cast<ODE5_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T *f4 = id->f[4];
  real_T *f5 = id->f[5];
  real_T hB[6];
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

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE5_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[0]);
  rtsiSetdX(si, f1);
  left_arm_simul_step();
  left_arm_simul_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE5_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[1]);
  rtsiSetdX(si, f2);
  left_arm_simul_step();
  left_arm_simul_derivatives();

  // f(:,4) = feval(odefile, t + hA(3), y + f*hB(:,3), args(:)(*));
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE5_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[2]);
  rtsiSetdX(si, f3);
  left_arm_simul_step();
  left_arm_simul_derivatives();

  // f(:,5) = feval(odefile, t + hA(4), y + f*hB(:,4), args(:)(*));
  for (i = 0; i <= 3; i++) {
    hB[i] = h * rt_ODE5_B[3][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2] +
                   f3[i]*hB[3]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[3]);
  rtsiSetdX(si, f4);
  left_arm_simul_step();
  left_arm_simul_derivatives();

  // f(:,6) = feval(odefile, t + hA(5), y + f*hB(:,5), args(:)(*));
  for (i = 0; i <= 4; i++) {
    hB[i] = h * rt_ODE5_B[4][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2] +
                   f3[i]*hB[3] + f4[i]*hB[4]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f5);
  left_arm_simul_step();
  left_arm_simul_derivatives();

  // tnew = t + hA(6);
  // ynew = y + f*hB(:,6);
  for (i = 0; i <= 5; i++) {
    hB[i] = h * rt_ODE5_B[5][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2] +
                   f3[i]*hB[3] + f4[i]*hB[4] + f5[i]*hB[5]);
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

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

// Function for MATLAB Function: '<S1>/Pose Quantization and Noise'
static real_T left_arm_sim_eml_rand_mt19937ar(uint32_T state[625])
{
  real_T r;
  uint32_T mti;
  uint32_T y;
  int32_T kk;
  int32_T k;
  boolean_T b_isvalid;
  int32_T exitg1;
  boolean_T exitg2;

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  do {
    exitg1 = 0;
    for (k = 0; k < 2; k++) {
      mti = state[624] + 1U;
      if (mti >= 625U) {
        for (kk = 0; kk < 227; kk++) {
          y = (state[kk + 1] & 2147483647U) | (state[kk] & 2147483648U);
          if ((y & 1U) == 0U) {
            y >>= 1U;
          } else {
            y = y >> 1U ^ 2567483615U;
          }

          state[kk] = state[kk + 397] ^ y;
        }

        for (kk = 0; kk < 396; kk++) {
          y = (state[kk + 227] & 2147483648U) | (state[kk + 228] & 2147483647U);
          if ((y & 1U) == 0U) {
            y >>= 1U;
          } else {
            y = y >> 1U ^ 2567483615U;
          }

          state[kk + 227] = state[kk] ^ y;
        }

        y = (state[623] & 2147483648U) | (state[0] & 2147483647U);
        if ((y & 1U) == 0U) {
          y >>= 1U;
        } else {
          y = y >> 1U ^ 2567483615U;
        }

        state[623] = state[396] ^ y;
        mti = 1U;
      }

      y = state[static_cast<int32_T>(mti) - 1];
      state[624] = mti;
      y ^= y >> 11U;
      y ^= y << 7U & 2636928640U;
      y ^= y << 15U & 4022730752U;
      left_arm_simul_B.u[k] = y >> 18U ^ y;
    }

    r = (static_cast<real_T>(left_arm_simul_B.u[0] >> 5U) * 6.7108864E+7 +
         static_cast<real_T>(left_arm_simul_B.u[1] >> 6U)) *
      1.1102230246251565E-16;
    if (r == 0.0) {
      b_isvalid = ((state[624] >= 1U) && (state[624] < 625U));
      if (b_isvalid) {
        b_isvalid = false;
        k = 1;
        exitg2 = false;
        while ((!exitg2) && (k < 625)) {
          if (state[k - 1] == 0U) {
            k++;
          } else {
            b_isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!b_isvalid) {
        mti = 5489U;
        state[0] = 5489U;
        for (k = 0; k < 623; k++) {
          mti = ((mti >> 30U ^ mti) * 1812433253U + k) + 1U;
          state[k + 1] = mti;
        }

        state[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

// Function for MATLAB Function: '<S1>/Pose Quantization and Noise'
static real_T left_arm_simul_rand(void)
{
  real_T r;
  int32_T hi;
  uint32_T test2;
  uint32_T b_r;
  switch (left_arm_simul_DW.method) {
   case 4U:
    hi = static_cast<int32_T>(left_arm_simul_DW.state / 127773U);
    b_r = (left_arm_simul_DW.state - hi * 127773U) * 16807U;
    test2 = 2836U * hi;
    if (b_r < test2) {
      b_r = ~(test2 - b_r) & 2147483647U;
    } else {
      b_r -= test2;
    }

    left_arm_simul_DW.state = b_r;
    r = static_cast<real_T>(b_r) * 4.6566128752457969E-10;
    break;

   case 5U:
    b_r = 69069U * left_arm_simul_DW.state_b[0] + 1234567U;
    test2 = left_arm_simul_DW.state_b[1] << 13 ^ left_arm_simul_DW.state_b[1];
    test2 ^= test2 >> 17;
    test2 ^= test2 << 5;
    left_arm_simul_DW.state_b[0] = b_r;
    left_arm_simul_DW.state_b[1] = test2;
    r = static_cast<real_T>(b_r + test2) * 2.328306436538696E-10;
    break;

   default:
    if (!left_arm_simul_DW.state_not_empty) {
      memset(&left_arm_simul_DW.state_i[0], 0, 625U * sizeof(uint32_T));
      b_r = 5489U;
      left_arm_simul_DW.state_i[0] = 5489U;
      for (hi = 0; hi < 623; hi++) {
        b_r = ((b_r >> 30U ^ b_r) * 1812433253U + hi) + 1U;
        left_arm_simul_DW.state_i[hi + 1] = b_r;
      }

      left_arm_simul_DW.state_i[624] = 624U;
      left_arm_simul_DW.state_not_empty = true;
    }

    r = left_arm_sim_eml_rand_mt19937ar(left_arm_simul_DW.state_i);
    break;
  }

  return r;
}

static void matlabCodegenHandle_matlab_k10o(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCod_k(ros_slros_internal_block_Curr_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCo_k1(ros_slros_internal_block_Publ_T *obj)
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
    boolean_T b_varargout_1;
    static const real_T b[7] = { 138.88888888888889, 138.88888888888889,
      158.29542289481014, 138.88888888888889, 158.29542289481014,
      158.29542289481014, 158.29542289481014 };

    static const uint8_T b_0[10] = { 108U, 97U, 95U, 49U, 95U, 106U, 111U, 105U,
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

    // SimscapeExecutionBlock: '<S49>/STATE_1' incorporates:
    //   SimscapeExecutionBlock: '<S49>/OUTPUT_1_0'

    simulationData = (NeslSimulationData *)left_arm_simul_DW.STATE_1_SimData;
    left_arm_simul_B.time_tmp = left_arm_simul_M->Timing.t[0];
    left_arm_simul_B.time = left_arm_simul_B.time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &left_arm_simul_B.time;
    simulationData->mData->mContStates.mN = 14;
    simulationData->mData->mContStates.mX =
      &left_arm_simul_X.left_arm_simulManipulatorla_1_j[0];
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
    left_arm_simul_B.dv1[0] = left_arm_simul_B.INPUT_8_1_1[0];
    left_arm_simul_B.dv1[1] = left_arm_simul_B.INPUT_8_1_1[1];
    left_arm_simul_B.dv1[2] = left_arm_simul_B.INPUT_8_1_1[2];
    left_arm_simul_B.dv1[3] = left_arm_simul_B.INPUT_8_1_1[3];
    left_arm_simul_B.iv1[1] = 4;
    left_arm_simul_B.dv1[4] = left_arm_simul_B.INPUT_1_1_1[0];
    left_arm_simul_B.dv1[5] = left_arm_simul_B.INPUT_1_1_1[1];
    left_arm_simul_B.dv1[6] = left_arm_simul_B.INPUT_1_1_1[2];
    left_arm_simul_B.dv1[7] = left_arm_simul_B.INPUT_1_1_1[3];
    left_arm_simul_B.iv1[2] = 8;
    left_arm_simul_B.dv1[8] = left_arm_simul_B.INPUT_2_1_1[0];
    left_arm_simul_B.dv1[9] = left_arm_simul_B.INPUT_2_1_1[1];
    left_arm_simul_B.dv1[10] = left_arm_simul_B.INPUT_2_1_1[2];
    left_arm_simul_B.dv1[11] = left_arm_simul_B.INPUT_2_1_1[3];
    left_arm_simul_B.iv1[3] = 12;
    left_arm_simul_B.dv1[12] = left_arm_simul_B.INPUT_3_1_1[0];
    left_arm_simul_B.dv1[13] = left_arm_simul_B.INPUT_3_1_1[1];
    left_arm_simul_B.dv1[14] = left_arm_simul_B.INPUT_3_1_1[2];
    left_arm_simul_B.dv1[15] = left_arm_simul_B.INPUT_3_1_1[3];
    left_arm_simul_B.iv1[4] = 16;
    left_arm_simul_B.dv1[16] = left_arm_simul_B.INPUT_4_1_1[0];
    left_arm_simul_B.dv1[17] = left_arm_simul_B.INPUT_4_1_1[1];
    left_arm_simul_B.dv1[18] = left_arm_simul_B.INPUT_4_1_1[2];
    left_arm_simul_B.dv1[19] = left_arm_simul_B.INPUT_4_1_1[3];
    left_arm_simul_B.iv1[5] = 20;
    left_arm_simul_B.dv1[20] = left_arm_simul_B.INPUT_5_1_1[0];
    left_arm_simul_B.dv1[21] = left_arm_simul_B.INPUT_5_1_1[1];
    left_arm_simul_B.dv1[22] = left_arm_simul_B.INPUT_5_1_1[2];
    left_arm_simul_B.dv1[23] = left_arm_simul_B.INPUT_5_1_1[3];
    left_arm_simul_B.iv1[6] = 24;
    left_arm_simul_B.dv1[24] = left_arm_simul_B.INPUT_6_1_1[0];
    left_arm_simul_B.dv1[25] = left_arm_simul_B.INPUT_6_1_1[1];
    left_arm_simul_B.dv1[26] = left_arm_simul_B.INPUT_6_1_1[2];
    left_arm_simul_B.dv1[27] = left_arm_simul_B.INPUT_6_1_1[3];
    left_arm_simul_B.iv1[7] = 28;
    left_arm_simul_B.dv1[28] = left_arm_simul_B.INPUT_7_1_1[0];
    left_arm_simul_B.dv1[29] = left_arm_simul_B.INPUT_7_1_1[1];
    left_arm_simul_B.dv1[30] = left_arm_simul_B.INPUT_7_1_1[2];
    left_arm_simul_B.dv1[31] = left_arm_simul_B.INPUT_7_1_1[3];
    left_arm_simul_B.iv1[8] = 32;
    simulationData->mData->mInputValues.mN = 32;
    simulationData->mData->mInputValues.mX = &left_arm_simul_B.dv1[0];
    simulationData->mData->mInputOffsets.mN = 9;
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
    left_arm_simul_B.i = ne_simulator_method((NeslSimulator *)
      left_arm_simul_DW.STATE_1_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diagnosticManager);
    if (left_arm_simul_B.i != 0) {
      b_varargout_1 = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
      if (b_varargout_1) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    if (rtmIsMajorTimeStep(left_arm_simul_M) &&
        left_arm_simul_M->Timing.TaskCounters.TID[2] == 0) {
      // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
      // MATLABSystem: '<S4>/SourceBlock' incorporates:
      //   Inport: '<S57>/In1'

      b_varargout_1 = Sub_left_arm_simul_272.getLatestMessage
        (&left_arm_simul_B.b_varargout_2_b);

      // Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S57>/Enable'

      if (b_varargout_1) {
        left_arm_simul_B.In1_j = left_arm_simul_B.b_varargout_2_b;
      }

      // End of MATLABSystem: '<S4>/SourceBlock'
      // End of Outputs for SubSystem: '<S4>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<Root>/Subscribe1'
    }

    // SimscapeInputBlock: '<S49>/INPUT_8_1_1'
    left_arm_simul_B.INPUT_8_1_1[0] = left_arm_simul_B.In1_j.Data;
    left_arm_simul_B.INPUT_8_1_1[1] = 0.0;
    left_arm_simul_B.INPUT_8_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(left_arm_simul_M)) {
      left_arm_simul_DW.INPUT_8_1_1_Discrete[0] =
        !(left_arm_simul_B.INPUT_8_1_1[0] ==
          left_arm_simul_DW.INPUT_8_1_1_Discrete[1]);
      left_arm_simul_DW.INPUT_8_1_1_Discrete[1] = left_arm_simul_B.INPUT_8_1_1[0];
    }

    left_arm_simul_B.INPUT_8_1_1[0] = left_arm_simul_DW.INPUT_8_1_1_Discrete[1];
    left_arm_simul_B.INPUT_8_1_1[3] = left_arm_simul_DW.INPUT_8_1_1_Discrete[0];

    // End of SimscapeInputBlock: '<S49>/INPUT_8_1_1'
    if (rtmIsMajorTimeStep(left_arm_simul_M) &&
        left_arm_simul_M->Timing.TaskCounters.TID[1] == 0) {
      // Outputs for Atomic SubSystem: '<Root>/Subscribe'
      // MATLABSystem: '<S3>/SourceBlock' incorporates:
      //   Inport: '<S56>/In1'

      left_arm_simul_SystemCore_step(&b_varargout_1,
        left_arm_simul_B.b_varargout_2_Data,
        &left_arm_simul_B.b_varargout_2_Data_SL_Info_Curr,
        &left_arm_simul_B.b_varargout_2_Data_SL_Info_Rece,
        &left_arm_simul_B.b_varargout_2_Layout_DataOffset,
        left_arm_simul_B.b_varargout_2_Layout_Dim,
        &left_arm_simul_B.b_varargout_2_Layout_Dim_SL_Inf,
        &left_arm_simul_B.b_varargout_2_Layout_Dim_SL_I_g);

      // Outputs for Enabled SubSystem: '<S3>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S56>/Enable'

      if (b_varargout_1) {
        for (left_arm_simul_B.i = 0; left_arm_simul_B.i < 7; left_arm_simul_B.i
             ++) {
          left_arm_simul_B.In1.Data[left_arm_simul_B.i] =
            left_arm_simul_B.b_varargout_2_Data[left_arm_simul_B.i];
        }

        left_arm_simul_B.In1.Data_SL_Info.CurrentLength =
          left_arm_simul_B.b_varargout_2_Data_SL_Info_Curr;
        left_arm_simul_B.In1.Data_SL_Info.ReceivedLength =
          left_arm_simul_B.b_varargout_2_Data_SL_Info_Rece;
        left_arm_simul_B.In1.Layout.DataOffset =
          left_arm_simul_B.b_varargout_2_Layout_DataOffset;
        memcpy(&left_arm_simul_B.In1.Layout.Dim[0],
               &left_arm_simul_B.b_varargout_2_Layout_Dim[0], sizeof
               (SL_Bus_left_arm_simul_std_msgs_MultiArrayDimension) << 4U);
        left_arm_simul_B.In1.Layout.Dim_SL_Info.CurrentLength =
          left_arm_simul_B.b_varargout_2_Layout_Dim_SL_Inf;
        left_arm_simul_B.In1.Layout.Dim_SL_Info.ReceivedLength =
          left_arm_simul_B.b_varargout_2_Layout_Dim_SL_I_g;
      }

      // End of MATLABSystem: '<S3>/SourceBlock'
      // End of Outputs for SubSystem: '<S3>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<Root>/Subscribe'

      // MATLAB Function: '<S1>/Torque Quantization' incorporates:
      //   SimscapeExecutionBlock: '<S49>/OUTPUT_1_0'
      //   SimscapeExecutionBlock: '<S49>/STATE_1'

      for (left_arm_simul_B.i = 0; left_arm_simul_B.i < 7; left_arm_simul_B.i++)
      {
        left_arm_simul_B.quantized_torque[left_arm_simul_B.i] = rt_roundd_snf
          (left_arm_simul_B.In1.Data[left_arm_simul_B.i] * b[left_arm_simul_B.i])
          / b[left_arm_simul_B.i];
      }

      // End of MATLAB Function: '<S1>/Torque Quantization'
    }

    // SimscapeInputBlock: '<S49>/INPUT_1_1_1'
    left_arm_simul_B.INPUT_1_1_1[0] = left_arm_simul_B.quantized_torque[0];
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

    // End of SimscapeInputBlock: '<S49>/INPUT_1_1_1'

    // SimscapeInputBlock: '<S49>/INPUT_2_1_1'
    left_arm_simul_B.INPUT_2_1_1[0] = left_arm_simul_B.quantized_torque[1];
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

    // End of SimscapeInputBlock: '<S49>/INPUT_2_1_1'

    // SimscapeInputBlock: '<S49>/INPUT_3_1_1'
    left_arm_simul_B.INPUT_3_1_1[0] = left_arm_simul_B.quantized_torque[2];
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

    // End of SimscapeInputBlock: '<S49>/INPUT_3_1_1'

    // SimscapeInputBlock: '<S49>/INPUT_4_1_1'
    left_arm_simul_B.INPUT_4_1_1[0] = left_arm_simul_B.quantized_torque[3];
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

    // End of SimscapeInputBlock: '<S49>/INPUT_4_1_1'

    // SimscapeInputBlock: '<S49>/INPUT_5_1_1'
    left_arm_simul_B.INPUT_5_1_1[0] = left_arm_simul_B.quantized_torque[4];
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

    // End of SimscapeInputBlock: '<S49>/INPUT_5_1_1'

    // SimscapeInputBlock: '<S49>/INPUT_6_1_1'
    left_arm_simul_B.INPUT_6_1_1[0] = left_arm_simul_B.quantized_torque[5];
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

    // End of SimscapeInputBlock: '<S49>/INPUT_6_1_1'

    // SimscapeInputBlock: '<S49>/INPUT_7_1_1'
    left_arm_simul_B.INPUT_7_1_1[0] = left_arm_simul_B.quantized_torque[6];
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

    // End of SimscapeInputBlock: '<S49>/INPUT_7_1_1'

    // SimscapeExecutionBlock: '<S49>/OUTPUT_1_0'
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
    left_arm_simul_B.dv[0] = left_arm_simul_B.INPUT_8_1_1[0];
    left_arm_simul_B.dv[1] = left_arm_simul_B.INPUT_8_1_1[1];
    left_arm_simul_B.dv[2] = left_arm_simul_B.INPUT_8_1_1[2];
    left_arm_simul_B.dv[3] = left_arm_simul_B.INPUT_8_1_1[3];
    left_arm_simul_B.iv[1] = 4;
    left_arm_simul_B.dv[4] = left_arm_simul_B.INPUT_1_1_1[0];
    left_arm_simul_B.dv[5] = left_arm_simul_B.INPUT_1_1_1[1];
    left_arm_simul_B.dv[6] = left_arm_simul_B.INPUT_1_1_1[2];
    left_arm_simul_B.dv[7] = left_arm_simul_B.INPUT_1_1_1[3];
    left_arm_simul_B.iv[2] = 8;
    left_arm_simul_B.dv[8] = left_arm_simul_B.INPUT_2_1_1[0];
    left_arm_simul_B.dv[9] = left_arm_simul_B.INPUT_2_1_1[1];
    left_arm_simul_B.dv[10] = left_arm_simul_B.INPUT_2_1_1[2];
    left_arm_simul_B.dv[11] = left_arm_simul_B.INPUT_2_1_1[3];
    left_arm_simul_B.iv[3] = 12;
    left_arm_simul_B.dv[12] = left_arm_simul_B.INPUT_3_1_1[0];
    left_arm_simul_B.dv[13] = left_arm_simul_B.INPUT_3_1_1[1];
    left_arm_simul_B.dv[14] = left_arm_simul_B.INPUT_3_1_1[2];
    left_arm_simul_B.dv[15] = left_arm_simul_B.INPUT_3_1_1[3];
    left_arm_simul_B.iv[4] = 16;
    left_arm_simul_B.dv[16] = left_arm_simul_B.INPUT_4_1_1[0];
    left_arm_simul_B.dv[17] = left_arm_simul_B.INPUT_4_1_1[1];
    left_arm_simul_B.dv[18] = left_arm_simul_B.INPUT_4_1_1[2];
    left_arm_simul_B.dv[19] = left_arm_simul_B.INPUT_4_1_1[3];
    left_arm_simul_B.iv[5] = 20;
    left_arm_simul_B.dv[20] = left_arm_simul_B.INPUT_5_1_1[0];
    left_arm_simul_B.dv[21] = left_arm_simul_B.INPUT_5_1_1[1];
    left_arm_simul_B.dv[22] = left_arm_simul_B.INPUT_5_1_1[2];
    left_arm_simul_B.dv[23] = left_arm_simul_B.INPUT_5_1_1[3];
    left_arm_simul_B.iv[6] = 24;
    left_arm_simul_B.dv[24] = left_arm_simul_B.INPUT_6_1_1[0];
    left_arm_simul_B.dv[25] = left_arm_simul_B.INPUT_6_1_1[1];
    left_arm_simul_B.dv[26] = left_arm_simul_B.INPUT_6_1_1[2];
    left_arm_simul_B.dv[27] = left_arm_simul_B.INPUT_6_1_1[3];
    left_arm_simul_B.iv[7] = 28;
    left_arm_simul_B.dv[28] = left_arm_simul_B.INPUT_7_1_1[0];
    left_arm_simul_B.dv[29] = left_arm_simul_B.INPUT_7_1_1[1];
    left_arm_simul_B.dv[30] = left_arm_simul_B.INPUT_7_1_1[2];
    left_arm_simul_B.dv[31] = left_arm_simul_B.INPUT_7_1_1[3];
    left_arm_simul_B.iv[8] = 32;
    memcpy(&left_arm_simul_B.dv[32], &left_arm_simul_B.STATE_1[0], 14U * sizeof
           (real_T));
    left_arm_simul_B.iv[9] = 46;
    simulationData->mData->mInputValues.mN = 46;
    simulationData->mData->mInputValues.mX = &left_arm_simul_B.dv[0];
    simulationData->mData->mInputOffsets.mN = 10;
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
    left_arm_simul_B.i = ne_simulator_method((NeslSimulator *)
      left_arm_simul_DW.OUTPUT_1_0_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diagnosticManager);
    if (left_arm_simul_B.i != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus(left_arm_simul_M));
      if (tmp) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    if (rtmIsMajorTimeStep(left_arm_simul_M) &&
        left_arm_simul_M->Timing.TaskCounters.TID[3] == 0) {
      // MATLABSystem: '<Root>/Get Parameter'
      ParamGet_left_arm_simul_433.get_parameter
        (&left_arm_simul_B.GetParameter_o1);
    }

    if (rtmIsMajorTimeStep(left_arm_simul_M) &&
        left_arm_simul_M->Timing.TaskCounters.TID[1] == 0) {
      // MATLAB Function: '<S1>/Pose Quantization and Noise'
      for (left_arm_simul_B.i = 0; left_arm_simul_B.i < 7; left_arm_simul_B.i++)
      {
        if (left_arm_simul_B.GetParameter_o1 <= MIN_int32_T) {
          left_arm_simul_B.saturatedUnaryMinus = MAX_int32_T;
        } else {
          left_arm_simul_B.saturatedUnaryMinus =
            -left_arm_simul_B.GetParameter_o1;
        }

        left_arm_simul_B.time_tmp = left_arm_simul_rand();
        left_arm_simul_B.quantized_q[left_arm_simul_B.i] = ((floor(((
          static_cast<real_T>(left_arm_simul_B.GetParameter_o1) -
          static_cast<real_T>(left_arm_simul_B.saturatedUnaryMinus)) + 1.0) *
          left_arm_simul_B.time_tmp) + static_cast<real_T>
          (left_arm_simul_B.saturatedUnaryMinus)) + rt_roundd_snf
          (left_arm_simul_B.OUTPUT_1_0[left_arm_simul_B.i] /
           0.0015339807878856412)) * 0.0015339807878856412;
      }

      // End of MATLAB Function: '<S1>/Pose Quantization and Noise'

      // RateTransition: '<S2>/Rate Transition1'
      if (rtmIsMajorTimeStep(left_arm_simul_M) &&
          left_arm_simul_M->Timing.TaskCounters.TID[2] == 0) {
        for (left_arm_simul_B.i = 0; left_arm_simul_B.i < 7; left_arm_simul_B.i
             ++) {
          left_arm_simul_DW.RateTransition1_Buffer[left_arm_simul_B.i] =
            left_arm_simul_B.quantized_q[left_arm_simul_B.i];
        }
      }
    }

    // RateTransition: '<S2>/Rate Transition1'
    if (rtmIsMajorTimeStep(left_arm_simul_M) &&
        left_arm_simul_M->Timing.TaskCounters.TID[2] == 0) {
      // MATLABSystem: '<S2>/Current Time'
      currentROSTimeBus(&left_arm_simul_B.rtb_CurrentTime_c);

      // MATLAB Function: '<S2>/MATLAB Function1' incorporates:
      //   Constant: '<S52>/Constant'
      //   MATLABSystem: '<S2>/Current Time'

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
      for (left_arm_simul_B.i = 0; left_arm_simul_B.i < 10; left_arm_simul_B.i++)
      {
        left_arm_simul_B.js.Name[0].Data[left_arm_simul_B.i] =
          b_0[left_arm_simul_B.i];
        left_arm_simul_B.js.Name[1].Data[left_arm_simul_B.i] =
          c[left_arm_simul_B.i];
        left_arm_simul_B.js.Name[2].Data[left_arm_simul_B.i] =
          d[left_arm_simul_B.i];
        left_arm_simul_B.js.Name[3].Data[left_arm_simul_B.i] =
          e[left_arm_simul_B.i];
        left_arm_simul_B.js.Name[4].Data[left_arm_simul_B.i] =
          f[left_arm_simul_B.i];
        left_arm_simul_B.js.Name[5].Data[left_arm_simul_B.i] =
          g[left_arm_simul_B.i];
        left_arm_simul_B.js.Name[6].Data[left_arm_simul_B.i] =
          h[left_arm_simul_B.i];
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

      // End of MATLAB Function: '<S2>/MATLAB Function1'

      // Outputs for Atomic SubSystem: '<S2>/Publish2'
      // MATLABSystem: '<S55>/SinkBlock'
      Pub_left_arm_simul_191.publish(&left_arm_simul_B.js);

      // End of Outputs for SubSystem: '<S2>/Publish2'
    }

    if (rtmIsMajorTimeStep(left_arm_simul_M) &&
        left_arm_simul_M->Timing.TaskCounters.TID[1] == 0) {
      // BusAssignment: '<S2>/Bus Assignment1' incorporates:
      //   Constant: '<S2>/Constant'
      //   Constant: '<S50>/Constant'

      left_arm_simul_B.BusAssignment1 = left_arm_simul_P.Constant_Value_m;
      for (left_arm_simul_B.i = 0; left_arm_simul_B.i < 7; left_arm_simul_B.i++)
      {
        left_arm_simul_B.BusAssignment1.Data[left_arm_simul_B.i] =
          left_arm_simul_B.quantized_q[left_arm_simul_B.i];
      }

      left_arm_simul_B.BusAssignment1.Data_SL_Info.CurrentLength =
        left_arm_simul_P.Constant_Value_mx;
      left_arm_simul_B.BusAssignment1.Data_SL_Info.ReceivedLength =
        left_arm_simul_P.Constant_Value_mx;

      // End of BusAssignment: '<S2>/Bus Assignment1'

      // Outputs for Atomic SubSystem: '<S2>/Publish1'
      // MATLABSystem: '<S54>/SinkBlock'
      Pub_left_arm_simul_163.publish(&left_arm_simul_B.BusAssignment1);

      // End of Outputs for SubSystem: '<S2>/Publish1'
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

    // Update for SimscapeExecutionBlock: '<S49>/STATE_1'
    simulationData = (NeslSimulationData *)left_arm_simul_DW.STATE_1_SimData;
    time = left_arm_simul_M->Timing.t[0];
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 14;
    simulationData->mData->mContStates.mX =
      &left_arm_simul_X.left_arm_simulManipulatorla_1_j[0];
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
    left_arm_simul_B.dv5[0] = left_arm_simul_B.INPUT_8_1_1[0];
    left_arm_simul_B.dv5[1] = left_arm_simul_B.INPUT_8_1_1[1];
    left_arm_simul_B.dv5[2] = left_arm_simul_B.INPUT_8_1_1[2];
    left_arm_simul_B.dv5[3] = left_arm_simul_B.INPUT_8_1_1[3];
    left_arm_simul_B.iv5[1] = 4;
    left_arm_simul_B.dv5[4] = left_arm_simul_B.INPUT_1_1_1[0];
    left_arm_simul_B.dv5[5] = left_arm_simul_B.INPUT_1_1_1[1];
    left_arm_simul_B.dv5[6] = left_arm_simul_B.INPUT_1_1_1[2];
    left_arm_simul_B.dv5[7] = left_arm_simul_B.INPUT_1_1_1[3];
    left_arm_simul_B.iv5[2] = 8;
    left_arm_simul_B.dv5[8] = left_arm_simul_B.INPUT_2_1_1[0];
    left_arm_simul_B.dv5[9] = left_arm_simul_B.INPUT_2_1_1[1];
    left_arm_simul_B.dv5[10] = left_arm_simul_B.INPUT_2_1_1[2];
    left_arm_simul_B.dv5[11] = left_arm_simul_B.INPUT_2_1_1[3];
    left_arm_simul_B.iv5[3] = 12;
    left_arm_simul_B.dv5[12] = left_arm_simul_B.INPUT_3_1_1[0];
    left_arm_simul_B.dv5[13] = left_arm_simul_B.INPUT_3_1_1[1];
    left_arm_simul_B.dv5[14] = left_arm_simul_B.INPUT_3_1_1[2];
    left_arm_simul_B.dv5[15] = left_arm_simul_B.INPUT_3_1_1[3];
    left_arm_simul_B.iv5[4] = 16;
    left_arm_simul_B.dv5[16] = left_arm_simul_B.INPUT_4_1_1[0];
    left_arm_simul_B.dv5[17] = left_arm_simul_B.INPUT_4_1_1[1];
    left_arm_simul_B.dv5[18] = left_arm_simul_B.INPUT_4_1_1[2];
    left_arm_simul_B.dv5[19] = left_arm_simul_B.INPUT_4_1_1[3];
    left_arm_simul_B.iv5[5] = 20;
    left_arm_simul_B.dv5[20] = left_arm_simul_B.INPUT_5_1_1[0];
    left_arm_simul_B.dv5[21] = left_arm_simul_B.INPUT_5_1_1[1];
    left_arm_simul_B.dv5[22] = left_arm_simul_B.INPUT_5_1_1[2];
    left_arm_simul_B.dv5[23] = left_arm_simul_B.INPUT_5_1_1[3];
    left_arm_simul_B.iv5[6] = 24;
    left_arm_simul_B.dv5[24] = left_arm_simul_B.INPUT_6_1_1[0];
    left_arm_simul_B.dv5[25] = left_arm_simul_B.INPUT_6_1_1[1];
    left_arm_simul_B.dv5[26] = left_arm_simul_B.INPUT_6_1_1[2];
    left_arm_simul_B.dv5[27] = left_arm_simul_B.INPUT_6_1_1[3];
    left_arm_simul_B.iv5[7] = 28;
    left_arm_simul_B.dv5[28] = left_arm_simul_B.INPUT_7_1_1[0];
    left_arm_simul_B.dv5[29] = left_arm_simul_B.INPUT_7_1_1[1];
    left_arm_simul_B.dv5[30] = left_arm_simul_B.INPUT_7_1_1[2];
    left_arm_simul_B.dv5[31] = left_arm_simul_B.INPUT_7_1_1[3];
    left_arm_simul_B.iv5[8] = 32;
    simulationData->mData->mInputValues.mN = 32;
    simulationData->mData->mInputValues.mX = &left_arm_simul_B.dv5[0];
    simulationData->mData->mInputOffsets.mN = 9;
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

    // End of Update for SimscapeExecutionBlock: '<S49>/STATE_1'
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
      // Update absolute timer for sample time: [0.004s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.004, which is the step size
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

  // Derivatives for SimscapeExecutionBlock: '<S49>/STATE_1'
  simulationData = (NeslSimulationData *)left_arm_simul_DW.STATE_1_SimData;
  time = left_arm_simul_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 14;
  simulationData->mData->mContStates.mX =
    &left_arm_simul_X.left_arm_simulManipulatorla_1_j[0];
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
  left_arm_simul_B.dv4[0] = left_arm_simul_B.INPUT_8_1_1[0];
  left_arm_simul_B.dv4[1] = left_arm_simul_B.INPUT_8_1_1[1];
  left_arm_simul_B.dv4[2] = left_arm_simul_B.INPUT_8_1_1[2];
  left_arm_simul_B.dv4[3] = left_arm_simul_B.INPUT_8_1_1[3];
  left_arm_simul_B.iv4[1] = 4;
  left_arm_simul_B.dv4[4] = left_arm_simul_B.INPUT_1_1_1[0];
  left_arm_simul_B.dv4[5] = left_arm_simul_B.INPUT_1_1_1[1];
  left_arm_simul_B.dv4[6] = left_arm_simul_B.INPUT_1_1_1[2];
  left_arm_simul_B.dv4[7] = left_arm_simul_B.INPUT_1_1_1[3];
  left_arm_simul_B.iv4[2] = 8;
  left_arm_simul_B.dv4[8] = left_arm_simul_B.INPUT_2_1_1[0];
  left_arm_simul_B.dv4[9] = left_arm_simul_B.INPUT_2_1_1[1];
  left_arm_simul_B.dv4[10] = left_arm_simul_B.INPUT_2_1_1[2];
  left_arm_simul_B.dv4[11] = left_arm_simul_B.INPUT_2_1_1[3];
  left_arm_simul_B.iv4[3] = 12;
  left_arm_simul_B.dv4[12] = left_arm_simul_B.INPUT_3_1_1[0];
  left_arm_simul_B.dv4[13] = left_arm_simul_B.INPUT_3_1_1[1];
  left_arm_simul_B.dv4[14] = left_arm_simul_B.INPUT_3_1_1[2];
  left_arm_simul_B.dv4[15] = left_arm_simul_B.INPUT_3_1_1[3];
  left_arm_simul_B.iv4[4] = 16;
  left_arm_simul_B.dv4[16] = left_arm_simul_B.INPUT_4_1_1[0];
  left_arm_simul_B.dv4[17] = left_arm_simul_B.INPUT_4_1_1[1];
  left_arm_simul_B.dv4[18] = left_arm_simul_B.INPUT_4_1_1[2];
  left_arm_simul_B.dv4[19] = left_arm_simul_B.INPUT_4_1_1[3];
  left_arm_simul_B.iv4[5] = 20;
  left_arm_simul_B.dv4[20] = left_arm_simul_B.INPUT_5_1_1[0];
  left_arm_simul_B.dv4[21] = left_arm_simul_B.INPUT_5_1_1[1];
  left_arm_simul_B.dv4[22] = left_arm_simul_B.INPUT_5_1_1[2];
  left_arm_simul_B.dv4[23] = left_arm_simul_B.INPUT_5_1_1[3];
  left_arm_simul_B.iv4[6] = 24;
  left_arm_simul_B.dv4[24] = left_arm_simul_B.INPUT_6_1_1[0];
  left_arm_simul_B.dv4[25] = left_arm_simul_B.INPUT_6_1_1[1];
  left_arm_simul_B.dv4[26] = left_arm_simul_B.INPUT_6_1_1[2];
  left_arm_simul_B.dv4[27] = left_arm_simul_B.INPUT_6_1_1[3];
  left_arm_simul_B.iv4[7] = 28;
  left_arm_simul_B.dv4[28] = left_arm_simul_B.INPUT_7_1_1[0];
  left_arm_simul_B.dv4[29] = left_arm_simul_B.INPUT_7_1_1[1];
  left_arm_simul_B.dv4[30] = left_arm_simul_B.INPUT_7_1_1[2];
  left_arm_simul_B.dv4[31] = left_arm_simul_B.INPUT_7_1_1[3];
  left_arm_simul_B.iv4[8] = 32;
  simulationData->mData->mInputValues.mN = 32;
  simulationData->mData->mInputValues.mX = &left_arm_simul_B.dv4[0];
  simulationData->mData->mInputOffsets.mN = 9;
  simulationData->mData->mInputOffsets.mX = &left_arm_simul_B.iv4[0];
  simulationData->mData->mDx.mN = 14;
  simulationData->mData->mDx.mX = &_rtXdot->left_arm_simulManipulatorla_1_j[0];
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

  // End of Derivatives for SimscapeExecutionBlock: '<S49>/STATE_1'
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
  left_arm_simul_M->intgData.f[4] = left_arm_simul_M->odeF[4];
  left_arm_simul_M->intgData.f[5] = left_arm_simul_M->odeF[5];
  left_arm_simul_M->contStates = ((X_left_arm_simul_T *) &left_arm_simul_X);
  rtsiSetSolverData(&left_arm_simul_M->solverInfo, static_cast<void *>
                    (&left_arm_simul_M->intgData));
  rtsiSetSolverName(&left_arm_simul_M->solverInfo,"ode5");
  rtmSetTPtr(left_arm_simul_M, &left_arm_simul_M->Timing.tArray[0]);
  left_arm_simul_M->Timing.stepSize0 = 0.004;

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
    static const char_T tmp_2[21] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'o', 'b', 'j', '_', 'm', 'a', 's', 's' };

    static const char_T tmp_3[19] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 't', 'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_4[25] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'n', 'o', 'i', 's', 'e', '_', 'f', 'a', 'c', 't',
      'o', 'r' };

    static const char_T tmp_5[13] = { '/', 'j', 'o', 'i', 'n', 't', '_', 's',
      't', 'a', 't', 'e', 's' };

    static const char_T tmp_6[25] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'c', 'u', 'r', 'r', 'e', 'n', 't', '_', 'p', 'o',
      's', 'e' };

    // Start for SimscapeExecutionBlock: '<S49>/STATE_1' incorporates:
    //   SimscapeExecutionBlock: '<S49>/OUTPUT_1_0'

    tmp = nesl_lease_simulator(
      "left_arm_simul/Manipulator/Solver Configuration_1", 0, 0);
    left_arm_simul_DW.STATE_1_Simulator = (void *)tmp;
    tmp_0 = pointer_is_null(left_arm_simul_DW.STATE_1_Simulator);
    if (tmp_0) {
      left_arm_simul_5aaa3abb_1_gateway();
      tmp = nesl_lease_simulator(
        "left_arm_simul/Manipulator/Solver Configuration_1", 0, 0);
      left_arm_simul_DW.STATE_1_Simulator = (void *)tmp;
    }

    simulationData = nesl_create_simulation_data();
    left_arm_simul_DW.STATE_1_SimData = (void *)simulationData;
    diagnosticManager = rtw_create_diagnostics();
    left_arm_simul_DW.STATE_1_DiagMgr = (void *)diagnosticManager;
    left_arm_simul_B.modelParameters.mSolverType = NE_SOLVER_TYPE_ODE;
    left_arm_simul_B.modelParameters.mSolverTolerance = 0.001;
    left_arm_simul_B.modelParameters.mVariableStepSolver = false;
    left_arm_simul_B.modelParameters.mFixedStepSize = 0.004;
    left_arm_simul_B.modelParameters.mStartTime = 0.0;
    left_arm_simul_B.modelParameters.mLoadInitialState = false;
    left_arm_simul_B.modelParameters.mUseSimState = false;
    left_arm_simul_B.modelParameters.mLinTrimCompile = false;
    left_arm_simul_B.modelParameters.mLoggingMode = SSC_LOGGING_NONE;
    left_arm_simul_B.modelParameters.mRTWModifiedTimeStamp = 5.17265059E+8;
    left_arm_simul_B.d = 0.001;
    left_arm_simul_B.modelParameters.mSolverTolerance = left_arm_simul_B.d;
    left_arm_simul_B.d = 0.004;
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
      &left_arm_simul_X.left_arm_simulManipulatorla_1_j[0];
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

    // End of Start for SimscapeExecutionBlock: '<S49>/STATE_1'

    // Start for Atomic SubSystem: '<Root>/Subscribe1'
    // Start for MATLABSystem: '<S4>/SourceBlock'
    left_arm_simul_DW.obj_k.matlabCodegenIsDeleted = true;
    left_arm_simul_DW.obj_k.isInitialized = 0;
    left_arm_simul_DW.obj_k.matlabCodegenIsDeleted = false;
    left_arm_simul_DW.obj_k.isSetupComplete = false;
    left_arm_simul_DW.obj_k.isInitialized = 1;
    for (i = 0; i < 21; i++) {
      left_arm_simul_B.cv1[i] = tmp_2[i];
    }

    left_arm_simul_B.cv1[21] = '\x00';
    Sub_left_arm_simul_272.createSubscriber(left_arm_simul_B.cv1, 1);
    left_arm_simul_DW.obj_k.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe1'

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S3>/SourceBlock'
    left_arm_simul_DW.obj_f.matlabCodegenIsDeleted = true;
    left_arm_simul_DW.obj_f.isInitialized = 0;
    left_arm_simul_DW.obj_f.matlabCodegenIsDeleted = false;
    left_arm_simul_DW.obj_f.isSetupComplete = false;
    left_arm_simul_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      left_arm_simul_B.cv2[i] = tmp_3[i];
    }

    left_arm_simul_B.cv2[19] = '\x00';
    Sub_left_arm_simul_160.createSubscriber(left_arm_simul_B.cv2, 1);
    left_arm_simul_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for SimscapeExecutionBlock: '<S49>/OUTPUT_1_0'
    tmp = nesl_lease_simulator(
      "left_arm_simul/Manipulator/Solver Configuration_1", 1, 0);
    left_arm_simul_DW.OUTPUT_1_0_Simulator = (void *)tmp;
    tmp_1 = pointer_is_null(left_arm_simul_DW.OUTPUT_1_0_Simulator);
    if (tmp_1) {
      left_arm_simul_5aaa3abb_1_gateway();
      tmp = nesl_lease_simulator(
        "left_arm_simul/Manipulator/Solver Configuration_1", 1, 0);
      left_arm_simul_DW.OUTPUT_1_0_Simulator = (void *)tmp;
    }

    simulationData = nesl_create_simulation_data();
    left_arm_simul_DW.OUTPUT_1_0_SimData = (void *)simulationData;
    diagnosticManager = rtw_create_diagnostics();
    left_arm_simul_DW.OUTPUT_1_0_DiagMgr = (void *)diagnosticManager;
    left_arm_simul_B.modelParameters_m.mSolverType = NE_SOLVER_TYPE_ODE;
    left_arm_simul_B.modelParameters_m.mSolverTolerance = 0.001;
    left_arm_simul_B.modelParameters_m.mVariableStepSolver = false;
    left_arm_simul_B.modelParameters_m.mFixedStepSize = 0.004;
    left_arm_simul_B.modelParameters_m.mStartTime = 0.0;
    left_arm_simul_B.modelParameters_m.mLoadInitialState = false;
    left_arm_simul_B.modelParameters_m.mUseSimState = false;
    left_arm_simul_B.modelParameters_m.mLinTrimCompile = false;
    left_arm_simul_B.modelParameters_m.mLoggingMode = SSC_LOGGING_NONE;
    left_arm_simul_B.modelParameters_m.mRTWModifiedTimeStamp = 5.17265059E+8;
    left_arm_simul_B.d = 0.001;
    left_arm_simul_B.modelParameters_m.mSolverTolerance = left_arm_simul_B.d;
    left_arm_simul_B.d = 0.004;
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

    // Start for MATLABSystem: '<Root>/Get Parameter'
    left_arm_simul_DW.obj.matlabCodegenIsDeleted = true;
    left_arm_simul_DW.obj.isInitialized = 0;
    left_arm_simul_DW.obj.matlabCodegenIsDeleted = false;
    left_arm_simul_DW.obj.isSetupComplete = false;
    left_arm_simul_DW.obj.isInitialized = 1;
    for (i = 0; i < 25; i++) {
      left_arm_simul_B.cv[i] = tmp_4[i];
    }

    left_arm_simul_B.cv[25] = '\x00';
    ParamGet_left_arm_simul_433.initialize(left_arm_simul_B.cv);
    ParamGet_left_arm_simul_433.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_simul_433.set_initial_value(0);
    left_arm_simul_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<Root>/Get Parameter'

    // Start for MATLABSystem: '<S2>/Current Time'
    left_arm_simul_DW.obj_b.matlabCodegenIsDeleted = true;
    left_arm_simul_DW.obj_b.isInitialized = 0;
    left_arm_simul_DW.obj_b.matlabCodegenIsDeleted = false;
    left_arm_simul_DW.obj_b.isSetupComplete = false;
    left_arm_simul_DW.obj_b.isInitialized = 1;
    left_arm_simul_DW.obj_b.isSetupComplete = true;

    // Start for Atomic SubSystem: '<S2>/Publish2'
    // Start for MATLABSystem: '<S55>/SinkBlock'
    left_arm_simul_DW.obj_a.matlabCodegenIsDeleted = true;
    left_arm_simul_DW.obj_a.isInitialized = 0;
    left_arm_simul_DW.obj_a.matlabCodegenIsDeleted = false;
    left_arm_simul_DW.obj_a.isSetupComplete = false;
    left_arm_simul_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      left_arm_simul_B.cv3[i] = tmp_5[i];
    }

    left_arm_simul_B.cv3[13] = '\x00';
    Pub_left_arm_simul_191.createPublisher(left_arm_simul_B.cv3, 1);
    left_arm_simul_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S55>/SinkBlock'
    // End of Start for SubSystem: '<S2>/Publish2'

    // Start for Atomic SubSystem: '<S2>/Publish1'
    // Start for MATLABSystem: '<S54>/SinkBlock'
    left_arm_simul_DW.obj_ag.matlabCodegenIsDeleted = true;
    left_arm_simul_DW.obj_ag.isInitialized = 0;
    left_arm_simul_DW.obj_ag.matlabCodegenIsDeleted = false;
    left_arm_simul_DW.obj_ag.isSetupComplete = false;
    left_arm_simul_DW.obj_ag.isInitialized = 1;
    for (i = 0; i < 25; i++) {
      left_arm_simul_B.cv[i] = tmp_6[i];
    }

    left_arm_simul_B.cv[25] = '\x00';
    Pub_left_arm_simul_163.createPublisher(left_arm_simul_B.cv, 1);
    left_arm_simul_DW.obj_ag.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S54>/SinkBlock'
    // End of Start for SubSystem: '<S2>/Publish1'

    // InitializeConditions for SimscapeExecutionBlock: '<S49>/STATE_1'
    tmp_0 = false;
    if (tmp_0) {
      i = strcmp("ode5", rtsiGetSolverName(&left_arm_simul_M->solverInfo));
      if (i != 0) {
        msg = solver_mismatch_message("ode5", rtsiGetSolverName
          (&left_arm_simul_M->solverInfo));
        rtmSetErrorStatus(left_arm_simul_M, msg);
      }
    }

    // End of InitializeConditions for SimscapeExecutionBlock: '<S49>/STATE_1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S4>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S57>/Out1'
    left_arm_simul_B.In1_j = left_arm_simul_P.Out1_Y0_h;

    // End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S3>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S56>/Out1'
    left_arm_simul_B.In1 = left_arm_simul_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S3>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for MATLAB Function: '<S1>/Pose Quantization and Noise'
    left_arm_simul_DW.method = 7U;
    left_arm_simul_DW.state = 1144108930U;
    left_arm_simul_DW.state_b[0] = 362436069U;
    left_arm_simul_DW.state_b[1] = 521288629U;
  }
}

// Model terminate function
void left_arm_simul_terminate(void)
{
  // Terminate for SimscapeExecutionBlock: '<S49>/STATE_1'
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    left_arm_simul_DW.STATE_1_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    left_arm_simul_DW.STATE_1_SimData);
  nesl_erase_simulator("left_arm_simul/Manipulator/Solver Configuration_1");

  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S4>/SourceBlock'
  matlabCodegenHandle_matlab_k10o(&left_arm_simul_DW.obj_k);

  // End of Terminate for SubSystem: '<Root>/Subscribe1'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S3>/SourceBlock'
  matlabCodegenHandle_matlab_k10o(&left_arm_simul_DW.obj_f);

  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for SimscapeExecutionBlock: '<S49>/OUTPUT_1_0'
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    left_arm_simul_DW.OUTPUT_1_0_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    left_arm_simul_DW.OUTPUT_1_0_SimData);
  nesl_erase_simulator("left_arm_simul/Manipulator/Solver Configuration_1");

  // Terminate for MATLABSystem: '<Root>/Get Parameter'
  matlabCodegenHandle_matlabCodeg(&left_arm_simul_DW.obj);

  // Terminate for MATLABSystem: '<S2>/Current Time'
  matlabCodegenHandle_matlabCod_k(&left_arm_simul_DW.obj_b);

  // Terminate for Atomic SubSystem: '<S2>/Publish2'
  // Terminate for MATLABSystem: '<S55>/SinkBlock'
  matlabCodegenHandle_matlabCo_k1(&left_arm_simul_DW.obj_a);

  // End of Terminate for SubSystem: '<S2>/Publish2'

  // Terminate for Atomic SubSystem: '<S2>/Publish1'
  // Terminate for MATLABSystem: '<S54>/SinkBlock'
  matlabCodegenHandle_matlabCo_k1(&left_arm_simul_DW.obj_ag);

  // End of Terminate for SubSystem: '<S2>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
