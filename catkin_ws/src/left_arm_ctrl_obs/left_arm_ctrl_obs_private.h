//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_ctrl_obs_private.h
//
// Code generated for Simulink model 'left_arm_ctrl_obs'.
//
// Model version                  : 1.257
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Tue Jul 28 11:17:49 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_left_arm_ctrl_obs_private_h_
#define RTW_HEADER_left_arm_ctrl_obs_private_h_
#include "rtwtypes.h"
#include "left_arm_ctrl_obs.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void left_arm_ctr_MATLABSystem_Start(B_MATLABSystem_left_arm_ctrl__T
  *localB, DW_MATLABSystem_left_arm_ctrl_T *localDW);
extern void left_arm_ctrl_obs_MATLABSystem(const real_T rtu_0[7], const real_T
  rtu_1[7], const real_T rtu_2[7], const real_T rtu_3[60],
  B_MATLABSystem_left_arm_ctrl__T *localB, DW_MATLABSystem_left_arm_ctrl_T
  *localDW);
extern void left_arm_ctrl_MATLABSystem_Term(DW_MATLABSystem_left_arm_ctrl_T
  *localDW);

// private model entry point functions
extern void left_arm_ctrl_obs_derivatives(void);

#endif                               // RTW_HEADER_left_arm_ctrl_obs_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
