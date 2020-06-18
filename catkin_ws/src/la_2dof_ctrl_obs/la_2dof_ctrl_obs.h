//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: la_2dof_ctrl_obs.h
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
#ifndef RTW_HEADER_la_2dof_ctrl_obs_h_
#define RTW_HEADER_la_2dof_ctrl_obs_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef la_2dof_ctrl_obs_COMMON_INCLUDES_
# define la_2dof_ctrl_obs_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#endif                                 // la_2dof_ctrl_obs_COMMON_INCLUDES_

#include "la_2dof_ctrl_obs_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

// Block signals (default storage)
typedef struct {
  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray In1;// '<S11>/In1'
  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray In1_d;// '<S10>/In1'
  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray BusAssignment1;// '<Root>/Bus Assignment1' 
  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray b_varargout_2_m;
  SL_Bus_la_2dof_ctrl_obs_std_msgs_MultiArrayDimension b_varargout_2_Layout_Dim
    [16];
  real_T xp_est[6];                    // '<Root>/Observer'
  real_T A[4];
  char_T cv[28];
  char_T cv1[26];
  real_T G[2];
  real_T torque[2];
  real_T m12;
  real_T z21;
  real_T z22;
  real_T a21;
} B_la_2dof_ctrl_obs_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_Publ_T obj; // '<S6>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_d;// '<S5>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_e;// '<S8>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_a;// '<S7>/SourceBlock'
} DW_la_2dof_ctrl_obs_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator_CSTATE[4];         // '<Root>/Integrator'
  real_T LowPassz1_CSTATE[5];          // '<Root>/Low Pass (z1)'
  real_T LowPassz2_CSTATE[5];          // '<Root>/Low Pass (z2)'
} X_la_2dof_ctrl_obs_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator_CSTATE[4];         // '<Root>/Integrator'
  real_T LowPassz1_CSTATE[5];          // '<Root>/Low Pass (z1)'
  real_T LowPassz2_CSTATE[5];          // '<Root>/Low Pass (z2)'
} XDot_la_2dof_ctrl_obs_T;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE[4];      // '<Root>/Integrator'
  boolean_T LowPassz1_CSTATE[5];       // '<Root>/Low Pass (z1)'
  boolean_T LowPassz2_CSTATE[5];       // '<Root>/Low Pass (z2)'
} XDis_la_2dof_ctrl_obs_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_la_2dof_ctrl_obs_T_ {
  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                                //  Referenced by: '<S10>/Out1'

  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S7>/Constant'

  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray Out1_Y0_o;// Computed Parameter: Out1_Y0_o
                                                                  //  Referenced by: '<S11>/Out1'

  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                                      //  Referenced by: '<S8>/Constant'

  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_j;// Computed Parameter: Constant_Value_j
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64 Constant_Value_e;// Computed Parameter: Constant_Value_e
                                                               //  Referenced by: '<S2>/Constant'

  real_T Integrator_IC[4];             // Expression: [0 0 0 0]
                                          //  Referenced by: '<Root>/Integrator'

  real_T LowPassz1_A[5];               // Computed Parameter: LowPassz1_A
                                          //  Referenced by: '<Root>/Low Pass (z1)'

  real_T LowPassz1_C[5];               // Computed Parameter: LowPassz1_C
                                          //  Referenced by: '<Root>/Low Pass (z1)'

  real_T LowPassz2_A[5];               // Computed Parameter: LowPassz2_A
                                          //  Referenced by: '<Root>/Low Pass (z2)'

  real_T LowPassz2_C[5];               // Computed Parameter: LowPassz2_C
                                          //  Referenced by: '<Root>/Low Pass (z2)'

  uint32_T Constant_Value_l;           // Computed Parameter: Constant_Value_l
                                          //  Referenced by: '<Root>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_la_2dof_ctrl_obs_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_la_2dof_ctrl_obs_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[14];
  real_T odeF[3][14];
  ODE3_IntgData intgData;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_la_2dof_ctrl_obs_T la_2dof_ctrl_obs_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_la_2dof_ctrl_obs_T la_2dof_ctrl_obs_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_la_2dof_ctrl_obs_T la_2dof_ctrl_obs_X;

// Block states (default storage)
extern DW_la_2dof_ctrl_obs_T la_2dof_ctrl_obs_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void la_2dof_ctrl_obs_initialize(void);
  extern void la_2dof_ctrl_obs_step(void);
  extern void la_2dof_ctrl_obs_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_la_2dof_ctrl_obs_T *const la_2dof_ctrl_obs_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<Root>/Rate Transition2' : Eliminated since input and output rates are identical


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'la_2dof_ctrl_obs'
//  '<S1>'   : 'la_2dof_ctrl_obs/Blank Message1'
//  '<S2>'   : 'la_2dof_ctrl_obs/Blank Message2'
//  '<S3>'   : 'la_2dof_ctrl_obs/MATLAB Function1'
//  '<S4>'   : 'la_2dof_ctrl_obs/Observer'
//  '<S5>'   : 'la_2dof_ctrl_obs/Publish1'
//  '<S6>'   : 'la_2dof_ctrl_obs/Publish2'
//  '<S7>'   : 'la_2dof_ctrl_obs/Subscribe'
//  '<S8>'   : 'la_2dof_ctrl_obs/Subscribe1'
//  '<S9>'   : 'la_2dof_ctrl_obs/mass estimator'
//  '<S10>'  : 'la_2dof_ctrl_obs/Subscribe/Enabled Subsystem'
//  '<S11>'  : 'la_2dof_ctrl_obs/Subscribe1/Enabled Subsystem'

#endif                                 // RTW_HEADER_la_2dof_ctrl_obs_h_

//
// File trailer for generated code.
//
// [EOF]
//
