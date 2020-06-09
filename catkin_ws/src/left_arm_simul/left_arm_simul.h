//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_simul.h
//
// Code generated for Simulink model 'left_arm_simul'.
//
// Model version                  : 1.90
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Mon Jun  8 23:26:45 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_left_arm_simul_h_
#define RTW_HEADER_left_arm_simul_h_
#include <string.h>
#include <stddef.h>
#ifndef left_arm_simul_COMMON_INCLUDES_
# define left_arm_simul_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "nesl_rtw.h"
#include "left_arm_simul_e2442d81_1_gateway.h"
#endif                                 // left_arm_simul_COMMON_INCLUDES_

#include "left_arm_simul_types.h"

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
  SL_Bus_left_arm_simul_std_msgs_Float64MultiArray In1;// '<S49>/In1'
  SL_Bus_left_arm_simul_std_msgs_Float64MultiArray BusAssignment1;// '<Root>/Bus Assignment1' 
  SL_Bus_left_arm_simul_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_left_arm_simul_std_msgs_MultiArrayDimension b_varargout_2_Layout_Dim[16];
  SL_Bus_left_arm_simul_sensor_msgs_JointState js;// '<Root>/MATLAB Function1'
  real_T dv[42];
  real_T dv1[28];
  real_T dv2[28];
  real_T dv3[28];
  real_T dv4[28];
  real_T dv5[28];
  real_T STATE_1[14];                  // '<S48>/STATE_1'
  real_T b_varargout_2_Data[7];
  NeParameterBundle expl_temp;
  NeModelParameters modelParameters;
  NeModelParameters modelParameters_m;
  int_T iv[9];
  int_T iv1[8];
  int_T iv2[8];
  int_T iv3[8];
  int_T iv4[8];
  int_T iv5[8];
  real_T INPUT_1_1_1[4];               // '<S48>/INPUT_1_1_1'
  real_T INPUT_2_1_1[4];               // '<S48>/INPUT_2_1_1'
  real_T INPUT_3_1_1[4];               // '<S48>/INPUT_3_1_1'
  real_T INPUT_4_1_1[4];               // '<S48>/INPUT_4_1_1'
  real_T INPUT_5_1_1[4];               // '<S48>/INPUT_5_1_1'
  real_T INPUT_6_1_1[4];               // '<S48>/INPUT_6_1_1'
  real_T INPUT_7_1_1[4];               // '<S48>/INPUT_7_1_1'
  real_T OUTPUT_1_0[7];                // '<S48>/OUTPUT_1_0'
  char_T cv[26];
  char_T cv1[20];
  SL_Bus_left_arm_simul_ros_time_Time rtb_CurrentTime_c;
  char_T cv2[14];
  real_T time;
  real_T time_k;
  real_T time_tmp;
  real_T d;
  real_T time_c;
} B_left_arm_simul_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_Curr_T obj; // '<Root>/Current Time'
  ros_slros_internal_block_Publ_T obj_a;// '<S13>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_ag;// '<S12>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_f;// '<S22>/SourceBlock'
  real_T INPUT_1_1_1_Discrete[2];      // '<S48>/INPUT_1_1_1'
  real_T INPUT_2_1_1_Discrete[2];      // '<S48>/INPUT_2_1_1'
  real_T INPUT_3_1_1_Discrete[2];      // '<S48>/INPUT_3_1_1'
  real_T INPUT_4_1_1_Discrete[2];      // '<S48>/INPUT_4_1_1'
  real_T INPUT_5_1_1_Discrete[2];      // '<S48>/INPUT_5_1_1'
  real_T INPUT_6_1_1_Discrete[2];      // '<S48>/INPUT_6_1_1'
  real_T INPUT_7_1_1_Discrete[2];      // '<S48>/INPUT_7_1_1'
  real_T STATE_1_Discrete;             // '<S48>/STATE_1'
  real_T OUTPUT_1_0_Discrete;          // '<S48>/OUTPUT_1_0'
  real_T RateTransition1_Buffer[7];    // '<Root>/Rate Transition1'
  void* STATE_1_Simulator;             // '<S48>/STATE_1'
  void* STATE_1_SimData;               // '<S48>/STATE_1'
  void* STATE_1_DiagMgr;               // '<S48>/STATE_1'
  void* STATE_1_ZcLogger;              // '<S48>/STATE_1'
  void* STATE_1_TsIndex;               // '<S48>/STATE_1'
  void* OUTPUT_1_0_Simulator;          // '<S48>/OUTPUT_1_0'
  void* OUTPUT_1_0_SimData;            // '<S48>/OUTPUT_1_0'
  void* OUTPUT_1_0_DiagMgr;            // '<S48>/OUTPUT_1_0'
  void* OUTPUT_1_0_ZcLogger;           // '<S48>/OUTPUT_1_0'
  void* OUTPUT_1_0_TsIndex;            // '<S48>/OUTPUT_1_0'
  int_T STATE_1_Modes;                 // '<S48>/STATE_1'
  int_T OUTPUT_1_0_Modes;              // '<S48>/OUTPUT_1_0'
  boolean_T STATE_1_FirstOutput;       // '<S48>/STATE_1'
  boolean_T OUTPUT_1_0_FirstOutput;    // '<S48>/OUTPUT_1_0'
} DW_left_arm_simul_T;

// Continuous states (default storage)
typedef struct {
  real_T left_arm_simulla_1_jointRzq[14];// '<S48>/STATE_1'
} X_left_arm_simul_T;

// State derivatives (default storage)
typedef struct {
  real_T left_arm_simulla_1_jointRzq[14];// '<S48>/STATE_1'
} XDot_left_arm_simul_T;

// State disabled
typedef struct {
  boolean_T left_arm_simulla_1_jointRzq[14];// '<S48>/STATE_1'
} XDis_left_arm_simul_T;

#ifndef ODE5_INTG
#define ODE5_INTG

// ODE5 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[6];                        // derivatives
} ODE5_IntgData;

#endif

// Parameters (default storage)
struct P_left_arm_simul_T_ {
  SL_Bus_left_arm_simul_std_msgs_Float64MultiArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                              //  Referenced by: '<S49>/Out1'

  SL_Bus_left_arm_simul_std_msgs_Float64MultiArray Constant_Value;// Computed Parameter: Constant_Value
                                                                     //  Referenced by: '<S22>/Constant'

  SL_Bus_left_arm_simul_std_msgs_Float64MultiArray Constant_Value_m;// Computed Parameter: Constant_Value_m
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_left_arm_simul_sensor_msgs_JointState Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                                   //  Referenced by: '<S3>/Constant'

  SL_Bus_left_arm_simul_std_msgs_String Constant_Value_bz;// Computed Parameter: Constant_Value_bz
                                                             //  Referenced by: '<S2>/Constant'

  uint32_T Constant_Value_mx;          // Computed Parameter: Constant_Value_mx
                                          //  Referenced by: '<Root>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_left_arm_simul_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_left_arm_simul_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[14];
  real_T odeF[6][14];
  ODE5_IntgData intgData;

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
    struct {
      uint8_T TID[3];
    } TaskCounters;

    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_left_arm_simul_T left_arm_simul_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_left_arm_simul_T left_arm_simul_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_left_arm_simul_T left_arm_simul_X;

// Block states (default storage)
extern DW_left_arm_simul_T left_arm_simul_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void left_arm_simul_initialize(void);
  extern void left_arm_simul_step(void);
  extern void left_arm_simul_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_left_arm_simul_T *const left_arm_simul_M;

#ifdef __cplusplus

}
#endif

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
//  '<Root>' : 'left_arm_simul'
//  '<S1>'   : 'left_arm_simul/Blank Message1'
//  '<S2>'   : 'left_arm_simul/Blank Message2'
//  '<S3>'   : 'left_arm_simul/Blank Message3'
//  '<S4>'   : 'left_arm_simul/MATLAB Function1'
//  '<S5>'   : 'left_arm_simul/PS-Simulink Converter'
//  '<S6>'   : 'left_arm_simul/PS-Simulink Converter1'
//  '<S7>'   : 'left_arm_simul/PS-Simulink Converter2'
//  '<S8>'   : 'left_arm_simul/PS-Simulink Converter3'
//  '<S9>'   : 'left_arm_simul/PS-Simulink Converter4'
//  '<S10>'  : 'left_arm_simul/PS-Simulink Converter5'
//  '<S11>'  : 'left_arm_simul/PS-Simulink Converter6'
//  '<S12>'  : 'left_arm_simul/Publish1'
//  '<S13>'  : 'left_arm_simul/Publish2'
//  '<S14>'  : 'left_arm_simul/Simulink-PS Converter'
//  '<S15>'  : 'left_arm_simul/Simulink-PS Converter1'
//  '<S16>'  : 'left_arm_simul/Simulink-PS Converter2'
//  '<S17>'  : 'left_arm_simul/Simulink-PS Converter3'
//  '<S18>'  : 'left_arm_simul/Simulink-PS Converter4'
//  '<S19>'  : 'left_arm_simul/Simulink-PS Converter5'
//  '<S20>'  : 'left_arm_simul/Simulink-PS Converter6'
//  '<S21>'  : 'left_arm_simul/Solver Configuration'
//  '<S22>'  : 'left_arm_simul/Subscribe'
//  '<S23>'  : 'left_arm_simul/left_arm_grip_center'
//  '<S24>'  : 'left_arm_simul/left_arm_grip_left'
//  '<S25>'  : 'left_arm_simul/left_arm_grip_right'
//  '<S26>'  : 'left_arm_simul/left_arm_link1'
//  '<S27>'  : 'left_arm_simul/left_arm_link2'
//  '<S28>'  : 'left_arm_simul/left_arm_link3'
//  '<S29>'  : 'left_arm_simul/left_arm_link4'
//  '<S30>'  : 'left_arm_simul/left_arm_link5'
//  '<S31>'  : 'left_arm_simul/left_arm_link6'
//  '<S32>'  : 'left_arm_simul/left_arm_link7'
//  '<S33>'  : 'left_arm_simul/shoulders_left_link'
//  '<S34>'  : 'left_arm_simul/PS-Simulink Converter/EVAL_KEY'
//  '<S35>'  : 'left_arm_simul/PS-Simulink Converter1/EVAL_KEY'
//  '<S36>'  : 'left_arm_simul/PS-Simulink Converter2/EVAL_KEY'
//  '<S37>'  : 'left_arm_simul/PS-Simulink Converter3/EVAL_KEY'
//  '<S38>'  : 'left_arm_simul/PS-Simulink Converter4/EVAL_KEY'
//  '<S39>'  : 'left_arm_simul/PS-Simulink Converter5/EVAL_KEY'
//  '<S40>'  : 'left_arm_simul/PS-Simulink Converter6/EVAL_KEY'
//  '<S41>'  : 'left_arm_simul/Simulink-PS Converter/EVAL_KEY'
//  '<S42>'  : 'left_arm_simul/Simulink-PS Converter1/EVAL_KEY'
//  '<S43>'  : 'left_arm_simul/Simulink-PS Converter2/EVAL_KEY'
//  '<S44>'  : 'left_arm_simul/Simulink-PS Converter3/EVAL_KEY'
//  '<S45>'  : 'left_arm_simul/Simulink-PS Converter4/EVAL_KEY'
//  '<S46>'  : 'left_arm_simul/Simulink-PS Converter5/EVAL_KEY'
//  '<S47>'  : 'left_arm_simul/Simulink-PS Converter6/EVAL_KEY'
//  '<S48>'  : 'left_arm_simul/Solver Configuration/EVAL_KEY'
//  '<S49>'  : 'left_arm_simul/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_left_arm_simul_h_

//
// File trailer for generated code.
//
// [EOF]
//
