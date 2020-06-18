//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: la_2dof_simul.h
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
#ifndef RTW_HEADER_la_2dof_simul_h_
#define RTW_HEADER_la_2dof_simul_h_
#include <string.h>
#include <stddef.h>
#ifndef la_2dof_simul_COMMON_INCLUDES_
# define la_2dof_simul_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "nesl_rtw.h"
#include "la_2dof_simul_e38647b6_1_gateway.h"
#endif                                 // la_2dof_simul_COMMON_INCLUDES_

#include "la_2dof_simul_types.h"

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
  SL_Bus_la_2dof_simul_std_msgs_Float64MultiArray In1;// '<S25>/In1'
  SL_Bus_la_2dof_simul_std_msgs_Float64MultiArray BusAssignment1;// '<Root>/Bus Assignment1' 
  SL_Bus_la_2dof_simul_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_la_2dof_simul_std_msgs_MultiArrayDimension b_varargout_2_Layout_Dim[16];
  real_T dv[12];
  real_T dv1[8];
  real_T dv2[8];
  real_T dv3[8];
  real_T dv4[8];
  real_T dv5[8];
  NeParameterBundle expl_temp;
  NeModelParameters modelParameters;
  NeModelParameters modelParameters_m;
  real_T STATE_1[4];                   // '<S24>/STATE_1'
  real_T INPUT_1_1_1[4];               // '<S24>/INPUT_1_1_1'
  real_T INPUT_2_1_1[4];               // '<S24>/INPUT_2_1_1'
  char_T cv[26];
  char_T cv1[20];
  int_T iv[4];
  real_T OUTPUT_1_0[2];                // '<S24>/OUTPUT_1_0'
  real_T b_varargout_2_Data[2];
  int_T iv1[3];
  real_T time;
  real_T time_c;
  real_T time_tmp;
  real_T d;
  real_T time_k;
} B_la_2dof_simul_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slros_internal_block_Publ_T obj; // '<S4>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_a;// '<S8>/SourceBlock'
  real_T INPUT_1_1_1_Discrete[2];      // '<S24>/INPUT_1_1_1'
  real_T INPUT_2_1_1_Discrete[2];      // '<S24>/INPUT_2_1_1'
  real_T STATE_1_Discrete;             // '<S24>/STATE_1'
  real_T OUTPUT_1_0_Discrete;          // '<S24>/OUTPUT_1_0'
  void* STATE_1_Simulator;             // '<S24>/STATE_1'
  void* STATE_1_SimData;               // '<S24>/STATE_1'
  void* STATE_1_DiagMgr;               // '<S24>/STATE_1'
  void* STATE_1_ZcLogger;              // '<S24>/STATE_1'
  void* STATE_1_TsIndex;               // '<S24>/STATE_1'
  void* OUTPUT_1_0_Simulator;          // '<S24>/OUTPUT_1_0'
  void* OUTPUT_1_0_SimData;            // '<S24>/OUTPUT_1_0'
  void* OUTPUT_1_0_DiagMgr;            // '<S24>/OUTPUT_1_0'
  void* OUTPUT_1_0_ZcLogger;           // '<S24>/OUTPUT_1_0'
  void* OUTPUT_1_0_TsIndex;            // '<S24>/OUTPUT_1_0'
  int_T STATE_1_Modes;                 // '<S24>/STATE_1'
  int_T OUTPUT_1_0_Modes;              // '<S24>/OUTPUT_1_0'
  boolean_T STATE_1_FirstOutput;       // '<S24>/STATE_1'
  boolean_T OUTPUT_1_0_FirstOutput;    // '<S24>/OUTPUT_1_0'
} DW_la_2dof_simul_T;

// Continuous states (default storage)
typedef struct {
  real_T la_2dof_simulla_1_jointRzq[4];// '<S24>/STATE_1'
} X_la_2dof_simul_T;

// State derivatives (default storage)
typedef struct {
  real_T la_2dof_simulla_1_jointRzq[4];// '<S24>/STATE_1'
} XDot_la_2dof_simul_T;

// State disabled
typedef struct {
  boolean_T la_2dof_simulla_1_jointRzq[4];// '<S24>/STATE_1'
} XDis_la_2dof_simul_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_la_2dof_simul_T_ {
  SL_Bus_la_2dof_simul_std_msgs_Float64MultiArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                             //  Referenced by: '<S25>/Out1'

  SL_Bus_la_2dof_simul_std_msgs_Float64MultiArray Constant_Value;// Computed Parameter: Constant_Value
                                                                    //  Referenced by: '<S8>/Constant'

  SL_Bus_la_2dof_simul_std_msgs_Float64MultiArray Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                                      //  Referenced by: '<S1>/Constant'

  uint32_T Constant_Value_d;           // Computed Parameter: Constant_Value_d
                                          //  Referenced by: '<Root>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_la_2dof_simul_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_la_2dof_simul_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[4];
  real_T odeF[3][4];
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

  extern P_la_2dof_simul_T la_2dof_simul_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_la_2dof_simul_T la_2dof_simul_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_la_2dof_simul_T la_2dof_simul_X;

// Block states (default storage)
extern DW_la_2dof_simul_T la_2dof_simul_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void la_2dof_simul_initialize(void);
  extern void la_2dof_simul_step(void);
  extern void la_2dof_simul_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_la_2dof_simul_T *const la_2dof_simul_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Rate Transition1' : Eliminated since input and output rates are identical


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
//  '<Root>' : 'la_2dof_simul'
//  '<S1>'   : 'la_2dof_simul/Blank Message1'
//  '<S2>'   : 'la_2dof_simul/PS-Simulink Converter3'
//  '<S3>'   : 'la_2dof_simul/PS-Simulink Converter4'
//  '<S4>'   : 'la_2dof_simul/Publish1'
//  '<S5>'   : 'la_2dof_simul/Simulink-PS Converter'
//  '<S6>'   : 'la_2dof_simul/Simulink-PS Converter1'
//  '<S7>'   : 'la_2dof_simul/Solver Configuration'
//  '<S8>'   : 'la_2dof_simul/Subscribe'
//  '<S9>'   : 'la_2dof_simul/left_arm_grip_center'
//  '<S10>'  : 'la_2dof_simul/left_arm_grip_left'
//  '<S11>'  : 'la_2dof_simul/left_arm_grip_right'
//  '<S12>'  : 'la_2dof_simul/left_arm_link1'
//  '<S13>'  : 'la_2dof_simul/left_arm_link2'
//  '<S14>'  : 'la_2dof_simul/left_arm_link3'
//  '<S15>'  : 'la_2dof_simul/left_arm_link4'
//  '<S16>'  : 'la_2dof_simul/left_arm_link5'
//  '<S17>'  : 'la_2dof_simul/left_arm_link6'
//  '<S18>'  : 'la_2dof_simul/left_arm_link7'
//  '<S19>'  : 'la_2dof_simul/shoulders_left_link'
//  '<S20>'  : 'la_2dof_simul/PS-Simulink Converter3/EVAL_KEY'
//  '<S21>'  : 'la_2dof_simul/PS-Simulink Converter4/EVAL_KEY'
//  '<S22>'  : 'la_2dof_simul/Simulink-PS Converter/EVAL_KEY'
//  '<S23>'  : 'la_2dof_simul/Simulink-PS Converter1/EVAL_KEY'
//  '<S24>'  : 'la_2dof_simul/Solver Configuration/EVAL_KEY'
//  '<S25>'  : 'la_2dof_simul/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_la_2dof_simul_h_

//
// File trailer for generated code.
//
// [EOF]
//
