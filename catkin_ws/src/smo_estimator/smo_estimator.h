//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: smo_estimator.h
//
// Code generated for Simulink model 'smo_estimator'.
//
// Model version                  : 1.257
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Thu Aug 27 13:07:03 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_smo_estimator_h_
#define RTW_HEADER_smo_estimator_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "smo_estimator_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
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
  SL_Bus_smo_estimator_std_msgs_Float64MultiArray In1;// '<S17>/In1'
  SL_Bus_smo_estimator_std_msgs_Float64MultiArray In1_k;// '<S16>/In1'
  SL_Bus_smo_estimator_std_msgs_Float64MultiArray BusAssignment3;// '<S4>/Bus Assignment3' 
  SL_Bus_smo_estimator_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_smo_estimator_std_msgs_Float64MultiArray b_varargout_2_m;
  SL_Bus_smo_estimator_std_msgs_MultiArrayDimension b_varargout_2_Layout_Dim[16];
  real_T b_I[36];
  real_T R[36];
  real_T X[36];
  real_T X_c[36];
  real_T T1[16];
  real_T T2[16];
  real_T R_k[16];
  real_T T[16];
  real_T Tinv[16];
  real_T TJ[16];
  real_T obj[16];
  real_T T_c[16];
  real_T dv[16];
  real_T TJ_b[16];
  real_T obj_p[16];
  real_T c_f1[16];
  real_T a[16];
  real_T b[16];
  real_T a_c[16];
  real_T T_f[16];
  real_T dv1[16];
  real_T TJ_g[16];
  real_T obj_g[16];
  real_T TJ_m[16];
  real_T obj_n[16];
  f_cell_wrap_smo_estimator_T expl_temp;
  real_T xp_est[14];                   // '<S6>/Observer'
  real_T R_p[9];
  real_T R_l[9];
  real_T R_j[9];
  real_T R_d[9];
  real_T dv2[9];
  real_T R_g[9];
  real_T tempR[9];
  real_T R_ld[9];
  real_T tempR_d[9];
  real_T R_dy[9];
  real_T tempR_l[9];
  real_T dv3[9];
  real_T dv4[9];
  real_T R_o[9];
  real_T tempR_b[9];
  real_T R_n[9];
  real_T tempR_bs[9];
  real_T R_ln[9];
  real_T R_h[9];
  real_T z[7];                         // '<S6>/Observer'
  real_T bias[7];
  real_T dv5[7];
  real_T q_data[7];
  real_T q_data_b[7];
  real_T q_data_d[7];
  real_T a0[6];
  real_T y[6];
  real_T X_e[6];
  real_T b_I_b[6];
  int32_T nonFixedIndices_data[10];
  int32_T ii_data[10];
  int8_T msubspace_data[36];
  real_T result_data[4];
  real_T result_data_j[4];
  real_T result_data_f[4];
  int32_T e_data[7];
  char_T cv[26];
  real_T v[3];
  real_T v_a[3];
  real_T v_j[3];
  real_T v_jz[3];
  real_T v_o[3];
  char_T cv1[22];
  char_T cv2[20];
  char_T b_n[19];
  char_T cv3[16];
  char_T b_i[14];
  char_T cv4[12];
  int32_T rtb_MATLABSystem_d_size[3];
  int32_T rtb_MATLABSystem_d_size_o[3];
  int32_T tmp_size[3];
  int32_T rtb_MATLABSystem_d_size_n[3];
  int32_T sy_size[3];
  char_T cv5[11];
  boolean_T mask[10];
  char_T b_m[9];
  char_T b_c[9];
  char_T b_md[8];
  char_T b_m3[8];
  char_T b_j[8];
  char_T b_h[8];
  char_T b_c0[8];
  char_T b_ct[8];
  real_T GetParameter7_o1;             // '<S3>/Get Parameter7'
  real_T GetParameter8_o1;             // '<S3>/Get Parameter8'
  real_T GetParameter9_o1;             // '<S3>/Get Parameter9'
  real_T MATLABSystem[49];             // '<S8>/MATLAB System'
  real_T CoordinateTransformationConvers[3];
                                // '<S2>/Coordinate Transformation Conversion1'
  real_T t;
  real_T bid1;
  real_T vNum;
  real_T j;
  real_T rtb_MATLABSystem_d_data;
  real_T rtb_MATLABSystem_d_data_p;
  real_T tmp_data;
  real_T rtb_MATLABSystem_d_data_p5;
  real_T rtb_LowPassz1_tmp;
  real_T rtb_LowPassz1_tmp_a;
  real_T nb;
  real_T a_idx_1;
  real_T a_idx_0;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_e;
  real_T tempR_tmp_a;
  real_T tempR_tmp_as;
  real_T tempR_tmp_i;
  real_T nb_l;
  real_T vNum_o;
  real_T pid;
  real_T s;
  real_T p_idx_1;
  real_T b_idx_0_o;
  real_T b_idx_1_i;
  real_T b_f;
  real_T axang_idx_0;
  real_T axang_idx_1;
  real_T axang_idx_2;
  real_T n;
  real_T k;
  real_T sth_i;
  real_T tempR_tmp_f;
  real_T tempR_tmp_g;
  real_T tempR_tmp_c;
  real_T tempR_tmp_o;
  real_T nb_lm;
  real_T vNum_m;
  real_T pid_m;
  real_T s_c;
  real_T p_idx_1_f;
  real_T b_idx_0_p;
  real_T b_idx_1_e;
  real_T cth_o;
  real_T sth_h;
  SL_Bus_smo_estimator_std_msgs_Float64 BusAssignment2;// '<S4>/Bus Assignment2' 
  char_T b_l[5];
  char_T b_h2[5];
  char_T b_me[5];
  char_T b_mc[5];
  char_T b_h3[5];
  int32_T b_kstr;
  int32_T n_c;
  int32_T iend;
  int32_T i;
  int32_T u1;
  int32_T i_k;
  int32_T rtb_MATLABSystem_d_tmp;
  int32_T i_p;
  int32_T i_px;
  int32_T b_k;
  int32_T p;
  int32_T m;
  int32_T inner;
  int32_T aoffset;
  int32_T i_p4;
  int32_T q_size;
  int32_T unnamed_idx_1;
  int32_T loop_ub_tmp;
  int32_T q_size_tmp;
  int32_T kstr;
  int32_T b_kstr_a;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  int32_T kstr_j;
  int32_T b_kstr_e;
  int32_T b_i_o;
  int32_T f;
  int32_T cb;
  int32_T idx;
  int32_T n_b;
  int32_T nm1d2;
  int32_T m_a;
  int32_T coffset;
  int32_T boffset;
  int32_T aoffset_g;
  int32_T loop_ub;
  int32_T q_size_e;
  int32_T pid_tmp;
  int32_T X_tmp;
  int32_T coffset_tmp;
  int32_T kstr_f;
  int32_T b_kstr_h;
  int32_T obj_tmp_e;
  int32_T obj_tmp_tmp_c;
  int32_T d;
  int32_T e;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr_ax;
  int32_T loop_ub_d;
  int32_T kstr_a;
  int32_T b_kstr_p;
  int32_T c;
  int32_T f_m;
  int32_T g;
  int32_T cb_o;
  int32_T n_n;
  int32_T m_l;
  int32_T coffset_p;
  int32_T boffset_p;
  int32_T aoffset_f;
  int32_T k_i;
  int32_T loop_ub_o;
  int32_T q_size_k;
  int32_T c_tmp;
  int32_T pid_tmp_i;
  int32_T X_tmp_o;
  int32_T i1;
  int32_T i2;
  int32_T X_tmp_m;
  int32_T X_tmp_c;
  int32_T i3;
  int32_T Tinv_tmp;
  int32_T newNumel;
  int32_T i_f;
  int32_T newNumel_h;
  int32_T i_m;
  int32_T newNumel_a;
  int32_T i_kb;
  int32_T i_pc;
  int32_T i_b;
  int32_T i_c;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_j;
  boolean_T b_varargout_1;
  boolean_T b_bool;
  boolean_T b_bool_e;
  boolean_T b_bool_m;
  boolean_T b_bool_m0;
  boolean_T b_bool_j;
  boolean_T b_bool_f;
  boolean_T b_bool_a;
} B_smo_estimator_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal__mr_T obj; // '<S18>/MATLAB System'
  robotics_slmanip_internal_b_m_T obj_h;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_1;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_2;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_3;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_4;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_5;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_6;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_7;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_8;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_9;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_10;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_11;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_12;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_13;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_14;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_15;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_16;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_17;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_18;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_19;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_20;// '<S18>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_1_m;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_2_g;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_3_b;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_4_m;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_5_o;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_6_i;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_7_f;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_8_b;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_9_g;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_10_p;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_11_o;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_12_p;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_13_n;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_14_a;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_15_k;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_16_e;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_17_i;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_18_h;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_19_o;// '<S8>/MATLAB System'
  o_robotics_manip_internal_R_m_T gobj_20_i;// '<S8>/MATLAB System'
  robotics_slmanip_internal_blo_T obj_o;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_1_g;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_2_i;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_3_b3;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_4_p;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_5_n;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_6_h;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_7_n;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_8_h;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_9_m;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_10_h;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_11_h;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_12_d;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_13_d;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_14_m;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_15_c;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_16_b;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_17_l;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_18_f;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_19_i;// '<S7>/MATLAB System'
  o_robotics_manip_internal_Rig_T gobj_20_n;// '<S7>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_a;// '<S3>/Get Parameter7'
  ros_slros_internal_block_GetP_T obj_oo;// '<S3>/Get Parameter8'
  ros_slros_internal_block_GetP_T obj_ok;// '<S3>/Get Parameter9'
  ros_slros_internal_block_Publ_T obj_b;// '<S13>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_ap;// '<S12>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_ay;// '<S15>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_m;// '<S14>/SourceBlock'
  real_T RateTransition_Buffer;        // '<S4>/Rate Transition'
} DW_smo_estimator_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator_CSTATE[14];        // '<S6>/Integrator'
  real_T LowPassz1_CSTATE[4];          // '<S1>/Low Pass (z1)'
  real_T LowPassz11_CSTATE[4];         // '<S1>/Low Pass (z1)1'
  real_T LowPassz12_CSTATE[4];         // '<S1>/Low Pass (z1)2'
  real_T LowPassz13_CSTATE[4];         // '<S1>/Low Pass (z1)3'
  real_T LowPassz14_CSTATE[4];         // '<S1>/Low Pass (z1)4'
  real_T LowPassz15_CSTATE[4];         // '<S1>/Low Pass (z1)5'
  real_T LowPassz16_CSTATE[4];         // '<S1>/Low Pass (z1)6'
} X_smo_estimator_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator_CSTATE[14];        // '<S6>/Integrator'
  real_T LowPassz1_CSTATE[4];          // '<S1>/Low Pass (z1)'
  real_T LowPassz11_CSTATE[4];         // '<S1>/Low Pass (z1)1'
  real_T LowPassz12_CSTATE[4];         // '<S1>/Low Pass (z1)2'
  real_T LowPassz13_CSTATE[4];         // '<S1>/Low Pass (z1)3'
  real_T LowPassz14_CSTATE[4];         // '<S1>/Low Pass (z1)4'
  real_T LowPassz15_CSTATE[4];         // '<S1>/Low Pass (z1)5'
  real_T LowPassz16_CSTATE[4];         // '<S1>/Low Pass (z1)6'
} XDot_smo_estimator_T;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE[14];     // '<S6>/Integrator'
  boolean_T LowPassz1_CSTATE[4];       // '<S1>/Low Pass (z1)'
  boolean_T LowPassz11_CSTATE[4];      // '<S1>/Low Pass (z1)1'
  boolean_T LowPassz12_CSTATE[4];      // '<S1>/Low Pass (z1)2'
  boolean_T LowPassz13_CSTATE[4];      // '<S1>/Low Pass (z1)3'
  boolean_T LowPassz14_CSTATE[4];      // '<S1>/Low Pass (z1)4'
  boolean_T LowPassz15_CSTATE[4];      // '<S1>/Low Pass (z1)5'
  boolean_T LowPassz16_CSTATE[4];      // '<S1>/Low Pass (z1)6'
} XDis_smo_estimator_T;

#ifndef ODE4_INTG
#define ODE4_INTG

// ODE4 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[4];                        // derivatives
} ODE4_IntgData;

#endif

// Parameters (default storage)
struct P_smo_estimator_T_ {
  real_T lowpass_A[5];                 // Variable: lowpass_A
                                          //  Referenced by:
                                          //    '<S1>/Low Pass (z1)'
                                          //    '<S1>/Low Pass (z1)1'
                                          //    '<S1>/Low Pass (z1)2'
                                          //    '<S1>/Low Pass (z1)3'
                                          //    '<S1>/Low Pass (z1)4'
                                          //    '<S1>/Low Pass (z1)5'
                                          //    '<S1>/Low Pass (z1)6'

  real_T lowpass_B[5];                 // Variable: lowpass_B
                                          //  Referenced by:
                                          //    '<S1>/Low Pass (z1)'
                                          //    '<S1>/Low Pass (z1)1'
                                          //    '<S1>/Low Pass (z1)2'
                                          //    '<S1>/Low Pass (z1)3'
                                          //    '<S1>/Low Pass (z1)4'
                                          //    '<S1>/Low Pass (z1)5'
                                          //    '<S1>/Low Pass (z1)6'

  SL_Bus_smo_estimator_std_msgs_Float64MultiArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                             //  Referenced by: '<S16>/Out1'

  SL_Bus_smo_estimator_std_msgs_Float64MultiArray Constant_Value;// Computed Parameter: Constant_Value
                                                                    //  Referenced by: '<S14>/Constant'

  SL_Bus_smo_estimator_std_msgs_Float64MultiArray Out1_Y0_k;// Computed Parameter: Out1_Y0_k
                                                               //  Referenced by: '<S17>/Out1'

  SL_Bus_smo_estimator_std_msgs_Float64MultiArray Constant_Value_f;// Computed Parameter: Constant_Value_f
                                                                      //  Referenced by: '<S15>/Constant'

  SL_Bus_smo_estimator_std_msgs_Float64MultiArray Constant_Value_i;// Computed Parameter: Constant_Value_i
                                                                      //  Referenced by: '<S11>/Constant'

  SL_Bus_smo_estimator_std_msgs_Float64 Constant_Value_k;// Computed Parameter: Constant_Value_k
                                                            //  Referenced by: '<S10>/Constant'

  real_T Constant2_Value[60];          // Expression: zeros(6,10)
                                          //  Referenced by: '<S6>/Constant2'

  real_T Integrator_IC[14];         // Expression: [0 0 0 0 0 0 0 0 0 0 0 0 0 0]
                                       //  Referenced by: '<S6>/Integrator'

  uint32_T Constant1_Value;            // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S4>/Constant1'

};

// Real-time Model Data Structure
struct tag_RTM_smo_estimator_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_smo_estimator_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[42];
  real_T odeF[4][42];
  ODE4_IntgData intgData;

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
      uint16_T TID[4];
    } TaskCounters;

    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[4];
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_smo_estimator_T smo_estimator_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_smo_estimator_T smo_estimator_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_smo_estimator_T smo_estimator_X;

// Block states (default storage)
extern DW_smo_estimator_T smo_estimator_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void smo_estimator_initialize(void);
  extern void smo_estimator_step(void);
  extern void smo_estimator_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_smo_estimator_T *const smo_estimator_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S4>/Rate Transition4' : Eliminated since input and output rates are identical
//  Block '<S5>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S5>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S18>/Reshape' : Reshape block reduction


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
//  '<Root>' : 'smo_estimator'
//  '<S1>'   : 'smo_estimator/Low Pass Filter'
//  '<S2>'   : 'smo_estimator/Mass Estimator'
//  '<S3>'   : 'smo_estimator/ROS Params SMO'
//  '<S4>'   : 'smo_estimator/ROS Publishers'
//  '<S5>'   : 'smo_estimator/ROS Subscribers'
//  '<S6>'   : 'smo_estimator/SMO2'
//  '<S7>'   : 'smo_estimator/Mass Estimator/Get Transform'
//  '<S8>'   : 'smo_estimator/Mass Estimator/Joint Space Mass Matrix'
//  '<S9>'   : 'smo_estimator/Mass Estimator/mass estimator'
//  '<S10>'  : 'smo_estimator/ROS Publishers/Blank Message2'
//  '<S11>'  : 'smo_estimator/ROS Publishers/Blank Message3'
//  '<S12>'  : 'smo_estimator/ROS Publishers/Publish2'
//  '<S13>'  : 'smo_estimator/ROS Publishers/Publish3'
//  '<S14>'  : 'smo_estimator/ROS Subscribers/Subscribe'
//  '<S15>'  : 'smo_estimator/ROS Subscribers/Subscribe1'
//  '<S16>'  : 'smo_estimator/ROS Subscribers/Subscribe/Enabled Subsystem'
//  '<S17>'  : 'smo_estimator/ROS Subscribers/Subscribe1/Enabled Subsystem'
//  '<S18>'  : 'smo_estimator/SMO2/Forward Dynamics'
//  '<S19>'  : 'smo_estimator/SMO2/Observer'

#endif                                 // RTW_HEADER_smo_estimator_h_

//
// File trailer for generated code.
//
// [EOF]
//
