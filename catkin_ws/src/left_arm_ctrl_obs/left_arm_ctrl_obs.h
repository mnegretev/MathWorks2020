//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_ctrl_obs.h
//
// Code generated for Simulink model 'left_arm_ctrl_obs'.
//
// Model version                  : 1.209
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Tue Jul 14 14:11:04 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_left_arm_ctrl_obs_h_
#define RTW_HEADER_left_arm_ctrl_obs_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#ifndef left_arm_ctrl_obs_COMMON_INCLUDES_
# define left_arm_ctrl_obs_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#endif                                 // left_arm_ctrl_obs_COMMON_INCLUDES_

#include "left_arm_ctrl_obs_types.h"
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
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray In1;// '<S16>/In1'
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray In1_e;// '<S17>/In1'
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray BusAssignment1;// '<Root>/Bus Assignment1' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray b_varargout_2_m;
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension
    b_varargout_2_Layout_Dim[16];
  real_T MATLABSystem[49];             // '<S7>/MATLAB System'
  real_T b_I[36];
  real_T tempR[36];
  real_T b_I_c[36];
  real_T Xtree[36];
  real_T R[36];
  real_T X[36];
  real_T X_k[36];
  real_T T1[16];
  real_T T2[16];
  real_T R_c[16];
  real_T T[16];
  real_T TJ[16];
  real_T a[16];
  real_T b[16];
  real_T a_b[16];
  real_T TJ_p[16];
  real_T obj[16];
  real_T T_c[16];
  real_T Tinv[16];
  real_T TJ_f[16];
  real_T obj_g[16];
  real_T T_g[16];
  real_T dv[16];
  real_T c_f1[16];
  real_T a_m[16];
  real_T b_n[16];
  real_T a_p[16];
  real_T T_l[16];
  real_T dv1[16];
  real_T TJ_j[16];
  real_T obj_d[16];
  real_T TJ_g[16];
  real_T obj_l[16];
  real_T TJ_d[16];
  real_T obj_dy[16];
  e_cell_wrap_left_arm_ctrl_obs_T expl_temp;
  real_T xp_est[14];                   // '<Root>/Observer'
  real_T R_l[9];
  real_T R_o[9];
  real_T R_b[9];
  real_T tempR_n[9];
  real_T dv2[9];
  real_T R_bs[9];
  real_T tempR_l[9];
  real_T R_h[9];
  real_T R_bn[9];
  real_T dv3[9];
  real_T dv4[9];
  real_T dv5[9];
  real_T R_d[9];
  real_T tempR_e[9];
  real_T dv6[9];
  real_T dv7[9];
  real_T R_bj[9];
  real_T tempR_j[9];
  real_T R_f[9];
  real_T R_a[9];
  real_T R_j[9];
  real_T tempR_jz[9];
  real_T R_o4[9];
  real_T tempR_ny[9];
  real_T R_i[9];
  real_T tempR_o[9];
  real_T z[7];                         // '<Root>/Observer'
  real_T qe[7];
  real_T dv8[7];
  real_T torque[7];
  real_T tau[7];
  real_T q[7];
  real_T qddoti_data[7];
  real_T q_data[7];
  real_T q_data_n[7];
  real_T q_data_m[7];
  real32_T b_varargout_2_Data[14];
  real_T a0[6];
  real_T y[6];
  real_T X_c[6];
  real_T b_I_m[6];
  real_T a0_m[6];
  real_T y_j[6];
  real_T vJ[6];
  real_T b_I_h[6];
  real_T b_I_c0[6];
  real_T R_ct[6];
  int32_T nonFixedIndices_data[10];
  int32_T ii_data[10];
  real_T result_data[4];
  real_T result_data_p[4];
  real_T result_data_p5[4];
  real_T result_data_a[4];
  int32_T l_data[7];
  int32_T e_data[7];
  real_T CoordinateTransformationConvers[3];
                               // '<Root>/Coordinate Transformation Conversion'
  real32_T MATLABSystem_p[7];          // '<S6>/MATLAB System'
  real_T v[3];
  real_T v_e[3];
  real_T v_a[3];
  real_T v_as[3];
  real_T v_i[3];
  real_T v_l[3];
  real_T v_o[3];
  real_T v_o2[3];
  real_T v_ip[3];
  real_T v_f[3];
  char_T b_i[19];
  char_T b_f[14];
  int32_T rtb_MATLABSystem_size[3];
  int32_T rtb_MATLABSystem_size_g[3];
  int32_T tmp_size[3];
  int32_T rtb_MATLABSystem_size_c[3];
  int32_T sy_size[3];
  boolean_T mask[10];
  char_T b_o[9];
  char_T b_l[9];
  char_T b_m[8];
  char_T b_mj[8];
  char_T b_c[8];
  char_T b_fm[8];
  char_T b_p[8];
  char_T b_e[8];
  char_T b_o4[8];
  real_T vel;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T bid1;
  real_T vNum;
  real_T j;
  real_T LowPassz25;                   // '<Root>/Low Pass (z2)5'
  real_T LowPassz24;                   // '<Root>/Low Pass (z2)4'
  real_T LowPassz23;                   // '<Root>/Low Pass (z2)3'
  real_T LowPassz22;                   // '<Root>/Low Pass (z2)2'
  real_T rtb_MATLABSystem_data;
  real_T rtb_MATLABSystem_data_h;
  real_T tmp_data;
  real_T rtb_MATLABSystem_data_l;
  real_T nb;
  real_T sth;
  real_T a_idx_1;
  real_T a_idx_0;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T tempR_tmp;
  real_T tempR_tmp_h;
  real_T tempR_tmp_m;
  real_T tempR_tmp_mc;
  real_T tempR_tmp_h3;
  real_T nb_c;
  real_T a_idx_1_k;
  real_T a_idx_0_p;
  real_T b_idx_0_p;
  real_T b_idx_1_p;
  real_T cth;
  real_T sth_a;
  real_T tempR_tmp_j;
  real_T tempR_tmp_e;
  real_T tempR_tmp_o;
  real_T tempR_tmp_b;
  real_T tempR_tmp_a;
  real_T b_g;
  real_T nb_e;
  real_T vNum_f;
  real_T pid;
  real_T temp;
  real_T p_idx_1;
  real_T b_idx_0_h;
  real_T b_idx_1_e;
  real_T n;
  real_T k;
  real_T sth_c;
  real_T tempR_tmp_ax;
  real_T tempR_tmp_d;
  real_T tempR_tmp_af;
  real_T tempR_tmp_p;
  real_T nb_m;
  real_T vNum_o;
  real_T pid_n;
  real_T temp_l;
  real_T p_idx_1_p;
  real_T b_idx_0_pt;
  real_T b_idx_1_f;
  real_T cth_i;
  real_T sth_o;
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment2;// '<Root>/Bus Assignment2' 
  char_T b_k[5];
  char_T b_ie[5];
  char_T b_oc[5];
  char_T b_m4[5];
  char_T b_cu[5];
  char_T b_fb[5];
  char_T b_h[5];
  int32_T n_m;
  int32_T iend;
  int32_T j_a;
  int32_T i;
  int32_T loop_ub;
  int32_T u1;
  int32_T i_k;
  int32_T b_k_p;
  int32_T j_b;
  int32_T t_c;
  int32_T u;
  int32_T m;
  int32_T inner;
  int32_T aoffset;
  int32_T c_i;
  int32_T i_n;
  int32_T unnamed_idx_1;
  int32_T kstr;
  int32_T b_kstr;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  int32_T kstr_i;
  int32_T b_kstr_m;
  int32_T b_k_j;
  int32_T m_e;
  int32_T inner_m;
  int32_T aoffset_m;
  int32_T c_i_j;
  int32_T i_f;
  int32_T q_size;
  int32_T unnamed_idx_1_a;
  int32_T loop_ub_tmp;
  int32_T p_tmp;
  int32_T o_tmp;
  int32_T kstr_g;
  int32_T b_kstr_n;
  int32_T obj_tmp_d;
  int32_T obj_tmp_tmp_n;
  int32_T i_c;
  int32_T i_fx;
  int32_T b_i_p;
  int32_T cb;
  int32_T idx;
  int32_T n_p;
  int32_T aoffset_n;
  int32_T i_kt;
  int32_T b_j;
  int32_T c_i_n;
  int32_T nm1d2;
  int32_T m_o;
  int32_T coffset;
  int32_T boffset;
  int32_T q_size_g;
  int32_T unnamed_idx_1_c;
  int32_T pid_tmp;
  int32_T q_size_tmp;
  int32_T i1;
  int32_T i2;
  int32_T X_tmp;
  int32_T X_tmp_c;
  int32_T d;
  int32_T e;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr_m1;
  int32_T loop_ub_j;
  int32_T kstr_k;
  int32_T b_kstr_mx;
  int32_T c;
  int32_T b_i_pr;
  int32_T f;
  int32_T n_d;
  int32_T aoffset_g;
  int32_T i_c3;
  int32_T b_j_c;
  int32_T c_i_i;
  int32_T m_d;
  int32_T coffset_g;
  int32_T boffset_l;
  int32_T q_size_f;
  int32_T c_tmp;
  int32_T pid_tmp_d;
  int32_T q_size_tmp_j;
  int32_T X_tmp_i;
  int32_T i3;
  int32_T Tinv_tmp;
  int32_T newNumel;
  int32_T i_h;
  int32_T newNumel_n;
  int32_T i_o;
  int32_T newNumel_c;
  int32_T i_b;
  int32_T newNumel_e;
  int32_T i_d;
  int32_T kstr_ik;
  int32_T b_kstr_g;
  int32_T i_nn;
  int32_T i_l;
  int32_T i_ce;
  int32_T i_nc;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_j;
  boolean_T b_varargout_1;
  boolean_T b_bool;
  boolean_T b_bool_c;
  boolean_T b_bool_h;
  boolean_T b_bool_d;
  boolean_T b_bool_c5;
  boolean_T b_bool_p;
  boolean_T b_bool_pi;
  boolean_T b_bool_a;
} B_left_arm_ctrl_obs_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal_blo_T obj; // '<S4>/MATLAB System'
  robotics_slmanip_internal__e0_T obj_j;// '<S6>/MATLAB System'
  robotics_slmanip_internal_e0h_T obj_jz;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_0;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_1;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_2;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_3;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_4;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_5;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_6;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_7;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_8;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_9;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_10;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_11;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_12;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_13;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_14;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_15;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_16;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_17;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_18;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_19;// '<S7>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_0_f;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_1_o;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_2_p;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_3_j;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_4_p;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_5_g;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_6_k;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_7_a;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_8_g;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_9_k;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_10_n;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_11_b;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_12_g;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_13_l;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_14_a;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_15_k;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_16_a;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_17_d;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_18_b;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_19_h;// '<S6>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_0_e;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_1_f;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_2_b;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_3_f;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_4_f;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_5_e;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_6_ke;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_7_i;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_8_j;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_9_l;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_10_e;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_11_p;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_12_m;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_13_n;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_14_f;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_15_d;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_16_e;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_17_e;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_18_m;// '<S4>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_19_i;// '<S4>/MATLAB System'
  robotics_slmanip_internal_b_e_T obj_f;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_0_eq;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_1_m;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_2_f;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_3_o;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_4_h;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_5_h;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_6_a;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_7_ia;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_8_o;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_9_e;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_10_h;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_11_h;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_12_h;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_13_d;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_14_n;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_15_h;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_16_i;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_17_m;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_18_j;// '<S5>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_19_b;// '<S5>/MATLAB System'
  ros_slros_internal_block_Publ_T obj_b;// '<S12>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_a;// '<S11>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_d;// '<S10>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_g;// '<S14>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_m;// '<S13>/SourceBlock'
  real_T RateTransition_Buffer;        // '<Root>/Rate Transition'
} DW_left_arm_ctrl_obs_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator_CSTATE[14];        // '<Root>/Integrator'
  real_T LowPassz1_CSTATE[5];          // '<Root>/Low Pass (z1)'
  real_T LowPassz2_CSTATE[5];          // '<Root>/Low Pass (z2)'
  real_T LowPassz21_CSTATE[5];         // '<Root>/Low Pass (z2)1'
  real_T LowPassz22_CSTATE[5];         // '<Root>/Low Pass (z2)2'
  real_T LowPassz23_CSTATE[5];         // '<Root>/Low Pass (z2)3'
  real_T LowPassz24_CSTATE[5];         // '<Root>/Low Pass (z2)4'
  real_T LowPassz25_CSTATE[5];         // '<Root>/Low Pass (z2)5'
} X_left_arm_ctrl_obs_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator_CSTATE[14];        // '<Root>/Integrator'
  real_T LowPassz1_CSTATE[5];          // '<Root>/Low Pass (z1)'
  real_T LowPassz2_CSTATE[5];          // '<Root>/Low Pass (z2)'
  real_T LowPassz21_CSTATE[5];         // '<Root>/Low Pass (z2)1'
  real_T LowPassz22_CSTATE[5];         // '<Root>/Low Pass (z2)2'
  real_T LowPassz23_CSTATE[5];         // '<Root>/Low Pass (z2)3'
  real_T LowPassz24_CSTATE[5];         // '<Root>/Low Pass (z2)4'
  real_T LowPassz25_CSTATE[5];         // '<Root>/Low Pass (z2)5'
} XDot_left_arm_ctrl_obs_T;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE[14];     // '<Root>/Integrator'
  boolean_T LowPassz1_CSTATE[5];       // '<Root>/Low Pass (z1)'
  boolean_T LowPassz2_CSTATE[5];       // '<Root>/Low Pass (z2)'
  boolean_T LowPassz21_CSTATE[5];      // '<Root>/Low Pass (z2)1'
  boolean_T LowPassz22_CSTATE[5];      // '<Root>/Low Pass (z2)2'
  boolean_T LowPassz23_CSTATE[5];      // '<Root>/Low Pass (z2)3'
  boolean_T LowPassz24_CSTATE[5];      // '<Root>/Low Pass (z2)4'
  boolean_T LowPassz25_CSTATE[5];      // '<Root>/Low Pass (z2)5'
} XDis_left_arm_ctrl_obs_T;

#ifndef ODE4_INTG
#define ODE4_INTG

// ODE4 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[4];                        // derivatives
} ODE4_IntgData;

#endif

// Parameters (default storage)
struct P_left_arm_ctrl_obs_T_ {
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                                 //  Referenced by: '<S17>/Out1'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S14>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Out1_Y0_a;// Computed Parameter: Out1_Y0_a
                                                                   //  Referenced by: '<S16>/Out1'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                      //  Referenced by: '<S13>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_i;// Computed Parameter: Constant_Value_i
                                                                      //  Referenced by: '<S3>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_e;// Computed Parameter: Constant_Value_e
                                                                      //  Referenced by: '<S1>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_k;// Computed Parameter: Constant_Value_k
                                                                //  Referenced by: '<S2>/Constant'

  real_T Constant2_Value[60];          // Expression: zeros(6,10)
                                          //  Referenced by: '<Root>/Constant2'

  real_T Integrator_IC[14];         // Expression: [0 0 0 0 0 0 0 0 0 0 0 0 0 0]
                                       //  Referenced by: '<Root>/Integrator'

  real_T LowPassz1_A[5];               // Computed Parameter: LowPassz1_A
                                          //  Referenced by: '<Root>/Low Pass (z1)'

  real_T LowPassz1_C[5];               // Computed Parameter: LowPassz1_C
                                          //  Referenced by: '<Root>/Low Pass (z1)'

  real_T LowPassz2_A[5];               // Computed Parameter: LowPassz2_A
                                          //  Referenced by: '<Root>/Low Pass (z2)'

  real_T LowPassz2_C[5];               // Computed Parameter: LowPassz2_C
                                          //  Referenced by: '<Root>/Low Pass (z2)'

  real_T LowPassz21_A[5];              // Computed Parameter: LowPassz21_A
                                          //  Referenced by: '<Root>/Low Pass (z2)1'

  real_T LowPassz21_C[5];              // Computed Parameter: LowPassz21_C
                                          //  Referenced by: '<Root>/Low Pass (z2)1'

  real_T LowPassz22_A[5];              // Computed Parameter: LowPassz22_A
                                          //  Referenced by: '<Root>/Low Pass (z2)2'

  real_T LowPassz22_C[5];              // Computed Parameter: LowPassz22_C
                                          //  Referenced by: '<Root>/Low Pass (z2)2'

  real_T LowPassz23_A[5];              // Computed Parameter: LowPassz23_A
                                          //  Referenced by: '<Root>/Low Pass (z2)3'

  real_T LowPassz23_C[5];              // Computed Parameter: LowPassz23_C
                                          //  Referenced by: '<Root>/Low Pass (z2)3'

  real_T LowPassz24_A[5];              // Computed Parameter: LowPassz24_A
                                          //  Referenced by: '<Root>/Low Pass (z2)4'

  real_T LowPassz24_C[5];              // Computed Parameter: LowPassz24_C
                                          //  Referenced by: '<Root>/Low Pass (z2)4'

  real_T LowPassz25_A[5];              // Computed Parameter: LowPassz25_A
                                          //  Referenced by: '<Root>/Low Pass (z2)5'

  real_T LowPassz25_C[5];              // Computed Parameter: LowPassz25_C
                                          //  Referenced by: '<Root>/Low Pass (z2)5'

  uint32_T Constant_Value_d;           // Computed Parameter: Constant_Value_d
                                          //  Referenced by: '<Root>/Constant'

  uint32_T Constant1_Value;            // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<Root>/Constant1'

};

// Real-time Model Data Structure
struct tag_RTM_left_arm_ctrl_obs_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_left_arm_ctrl_obs_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[49];
  real_T odeF[4][49];
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

  extern P_left_arm_ctrl_obs_T left_arm_ctrl_obs_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_left_arm_ctrl_obs_T left_arm_ctrl_obs_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_left_arm_ctrl_obs_T left_arm_ctrl_obs_X;

// Block states (default storage)
extern DW_left_arm_ctrl_obs_T left_arm_ctrl_obs_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void left_arm_ctrl_obs_initialize(void);
  extern void left_arm_ctrl_obs_step(void);
  extern void left_arm_ctrl_obs_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_left_arm_ctrl_obs_T *const left_arm_ctrl_obs_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S4>/Reshape' : Reshape block reduction
//  Block '<S6>/Reshape' : Reshape block reduction
//  Block '<Root>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<Root>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<Root>/Rate Transition3' : Eliminated since input and output rates are identical
//  Block '<Root>/Rate Transition4' : Eliminated since input and output rates are identical


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
//  '<Root>' : 'left_arm_ctrl_obs'
//  '<S1>'   : 'left_arm_ctrl_obs/Blank Message1'
//  '<S2>'   : 'left_arm_ctrl_obs/Blank Message2'
//  '<S3>'   : 'left_arm_ctrl_obs/Blank Message3'
//  '<S4>'   : 'left_arm_ctrl_obs/Forward Dynamics'
//  '<S5>'   : 'left_arm_ctrl_obs/Get Transform'
//  '<S6>'   : 'left_arm_ctrl_obs/Gravity Torque'
//  '<S7>'   : 'left_arm_ctrl_obs/Joint Space Mass Matrix'
//  '<S8>'   : 'left_arm_ctrl_obs/MATLAB Function1'
//  '<S9>'   : 'left_arm_ctrl_obs/Observer'
//  '<S10>'  : 'left_arm_ctrl_obs/Publish1'
//  '<S11>'  : 'left_arm_ctrl_obs/Publish2'
//  '<S12>'  : 'left_arm_ctrl_obs/Publish3'
//  '<S13>'  : 'left_arm_ctrl_obs/Subscribe'
//  '<S14>'  : 'left_arm_ctrl_obs/Subscribe1'
//  '<S15>'  : 'left_arm_ctrl_obs/mass estimator'
//  '<S16>'  : 'left_arm_ctrl_obs/Subscribe/Enabled Subsystem'
//  '<S17>'  : 'left_arm_ctrl_obs/Subscribe1/Enabled Subsystem'

#endif                                 // RTW_HEADER_left_arm_ctrl_obs_h_

//
// File trailer for generated code.
//
// [EOF]
//
