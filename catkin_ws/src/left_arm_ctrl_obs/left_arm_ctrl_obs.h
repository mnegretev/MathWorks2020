//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_ctrl_obs.h
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

// Block signals for system '<S13>/MATLAB System'
typedef struct {
  real_T b_I[36];
  real_T Xtree[36];
  real_T R[36];
  real_T X[36];
  real_T T[16];
  real_T Tinv[16];
  real_T TJ[16];
  real_T obj[16];
  real_T T_m[16];
  real_T dv[16];
  real_T TJ_c[16];
  real_T obj_k[16];
  real_T R_c[9];
  real_T R_b[9];
  real_T dv1[9];
  real_T dv2[9];
  real_T dv3[9];
  real_T R_p[9];
  real_T tempR[9];
  real_T R_cv[9];
  real_T tempR_f[9];
  real_T dv4[9];
  real_T dv5[9];
  real_T R_g[9];
  real_T R_g1[9];
  real_T MATLABSystem[7];              // '<S13>/MATLAB System'
  real_T u0[7];
  real_T u1[7];
  real_T q_data[7];
  real_T q_data_m[7];
  real_T a0[6];
  real_T y[6];
  real_T vJ[6];
  real_T b_I_n[6];
  real_T b_I_p[6];
  real_T R_l[6];
  int32_T nonFixedIndices_data[10];
  int32_T ii_data[10];
  real_T result_data[4];
  real_T v[3];
  real_T v_j[3];
  real_T v_d[3];
  boolean_T mask[10];
  char_T b[9];
  char_T b_g[8];
  char_T b_l[8];
  char_T b_d[8];
  real_T vNum;
  real_T k;
  real_T j;
  real_T nb;
  real_T a_idx_1;
  real_T a_idx_0;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T cth;
  real_T sth;
  real_T tempR_tmp;
  real_T tempR_tmp_d;
  real_T tempR_tmp_l;
  real_T tempR_tmp_o;
  real_T tempR_tmp_b;
  real_T nb_n;
  real_T vNum_b;
  real_T pid;
  real_T temp;
  real_T p_idx_1;
  real_T b_idx_0_l;
  real_T b_idx_1_h;
  real_T tempR_tmp_bn;
  real_T tempR_tmp_da;
  real_T tempR_tmp_e;
  real_T b_b;
  char_T b_j[5];
  char_T b_f[5];
  char_T b_a[5];
  char_T b_ju[5];
  int32_T n;
  int32_T iend;
  int32_T j_j;
  int32_T i;
  int32_T u0_o;
  int32_T i_n;
  int32_T MATLABSystem_tmp;
  int32_T b_k;
  int32_T m;
  int32_T inner;
  int32_T aoffset;
  int32_T c_i;
  int32_T i_i;
  int32_T q_size;
  int32_T unnamed_idx_1;
  int32_T loop_ub_tmp;
  int32_T p_tmp;
  int32_T o_tmp;
  int32_T kstr;
  int32_T b_kstr;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  int32_T kstr_o;
  int32_T b_kstr_n;
  int32_T b_i;
  int32_T cb;
  int32_T idx;
  int32_T n_m;
  int32_T aoffset_c;
  int32_T i_m;
  int32_T b_j_m;
  int32_T c_i_j;
  int32_T nm1d2;
  int32_T m_h;
  int32_T coffset;
  int32_T boffset;
  int32_T q_size_c;
  int32_T unnamed_idx_1_c;
  int32_T pid_tmp;
  int32_T q_size_tmp;
  int32_T kstr_p;
  int32_T b_kstr_p;
  int32_T obj_tmp_a;
  int32_T obj_tmp_tmp_e;
  int32_T i1;
  int32_T i2;
  int32_T X_tmp;
  int32_T X_tmp_a;
  int32_T i3;
  int32_T Tinv_tmp;
  int32_T newNumel;
  int32_T i_a;
  int32_T newNumel_i;
  int32_T i_l;
  int32_T newNumel_o;
  int32_T i_o;
  int32_T i_ip;
  int32_T i_f;
  int32_T i_iz;
  boolean_T b_bool;
  boolean_T b_bool_f;
  boolean_T b_bool_g;
  boolean_T b_bool_c;
  boolean_T b_bool_o;
} B_MATLABSystem_left_arm_ctrl__T;

// Block states (default storage) for system '<S13>/MATLAB System'
typedef struct {
  robotics_slmanip_internal_blo_T obj; // '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_0;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_1;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_2;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_3;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_4;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_5;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_6;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_7;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_8;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_9;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_10;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_11;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_12;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_13;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_14;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_15;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_16;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_17;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_18;// '<S13>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_19;// '<S13>/MATLAB System'
  boolean_T objisempty;                // '<S13>/MATLAB System'
} DW_MATLABSystem_left_arm_ctrl_T;

// Block signals (default storage)
typedef struct {
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray In1;// '<S31>/In1'
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray In1_e;// '<S32>/In1'
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray BusAssignment3;// '<S9>/Bus Assignment3' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray b_varargout_2_m;
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension
    b_varargout_2_Layout_Dim[16];
  real_T F[196];
  real_T F_c[196];
  real_T dv[196];
  real_T K[98];
  real_T K_k[98];
  real_T R[49];
  real_T R_tmp[49];
  real_T b_I[36];
  real_T tempR[36];
  real_T X[36];
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
  real_T dv1[16];
  real_T TJ_f[16];
  real_T obj_g[16];
  real_T c_f1[16];
  real_T a_g[16];
  real_T b_m[16];
  real_T a_n[16];
  real_T TJ_pp[16];
  real_T obj_l[16];
  e_cell_wrap_left_arm_ctrl_obs_T expl_temp;
  real_T Integrator[14];               // '<S1>/Integrator'
  real_T Integrator_f[14];             // '<S11>/Integrator'
  real_T xp_est[14];                   // '<S11>/Observer'
  real_T R_j[9];
  real_T R_d[9];
  real_T R_g[9];
  real_T tempR_l[9];
  real_T dv2[9];
  real_T R_dh[9];
  real_T tempR_d[9];
  real_T R_l[9];
  real_T tempR_o[9];
  real_T R_b[9];
  real_T tempR_n[9];
  real_T R_bs[9];
  real_T tempR_ln[9];
  real_T dv3[9];
  real_T dv4[9];
  real_T R_h[9];
  real_T R_bn[9];
  real_T z[7];                         // '<S11>/Observer'
  real_T TmpSignalConversionAtSFun_j[7];// '<S3>/mass estimator'
  real_T TmpSignalConversionAtSFunct[7];// '<S4>/MATLAB Function1'
  real_T tau[7];
  real_T q[7];
  real_T qddoti_data[7];
  real_T q_data[7];
  real32_T b_varargout_2_Data[14];
  real_T a0[6];
  real_T y[6];
  real_T X_d[6];
  real_T b_I_e[6];
  int8_T msubspace_data[36];
  real_T result_data[4];
  real_T result_data_b[4];
  real_T result_data_j[4];
  char_T cv[29];
  int32_T l_data[7];
  int32_T e_data[7];
  char_T cv1[26];
  real_T CoordinateTransformationConvers[3];
                                // '<S3>/Coordinate Transformation Conversion1'
  real_T xp[14];                       // '<S1>/EKF'
  real_T Pp[196];                      // '<S1>/EKF'
  real32_T MATLABSystem_n[7];          // '<S17>/MATLAB System'
  real_T v[3];
  real_T v_f[3];
  real_T v_a[3];
  real_T v_j[3];
  real_T v_jz[3];
  real_T v_o[3];
  real_T v_n[3];
  char_T cv2[22];
  char_T cv3[20];
  char_T b_i[19];
  char_T cv4[17];
  char_T cv5[16];
  char_T b_o[14];
  char_T cv6[12];
  int32_T rtb_MATLABSystem_size[3];
  int32_T rtb_MATLABSystem_size_n[3];
  int32_T tmp_size[3];
  int32_T rtb_MATLABSystem_size_m[3];
  int32_T sy_size[3];
  char_T cv7[11];
  char_T b_c[9];
  char_T b_md[9];
  char_T b_m3[9];
  char_T b_j[8];
  char_T b_h[8];
  char_T b_c0[8];
  char_T b_ct[8];
  char_T b_p[8];
  char_T b_p5[8];
  char_T b_a[8];
  real_T GetParameter7_o1;             // '<S8>/Get Parameter7'
  real_T GetParameter8_o1;             // '<S8>/Get Parameter8'
  real_T GetParameter7_o1_j;           // '<S7>/Get Parameter7'
  real_T GetParameter8_o1_m;           // '<S7>/Get Parameter8'
  real_T GetParameter9_o1;             // '<S7>/Get Parameter9'
  real_T GetParameter_o1;              // '<S6>/Get Parameter'
  real_T GetParameter1_o1;             // '<S6>/Get Parameter1'
  real_T GetParameter2_o1;             // '<S6>/Get Parameter2'
  real_T GetParameter3_o1;             // '<S6>/Get Parameter3'
  real_T GetParameter4_o1;             // '<S6>/Get Parameter4'
  real_T GetParameter5_o1;             // '<S6>/Get Parameter5'
  real_T GetParameter6_o1;             // '<S6>/Get Parameter6'
  real_T GetParameter7_o1_n;           // '<S6>/Get Parameter7'
  real_T GetParameter8_o1_k;           // '<S6>/Get Parameter8'
  real_T GetParameter9_o1_o;           // '<S6>/Get Parameter9'
  real_T GetParameter10_o1;            // '<S6>/Get Parameter10'
  real_T GetParameter11_o1;            // '<S6>/Get Parameter11'
  real_T GetParameter12_o1;            // '<S6>/Get Parameter12'
  real_T GetParameter13_o1;            // '<S6>/Get Parameter13'
  real_T torque[7];                    // '<S4>/MATLAB Function1'
  real_T MATLABSystem_k[49];           // '<S15>/MATLAB System'
  real_T smax;
  real_T absxk;
  real_T t;
  real_T bid1;
  real_T rtb_MATLABSystem_data;
  real_T rtb_MATLABSystem_data_e;
  real_T tmp_data;
  real_T rtb_MATLABSystem_data_a;
  real_T rtb_LowPassz1_tmp;
  real_T rtb_LowPassz1_tmp_a;
  real_T nb;
  real_T sth;
  real_T a_idx_1;
  real_T a_idx_0;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T tempR_tmp;
  real_T tempR_tmp_i;
  real_T tempR_tmp_l;
  real_T tempR_tmp_o;
  real_T tempR_tmp_o2;
  real_T nb_i;
  real_T vNum;
  real_T pid;
  real_T temp;
  real_T p_idx_1;
  real_T b_idx_0_f;
  real_T b_idx_1_i;
  real_T cth;
  real_T sth_f;
  real_T tempR_tmp_g;
  real_T tempR_tmp_c;
  real_T tempR_tmp_o3;
  real_T tempR_tmp_lm;
  real_T tempR_tmp_m;
  real_T b_mj;
  real_T n;
  real_T k;
  real_T sth_c;
  real_T tempR_tmp_f;
  real_T tempR_tmp_p;
  real_T tempR_tmp_e;
  real_T tempR_tmp_o4;
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment2;// '<S9>/Bus Assignment2' 
  int8_T ipiv[7];
  char_T cv8[7];
  int8_T iv[6];
  char_T b_hh[5];
  char_T b_l[5];
  char_T b_h2[5];
  char_T b_me[5];
  char_T b_mc[5];
  int32_T jAcol;
  int32_T kBcol;
  int32_T c;
  int32_T ix;
  int32_T b_ix;
  int32_T c_ix;
  int32_T ijA;
  int32_T i;
  int32_T b_k;
  int32_T j;
  int32_T t_h;
  int32_T u;
  int32_T m;
  int32_T inner;
  int32_T aoffset;
  int32_T c_i;
  int32_T i_c;
  int32_T unnamed_idx_1;
  int32_T kstr;
  int32_T b_kstr;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  int32_T kstr_k;
  int32_T b_kstr_p;
  int32_T i_p;
  int32_T i_p4;
  int32_T c_a;
  int32_T b_i_j;
  int32_T f;
  int32_T n_e;
  int32_T aoffset_o;
  int32_T i_b;
  int32_T b_j_a;
  int32_T c_i_g;
  int32_T m_e;
  int32_T coffset;
  int32_T boffset;
  int32_T q_size;
  int32_T c_tmp;
  int32_T pid_tmp;
  int32_T q_size_tmp;
  int32_T X_tmp;
  int32_T kstr_f;
  int32_T b_kstr_h;
  int32_T obj_tmp_e;
  int32_T obj_tmp_tmp_c;
  int32_T d;
  int32_T e;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr_a;
  int32_T loop_ub;
  int32_T kstr_d;
  int32_T b_kstr_af;
  int32_T i_pb;
  int32_T i1;
  int32_T X_tmp_m;
  int32_T X_tmp_o;
  int32_T i2;
  int32_T Tinv_tmp;
  int32_T i_n;
  int32_T i3;
  int32_T b_kstr_l;
  int32_T loop_ub_p;
  int32_T newNumel;
  int32_T i_pt;
  int32_T newNumel_f;
  int32_T i_i;
  int32_T newNumel_o;
  int32_T i_k;
  int32_T kstr_i;
  int32_T b_kstr_o;
  int32_T i_m;
  int32_T i_cu;
  int32_T i_f;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_k;
  boolean_T b_varargout_1;
  boolean_T b_bool;
  boolean_T b_bool_p;
  boolean_T b_bool_b;
  boolean_T b_bool_c;
  boolean_T b_bool_n;
  boolean_T b_bool_i;
  boolean_T b_bool_m;
  B_MATLABSystem_left_arm_ctrl__T MATLABSystem_me;// '<S13>/MATLAB System'
  B_MATLABSystem_left_arm_ctrl__T MATLABSystem;// '<S13>/MATLAB System'
} B_left_arm_ctrl_obs_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal_e0h_T obj; // '<S17>/MATLAB System'
  robotics_slmanip_internal__e0_T obj_h;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_0;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_1;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_2;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_3;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_4;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_5;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_6;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_7;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_8;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_9;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_10;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_11;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_12;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_13;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_14;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_15;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_16;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_17;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_18;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_19;// '<S17>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_0_h;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_1_p;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_2_e;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_3_d;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_4_m;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_5_n;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_6_c;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_7_a;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_8_j;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_9_b;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_10_j;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_11_g;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_12_f;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_13_a;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_14_m;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_15_j;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_16_f;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_17_a;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_18_i;// '<S15>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_19_g;// '<S15>/MATLAB System'
  robotics_slmanip_internal_b_e_T obj_o;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_0_h4;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_1_h;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_2_h;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_3_l;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_4_n;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_5_nh;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_6_k;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_7_e;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_8_h;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_9_a;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_10_l;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_11_d;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_12_c;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_13_g;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_14_b;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_15_i;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_16_e;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_17_p;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_18_d;// '<S14>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_19_e;// '<S14>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_f;// '<S8>/Get Parameter7'
  ros_slros_internal_block_GetP_T obj_hh;// '<S8>/Get Parameter8'
  ros_slros_internal_block_GetP_T obj_a;// '<S7>/Get Parameter7'
  ros_slros_internal_block_GetP_T obj_oo;// '<S7>/Get Parameter8'
  ros_slros_internal_block_GetP_T obj_ok;// '<S7>/Get Parameter9'
  ros_slros_internal_block_GetP_T obj_l;// '<S6>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_oi;// '<S6>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_o1;// '<S6>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_k;// '<S6>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_d;// '<S6>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_i;// '<S6>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_fn;// '<S6>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_hn;// '<S6>/Get Parameter7'
  ros_slros_internal_block_GetP_T obj_og;// '<S6>/Get Parameter8'
  ros_slros_internal_block_GetP_T obj_fy;// '<S6>/Get Parameter9'
  ros_slros_internal_block_GetP_T obj_if;// '<S6>/Get Parameter10'
  ros_slros_internal_block_GetP_T obj_g;// '<S6>/Get Parameter11'
  ros_slros_internal_block_GetP_T obj_c;// '<S6>/Get Parameter12'
  ros_slros_internal_block_GetP_T obj_p;// '<S6>/Get Parameter13'
  ros_slros_internal_block_Publ_T obj_b;// '<S28>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_ap;// '<S27>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_d1;// '<S26>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_kp;// '<S22>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_j;// '<S21>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_gx;// '<S30>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_m;// '<S29>/SourceBlock'
  real_T RateTransition_Buffer;        // '<S9>/Rate Transition'
  DW_MATLABSystem_left_arm_ctrl_T MATLABSystem_me;// '<S13>/MATLAB System'
  DW_MATLABSystem_left_arm_ctrl_T MATLABSystem;// '<S13>/MATLAB System'
} DW_left_arm_ctrl_obs_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator1_CSTATE[196];      // '<S1>/Integrator1'
  real_T Integrator_CSTATE[14];        // '<S1>/Integrator'
  real_T Integrator_CSTATE_p[14];      // '<S11>/Integrator'
  real_T LowPassz1_CSTATE[4];          // '<S2>/Low Pass (z1)'
  real_T LowPassz11_CSTATE[4];         // '<S2>/Low Pass (z1)1'
  real_T LowPassz12_CSTATE[4];         // '<S2>/Low Pass (z1)2'
  real_T LowPassz13_CSTATE[4];         // '<S2>/Low Pass (z1)3'
  real_T LowPassz14_CSTATE[4];         // '<S2>/Low Pass (z1)4'
  real_T LowPassz15_CSTATE[4];         // '<S2>/Low Pass (z1)5'
  real_T LowPassz16_CSTATE[4];         // '<S2>/Low Pass (z1)6'
} X_left_arm_ctrl_obs_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator1_CSTATE[196];      // '<S1>/Integrator1'
  real_T Integrator_CSTATE[14];        // '<S1>/Integrator'
  real_T Integrator_CSTATE_p[14];      // '<S11>/Integrator'
  real_T LowPassz1_CSTATE[4];          // '<S2>/Low Pass (z1)'
  real_T LowPassz11_CSTATE[4];         // '<S2>/Low Pass (z1)1'
  real_T LowPassz12_CSTATE[4];         // '<S2>/Low Pass (z1)2'
  real_T LowPassz13_CSTATE[4];         // '<S2>/Low Pass (z1)3'
  real_T LowPassz14_CSTATE[4];         // '<S2>/Low Pass (z1)4'
  real_T LowPassz15_CSTATE[4];         // '<S2>/Low Pass (z1)5'
  real_T LowPassz16_CSTATE[4];         // '<S2>/Low Pass (z1)6'
} XDot_left_arm_ctrl_obs_T;

// State disabled
typedef struct {
  boolean_T Integrator1_CSTATE[196];   // '<S1>/Integrator1'
  boolean_T Integrator_CSTATE[14];     // '<S1>/Integrator'
  boolean_T Integrator_CSTATE_p[14];   // '<S11>/Integrator'
  boolean_T LowPassz1_CSTATE[4];       // '<S2>/Low Pass (z1)'
  boolean_T LowPassz11_CSTATE[4];      // '<S2>/Low Pass (z1)1'
  boolean_T LowPassz12_CSTATE[4];      // '<S2>/Low Pass (z1)2'
  boolean_T LowPassz13_CSTATE[4];      // '<S2>/Low Pass (z1)3'
  boolean_T LowPassz14_CSTATE[4];      // '<S2>/Low Pass (z1)4'
  boolean_T LowPassz15_CSTATE[4];      // '<S2>/Low Pass (z1)5'
  boolean_T LowPassz16_CSTATE[4];      // '<S2>/Low Pass (z1)6'
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
  real_T lowpass_A[5];                 // Variable: lowpass_A
                                          //  Referenced by:
                                          //    '<S2>/Low Pass (z1)'
                                          //    '<S2>/Low Pass (z1)1'
                                          //    '<S2>/Low Pass (z1)2'
                                          //    '<S2>/Low Pass (z1)3'
                                          //    '<S2>/Low Pass (z1)4'
                                          //    '<S2>/Low Pass (z1)5'
                                          //    '<S2>/Low Pass (z1)6'

  real_T lowpass_B[5];                 // Variable: lowpass_B
                                          //  Referenced by:
                                          //    '<S2>/Low Pass (z1)'
                                          //    '<S2>/Low Pass (z1)1'
                                          //    '<S2>/Low Pass (z1)2'
                                          //    '<S2>/Low Pass (z1)3'
                                          //    '<S2>/Low Pass (z1)4'
                                          //    '<S2>/Low Pass (z1)5'
                                          //    '<S2>/Low Pass (z1)6'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                                 //  Referenced by: '<S32>/Out1'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S30>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Out1_Y0_a;// Computed Parameter: Out1_Y0_a
                                                                   //  Referenced by: '<S31>/Out1'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                      //  Referenced by: '<S29>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                                      //  Referenced by: '<S19>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_n;// Computed Parameter: Constant_Value_n
                                                                      //  Referenced by: '<S20>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_e;// Computed Parameter: Constant_Value_e
                                                                      //  Referenced by: '<S23>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_i;// Computed Parameter: Constant_Value_i
                                                                      //  Referenced by: '<S25>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_k;// Computed Parameter: Constant_Value_k
                                                                //  Referenced by: '<S24>/Constant'

  real_T Integrator1_IC[196];          // Expression: eye(14)*0.001
                                          //  Referenced by: '<S1>/Integrator1'

  real_T Integrator_IC;                // Expression: 0
                                          //  Referenced by: '<S1>/Integrator'

  real_T Constant2_Value[60];          // Expression: zeros(6,10)
                                          //  Referenced by: '<S1>/Constant2'

  real_T delta_t_Value;                // Expression: 0.004
                                          //  Referenced by: '<S1>/delta_t'

  real_T Constant2_Value_k[60];        // Expression: zeros(6,10)
                                          //  Referenced by: '<S11>/Constant2'

  real_T Integrator_IC_c[14];       // Expression: [0 0 0 0 0 0 0 0 0 0 0 0 0 0]
                                       //  Referenced by: '<S11>/Integrator'

  uint32_T Constant1_Value;            // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S9>/Constant1'

  uint32_T Constant_Value_d;           // Computed Parameter: Constant_Value_d
                                          //  Referenced by: '<S9>/Constant'

  uint32_T Constant_Value_kl;          // Computed Parameter: Constant_Value_kl
                                          //  Referenced by: '<S5>/Constant'

  uint32_T Constant1_Value_i;          // Computed Parameter: Constant1_Value_i
                                          //  Referenced by: '<S5>/Constant1'

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
  real_T odeY[252];
  real_T odeF[4][252];
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
//  Block '<S13>/Reshape' : Reshape block reduction
//  Block '<S17>/Reshape' : Reshape block reduction
//  Block '<S5>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S5>/Rate Transition4' : Eliminated since input and output rates are identical
//  Block '<S9>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S9>/Rate Transition4' : Eliminated since input and output rates are identical
//  Block '<S10>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S10>/Rate Transition3' : Eliminated since input and output rates are identical
//  Block '<S33>/Reshape' : Reshape block reduction


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
//  '<S1>'   : 'left_arm_ctrl_obs/EKF'
//  '<S2>'   : 'left_arm_ctrl_obs/Low Pass Filter'
//  '<S3>'   : 'left_arm_ctrl_obs/Mass Estimator'
//  '<S4>'   : 'left_arm_ctrl_obs/PD+ Control1'
//  '<S5>'   : 'left_arm_ctrl_obs/ROS EKF est'
//  '<S6>'   : 'left_arm_ctrl_obs/ROS Params PD'
//  '<S7>'   : 'left_arm_ctrl_obs/ROS Params SMO'
//  '<S8>'   : 'left_arm_ctrl_obs/ROS Params SMO1'
//  '<S9>'   : 'left_arm_ctrl_obs/ROS Publishers'
//  '<S10>'  : 'left_arm_ctrl_obs/ROS Subscribers'
//  '<S11>'  : 'left_arm_ctrl_obs/SMO2'
//  '<S12>'  : 'left_arm_ctrl_obs/EKF/EKF'
//  '<S13>'  : 'left_arm_ctrl_obs/EKF/NominalModel'
//  '<S14>'  : 'left_arm_ctrl_obs/Mass Estimator/Get Transform'
//  '<S15>'  : 'left_arm_ctrl_obs/Mass Estimator/Joint Space Mass Matrix'
//  '<S16>'  : 'left_arm_ctrl_obs/Mass Estimator/mass estimator'
//  '<S17>'  : 'left_arm_ctrl_obs/PD+ Control1/Gravity Torque'
//  '<S18>'  : 'left_arm_ctrl_obs/PD+ Control1/MATLAB Function1'
//  '<S19>'  : 'left_arm_ctrl_obs/ROS EKF est/Blank Message1'
//  '<S20>'  : 'left_arm_ctrl_obs/ROS EKF est/Blank Message3'
//  '<S21>'  : 'left_arm_ctrl_obs/ROS EKF est/Publish1'
//  '<S22>'  : 'left_arm_ctrl_obs/ROS EKF est/Publish3'
//  '<S23>'  : 'left_arm_ctrl_obs/ROS Publishers/Blank Message1'
//  '<S24>'  : 'left_arm_ctrl_obs/ROS Publishers/Blank Message2'
//  '<S25>'  : 'left_arm_ctrl_obs/ROS Publishers/Blank Message3'
//  '<S26>'  : 'left_arm_ctrl_obs/ROS Publishers/Publish1'
//  '<S27>'  : 'left_arm_ctrl_obs/ROS Publishers/Publish2'
//  '<S28>'  : 'left_arm_ctrl_obs/ROS Publishers/Publish3'
//  '<S29>'  : 'left_arm_ctrl_obs/ROS Subscribers/Subscribe'
//  '<S30>'  : 'left_arm_ctrl_obs/ROS Subscribers/Subscribe1'
//  '<S31>'  : 'left_arm_ctrl_obs/ROS Subscribers/Subscribe/Enabled Subsystem'
//  '<S32>'  : 'left_arm_ctrl_obs/ROS Subscribers/Subscribe1/Enabled Subsystem'
//  '<S33>'  : 'left_arm_ctrl_obs/SMO2/Forward Dynamics'
//  '<S34>'  : 'left_arm_ctrl_obs/SMO2/Observer'

#endif                                 // RTW_HEADER_left_arm_ctrl_obs_h_

//
// File trailer for generated code.
//
// [EOF]
//
