//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_ctrl_obs.h
//
// Code generated for Simulink model 'left_arm_ctrl_obs'.
//
// Model version                  : 1.244
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Fri Jul 24 22:50:18 2020
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
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray In1;// '<S45>/In1'
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray In1_e;// '<S46>/In1'
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray BusAssignment1;// '<S6>/Bus Assignment1' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray b_varargout_2;
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray b_varargout_2_m;
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension
    b_varargout_2_Layout_Dim[16];
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
  real_T dv[16];
  real_T TJ_p[16];
  real_T obj[16];
  real_T T_c[16];
  real_T Tinv[16];
  real_T TJ_f[16];
  real_T obj_g[16];
  real_T T_g[16];
  real_T dv1[16];
  real_T c_f1[16];
  real_T a_m[16];
  real_T b_n[16];
  real_T a_p[16];
  real_T T_l[16];
  real_T dv2[16];
  real_T TJ_j[16];
  real_T obj_d[16];
  real_T TJ_g[16];
  real_T obj_l[16];
  real_T TJ_d[16];
  real_T obj_dy[16];
  e_cell_wrap_left_arm_ctrl_obs_T expl_temp;
  real_T xp_est[14];                   // '<S11>/Observer'
  real_T R_l[9];
  real_T R_o[9];
  real_T R_b[9];
  real_T tempR_n[9];
  real_T dv3[9];
  real_T R_bs[9];
  real_T tempR_l[9];
  real_T R_h[9];
  real_T R_bn[9];
  real_T dv4[9];
  real_T dv5[9];
  real_T dv6[9];
  real_T R_d[9];
  real_T tempR_e[9];
  real_T dv7[9];
  real_T dv8[9];
  real_T R_bj[9];
  real_T tempR_j[9];
  real_T R_f[9];
  real_T R_a[9];
  real_T V[9];
  real_T b_I_j[9];
  real_T b_U[9];
  real_T A[9];
  real_T A_j[9];
  real_T R_o4[9];
  real_T tempR_ny[9];
  real_T R_i[9];
  real_T tempR_o[9];
  real_T R_n[9];
  real_T tempR_m[9];
  real_T z[7];                         // '<S11>/Observer'
  real_T TmpSignalConversionAtSFun_k[7];// '<S2>/mass estimator'
  real_T TmpSignalConversionAtSFunct[7];// '<S3>/MATLAB Function1'
  real_T torque[7];
  real_T tau[7];
  real_T q[7];
  real_T qddoti_data[7];
  real_T q_data[7];
  real_T q_data_c[7];
  real_T q_data_m[7];
  real32_T b_varargout_2_Data[14];
  real_T a0[6];
  real_T y[6];
  real_T X_m[6];
  real_T b_I_ja[6];
  real_T a0_h[6];
  real_T y_c[6];
  real_T vJ[6];
  real_T b_I_ct[6];
  real_T b_I_p[6];
  real_T R_p[6];
  int32_T nonFixedIndices_data[10];
  int32_T ii_data[10];
  int8_T msubspace_data[36];
  real_T out[4];
  real_T result_data[4];
  real_T result_data_a[4];
  real_T result_data_e[4];
  real_T result_data_ax[4];
  char_T cv[29];
  int32_T l_data[7];
  int32_T e_data[7];
  char_T cv1[26];
  real_T eulShaped[3];
  real_T v[3];
  real_T v_a[3];
  real_T v_i[3];
  real_T v_l[3];
  real_T v_o[3];
  real_T v_o2[3];
  real_T vspecial_data[3];
  real_T s[3];
  real_T e[3];
  real_T work[3];
  real_T v_ip[3];
  real_T v_f[3];
  real_T v_iz[3];
  real_T v_ff[3];
  real_T v_g[3];
  char_T cv2[20];
  char_T b_c[19];
  char_T cv3[16];
  creal_T v_o3;
  creal_T u;
  creal_T u_l;
  creal_T dc;
  char_T b_m[14];
  char_T cv4[12];
  int32_T tmp_size[3];
  int32_T rtb_MATLABSystem_d_size[3];
  int32_T sy_size[3];
  int32_T rtb_MATLABSystem_d_size_m[3];
  int32_T rtb_MATLABSystem_d_size_c[3];
  int32_T rtb_MATLABSystem_d_size_f[3];
  int32_T sy_size_p[3];
  int32_T rtb_MATLABSystem_d_size_e[3];
  int32_T rtb_MATLABSystem_d_size_o[3];
  char_T cv5[11];
  boolean_T mask[10];
  char_T b_h[9];
  char_T b_l[9];
  char_T b_h2[9];
  char_T b_me[8];
  char_T b_mc[8];
  char_T b_h3[8];
  char_T b_cs[8];
  char_T b_k[8];
  char_T b_p[8];
  char_T b_px[8];
  char_T cv6[8];
  real_T GetParameter7_o1;             // '<S5>/Get Parameter7'
  real_T GetParameter8_o1;             // '<S5>/Get Parameter8'
  real_T GetParameter9_o1;             // '<S5>/Get Parameter9'
  real_T GetParameter_o1;              // '<S4>/Get Parameter'
  real_T GetParameter1_o1;             // '<S4>/Get Parameter1'
  real_T GetParameter2_o1;             // '<S4>/Get Parameter2'
  real_T GetParameter3_o1;             // '<S4>/Get Parameter3'
  real_T GetParameter4_o1;             // '<S4>/Get Parameter4'
  real_T GetParameter5_o1;             // '<S4>/Get Parameter5'
  real_T GetParameter6_o1;             // '<S4>/Get Parameter6'
  real_T GetParameter7_o1_n;           // '<S4>/Get Parameter7'
  real_T GetParameter8_o1_k;           // '<S4>/Get Parameter8'
  real_T GetParameter9_o1_o;           // '<S4>/Get Parameter9'
  real_T GetParameter10_o1;            // '<S4>/Get Parameter10'
  real_T GetParameter11_o1;            // '<S4>/Get Parameter11'
  real_T GetParameter12_o1;            // '<S4>/Get Parameter12'
  real_T GetParameter13_o1;            // '<S4>/Get Parameter13'
  real_T MATLABSystem[49];             // '<S13>/MATLAB System'
  real_T CoordinateTransformationConvers[3];
                                // '<S2>/Coordinate Transformation Conversion1'
  real32_T MATLABSystem_n[7];          // '<S15>/MATLAB System'
  real_T bid1;
  real_T vNum;
  real_T j;
  real_T RateTransition1;              // '<S9>/Rate Transition1'
  real_T RateTransition;               // '<S9>/Rate Transition'
  real_T RateTransition2;              // '<S9>/Rate Transition2'
  real_T RateTransition3;              // '<S9>/Rate Transition3'
  real_T RateTransition1_f;            // '<S8>/Rate Transition1'
  real_T RateTransition_d;             // '<S8>/Rate Transition'
  real_T RateTransition2_h;            // '<S8>/Rate Transition2'
  real_T RateTransition1_a;            // '<S7>/Rate Transition1'
  real_T RateTransition_c;             // '<S7>/Rate Transition'
  real_T tmp_data;
  real_T rtb_MATLABSystem_d_data;
  real_T rtb_MATLABSystem_d_data_p;
  real_T rtb_MATLABSystem_d_data_a;
  real_T rtb_CoordinateTransformationC_j;
  real_T rtb_CoordinateTransformation_ek;
  real_T rtb_CoordinateTransformationC_o;
  real_T rtb_CoordinateTransformation_bb;
  real_T rtb_CoordinateTransformationC_a;
  real_T nb;
  real_T sth;
  real_T a_idx_1;
  real_T a_idx_0;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T tempR_tmp;
  real_T tempR_tmp_g;
  real_T tempR_tmp_e;
  real_T tempR_tmp_f;
  real_T tempR_tmp_h;
  real_T nb_e;
  real_T a_idx_1_c;
  real_T a_idx_0_a;
  real_T b_idx_0_d;
  real_T b_idx_1_a;
  real_T cth;
  real_T sth_p;
  real_T tempR_tmp_m;
  real_T tempR_tmp_o;
  real_T tempR_tmp_n;
  real_T tempR_tmp_l;
  real_T tempR_tmp_p;
  real_T b_pt;
  real_T nb_f;
  real_T vNum_i;
  real_T pid;
  real_T temp;
  real_T p_idx_1;
  real_T b_idx_0_o;
  real_T b_idx_1_k;
  real_T n;
  real_T k;
  real_T sth_i;
  real_T tempR_tmp_oc;
  real_T tempR_tmp_m4;
  real_T tempR_tmp_c;
  real_T tempR_tmp_fb;
  real_T nb_h;
  real_T vNum_m;
  real_T pid_a;
  real_T temp_k;
  real_T p_idx_1_p;
  real_T b_idx_0_b;
  real_T b_idx_1_c;
  real_T a_n;
  real_T q_i;
  real_T nrm;
  real_T rt;
  real_T ztest;
  real_T smm1;
  real_T emm1;
  real_T sqds;
  real_T b_my;
  real_T unusedU2;
  real_T d_sn;
  real_T roe;
  real_T absa;
  real_T absb;
  real_T scale;
  real_T ads;
  real_T bds;
  real_T cth_j;
  real_T sth_e;
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment1_p;// '<S9>/Bus Assignment1' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment2_a;// '<S9>/Bus Assignment2' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment3_j;// '<S9>/Bus Assignment3' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment4;// '<S9>/Bus Assignment4' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment2_m;// '<S6>/Bus Assignment2' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment1_j;// '<S8>/Bus Assignment1' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment2_h;// '<S8>/Bus Assignment2' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment3_o;// '<S8>/Bus Assignment3' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment1_f;// '<S7>/Bus Assignment1' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment2;// '<S7>/Bus Assignment2' 
  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 BusAssignment3_b;// '<S7>/Bus Assignment3' 
  char_T b_mv[5];
  char_T b_m0[5];
  char_T b_j[5];
  char_T b_f[5];
  char_T b_a[5];
  char_T b_g[5];
  char_T b_nr[5];
  int32_T n_d;
  int32_T iend;
  int32_T j_n;
  int32_T i;
  int32_T loop_ub;
  int32_T u1;
  int32_T i_c;
  int32_T b_k_f;
  int32_T j_p;
  int32_T t;
  int32_T u_p;
  int32_T m;
  int32_T inner;
  int32_T aoffset;
  int32_T i_n;
  int32_T unnamed_idx_1;
  int32_T p_tmp;
  int32_T kstr;
  int32_T b_kstr;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  int32_T kstr_k;
  int32_T b_kstr_n;
  int32_T b_k_o;
  int32_T m_g;
  int32_T inner_c;
  int32_T aoffset_c;
  int32_T c_i;
  int32_T i_m;
  int32_T q_size;
  int32_T unnamed_idx_1_j;
  int32_T loop_ub_tmp;
  int32_T p_tmp_k;
  int32_T o_tmp;
  int32_T kstr_m;
  int32_T b_kstr_p;
  int32_T obj_tmp_d;
  int32_T obj_tmp_tmp_g;
  int32_T i_c3;
  int32_T i_cx;
  int32_T b_i;
  int32_T cb;
  int32_T idx;
  int32_T n_i;
  int32_T aoffset_d;
  int32_T i_g;
  int32_T b_j_l;
  int32_T c_i_f;
  int32_T nm1d2;
  int32_T m_d;
  int32_T coffset;
  int32_T boffset;
  int32_T q_size_j;
  int32_T unnamed_idx_1_i;
  int32_T pid_tmp;
  int32_T q_size_tmp;
  int32_T i1;
  int32_T i2;
  int32_T X_tmp;
  int32_T X_tmp_h;
  int32_T d;
  int32_T e_n;
  int32_T ntilecols;
  int32_T b_jtilecol;
  int32_T b_kstr_o;
  int32_T loop_ub_c;
  int32_T kstr_b;
  int32_T b_kstr_e;
  int32_T c;
  int32_T b_i_d;
  int32_T f;
  int32_T n_ik;
  int32_T aoffset_g;
  int32_T i_nn;
  int32_T b_j_l0;
  int32_T c_i_c;
  int32_T m_n;
  int32_T coffset_p;
  int32_T boffset_d;
  int32_T q_size_o;
  int32_T c_tmp;
  int32_T pid_tmp_j;
  int32_T q_size_tmp_c;
  int32_T X_tmp_hp;
  int32_T i3;
  int32_T Tinv_tmp;
  int32_T b_k_d;
  int32_T loop_ub_c5;
  int32_T loop_ub_tmp_p;
  int32_T b_I_tmp;
  int32_T b_I_tmp_p;
  int32_T q_a;
  int32_T qp1;
  int32_T qq;
  int32_T qjj;
  int32_T m_o;
  int32_T b_jw;
  int32_T qq_tmp;
  int32_T newNumel;
  int32_T i_p;
  int32_T newNumel_o;
  int32_T i_l;
  int32_T newNumel_k;
  int32_T i_j;
  int32_T newNumel_f;
  int32_T i_cm;
  int32_T kstr_f;
  int32_T b_kstr_nl;
  int32_T i_i;
  int32_T i_l4;
  int32_T i_i3;
  int32_T i_k;
  uint32_T b_varargout_2_Data_SL_Info_Curr;
  uint32_T b_varargout_2_Data_SL_Info_Rece;
  uint32_T b_varargout_2_Layout_DataOffset;
  uint32_T b_varargout_2_Layout_Dim_SL_Inf;
  uint32_T b_varargout_2_Layout_Dim_SL_I_e;
  boolean_T b_varargout_1;
  boolean_T b_bool;
  boolean_T b_bool_e;
  boolean_T b_bool_b;
  boolean_T b_bool_a;
  boolean_T b_bool_i;
  boolean_T b_bool_f;
  boolean_T b_bool_j;
  boolean_T b_bool_o;
  boolean_T e_f;
  boolean_T p;
  boolean_T rEQ0;
  boolean_T apply_transform;
} B_left_arm_ctrl_obs_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slmanip_internal_e0h_T obj; // '<S47>/MATLAB System'
  robotics_slmanip_internal__e0_T obj_f;// '<S15>/MATLAB System'
  robotics_slmanip_internal_b_e_T obj_h;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_0;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_1;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_2;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_3;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_4;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_5;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_6;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_7;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_8;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_9;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_10;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_11;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_12;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_13;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_14;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_15;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_16;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_17;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_18;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_19;// '<S47>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_0_f;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_1_o;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_2_l;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_3_p;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_4_f;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_5_l;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_6_m;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_7_e;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_8_n;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_9_f;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_10_a;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_11_n;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_12_i;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_13_a;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_14_p;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_15_i;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_16_n;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_17_e;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_18_k;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_19_h;// '<S15>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_0_h;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_1_p;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_2_e;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_3_d;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_4_m;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_5_n;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_6_c;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_7_a;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_8_j;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_9_b;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_10_j;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_11_g;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_12_f;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_13_ak;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_14_m;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_15_j;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_16_f;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_17_a;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_18_i;// '<S13>/MATLAB System'
  j_robotics_manip_internal_R_e_T gobj_19_g;// '<S13>/MATLAB System'
  robotics_slmanip_internal_blo_T obj_o;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_0_h4;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_1_h;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_2_h;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_3_l;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_4_n;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_5_nh;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_6_k;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_7_eo;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_8_h;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_9_a;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_10_l;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_11_d;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_12_c;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_13_g;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_14_b;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_15_ih;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_16_e;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_17_p;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_18_d;// '<S12>/MATLAB System'
  j_robotics_manip_internal_Rig_T gobj_19_e;// '<S12>/MATLAB System'
  ros_slros_internal_block_GetP_T obj_a;// '<S5>/Get Parameter7'
  ros_slros_internal_block_GetP_T obj_oo;// '<S5>/Get Parameter8'
  ros_slros_internal_block_GetP_T obj_ok;// '<S5>/Get Parameter9'
  ros_slros_internal_block_GetP_T obj_l;// '<S4>/Get Parameter'
  ros_slros_internal_block_GetP_T obj_oi;// '<S4>/Get Parameter1'
  ros_slros_internal_block_GetP_T obj_o1;// '<S4>/Get Parameter2'
  ros_slros_internal_block_GetP_T obj_k;// '<S4>/Get Parameter3'
  ros_slros_internal_block_GetP_T obj_d;// '<S4>/Get Parameter4'
  ros_slros_internal_block_GetP_T obj_i;// '<S4>/Get Parameter5'
  ros_slros_internal_block_GetP_T obj_fn;// '<S4>/Get Parameter6'
  ros_slros_internal_block_GetP_T obj_hn;// '<S4>/Get Parameter7'
  ros_slros_internal_block_GetP_T obj_og;// '<S4>/Get Parameter8'
  ros_slros_internal_block_GetP_T obj_fy;// '<S4>/Get Parameter9'
  ros_slros_internal_block_GetP_T obj_if;// '<S4>/Get Parameter10'
  ros_slros_internal_block_GetP_T obj_g;// '<S4>/Get Parameter11'
  ros_slros_internal_block_GetP_T obj_c;// '<S4>/Get Parameter12'
  ros_slros_internal_block_GetP_T obj_p;// '<S4>/Get Parameter13'
  ros_slros_internal_block_Publ_T obj_b;// '<S42>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_lq;// '<S41>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_cw;// '<S40>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_j;// '<S39>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_cy;// '<S34>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_oa;// '<S33>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_oz;// '<S32>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_n;// '<S28>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_oe;// '<S27>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_dt;// '<S26>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_ba;// '<S22>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_ap;// '<S21>/SinkBlock'
  ros_slros_internal_block_Publ_T obj_d1;// '<S20>/SinkBlock'
  ros_slros_internal_block_Subs_T obj_gx;// '<S44>/SourceBlock'
  ros_slros_internal_block_Subs_T obj_m;// '<S43>/SourceBlock'
  real_T RateTransition1_Buffer;       // '<S9>/Rate Transition1'
  real_T RateTransition_Buffer;        // '<S9>/Rate Transition'
  real_T RateTransition2_Buffer;       // '<S9>/Rate Transition2'
  real_T RateTransition3_Buffer;       // '<S9>/Rate Transition3'
  real_T RateTransition1_Buffer_c;     // '<S8>/Rate Transition1'
  real_T RateTransition_Buffer_n;      // '<S8>/Rate Transition'
  real_T RateTransition2_Buffer_d;     // '<S8>/Rate Transition2'
  real_T RateTransition1_Buffer_n;     // '<S7>/Rate Transition1'
  real_T RateTransition_Buffer_a;      // '<S7>/Rate Transition'
  real_T RateTransition2_Buffer_j;     // '<S7>/Rate Transition2'
  real_T RateTransition_Buffer_k;      // '<S6>/Rate Transition'
} DW_left_arm_ctrl_obs_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator_CSTATE[14];        // '<S11>/Integrator'
  real_T LowPassz1_CSTATE[5];          // '<S1>/Low Pass (z1)'
  real_T LowPassz2_CSTATE[5];          // '<S1>/Low Pass (z2)'
  real_T LowPassz21_CSTATE[5];         // '<S1>/Low Pass (z2)1'
  real_T LowPassz22_CSTATE[5];         // '<S1>/Low Pass (z2)2'
  real_T LowPassz23_CSTATE[5];         // '<S1>/Low Pass (z2)3'
  real_T LowPassz24_CSTATE[5];         // '<S1>/Low Pass (z2)4'
  real_T LowPassz25_CSTATE[5];         // '<S1>/Low Pass (z2)5'
} X_left_arm_ctrl_obs_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator_CSTATE[14];        // '<S11>/Integrator'
  real_T LowPassz1_CSTATE[5];          // '<S1>/Low Pass (z1)'
  real_T LowPassz2_CSTATE[5];          // '<S1>/Low Pass (z2)'
  real_T LowPassz21_CSTATE[5];         // '<S1>/Low Pass (z2)1'
  real_T LowPassz22_CSTATE[5];         // '<S1>/Low Pass (z2)2'
  real_T LowPassz23_CSTATE[5];         // '<S1>/Low Pass (z2)3'
  real_T LowPassz24_CSTATE[5];         // '<S1>/Low Pass (z2)4'
  real_T LowPassz25_CSTATE[5];         // '<S1>/Low Pass (z2)5'
} XDot_left_arm_ctrl_obs_T;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE[14];     // '<S11>/Integrator'
  boolean_T LowPassz1_CSTATE[5];       // '<S1>/Low Pass (z1)'
  boolean_T LowPassz2_CSTATE[5];       // '<S1>/Low Pass (z2)'
  boolean_T LowPassz21_CSTATE[5];      // '<S1>/Low Pass (z2)1'
  boolean_T LowPassz22_CSTATE[5];      // '<S1>/Low Pass (z2)2'
  boolean_T LowPassz23_CSTATE[5];      // '<S1>/Low Pass (z2)3'
  boolean_T LowPassz24_CSTATE[5];      // '<S1>/Low Pass (z2)4'
  boolean_T LowPassz25_CSTATE[5];      // '<S1>/Low Pass (z2)5'
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
                                                                 //  Referenced by: '<S46>/Out1'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S44>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Out1_Y0_a;// Computed Parameter: Out1_Y0_a
                                                                   //  Referenced by: '<S45>/Out1'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                      //  Referenced by: '<S43>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_i;// Computed Parameter: Constant_Value_i
                                                                      //  Referenced by: '<S19>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray Constant_Value_e;// Computed Parameter: Constant_Value_e
                                                                      //  Referenced by: '<S17>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_j;// Computed Parameter: Constant_Value_j
                                                                //  Referenced by: '<S35>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_f;// Computed Parameter: Constant_Value_f
                                                                //  Referenced by: '<S36>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_n;// Computed Parameter: Constant_Value_n
                                                                //  Referenced by: '<S37>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_g;// Computed Parameter: Constant_Value_g
                                                                //  Referenced by: '<S38>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_f4;// Computed Parameter: Constant_Value_f4
                                                                 //  Referenced by: '<S29>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_a;// Computed Parameter: Constant_Value_a
                                                                //  Referenced by: '<S30>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                                //  Referenced by: '<S31>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                                //  Referenced by: '<S23>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_k;// Computed Parameter: Constant_Value_k
                                                                //  Referenced by: '<S24>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_kl;// Computed Parameter: Constant_Value_kl
                                                                 //  Referenced by: '<S25>/Constant'

  SL_Bus_left_arm_ctrl_obs_std_msgs_Float64 Constant_Value_kc;// Computed Parameter: Constant_Value_kc
                                                                 //  Referenced by: '<S18>/Constant'

  real_T Constant2_Value[60];          // Expression: zeros(6,10)
                                          //  Referenced by: '<S11>/Constant2'

  real_T Integrator_IC[14];         // Expression: [0 0 0 0 0 0 0 0 0 0 0 0 0 0]
                                       //  Referenced by: '<S11>/Integrator'

  real_T LowPassz1_A[5];               // Computed Parameter: LowPassz1_A
                                          //  Referenced by: '<S1>/Low Pass (z1)'

  real_T LowPassz1_C[5];               // Computed Parameter: LowPassz1_C
                                          //  Referenced by: '<S1>/Low Pass (z1)'

  real_T LowPassz2_A[5];               // Computed Parameter: LowPassz2_A
                                          //  Referenced by: '<S1>/Low Pass (z2)'

  real_T LowPassz2_C[5];               // Computed Parameter: LowPassz2_C
                                          //  Referenced by: '<S1>/Low Pass (z2)'

  real_T LowPassz21_A[5];              // Computed Parameter: LowPassz21_A
                                          //  Referenced by: '<S1>/Low Pass (z2)1'

  real_T LowPassz21_C[5];              // Computed Parameter: LowPassz21_C
                                          //  Referenced by: '<S1>/Low Pass (z2)1'

  real_T LowPassz22_A[5];              // Computed Parameter: LowPassz22_A
                                          //  Referenced by: '<S1>/Low Pass (z2)2'

  real_T LowPassz22_C[5];              // Computed Parameter: LowPassz22_C
                                          //  Referenced by: '<S1>/Low Pass (z2)2'

  real_T LowPassz23_A[5];              // Computed Parameter: LowPassz23_A
                                          //  Referenced by: '<S1>/Low Pass (z2)3'

  real_T LowPassz23_C[5];              // Computed Parameter: LowPassz23_C
                                          //  Referenced by: '<S1>/Low Pass (z2)3'

  real_T LowPassz24_A[5];              // Computed Parameter: LowPassz24_A
                                          //  Referenced by: '<S1>/Low Pass (z2)4'

  real_T LowPassz24_C[5];              // Computed Parameter: LowPassz24_C
                                          //  Referenced by: '<S1>/Low Pass (z2)4'

  real_T LowPassz25_A[5];              // Computed Parameter: LowPassz25_A
                                          //  Referenced by: '<S1>/Low Pass (z2)5'

  real_T LowPassz25_C[5];              // Computed Parameter: LowPassz25_C
                                          //  Referenced by: '<S1>/Low Pass (z2)5'

  uint32_T Constant_Value_d;           // Computed Parameter: Constant_Value_d
                                          //  Referenced by: '<S6>/Constant'

  uint32_T Constant1_Value;            // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S6>/Constant1'

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
//  Block '<S15>/Reshape' : Reshape block reduction
//  Block '<S6>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S6>/Rate Transition4' : Eliminated since input and output rates are identical
//  Block '<S10>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S10>/Rate Transition3' : Eliminated since input and output rates are identical
//  Block '<S47>/Reshape' : Reshape block reduction


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
//  '<S1>'   : 'left_arm_ctrl_obs/Low Pass Filter'
//  '<S2>'   : 'left_arm_ctrl_obs/Mass Estimator'
//  '<S3>'   : 'left_arm_ctrl_obs/PD+ Control1'
//  '<S4>'   : 'left_arm_ctrl_obs/ROS Params PD'
//  '<S5>'   : 'left_arm_ctrl_obs/ROS Params SMO'
//  '<S6>'   : 'left_arm_ctrl_obs/ROS Publishers'
//  '<S7>'   : 'left_arm_ctrl_obs/ROS Publishers1'
//  '<S8>'   : 'left_arm_ctrl_obs/ROS Publishers2'
//  '<S9>'   : 'left_arm_ctrl_obs/ROS Publishers3'
//  '<S10>'  : 'left_arm_ctrl_obs/ROS Subscribers'
//  '<S11>'  : 'left_arm_ctrl_obs/SMO2'
//  '<S12>'  : 'left_arm_ctrl_obs/Mass Estimator/Get Transform'
//  '<S13>'  : 'left_arm_ctrl_obs/Mass Estimator/Joint Space Mass Matrix'
//  '<S14>'  : 'left_arm_ctrl_obs/Mass Estimator/mass estimator'
//  '<S15>'  : 'left_arm_ctrl_obs/PD+ Control1/Gravity Torque'
//  '<S16>'  : 'left_arm_ctrl_obs/PD+ Control1/MATLAB Function1'
//  '<S17>'  : 'left_arm_ctrl_obs/ROS Publishers/Blank Message1'
//  '<S18>'  : 'left_arm_ctrl_obs/ROS Publishers/Blank Message2'
//  '<S19>'  : 'left_arm_ctrl_obs/ROS Publishers/Blank Message3'
//  '<S20>'  : 'left_arm_ctrl_obs/ROS Publishers/Publish1'
//  '<S21>'  : 'left_arm_ctrl_obs/ROS Publishers/Publish2'
//  '<S22>'  : 'left_arm_ctrl_obs/ROS Publishers/Publish3'
//  '<S23>'  : 'left_arm_ctrl_obs/ROS Publishers1/Blank Message1'
//  '<S24>'  : 'left_arm_ctrl_obs/ROS Publishers1/Blank Message2'
//  '<S25>'  : 'left_arm_ctrl_obs/ROS Publishers1/Blank Message3'
//  '<S26>'  : 'left_arm_ctrl_obs/ROS Publishers1/Publish1'
//  '<S27>'  : 'left_arm_ctrl_obs/ROS Publishers1/Publish2'
//  '<S28>'  : 'left_arm_ctrl_obs/ROS Publishers1/Publish3'
//  '<S29>'  : 'left_arm_ctrl_obs/ROS Publishers2/Blank Message1'
//  '<S30>'  : 'left_arm_ctrl_obs/ROS Publishers2/Blank Message2'
//  '<S31>'  : 'left_arm_ctrl_obs/ROS Publishers2/Blank Message3'
//  '<S32>'  : 'left_arm_ctrl_obs/ROS Publishers2/Publish1'
//  '<S33>'  : 'left_arm_ctrl_obs/ROS Publishers2/Publish2'
//  '<S34>'  : 'left_arm_ctrl_obs/ROS Publishers2/Publish3'
//  '<S35>'  : 'left_arm_ctrl_obs/ROS Publishers3/Blank Message1'
//  '<S36>'  : 'left_arm_ctrl_obs/ROS Publishers3/Blank Message2'
//  '<S37>'  : 'left_arm_ctrl_obs/ROS Publishers3/Blank Message3'
//  '<S38>'  : 'left_arm_ctrl_obs/ROS Publishers3/Blank Message4'
//  '<S39>'  : 'left_arm_ctrl_obs/ROS Publishers3/Publish1'
//  '<S40>'  : 'left_arm_ctrl_obs/ROS Publishers3/Publish2'
//  '<S41>'  : 'left_arm_ctrl_obs/ROS Publishers3/Publish3'
//  '<S42>'  : 'left_arm_ctrl_obs/ROS Publishers3/Publish4'
//  '<S43>'  : 'left_arm_ctrl_obs/ROS Subscribers/Subscribe'
//  '<S44>'  : 'left_arm_ctrl_obs/ROS Subscribers/Subscribe1'
//  '<S45>'  : 'left_arm_ctrl_obs/ROS Subscribers/Subscribe/Enabled Subsystem'
//  '<S46>'  : 'left_arm_ctrl_obs/ROS Subscribers/Subscribe1/Enabled Subsystem'
//  '<S47>'  : 'left_arm_ctrl_obs/SMO2/Forward Dynamics'
//  '<S48>'  : 'left_arm_ctrl_obs/SMO2/Observer'

#endif                                 // RTW_HEADER_left_arm_ctrl_obs_h_

//
// File trailer for generated code.
//
// [EOF]
//
