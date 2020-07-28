//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_ctrl_obs.cpp
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
#include "left_arm_ctrl_obs.h"
#include "left_arm_ctrl_obs_private.h"

// Block signals (default storage)
B_left_arm_ctrl_obs_T left_arm_ctrl_obs_B;

// Continuous states
X_left_arm_ctrl_obs_T left_arm_ctrl_obs_X;

// Block states (default storage)
DW_left_arm_ctrl_obs_T left_arm_ctrl_obs_DW;

// Real-time model
RT_MODEL_left_arm_ctrl_obs_T left_arm_ctrl_obs_M_ = RT_MODEL_left_arm_ctrl_obs_T
  ();
RT_MODEL_left_arm_ctrl_obs_T *const left_arm_ctrl_obs_M = &left_arm_ctrl_obs_M_;

// Forward declaration for local functions
static void left_arm_ctrl_ob_emxInit_char_T(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions, B_MATLABSystem_left_arm_ctrl__T *localB);
static void left_arm_ctrl_ob_emxInit_real_T(emxArray_real_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions, B_MATLABSystem_left_arm_ctrl__T *localB);
static void le_emxInitStruct_rigidBodyJoint(rigidBodyJoint_left_arm_ctrl__T
  *pStruct, B_MATLABSystem_left_arm_ctrl__T *localB);
static void emxInitStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_left_arm_ctrl__T *localB);
static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_left_arm_ctrl__T *localB);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct, B_MATLABSystem_left_arm_ctrl__T *localB);
static void emxInitStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_left_arm_ctrl__T *localB);
static void left_a_emxEnsureCapacity_char_T(emxArray_char_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel, B_MATLABSystem_left_arm_ctrl__T *localB);
static void left_arm_ctrl_ob_emxFree_char_T(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray);
static void left_a_emxEnsureCapacity_real_T(emxArray_real_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel, B_MATLABSystem_left_arm_ctrl__T *localB);
static j_robotics_manip_internal_Rig_T *left_arm_ct_RigidBody_RigidBody
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static j_robotics_manip_internal_Rig_T *left_arm__RigidBody_RigidBody_m
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static j_robotics_manip_internal_Rig_T *left_arm_RigidBody_RigidBody_my
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static j_robotics_manip_internal_Rig_T *left_ar_RigidBody_RigidBody_mye
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static j_robotics_manip_internal_Rig_T *left_a_RigidBody_RigidBody_myeu
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static j_robotics_manip_internal_Rig_T *left__RigidBody_RigidBody_myeu0
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static j_robotics_manip_internal_Rig_T *left_RigidBody_RigidBody_myeu0z
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static j_robotics_manip_internal_Rig_T *lef_RigidBody_RigidBody_myeu0zw
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static j_robotics_manip_internal_Rig_T *le_RigidBody_RigidBody_myeu0zwr
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static j_robotics_manip_internal_Rig_T *l_RigidBody_RigidBody_myeu0zwrs
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static i_robotics_manip_internal_Rig_T *RigidBody_RigidBody_myeu0zwrsj
  (i_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB);
static k_robotics_manip_internal_Rig_T *lef_RigidBodyTree_RigidBodyTree
  (k_robotics_manip_internal_Rig_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9,
   B_MATLABSystem_left_arm_ctrl__T *localB);
static void left_arm_ct_emxInit_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions, B_MATLABSystem_left_arm_ctrl__T *localB);
static void l_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel, B_MATLABSystem_left_arm_ctrl__T *localB);
static void le_rigidBodyJoint_get_JointAxis(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T ax[3],
  B_MATLABSystem_left_arm_ctrl__T *localB);
static void left_arm_ctrl_obs_normalizeRows(const real_T matrix[3], real_T
  normRowMatrix[3], B_MATLABSystem_left_arm_ctrl__T *localB);
static void left_arm_ctrl_obs_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyToP(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T T[16],
  B_MATLABSystem_left_arm_ctrl__T *localB);
static void left_arm_ctrl_ob_emxFree_real_T(emxArray_real_T_left_arm_ctrl_T
  **pEmxArray);
static void left_arm_ctrl_obs_tforminv(const real_T T[16], real_T Tinv[16],
  B_MATLABSystem_left_arm_ctrl__T *localB);
static void left_arm_ct_tformToSpatialXform(const real_T T[16], real_T X[36],
  B_MATLABSystem_left_arm_ctrl__T *localB);
static void left_arm_ct_emxFree_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray);
static void rigidBodyJoint_transformBodyT_o(const
  rigidBodyJoint_left_arm_ctrl__T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16], B_MATLABSystem_left_arm_ctrl__T *localB);
static void RigidBodyTreeDynamics_massMatri(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_ctrl_T *H,
  emxArray_real_T_left_arm_ctrl_T *lambda, B_MATLABSystem_left_arm_ctrl__T
  *localB);
static void RigidBodyTreeDynamics_inverseDy(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], const real_T qdot[7], const
  emxArray_real_T_left_arm_ctrl_T *qddot, const real_T fext[60], real_T tau[7],
  B_MATLABSystem_left_arm_ctrl__T *localB);
static void le_emxFreeStruct_rigidBodyJoint(rigidBodyJoint_left_arm_ctrl__T
  *pStruct);
static void emxFreeStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct);

// Forward declaration for local functions
static void left_arm_ctrl_o_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void left_arm_ctrl_SystemCore_step_e(boolean_T *varargout_1, real32_T
  varargout_2_Data[14], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void left_arm_ctrl__emxInit_real_T_e(emxArray_real_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions);
static void left_emxEnsureCapacity_real_T_e(emxArray_real_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel);
static void left_arm_c_emxInit_e_cell_wrap1(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions);
static void emxEnsureCapacity_e_cell_wrap1(emxArray_f_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel);
static void left_arm_ctrl_obs_eye(real_T b_I[36]);
static void left_arm_ctrl__emxInit_char_T_e(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions);
static void left_emxEnsureCapacity_char_T_e(emxArray_char_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel);
static void rigidBodyJoint_get_JointAxis_e0(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T ax[3]);
static void left_arm_ctrl_o_normalizeRows_e(const real_T matrix[3], real_T
  normRowMatrix[3]);
static void left_arm_ctrl_obs_cat_e(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_e(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T T[16]);
static void left_arm_ctrl_obs_tforminv_e(const real_T T[16], real_T Tinv[16]);
static void left_arm__tformToSpatialXform_e(const real_T T[16], real_T X[36]);
static void left_arm_ctrl__emxFree_char_T_e(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray);
static void left_arm_ctrl__emxFree_real_T_e(emxArray_real_T_left_arm_ctrl_T
  **pEmxArray);
static void left_arm_c_emxFree_e_cell_wrap1(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray);
static boolean_T left_arm_ctrl_obs_strcmp(const emxArray_char_T_left_arm_ctrl_T *
  a);
static void lef_GravityTorqueBlock_stepImpl(robotics_slmanip_internal_e0h_T *obj,
  const real32_T q[7], real32_T jointTorq[7]);
static void rigidBodyJoint_transformBody_e0(const
  rigidBodyJoint_left_arm_ctrl__T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void RigidBodyTreeDynamics_massMat_e(k_robotics_manip_internal__e0_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_ctrl_T *H);
static void left_arm_ct_emxInit_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions);
static void l_emxEnsureCapacity_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel);
static void rigidBodyJoint_get_JointAxis_e(const rigidBodyJoint_left_arm_ctr_e_T
  *obj, real_T ax[3]);
static void RigidBodyTree_forwardKinematics(k_robotics_manip_internal_R_e_T *obj,
  const real_T qvec[7], emxArray_e_cell_wrap_left_arm_T *Ttree);
static void left_arm_ct_emxFree_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray);
static void left_arm_ctrl_obs_atan2(const real_T y_data[], const int32_T y_size
  [3], const real_T x_data[], const int32_T x_size[3], real_T r_data[], int32_T
  r_size[3]);
static void left_arm_ctrl_matlabCodegenHa_l(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCo_e0(ros_slros_internal_block_GetP_T *obj);
static void emxFreeStruct_rigidBodyJoint_e(rigidBodyJoint_left_arm_ctrl__T
  *pStruct);
static void emxFreeStruct_i_robotics_mani_e(i_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_k_robotics_mani_e(k_robotics_manip_internal_e0h_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_e(robotics_slmanip_internal_e0h_T
  *pStruct);
static void emxFreeStruct_j_robotics_mani_e(j_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_k_robotics_man_e0(k_robotics_manip_internal__e0_T
  *pStruct);
static void emxFreeStruct_robotics_slman_e0(robotics_slmanip_internal__e0_T
  *pStruct);
static void l_emxFreeStruct_rigidBodyJoint1(rigidBodyJoint_left_arm_ctr_e_T
  *pStruct);
static void emxFreeStruct_i_robotics_man_e0(i_robotics_manip_internal_R_e_T
  *pStruct);
static void emxFreeStruct_k_robotics_ma_e0h(k_robotics_manip_internal_R_e_T
  *pStruct);
static void emxFreeStruct_robotics_slma_e0h(robotics_slmanip_internal_b_e_T
  *pStruct);
static void emxFreeStruct_j_robotics_man_e0(j_robotics_manip_internal_R_e_T
  *pStruct);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_rigidBodyJoint_e(rigidBodyJoint_left_arm_ctrl__T
  *pStruct);
static void emxInitStruct_i_robotics_mani_e(i_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_k_robotics_mani_e(k_robotics_manip_internal_e0h_T
  *pStruct);
static void emxInitStruct_robotics_slmani_e(robotics_slmanip_internal_e0h_T
  *pStruct);
static void emxInitStruct_j_robotics_mani_e(j_robotics_manip_internal_Rig_T
  *pStruct);
static j_robotics_manip_internal_Rig_T *le_RigidBody_RigidBody_e0h4ewmd
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *l_RigidBody_RigidBody_e0h4ewmdi
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *RigidBody_RigidBody_e0h4ewmdid
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *RigidBody_RigidBody_e0h4ewmdidb
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *RigidBody_RigidBod_e0h4ewmdidbj
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *RigidBody_RigidBo_e0h4ewmdidbjt
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *RigidBody_RigidB_e0h4ewmdidbjtq
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *RigidBody_Rigid_e0h4ewmdidbjtqt
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *l_RigidBody_Rigid_e
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *l_RigidBody_Rigid_i
  (j_robotics_manip_internal_Rig_T *obj);
static i_robotics_manip_internal_Rig_T *l_RigidBody_Rigid_n
  (i_robotics_manip_internal_Rig_T *obj);
static k_robotics_manip_internal_e0h_T *RigidBodyTree_RigidBodyTree_e0h
  (k_robotics_manip_internal_e0h_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9);
static void emxInitStruct_k_robotics_man_e0(k_robotics_manip_internal__e0_T
  *pStruct);
static void emxInitStruct_robotics_slman_e0(robotics_slmanip_internal__e0_T
  *pStruct);
static k_robotics_manip_internal__e0_T *RigidBodyTree_RigidBodyTree_e0
  (k_robotics_manip_internal__e0_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9);
static void l_emxInitStruct_rigidBodyJoint1(rigidBodyJoint_left_arm_ctr_e_T
  *pStruct);
static void emxInitStruct_i_robotics_man_e0(i_robotics_manip_internal_R_e_T
  *pStruct);
static void emxInitStruct_k_robotics_ma_e0h(k_robotics_manip_internal_R_e_T
  *pStruct);
static void emxInitStruct_robotics_slma_e0h(robotics_slmanip_internal_b_e_T
  *pStruct);
static void emxInitStruct_j_robotics_man_e0(j_robotics_manip_internal_R_e_T
  *pStruct);
static j_robotics_manip_internal_R_e_T *left_arm__RigidBody_RigidBody_e
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *left_arm_RigidBody_RigidBody_e0
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *left_ar_RigidBody_RigidBody_e0h
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *left_a_RigidBody_RigidBody_e0h4
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *left__RigidBody_RigidBody_e0h4e
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *left_RigidBody_RigidBody_e0h4ew
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *lef_RigidBody_RigidBody_e0h4ewm
  (j_robotics_manip_internal_R_e_T *obj);
static k_robotics_manip_internal_R_e_T *l_RigidBodyTree_RigidBodyTree_e
  (k_robotics_manip_internal_R_e_T *obj, j_robotics_manip_internal_R_e_T *iobj_0,
   j_robotics_manip_internal_R_e_T *iobj_1, j_robotics_manip_internal_R_e_T
   *iobj_2, j_robotics_manip_internal_R_e_T *iobj_3,
   j_robotics_manip_internal_R_e_T *iobj_4, j_robotics_manip_internal_R_e_T
   *iobj_5, j_robotics_manip_internal_R_e_T *iobj_6,
   j_robotics_manip_internal_R_e_T *iobj_7, j_robotics_manip_internal_R_e_T
   *iobj_8, j_robotics_manip_internal_R_e_T *iobj_9);
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

  (left_arm_ctrl_obs_M->Timing.TaskCounters.TID[2])++;
  if ((left_arm_ctrl_obs_M->Timing.TaskCounters.TID[2]) > 9) {// Sample time: [0.04s, 0.0s] 
    left_arm_ctrl_obs_M->Timing.TaskCounters.TID[2] = 0;
  }

  (left_arm_ctrl_obs_M->Timing.TaskCounters.TID[3])++;
  if ((left_arm_ctrl_obs_M->Timing.TaskCounters.TID[3]) > 249) {// Sample time: [1.0s, 0.0s] 
    left_arm_ctrl_obs_M->Timing.TaskCounters.TID[3] = 0;
  }
}

//
// This function updates continuous states using the ODE4 fixed-step
// solver algorithm
//
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = static_cast<ODE4_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 252;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  left_arm_ctrl_obs_derivatives();

  // f1 = f(t + (h/2), y + (h/2)*f0)
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  left_arm_ctrl_obs_step();
  left_arm_ctrl_obs_derivatives();

  // f2 = f(t + (h/2), y + (h/2)*f1)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  left_arm_ctrl_obs_step();
  left_arm_ctrl_obs_derivatives();

  // f3 = f(t + h, y + h*f2)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  left_arm_ctrl_obs_step();
  left_arm_ctrl_obs_derivatives();

  // tnew = t + h
  // ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3)
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void left_arm_ctrl_ob_emxInit_char_T(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  emxArray_char_T_left_arm_ctrl_T *emxArray;
  *pEmxArray = (emxArray_char_T_left_arm_ctrl_T *)malloc(sizeof
    (emxArray_char_T_left_arm_ctrl_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (localB->i_f = 0; localB->i_f < numDimensions; localB->i_f++) {
    emxArray->size[localB->i_f] = 0;
  }
}

static void left_arm_ctrl_ob_emxInit_real_T(emxArray_real_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  emxArray_real_T_left_arm_ctrl_T *emxArray;
  *pEmxArray = (emxArray_real_T_left_arm_ctrl_T *)malloc(sizeof
    (emxArray_real_T_left_arm_ctrl_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (localB->i_ip = 0; localB->i_ip < numDimensions; localB->i_ip++) {
    emxArray->size[localB->i_ip] = 0;
  }
}

static void le_emxInitStruct_rigidBodyJoint(rigidBodyJoint_left_arm_ctrl__T
  *pStruct, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  left_arm_ctrl_ob_emxInit_char_T(&pStruct->Type, 2, localB);
  left_arm_ctrl_ob_emxInit_real_T(&pStruct->MotionSubspace, 2, localB);
}

static void emxInitStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  le_emxInitStruct_rigidBodyJoint(&pStruct->JointInternal, localB);
}

static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  emxInitStruct_i_robotics_manip_(&pStruct->Base, localB);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  emxInitStruct_k_robotics_manip_(&pStruct->TreeInternal, localB);
}

static void emxInitStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  le_emxInitStruct_rigidBodyJoint(&pStruct->JointInternal, localB);
}

static void left_a_emxEnsureCapacity_char_T(emxArray_char_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  localB->newNumel_i = 1;
  for (localB->i_l = 0; localB->i_l < emxArray->numDimensions; localB->i_l++) {
    localB->newNumel_i *= emxArray->size[localB->i_l];
  }

  if (localB->newNumel_i > emxArray->allocatedSize) {
    localB->i_l = emxArray->allocatedSize;
    if (localB->i_l < 16) {
      localB->i_l = 16;
    }

    while (localB->i_l < localB->newNumel_i) {
      if (localB->i_l > 1073741823) {
        localB->i_l = MAX_int32_T;
      } else {
        localB->i_l <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(localB->i_l), sizeof(char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = localB->i_l;
    emxArray->canFreeData = true;
  }
}

static void left_arm_ctrl_ob_emxFree_char_T(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_left_arm_ctrl_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_left_arm_ctrl_T *)NULL;
  }
}

static void left_a_emxEnsureCapacity_real_T(emxArray_real_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  localB->newNumel = 1;
  for (localB->i_a = 0; localB->i_a < emxArray->numDimensions; localB->i_a++) {
    localB->newNumel *= emxArray->size[localB->i_a];
  }

  if (localB->newNumel > emxArray->allocatedSize) {
    localB->i_a = emxArray->allocatedSize;
    if (localB->i_a < 16) {
      localB->i_a = 16;
    }

    while (localB->i_a < localB->newNumel) {
      if (localB->i_a > 1073741823) {
        localB->i_a = MAX_int32_T;
      } else {
        localB->i_a <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(localB->i_a), sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = localB->i_a;
    emxArray->canFreeData = true;
  }
}

static j_robotics_manip_internal_Rig_T *left_arm_ct_RigidBody_RigidBody
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.30007875, 0.00021157499999999998, 0.0, 0.0,
    -0.0, -0.0052499999999999995, 0.00021157499999999998, 0.3005684315, 0.0, 0.0,
    0.0, -0.014105, 0.0, 0.0, 0.30064718149999997, 0.0052499999999999995,
    0.014105, 0.0, 0.0, 0.0, 0.0052499999999999995, 0.35, 0.0, 0.0, -0.0, 0.0,
    0.014105, 0.0, 0.35, 0.0, -0.0052499999999999995, -0.014105, 0.0, 0.0, 0.0,
    0.35 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.055, 0.09, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 0.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left_arm__RigidBody_RigidBody_m
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.302205,
    0.0, 0.0, 0.0, -0.021, 0.0, 0.0, 0.302205, -0.0, 0.021, 0.0, 0.0, 0.0, -0.0,
    0.2, 0.0, 0.0, -0.0, 0.0, 0.021, 0.0, 0.2, 0.0, 0.0, -0.021, 0.0, 0.0, 0.0,
    0.2 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0603, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 1.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left_arm_RigidBody_RigidBody_my
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.307938,
    0.0, 0.0, 0.0, -0.0378, 0.0, 0.0, 0.307938, -0.0, 0.0378, 0.0, 0.0, 0.0,
    -0.0, 0.18, 0.0, 0.0, -0.0, 0.0, 0.0378, 0.0, 0.18, 0.0, 0.0, -0.0378, 0.0,
    0.0, 0.0, 0.18 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 2.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left_ar_RigidBody_RigidBody_mye
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.301458,
    0.0, 0.0, 0.0, -0.0162, 0.0, 0.0, 0.301458, -0.0, 0.0162, 0.0, 0.0, 0.0,
    -0.0, 0.18, 0.0, 0.0, -0.0, 0.0, 0.0162, 0.0, 0.18, 0.0, 0.0, -0.0162, 0.0,
    0.0, 0.0, 0.18 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.27, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 3.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left_a_RigidBody_RigidBody_myeu
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.305832,
    0.0, 0.0, 0.0, -0.0324, 0.0, 0.0, 0.305832, -0.0, 0.0324, 0.0, 0.0, 0.0,
    -0.0, 0.18, 0.0, 0.0, -0.0, 0.0, 0.0324, 0.0, 0.18, 0.0, 0.0, -0.0324, 0.0,
    0.0, 0.0, 0.18 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 4.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left__RigidBody_RigidBody_myeu0
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.300375,
    0.0, 0.0, 0.0, -0.0075, 0.0, 0.0, 0.300375, -0.0, 0.0075, 0.0, 0.0, 0.0,
    -0.0, 0.15, 0.0, 0.0, -0.0, 0.0, 0.0075, 0.0, 0.15, 0.0, 0.0, -0.0075, 0.0,
    0.0, 0.0, 0.15 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.2126, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 5.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left_RigidBody_RigidBody_myeu0z
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.3015,
    0.0, 0.0, 0.0, -0.015, 0.0, 0.0, 0.3015, -0.0, 0.015, 0.0, 0.0, 0.0, -0.0,
    0.15, 0.0, 0.0, -0.0, 0.0, 0.015, 0.0, 0.15, 0.0, 0.0, -0.015, 0.0, 0.0, 0.0,
    0.15 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 6.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *lef_RigidBody_RigidBody_myeu0zw
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.13, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 7.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *le_RigidBody_RigidBody_myeu0zwr
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, -0.04, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 8.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *l_RigidBody_RigidBody_myeu0zwrs
  (j_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.04, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 8.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static i_robotics_manip_internal_Rig_T *RigidBody_RigidBody_myeu0zwrsj
  (i_robotics_manip_internal_Rig_T *obj, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  i_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr, localB);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2, localB);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr, localB);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr,
    localB);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  return b_obj;
}

static k_robotics_manip_internal_Rig_T *lef_RigidBodyTree_RigidBodyTree
  (k_robotics_manip_internal_Rig_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9,
   B_MATLABSystem_left_arm_ctrl__T *localB)
{
  k_robotics_manip_internal_Rig_T *b_obj;
  int32_T i;
  static const int8_T tmp[20] = { 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 1, 2, 3, 4, 5, 6,
    7, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = left_arm_ct_RigidBody_RigidBody(iobj_0, localB);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = left_arm__RigidBody_RigidBody_m(iobj_1, localB);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = left_arm_RigidBody_RigidBody_my(iobj_2, localB);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = left_ar_RigidBody_RigidBody_mye(iobj_3, localB);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = left_a_RigidBody_RigidBody_myeu(iobj_4, localB);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = left__RigidBody_RigidBody_myeu0(iobj_5, localB);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = left_RigidBody_RigidBody_myeu0z(iobj_6, localB);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = lef_RigidBody_RigidBody_myeu0zw(iobj_7, localB);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = le_RigidBody_RigidBody_myeu0zwr(iobj_8, localB);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = l_RigidBody_RigidBody_myeu0zwrs(iobj_9, localB);
  obj->Bodies[9]->Index = 10.0;
  obj->NumBodies = 10.0;
  obj->Gravity[0] = 9.81;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = 0.0;
  obj->VelocityNumber = 7.0;
  for (i = 0; i < 20; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  for (i = 0; i < 20; i++) {
    obj->VelocityDoFMap[i] = tmp[i];
  }

  RigidBody_RigidBody_myeu0zwrsj(&obj->Base, localB);
  return b_obj;
}

static void left_arm_ct_emxInit_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  emxArray_f_cell_wrap_left_arm_T *emxArray;
  *pEmxArray = (emxArray_f_cell_wrap_left_arm_T *)malloc(sizeof
    (emxArray_f_cell_wrap_left_arm_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_left_arm_ctrl_obs_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (localB->i_iz = 0; localB->i_iz < numDimensions; localB->i_iz++) {
    emxArray->size[localB->i_iz] = 0;
  }
}

static void l_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel, B_MATLABSystem_left_arm_ctrl__T *localB)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  localB->newNumel_o = 1;
  for (localB->i_o = 0; localB->i_o < emxArray->numDimensions; localB->i_o++) {
    localB->newNumel_o *= emxArray->size[localB->i_o];
  }

  if (localB->newNumel_o > emxArray->allocatedSize) {
    localB->i_o = emxArray->allocatedSize;
    if (localB->i_o < 16) {
      localB->i_o = 16;
    }

    while (localB->i_o < localB->newNumel_o) {
      if (localB->i_o > 1073741823) {
        localB->i_o = MAX_int32_T;
      } else {
        localB->i_o <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(localB->i_o), sizeof
                     (f_cell_wrap_left_arm_ctrl_obs_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_left_arm_ctrl_obs_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_left_arm_ctrl_obs_T *)newData;
    emxArray->allocatedSize = localB->i_o;
    emxArray->canFreeData = true;
  }
}

static void le_rigidBodyJoint_get_JointAxis(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T ax[3],
  B_MATLABSystem_left_arm_ctrl__T *localB)
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (localB->b_kstr_n = 0; localB->b_kstr_n < 8; localB->b_kstr_n++) {
    localB->b_l[localB->b_kstr_n] = tmp[localB->b_kstr_n];
  }

  localB->b_bool_g = false;
  if (obj->Type->size[1] == 8) {
    localB->b_kstr_n = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr_n - 1 < 8) {
        localB->kstr_o = localB->b_kstr_n - 1;
        if (obj->Type->data[localB->kstr_o] != localB->b_l[localB->kstr_o]) {
          exitg1 = 1;
        } else {
          localB->b_kstr_n++;
        }
      } else {
        localB->b_bool_g = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (localB->b_bool_g) {
    guard1 = true;
  } else {
    for (localB->b_kstr_n = 0; localB->b_kstr_n < 9; localB->b_kstr_n++) {
      localB->b[localB->b_kstr_n] = tmp_0[localB->b_kstr_n];
    }

    localB->b_bool_g = false;
    if (obj->Type->size[1] == 9) {
      localB->b_kstr_n = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr_n - 1 < 9) {
          localB->kstr_o = localB->b_kstr_n - 1;
          if (obj->Type->data[localB->kstr_o] != localB->b[localB->kstr_o]) {
            exitg1 = 1;
          } else {
            localB->b_kstr_n++;
          }
        } else {
          localB->b_bool_g = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_g) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void left_arm_ctrl_obs_normalizeRows(const real_T matrix[3], real_T
  normRowMatrix[3], B_MATLABSystem_left_arm_ctrl__T *localB)
{
  localB->b_b = 1.0 / sqrt((matrix[0] * matrix[0] + matrix[1] * matrix[1]) +
    matrix[2] * matrix[2]);
  normRowMatrix[0] = matrix[0] * localB->b_b;
  normRowMatrix[1] = matrix[1] * localB->b_b;
  normRowMatrix[2] = matrix[2] * localB->b_b;
}

static void left_arm_ctrl_obs_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9])
{
  y[0] = varargin_1;
  y[1] = varargin_2;
  y[2] = varargin_3;
  y[3] = varargin_4;
  y[4] = varargin_5;
  y[5] = varargin_6;
  y[6] = varargin_7;
  y[7] = varargin_8;
  y[8] = varargin_9;
}

static void rigidBodyJoint_transformBodyToP(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T T[16],
  B_MATLABSystem_left_arm_ctrl__T *localB)
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (localB->b_kstr_p = 0; localB->b_kstr_p < 5; localB->b_kstr_p++) {
    localB->b_ju[localB->b_kstr_p] = tmp[localB->b_kstr_p];
  }

  localB->b_bool_o = false;
  if (obj->Type->size[1] == 5) {
    localB->b_kstr_p = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr_p - 1 < 5) {
        localB->kstr_p = localB->b_kstr_p - 1;
        if (obj->Type->data[localB->kstr_p] != localB->b_ju[localB->kstr_p]) {
          exitg1 = 1;
        } else {
          localB->b_kstr_p++;
        }
      } else {
        localB->b_bool_o = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (localB->b_bool_o) {
    localB->b_kstr_p = 0;
  } else {
    for (localB->b_kstr_p = 0; localB->b_kstr_p < 8; localB->b_kstr_p++) {
      localB->b_d[localB->b_kstr_p] = tmp_0[localB->b_kstr_p];
    }

    localB->b_bool_o = false;
    if (obj->Type->size[1] == 8) {
      localB->b_kstr_p = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr_p - 1 < 8) {
          localB->kstr_p = localB->b_kstr_p - 1;
          if (obj->Type->data[localB->kstr_p] != localB->b_d[localB->kstr_p]) {
            exitg1 = 1;
          } else {
            localB->b_kstr_p++;
          }
        } else {
          localB->b_bool_o = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_o) {
      localB->b_kstr_p = 1;
    } else {
      localB->b_kstr_p = -1;
    }
  }

  switch (localB->b_kstr_p) {
   case 0:
    memset(&localB->TJ_c[0], 0, sizeof(real_T) << 4U);
    localB->TJ_c[0] = 1.0;
    localB->TJ_c[5] = 1.0;
    localB->TJ_c[10] = 1.0;
    localB->TJ_c[15] = 1.0;
    break;

   case 1:
    le_rigidBodyJoint_get_JointAxis(obj, localB->v_j, localB);
    localB->v_d[0] = localB->v_j[0];
    localB->v_d[1] = localB->v_j[1];
    localB->v_d[2] = localB->v_j[2];
    left_arm_ctrl_obs_normalizeRows(localB->v_d, localB->v_j, localB);
    localB->tempR_tmp_bn = localB->v_j[1] * localB->v_j[0] * 0.0;
    localB->tempR_tmp_da = localB->v_j[2] * localB->v_j[0] * 0.0;
    localB->tempR_tmp_e = localB->v_j[2] * localB->v_j[1] * 0.0;
    left_arm_ctrl_obs_cat(localB->v_j[0] * localB->v_j[0] * 0.0 + 1.0,
                          localB->tempR_tmp_bn - localB->v_j[2] * 0.0,
                          localB->tempR_tmp_da + localB->v_j[1] * 0.0,
                          localB->tempR_tmp_bn + localB->v_j[2] * 0.0,
                          localB->v_j[1] * localB->v_j[1] * 0.0 + 1.0,
                          localB->tempR_tmp_e - localB->v_j[0] * 0.0,
                          localB->tempR_tmp_da - localB->v_j[1] * 0.0,
                          localB->tempR_tmp_e + localB->v_j[0] * 0.0,
                          localB->v_j[2] * localB->v_j[2] * 0.0 + 1.0,
                          localB->tempR_f);
    for (localB->b_kstr_p = 0; localB->b_kstr_p < 3; localB->b_kstr_p++) {
      localB->kstr_p = localB->b_kstr_p + 1;
      localB->R_cv[localB->kstr_p - 1] = localB->tempR_f[(localB->kstr_p - 1) *
        3];
      localB->kstr_p = localB->b_kstr_p + 1;
      localB->R_cv[localB->kstr_p + 2] = localB->tempR_f[(localB->kstr_p - 1) *
        3 + 1];
      localB->kstr_p = localB->b_kstr_p + 1;
      localB->R_cv[localB->kstr_p + 5] = localB->tempR_f[(localB->kstr_p - 1) *
        3 + 2];
    }

    memset(&localB->TJ_c[0], 0, sizeof(real_T) << 4U);
    for (localB->b_kstr_p = 0; localB->b_kstr_p < 3; localB->b_kstr_p++) {
      localB->kstr_p = localB->b_kstr_p << 2;
      localB->TJ_c[localB->kstr_p] = localB->R_cv[3 * localB->b_kstr_p];
      localB->TJ_c[localB->kstr_p + 1] = localB->R_cv[3 * localB->b_kstr_p + 1];
      localB->TJ_c[localB->kstr_p + 2] = localB->R_cv[3 * localB->b_kstr_p + 2];
    }

    localB->TJ_c[15] = 1.0;
    break;

   default:
    le_rigidBodyJoint_get_JointAxis(obj, localB->v_j, localB);
    memset(&localB->tempR_f[0], 0, 9U * sizeof(real_T));
    localB->tempR_f[0] = 1.0;
    localB->tempR_f[4] = 1.0;
    localB->tempR_f[8] = 1.0;
    for (localB->b_kstr_p = 0; localB->b_kstr_p < 3; localB->b_kstr_p++) {
      localB->kstr_p = localB->b_kstr_p << 2;
      localB->TJ_c[localB->kstr_p] = localB->tempR_f[3 * localB->b_kstr_p];
      localB->TJ_c[localB->kstr_p + 1] = localB->tempR_f[3 * localB->b_kstr_p +
        1];
      localB->TJ_c[localB->kstr_p + 2] = localB->tempR_f[3 * localB->b_kstr_p +
        2];
      localB->TJ_c[localB->b_kstr_p + 12] = localB->v_j[localB->b_kstr_p] * 0.0;
    }

    localB->TJ_c[3] = 0.0;
    localB->TJ_c[7] = 0.0;
    localB->TJ_c[11] = 0.0;
    localB->TJ_c[15] = 1.0;
    break;
  }

  for (localB->b_kstr_p = 0; localB->b_kstr_p < 4; localB->b_kstr_p++) {
    for (localB->kstr_p = 0; localB->kstr_p < 4; localB->kstr_p++) {
      localB->obj_tmp_tmp_e = localB->kstr_p << 2;
      localB->obj_tmp_a = localB->b_kstr_p + localB->obj_tmp_tmp_e;
      localB->obj_k[localB->obj_tmp_a] = 0.0;
      localB->obj_k[localB->obj_tmp_a] += localB->TJ_c[localB->obj_tmp_tmp_e] *
        obj->JointToParentTransform[localB->b_kstr_p];
      localB->obj_k[localB->obj_tmp_a] += localB->TJ_c[localB->obj_tmp_tmp_e + 1]
        * obj->JointToParentTransform[localB->b_kstr_p + 4];
      localB->obj_k[localB->obj_tmp_a] += localB->TJ_c[localB->obj_tmp_tmp_e + 2]
        * obj->JointToParentTransform[localB->b_kstr_p + 8];
      localB->obj_k[localB->obj_tmp_a] += localB->TJ_c[localB->obj_tmp_tmp_e + 3]
        * obj->JointToParentTransform[localB->b_kstr_p + 12];
    }

    for (localB->kstr_p = 0; localB->kstr_p < 4; localB->kstr_p++) {
      localB->obj_tmp_tmp_e = localB->kstr_p << 2;
      localB->obj_tmp_a = localB->b_kstr_p + localB->obj_tmp_tmp_e;
      T[localB->obj_tmp_a] = 0.0;
      T[localB->obj_tmp_a] += obj->ChildToJointTransform[localB->obj_tmp_tmp_e] *
        localB->obj_k[localB->b_kstr_p];
      T[localB->obj_tmp_a] += obj->ChildToJointTransform[localB->obj_tmp_tmp_e +
        1] * localB->obj_k[localB->b_kstr_p + 4];
      T[localB->obj_tmp_a] += obj->ChildToJointTransform[localB->obj_tmp_tmp_e +
        2] * localB->obj_k[localB->b_kstr_p + 8];
      T[localB->obj_tmp_a] += obj->ChildToJointTransform[localB->obj_tmp_tmp_e +
        3] * localB->obj_k[localB->b_kstr_p + 12];
    }
  }
}

static void left_arm_ctrl_ob_emxFree_real_T(emxArray_real_T_left_arm_ctrl_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_left_arm_ctrl_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_left_arm_ctrl_T *)NULL;
  }
}

static void left_arm_ctrl_obs_tforminv(const real_T T[16], real_T Tinv[16],
  B_MATLABSystem_left_arm_ctrl__T *localB)
{
  for (localB->i3 = 0; localB->i3 < 3; localB->i3++) {
    localB->R_g[3 * localB->i3] = T[localB->i3];
    localB->R_g[3 * localB->i3 + 1] = T[localB->i3 + 4];
    localB->R_g[3 * localB->i3 + 2] = T[localB->i3 + 8];
  }

  for (localB->i3 = 0; localB->i3 < 9; localB->i3++) {
    localB->R_g1[localB->i3] = -localB->R_g[localB->i3];
  }

  for (localB->i3 = 0; localB->i3 < 3; localB->i3++) {
    localB->Tinv_tmp = localB->i3 << 2;
    Tinv[localB->Tinv_tmp] = localB->R_g[3 * localB->i3];
    Tinv[localB->Tinv_tmp + 1] = localB->R_g[3 * localB->i3 + 1];
    Tinv[localB->Tinv_tmp + 2] = localB->R_g[3 * localB->i3 + 2];
    Tinv[localB->i3 + 12] = localB->R_g1[localB->i3 + 6] * T[14] + (localB->
      R_g1[localB->i3 + 3] * T[13] + localB->R_g1[localB->i3] * T[12]);
  }

  Tinv[3] = 0.0;
  Tinv[7] = 0.0;
  Tinv[11] = 0.0;
  Tinv[15] = 1.0;
}

static void left_arm_ct_tformToSpatialXform(const real_T T[16], real_T X[36],
  B_MATLABSystem_left_arm_ctrl__T *localB)
{
  localB->dv4[0] = 0.0;
  localB->dv4[3] = -T[14];
  localB->dv4[6] = T[13];
  localB->dv4[1] = T[14];
  localB->dv4[4] = 0.0;
  localB->dv4[7] = -T[12];
  localB->dv4[2] = -T[13];
  localB->dv4[5] = T[12];
  localB->dv4[8] = 0.0;
  for (localB->i1 = 0; localB->i1 < 3; localB->i1++) {
    for (localB->X_tmp = 0; localB->X_tmp < 3; localB->X_tmp++) {
      localB->X_tmp_a = localB->i1 + 3 * localB->X_tmp;
      localB->dv5[localB->X_tmp_a] = 0.0;
      localB->i2 = localB->X_tmp << 2;
      localB->dv5[localB->X_tmp_a] += T[localB->i2] * localB->dv4[localB->i1];
      localB->dv5[localB->X_tmp_a] += T[localB->i2 + 1] * localB->dv4[localB->i1
        + 3];
      localB->dv5[localB->X_tmp_a] += T[localB->i2 + 2] * localB->dv4[localB->i1
        + 6];
      X[localB->X_tmp + 6 * localB->i1] = T[(localB->i1 << 2) + localB->X_tmp];
      X[localB->X_tmp + 6 * (localB->i1 + 3)] = 0.0;
    }
  }

  for (localB->i1 = 0; localB->i1 < 3; localB->i1++) {
    X[6 * localB->i1 + 3] = localB->dv5[3 * localB->i1];
    localB->X_tmp = localB->i1 << 2;
    localB->X_tmp_a = 6 * (localB->i1 + 3);
    X[localB->X_tmp_a + 3] = T[localB->X_tmp];
    X[6 * localB->i1 + 4] = localB->dv5[3 * localB->i1 + 1];
    X[localB->X_tmp_a + 4] = T[localB->X_tmp + 1];
    X[6 * localB->i1 + 5] = localB->dv5[3 * localB->i1 + 2];
    X[localB->X_tmp_a + 5] = T[localB->X_tmp + 2];
  }
}

static void left_arm_ct_emxFree_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_left_arm_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_left_arm_ctrl_obs_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_left_arm_T *)NULL;
  }
}

static void rigidBodyJoint_transformBodyT_o(const
  rigidBodyJoint_left_arm_ctrl__T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16], B_MATLABSystem_left_arm_ctrl__T *localB)
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (localB->b_kstr = 0; localB->b_kstr < 5; localB->b_kstr++) {
    localB->b_f[localB->b_kstr] = tmp[localB->b_kstr];
  }

  localB->b_bool_f = false;
  if (obj->Type->size[1] == 5) {
    localB->b_kstr = 1;
    do {
      exitg1 = 0;
      if (localB->b_kstr - 1 < 5) {
        localB->kstr = localB->b_kstr - 1;
        if (obj->Type->data[localB->kstr] != localB->b_f[localB->kstr]) {
          exitg1 = 1;
        } else {
          localB->b_kstr++;
        }
      } else {
        localB->b_bool_f = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (localB->b_bool_f) {
    localB->b_kstr = 0;
  } else {
    for (localB->b_kstr = 0; localB->b_kstr < 8; localB->b_kstr++) {
      localB->b_g[localB->b_kstr] = tmp_0[localB->b_kstr];
    }

    localB->b_bool_f = false;
    if (obj->Type->size[1] == 8) {
      localB->b_kstr = 1;
      do {
        exitg1 = 0;
        if (localB->b_kstr - 1 < 8) {
          localB->kstr = localB->b_kstr - 1;
          if (obj->Type->data[localB->kstr] != localB->b_g[localB->kstr]) {
            exitg1 = 1;
          } else {
            localB->b_kstr++;
          }
        } else {
          localB->b_bool_f = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (localB->b_bool_f) {
      localB->b_kstr = 1;
    } else {
      localB->b_kstr = -1;
    }
  }

  switch (localB->b_kstr) {
   case 0:
    memset(&localB->TJ[0], 0, sizeof(real_T) << 4U);
    localB->TJ[0] = 1.0;
    localB->TJ[5] = 1.0;
    localB->TJ[10] = 1.0;
    localB->TJ[15] = 1.0;
    break;

   case 1:
    le_rigidBodyJoint_get_JointAxis(obj, localB->v, localB);
    localB->result_data[0] = localB->v[0];
    localB->result_data[1] = localB->v[1];
    localB->result_data[2] = localB->v[2];
    if (0 <= (*q_size != 0) - 1) {
      localB->result_data[3] = q_data[0];
    }

    left_arm_ctrl_obs_normalizeRows(&localB->result_data[0], localB->v, localB);
    localB->cth = cos(localB->result_data[3]);
    localB->sth = sin(localB->result_data[3]);
    localB->tempR_tmp = localB->v[1] * localB->v[0] * (1.0 - localB->cth);
    localB->tempR_tmp_d = localB->v[2] * localB->sth;
    localB->tempR_tmp_l = localB->v[2] * localB->v[0] * (1.0 - localB->cth);
    localB->tempR_tmp_o = localB->v[1] * localB->sth;
    localB->tempR_tmp_b = localB->v[2] * localB->v[1] * (1.0 - localB->cth);
    localB->sth *= localB->v[0];
    left_arm_ctrl_obs_cat(localB->v[0] * localB->v[0] * (1.0 - localB->cth) +
                          localB->cth, localB->tempR_tmp - localB->tempR_tmp_d,
                          localB->tempR_tmp_l + localB->tempR_tmp_o,
                          localB->tempR_tmp + localB->tempR_tmp_d, localB->v[1] *
                          localB->v[1] * (1.0 - localB->cth) + localB->cth,
                          localB->tempR_tmp_b - localB->sth, localB->tempR_tmp_l
                          - localB->tempR_tmp_o, localB->tempR_tmp_b +
                          localB->sth, localB->v[2] * localB->v[2] * (1.0 -
      localB->cth) + localB->cth, localB->tempR);
    for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
      localB->kstr = localB->b_kstr + 1;
      localB->R_p[localB->kstr - 1] = localB->tempR[(localB->kstr - 1) * 3];
      localB->kstr = localB->b_kstr + 1;
      localB->R_p[localB->kstr + 2] = localB->tempR[(localB->kstr - 1) * 3 + 1];
      localB->kstr = localB->b_kstr + 1;
      localB->R_p[localB->kstr + 5] = localB->tempR[(localB->kstr - 1) * 3 + 2];
    }

    memset(&localB->TJ[0], 0, sizeof(real_T) << 4U);
    for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
      localB->kstr = localB->b_kstr << 2;
      localB->TJ[localB->kstr] = localB->R_p[3 * localB->b_kstr];
      localB->TJ[localB->kstr + 1] = localB->R_p[3 * localB->b_kstr + 1];
      localB->TJ[localB->kstr + 2] = localB->R_p[3 * localB->b_kstr + 2];
    }

    localB->TJ[15] = 1.0;
    break;

   default:
    le_rigidBodyJoint_get_JointAxis(obj, localB->v, localB);
    memset(&localB->tempR[0], 0, 9U * sizeof(real_T));
    localB->tempR[0] = 1.0;
    localB->tempR[4] = 1.0;
    localB->tempR[8] = 1.0;
    for (localB->b_kstr = 0; localB->b_kstr < 3; localB->b_kstr++) {
      localB->kstr = localB->b_kstr << 2;
      localB->TJ[localB->kstr] = localB->tempR[3 * localB->b_kstr];
      localB->TJ[localB->kstr + 1] = localB->tempR[3 * localB->b_kstr + 1];
      localB->TJ[localB->kstr + 2] = localB->tempR[3 * localB->b_kstr + 2];
      localB->TJ[localB->b_kstr + 12] = localB->v[localB->b_kstr] * q_data[0];
    }

    localB->TJ[3] = 0.0;
    localB->TJ[7] = 0.0;
    localB->TJ[11] = 0.0;
    localB->TJ[15] = 1.0;
    break;
  }

  for (localB->b_kstr = 0; localB->b_kstr < 4; localB->b_kstr++) {
    for (localB->kstr = 0; localB->kstr < 4; localB->kstr++) {
      localB->obj_tmp_tmp = localB->kstr << 2;
      localB->obj_tmp = localB->b_kstr + localB->obj_tmp_tmp;
      localB->obj[localB->obj_tmp] = 0.0;
      localB->obj[localB->obj_tmp] += localB->TJ[localB->obj_tmp_tmp] *
        obj->JointToParentTransform[localB->b_kstr];
      localB->obj[localB->obj_tmp] += localB->TJ[localB->obj_tmp_tmp + 1] *
        obj->JointToParentTransform[localB->b_kstr + 4];
      localB->obj[localB->obj_tmp] += localB->TJ[localB->obj_tmp_tmp + 2] *
        obj->JointToParentTransform[localB->b_kstr + 8];
      localB->obj[localB->obj_tmp] += localB->TJ[localB->obj_tmp_tmp + 3] *
        obj->JointToParentTransform[localB->b_kstr + 12];
    }

    for (localB->kstr = 0; localB->kstr < 4; localB->kstr++) {
      localB->obj_tmp_tmp = localB->kstr << 2;
      localB->obj_tmp = localB->b_kstr + localB->obj_tmp_tmp;
      T[localB->obj_tmp] = 0.0;
      T[localB->obj_tmp] += obj->ChildToJointTransform[localB->obj_tmp_tmp] *
        localB->obj[localB->b_kstr];
      T[localB->obj_tmp] += obj->ChildToJointTransform[localB->obj_tmp_tmp + 1] *
        localB->obj[localB->b_kstr + 4];
      T[localB->obj_tmp] += obj->ChildToJointTransform[localB->obj_tmp_tmp + 2] *
        localB->obj[localB->b_kstr + 8];
      T[localB->obj_tmp] += obj->ChildToJointTransform[localB->obj_tmp_tmp + 3] *
        localB->obj[localB->b_kstr + 12];
    }
  }
}

static void RigidBodyTreeDynamics_massMatri(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_ctrl_T *H,
  emxArray_real_T_left_arm_ctrl_T *lambda, B_MATLABSystem_left_arm_ctrl__T
  *localB)
{
  emxArray_f_cell_wrap_left_arm_T *Ic;
  emxArray_f_cell_wrap_left_arm_T *X;
  emxArray_real_T_left_arm_ctrl_T *lambda_;
  emxArray_real_T_left_arm_ctrl_T *Si;
  emxArray_real_T_left_arm_ctrl_T *Fi;
  emxArray_real_T_left_arm_ctrl_T *Sj;
  emxArray_real_T_left_arm_ctrl_T *Hji;
  emxArray_real_T_left_arm_ctrl_T *s;
  j_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_left_arm_ctrl_T *a;
  emxArray_real_T_left_arm_ctrl_T *a_0;
  emxArray_real_T_left_arm_ctrl_T *B;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  localB->nb_n = robot->NumBodies;
  localB->vNum_b = robot->VelocityNumber;
  localB->nm1d2 = H->size[0] * H->size[1];
  localB->b_i = static_cast<int32_T>(localB->vNum_b);
  H->size[0] = localB->b_i;
  H->size[1] = localB->b_i;
  left_a_emxEnsureCapacity_real_T(H, localB->nm1d2, localB);
  localB->n_m = localB->b_i * localB->b_i - 1;
  for (localB->nm1d2 = 0; localB->nm1d2 <= localB->n_m; localB->nm1d2++) {
    H->data[localB->nm1d2] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&lambda_, 2, localB);
  localB->nm1d2 = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  localB->unnamed_idx_1_c = static_cast<int32_T>(localB->nb_n);
  lambda_->size[1] = localB->unnamed_idx_1_c;
  left_a_emxEnsureCapacity_real_T(lambda_, localB->nm1d2, localB);
  localB->idx = localB->unnamed_idx_1_c - 1;
  for (localB->nm1d2 = 0; localB->nm1d2 <= localB->idx; localB->nm1d2++) {
    lambda_->data[localB->nm1d2] = 0.0;
  }

  localB->nm1d2 = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = localB->b_i;
  left_a_emxEnsureCapacity_real_T(lambda, localB->nm1d2, localB);
  localB->n_m = localB->b_i - 1;
  for (localB->nm1d2 = 0; localB->nm1d2 <= localB->n_m; localB->nm1d2++) {
    lambda->data[localB->nm1d2] = 0.0;
  }

  left_arm_ct_emxInit_f_cell_wrap(&Ic, 2, localB);
  left_arm_ct_emxInit_f_cell_wrap(&X, 2, localB);
  localB->nm1d2 = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = localB->unnamed_idx_1_c;
  l_emxEnsureCapacity_f_cell_wrap(Ic, localB->nm1d2, localB);
  localB->nm1d2 = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = localB->unnamed_idx_1_c;
  l_emxEnsureCapacity_f_cell_wrap(X, localB->nm1d2, localB);
  for (localB->b_i = 0; localB->b_i <= localB->idx; localB->b_i++) {
    for (localB->nm1d2 = 0; localB->nm1d2 < 36; localB->nm1d2++) {
      Ic->data[localB->b_i].f1[localB->nm1d2] = robot->Bodies[localB->b_i]
        ->SpatialInertia[localB->nm1d2];
    }

    localB->vNum_b = robot->PositionDoFMap[localB->b_i];
    localB->p_idx_1 = robot->PositionDoFMap[localB->b_i + 10];
    if (localB->p_idx_1 < localB->vNum_b) {
      obj = robot->Bodies[localB->b_i];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, localB->T_m, localB);
    } else {
      if (localB->vNum_b > localB->p_idx_1) {
        localB->unnamed_idx_1_c = 0;
        localB->nm1d2 = -1;
      } else {
        localB->unnamed_idx_1_c = static_cast<int32_T>(localB->vNum_b) - 1;
        localB->nm1d2 = static_cast<int32_T>(localB->p_idx_1) - 1;
      }

      obj = robot->Bodies[localB->b_i];
      localB->q_size_tmp = localB->nm1d2 - localB->unnamed_idx_1_c;
      localB->q_size_c = localB->q_size_tmp + 1;
      for (localB->nm1d2 = 0; localB->nm1d2 <= localB->q_size_tmp; localB->nm1d2
           ++) {
        localB->q_data_m[localB->nm1d2] = q[localB->unnamed_idx_1_c +
          localB->nm1d2];
      }

      rigidBodyJoint_transformBodyT_o(&obj->JointInternal, localB->q_data_m,
        &localB->q_size_c, localB->T_m, localB);
    }

    left_arm_ctrl_obs_tforminv(localB->T_m, localB->dv, localB);
    left_arm_ct_tformToSpatialXform(localB->dv, X->data[localB->b_i].f1, localB);
  }

  localB->idx = static_cast<int32_T>(((-1.0 - localB->nb_n) + 1.0) / -1.0) - 1;
  left_arm_ctrl_ob_emxInit_real_T(&Si, 2, localB);
  left_arm_ctrl_ob_emxInit_real_T(&Fi, 2, localB);
  left_arm_ctrl_ob_emxInit_real_T(&Sj, 2, localB);
  left_arm_ctrl_ob_emxInit_real_T(&Hji, 2, localB);
  left_arm_ctrl_ob_emxInit_char_T(&a, 2, localB);
  left_arm_ctrl_ob_emxInit_real_T(&a_0, 2, localB);
  left_arm_ctrl_ob_emxInit_real_T(&B, 2, localB);
  for (localB->unnamed_idx_1_c = 0; localB->unnamed_idx_1_c <= localB->idx;
       localB->unnamed_idx_1_c++) {
    localB->pid_tmp = static_cast<int32_T>(localB->nb_n + -static_cast<real_T>
      (localB->unnamed_idx_1_c));
    localB->q_size_tmp = localB->pid_tmp - 1;
    localB->pid = robot->Bodies[localB->q_size_tmp]->ParentIndex;
    localB->vNum_b = robot->VelocityDoFMap[localB->pid_tmp - 1];
    localB->p_idx_1 = robot->VelocityDoFMap[localB->pid_tmp + 9];
    if (localB->pid > 0.0) {
      for (localB->nm1d2 = 0; localB->nm1d2 < 6; localB->nm1d2++) {
        for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
          localB->n_m = localB->nm1d2 + 6 * localB->b_i;
          localB->X[localB->n_m] = 0.0;
          for (localB->cb = 0; localB->cb < 6; localB->cb++) {
            localB->X[localB->n_m] += X->data[localB->q_size_tmp].f1[6 *
              localB->nm1d2 + localB->cb] * Ic->data[localB->q_size_tmp].f1[6 *
              localB->b_i + localB->cb];
          }
        }
      }

      for (localB->nm1d2 = 0; localB->nm1d2 < 6; localB->nm1d2++) {
        for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
          localB->b_idx_0_l = 0.0;
          for (localB->cb = 0; localB->cb < 6; localB->cb++) {
            localB->b_idx_0_l += localB->X[6 * localB->cb + localB->nm1d2] *
              X->data[localB->q_size_tmp].f1[6 * localB->b_i + localB->cb];
          }

          localB->cb = 6 * localB->b_i + localB->nm1d2;
          Ic->data[static_cast<int32_T>(localB->pid) - 1].f1[localB->cb] +=
            localB->b_idx_0_l;
        }
      }

      lambda_->data[localB->q_size_tmp] = localB->pid;
      if (lambda_->data[localB->q_size_tmp] > 0.0) {
        for (localB->nm1d2 = 0; localB->nm1d2 < 5; localB->nm1d2++) {
          localB->b_a[localB->nm1d2] = tmp[localB->nm1d2];
        }
      }

      exitg1 = false;
      while ((!exitg1) && (lambda_->data[localB->q_size_tmp] > 0.0)) {
        obj = robot->Bodies[static_cast<int32_T>(lambda_->data
          [localB->q_size_tmp]) - 1];
        localB->nm1d2 = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = obj->JointInternal.Type->size[1];
        left_a_emxEnsureCapacity_char_T(a, localB->nm1d2, localB);
        localB->n_m = obj->JointInternal.Type->size[0] * obj->
          JointInternal.Type->size[1] - 1;
        for (localB->nm1d2 = 0; localB->nm1d2 <= localB->n_m; localB->nm1d2++) {
          a->data[localB->nm1d2] = obj->JointInternal.Type->data[localB->nm1d2];
        }

        localB->b_bool_c = false;
        if (a->size[1] == 5) {
          localB->nm1d2 = 1;
          do {
            exitg2 = 0;
            if (localB->nm1d2 - 1 < 5) {
              localB->n_m = localB->nm1d2 - 1;
              if (a->data[localB->n_m] != localB->b_a[localB->n_m]) {
                exitg2 = 1;
              } else {
                localB->nm1d2++;
              }
            } else {
              localB->b_bool_c = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (localB->b_bool_c) {
          lambda_->data[localB->q_size_tmp] = robot->Bodies[static_cast<int32_T>
            (lambda_->data[localB->q_size_tmp]) - 1]->ParentIndex;
        } else {
          exitg1 = true;
        }
      }
    }

    localB->b_idx_0_l = robot->VelocityDoFMap[localB->pid_tmp - 1];
    localB->b_idx_1_h = robot->VelocityDoFMap[localB->pid_tmp + 9];
    if (localB->b_idx_0_l <= localB->b_idx_1_h) {
      obj = robot->Bodies[localB->q_size_tmp];
      localB->nm1d2 = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(Si, localB->nm1d2, localB);
      localB->n_m = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (localB->nm1d2 = 0; localB->nm1d2 <= localB->n_m; localB->nm1d2++) {
        Si->data[localB->nm1d2] = obj->JointInternal.MotionSubspace->data
          [localB->nm1d2];
      }

      localB->n_m = Si->size[1] - 1;
      localB->nm1d2 = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      left_a_emxEnsureCapacity_real_T(Fi, localB->nm1d2, localB);
      for (localB->b_j_m = 0; localB->b_j_m <= localB->n_m; localB->b_j_m++) {
        localB->pid_tmp = localB->b_j_m * 6 - 1;
        for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
          Fi->data[(localB->pid_tmp + localB->b_i) + 1] = 0.0;
        }

        for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
          localB->aoffset_c = localB->b_i * 6 - 1;
          localB->temp = Si->data[(localB->pid_tmp + localB->b_i) + 1];
          for (localB->c_i_j = 0; localB->c_i_j < 6; localB->c_i_j++) {
            localB->i_m = localB->c_i_j + 1;
            localB->nm1d2 = localB->pid_tmp + localB->i_m;
            Fi->data[localB->nm1d2] += Ic->data[localB->q_size_tmp].f1
              [localB->aoffset_c + localB->i_m] * localB->temp;
          }
        }
      }

      if (localB->vNum_b > localB->p_idx_1) {
        localB->pid_tmp = 0;
        localB->cb = 0;
      } else {
        localB->pid_tmp = static_cast<int32_T>(localB->vNum_b) - 1;
        localB->cb = localB->pid_tmp;
      }

      localB->nm1d2 = a_0->size[0] * a_0->size[1];
      a_0->size[0] = Si->size[1];
      a_0->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a_0, localB->nm1d2, localB);
      for (localB->nm1d2 = 0; localB->nm1d2 < 6; localB->nm1d2++) {
        localB->n_m = Si->size[1];
        for (localB->b_i = 0; localB->b_i < localB->n_m; localB->b_i++) {
          a_0->data[localB->b_i + a_0->size[0] * localB->nm1d2] = Si->data[6 *
            localB->b_i + localB->nm1d2];
        }
      }

      localB->m_h = a_0->size[0];
      localB->n_m = Fi->size[1] - 1;
      localB->nm1d2 = Hji->size[0] * Hji->size[1];
      Hji->size[0] = a_0->size[0];
      Hji->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(Hji, localB->nm1d2, localB);
      for (localB->b_j_m = 0; localB->b_j_m <= localB->n_m; localB->b_j_m++) {
        localB->coffset = localB->b_j_m * localB->m_h - 1;
        localB->boffset = localB->b_j_m * 6 - 1;
        for (localB->b_i = 0; localB->b_i < localB->m_h; localB->b_i++) {
          Hji->data[(localB->coffset + localB->b_i) + 1] = 0.0;
        }

        for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
          localB->aoffset_c = localB->b_i * localB->m_h - 1;
          localB->temp = Fi->data[(localB->boffset + localB->b_i) + 1];
          for (localB->c_i_j = 0; localB->c_i_j < localB->m_h; localB->c_i_j++)
          {
            localB->i_m = localB->c_i_j + 1;
            localB->nm1d2 = localB->coffset + localB->i_m;
            Hji->data[localB->nm1d2] += a_0->data[localB->aoffset_c +
              localB->i_m] * localB->temp;
          }
        }
      }

      localB->n_m = Hji->size[1];
      for (localB->nm1d2 = 0; localB->nm1d2 < localB->n_m; localB->nm1d2++) {
        localB->b_j_m = Hji->size[0];
        for (localB->b_i = 0; localB->b_i < localB->b_j_m; localB->b_i++) {
          H->data[(localB->pid_tmp + localB->b_i) + H->size[0] * (localB->cb +
            localB->nm1d2)] = Hji->data[Hji->size[0] * localB->nm1d2 +
            localB->b_i];
        }
      }

      for (localB->nm1d2 = 0; localB->nm1d2 < 6; localB->nm1d2++) {
        for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
          localB->X[localB->b_i + 6 * localB->nm1d2] = X->data
            [localB->q_size_tmp].f1[6 * localB->b_i + localB->nm1d2];
        }
      }

      localB->nm1d2 = B->size[0] * B->size[1];
      B->size[0] = 6;
      B->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(B, localB->nm1d2, localB);
      localB->n_m = Fi->size[0] * Fi->size[1] - 1;
      for (localB->nm1d2 = 0; localB->nm1d2 <= localB->n_m; localB->nm1d2++) {
        B->data[localB->nm1d2] = Fi->data[localB->nm1d2];
      }

      localB->n_m = Fi->size[1];
      localB->nm1d2 = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = localB->n_m;
      left_a_emxEnsureCapacity_real_T(Fi, localB->nm1d2, localB);
      for (localB->b_j_m = 0; localB->b_j_m < localB->n_m; localB->b_j_m++) {
        localB->pid_tmp = localB->b_j_m * 6 - 1;
        for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
          Fi->data[(localB->pid_tmp + localB->b_i) + 1] = 0.0;
        }

        for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
          localB->aoffset_c = localB->b_i * 6 - 1;
          localB->temp = B->data[(localB->pid_tmp + localB->b_i) + 1];
          for (localB->c_i_j = 0; localB->c_i_j < 6; localB->c_i_j++) {
            localB->i_m = localB->c_i_j + 1;
            localB->nm1d2 = localB->pid_tmp + localB->i_m;
            Fi->data[localB->nm1d2] += localB->X[localB->aoffset_c + localB->i_m]
              * localB->temp;
          }
        }
      }

      while (localB->pid > 0.0) {
        localB->b_i = static_cast<int32_T>(localB->pid);
        localB->q_size_tmp = localB->b_i - 1;
        obj = robot->Bodies[localB->q_size_tmp];
        localB->nm1d2 = Sj->size[0] * Sj->size[1];
        Sj->size[0] = 6;
        Sj->size[1] = obj->JointInternal.MotionSubspace->size[1];
        left_a_emxEnsureCapacity_real_T(Sj, localB->nm1d2, localB);
        localB->n_m = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (localB->nm1d2 = 0; localB->nm1d2 <= localB->n_m; localB->nm1d2++) {
          Sj->data[localB->nm1d2] = obj->JointInternal.MotionSubspace->
            data[localB->nm1d2];
        }

        localB->b_idx_0_l = robot->VelocityDoFMap[localB->b_i - 1];
        localB->b_idx_1_h = robot->VelocityDoFMap[localB->b_i + 9];
        if (localB->b_idx_0_l <= localB->b_idx_1_h) {
          localB->nm1d2 = a_0->size[0] * a_0->size[1];
          a_0->size[0] = Sj->size[1];
          a_0->size[1] = 6;
          left_a_emxEnsureCapacity_real_T(a_0, localB->nm1d2, localB);
          for (localB->nm1d2 = 0; localB->nm1d2 < 6; localB->nm1d2++) {
            localB->n_m = Sj->size[1];
            for (localB->b_i = 0; localB->b_i < localB->n_m; localB->b_i++) {
              a_0->data[localB->b_i + a_0->size[0] * localB->nm1d2] = Sj->data[6
                * localB->b_i + localB->nm1d2];
            }
          }

          localB->m_h = a_0->size[0];
          localB->n_m = Fi->size[1] - 1;
          localB->nm1d2 = Hji->size[0] * Hji->size[1];
          Hji->size[0] = a_0->size[0];
          Hji->size[1] = Fi->size[1];
          left_a_emxEnsureCapacity_real_T(Hji, localB->nm1d2, localB);
          for (localB->b_j_m = 0; localB->b_j_m <= localB->n_m; localB->b_j_m++)
          {
            localB->coffset = localB->b_j_m * localB->m_h - 1;
            localB->boffset = localB->b_j_m * 6 - 1;
            for (localB->b_i = 0; localB->b_i < localB->m_h; localB->b_i++) {
              Hji->data[(localB->coffset + localB->b_i) + 1] = 0.0;
            }

            for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
              localB->aoffset_c = localB->b_i * localB->m_h - 1;
              localB->temp = Fi->data[(localB->boffset + localB->b_i) + 1];
              for (localB->c_i_j = 0; localB->c_i_j < localB->m_h; localB->c_i_j
                   ++) {
                localB->i_m = localB->c_i_j + 1;
                localB->nm1d2 = localB->coffset + localB->i_m;
                Hji->data[localB->nm1d2] += a_0->data[localB->aoffset_c +
                  localB->i_m] * localB->temp;
              }
            }
          }

          if (localB->b_idx_0_l > localB->b_idx_1_h) {
            localB->pid_tmp = 0;
          } else {
            localB->pid_tmp = static_cast<int32_T>(localB->b_idx_0_l) - 1;
          }

          if (localB->vNum_b > localB->p_idx_1) {
            localB->cb = 0;
          } else {
            localB->cb = static_cast<int32_T>(localB->vNum_b) - 1;
          }

          localB->n_m = Hji->size[1];
          for (localB->nm1d2 = 0; localB->nm1d2 < localB->n_m; localB->nm1d2++)
          {
            localB->b_j_m = Hji->size[0];
            for (localB->b_i = 0; localB->b_i < localB->b_j_m; localB->b_i++) {
              H->data[(localB->pid_tmp + localB->b_i) + H->size[0] * (localB->cb
                + localB->nm1d2)] = Hji->data[Hji->size[0] * localB->nm1d2 +
                localB->b_i];
            }
          }

          if (localB->vNum_b > localB->p_idx_1) {
            localB->pid_tmp = 0;
          } else {
            localB->pid_tmp = static_cast<int32_T>(localB->vNum_b) - 1;
          }

          if (localB->b_idx_0_l > localB->b_idx_1_h) {
            localB->cb = 0;
          } else {
            localB->cb = static_cast<int32_T>(localB->b_idx_0_l) - 1;
          }

          localB->n_m = Hji->size[0];
          for (localB->nm1d2 = 0; localB->nm1d2 < localB->n_m; localB->nm1d2++)
          {
            localB->b_j_m = Hji->size[1];
            for (localB->b_i = 0; localB->b_i < localB->b_j_m; localB->b_i++) {
              H->data[(localB->pid_tmp + localB->b_i) + H->size[0] * (localB->cb
                + localB->nm1d2)] = Hji->data[Hji->size[0] * localB->b_i +
                localB->nm1d2];
            }
          }
        }

        for (localB->nm1d2 = 0; localB->nm1d2 < 6; localB->nm1d2++) {
          for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
            localB->X[localB->b_i + 6 * localB->nm1d2] = X->data
              [localB->q_size_tmp].f1[6 * localB->b_i + localB->nm1d2];
          }
        }

        localB->nm1d2 = B->size[0] * B->size[1];
        B->size[0] = 6;
        B->size[1] = Fi->size[1];
        left_a_emxEnsureCapacity_real_T(B, localB->nm1d2, localB);
        localB->n_m = Fi->size[0] * Fi->size[1] - 1;
        for (localB->nm1d2 = 0; localB->nm1d2 <= localB->n_m; localB->nm1d2++) {
          B->data[localB->nm1d2] = Fi->data[localB->nm1d2];
        }

        localB->n_m = Fi->size[1];
        localB->nm1d2 = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = localB->n_m;
        left_a_emxEnsureCapacity_real_T(Fi, localB->nm1d2, localB);
        for (localB->b_j_m = 0; localB->b_j_m < localB->n_m; localB->b_j_m++) {
          localB->pid_tmp = localB->b_j_m * 6 - 1;
          for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
            Fi->data[(localB->pid_tmp + localB->b_i) + 1] = 0.0;
          }

          for (localB->b_i = 0; localB->b_i < 6; localB->b_i++) {
            localB->aoffset_c = localB->b_i * 6 - 1;
            localB->temp = B->data[(localB->pid_tmp + localB->b_i) + 1];
            for (localB->c_i_j = 0; localB->c_i_j < 6; localB->c_i_j++) {
              localB->i_m = localB->c_i_j + 1;
              localB->nm1d2 = localB->pid_tmp + localB->i_m;
              Fi->data[localB->nm1d2] += localB->X[localB->aoffset_c +
                localB->i_m] * localB->temp;
            }
          }
        }

        localB->pid = robot->Bodies[localB->q_size_tmp]->ParentIndex;
      }
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&B);
  left_arm_ctrl_ob_emxFree_real_T(&a_0);
  left_arm_ctrl_ob_emxFree_char_T(&a);
  left_arm_ctrl_ob_emxFree_real_T(&Hji);
  left_arm_ctrl_ob_emxFree_real_T(&Sj);
  left_arm_ctrl_ob_emxFree_real_T(&Fi);
  left_arm_ctrl_ob_emxFree_real_T(&Si);
  left_arm_ct_emxFree_f_cell_wrap(&X);
  left_arm_ct_emxFree_f_cell_wrap(&Ic);
  for (localB->nm1d2 = 0; localB->nm1d2 < 10; localB->nm1d2++) {
    localB->mask[localB->nm1d2] = (robot->VelocityDoFMap[localB->nm1d2] <=
      robot->VelocityDoFMap[localB->nm1d2 + 10]);
  }

  localB->idx = 0;
  localB->nm1d2 = 1;
  exitg1 = false;
  while ((!exitg1) && (localB->nm1d2 - 1 < 10)) {
    if (localB->mask[localB->nm1d2 - 1]) {
      localB->idx++;
      localB->ii_data[localB->idx - 1] = localB->nm1d2;
      if (localB->idx >= 10) {
        exitg1 = true;
      } else {
        localB->nm1d2++;
      }
    } else {
      localB->nm1d2++;
    }
  }

  if (1 > localB->idx) {
    localB->idx = 0;
  }

  for (localB->nm1d2 = 0; localB->nm1d2 < localB->idx; localB->nm1d2++) {
    localB->nonFixedIndices_data[localB->nm1d2] = localB->ii_data[localB->nm1d2];
  }

  localB->idx--;
  left_arm_ctrl_ob_emxInit_real_T(&s, 2, localB);
  for (localB->unnamed_idx_1_c = 0; localB->unnamed_idx_1_c <= localB->idx;
       localB->unnamed_idx_1_c++) {
    localB->vNum_b = robot->VelocityDoFMap[localB->nonFixedIndices_data
      [localB->unnamed_idx_1_c] - 1];
    localB->p_idx_1 = robot->VelocityDoFMap[localB->nonFixedIndices_data
      [localB->unnamed_idx_1_c] + 9];
    if (rtIsNaN(localB->vNum_b) || rtIsNaN(localB->p_idx_1)) {
      localB->nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      left_a_emxEnsureCapacity_real_T(s, localB->nm1d2, localB);
      s->data[0] = (rtNaN);
    } else if (localB->p_idx_1 < localB->vNum_b) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(localB->vNum_b) || rtIsInf(localB->p_idx_1)) &&
               (localB->vNum_b == localB->p_idx_1)) {
      localB->nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      left_a_emxEnsureCapacity_real_T(s, localB->nm1d2, localB);
      s->data[0] = (rtNaN);
    } else if (floor(localB->vNum_b) == localB->vNum_b) {
      localB->nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      localB->n_m = static_cast<int32_T>(floor(localB->p_idx_1 - localB->vNum_b));
      s->size[1] = localB->n_m + 1;
      left_a_emxEnsureCapacity_real_T(s, localB->nm1d2, localB);
      for (localB->nm1d2 = 0; localB->nm1d2 <= localB->n_m; localB->nm1d2++) {
        s->data[localB->nm1d2] = localB->vNum_b + static_cast<real_T>
          (localB->nm1d2);
      }
    } else {
      localB->nb_n = floor((localB->p_idx_1 - localB->vNum_b) + 0.5);
      localB->pid = localB->vNum_b + localB->nb_n;
      localB->b_idx_0_l = localB->pid - localB->p_idx_1;
      localB->b_idx_1_h = fabs(localB->vNum_b);
      localB->temp = fabs(localB->p_idx_1);
      if ((localB->b_idx_1_h > localB->temp) || rtIsNaN(localB->temp)) {
        localB->temp = localB->b_idx_1_h;
      }

      if (fabs(localB->b_idx_0_l) < 4.4408920985006262E-16 * localB->temp) {
        localB->nb_n++;
        localB->pid = localB->p_idx_1;
      } else if (localB->b_idx_0_l > 0.0) {
        localB->pid = (localB->nb_n - 1.0) + localB->vNum_b;
      } else {
        localB->nb_n++;
      }

      if (localB->nb_n >= 0.0) {
        localB->nm1d2 = static_cast<int32_T>(localB->nb_n);
      } else {
        localB->nm1d2 = 0;
      }

      localB->n_m = localB->nm1d2 - 1;
      localB->nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = localB->n_m + 1;
      left_a_emxEnsureCapacity_real_T(s, localB->nm1d2, localB);
      if (localB->n_m + 1 > 0) {
        s->data[0] = localB->vNum_b;
        if (localB->n_m + 1 > 1) {
          s->data[localB->n_m] = localB->pid;
          localB->nm1d2 = localB->n_m / 2;
          localB->q_size_tmp = localB->nm1d2 - 2;
          for (localB->b_i = 0; localB->b_i <= localB->q_size_tmp; localB->b_i++)
          {
            localB->pid_tmp = localB->b_i + 1;
            s->data[localB->pid_tmp] = localB->vNum_b + static_cast<real_T>
              (localB->pid_tmp);
            s->data[localB->n_m - localB->pid_tmp] = localB->pid -
              static_cast<real_T>(localB->pid_tmp);
          }

          if (localB->nm1d2 << 1 == localB->n_m) {
            s->data[localB->nm1d2] = (localB->vNum_b + localB->pid) / 2.0;
          } else {
            s->data[localB->nm1d2] = localB->vNum_b + static_cast<real_T>
              (localB->nm1d2);
            s->data[localB->nm1d2 + 1] = localB->pid - static_cast<real_T>
              (localB->nm1d2);
          }
        }
      }
    }

    if (localB->vNum_b > localB->p_idx_1) {
      localB->q_size_tmp = 0;
    } else {
      localB->q_size_tmp = static_cast<int32_T>(localB->vNum_b) - 1;
    }

    localB->n_m = s->size[1];
    for (localB->nm1d2 = 0; localB->nm1d2 < localB->n_m; localB->nm1d2++) {
      lambda->data[localB->q_size_tmp + localB->nm1d2] = s->data[localB->nm1d2]
        - 1.0;
    }

    if (lambda_->data[localB->nonFixedIndices_data[localB->unnamed_idx_1_c] - 1]
        == 0.0) {
      lambda->data[static_cast<int32_T>(localB->vNum_b) - 1] = 0.0;
    } else {
      localB->nm1d2 = static_cast<int32_T>(lambda_->data
        [localB->nonFixedIndices_data[localB->unnamed_idx_1_c] - 1]);
      localB->b_idx_1_h = robot->VelocityDoFMap[localB->nm1d2 + 9];
      lambda->data[static_cast<int32_T>(localB->vNum_b) - 1] = localB->b_idx_1_h;
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&s);
  left_arm_ctrl_ob_emxFree_real_T(&lambda_);
}

static void RigidBodyTreeDynamics_inverseDy(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], const real_T qdot[7], const
  emxArray_real_T_left_arm_ctrl_T *qddot, const real_T fext[60], real_T tau[7],
  B_MATLABSystem_left_arm_ctrl__T *localB)
{
  emxArray_f_cell_wrap_left_arm_T *X;
  emxArray_f_cell_wrap_left_arm_T *Xtree;
  emxArray_real_T_left_arm_ctrl_T *vJ;
  emxArray_real_T_left_arm_ctrl_T *vB;
  emxArray_real_T_left_arm_ctrl_T *aB;
  emxArray_real_T_left_arm_ctrl_T *f;
  emxArray_real_T_left_arm_ctrl_T *S;
  emxArray_real_T_left_arm_ctrl_T *qddoti;
  j_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_left_arm_ctrl_T *a;
  emxArray_real_T_left_arm_ctrl_T *a_0;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  localB->a0[0] = 0.0;
  localB->a0[1] = 0.0;
  localB->a0[2] = 0.0;
  localB->a0[3] = -robot->Gravity[0];
  localB->a0[4] = -robot->Gravity[1];
  localB->a0[5] = -robot->Gravity[2];
  left_arm_ctrl_ob_emxInit_real_T(&vJ, 2, localB);
  localB->nb = robot->NumBodies;
  localB->i_i = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  localB->unnamed_idx_1 = static_cast<int32_T>(localB->nb);
  vJ->size[1] = localB->unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(vJ, localB->i_i, localB);
  localB->loop_ub_tmp = 6 * localB->unnamed_idx_1 - 1;
  for (localB->i_i = 0; localB->i_i <= localB->loop_ub_tmp; localB->i_i++) {
    vJ->data[localB->i_i] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&vB, 2, localB);
  localB->i_i = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = localB->unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(vB, localB->i_i, localB);
  for (localB->i_i = 0; localB->i_i <= localB->loop_ub_tmp; localB->i_i++) {
    vB->data[localB->i_i] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&aB, 2, localB);
  localB->i_i = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = localB->unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(aB, localB->i_i, localB);
  for (localB->i_i = 0; localB->i_i <= localB->loop_ub_tmp; localB->i_i++) {
    aB->data[localB->i_i] = 0.0;
  }

  for (localB->i_i = 0; localB->i_i < 7; localB->i_i++) {
    tau[localB->i_i] = 0.0;
  }

  left_arm_ct_emxInit_f_cell_wrap(&X, 2, localB);
  left_arm_ct_emxInit_f_cell_wrap(&Xtree, 2, localB);
  localB->loop_ub_tmp = localB->unnamed_idx_1 - 1;
  localB->i_i = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = localB->unnamed_idx_1;
  l_emxEnsureCapacity_f_cell_wrap(Xtree, localB->i_i, localB);
  localB->i_i = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = localB->unnamed_idx_1;
  l_emxEnsureCapacity_f_cell_wrap(X, localB->i_i, localB);
  for (localB->b_k = 0; localB->b_k <= localB->loop_ub_tmp; localB->b_k++) {
    for (localB->i_i = 0; localB->i_i < 36; localB->i_i++) {
      Xtree->data[localB->b_k].f1[localB->i_i] = 0.0;
    }

    for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
      Xtree->data[localB->b_k].f1[localB->i_i + 6 * localB->i_i] = 1.0;
    }

    for (localB->i_i = 0; localB->i_i < 36; localB->i_i++) {
      X->data[localB->b_k].f1[localB->i_i] = 0.0;
    }

    for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
      X->data[localB->b_k].f1[localB->i_i + 6 * localB->i_i] = 1.0;
    }
  }

  left_arm_ctrl_ob_emxInit_real_T(&f, 2, localB);
  localB->i_i = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = localB->unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(f, localB->i_i, localB);
  left_arm_ctrl_ob_emxInit_real_T(&S, 2, localB);
  left_arm_ctrl_ob_emxInit_real_T(&qddoti, 1, localB);
  if (0 <= localB->loop_ub_tmp) {
    localB->dv1[0] = 0.0;
    localB->dv1[4] = 0.0;
    localB->dv1[8] = 0.0;
  }

  for (localB->unnamed_idx_1 = 0; localB->unnamed_idx_1 <= localB->loop_ub_tmp;
       localB->unnamed_idx_1++) {
    obj = robot->Bodies[localB->unnamed_idx_1];
    localB->i_i = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    left_a_emxEnsureCapacity_real_T(S, localB->i_i, localB);
    localB->b_k = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (localB->i_i = 0; localB->i_i <= localB->b_k; localB->i_i++) {
      S->data[localB->i_i] = obj->JointInternal.MotionSubspace->data[localB->i_i];
    }

    localB->a_idx_0 = robot->PositionDoFMap[localB->unnamed_idx_1];
    localB->a_idx_1 = robot->PositionDoFMap[localB->unnamed_idx_1 + 10];
    localB->b_idx_0 = robot->VelocityDoFMap[localB->unnamed_idx_1];
    localB->b_idx_1 = robot->VelocityDoFMap[localB->unnamed_idx_1 + 10];
    if (localB->a_idx_1 < localB->a_idx_0) {
      obj = robot->Bodies[localB->unnamed_idx_1];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, localB->T, localB);
      localB->i_i = qddoti->size[0];
      qddoti->size[0] = 1;
      left_a_emxEnsureCapacity_real_T(qddoti, localB->i_i, localB);
      qddoti->data[0] = 0.0;
      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        vJ->data[localB->i_i + 6 * localB->unnamed_idx_1] = 0.0;
      }
    } else {
      if (localB->a_idx_0 > localB->a_idx_1) {
        localB->inner = 0;
        localB->m = -1;
      } else {
        localB->inner = static_cast<int32_T>(localB->a_idx_0) - 1;
        localB->m = static_cast<int32_T>(localB->a_idx_1) - 1;
      }

      if (localB->b_idx_0 > localB->b_idx_1) {
        localB->p_tmp = 0;
        localB->o_tmp = 0;
        localB->aoffset = 0;
        localB->b_k = -1;
      } else {
        localB->p_tmp = static_cast<int32_T>(localB->b_idx_0) - 1;
        localB->o_tmp = static_cast<int32_T>(localB->b_idx_1);
        localB->aoffset = localB->p_tmp;
        localB->b_k = localB->o_tmp - 1;
      }

      localB->i_i = qddoti->size[0];
      localB->b_k -= localB->aoffset;
      qddoti->size[0] = localB->b_k + 1;
      left_a_emxEnsureCapacity_real_T(qddoti, localB->i_i, localB);
      for (localB->i_i = 0; localB->i_i <= localB->b_k; localB->i_i++) {
        qddoti->data[localB->i_i] = qddot->data[localB->aoffset + localB->i_i];
      }

      obj = robot->Bodies[localB->unnamed_idx_1];
      localB->m -= localB->inner;
      localB->q_size = localB->m + 1;
      for (localB->i_i = 0; localB->i_i <= localB->m; localB->i_i++) {
        localB->q_data[localB->i_i] = q[localB->inner + localB->i_i];
      }

      rigidBodyJoint_transformBodyT_o(&obj->JointInternal, localB->q_data,
        &localB->q_size, localB->T, localB);
      if ((S->size[1] == 1) || (localB->o_tmp - localB->p_tmp == 1)) {
        for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
          localB->aoffset = localB->i_i + 6 * localB->unnamed_idx_1;
          vJ->data[localB->aoffset] = 0.0;
          localB->b_k = S->size[1];
          for (localB->inner = 0; localB->inner < localB->b_k; localB->inner++)
          {
            vJ->data[localB->aoffset] += S->data[6 * localB->inner + localB->i_i]
              * qdot[localB->p_tmp + localB->inner];
          }
        }
      } else {
        localB->inner = S->size[1] - 1;
        for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
          vJ->data[localB->i_i + 6 * localB->unnamed_idx_1] = 0.0;
        }

        for (localB->b_k = 0; localB->b_k <= localB->inner; localB->b_k++) {
          localB->aoffset = localB->b_k * 6 - 1;
          for (localB->c_i = 0; localB->c_i < 6; localB->c_i++) {
            localB->i_i = 6 * localB->unnamed_idx_1 + localB->c_i;
            vJ->data[localB->i_i] += S->data[(localB->aoffset + localB->c_i) + 1]
              * qdot[localB->p_tmp + localB->b_k];
          }
        }
      }
    }

    for (localB->i_i = 0; localB->i_i < 3; localB->i_i++) {
      localB->R_c[3 * localB->i_i] = localB->T[localB->i_i];
      localB->R_c[3 * localB->i_i + 1] = localB->T[localB->i_i + 4];
      localB->R_c[3 * localB->i_i + 2] = localB->T[localB->i_i + 8];
    }

    for (localB->i_i = 0; localB->i_i < 9; localB->i_i++) {
      localB->R_b[localB->i_i] = -localB->R_c[localB->i_i];
    }

    for (localB->i_i = 0; localB->i_i < 3; localB->i_i++) {
      localB->p_tmp = localB->i_i << 2;
      localB->Tinv[localB->p_tmp] = localB->R_c[3 * localB->i_i];
      localB->Tinv[localB->p_tmp + 1] = localB->R_c[3 * localB->i_i + 1];
      localB->Tinv[localB->p_tmp + 2] = localB->R_c[3 * localB->i_i + 2];
      localB->Tinv[localB->i_i + 12] = localB->R_b[localB->i_i + 6] * localB->T
        [14] + (localB->R_b[localB->i_i + 3] * localB->T[13] + localB->
                R_b[localB->i_i] * localB->T[12]);
    }

    localB->Tinv[3] = 0.0;
    localB->Tinv[7] = 0.0;
    localB->Tinv[11] = 0.0;
    localB->Tinv[15] = 1.0;
    localB->dv1[3] = -localB->Tinv[14];
    localB->dv1[6] = localB->Tinv[13];
    localB->dv1[1] = localB->Tinv[14];
    localB->dv1[7] = -localB->Tinv[12];
    localB->dv1[2] = -localB->Tinv[13];
    localB->dv1[5] = localB->Tinv[12];
    for (localB->i_i = 0; localB->i_i < 3; localB->i_i++) {
      for (localB->aoffset = 0; localB->aoffset < 3; localB->aoffset++) {
        localB->inner = localB->i_i + 3 * localB->aoffset;
        localB->dv2[localB->inner] = 0.0;
        localB->p_tmp = localB->aoffset << 2;
        localB->dv2[localB->inner] += localB->Tinv[localB->p_tmp] * localB->
          dv1[localB->i_i];
        localB->dv2[localB->inner] += localB->Tinv[localB->p_tmp + 1] *
          localB->dv1[localB->i_i + 3];
        localB->dv2[localB->inner] += localB->Tinv[localB->p_tmp + 2] *
          localB->dv1[localB->i_i + 6];
        X->data[localB->unnamed_idx_1].f1[localB->aoffset + 6 * localB->i_i] =
          localB->Tinv[(localB->i_i << 2) + localB->aoffset];
        X->data[localB->unnamed_idx_1].f1[localB->aoffset + 6 * (localB->i_i + 3)]
          = 0.0;
      }
    }

    for (localB->i_i = 0; localB->i_i < 3; localB->i_i++) {
      X->data[localB->unnamed_idx_1].f1[6 * localB->i_i + 3] = localB->dv2[3 *
        localB->i_i];
      localB->aoffset = localB->i_i << 2;
      localB->inner = 6 * (localB->i_i + 3);
      X->data[localB->unnamed_idx_1].f1[localB->inner + 3] = localB->Tinv
        [localB->aoffset];
      X->data[localB->unnamed_idx_1].f1[6 * localB->i_i + 4] = localB->dv2[3 *
        localB->i_i + 1];
      X->data[localB->unnamed_idx_1].f1[localB->inner + 4] = localB->Tinv
        [localB->aoffset + 1];
      X->data[localB->unnamed_idx_1].f1[6 * localB->i_i + 5] = localB->dv2[3 *
        localB->i_i + 2];
      X->data[localB->unnamed_idx_1].f1[localB->inner + 5] = localB->Tinv
        [localB->aoffset + 2];
    }

    localB->a_idx_0 = robot->Bodies[localB->unnamed_idx_1]->ParentIndex;
    if (localB->a_idx_0 > 0.0) {
      localB->m = static_cast<int32_T>(localB->a_idx_0);
      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        localB->a_idx_1 = 0.0;
        for (localB->aoffset = 0; localB->aoffset < 6; localB->aoffset++) {
          localB->a_idx_1 += vB->data[(localB->m - 1) * 6 + localB->aoffset] *
            X->data[localB->unnamed_idx_1].f1[6 * localB->aoffset + localB->i_i];
        }

        localB->vJ[localB->i_i] = vJ->data[6 * localB->unnamed_idx_1 +
          localB->i_i] + localB->a_idx_1;
      }

      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        vB->data[localB->i_i + 6 * localB->unnamed_idx_1] = localB->vJ
          [localB->i_i];
      }

      if ((S->size[1] == 1) || (qddoti->size[0] == 1)) {
        localB->b_k = S->size[1];
        for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
          localB->y[localB->i_i] = 0.0;
          for (localB->aoffset = 0; localB->aoffset < localB->b_k;
               localB->aoffset++) {
            localB->a_idx_1 = S->data[6 * localB->aoffset + localB->i_i] *
              qddoti->data[localB->aoffset] + localB->y[localB->i_i];
            localB->y[localB->i_i] = localB->a_idx_1;
          }
        }
      } else {
        localB->inner = S->size[1] - 1;
        for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
          localB->y[localB->i_i] = 0.0;
        }

        for (localB->b_k = 0; localB->b_k <= localB->inner; localB->b_k++) {
          localB->aoffset = localB->b_k * 6 - 1;
          for (localB->c_i = 0; localB->c_i < 6; localB->c_i++) {
            localB->a_idx_1 = S->data[(localB->aoffset + localB->c_i) + 1] *
              qddoti->data[localB->b_k] + localB->y[localB->c_i];
            localB->y[localB->c_i] = localB->a_idx_1;
          }
        }
      }

      localB->R_c[0] = 0.0;
      localB->p_tmp = 6 * localB->unnamed_idx_1 + 2;
      localB->R_c[3] = -vB->data[localB->p_tmp];
      localB->i_i = 6 * localB->unnamed_idx_1 + 1;
      localB->R_c[6] = vB->data[localB->i_i];
      localB->R_c[1] = vB->data[localB->p_tmp];
      localB->R_c[4] = 0.0;
      localB->R_c[7] = -vB->data[6 * localB->unnamed_idx_1];
      localB->R_c[2] = -vB->data[localB->i_i];
      localB->R_c[5] = vB->data[6 * localB->unnamed_idx_1];
      localB->R_c[8] = 0.0;
      localB->b_I[3] = 0.0;
      localB->p_tmp = 6 * localB->unnamed_idx_1 + 5;
      localB->b_I[9] = -vB->data[localB->p_tmp];
      localB->i_i = 6 * localB->unnamed_idx_1 + 4;
      localB->b_I[15] = vB->data[localB->i_i];
      localB->b_I[4] = vB->data[localB->p_tmp];
      localB->b_I[10] = 0.0;
      localB->p_tmp = 6 * localB->unnamed_idx_1 + 3;
      localB->b_I[16] = -vB->data[localB->p_tmp];
      localB->b_I[5] = -vB->data[localB->i_i];
      localB->b_I[11] = vB->data[localB->p_tmp];
      localB->b_I[17] = 0.0;
      for (localB->i_i = 0; localB->i_i < 3; localB->i_i++) {
        localB->a_idx_1 = localB->R_c[3 * localB->i_i];
        localB->b_I[6 * localB->i_i] = localB->a_idx_1;
        localB->p_tmp = 6 * (localB->i_i + 3);
        localB->b_I[localB->p_tmp] = 0.0;
        localB->b_I[localB->p_tmp + 3] = localB->a_idx_1;
        localB->a_idx_1 = localB->R_c[3 * localB->i_i + 1];
        localB->b_I[6 * localB->i_i + 1] = localB->a_idx_1;
        localB->b_I[localB->p_tmp + 1] = 0.0;
        localB->b_I[localB->p_tmp + 4] = localB->a_idx_1;
        localB->a_idx_1 = localB->R_c[3 * localB->i_i + 2];
        localB->b_I[6 * localB->i_i + 2] = localB->a_idx_1;
        localB->b_I[localB->p_tmp + 2] = 0.0;
        localB->b_I[localB->p_tmp + 5] = localB->a_idx_1;
      }

      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        localB->a_idx_1 = 0.0;
        for (localB->aoffset = 0; localB->aoffset < 6; localB->aoffset++) {
          localB->a_idx_1 += aB->data[(localB->m - 1) * 6 + localB->aoffset] *
            X->data[localB->unnamed_idx_1].f1[6 * localB->aoffset + localB->i_i];
        }

        localB->vJ[localB->i_i] = localB->a_idx_1 + localB->y[localB->i_i];
      }

      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        localB->y[localB->i_i] = 0.0;
        for (localB->aoffset = 0; localB->aoffset < 6; localB->aoffset++) {
          localB->a_idx_1 = localB->b_I[6 * localB->aoffset + localB->i_i] *
            vJ->data[6 * localB->unnamed_idx_1 + localB->aoffset] + localB->
            y[localB->i_i];
          localB->y[localB->i_i] = localB->a_idx_1;
        }

        aB->data[localB->i_i + 6 * localB->unnamed_idx_1] = localB->vJ
          [localB->i_i] + localB->y[localB->i_i];
      }

      localB->R_b[0] = 0.0;
      localB->R_b[3] = -localB->T[14];
      localB->R_b[6] = localB->T[13];
      localB->R_b[1] = localB->T[14];
      localB->R_b[4] = 0.0;
      localB->R_b[7] = -localB->T[12];
      localB->R_b[2] = -localB->T[13];
      localB->R_b[5] = localB->T[12];
      localB->R_b[8] = 0.0;
      for (localB->i_i = 0; localB->i_i < 3; localB->i_i++) {
        for (localB->aoffset = 0; localB->aoffset < 3; localB->aoffset++) {
          localB->inner = localB->i_i + 3 * localB->aoffset;
          localB->dv3[localB->inner] = 0.0;
          localB->p_tmp = localB->aoffset << 2;
          localB->dv3[localB->inner] += localB->T[localB->p_tmp] * localB->
            R_b[localB->i_i];
          localB->dv3[localB->inner] += localB->T[localB->p_tmp + 1] *
            localB->R_b[localB->i_i + 3];
          localB->dv3[localB->inner] += localB->T[localB->p_tmp + 2] *
            localB->R_b[localB->i_i + 6];
          localB->b_I[localB->aoffset + 6 * localB->i_i] = localB->T
            [(localB->i_i << 2) + localB->aoffset];
          localB->b_I[localB->aoffset + 6 * (localB->i_i + 3)] = 0.0;
        }
      }

      for (localB->i_i = 0; localB->i_i < 3; localB->i_i++) {
        localB->b_I[6 * localB->i_i + 3] = localB->dv3[3 * localB->i_i];
        localB->p_tmp = localB->i_i << 2;
        localB->inner = 6 * (localB->i_i + 3);
        localB->b_I[localB->inner + 3] = localB->T[localB->p_tmp];
        localB->b_I[6 * localB->i_i + 4] = localB->dv3[3 * localB->i_i + 1];
        localB->b_I[localB->inner + 4] = localB->T[localB->p_tmp + 1];
        localB->b_I[6 * localB->i_i + 5] = localB->dv3[3 * localB->i_i + 2];
        localB->b_I[localB->inner + 5] = localB->T[localB->p_tmp + 2];
      }

      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        for (localB->aoffset = 0; localB->aoffset < 6; localB->aoffset++) {
          localB->p_tmp = localB->i_i + 6 * localB->aoffset;
          localB->Xtree[localB->p_tmp] = 0.0;
          for (localB->inner = 0; localB->inner < 6; localB->inner++) {
            localB->Xtree[localB->p_tmp] += Xtree->data[static_cast<int32_T>
              (localB->a_idx_0) - 1].f1[6 * localB->inner + localB->i_i] *
              localB->b_I[6 * localB->aoffset + localB->inner];
          }
        }
      }

      for (localB->i_i = 0; localB->i_i < 36; localB->i_i++) {
        Xtree->data[localB->unnamed_idx_1].f1[localB->i_i] = localB->
          Xtree[localB->i_i];
      }
    } else {
      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        localB->aoffset = 6 * localB->unnamed_idx_1 + localB->i_i;
        vB->data[localB->aoffset] = vJ->data[localB->aoffset];
      }

      if ((S->size[1] == 1) || (qddoti->size[0] == 1)) {
        localB->b_k = S->size[1];
        for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
          localB->y[localB->i_i] = 0.0;
          for (localB->aoffset = 0; localB->aoffset < localB->b_k;
               localB->aoffset++) {
            localB->a_idx_1 = S->data[6 * localB->aoffset + localB->i_i] *
              qddoti->data[localB->aoffset] + localB->y[localB->i_i];
            localB->y[localB->i_i] = localB->a_idx_1;
          }
        }
      } else {
        localB->inner = S->size[1] - 1;
        for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
          localB->y[localB->i_i] = 0.0;
        }

        for (localB->b_k = 0; localB->b_k <= localB->inner; localB->b_k++) {
          localB->aoffset = localB->b_k * 6 - 1;
          for (localB->c_i = 0; localB->c_i < 6; localB->c_i++) {
            localB->a_idx_1 = S->data[(localB->aoffset + localB->c_i) + 1] *
              qddoti->data[localB->b_k] + localB->y[localB->c_i];
            localB->y[localB->c_i] = localB->a_idx_1;
          }
        }
      }

      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        localB->a_idx_1 = 0.0;
        for (localB->aoffset = 0; localB->aoffset < 6; localB->aoffset++) {
          localB->a_idx_1 += X->data[localB->unnamed_idx_1].f1[6 *
            localB->aoffset + localB->i_i] * localB->a0[localB->aoffset];
        }

        aB->data[localB->i_i + 6 * localB->unnamed_idx_1] = localB->a_idx_1 +
          localB->y[localB->i_i];
      }

      localB->R_b[0] = 0.0;
      localB->R_b[3] = -localB->T[14];
      localB->R_b[6] = localB->T[13];
      localB->R_b[1] = localB->T[14];
      localB->R_b[4] = 0.0;
      localB->R_b[7] = -localB->T[12];
      localB->R_b[2] = -localB->T[13];
      localB->R_b[5] = localB->T[12];
      localB->R_b[8] = 0.0;
      for (localB->i_i = 0; localB->i_i < 3; localB->i_i++) {
        for (localB->aoffset = 0; localB->aoffset < 3; localB->aoffset++) {
          localB->inner = localB->i_i + 3 * localB->aoffset;
          localB->dv3[localB->inner] = 0.0;
          localB->p_tmp = localB->aoffset << 2;
          localB->dv3[localB->inner] += localB->T[localB->p_tmp] * localB->
            R_b[localB->i_i];
          localB->dv3[localB->inner] += localB->T[localB->p_tmp + 1] *
            localB->R_b[localB->i_i + 3];
          localB->dv3[localB->inner] += localB->T[localB->p_tmp + 2] *
            localB->R_b[localB->i_i + 6];
          Xtree->data[localB->unnamed_idx_1].f1[localB->aoffset + 6 *
            localB->i_i] = localB->T[(localB->i_i << 2) + localB->aoffset];
          Xtree->data[localB->unnamed_idx_1].f1[localB->aoffset + 6 *
            (localB->i_i + 3)] = 0.0;
        }
      }

      for (localB->i_i = 0; localB->i_i < 3; localB->i_i++) {
        Xtree->data[localB->unnamed_idx_1].f1[6 * localB->i_i + 3] = localB->
          dv3[3 * localB->i_i];
        localB->aoffset = localB->i_i << 2;
        localB->inner = 6 * (localB->i_i + 3);
        Xtree->data[localB->unnamed_idx_1].f1[localB->inner + 3] = localB->
          T[localB->aoffset];
        Xtree->data[localB->unnamed_idx_1].f1[6 * localB->i_i + 4] = localB->
          dv3[3 * localB->i_i + 1];
        Xtree->data[localB->unnamed_idx_1].f1[localB->inner + 4] = localB->
          T[localB->aoffset + 1];
        Xtree->data[localB->unnamed_idx_1].f1[6 * localB->i_i + 5] = localB->
          dv3[3 * localB->i_i + 2];
        Xtree->data[localB->unnamed_idx_1].f1[localB->inner + 5] = localB->
          T[localB->aoffset + 2];
      }
    }

    for (localB->i_i = 0; localB->i_i < 36; localB->i_i++) {
      localB->b_I[localB->i_i] = robot->Bodies[localB->unnamed_idx_1]
        ->SpatialInertia[localB->i_i];
    }

    localB->R_c[0] = 0.0;
    localB->p_tmp = 6 * localB->unnamed_idx_1 + 2;
    localB->R_c[3] = -vB->data[localB->p_tmp];
    localB->i_i = 6 * localB->unnamed_idx_1 + 1;
    localB->R_c[6] = vB->data[localB->i_i];
    localB->R_c[1] = vB->data[localB->p_tmp];
    localB->R_c[4] = 0.0;
    localB->R_c[7] = -vB->data[6 * localB->unnamed_idx_1];
    localB->R_c[2] = -vB->data[localB->i_i];
    localB->R_c[5] = vB->data[6 * localB->unnamed_idx_1];
    localB->R_c[8] = 0.0;
    localB->R[18] = 0.0;
    localB->p_tmp = 6 * localB->unnamed_idx_1 + 5;
    localB->R[24] = -vB->data[localB->p_tmp];
    localB->i_i = 6 * localB->unnamed_idx_1 + 4;
    localB->R[30] = vB->data[localB->i_i];
    localB->R[19] = vB->data[localB->p_tmp];
    localB->R[25] = 0.0;
    localB->p_tmp = 6 * localB->unnamed_idx_1 + 3;
    localB->R[31] = -vB->data[localB->p_tmp];
    localB->R[20] = -vB->data[localB->i_i];
    localB->R[26] = vB->data[localB->p_tmp];
    localB->R[32] = 0.0;
    for (localB->i_i = 0; localB->i_i < 3; localB->i_i++) {
      localB->a_idx_1 = localB->R_c[3 * localB->i_i];
      localB->R[6 * localB->i_i] = localB->a_idx_1;
      localB->R[6 * localB->i_i + 3] = 0.0;
      localB->p_tmp = 6 * (localB->i_i + 3);
      localB->R[localB->p_tmp + 3] = localB->a_idx_1;
      localB->a_idx_1 = localB->R_c[3 * localB->i_i + 1];
      localB->R[6 * localB->i_i + 1] = localB->a_idx_1;
      localB->R[6 * localB->i_i + 4] = 0.0;
      localB->R[localB->p_tmp + 4] = localB->a_idx_1;
      localB->a_idx_1 = localB->R_c[3 * localB->i_i + 2];
      localB->R[6 * localB->i_i + 2] = localB->a_idx_1;
      localB->R[6 * localB->i_i + 5] = 0.0;
      localB->R[localB->p_tmp + 5] = localB->a_idx_1;
    }

    for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
      localB->b_I_n[localB->i_i] = 0.0;
      localB->b_I_p[localB->i_i] = 0.0;
      for (localB->aoffset = 0; localB->aoffset < 6; localB->aoffset++) {
        localB->a_idx_0 = localB->b_I[6 * localB->aoffset + localB->i_i];
        localB->p_tmp = 6 * localB->unnamed_idx_1 + localB->aoffset;
        localB->a_idx_1 = vB->data[localB->p_tmp] * localB->a_idx_0 +
          localB->b_I_n[localB->i_i];
        localB->a_idx_0 = aB->data[localB->p_tmp] * localB->a_idx_0 +
          localB->b_I_p[localB->i_i];
        localB->b_I_n[localB->i_i] = localB->a_idx_1;
        localB->b_I_p[localB->i_i] = localB->a_idx_0;
      }
    }

    for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
      localB->R_l[localB->i_i] = 0.0;
      localB->a_idx_1 = 0.0;
      for (localB->aoffset = 0; localB->aoffset < 6; localB->aoffset++) {
        localB->a_idx_1 += Xtree->data[localB->unnamed_idx_1].f1[6 * localB->i_i
          + localB->aoffset] * fext[6 * localB->unnamed_idx_1 + localB->aoffset];
        localB->R_l[localB->i_i] += localB->R[6 * localB->aoffset + localB->i_i]
          * localB->b_I_n[localB->aoffset];
      }

      f->data[localB->i_i + 6 * localB->unnamed_idx_1] = (localB->b_I_p
        [localB->i_i] + localB->R_l[localB->i_i]) - localB->a_idx_1;
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&aB);
  left_arm_ctrl_ob_emxFree_real_T(&vB);
  left_arm_ctrl_ob_emxFree_real_T(&vJ);
  left_arm_ct_emxFree_f_cell_wrap(&Xtree);
  localB->loop_ub_tmp = static_cast<int32_T>(((-1.0 - localB->nb) + 1.0) / -1.0)
    - 1;
  left_arm_ctrl_ob_emxInit_char_T(&a, 2, localB);
  left_arm_ctrl_ob_emxInit_real_T(&a_0, 2, localB);
  if (0 <= localB->loop_ub_tmp) {
    for (localB->i_i = 0; localB->i_i < 5; localB->i_i++) {
      localB->b_j[localB->i_i] = tmp[localB->i_i];
    }
  }

  for (localB->p_tmp = 0; localB->p_tmp <= localB->loop_ub_tmp; localB->p_tmp++)
  {
    localB->a_idx_0 = localB->nb + -static_cast<real_T>(localB->p_tmp);
    localB->inner = static_cast<int32_T>(localB->a_idx_0);
    localB->o_tmp = localB->inner - 1;
    obj = robot->Bodies[localB->o_tmp];
    localB->i_i = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    left_a_emxEnsureCapacity_char_T(a, localB->i_i, localB);
    localB->b_k = obj->JointInternal.Type->size[0] * obj->
      JointInternal.Type->size[1] - 1;
    for (localB->i_i = 0; localB->i_i <= localB->b_k; localB->i_i++) {
      a->data[localB->i_i] = obj->JointInternal.Type->data[localB->i_i];
    }

    localB->b_bool = false;
    if (a->size[1] == 5) {
      localB->i_i = 1;
      do {
        exitg1 = 0;
        if (localB->i_i - 1 < 5) {
          localB->unnamed_idx_1 = localB->i_i - 1;
          if (a->data[localB->unnamed_idx_1] != localB->b_j
              [localB->unnamed_idx_1]) {
            exitg1 = 1;
          } else {
            localB->i_i++;
          }
        } else {
          localB->b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!localB->b_bool) {
      obj = robot->Bodies[localB->o_tmp];
      localB->i_i = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(S, localB->i_i, localB);
      localB->b_k = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (localB->i_i = 0; localB->i_i <= localB->b_k; localB->i_i++) {
        S->data[localB->i_i] = obj->JointInternal.MotionSubspace->data
          [localB->i_i];
      }

      localB->i_i = a_0->size[0] * a_0->size[1];
      a_0->size[0] = S->size[1];
      a_0->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a_0, localB->i_i, localB);
      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        localB->b_k = S->size[1];
        for (localB->aoffset = 0; localB->aoffset < localB->b_k; localB->aoffset
             ++) {
          a_0->data[localB->aoffset + a_0->size[0] * localB->i_i] = S->data[6 *
            localB->aoffset + localB->i_i];
        }
      }

      localB->m = a_0->size[0] - 1;
      localB->i_i = qddoti->size[0];
      qddoti->size[0] = a_0->size[0];
      left_a_emxEnsureCapacity_real_T(qddoti, localB->i_i, localB);
      for (localB->unnamed_idx_1 = 0; localB->unnamed_idx_1 <= localB->m;
           localB->unnamed_idx_1++) {
        qddoti->data[localB->unnamed_idx_1] = 0.0;
      }

      for (localB->b_k = 0; localB->b_k < 6; localB->b_k++) {
        localB->aoffset = (localB->m + 1) * localB->b_k - 1;
        for (localB->c_i = 0; localB->c_i <= localB->m; localB->c_i++) {
          qddoti->data[localB->c_i] += f->data[(static_cast<int32_T>
            (localB->a_idx_0) - 1) * 6 + localB->b_k] * a_0->data
            [(localB->aoffset + localB->c_i) + 1];
        }
      }

      localB->b_idx_0 = robot->VelocityDoFMap[localB->inner - 1];
      localB->b_idx_1 = robot->VelocityDoFMap[localB->inner + 9];
      if (localB->b_idx_0 > localB->b_idx_1) {
        localB->m = 0;
        localB->i_i = 0;
      } else {
        localB->m = static_cast<int32_T>(localB->b_idx_0) - 1;
        localB->i_i = static_cast<int32_T>(localB->b_idx_1);
      }

      localB->unnamed_idx_1 = localB->i_i - localB->m;
      for (localB->i_i = 0; localB->i_i < localB->unnamed_idx_1; localB->i_i++)
      {
        tau[localB->m + localB->i_i] = qddoti->data[localB->i_i];
      }
    }

    localB->a_idx_0 = robot->Bodies[localB->o_tmp]->ParentIndex;
    if (localB->a_idx_0 > 0.0) {
      localB->m = static_cast<int32_T>(localB->a_idx_0);
      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        localB->a_idx_1 = 0.0;
        for (localB->aoffset = 0; localB->aoffset < 6; localB->aoffset++) {
          localB->a_idx_1 += f->data[(localB->inner - 1) * 6 + localB->aoffset] *
            X->data[localB->o_tmp].f1[6 * localB->i_i + localB->aoffset];
        }

        localB->a0[localB->i_i] = f->data[(localB->m - 1) * 6 + localB->i_i] +
          localB->a_idx_1;
      }

      for (localB->i_i = 0; localB->i_i < 6; localB->i_i++) {
        f->data[localB->i_i + 6 * (localB->m - 1)] = localB->a0[localB->i_i];
      }
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&a_0);
  left_arm_ctrl_ob_emxFree_char_T(&a);
  left_arm_ctrl_ob_emxFree_real_T(&qddoti);
  left_arm_ctrl_ob_emxFree_real_T(&S);
  left_arm_ctrl_ob_emxFree_real_T(&f);
  left_arm_ct_emxFree_f_cell_wrap(&X);
}

static void le_emxFreeStruct_rigidBodyJoint(rigidBodyJoint_left_arm_ctrl__T
  *pStruct)
{
  left_arm_ctrl_ob_emxFree_char_T(&pStruct->Type);
  left_arm_ctrl_ob_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct)
{
  le_emxFreeStruct_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_k_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct)
{
  le_emxFreeStruct_rigidBodyJoint(&pStruct->JointInternal);
}

//
// Start for atomic system:
//    synthesized block
//    synthesized block
//
void left_arm_ctr_MATLABSystem_Start(B_MATLABSystem_left_arm_ctrl__T *localB,
  DW_MATLABSystem_left_arm_ctrl_T *localDW)
{
  emxInitStruct_robotics_slmanip_(&localDW->obj, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_0, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_19, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_18, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_17, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_16, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_15, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_14, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_13, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_12, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_11, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_10, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_9, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_8, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_7, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_6, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_5, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_4, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_3, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_2, localB);
  emxInitStruct_j_robotics_manip_(&localDW->gobj_1, localB);

  // Start for MATLABSystem: '<S13>/MATLAB System'
  localDW->obj.isInitialized = 0;
  localDW->objisempty = true;
  localDW->obj.isInitialized = 1;
  lef_RigidBodyTree_RigidBodyTree(&localDW->obj.TreeInternal, &localDW->gobj_0,
    &localDW->gobj_19, &localDW->gobj_18, &localDW->gobj_17, &localDW->gobj_16,
    &localDW->gobj_15, &localDW->gobj_14, &localDW->gobj_13, &localDW->gobj_12,
    &localDW->gobj_11, localB);
}

//
// Output and update for atomic system:
//    synthesized block
//    synthesized block
//
void left_arm_ctrl_obs_MATLABSystem(const real_T rtu_0[7], const real_T rtu_1[7],
  const real_T rtu_2[7], const real_T rtu_3[60], B_MATLABSystem_left_arm_ctrl__T
  *localB, DW_MATLABSystem_left_arm_ctrl_T *localDW)
{
  robotics_slmanip_internal_blo_T *obj;
  emxArray_real_T_left_arm_ctrl_T *L;
  emxArray_real_T_left_arm_ctrl_T *H;
  emxArray_real_T_left_arm_ctrl_T *lambda;
  emxArray_real_T_left_arm_ctrl_T *H_0;
  emxArray_real_T_left_arm_ctrl_T *tmp;

  // MATLABSystem: '<S13>/MATLAB System'
  for (localB->i = 0; localB->i < 7; localB->i++) {
    localB->u0[localB->i] = rtu_0[localB->i];
  }

  for (localB->i = 0; localB->i < 7; localB->i++) {
    localB->u1[localB->i] = rtu_1[localB->i];
  }

  left_arm_ctrl_ob_emxInit_real_T(&H, 2, localB);
  left_arm_ctrl_ob_emxInit_real_T(&lambda, 2, localB);
  left_arm_ctrl_ob_emxInit_real_T(&tmp, 1, localB);

  // MATLABSystem: '<S13>/MATLAB System'
  obj = &localDW->obj;
  RigidBodyTreeDynamics_massMatri(&localDW->obj.TreeInternal, localB->u0, H,
    lambda, localB);
  localB->vNum = obj->TreeInternal.VelocityNumber;
  localB->j_j = tmp->size[0];
  localB->i = static_cast<int32_T>(localB->vNum);
  tmp->size[0] = localB->i;
  left_a_emxEnsureCapacity_real_T(tmp, localB->j_j, localB);
  for (localB->j_j = 0; localB->j_j < localB->i; localB->j_j++) {
    tmp->data[localB->j_j] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj->TreeInternal, localB->u0, localB->u1,
    tmp, rtu_3, localB->MATLABSystem, localB);
  left_arm_ctrl_ob_emxFree_real_T(&tmp);

  // MATLABSystem: '<S13>/MATLAB System'
  for (localB->j_j = 0; localB->j_j < 7; localB->j_j++) {
    localB->MATLABSystem[localB->j_j] = rtu_2[localB->j_j] -
      localB->MATLABSystem[localB->j_j];
  }

  if ((H->size[0] == 0) || (H->size[1] == 0)) {
    localB->iend = 0;
  } else {
    localB->u0_o = H->size[0];
    localB->iend = H->size[1];
    if (localB->u0_o > localB->iend) {
      localB->iend = localB->u0_o;
    }
  }

  left_arm_ctrl_ob_emxInit_real_T(&H_0, 2, localB);

  // MATLABSystem: '<S13>/MATLAB System'
  localB->j_j = H_0->size[0] * H_0->size[1];
  H_0->size[0] = H->size[0];
  H_0->size[1] = H->size[1];
  left_a_emxEnsureCapacity_real_T(H_0, localB->j_j, localB);
  localB->u0_o = H->size[0] * H->size[1] - 1;
  for (localB->j_j = 0; localB->j_j <= localB->u0_o; localB->j_j++) {
    H_0->data[localB->j_j] = H->data[localB->j_j];
  }

  left_arm_ctrl_ob_emxFree_real_T(&H);

  // MATLABSystem: '<S13>/MATLAB System'
  localB->n = static_cast<int32_T>(((-1.0 - static_cast<real_T>(localB->iend)) +
    1.0) / -1.0) - 1;
  for (localB->u0_o = 0; localB->u0_o <= localB->n; localB->u0_o++) {
    localB->j = static_cast<real_T>(localB->iend) + -static_cast<real_T>
      (localB->u0_o);
    localB->j_j = static_cast<int32_T>(localB->j);
    localB->MATLABSystem_tmp = localB->j_j - 1;
    H_0->data[(static_cast<int32_T>(localB->j) + H_0->size[0] *
               (static_cast<int32_T>(localB->j) - 1)) - 1] = sqrt(H_0->data
      [(localB->MATLABSystem_tmp * H_0->size[0] + localB->j_j) - 1]);
    localB->k = lambda->data[localB->MATLABSystem_tmp];
    while (localB->k > 0.0) {
      localB->i_n = static_cast<int32_T>(localB->k) - 1;
      H_0->data[(static_cast<int32_T>(localB->j) + H_0->size[0] *
                 (static_cast<int32_T>(localB->k) - 1)) - 1] = H_0->data
        [(localB->i_n * H_0->size[0] + localB->j_j) - 1] / H_0->data[((
        static_cast<int32_T>(localB->j) - 1) * H_0->size[0] +
        static_cast<int32_T>(localB->j)) - 1];
      localB->k = lambda->data[localB->i_n];
    }

    localB->k = lambda->data[localB->MATLABSystem_tmp];
    while (localB->k > 0.0) {
      localB->j = localB->k;
      while (localB->j > 0.0) {
        localB->MATLABSystem_tmp = static_cast<int32_T>(localB->j) - 1;
        H_0->data[(static_cast<int32_T>(localB->k) + H_0->size[0] * (
                    static_cast<int32_T>(localB->j) - 1)) - 1] = H_0->data
          [(localB->MATLABSystem_tmp * H_0->size[0] + static_cast<int32_T>
            (localB->k)) - 1] - H_0->data[((static_cast<int32_T>(localB->k) - 1)
          * H_0->size[0] + localB->j_j) - 1] * H_0->data[((static_cast<int32_T>
          (localB->j) - 1) * H_0->size[0] + localB->j_j) - 1];
        localB->j = lambda->data[localB->MATLABSystem_tmp];
      }

      localB->k = lambda->data[static_cast<int32_T>(localB->k) - 1];
    }
  }

  left_arm_ctrl_ob_emxInit_real_T(&L, 2, localB);

  // MATLABSystem: '<S13>/MATLAB System'
  localB->j_j = L->size[0] * L->size[1];
  L->size[0] = H_0->size[0];
  L->size[1] = H_0->size[1];
  left_a_emxEnsureCapacity_real_T(L, localB->j_j, localB);
  localB->u0_o = H_0->size[0] * H_0->size[1] - 1;
  for (localB->j_j = 0; localB->j_j <= localB->u0_o; localB->j_j++) {
    L->data[localB->j_j] = H_0->data[localB->j_j];
  }

  localB->n = H_0->size[1];
  if ((H_0->size[0] == 0) || (H_0->size[1] == 0) || (1 >= H_0->size[1])) {
  } else {
    localB->iend = 0;
    for (localB->j_j = 2; localB->j_j <= localB->n; localB->j_j++) {
      for (localB->u0_o = 0; localB->u0_o <= localB->iend; localB->u0_o++) {
        L->data[localB->u0_o + L->size[0] * (localB->j_j - 1)] = 0.0;
      }

      if (localB->iend + 1 < H_0->size[0]) {
        localB->iend++;
      }
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&H_0);

  // MATLABSystem: '<S13>/MATLAB System'
  localB->n = static_cast<int32_T>(((-1.0 - localB->vNum) + 1.0) / -1.0) - 1;
  for (localB->u0_o = 0; localB->u0_o <= localB->n; localB->u0_o++) {
    localB->j_j = static_cast<int32_T>(localB->vNum + -static_cast<real_T>
      (localB->u0_o));
    localB->iend = localB->j_j - 1;
    localB->MATLABSystem[localB->iend] /= L->data[(localB->iend * L->size[0] +
      localB->j_j) - 1];
    localB->j = lambda->data[localB->iend];
    while (localB->j > 0.0) {
      localB->MATLABSystem_tmp = static_cast<int32_T>(localB->j) - 1;
      localB->MATLABSystem[localB->MATLABSystem_tmp] -= L->data
        [(localB->MATLABSystem_tmp * L->size[0] + localB->j_j) - 1] *
        localB->MATLABSystem[localB->iend];
      localB->j = lambda->data[localB->MATLABSystem_tmp];
    }
  }

  localB->i--;
  for (localB->u0_o = 0; localB->u0_o <= localB->i; localB->u0_o++) {
    localB->j = lambda->data[localB->u0_o];
    while (localB->j > 0.0) {
      localB->iend = static_cast<int32_T>(localB->j) - 1;
      localB->MATLABSystem[localB->u0_o] -= L->data[localB->iend * L->size[0] +
        localB->u0_o] * localB->MATLABSystem[localB->iend];
      localB->j = lambda->data[localB->iend];
    }

    localB->MATLABSystem[localB->u0_o] /= L->data[L->size[0] * localB->u0_o +
      localB->u0_o];
  }

  left_arm_ctrl_ob_emxFree_real_T(&lambda);
  left_arm_ctrl_ob_emxFree_real_T(&L);
}

//
// Termination for atomic system:
//    synthesized block
//    synthesized block
//
void left_arm_ctrl_MATLABSystem_Term(DW_MATLABSystem_left_arm_ctrl_T *localDW)
{
  emxFreeStruct_robotics_slmanip_(&localDW->obj);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_0);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_19);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_18);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_17);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_16);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_15);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_14);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_13);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_12);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_11);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_10);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_9);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_8);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_7);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_6);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_5);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_4);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_3);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_2);
  emxFreeStruct_j_robotics_manip_(&localDW->gobj_1);
}

static void left_arm_ctrl_o_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_left_arm_ctrl_obs_299.getLatestMessage
    (&left_arm_ctrl_obs_B.b_varargout_2);
  for (left_arm_ctrl_obs_B.i_p = 0; left_arm_ctrl_obs_B.i_p < 7;
       left_arm_ctrl_obs_B.i_p++) {
    varargout_2_Data[left_arm_ctrl_obs_B.i_p] =
      left_arm_ctrl_obs_B.b_varargout_2.Data[left_arm_ctrl_obs_B.i_p];
  }

  *varargout_2_Data_SL_Info_Curren =
    left_arm_ctrl_obs_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    left_arm_ctrl_obs_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    left_arm_ctrl_obs_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &left_arm_ctrl_obs_B.b_varargout_2.Layout.Dim[0], sizeof
         (SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    left_arm_ctrl_obs_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    left_arm_ctrl_obs_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void left_arm_ctrl_SystemCore_step_e(boolean_T *varargout_1, real32_T
  varargout_2_Data[14], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_left_arm_ctrl_obs_318.getLatestMessage
    (&left_arm_ctrl_obs_B.b_varargout_2_m);
  for (left_arm_ctrl_obs_B.i_p4 = 0; left_arm_ctrl_obs_B.i_p4 < 14;
       left_arm_ctrl_obs_B.i_p4++) {
    varargout_2_Data[left_arm_ctrl_obs_B.i_p4] =
      left_arm_ctrl_obs_B.b_varargout_2_m.Data[left_arm_ctrl_obs_B.i_p4];
  }

  *varargout_2_Data_SL_Info_Curren =
    left_arm_ctrl_obs_B.b_varargout_2_m.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    left_arm_ctrl_obs_B.b_varargout_2_m.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    left_arm_ctrl_obs_B.b_varargout_2_m.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &left_arm_ctrl_obs_B.b_varargout_2_m.Layout.Dim[0], sizeof
         (SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    left_arm_ctrl_obs_B.b_varargout_2_m.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    left_arm_ctrl_obs_B.b_varargout_2_m.Layout.Dim_SL_Info.ReceivedLength;
}

static void left_arm_ctrl__emxInit_real_T_e(emxArray_real_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_left_arm_ctrl_T *emxArray;
  *pEmxArray = (emxArray_real_T_left_arm_ctrl_T *)malloc(sizeof
    (emxArray_real_T_left_arm_ctrl_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (left_arm_ctrl_obs_B.i_m = 0; left_arm_ctrl_obs_B.i_m < numDimensions;
       left_arm_ctrl_obs_B.i_m++) {
    emxArray->size[left_arm_ctrl_obs_B.i_m] = 0;
  }
}

static void left_emxEnsureCapacity_real_T_e(emxArray_real_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_ctrl_obs_B.newNumel = 1;
  for (left_arm_ctrl_obs_B.i_pt = 0; left_arm_ctrl_obs_B.i_pt <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_pt++) {
    left_arm_ctrl_obs_B.newNumel *= emxArray->size[left_arm_ctrl_obs_B.i_pt];
  }

  if (left_arm_ctrl_obs_B.newNumel > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_pt = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_pt < 16) {
      left_arm_ctrl_obs_B.i_pt = 16;
    }

    while (left_arm_ctrl_obs_B.i_pt < left_arm_ctrl_obs_B.newNumel) {
      if (left_arm_ctrl_obs_B.i_pt > 1073741823) {
        left_arm_ctrl_obs_B.i_pt = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_pt <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_pt), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_pt;
    emxArray->canFreeData = true;
  }
}

static void left_arm_c_emxInit_e_cell_wrap1(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_left_arm_T *emxArray;
  *pEmxArray = (emxArray_f_cell_wrap_left_arm_T *)malloc(sizeof
    (emxArray_f_cell_wrap_left_arm_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_left_arm_ctrl_obs_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < numDimensions;
       left_arm_ctrl_obs_B.i_f++) {
    emxArray->size[left_arm_ctrl_obs_B.i_f] = 0;
  }
}

static void emxEnsureCapacity_e_cell_wrap1(emxArray_f_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_ctrl_obs_B.newNumel_o = 1;
  for (left_arm_ctrl_obs_B.i_k = 0; left_arm_ctrl_obs_B.i_k <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_k++) {
    left_arm_ctrl_obs_B.newNumel_o *= emxArray->size[left_arm_ctrl_obs_B.i_k];
  }

  if (left_arm_ctrl_obs_B.newNumel_o > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_k = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_k < 16) {
      left_arm_ctrl_obs_B.i_k = 16;
    }

    while (left_arm_ctrl_obs_B.i_k < left_arm_ctrl_obs_B.newNumel_o) {
      if (left_arm_ctrl_obs_B.i_k > 1073741823) {
        left_arm_ctrl_obs_B.i_k = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_k <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_k), sizeof
                     (f_cell_wrap_left_arm_ctrl_obs_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_left_arm_ctrl_obs_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_left_arm_ctrl_obs_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_k;
    emxArray->canFreeData = true;
  }
}

static void left_arm_ctrl_obs_eye(real_T b_I[36])
{
  int32_T b_k;
  memset(&b_I[0], 0, 36U * sizeof(real_T));
  for (b_k = 0; b_k < 6; b_k++) {
    b_I[b_k + 6 * b_k] = 1.0;
  }
}

static void left_arm_ctrl__emxInit_char_T_e(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_left_arm_ctrl_T *emxArray;
  *pEmxArray = (emxArray_char_T_left_arm_ctrl_T *)malloc(sizeof
    (emxArray_char_T_left_arm_ctrl_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (left_arm_ctrl_obs_B.i_cu = 0; left_arm_ctrl_obs_B.i_cu < numDimensions;
       left_arm_ctrl_obs_B.i_cu++) {
    emxArray->size[left_arm_ctrl_obs_B.i_cu] = 0;
  }
}

static void left_emxEnsureCapacity_char_T_e(emxArray_char_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_ctrl_obs_B.newNumel_f = 1;
  for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_i++) {
    left_arm_ctrl_obs_B.newNumel_f *= emxArray->size[left_arm_ctrl_obs_B.i_i];
  }

  if (left_arm_ctrl_obs_B.newNumel_f > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_i = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_i < 16) {
      left_arm_ctrl_obs_B.i_i = 16;
    }

    while (left_arm_ctrl_obs_B.i_i < left_arm_ctrl_obs_B.newNumel_f) {
      if (left_arm_ctrl_obs_B.i_i > 1073741823) {
        left_arm_ctrl_obs_B.i_i = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_i), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_i;
    emxArray->canFreeData = true;
  }
}

static void rigidBodyJoint_get_JointAxis_e0(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (left_arm_ctrl_obs_B.b_kstr_p = 0; left_arm_ctrl_obs_B.b_kstr_p < 8;
       left_arm_ctrl_obs_B.b_kstr_p++) {
    left_arm_ctrl_obs_B.b_c0[left_arm_ctrl_obs_B.b_kstr_p] =
      tmp[left_arm_ctrl_obs_B.b_kstr_p];
  }

  left_arm_ctrl_obs_B.b_bool_b = false;
  if (obj->Type->size[1] == 8) {
    left_arm_ctrl_obs_B.b_kstr_p = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_p - 1 < 8) {
        left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_p - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr_k] !=
            left_arm_ctrl_obs_B.b_c0[left_arm_ctrl_obs_B.kstr_k]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_p++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_b = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (left_arm_ctrl_obs_B.b_bool_b) {
    guard1 = true;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr_p = 0; left_arm_ctrl_obs_B.b_kstr_p < 9;
         left_arm_ctrl_obs_B.b_kstr_p++) {
      left_arm_ctrl_obs_B.b_c[left_arm_ctrl_obs_B.b_kstr_p] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr_p];
    }

    left_arm_ctrl_obs_B.b_bool_b = false;
    if (obj->Type->size[1] == 9) {
      left_arm_ctrl_obs_B.b_kstr_p = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_p - 1 < 9) {
          left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_p - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr_k] !=
              left_arm_ctrl_obs_B.b_c[left_arm_ctrl_obs_B.kstr_k]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_p++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_b = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_b) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void left_arm_ctrl_o_normalizeRows_e(const real_T matrix[3], real_T
  normRowMatrix[3])
{
  left_arm_ctrl_obs_B.b_mj = 1.0 / sqrt((matrix[0] * matrix[0] + matrix[1] *
    matrix[1]) + matrix[2] * matrix[2]);
  normRowMatrix[0] = matrix[0] * left_arm_ctrl_obs_B.b_mj;
  normRowMatrix[1] = matrix[1] * left_arm_ctrl_obs_B.b_mj;
  normRowMatrix[2] = matrix[2] * left_arm_ctrl_obs_B.b_mj;
}

static void left_arm_ctrl_obs_cat_e(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9])
{
  y[0] = varargin_1;
  y[1] = varargin_2;
  y[2] = varargin_3;
  y[3] = varargin_4;
  y[4] = varargin_5;
  y[5] = varargin_6;
  y[6] = varargin_7;
  y[7] = varargin_8;
  y[8] = varargin_9;
}

static void rigidBodyJoint_transformBodyT_e(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 5;
       left_arm_ctrl_obs_B.b_kstr++) {
    left_arm_ctrl_obs_B.b_l[left_arm_ctrl_obs_B.b_kstr] =
      tmp[left_arm_ctrl_obs_B.b_kstr];
  }

  left_arm_ctrl_obs_B.b_bool_p = false;
  if (obj->Type->size[1] == 5) {
    left_arm_ctrl_obs_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr - 1 < 5) {
        left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr] !=
            left_arm_ctrl_obs_B.b_l[left_arm_ctrl_obs_B.kstr]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_p = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (left_arm_ctrl_obs_B.b_bool_p) {
    left_arm_ctrl_obs_B.b_kstr = 0;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 8;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.b_h[left_arm_ctrl_obs_B.b_kstr] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr];
    }

    left_arm_ctrl_obs_B.b_bool_p = false;
    if (obj->Type->size[1] == 8) {
      left_arm_ctrl_obs_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr - 1 < 8) {
          left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr] !=
              left_arm_ctrl_obs_B.b_h[left_arm_ctrl_obs_B.kstr]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_p = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_p) {
      left_arm_ctrl_obs_B.b_kstr = 1;
    } else {
      left_arm_ctrl_obs_B.b_kstr = -1;
    }
  }

  switch (left_arm_ctrl_obs_B.b_kstr) {
   case 0:
    memset(&left_arm_ctrl_obs_B.TJ_p[0], 0, sizeof(real_T) << 4U);
    left_arm_ctrl_obs_B.TJ_p[0] = 1.0;
    left_arm_ctrl_obs_B.TJ_p[5] = 1.0;
    left_arm_ctrl_obs_B.TJ_p[10] = 1.0;
    left_arm_ctrl_obs_B.TJ_p[15] = 1.0;
    break;

   case 1:
    rigidBodyJoint_get_JointAxis_e0(obj, left_arm_ctrl_obs_B.v_f);
    left_arm_ctrl_obs_B.v_a[0] = left_arm_ctrl_obs_B.v_f[0];
    left_arm_ctrl_obs_B.v_a[1] = left_arm_ctrl_obs_B.v_f[1];
    left_arm_ctrl_obs_B.v_a[2] = left_arm_ctrl_obs_B.v_f[2];
    left_arm_ctrl_o_normalizeRows_e(left_arm_ctrl_obs_B.v_a,
      left_arm_ctrl_obs_B.v_f);
    left_arm_ctrl_obs_B.tempR_tmp_l = left_arm_ctrl_obs_B.v_f[1] *
      left_arm_ctrl_obs_B.v_f[0] * 0.0;
    left_arm_ctrl_obs_B.tempR_tmp_o = left_arm_ctrl_obs_B.v_f[2] *
      left_arm_ctrl_obs_B.v_f[0] * 0.0;
    left_arm_ctrl_obs_B.tempR_tmp_o2 = left_arm_ctrl_obs_B.v_f[2] *
      left_arm_ctrl_obs_B.v_f[1] * 0.0;
    left_arm_ctrl_obs_cat_e(left_arm_ctrl_obs_B.v_f[0] *
      left_arm_ctrl_obs_B.v_f[0] * 0.0 + 1.0, left_arm_ctrl_obs_B.tempR_tmp_l -
      left_arm_ctrl_obs_B.v_f[2] * 0.0, left_arm_ctrl_obs_B.tempR_tmp_o +
      left_arm_ctrl_obs_B.v_f[1] * 0.0, left_arm_ctrl_obs_B.tempR_tmp_l +
      left_arm_ctrl_obs_B.v_f[2] * 0.0, left_arm_ctrl_obs_B.v_f[1] *
      left_arm_ctrl_obs_B.v_f[1] * 0.0 + 1.0, left_arm_ctrl_obs_B.tempR_tmp_o2 -
      left_arm_ctrl_obs_B.v_f[0] * 0.0, left_arm_ctrl_obs_B.tempR_tmp_o -
      left_arm_ctrl_obs_B.v_f[1] * 0.0, left_arm_ctrl_obs_B.tempR_tmp_o2 +
      left_arm_ctrl_obs_B.v_f[0] * 0.0, left_arm_ctrl_obs_B.v_f[2] *
      left_arm_ctrl_obs_B.v_f[2] * 0.0 + 1.0, left_arm_ctrl_obs_B.tempR_d);
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 3;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr + 1;
      left_arm_ctrl_obs_B.R_dh[left_arm_ctrl_obs_B.kstr - 1] =
        left_arm_ctrl_obs_B.tempR_d[(left_arm_ctrl_obs_B.kstr - 1) * 3];
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr + 1;
      left_arm_ctrl_obs_B.R_dh[left_arm_ctrl_obs_B.kstr + 2] =
        left_arm_ctrl_obs_B.tempR_d[(left_arm_ctrl_obs_B.kstr - 1) * 3 + 1];
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr + 1;
      left_arm_ctrl_obs_B.R_dh[left_arm_ctrl_obs_B.kstr + 5] =
        left_arm_ctrl_obs_B.tempR_d[(left_arm_ctrl_obs_B.kstr - 1) * 3 + 2];
    }

    memset(&left_arm_ctrl_obs_B.TJ_p[0], 0, sizeof(real_T) << 4U);
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 3;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr << 2;
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr] =
        left_arm_ctrl_obs_B.R_dh[3 * left_arm_ctrl_obs_B.b_kstr];
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr + 1] =
        left_arm_ctrl_obs_B.R_dh[3 * left_arm_ctrl_obs_B.b_kstr + 1];
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr + 2] =
        left_arm_ctrl_obs_B.R_dh[3 * left_arm_ctrl_obs_B.b_kstr + 2];
    }

    left_arm_ctrl_obs_B.TJ_p[15] = 1.0;
    break;

   default:
    rigidBodyJoint_get_JointAxis_e0(obj, left_arm_ctrl_obs_B.v_f);
    memset(&left_arm_ctrl_obs_B.tempR_d[0], 0, 9U * sizeof(real_T));
    left_arm_ctrl_obs_B.tempR_d[0] = 1.0;
    left_arm_ctrl_obs_B.tempR_d[4] = 1.0;
    left_arm_ctrl_obs_B.tempR_d[8] = 1.0;
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 3;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr << 2;
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr] =
        left_arm_ctrl_obs_B.tempR_d[3 * left_arm_ctrl_obs_B.b_kstr];
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr + 1] =
        left_arm_ctrl_obs_B.tempR_d[3 * left_arm_ctrl_obs_B.b_kstr + 1];
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr + 2] =
        left_arm_ctrl_obs_B.tempR_d[3 * left_arm_ctrl_obs_B.b_kstr + 2];
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.b_kstr + 12] =
        left_arm_ctrl_obs_B.v_f[left_arm_ctrl_obs_B.b_kstr] * 0.0;
    }

    left_arm_ctrl_obs_B.TJ_p[3] = 0.0;
    left_arm_ctrl_obs_B.TJ_p[7] = 0.0;
    left_arm_ctrl_obs_B.TJ_p[11] = 0.0;
    left_arm_ctrl_obs_B.TJ_p[15] = 1.0;
    break;
  }

  for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 4;
       left_arm_ctrl_obs_B.b_kstr++) {
    for (left_arm_ctrl_obs_B.kstr = 0; left_arm_ctrl_obs_B.kstr < 4;
         left_arm_ctrl_obs_B.kstr++) {
      left_arm_ctrl_obs_B.obj_tmp_tmp = left_arm_ctrl_obs_B.kstr << 2;
      left_arm_ctrl_obs_B.obj_tmp = left_arm_ctrl_obs_B.b_kstr +
        left_arm_ctrl_obs_B.obj_tmp_tmp;
      left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.obj_tmp] = 0.0;
      left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.obj_tmp] +=
        left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.obj_tmp_tmp] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr];
      left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.obj_tmp] +=
        left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.obj_tmp_tmp + 1] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr + 4];
      left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.obj_tmp] +=
        left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.obj_tmp_tmp + 2] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr + 8];
      left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.obj_tmp] +=
        left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.obj_tmp_tmp + 3] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr + 12];
    }

    for (left_arm_ctrl_obs_B.kstr = 0; left_arm_ctrl_obs_B.kstr < 4;
         left_arm_ctrl_obs_B.kstr++) {
      left_arm_ctrl_obs_B.obj_tmp_tmp = left_arm_ctrl_obs_B.kstr << 2;
      left_arm_ctrl_obs_B.obj_tmp = left_arm_ctrl_obs_B.b_kstr +
        left_arm_ctrl_obs_B.obj_tmp_tmp;
      T[left_arm_ctrl_obs_B.obj_tmp] = 0.0;
      T[left_arm_ctrl_obs_B.obj_tmp] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp] *
        left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.b_kstr];
      T[left_arm_ctrl_obs_B.obj_tmp] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp + 1] *
        left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.b_kstr + 4];
      T[left_arm_ctrl_obs_B.obj_tmp] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp + 2] *
        left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.b_kstr + 8];
      T[left_arm_ctrl_obs_B.obj_tmp] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp + 3] *
        left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.b_kstr + 12];
    }
  }
}

static void left_arm_ctrl_obs_tforminv_e(const real_T T[16], real_T Tinv[16])
{
  for (left_arm_ctrl_obs_B.i2 = 0; left_arm_ctrl_obs_B.i2 < 3;
       left_arm_ctrl_obs_B.i2++) {
    left_arm_ctrl_obs_B.R_h[3 * left_arm_ctrl_obs_B.i2] =
      T[left_arm_ctrl_obs_B.i2];
    left_arm_ctrl_obs_B.R_h[3 * left_arm_ctrl_obs_B.i2 + 1] =
      T[left_arm_ctrl_obs_B.i2 + 4];
    left_arm_ctrl_obs_B.R_h[3 * left_arm_ctrl_obs_B.i2 + 2] =
      T[left_arm_ctrl_obs_B.i2 + 8];
  }

  for (left_arm_ctrl_obs_B.i2 = 0; left_arm_ctrl_obs_B.i2 < 9;
       left_arm_ctrl_obs_B.i2++) {
    left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i2] =
      -left_arm_ctrl_obs_B.R_h[left_arm_ctrl_obs_B.i2];
  }

  for (left_arm_ctrl_obs_B.i2 = 0; left_arm_ctrl_obs_B.i2 < 3;
       left_arm_ctrl_obs_B.i2++) {
    left_arm_ctrl_obs_B.Tinv_tmp = left_arm_ctrl_obs_B.i2 << 2;
    Tinv[left_arm_ctrl_obs_B.Tinv_tmp] = left_arm_ctrl_obs_B.R_h[3 *
      left_arm_ctrl_obs_B.i2];
    Tinv[left_arm_ctrl_obs_B.Tinv_tmp + 1] = left_arm_ctrl_obs_B.R_h[3 *
      left_arm_ctrl_obs_B.i2 + 1];
    Tinv[left_arm_ctrl_obs_B.Tinv_tmp + 2] = left_arm_ctrl_obs_B.R_h[3 *
      left_arm_ctrl_obs_B.i2 + 2];
    Tinv[left_arm_ctrl_obs_B.i2 + 12] =
      left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i2 + 6] * T[14] +
      (left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i2 + 3] * T[13] +
       left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i2] * T[12]);
  }

  Tinv[3] = 0.0;
  Tinv[7] = 0.0;
  Tinv[11] = 0.0;
  Tinv[15] = 1.0;
}

static void left_arm__tformToSpatialXform_e(const real_T T[16], real_T X[36])
{
  left_arm_ctrl_obs_B.dv3[0] = 0.0;
  left_arm_ctrl_obs_B.dv3[3] = -T[14];
  left_arm_ctrl_obs_B.dv3[6] = T[13];
  left_arm_ctrl_obs_B.dv3[1] = T[14];
  left_arm_ctrl_obs_B.dv3[4] = 0.0;
  left_arm_ctrl_obs_B.dv3[7] = -T[12];
  left_arm_ctrl_obs_B.dv3[2] = -T[13];
  left_arm_ctrl_obs_B.dv3[5] = T[12];
  left_arm_ctrl_obs_B.dv3[8] = 0.0;
  for (left_arm_ctrl_obs_B.i_pb = 0; left_arm_ctrl_obs_B.i_pb < 3;
       left_arm_ctrl_obs_B.i_pb++) {
    for (left_arm_ctrl_obs_B.X_tmp_m = 0; left_arm_ctrl_obs_B.X_tmp_m < 3;
         left_arm_ctrl_obs_B.X_tmp_m++) {
      left_arm_ctrl_obs_B.X_tmp_o = left_arm_ctrl_obs_B.i_pb + 3 *
        left_arm_ctrl_obs_B.X_tmp_m;
      left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.X_tmp_o] = 0.0;
      left_arm_ctrl_obs_B.i1 = left_arm_ctrl_obs_B.X_tmp_m << 2;
      left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.X_tmp_o] +=
        T[left_arm_ctrl_obs_B.i1] *
        left_arm_ctrl_obs_B.dv3[left_arm_ctrl_obs_B.i_pb];
      left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.X_tmp_o] +=
        T[left_arm_ctrl_obs_B.i1 + 1] *
        left_arm_ctrl_obs_B.dv3[left_arm_ctrl_obs_B.i_pb + 3];
      left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.X_tmp_o] +=
        T[left_arm_ctrl_obs_B.i1 + 2] *
        left_arm_ctrl_obs_B.dv3[left_arm_ctrl_obs_B.i_pb + 6];
      X[left_arm_ctrl_obs_B.X_tmp_m + 6 * left_arm_ctrl_obs_B.i_pb] = T
        [(left_arm_ctrl_obs_B.i_pb << 2) + left_arm_ctrl_obs_B.X_tmp_m];
      X[left_arm_ctrl_obs_B.X_tmp_m + 6 * (left_arm_ctrl_obs_B.i_pb + 3)] = 0.0;
    }
  }

  for (left_arm_ctrl_obs_B.i_pb = 0; left_arm_ctrl_obs_B.i_pb < 3;
       left_arm_ctrl_obs_B.i_pb++) {
    X[6 * left_arm_ctrl_obs_B.i_pb + 3] = left_arm_ctrl_obs_B.dv4[3 *
      left_arm_ctrl_obs_B.i_pb];
    left_arm_ctrl_obs_B.X_tmp_m = left_arm_ctrl_obs_B.i_pb << 2;
    left_arm_ctrl_obs_B.X_tmp_o = 6 * (left_arm_ctrl_obs_B.i_pb + 3);
    X[left_arm_ctrl_obs_B.X_tmp_o + 3] = T[left_arm_ctrl_obs_B.X_tmp_m];
    X[6 * left_arm_ctrl_obs_B.i_pb + 4] = left_arm_ctrl_obs_B.dv4[3 *
      left_arm_ctrl_obs_B.i_pb + 1];
    X[left_arm_ctrl_obs_B.X_tmp_o + 4] = T[left_arm_ctrl_obs_B.X_tmp_m + 1];
    X[6 * left_arm_ctrl_obs_B.i_pb + 5] = left_arm_ctrl_obs_B.dv4[3 *
      left_arm_ctrl_obs_B.i_pb + 2];
    X[left_arm_ctrl_obs_B.X_tmp_o + 5] = T[left_arm_ctrl_obs_B.X_tmp_m + 2];
  }
}

static void left_arm_ctrl__emxFree_char_T_e(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_left_arm_ctrl_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_left_arm_ctrl_T *)NULL;
  }
}

static void left_arm_ctrl__emxFree_real_T_e(emxArray_real_T_left_arm_ctrl_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_left_arm_ctrl_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_left_arm_ctrl_T *)NULL;
  }
}

static void left_arm_c_emxFree_e_cell_wrap1(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_left_arm_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_left_arm_ctrl_obs_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_left_arm_T *)NULL;
  }
}

static boolean_T left_arm_ctrl_obs_strcmp(const emxArray_char_T_left_arm_ctrl_T *
  a)
{
  boolean_T b_bool;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  for (left_arm_ctrl_obs_B.b_kstr_o = 0; left_arm_ctrl_obs_B.b_kstr_o < 5;
       left_arm_ctrl_obs_B.b_kstr_o++) {
    left_arm_ctrl_obs_B.b_mc[left_arm_ctrl_obs_B.b_kstr_o] =
      tmp[left_arm_ctrl_obs_B.b_kstr_o];
  }

  b_bool = false;
  if (a->size[1] == 5) {
    left_arm_ctrl_obs_B.b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_o - 1 < 5) {
        left_arm_ctrl_obs_B.kstr_i = left_arm_ctrl_obs_B.b_kstr_o - 1;
        if (a->data[left_arm_ctrl_obs_B.kstr_i] !=
            left_arm_ctrl_obs_B.b_mc[left_arm_ctrl_obs_B.kstr_i]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_o++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

static void lef_GravityTorqueBlock_stepImpl(robotics_slmanip_internal_e0h_T *obj,
  const real32_T q[7], real32_T jointTorq[7])
{
  k_robotics_manip_internal_e0h_T *robot;
  emxArray_f_cell_wrap_left_arm_T *X;
  emxArray_f_cell_wrap_left_arm_T *Xtree;
  emxArray_real_T_left_arm_ctrl_T *vJ;
  emxArray_real_T_left_arm_ctrl_T *vB;
  emxArray_real_T_left_arm_ctrl_T *aB;
  emxArray_real_T_left_arm_ctrl_T *f;
  emxArray_real_T_left_arm_ctrl_T *S;
  emxArray_real_T_left_arm_ctrl_T *taui;
  j_robotics_manip_internal_Rig_T *obj_0;
  emxArray_real_T_left_arm_ctrl_T *a;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  robot = &obj->TreeInternal;
  for (left_arm_ctrl_obs_B.i_c = 0; left_arm_ctrl_obs_B.i_c < 7;
       left_arm_ctrl_obs_B.i_c++) {
    left_arm_ctrl_obs_B.q[left_arm_ctrl_obs_B.i_c] = q[left_arm_ctrl_obs_B.i_c];
  }

  left_arm_ctrl_obs_B.a0[0] = 0.0;
  left_arm_ctrl_obs_B.a0[1] = 0.0;
  left_arm_ctrl_obs_B.a0[2] = 0.0;
  left_arm_ctrl_obs_B.a0[3] = -obj->TreeInternal.Gravity[0];
  left_arm_ctrl_obs_B.a0[4] = -obj->TreeInternal.Gravity[1];
  left_arm_ctrl_obs_B.a0[5] = -obj->TreeInternal.Gravity[2];
  left_arm_ctrl__emxInit_real_T_e(&vJ, 2);
  left_arm_ctrl_obs_B.nb = obj->TreeInternal.NumBodies;
  left_arm_ctrl_obs_B.u = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  left_arm_ctrl_obs_B.unnamed_idx_1 = static_cast<int32_T>
    (left_arm_ctrl_obs_B.nb);
  vJ->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  left_emxEnsureCapacity_real_T_e(vJ, left_arm_ctrl_obs_B.u);
  left_arm_ctrl_obs_B.aoffset = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 - 1;
  for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <=
       left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
    vJ->data[left_arm_ctrl_obs_B.u] = 0.0;
  }

  left_arm_ctrl__emxInit_real_T_e(&vB, 2);
  left_arm_ctrl_obs_B.u = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  left_emxEnsureCapacity_real_T_e(vB, left_arm_ctrl_obs_B.u);
  for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <=
       left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
    vB->data[left_arm_ctrl_obs_B.u] = 0.0;
  }

  left_arm_ctrl__emxInit_real_T_e(&aB, 2);
  left_arm_ctrl_obs_B.u = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  left_emxEnsureCapacity_real_T_e(aB, left_arm_ctrl_obs_B.u);
  for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <=
       left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
    aB->data[left_arm_ctrl_obs_B.u] = 0.0;
  }

  for (left_arm_ctrl_obs_B.i_c = 0; left_arm_ctrl_obs_B.i_c < 7;
       left_arm_ctrl_obs_B.i_c++) {
    left_arm_ctrl_obs_B.tau[left_arm_ctrl_obs_B.i_c] = 0.0;
  }

  left_arm_c_emxInit_e_cell_wrap1(&X, 2);
  left_arm_c_emxInit_e_cell_wrap1(&Xtree, 2);
  left_arm_ctrl_obs_B.i_c = left_arm_ctrl_obs_B.unnamed_idx_1 - 1;
  left_arm_ctrl_obs_B.u = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  emxEnsureCapacity_e_cell_wrap1(Xtree, left_arm_ctrl_obs_B.u);
  left_arm_ctrl_obs_B.u = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  emxEnsureCapacity_e_cell_wrap1(X, left_arm_ctrl_obs_B.u);
  if (0 <= left_arm_ctrl_obs_B.i_c) {
    left_arm_ctrl_obs_eye(left_arm_ctrl_obs_B.b_I);
  }

  for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k <=
       left_arm_ctrl_obs_B.i_c; left_arm_ctrl_obs_B.b_k++) {
    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 36;
         left_arm_ctrl_obs_B.u++) {
      Xtree->data[left_arm_ctrl_obs_B.b_k].f1[left_arm_ctrl_obs_B.u] =
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.u];
      X->data[left_arm_ctrl_obs_B.b_k].f1[left_arm_ctrl_obs_B.u] =
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.u];
    }
  }

  left_arm_ctrl__emxInit_real_T_e(&f, 2);
  left_arm_ctrl_obs_B.u = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  left_emxEnsureCapacity_real_T_e(f, left_arm_ctrl_obs_B.u);
  left_arm_ctrl__emxInit_real_T_e(&S, 2);
  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  for (left_arm_ctrl_obs_B.unnamed_idx_1 = 0; left_arm_ctrl_obs_B.unnamed_idx_1 <=
       left_arm_ctrl_obs_B.i_c; left_arm_ctrl_obs_B.unnamed_idx_1++) {
    obj_0 = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1];
    left_arm_ctrl_obs_B.u = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj_0->JointInternal.MotionSubspace->size[1];
    left_emxEnsureCapacity_real_T_e(S, left_arm_ctrl_obs_B.u);
    left_arm_ctrl_obs_B.aoffset = obj_0->JointInternal.MotionSubspace->size[0] *
      obj_0->JointInternal.MotionSubspace->size[1] - 1;
    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <=
         left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
      S->data[left_arm_ctrl_obs_B.u] = obj_0->JointInternal.MotionSubspace->
        data[left_arm_ctrl_obs_B.u];
    }

    left_arm_ctrl_obs_B.a_idx_0 = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1];
    left_arm_ctrl_obs_B.a_idx_1 = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1 + 10];
    left_arm_ctrl_obs_B.b_idx_0 = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1];
    left_arm_ctrl_obs_B.b_idx_1 = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1 + 10];
    if (left_arm_ctrl_obs_B.a_idx_1 < left_arm_ctrl_obs_B.a_idx_0) {
      obj_0 = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1];
      rigidBodyJoint_transformBodyT_e(&obj_0->JointInternal,
        left_arm_ctrl_obs_B.T);
      left_arm_ctrl_obs_B.t_h = 1;
      left_arm_ctrl_obs_B.qddoti_data[0] = 0.0;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        vJ->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
          0.0;
      }
    } else {
      if (left_arm_ctrl_obs_B.a_idx_0 > left_arm_ctrl_obs_B.a_idx_1) {
        left_arm_ctrl_obs_B.b_k = 0;
        left_arm_ctrl_obs_B.j = 0;
      } else {
        left_arm_ctrl_obs_B.b_k = static_cast<int32_T>
          (left_arm_ctrl_obs_B.a_idx_0) - 1;
        left_arm_ctrl_obs_B.j = static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_1);
      }

      if (left_arm_ctrl_obs_B.b_idx_0 > left_arm_ctrl_obs_B.b_idx_1) {
        left_arm_ctrl_obs_B.m = 0;
        left_arm_ctrl_obs_B.inner = 0;
        left_arm_ctrl_obs_B.u = 0;
        left_arm_ctrl_obs_B.t_h = -1;
      } else {
        left_arm_ctrl_obs_B.m = static_cast<int32_T>(left_arm_ctrl_obs_B.b_idx_0)
          - 1;
        left_arm_ctrl_obs_B.inner = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_1);
        left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.m;
        left_arm_ctrl_obs_B.t_h = left_arm_ctrl_obs_B.inner - 1;
      }

      left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.t_h - left_arm_ctrl_obs_B.u;
      left_arm_ctrl_obs_B.t_h = left_arm_ctrl_obs_B.u + 1;
      if (0 <= left_arm_ctrl_obs_B.u) {
        memset(&left_arm_ctrl_obs_B.qddoti_data[0], 0, (left_arm_ctrl_obs_B.u +
                1) * sizeof(real_T));
      }

      obj_0 = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1];
      left_arm_ctrl_obs_B.u = switch_expression->size[0] *
        switch_expression->size[1];
      switch_expression->size[0] = 1;
      switch_expression->size[1] = obj_0->JointInternal.Type->size[1];
      left_emxEnsureCapacity_char_T_e(switch_expression, left_arm_ctrl_obs_B.u);
      left_arm_ctrl_obs_B.aoffset = obj_0->JointInternal.Type->size[0] *
        obj_0->JointInternal.Type->size[1] - 1;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <=
           left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
        switch_expression->data[left_arm_ctrl_obs_B.u] =
          obj_0->JointInternal.Type->data[left_arm_ctrl_obs_B.u];
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 5;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.b_hh[left_arm_ctrl_obs_B.u] =
          tmp[left_arm_ctrl_obs_B.u];
      }

      left_arm_ctrl_obs_B.b_bool = false;
      if (switch_expression->size[1] == 5) {
        left_arm_ctrl_obs_B.u = 1;
        do {
          exitg1 = 0;
          if (left_arm_ctrl_obs_B.u - 1 < 5) {
            left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u - 1;
            if (switch_expression->data[left_arm_ctrl_obs_B.aoffset] !=
                left_arm_ctrl_obs_B.b_hh[left_arm_ctrl_obs_B.aoffset]) {
              exitg1 = 1;
            } else {
              left_arm_ctrl_obs_B.u++;
            }
          } else {
            left_arm_ctrl_obs_B.b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (left_arm_ctrl_obs_B.b_bool) {
        left_arm_ctrl_obs_B.u = 0;
      } else {
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 8;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.b_j[left_arm_ctrl_obs_B.u] =
            tmp_0[left_arm_ctrl_obs_B.u];
        }

        left_arm_ctrl_obs_B.b_bool = false;
        if (switch_expression->size[1] == 8) {
          left_arm_ctrl_obs_B.u = 1;
          do {
            exitg1 = 0;
            if (left_arm_ctrl_obs_B.u - 1 < 8) {
              left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u - 1;
              if (switch_expression->data[left_arm_ctrl_obs_B.aoffset] !=
                  left_arm_ctrl_obs_B.b_j[left_arm_ctrl_obs_B.aoffset]) {
                exitg1 = 1;
              } else {
                left_arm_ctrl_obs_B.u++;
              }
            } else {
              left_arm_ctrl_obs_B.b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (left_arm_ctrl_obs_B.b_bool) {
          left_arm_ctrl_obs_B.u = 1;
        } else {
          left_arm_ctrl_obs_B.u = -1;
        }
      }

      switch (left_arm_ctrl_obs_B.u) {
       case 0:
        memset(&left_arm_ctrl_obs_B.TJ[0], 0, sizeof(real_T) << 4U);
        left_arm_ctrl_obs_B.TJ[0] = 1.0;
        left_arm_ctrl_obs_B.TJ[5] = 1.0;
        left_arm_ctrl_obs_B.TJ[10] = 1.0;
        left_arm_ctrl_obs_B.TJ[15] = 1.0;
        break;

       case 1:
        rigidBodyJoint_get_JointAxis_e0(&obj_0->JointInternal,
          left_arm_ctrl_obs_B.v);
        left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.j -
          left_arm_ctrl_obs_B.b_k;
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <
             left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.l_data[left_arm_ctrl_obs_B.u] =
            left_arm_ctrl_obs_B.b_k + left_arm_ctrl_obs_B.u;
        }

        left_arm_ctrl_obs_B.result_data[0] = left_arm_ctrl_obs_B.v[0];
        left_arm_ctrl_obs_B.result_data[1] = left_arm_ctrl_obs_B.v[1];
        left_arm_ctrl_obs_B.result_data[2] = left_arm_ctrl_obs_B.v[2];
        if (0 <= (left_arm_ctrl_obs_B.aoffset != 0) - 1) {
          left_arm_ctrl_obs_B.result_data[3] =
            left_arm_ctrl_obs_B.q[left_arm_ctrl_obs_B.l_data[0]];
        }

        left_arm_ctrl_o_normalizeRows_e(&left_arm_ctrl_obs_B.result_data[0],
          left_arm_ctrl_obs_B.v);
        left_arm_ctrl_obs_B.a_idx_0 = cos(left_arm_ctrl_obs_B.result_data[3]);
        left_arm_ctrl_obs_B.sth = sin(left_arm_ctrl_obs_B.result_data[3]);
        left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.v[1] *
          left_arm_ctrl_obs_B.v[0] * (1.0 - left_arm_ctrl_obs_B.a_idx_0);
        left_arm_ctrl_obs_B.b_idx_0 = left_arm_ctrl_obs_B.v[2] *
          left_arm_ctrl_obs_B.sth;
        left_arm_ctrl_obs_B.b_idx_1 = left_arm_ctrl_obs_B.v[2] *
          left_arm_ctrl_obs_B.v[0] * (1.0 - left_arm_ctrl_obs_B.a_idx_0);
        left_arm_ctrl_obs_B.tempR_tmp = left_arm_ctrl_obs_B.v[1] *
          left_arm_ctrl_obs_B.sth;
        left_arm_ctrl_obs_B.tempR_tmp_i = left_arm_ctrl_obs_B.v[2] *
          left_arm_ctrl_obs_B.v[1] * (1.0 - left_arm_ctrl_obs_B.a_idx_0);
        left_arm_ctrl_obs_B.sth *= left_arm_ctrl_obs_B.v[0];
        left_arm_ctrl_obs_cat_e(left_arm_ctrl_obs_B.v[0] *
          left_arm_ctrl_obs_B.v[0] * (1.0 - left_arm_ctrl_obs_B.a_idx_0) +
          left_arm_ctrl_obs_B.a_idx_0, left_arm_ctrl_obs_B.a_idx_1 -
          left_arm_ctrl_obs_B.b_idx_0, left_arm_ctrl_obs_B.b_idx_1 +
          left_arm_ctrl_obs_B.tempR_tmp, left_arm_ctrl_obs_B.a_idx_1 +
          left_arm_ctrl_obs_B.b_idx_0, left_arm_ctrl_obs_B.v[1] *
          left_arm_ctrl_obs_B.v[1] * (1.0 - left_arm_ctrl_obs_B.a_idx_0) +
          left_arm_ctrl_obs_B.a_idx_0, left_arm_ctrl_obs_B.tempR_tmp_i -
          left_arm_ctrl_obs_B.sth, left_arm_ctrl_obs_B.b_idx_1 -
          left_arm_ctrl_obs_B.tempR_tmp, left_arm_ctrl_obs_B.tempR_tmp_i +
          left_arm_ctrl_obs_B.sth, left_arm_ctrl_obs_B.v[2] *
          left_arm_ctrl_obs_B.v[2] * (1.0 - left_arm_ctrl_obs_B.a_idx_0) +
          left_arm_ctrl_obs_B.a_idx_0, left_arm_ctrl_obs_B.tempR_l);
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 3;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.b_k + 1;
          left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u - 1] =
            left_arm_ctrl_obs_B.tempR_l[(left_arm_ctrl_obs_B.u - 1) * 3];
          left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.b_k + 1;
          left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u + 2] =
            left_arm_ctrl_obs_B.tempR_l[(left_arm_ctrl_obs_B.u - 1) * 3 + 1];
          left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.b_k + 1;
          left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u + 5] =
            left_arm_ctrl_obs_B.tempR_l[(left_arm_ctrl_obs_B.u - 1) * 3 + 2];
        }

        memset(&left_arm_ctrl_obs_B.TJ[0], 0, sizeof(real_T) << 4U);
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u << 2;
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset] =
            left_arm_ctrl_obs_B.R_g[3 * left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 1] =
            left_arm_ctrl_obs_B.R_g[3 * left_arm_ctrl_obs_B.u + 1];
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 2] =
            left_arm_ctrl_obs_B.R_g[3 * left_arm_ctrl_obs_B.u + 2];
        }

        left_arm_ctrl_obs_B.TJ[15] = 1.0;
        break;

       default:
        rigidBodyJoint_get_JointAxis_e0(&obj_0->JointInternal,
          left_arm_ctrl_obs_B.v);
        memset(&left_arm_ctrl_obs_B.tempR_l[0], 0, 9U * sizeof(real_T));
        left_arm_ctrl_obs_B.tempR_l[0] = 1.0;
        left_arm_ctrl_obs_B.tempR_l[4] = 1.0;
        left_arm_ctrl_obs_B.tempR_l[8] = 1.0;
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u << 2;
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset] =
            left_arm_ctrl_obs_B.tempR_l[3 * left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 1] =
            left_arm_ctrl_obs_B.tempR_l[3 * left_arm_ctrl_obs_B.u + 1];
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 2] =
            left_arm_ctrl_obs_B.tempR_l[3 * left_arm_ctrl_obs_B.u + 2];
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.u + 12] =
            left_arm_ctrl_obs_B.v[left_arm_ctrl_obs_B.u] *
            left_arm_ctrl_obs_B.q[left_arm_ctrl_obs_B.b_k];
        }

        left_arm_ctrl_obs_B.TJ[3] = 0.0;
        left_arm_ctrl_obs_B.TJ[7] = 0.0;
        left_arm_ctrl_obs_B.TJ[11] = 0.0;
        left_arm_ctrl_obs_B.TJ[15] = 1.0;
        break;
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 16;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u] =
          obj_0->JointInternal.JointToParentTransform[left_arm_ctrl_obs_B.u];
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 16;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.u] =
          obj_0->JointInternal.ChildToJointTransform[left_arm_ctrl_obs_B.u];
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 4;
           left_arm_ctrl_obs_B.u++) {
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 4;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.b_k << 2;
          left_arm_ctrl_obs_B.j = left_arm_ctrl_obs_B.u +
            left_arm_ctrl_obs_B.aoffset;
          left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.j] = 0.0;
          left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.j] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.j] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 1] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u + 4];
          left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.j] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 2] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u + 8];
          left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.j] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 3] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u + 12];
        }

        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 4;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.j = left_arm_ctrl_obs_B.b_k << 2;
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u +
            left_arm_ctrl_obs_B.j;
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] = 0.0;
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j] *
            left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j + 1] *
            left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.u + 4];
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j + 2] *
            left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.u + 8];
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j + 3] *
            left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.u + 12];
        }
      }

      if ((S->size[1] == 1) || (left_arm_ctrl_obs_B.inner -
           left_arm_ctrl_obs_B.m == 1)) {
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.b_k = left_arm_ctrl_obs_B.u + 6 *
            left_arm_ctrl_obs_B.unnamed_idx_1;
          vJ->data[left_arm_ctrl_obs_B.b_k] = 0.0;
          left_arm_ctrl_obs_B.aoffset = S->size[1];
          for (left_arm_ctrl_obs_B.m = 0; left_arm_ctrl_obs_B.m <
               left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.m++) {
            vJ->data[left_arm_ctrl_obs_B.b_k] += S->data[6 *
              left_arm_ctrl_obs_B.m + left_arm_ctrl_obs_B.u] * 0.0;
          }
        }
      } else {
        left_arm_ctrl_obs_B.inner = S->size[1] - 1;
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
             left_arm_ctrl_obs_B.u++) {
          vJ->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1]
            = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k <=
             left_arm_ctrl_obs_B.inner; left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.b_k * 6 - 1;
          for (left_arm_ctrl_obs_B.c_i = 0; left_arm_ctrl_obs_B.c_i < 6;
               left_arm_ctrl_obs_B.c_i++) {
            left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 +
              left_arm_ctrl_obs_B.c_i;
            vJ->data[left_arm_ctrl_obs_B.u] += S->data
              [(left_arm_ctrl_obs_B.aoffset + left_arm_ctrl_obs_B.c_i) + 1] *
              0.0;
          }
        }
      }
    }

    left_arm_ctrl_obs_tforminv_e(left_arm_ctrl_obs_B.T, left_arm_ctrl_obs_B.TJ);
    left_arm__tformToSpatialXform_e(left_arm_ctrl_obs_B.TJ, X->
      data[left_arm_ctrl_obs_B.unnamed_idx_1].f1);
    left_arm_ctrl_obs_B.a_idx_0 = robot->
      Bodies[left_arm_ctrl_obs_B.unnamed_idx_1]->ParentIndex;
    if (left_arm_ctrl_obs_B.a_idx_0 > 0.0) {
      left_arm_ctrl_obs_B.m = static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0);
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = 0.0;
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.a_idx_1 += vB->data[(left_arm_ctrl_obs_B.m - 1) *
            6 + left_arm_ctrl_obs_B.b_k] * X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
            left_arm_ctrl_obs_B.b_k + left_arm_ctrl_obs_B.u];
        }

        left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = vJ->data[6 *
          left_arm_ctrl_obs_B.unnamed_idx_1 + left_arm_ctrl_obs_B.u] +
          left_arm_ctrl_obs_B.a_idx_1;
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        vB->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
      }

      if ((S->size[1] == 1) || (left_arm_ctrl_obs_B.t_h == 1)) {
        left_arm_ctrl_obs_B.aoffset = S->size[1];
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = 0.0;
          for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k <
               left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.b_k++) {
            left_arm_ctrl_obs_B.a_idx_1 = S->data[6 * left_arm_ctrl_obs_B.b_k +
              left_arm_ctrl_obs_B.u] *
              left_arm_ctrl_obs_B.qddoti_data[left_arm_ctrl_obs_B.b_k] +
              left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
            left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] =
              left_arm_ctrl_obs_B.a_idx_1;
          }
        }
      } else {
        left_arm_ctrl_obs_B.inner = S->size[1] - 1;
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k <=
             left_arm_ctrl_obs_B.inner; left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.b_k * 6 - 1;
          for (left_arm_ctrl_obs_B.c_i = 0; left_arm_ctrl_obs_B.c_i < 6;
               left_arm_ctrl_obs_B.c_i++) {
            left_arm_ctrl_obs_B.a_idx_1 = S->data[(left_arm_ctrl_obs_B.aoffset +
              left_arm_ctrl_obs_B.c_i) + 1] * 0.0 +
              left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.c_i];
            left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.c_i] =
              left_arm_ctrl_obs_B.a_idx_1;
          }
        }
      }

      left_arm_ctrl_obs_B.tempR_l[0] = 0.0;
      left_arm_ctrl_obs_B.t_h = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 2;
      left_arm_ctrl_obs_B.tempR_l[3] = -vB->data[left_arm_ctrl_obs_B.t_h];
      left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 1;
      left_arm_ctrl_obs_B.tempR_l[6] = vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR_l[1] = vB->data[left_arm_ctrl_obs_B.t_h];
      left_arm_ctrl_obs_B.tempR_l[4] = 0.0;
      left_arm_ctrl_obs_B.tempR_l[7] = -vB->data[6 *
        left_arm_ctrl_obs_B.unnamed_idx_1];
      left_arm_ctrl_obs_B.tempR_l[2] = -vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR_l[5] = vB->data[6 *
        left_arm_ctrl_obs_B.unnamed_idx_1];
      left_arm_ctrl_obs_B.tempR_l[8] = 0.0;
      left_arm_ctrl_obs_B.tempR[3] = 0.0;
      left_arm_ctrl_obs_B.t_h = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 5;
      left_arm_ctrl_obs_B.tempR[9] = -vB->data[left_arm_ctrl_obs_B.t_h];
      left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 4;
      left_arm_ctrl_obs_B.tempR[15] = vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR[4] = vB->data[left_arm_ctrl_obs_B.t_h];
      left_arm_ctrl_obs_B.tempR[10] = 0.0;
      left_arm_ctrl_obs_B.t_h = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 3;
      left_arm_ctrl_obs_B.tempR[16] = -vB->data[left_arm_ctrl_obs_B.t_h];
      left_arm_ctrl_obs_B.tempR[5] = -vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR[11] = vB->data[left_arm_ctrl_obs_B.t_h];
      left_arm_ctrl_obs_B.tempR[17] = 0.0;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
          left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.t_h = 6 * (left_arm_ctrl_obs_B.u + 3);
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h] = 0.0;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h + 3] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
          left_arm_ctrl_obs_B.u + 1];
        left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 1] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h + 1] = 0.0;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h + 4] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
          left_arm_ctrl_obs_B.u + 2];
        left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 2] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h + 2] = 0.0;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h + 5] =
          left_arm_ctrl_obs_B.a_idx_1;
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = 0.0;
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.a_idx_1 += aB->data[(left_arm_ctrl_obs_B.m - 1) *
            6 + left_arm_ctrl_obs_B.b_k] * X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
            left_arm_ctrl_obs_B.b_k + left_arm_ctrl_obs_B.u];
        }

        left_arm_ctrl_obs_B.X_d[left_arm_ctrl_obs_B.u] =
          left_arm_ctrl_obs_B.a_idx_1 +
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = 0.0;
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR[6 *
            left_arm_ctrl_obs_B.b_k + left_arm_ctrl_obs_B.u] * vJ->data[6 *
            left_arm_ctrl_obs_B.unnamed_idx_1 + left_arm_ctrl_obs_B.b_k] +
            left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] =
            left_arm_ctrl_obs_B.a_idx_1;
        }

        aB->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
          left_arm_ctrl_obs_B.X_d[left_arm_ctrl_obs_B.u] +
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
      }

      left_arm_ctrl_obs_B.R_g[0] = 0.0;
      left_arm_ctrl_obs_B.R_g[3] = -left_arm_ctrl_obs_B.T[14];
      left_arm_ctrl_obs_B.R_g[6] = left_arm_ctrl_obs_B.T[13];
      left_arm_ctrl_obs_B.R_g[1] = left_arm_ctrl_obs_B.T[14];
      left_arm_ctrl_obs_B.R_g[4] = 0.0;
      left_arm_ctrl_obs_B.R_g[7] = -left_arm_ctrl_obs_B.T[12];
      left_arm_ctrl_obs_B.R_g[2] = -left_arm_ctrl_obs_B.T[13];
      left_arm_ctrl_obs_B.R_g[5] = left_arm_ctrl_obs_B.T[12];
      left_arm_ctrl_obs_B.R_g[8] = 0.0;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 3;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.m = left_arm_ctrl_obs_B.u + 3 *
            left_arm_ctrl_obs_B.b_k;
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] = 0.0;
          left_arm_ctrl_obs_B.t_h = left_arm_ctrl_obs_B.b_k << 2;
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_h] *
            left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_h + 1] *
            left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u + 3];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_h + 2] *
            left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u + 6];
          left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.b_k + 6 *
            left_arm_ctrl_obs_B.u] = left_arm_ctrl_obs_B.T
            [(left_arm_ctrl_obs_B.u << 2) + left_arm_ctrl_obs_B.b_k];
          left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.b_k + 6 *
            (left_arm_ctrl_obs_B.u + 3)] = 0.0;
        }
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.b_I[6 * left_arm_ctrl_obs_B.u + 3] =
          left_arm_ctrl_obs_B.dv2[3 * left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u << 2;
        left_arm_ctrl_obs_B.t_h = 6 * (left_arm_ctrl_obs_B.u + 3);
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.t_h + 3] =
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset];
        left_arm_ctrl_obs_B.b_I[6 * left_arm_ctrl_obs_B.u + 4] =
          left_arm_ctrl_obs_B.dv2[3 * left_arm_ctrl_obs_B.u + 1];
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.t_h + 4] =
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset + 1];
        left_arm_ctrl_obs_B.b_I[6 * left_arm_ctrl_obs_B.u + 5] =
          left_arm_ctrl_obs_B.dv2[3 * left_arm_ctrl_obs_B.u + 2];
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.t_h + 5] =
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset + 2];
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.t_h = left_arm_ctrl_obs_B.u + 6 *
            left_arm_ctrl_obs_B.b_k;
          left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h] = 0.0;
          for (left_arm_ctrl_obs_B.m = 0; left_arm_ctrl_obs_B.m < 6;
               left_arm_ctrl_obs_B.m++) {
            left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h] += Xtree->data[
              static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0) - 1].f1[6 *
              left_arm_ctrl_obs_B.m + left_arm_ctrl_obs_B.u] *
              left_arm_ctrl_obs_B.b_I[6 * left_arm_ctrl_obs_B.b_k +
              left_arm_ctrl_obs_B.m];
          }
        }
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 36;
           left_arm_ctrl_obs_B.u++) {
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[left_arm_ctrl_obs_B.u]
          = left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.u];
      }
    } else {
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.b_k = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 +
          left_arm_ctrl_obs_B.u;
        vB->data[left_arm_ctrl_obs_B.b_k] = vJ->data[left_arm_ctrl_obs_B.b_k];
      }

      if ((S->size[1] == 1) || (left_arm_ctrl_obs_B.t_h == 1)) {
        left_arm_ctrl_obs_B.aoffset = S->size[1];
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = 0.0;
          for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k <
               left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.b_k++) {
            left_arm_ctrl_obs_B.a_idx_1 = S->data[6 * left_arm_ctrl_obs_B.b_k +
              left_arm_ctrl_obs_B.u] *
              left_arm_ctrl_obs_B.qddoti_data[left_arm_ctrl_obs_B.b_k] +
              left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
            left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] =
              left_arm_ctrl_obs_B.a_idx_1;
          }
        }
      } else {
        left_arm_ctrl_obs_B.inner = S->size[1] - 1;
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k <=
             left_arm_ctrl_obs_B.inner; left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.b_k * 6 - 1;
          for (left_arm_ctrl_obs_B.c_i = 0; left_arm_ctrl_obs_B.c_i < 6;
               left_arm_ctrl_obs_B.c_i++) {
            left_arm_ctrl_obs_B.a_idx_1 = S->data[(left_arm_ctrl_obs_B.aoffset +
              left_arm_ctrl_obs_B.c_i) + 1] * 0.0 +
              left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.c_i];
            left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.c_i] =
              left_arm_ctrl_obs_B.a_idx_1;
          }
        }
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = 0.0;
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.a_idx_1 += X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
            left_arm_ctrl_obs_B.b_k + left_arm_ctrl_obs_B.u] *
            left_arm_ctrl_obs_B.a0[left_arm_ctrl_obs_B.b_k];
        }

        aB->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
          left_arm_ctrl_obs_B.a_idx_1 +
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
      }

      left_arm_ctrl_obs_B.R_g[0] = 0.0;
      left_arm_ctrl_obs_B.R_g[3] = -left_arm_ctrl_obs_B.T[14];
      left_arm_ctrl_obs_B.R_g[6] = left_arm_ctrl_obs_B.T[13];
      left_arm_ctrl_obs_B.R_g[1] = left_arm_ctrl_obs_B.T[14];
      left_arm_ctrl_obs_B.R_g[4] = 0.0;
      left_arm_ctrl_obs_B.R_g[7] = -left_arm_ctrl_obs_B.T[12];
      left_arm_ctrl_obs_B.R_g[2] = -left_arm_ctrl_obs_B.T[13];
      left_arm_ctrl_obs_B.R_g[5] = left_arm_ctrl_obs_B.T[12];
      left_arm_ctrl_obs_B.R_g[8] = 0.0;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 3;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.m = left_arm_ctrl_obs_B.u + 3 *
            left_arm_ctrl_obs_B.b_k;
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] = 0.0;
          left_arm_ctrl_obs_B.t_h = left_arm_ctrl_obs_B.b_k << 2;
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_h] *
            left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_h + 1] *
            left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u + 3];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_h + 2] *
            left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u + 6];
          Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1]
            .f1[left_arm_ctrl_obs_B.b_k + 6 * left_arm_ctrl_obs_B.u] =
            left_arm_ctrl_obs_B.T[(left_arm_ctrl_obs_B.u << 2) +
            left_arm_ctrl_obs_B.b_k];
          Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1]
            .f1[left_arm_ctrl_obs_B.b_k + 6 * (left_arm_ctrl_obs_B.u + 3)] = 0.0;
        }
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
          left_arm_ctrl_obs_B.u + 3] = left_arm_ctrl_obs_B.dv2[3 *
          left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.b_k = left_arm_ctrl_obs_B.u << 2;
        left_arm_ctrl_obs_B.m = 6 * (left_arm_ctrl_obs_B.u + 3);
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[left_arm_ctrl_obs_B.m
          + 3] = left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.b_k];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
          left_arm_ctrl_obs_B.u + 4] = left_arm_ctrl_obs_B.dv2[3 *
          left_arm_ctrl_obs_B.u + 1];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[left_arm_ctrl_obs_B.m
          + 4] = left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.b_k + 1];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
          left_arm_ctrl_obs_B.u + 5] = left_arm_ctrl_obs_B.dv2[3 *
          left_arm_ctrl_obs_B.u + 2];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[left_arm_ctrl_obs_B.m
          + 5] = left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.b_k + 2];
      }
    }

    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 36;
         left_arm_ctrl_obs_B.u++) {
      left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.u] = robot->
        Bodies[left_arm_ctrl_obs_B.unnamed_idx_1]->
        SpatialInertia[left_arm_ctrl_obs_B.u];
    }

    left_arm_ctrl_obs_B.tempR_l[0] = 0.0;
    left_arm_ctrl_obs_B.t_h = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 2;
    left_arm_ctrl_obs_B.tempR_l[3] = -vB->data[left_arm_ctrl_obs_B.t_h];
    left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 1;
    left_arm_ctrl_obs_B.tempR_l[6] = vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR_l[1] = vB->data[left_arm_ctrl_obs_B.t_h];
    left_arm_ctrl_obs_B.tempR_l[4] = 0.0;
    left_arm_ctrl_obs_B.tempR_l[7] = -vB->data[6 *
      left_arm_ctrl_obs_B.unnamed_idx_1];
    left_arm_ctrl_obs_B.tempR_l[2] = -vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR_l[5] = vB->data[6 *
      left_arm_ctrl_obs_B.unnamed_idx_1];
    left_arm_ctrl_obs_B.tempR_l[8] = 0.0;
    left_arm_ctrl_obs_B.tempR[18] = 0.0;
    left_arm_ctrl_obs_B.t_h = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 5;
    left_arm_ctrl_obs_B.tempR[24] = -vB->data[left_arm_ctrl_obs_B.t_h];
    left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 4;
    left_arm_ctrl_obs_B.tempR[30] = vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR[19] = vB->data[left_arm_ctrl_obs_B.t_h];
    left_arm_ctrl_obs_B.tempR[25] = 0.0;
    left_arm_ctrl_obs_B.t_h = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 3;
    left_arm_ctrl_obs_B.tempR[31] = -vB->data[left_arm_ctrl_obs_B.t_h];
    left_arm_ctrl_obs_B.tempR[20] = -vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR[26] = vB->data[left_arm_ctrl_obs_B.t_h];
    left_arm_ctrl_obs_B.tempR[32] = 0.0;
    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
         left_arm_ctrl_obs_B.u++) {
      left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
        left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 3] = 0.0;
      left_arm_ctrl_obs_B.t_h = 6 * (left_arm_ctrl_obs_B.u + 3);
      left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h + 3] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
        left_arm_ctrl_obs_B.u + 1];
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 1] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 4] = 0.0;
      left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h + 4] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
        left_arm_ctrl_obs_B.u + 2];
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 2] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 5] = 0.0;
      left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_h + 5] =
        left_arm_ctrl_obs_B.a_idx_1;
    }

    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
         left_arm_ctrl_obs_B.u++) {
      left_arm_ctrl_obs_B.X_d[left_arm_ctrl_obs_B.u] = 0.0;
      left_arm_ctrl_obs_B.b_I_e[left_arm_ctrl_obs_B.u] = 0.0;
      for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
           left_arm_ctrl_obs_B.b_k++) {
        left_arm_ctrl_obs_B.a_idx_0 = left_arm_ctrl_obs_B.b_I[6 *
          left_arm_ctrl_obs_B.b_k + left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.t_h = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 +
          left_arm_ctrl_obs_B.b_k;
        left_arm_ctrl_obs_B.a_idx_1 = vB->data[left_arm_ctrl_obs_B.t_h] *
          left_arm_ctrl_obs_B.a_idx_0 +
          left_arm_ctrl_obs_B.X_d[left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.a_idx_0 = aB->data[left_arm_ctrl_obs_B.t_h] *
          left_arm_ctrl_obs_B.a_idx_0 +
          left_arm_ctrl_obs_B.b_I_e[left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.X_d[left_arm_ctrl_obs_B.u] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.b_I_e[left_arm_ctrl_obs_B.u] =
          left_arm_ctrl_obs_B.a_idx_0;
      }
    }

    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
         left_arm_ctrl_obs_B.u++) {
      left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = 0.0;
      left_arm_ctrl_obs_B.a_idx_1 = 0.0;
      for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
           left_arm_ctrl_obs_B.b_k++) {
        left_arm_ctrl_obs_B.a_idx_1 += Xtree->
          data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 * left_arm_ctrl_obs_B.u +
          left_arm_ctrl_obs_B.b_k] * 0.0;
        left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] +=
          left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.b_k +
          left_arm_ctrl_obs_B.u] *
          left_arm_ctrl_obs_B.X_d[left_arm_ctrl_obs_B.b_k];
      }

      f->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
        (left_arm_ctrl_obs_B.b_I_e[left_arm_ctrl_obs_B.u] +
         left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u]) -
        left_arm_ctrl_obs_B.a_idx_1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  left_arm_ctrl__emxFree_real_T_e(&aB);
  left_arm_ctrl__emxFree_real_T_e(&vB);
  left_arm_ctrl__emxFree_real_T_e(&vJ);
  left_arm_c_emxFree_e_cell_wrap1(&Xtree);
  left_arm_ctrl_obs_B.i_c = static_cast<int32_T>(((-1.0 - left_arm_ctrl_obs_B.nb)
    + 1.0) / -1.0) - 1;
  left_arm_ctrl__emxInit_real_T_e(&taui, 1);
  left_arm_ctrl__emxInit_real_T_e(&a, 2);
  for (left_arm_ctrl_obs_B.t_h = 0; left_arm_ctrl_obs_B.t_h <=
       left_arm_ctrl_obs_B.i_c; left_arm_ctrl_obs_B.t_h++) {
    left_arm_ctrl_obs_B.a_idx_0 = left_arm_ctrl_obs_B.nb + -static_cast<real_T>
      (left_arm_ctrl_obs_B.t_h);
    left_arm_ctrl_obs_B.inner = static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0);
    left_arm_ctrl_obs_B.j = left_arm_ctrl_obs_B.inner - 1;
    obj_0 = robot->Bodies[left_arm_ctrl_obs_B.j];
    if (!left_arm_ctrl_obs_strcmp(obj_0->JointInternal.Type)) {
      obj_0 = robot->Bodies[left_arm_ctrl_obs_B.j];
      left_arm_ctrl_obs_B.u = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj_0->JointInternal.MotionSubspace->size[1];
      left_emxEnsureCapacity_real_T_e(S, left_arm_ctrl_obs_B.u);
      left_arm_ctrl_obs_B.aoffset = obj_0->JointInternal.MotionSubspace->size[0]
        * obj_0->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <=
           left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
        S->data[left_arm_ctrl_obs_B.u] = obj_0->
          JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.u];
      }

      left_arm_ctrl_obs_B.u = a->size[0] * a->size[1];
      a->size[0] = S->size[1];
      a->size[1] = 6;
      left_emxEnsureCapacity_real_T_e(a, left_arm_ctrl_obs_B.u);
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.aoffset = S->size[1];
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k <
             left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.b_k++) {
          a->data[left_arm_ctrl_obs_B.b_k + a->size[0] * left_arm_ctrl_obs_B.u] =
            S->data[6 * left_arm_ctrl_obs_B.b_k + left_arm_ctrl_obs_B.u];
        }
      }

      left_arm_ctrl_obs_B.m = a->size[0] - 1;
      left_arm_ctrl_obs_B.u = taui->size[0];
      taui->size[0] = a->size[0];
      left_emxEnsureCapacity_real_T_e(taui, left_arm_ctrl_obs_B.u);
      for (left_arm_ctrl_obs_B.unnamed_idx_1 = 0;
           left_arm_ctrl_obs_B.unnamed_idx_1 <= left_arm_ctrl_obs_B.m;
           left_arm_ctrl_obs_B.unnamed_idx_1++) {
        taui->data[left_arm_ctrl_obs_B.unnamed_idx_1] = 0.0;
      }

      for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
           left_arm_ctrl_obs_B.b_k++) {
        left_arm_ctrl_obs_B.aoffset = (left_arm_ctrl_obs_B.m + 1) *
          left_arm_ctrl_obs_B.b_k - 1;
        for (left_arm_ctrl_obs_B.c_i = 0; left_arm_ctrl_obs_B.c_i <=
             left_arm_ctrl_obs_B.m; left_arm_ctrl_obs_B.c_i++) {
          taui->data[left_arm_ctrl_obs_B.c_i] += f->data[(static_cast<int32_T>
            (left_arm_ctrl_obs_B.a_idx_0) - 1) * 6 + left_arm_ctrl_obs_B.b_k] *
            a->data[(left_arm_ctrl_obs_B.aoffset + left_arm_ctrl_obs_B.c_i) + 1];
        }
      }

      left_arm_ctrl_obs_B.b_idx_0 = robot->
        VelocityDoFMap[left_arm_ctrl_obs_B.inner - 1];
      left_arm_ctrl_obs_B.b_idx_1 = robot->
        VelocityDoFMap[left_arm_ctrl_obs_B.inner + 9];
      if (left_arm_ctrl_obs_B.b_idx_0 > left_arm_ctrl_obs_B.b_idx_1) {
        left_arm_ctrl_obs_B.m = 0;
        left_arm_ctrl_obs_B.unnamed_idx_1 = 0;
      } else {
        left_arm_ctrl_obs_B.m = static_cast<int32_T>(left_arm_ctrl_obs_B.b_idx_0)
          - 1;
        left_arm_ctrl_obs_B.unnamed_idx_1 = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_1);
      }

      left_arm_ctrl_obs_B.unnamed_idx_1 -= left_arm_ctrl_obs_B.m;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <
           left_arm_ctrl_obs_B.unnamed_idx_1; left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.tau[left_arm_ctrl_obs_B.m + left_arm_ctrl_obs_B.u] =
          taui->data[left_arm_ctrl_obs_B.u];
      }
    }

    left_arm_ctrl_obs_B.a_idx_0 = robot->Bodies[left_arm_ctrl_obs_B.j]
      ->ParentIndex;
    if (left_arm_ctrl_obs_B.a_idx_0 > 0.0) {
      left_arm_ctrl_obs_B.m = static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0);
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = 0.0;
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.a_idx_1 += f->data[(left_arm_ctrl_obs_B.inner - 1)
            * 6 + left_arm_ctrl_obs_B.b_k] * X->data[left_arm_ctrl_obs_B.j].f1[6
            * left_arm_ctrl_obs_B.u + left_arm_ctrl_obs_B.b_k];
        }

        left_arm_ctrl_obs_B.a0[left_arm_ctrl_obs_B.u] = f->data
          [(left_arm_ctrl_obs_B.m - 1) * 6 + left_arm_ctrl_obs_B.u] +
          left_arm_ctrl_obs_B.a_idx_1;
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        f->data[left_arm_ctrl_obs_B.u + 6 * (left_arm_ctrl_obs_B.m - 1)] =
          left_arm_ctrl_obs_B.a0[left_arm_ctrl_obs_B.u];
      }
    }
  }

  left_arm_ctrl__emxFree_real_T_e(&a);
  left_arm_ctrl__emxFree_real_T_e(&taui);
  left_arm_ctrl__emxFree_real_T_e(&S);
  left_arm_ctrl__emxFree_real_T_e(&f);
  left_arm_c_emxFree_e_cell_wrap1(&X);
  for (left_arm_ctrl_obs_B.i_c = 0; left_arm_ctrl_obs_B.i_c < 7;
       left_arm_ctrl_obs_B.i_c++) {
    jointTorq[left_arm_ctrl_obs_B.i_c] = static_cast<real32_T>
      (left_arm_ctrl_obs_B.tau[left_arm_ctrl_obs_B.i_c]);
  }
}

static void rigidBodyJoint_transformBody_e0(const
  rigidBodyJoint_left_arm_ctrl__T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (left_arm_ctrl_obs_B.b_kstr_h = 0; left_arm_ctrl_obs_B.b_kstr_h < 5;
       left_arm_ctrl_obs_B.b_kstr_h++) {
    left_arm_ctrl_obs_B.b_h2[left_arm_ctrl_obs_B.b_kstr_h] =
      tmp[left_arm_ctrl_obs_B.b_kstr_h];
  }

  left_arm_ctrl_obs_B.b_bool_c = false;
  if (obj->Type->size[1] == 5) {
    left_arm_ctrl_obs_B.b_kstr_h = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_h - 1 < 5) {
        left_arm_ctrl_obs_B.kstr_f = left_arm_ctrl_obs_B.b_kstr_h - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr_f] !=
            left_arm_ctrl_obs_B.b_h2[left_arm_ctrl_obs_B.kstr_f]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_h++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_c = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (left_arm_ctrl_obs_B.b_bool_c) {
    left_arm_ctrl_obs_B.b_kstr_h = 0;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr_h = 0; left_arm_ctrl_obs_B.b_kstr_h < 8;
         left_arm_ctrl_obs_B.b_kstr_h++) {
      left_arm_ctrl_obs_B.b_ct[left_arm_ctrl_obs_B.b_kstr_h] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr_h];
    }

    left_arm_ctrl_obs_B.b_bool_c = false;
    if (obj->Type->size[1] == 8) {
      left_arm_ctrl_obs_B.b_kstr_h = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_h - 1 < 8) {
          left_arm_ctrl_obs_B.kstr_f = left_arm_ctrl_obs_B.b_kstr_h - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr_f] !=
              left_arm_ctrl_obs_B.b_ct[left_arm_ctrl_obs_B.kstr_f]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_h++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_c) {
      left_arm_ctrl_obs_B.b_kstr_h = 1;
    } else {
      left_arm_ctrl_obs_B.b_kstr_h = -1;
    }
  }

  switch (left_arm_ctrl_obs_B.b_kstr_h) {
   case 0:
    memset(&left_arm_ctrl_obs_B.TJ_f[0], 0, sizeof(real_T) << 4U);
    left_arm_ctrl_obs_B.TJ_f[0] = 1.0;
    left_arm_ctrl_obs_B.TJ_f[5] = 1.0;
    left_arm_ctrl_obs_B.TJ_f[10] = 1.0;
    left_arm_ctrl_obs_B.TJ_f[15] = 1.0;
    break;

   case 1:
    rigidBodyJoint_get_JointAxis_e0(obj, left_arm_ctrl_obs_B.v_j);
    left_arm_ctrl_obs_B.result_data_b[0] = left_arm_ctrl_obs_B.v_j[0];
    left_arm_ctrl_obs_B.result_data_b[1] = left_arm_ctrl_obs_B.v_j[1];
    left_arm_ctrl_obs_B.result_data_b[2] = left_arm_ctrl_obs_B.v_j[2];
    if (0 <= (*q_size != 0) - 1) {
      left_arm_ctrl_obs_B.result_data_b[3] = q_data[0];
    }

    left_arm_ctrl_o_normalizeRows_e(&left_arm_ctrl_obs_B.result_data_b[0],
      left_arm_ctrl_obs_B.v_j);
    left_arm_ctrl_obs_B.cth = cos(left_arm_ctrl_obs_B.result_data_b[3]);
    left_arm_ctrl_obs_B.sth_f = sin(left_arm_ctrl_obs_B.result_data_b[3]);
    left_arm_ctrl_obs_B.tempR_tmp_g = left_arm_ctrl_obs_B.v_j[1] *
      left_arm_ctrl_obs_B.v_j[0] * (1.0 - left_arm_ctrl_obs_B.cth);
    left_arm_ctrl_obs_B.tempR_tmp_c = left_arm_ctrl_obs_B.v_j[2] *
      left_arm_ctrl_obs_B.sth_f;
    left_arm_ctrl_obs_B.tempR_tmp_o3 = left_arm_ctrl_obs_B.v_j[2] *
      left_arm_ctrl_obs_B.v_j[0] * (1.0 - left_arm_ctrl_obs_B.cth);
    left_arm_ctrl_obs_B.tempR_tmp_lm = left_arm_ctrl_obs_B.v_j[1] *
      left_arm_ctrl_obs_B.sth_f;
    left_arm_ctrl_obs_B.tempR_tmp_m = left_arm_ctrl_obs_B.v_j[2] *
      left_arm_ctrl_obs_B.v_j[1] * (1.0 - left_arm_ctrl_obs_B.cth);
    left_arm_ctrl_obs_B.sth_f *= left_arm_ctrl_obs_B.v_j[0];
    left_arm_ctrl_obs_cat_e(left_arm_ctrl_obs_B.v_j[0] *
      left_arm_ctrl_obs_B.v_j[0] * (1.0 - left_arm_ctrl_obs_B.cth) +
      left_arm_ctrl_obs_B.cth, left_arm_ctrl_obs_B.tempR_tmp_g -
      left_arm_ctrl_obs_B.tempR_tmp_c, left_arm_ctrl_obs_B.tempR_tmp_o3 +
      left_arm_ctrl_obs_B.tempR_tmp_lm, left_arm_ctrl_obs_B.tempR_tmp_g +
      left_arm_ctrl_obs_B.tempR_tmp_c, left_arm_ctrl_obs_B.v_j[1] *
      left_arm_ctrl_obs_B.v_j[1] * (1.0 - left_arm_ctrl_obs_B.cth) +
      left_arm_ctrl_obs_B.cth, left_arm_ctrl_obs_B.tempR_tmp_m -
      left_arm_ctrl_obs_B.sth_f, left_arm_ctrl_obs_B.tempR_tmp_o3 -
      left_arm_ctrl_obs_B.tempR_tmp_lm, left_arm_ctrl_obs_B.tempR_tmp_m +
      left_arm_ctrl_obs_B.sth_f, left_arm_ctrl_obs_B.v_j[2] *
      left_arm_ctrl_obs_B.v_j[2] * (1.0 - left_arm_ctrl_obs_B.cth) +
      left_arm_ctrl_obs_B.cth, left_arm_ctrl_obs_B.tempR_o);
    for (left_arm_ctrl_obs_B.b_kstr_h = 0; left_arm_ctrl_obs_B.b_kstr_h < 3;
         left_arm_ctrl_obs_B.b_kstr_h++) {
      left_arm_ctrl_obs_B.kstr_f = left_arm_ctrl_obs_B.b_kstr_h + 1;
      left_arm_ctrl_obs_B.R_l[left_arm_ctrl_obs_B.kstr_f - 1] =
        left_arm_ctrl_obs_B.tempR_o[(left_arm_ctrl_obs_B.kstr_f - 1) * 3];
      left_arm_ctrl_obs_B.kstr_f = left_arm_ctrl_obs_B.b_kstr_h + 1;
      left_arm_ctrl_obs_B.R_l[left_arm_ctrl_obs_B.kstr_f + 2] =
        left_arm_ctrl_obs_B.tempR_o[(left_arm_ctrl_obs_B.kstr_f - 1) * 3 + 1];
      left_arm_ctrl_obs_B.kstr_f = left_arm_ctrl_obs_B.b_kstr_h + 1;
      left_arm_ctrl_obs_B.R_l[left_arm_ctrl_obs_B.kstr_f + 5] =
        left_arm_ctrl_obs_B.tempR_o[(left_arm_ctrl_obs_B.kstr_f - 1) * 3 + 2];
    }

    memset(&left_arm_ctrl_obs_B.TJ_f[0], 0, sizeof(real_T) << 4U);
    for (left_arm_ctrl_obs_B.b_kstr_h = 0; left_arm_ctrl_obs_B.b_kstr_h < 3;
         left_arm_ctrl_obs_B.b_kstr_h++) {
      left_arm_ctrl_obs_B.kstr_f = left_arm_ctrl_obs_B.b_kstr_h << 2;
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_f] =
        left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.b_kstr_h];
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_f + 1] =
        left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.b_kstr_h + 1];
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_f + 2] =
        left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.b_kstr_h + 2];
    }

    left_arm_ctrl_obs_B.TJ_f[15] = 1.0;
    break;

   default:
    rigidBodyJoint_get_JointAxis_e0(obj, left_arm_ctrl_obs_B.v_j);
    memset(&left_arm_ctrl_obs_B.tempR_o[0], 0, 9U * sizeof(real_T));
    left_arm_ctrl_obs_B.tempR_o[0] = 1.0;
    left_arm_ctrl_obs_B.tempR_o[4] = 1.0;
    left_arm_ctrl_obs_B.tempR_o[8] = 1.0;
    for (left_arm_ctrl_obs_B.b_kstr_h = 0; left_arm_ctrl_obs_B.b_kstr_h < 3;
         left_arm_ctrl_obs_B.b_kstr_h++) {
      left_arm_ctrl_obs_B.kstr_f = left_arm_ctrl_obs_B.b_kstr_h << 2;
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_f] =
        left_arm_ctrl_obs_B.tempR_o[3 * left_arm_ctrl_obs_B.b_kstr_h];
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_f + 1] =
        left_arm_ctrl_obs_B.tempR_o[3 * left_arm_ctrl_obs_B.b_kstr_h + 1];
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_f + 2] =
        left_arm_ctrl_obs_B.tempR_o[3 * left_arm_ctrl_obs_B.b_kstr_h + 2];
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.b_kstr_h + 12] =
        left_arm_ctrl_obs_B.v_j[left_arm_ctrl_obs_B.b_kstr_h] * q_data[0];
    }

    left_arm_ctrl_obs_B.TJ_f[3] = 0.0;
    left_arm_ctrl_obs_B.TJ_f[7] = 0.0;
    left_arm_ctrl_obs_B.TJ_f[11] = 0.0;
    left_arm_ctrl_obs_B.TJ_f[15] = 1.0;
    break;
  }

  for (left_arm_ctrl_obs_B.b_kstr_h = 0; left_arm_ctrl_obs_B.b_kstr_h < 4;
       left_arm_ctrl_obs_B.b_kstr_h++) {
    for (left_arm_ctrl_obs_B.kstr_f = 0; left_arm_ctrl_obs_B.kstr_f < 4;
         left_arm_ctrl_obs_B.kstr_f++) {
      left_arm_ctrl_obs_B.obj_tmp_tmp_c = left_arm_ctrl_obs_B.kstr_f << 2;
      left_arm_ctrl_obs_B.obj_tmp_e = left_arm_ctrl_obs_B.b_kstr_h +
        left_arm_ctrl_obs_B.obj_tmp_tmp_c;
      left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.obj_tmp_e] = 0.0;
      left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.obj_tmp_e] +=
        left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.obj_tmp_tmp_c] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_h];
      left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.obj_tmp_e] +=
        left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.obj_tmp_tmp_c + 1] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_h + 4];
      left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.obj_tmp_e] +=
        left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.obj_tmp_tmp_c + 2] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_h + 8];
      left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.obj_tmp_e] +=
        left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.obj_tmp_tmp_c + 3] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_h + 12];
    }

    for (left_arm_ctrl_obs_B.kstr_f = 0; left_arm_ctrl_obs_B.kstr_f < 4;
         left_arm_ctrl_obs_B.kstr_f++) {
      left_arm_ctrl_obs_B.obj_tmp_tmp_c = left_arm_ctrl_obs_B.kstr_f << 2;
      left_arm_ctrl_obs_B.obj_tmp_e = left_arm_ctrl_obs_B.b_kstr_h +
        left_arm_ctrl_obs_B.obj_tmp_tmp_c;
      T[left_arm_ctrl_obs_B.obj_tmp_e] = 0.0;
      T[left_arm_ctrl_obs_B.obj_tmp_e] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_c] *
        left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.b_kstr_h];
      T[left_arm_ctrl_obs_B.obj_tmp_e] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_c + 1] *
        left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.b_kstr_h + 4];
      T[left_arm_ctrl_obs_B.obj_tmp_e] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_c + 2] *
        left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.b_kstr_h + 8];
      T[left_arm_ctrl_obs_B.obj_tmp_e] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_c + 3] *
        left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.b_kstr_h + 12];
    }
  }
}

static void RigidBodyTreeDynamics_massMat_e(k_robotics_manip_internal__e0_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_ctrl_T *H)
{
  emxArray_f_cell_wrap_left_arm_T *Ic;
  emxArray_f_cell_wrap_left_arm_T *X;
  emxArray_real_T_left_arm_ctrl_T *Si;
  emxArray_real_T_left_arm_ctrl_T *Fi;
  emxArray_real_T_left_arm_ctrl_T *Sj;
  emxArray_real_T_left_arm_ctrl_T *Hji;
  j_robotics_manip_internal_Rig_T *obj;
  emxArray_real_T_left_arm_ctrl_T *a;
  emxArray_real_T_left_arm_ctrl_T *B;
  left_arm_ctrl_obs_B.nb_i = robot->NumBodies;
  left_arm_ctrl_obs_B.vNum = robot->VelocityNumber;
  left_arm_ctrl_obs_B.f = H->size[0] * H->size[1];
  left_arm_ctrl_obs_B.b_i_j = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum);
  H->size[0] = left_arm_ctrl_obs_B.b_i_j;
  H->size[1] = left_arm_ctrl_obs_B.b_i_j;
  left_emxEnsureCapacity_real_T_e(H, left_arm_ctrl_obs_B.f);
  left_arm_ctrl_obs_B.n_e = left_arm_ctrl_obs_B.b_i_j *
    left_arm_ctrl_obs_B.b_i_j - 1;
  for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
       left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.f++) {
    H->data[left_arm_ctrl_obs_B.f] = 0.0;
  }

  left_arm_c_emxInit_e_cell_wrap1(&Ic, 2);
  left_arm_c_emxInit_e_cell_wrap1(&X, 2);
  left_arm_ctrl_obs_B.c_tmp = static_cast<int32_T>(left_arm_ctrl_obs_B.nb_i);
  left_arm_ctrl_obs_B.c_a = left_arm_ctrl_obs_B.c_tmp - 1;
  left_arm_ctrl_obs_B.f = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = left_arm_ctrl_obs_B.c_tmp;
  emxEnsureCapacity_e_cell_wrap1(Ic, left_arm_ctrl_obs_B.f);
  left_arm_ctrl_obs_B.f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_ctrl_obs_B.c_tmp;
  emxEnsureCapacity_e_cell_wrap1(X, left_arm_ctrl_obs_B.f);
  for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j <=
       left_arm_ctrl_obs_B.c_a; left_arm_ctrl_obs_B.b_i_j++) {
    for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 36;
         left_arm_ctrl_obs_B.f++) {
      Ic->data[left_arm_ctrl_obs_B.b_i_j].f1[left_arm_ctrl_obs_B.f] =
        robot->Bodies[left_arm_ctrl_obs_B.b_i_j]->
        SpatialInertia[left_arm_ctrl_obs_B.f];
    }

    left_arm_ctrl_obs_B.vNum = robot->PositionDoFMap[left_arm_ctrl_obs_B.b_i_j];
    left_arm_ctrl_obs_B.p_idx_1 = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.b_i_j + 10];
    if (left_arm_ctrl_obs_B.p_idx_1 < left_arm_ctrl_obs_B.vNum) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.b_i_j];
      rigidBodyJoint_transformBodyT_e(&obj->JointInternal,
        left_arm_ctrl_obs_B.T_c);
    } else {
      if (left_arm_ctrl_obs_B.vNum > left_arm_ctrl_obs_B.p_idx_1) {
        left_arm_ctrl_obs_B.c_tmp = 0;
        left_arm_ctrl_obs_B.f = -1;
      } else {
        left_arm_ctrl_obs_B.c_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.vNum) - 1;
        left_arm_ctrl_obs_B.f = static_cast<int32_T>(left_arm_ctrl_obs_B.p_idx_1)
          - 1;
      }

      obj = robot->Bodies[left_arm_ctrl_obs_B.b_i_j];
      left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.f -
        left_arm_ctrl_obs_B.c_tmp;
      left_arm_ctrl_obs_B.q_size = left_arm_ctrl_obs_B.q_size_tmp + 1;
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
           left_arm_ctrl_obs_B.q_size_tmp; left_arm_ctrl_obs_B.f++) {
        left_arm_ctrl_obs_B.q_data[left_arm_ctrl_obs_B.f] =
          q[left_arm_ctrl_obs_B.c_tmp + left_arm_ctrl_obs_B.f];
      }

      rigidBodyJoint_transformBody_e0(&obj->JointInternal,
        left_arm_ctrl_obs_B.q_data, &left_arm_ctrl_obs_B.q_size,
        left_arm_ctrl_obs_B.T_c);
    }

    left_arm_ctrl_obs_tforminv_e(left_arm_ctrl_obs_B.T_c,
      left_arm_ctrl_obs_B.dv1);
    left_arm__tformToSpatialXform_e(left_arm_ctrl_obs_B.dv1, X->
      data[left_arm_ctrl_obs_B.b_i_j].f1);
  }

  left_arm_ctrl_obs_B.c_a = static_cast<int32_T>(((-1.0 -
    left_arm_ctrl_obs_B.nb_i) + 1.0) / -1.0) - 1;
  left_arm_ctrl__emxInit_real_T_e(&Si, 2);
  left_arm_ctrl__emxInit_real_T_e(&Fi, 2);
  left_arm_ctrl__emxInit_real_T_e(&Sj, 2);
  left_arm_ctrl__emxInit_real_T_e(&Hji, 2);
  left_arm_ctrl__emxInit_real_T_e(&a, 2);
  left_arm_ctrl__emxInit_real_T_e(&B, 2);
  for (left_arm_ctrl_obs_B.c_tmp = 0; left_arm_ctrl_obs_B.c_tmp <=
       left_arm_ctrl_obs_B.c_a; left_arm_ctrl_obs_B.c_tmp++) {
    left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>(left_arm_ctrl_obs_B.nb_i
      + -static_cast<real_T>(left_arm_ctrl_obs_B.c_tmp));
    left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.pid_tmp - 1;
    left_arm_ctrl_obs_B.pid = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp]
      ->ParentIndex;
    left_arm_ctrl_obs_B.vNum = robot->VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp
      - 1];
    left_arm_ctrl_obs_B.p_idx_1 = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp + 9];
    if (left_arm_ctrl_obs_B.pid > 0.0) {
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
             left_arm_ctrl_obs_B.b_i_j++) {
          left_arm_ctrl_obs_B.X_tmp = left_arm_ctrl_obs_B.f + 6 *
            left_arm_ctrl_obs_B.b_i_j;
          left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.X_tmp] = 0.0;
          for (left_arm_ctrl_obs_B.n_e = 0; left_arm_ctrl_obs_B.n_e < 6;
               left_arm_ctrl_obs_B.n_e++) {
            left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.X_tmp] += X->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 * left_arm_ctrl_obs_B.f
              + left_arm_ctrl_obs_B.n_e] * Ic->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 *
              left_arm_ctrl_obs_B.b_i_j + left_arm_ctrl_obs_B.n_e];
          }
        }
      }

      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
             left_arm_ctrl_obs_B.b_i_j++) {
          left_arm_ctrl_obs_B.b_idx_0_f = 0.0;
          for (left_arm_ctrl_obs_B.n_e = 0; left_arm_ctrl_obs_B.n_e < 6;
               left_arm_ctrl_obs_B.n_e++) {
            left_arm_ctrl_obs_B.b_idx_0_f += left_arm_ctrl_obs_B.X[6 *
              left_arm_ctrl_obs_B.n_e + left_arm_ctrl_obs_B.f] * X->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 *
              left_arm_ctrl_obs_B.b_i_j + left_arm_ctrl_obs_B.n_e];
          }

          left_arm_ctrl_obs_B.n_e = 6 * left_arm_ctrl_obs_B.b_i_j +
            left_arm_ctrl_obs_B.f;
          Ic->data[static_cast<int32_T>(left_arm_ctrl_obs_B.pid) - 1]
            .f1[left_arm_ctrl_obs_B.n_e] += left_arm_ctrl_obs_B.b_idx_0_f;
        }
      }
    }

    left_arm_ctrl_obs_B.b_idx_0_f = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp - 1];
    left_arm_ctrl_obs_B.b_idx_1_i = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp + 9];
    if (left_arm_ctrl_obs_B.b_idx_0_f <= left_arm_ctrl_obs_B.b_idx_1_i) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp];
      left_arm_ctrl_obs_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_emxEnsureCapacity_real_T_e(Si, left_arm_ctrl_obs_B.f);
      left_arm_ctrl_obs_B.n_e = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
           left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.f++) {
        Si->data[left_arm_ctrl_obs_B.f] = obj->
          JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.f];
      }

      left_arm_ctrl_obs_B.n_e = Si->size[1] - 1;
      left_arm_ctrl_obs_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      left_emxEnsureCapacity_real_T_e(Fi, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.b_j_a = 0; left_arm_ctrl_obs_B.b_j_a <=
           left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.b_j_a++) {
        left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_j_a * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
             left_arm_ctrl_obs_B.b_i_j++) {
          Fi->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_j) + 1]
            = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
             left_arm_ctrl_obs_B.b_i_j++) {
          left_arm_ctrl_obs_B.aoffset_o = left_arm_ctrl_obs_B.b_i_j * 6 - 1;
          left_arm_ctrl_obs_B.temp = Si->data[(left_arm_ctrl_obs_B.pid_tmp +
            left_arm_ctrl_obs_B.b_i_j) + 1];
          for (left_arm_ctrl_obs_B.c_i_g = 0; left_arm_ctrl_obs_B.c_i_g < 6;
               left_arm_ctrl_obs_B.c_i_g++) {
            left_arm_ctrl_obs_B.i_b = left_arm_ctrl_obs_B.c_i_g + 1;
            left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.pid_tmp +
              left_arm_ctrl_obs_B.i_b;
            Fi->data[left_arm_ctrl_obs_B.f] += Ic->
              data[left_arm_ctrl_obs_B.q_size_tmp]
              .f1[left_arm_ctrl_obs_B.aoffset_o + left_arm_ctrl_obs_B.i_b] *
              left_arm_ctrl_obs_B.temp;
          }
        }
      }

      if (left_arm_ctrl_obs_B.vNum > left_arm_ctrl_obs_B.p_idx_1) {
        left_arm_ctrl_obs_B.pid_tmp = 0;
        left_arm_ctrl_obs_B.X_tmp = 0;
      } else {
        left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.vNum) - 1;
        left_arm_ctrl_obs_B.X_tmp = left_arm_ctrl_obs_B.pid_tmp;
      }

      left_arm_ctrl_obs_B.f = a->size[0] * a->size[1];
      a->size[0] = Si->size[1];
      a->size[1] = 6;
      left_emxEnsureCapacity_real_T_e(a, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        left_arm_ctrl_obs_B.n_e = Si->size[1];
        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j <
             left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.b_i_j++) {
          a->data[left_arm_ctrl_obs_B.b_i_j + a->size[0] * left_arm_ctrl_obs_B.f]
            = Si->data[6 * left_arm_ctrl_obs_B.b_i_j + left_arm_ctrl_obs_B.f];
        }
      }

      left_arm_ctrl_obs_B.m_e = a->size[0];
      left_arm_ctrl_obs_B.n_e = Fi->size[1] - 1;
      left_arm_ctrl_obs_B.f = Hji->size[0] * Hji->size[1];
      Hji->size[0] = a->size[0];
      Hji->size[1] = Fi->size[1];
      left_emxEnsureCapacity_real_T_e(Hji, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.b_j_a = 0; left_arm_ctrl_obs_B.b_j_a <=
           left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.b_j_a++) {
        left_arm_ctrl_obs_B.coffset = left_arm_ctrl_obs_B.b_j_a *
          left_arm_ctrl_obs_B.m_e - 1;
        left_arm_ctrl_obs_B.boffset = left_arm_ctrl_obs_B.b_j_a * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j <
             left_arm_ctrl_obs_B.m_e; left_arm_ctrl_obs_B.b_i_j++) {
          Hji->data[(left_arm_ctrl_obs_B.coffset + left_arm_ctrl_obs_B.b_i_j) +
            1] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
             left_arm_ctrl_obs_B.b_i_j++) {
          left_arm_ctrl_obs_B.aoffset_o = left_arm_ctrl_obs_B.b_i_j *
            left_arm_ctrl_obs_B.m_e - 1;
          left_arm_ctrl_obs_B.temp = Fi->data[(left_arm_ctrl_obs_B.boffset +
            left_arm_ctrl_obs_B.b_i_j) + 1];
          for (left_arm_ctrl_obs_B.c_i_g = 0; left_arm_ctrl_obs_B.c_i_g <
               left_arm_ctrl_obs_B.m_e; left_arm_ctrl_obs_B.c_i_g++) {
            left_arm_ctrl_obs_B.i_b = left_arm_ctrl_obs_B.c_i_g + 1;
            left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.coffset +
              left_arm_ctrl_obs_B.i_b;
            Hji->data[left_arm_ctrl_obs_B.f] += a->
              data[left_arm_ctrl_obs_B.aoffset_o + left_arm_ctrl_obs_B.i_b] *
              left_arm_ctrl_obs_B.temp;
          }
        }
      }

      left_arm_ctrl_obs_B.n_e = Hji->size[1];
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <
           left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.f++) {
        left_arm_ctrl_obs_B.b_j_a = Hji->size[0];
        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j <
             left_arm_ctrl_obs_B.b_j_a; left_arm_ctrl_obs_B.b_i_j++) {
          H->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_j) +
            H->size[0] * (left_arm_ctrl_obs_B.X_tmp + left_arm_ctrl_obs_B.f)] =
            Hji->data[Hji->size[0] * left_arm_ctrl_obs_B.f +
            left_arm_ctrl_obs_B.b_i_j];
        }
      }

      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
             left_arm_ctrl_obs_B.b_i_j++) {
          left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.b_i_j + 6 *
            left_arm_ctrl_obs_B.f] = X->data[left_arm_ctrl_obs_B.q_size_tmp].f1
            [6 * left_arm_ctrl_obs_B.b_i_j + left_arm_ctrl_obs_B.f];
        }
      }

      left_arm_ctrl_obs_B.f = B->size[0] * B->size[1];
      B->size[0] = 6;
      B->size[1] = Fi->size[1];
      left_emxEnsureCapacity_real_T_e(B, left_arm_ctrl_obs_B.f);
      left_arm_ctrl_obs_B.n_e = Fi->size[0] * Fi->size[1] - 1;
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
           left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.f++) {
        B->data[left_arm_ctrl_obs_B.f] = Fi->data[left_arm_ctrl_obs_B.f];
      }

      left_arm_ctrl_obs_B.n_e = Fi->size[1];
      left_arm_ctrl_obs_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = left_arm_ctrl_obs_B.n_e;
      left_emxEnsureCapacity_real_T_e(Fi, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.b_j_a = 0; left_arm_ctrl_obs_B.b_j_a <
           left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.b_j_a++) {
        left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_j_a * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
             left_arm_ctrl_obs_B.b_i_j++) {
          Fi->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_j) + 1]
            = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
             left_arm_ctrl_obs_B.b_i_j++) {
          left_arm_ctrl_obs_B.aoffset_o = left_arm_ctrl_obs_B.b_i_j * 6 - 1;
          left_arm_ctrl_obs_B.temp = B->data[(left_arm_ctrl_obs_B.pid_tmp +
            left_arm_ctrl_obs_B.b_i_j) + 1];
          for (left_arm_ctrl_obs_B.c_i_g = 0; left_arm_ctrl_obs_B.c_i_g < 6;
               left_arm_ctrl_obs_B.c_i_g++) {
            left_arm_ctrl_obs_B.i_b = left_arm_ctrl_obs_B.c_i_g + 1;
            left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.pid_tmp +
              left_arm_ctrl_obs_B.i_b;
            Fi->data[left_arm_ctrl_obs_B.f] +=
              left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.aoffset_o +
              left_arm_ctrl_obs_B.i_b] * left_arm_ctrl_obs_B.temp;
          }
        }
      }

      while (left_arm_ctrl_obs_B.pid > 0.0) {
        left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.pid);
        left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.pid_tmp - 1;
        obj = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp];
        left_arm_ctrl_obs_B.f = Sj->size[0] * Sj->size[1];
        Sj->size[0] = 6;
        Sj->size[1] = obj->JointInternal.MotionSubspace->size[1];
        left_emxEnsureCapacity_real_T_e(Sj, left_arm_ctrl_obs_B.f);
        left_arm_ctrl_obs_B.n_e = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
             left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.f++) {
          Sj->data[left_arm_ctrl_obs_B.f] = obj->
            JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.f];
        }

        left_arm_ctrl_obs_B.b_idx_0_f = robot->
          VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp - 1];
        left_arm_ctrl_obs_B.b_idx_1_i = robot->
          VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp + 9];
        if (left_arm_ctrl_obs_B.b_idx_0_f <= left_arm_ctrl_obs_B.b_idx_1_i) {
          left_arm_ctrl_obs_B.f = a->size[0] * a->size[1];
          a->size[0] = Sj->size[1];
          a->size[1] = 6;
          left_emxEnsureCapacity_real_T_e(a, left_arm_ctrl_obs_B.f);
          for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
               left_arm_ctrl_obs_B.f++) {
            left_arm_ctrl_obs_B.n_e = Sj->size[1];
            for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j <
                 left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.b_i_j++) {
              a->data[left_arm_ctrl_obs_B.b_i_j + a->size[0] *
                left_arm_ctrl_obs_B.f] = Sj->data[6 * left_arm_ctrl_obs_B.b_i_j
                + left_arm_ctrl_obs_B.f];
            }
          }

          left_arm_ctrl_obs_B.m_e = a->size[0];
          left_arm_ctrl_obs_B.n_e = Fi->size[1] - 1;
          left_arm_ctrl_obs_B.f = Hji->size[0] * Hji->size[1];
          Hji->size[0] = a->size[0];
          Hji->size[1] = Fi->size[1];
          left_emxEnsureCapacity_real_T_e(Hji, left_arm_ctrl_obs_B.f);
          for (left_arm_ctrl_obs_B.b_j_a = 0; left_arm_ctrl_obs_B.b_j_a <=
               left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.b_j_a++) {
            left_arm_ctrl_obs_B.coffset = left_arm_ctrl_obs_B.b_j_a *
              left_arm_ctrl_obs_B.m_e - 1;
            left_arm_ctrl_obs_B.boffset = left_arm_ctrl_obs_B.b_j_a * 6 - 1;
            for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j <
                 left_arm_ctrl_obs_B.m_e; left_arm_ctrl_obs_B.b_i_j++) {
              Hji->data[(left_arm_ctrl_obs_B.coffset + left_arm_ctrl_obs_B.b_i_j)
                + 1] = 0.0;
            }

            for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
                 left_arm_ctrl_obs_B.b_i_j++) {
              left_arm_ctrl_obs_B.aoffset_o = left_arm_ctrl_obs_B.b_i_j *
                left_arm_ctrl_obs_B.m_e - 1;
              left_arm_ctrl_obs_B.temp = Fi->data[(left_arm_ctrl_obs_B.boffset +
                left_arm_ctrl_obs_B.b_i_j) + 1];
              for (left_arm_ctrl_obs_B.c_i_g = 0; left_arm_ctrl_obs_B.c_i_g <
                   left_arm_ctrl_obs_B.m_e; left_arm_ctrl_obs_B.c_i_g++) {
                left_arm_ctrl_obs_B.i_b = left_arm_ctrl_obs_B.c_i_g + 1;
                left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.coffset +
                  left_arm_ctrl_obs_B.i_b;
                Hji->data[left_arm_ctrl_obs_B.f] += a->
                  data[left_arm_ctrl_obs_B.aoffset_o + left_arm_ctrl_obs_B.i_b] *
                  left_arm_ctrl_obs_B.temp;
              }
            }
          }

          if (left_arm_ctrl_obs_B.b_idx_0_f > left_arm_ctrl_obs_B.b_idx_1_i) {
            left_arm_ctrl_obs_B.pid_tmp = 0;
          } else {
            left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>
              (left_arm_ctrl_obs_B.b_idx_0_f) - 1;
          }

          if (left_arm_ctrl_obs_B.vNum > left_arm_ctrl_obs_B.p_idx_1) {
            left_arm_ctrl_obs_B.X_tmp = 0;
          } else {
            left_arm_ctrl_obs_B.X_tmp = static_cast<int32_T>
              (left_arm_ctrl_obs_B.vNum) - 1;
          }

          left_arm_ctrl_obs_B.n_e = Hji->size[1];
          for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <
               left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.f++) {
            left_arm_ctrl_obs_B.b_j_a = Hji->size[0];
            for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j <
                 left_arm_ctrl_obs_B.b_j_a; left_arm_ctrl_obs_B.b_i_j++) {
              H->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_j)
                + H->size[0] * (left_arm_ctrl_obs_B.X_tmp +
                                left_arm_ctrl_obs_B.f)] = Hji->data[Hji->size[0]
                * left_arm_ctrl_obs_B.f + left_arm_ctrl_obs_B.b_i_j];
            }
          }

          if (left_arm_ctrl_obs_B.vNum > left_arm_ctrl_obs_B.p_idx_1) {
            left_arm_ctrl_obs_B.pid_tmp = 0;
          } else {
            left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>
              (left_arm_ctrl_obs_B.vNum) - 1;
          }

          if (left_arm_ctrl_obs_B.b_idx_0_f > left_arm_ctrl_obs_B.b_idx_1_i) {
            left_arm_ctrl_obs_B.X_tmp = 0;
          } else {
            left_arm_ctrl_obs_B.X_tmp = static_cast<int32_T>
              (left_arm_ctrl_obs_B.b_idx_0_f) - 1;
          }

          left_arm_ctrl_obs_B.n_e = Hji->size[0];
          for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <
               left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.f++) {
            left_arm_ctrl_obs_B.b_j_a = Hji->size[1];
            for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j <
                 left_arm_ctrl_obs_B.b_j_a; left_arm_ctrl_obs_B.b_i_j++) {
              H->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_j)
                + H->size[0] * (left_arm_ctrl_obs_B.X_tmp +
                                left_arm_ctrl_obs_B.f)] = Hji->data[Hji->size[0]
                * left_arm_ctrl_obs_B.b_i_j + left_arm_ctrl_obs_B.f];
            }
          }
        }

        for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
             left_arm_ctrl_obs_B.f++) {
          for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
               left_arm_ctrl_obs_B.b_i_j++) {
            left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.b_i_j + 6 *
              left_arm_ctrl_obs_B.f] = X->data[left_arm_ctrl_obs_B.q_size_tmp].
              f1[6 * left_arm_ctrl_obs_B.b_i_j + left_arm_ctrl_obs_B.f];
          }
        }

        left_arm_ctrl_obs_B.f = B->size[0] * B->size[1];
        B->size[0] = 6;
        B->size[1] = Fi->size[1];
        left_emxEnsureCapacity_real_T_e(B, left_arm_ctrl_obs_B.f);
        left_arm_ctrl_obs_B.n_e = Fi->size[0] * Fi->size[1] - 1;
        for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
             left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.f++) {
          B->data[left_arm_ctrl_obs_B.f] = Fi->data[left_arm_ctrl_obs_B.f];
        }

        left_arm_ctrl_obs_B.n_e = Fi->size[1];
        left_arm_ctrl_obs_B.f = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = left_arm_ctrl_obs_B.n_e;
        left_emxEnsureCapacity_real_T_e(Fi, left_arm_ctrl_obs_B.f);
        for (left_arm_ctrl_obs_B.b_j_a = 0; left_arm_ctrl_obs_B.b_j_a <
             left_arm_ctrl_obs_B.n_e; left_arm_ctrl_obs_B.b_j_a++) {
          left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_j_a * 6 - 1;
          for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
               left_arm_ctrl_obs_B.b_i_j++) {
            Fi->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_j) +
              1] = 0.0;
          }

          for (left_arm_ctrl_obs_B.b_i_j = 0; left_arm_ctrl_obs_B.b_i_j < 6;
               left_arm_ctrl_obs_B.b_i_j++) {
            left_arm_ctrl_obs_B.aoffset_o = left_arm_ctrl_obs_B.b_i_j * 6 - 1;
            left_arm_ctrl_obs_B.temp = B->data[(left_arm_ctrl_obs_B.pid_tmp +
              left_arm_ctrl_obs_B.b_i_j) + 1];
            for (left_arm_ctrl_obs_B.c_i_g = 0; left_arm_ctrl_obs_B.c_i_g < 6;
                 left_arm_ctrl_obs_B.c_i_g++) {
              left_arm_ctrl_obs_B.i_b = left_arm_ctrl_obs_B.c_i_g + 1;
              left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.pid_tmp +
                left_arm_ctrl_obs_B.i_b;
              Fi->data[left_arm_ctrl_obs_B.f] +=
                left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.aoffset_o +
                left_arm_ctrl_obs_B.i_b] * left_arm_ctrl_obs_B.temp;
            }
          }
        }

        left_arm_ctrl_obs_B.pid = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp
          ]->ParentIndex;
      }
    }
  }

  left_arm_ctrl__emxFree_real_T_e(&B);
  left_arm_ctrl__emxFree_real_T_e(&a);
  left_arm_ctrl__emxFree_real_T_e(&Hji);
  left_arm_ctrl__emxFree_real_T_e(&Sj);
  left_arm_ctrl__emxFree_real_T_e(&Fi);
  left_arm_ctrl__emxFree_real_T_e(&Si);
  left_arm_c_emxFree_e_cell_wrap1(&X);
  left_arm_c_emxFree_e_cell_wrap1(&Ic);
}

static void left_arm_ct_emxInit_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_e_cell_wrap_left_arm_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_e_cell_wrap_left_arm_T *)malloc(sizeof
    (emxArray_e_cell_wrap_left_arm_T));
  emxArray = *pEmxArray;
  emxArray->data = (e_cell_wrap_left_arm_ctrl_obs_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void l_emxEnsureCapacity_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(i), sizeof
                     (e_cell_wrap_left_arm_ctrl_obs_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(e_cell_wrap_left_arm_ctrl_obs_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (e_cell_wrap_left_arm_ctrl_obs_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void rigidBodyJoint_get_JointAxis_e(const rigidBodyJoint_left_arm_ctr_e_T
  *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (left_arm_ctrl_obs_B.b_kstr_af = 0; left_arm_ctrl_obs_B.b_kstr_af < 8;
       left_arm_ctrl_obs_B.b_kstr_af++) {
    left_arm_ctrl_obs_B.b_p5[left_arm_ctrl_obs_B.b_kstr_af] =
      tmp[left_arm_ctrl_obs_B.b_kstr_af];
  }

  left_arm_ctrl_obs_B.b_bool_i = false;
  if (obj->Type->size[1] == 8) {
    left_arm_ctrl_obs_B.b_kstr_af = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_af - 1 < 8) {
        left_arm_ctrl_obs_B.kstr_d = left_arm_ctrl_obs_B.b_kstr_af - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr_d] !=
            left_arm_ctrl_obs_B.b_p5[left_arm_ctrl_obs_B.kstr_d]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_af++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_i = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (left_arm_ctrl_obs_B.b_bool_i) {
    guard1 = true;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr_af = 0; left_arm_ctrl_obs_B.b_kstr_af < 9;
         left_arm_ctrl_obs_B.b_kstr_af++) {
      left_arm_ctrl_obs_B.b_md[left_arm_ctrl_obs_B.b_kstr_af] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr_af];
    }

    left_arm_ctrl_obs_B.b_bool_i = false;
    if (obj->Type->size[1] == 9) {
      left_arm_ctrl_obs_B.b_kstr_af = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_af - 1 < 9) {
          left_arm_ctrl_obs_B.kstr_d = left_arm_ctrl_obs_B.b_kstr_af - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr_d] !=
              left_arm_ctrl_obs_B.b_md[left_arm_ctrl_obs_B.kstr_d]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_af++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_i = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_i) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void RigidBodyTree_forwardKinematics(k_robotics_manip_internal_R_e_T *obj,
  const real_T qvec[7], emxArray_e_cell_wrap_left_arm_T *Ttree)
{
  j_robotics_manip_internal_R_e_T *body;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  left_arm_ctrl_obs_B.n = obj->NumBodies;
  for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 16;
       left_arm_ctrl_obs_B.b_kstr_a++) {
    left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.b_kstr_a] =
      tmp[left_arm_ctrl_obs_B.b_kstr_a];
  }

  left_arm_ctrl_obs_B.ntilecols = static_cast<int32_T>(left_arm_ctrl_obs_B.n);
  left_arm_ctrl_obs_B.b_kstr_a = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  Ttree->size[1] = left_arm_ctrl_obs_B.ntilecols;
  l_emxEnsureCapacity_e_cell_wrap(Ttree, left_arm_ctrl_obs_B.b_kstr_a);
  if (left_arm_ctrl_obs_B.ntilecols != 0) {
    left_arm_ctrl_obs_B.ntilecols--;
    if (0 <= left_arm_ctrl_obs_B.ntilecols) {
      memcpy(&left_arm_ctrl_obs_B.expl_temp.f1[0], &left_arm_ctrl_obs_B.c_f1[0],
             sizeof(real_T) << 4U);
    }

    for (left_arm_ctrl_obs_B.b_jtilecol = 0; left_arm_ctrl_obs_B.b_jtilecol <=
         left_arm_ctrl_obs_B.ntilecols; left_arm_ctrl_obs_B.b_jtilecol++) {
      Ttree->data[left_arm_ctrl_obs_B.b_jtilecol] =
        left_arm_ctrl_obs_B.expl_temp;
    }
  }

  left_arm_ctrl_obs_B.k = 1.0;
  left_arm_ctrl_obs_B.ntilecols = static_cast<int32_T>(left_arm_ctrl_obs_B.n) -
    1;
  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  if (0 <= left_arm_ctrl_obs_B.ntilecols) {
    for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 5;
         left_arm_ctrl_obs_B.b_kstr_a++) {
      left_arm_ctrl_obs_B.b_me[left_arm_ctrl_obs_B.b_kstr_a] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr_a];
    }
  }

  for (left_arm_ctrl_obs_B.b_jtilecol = 0; left_arm_ctrl_obs_B.b_jtilecol <=
       left_arm_ctrl_obs_B.ntilecols; left_arm_ctrl_obs_B.b_jtilecol++) {
    body = obj->Bodies[left_arm_ctrl_obs_B.b_jtilecol];
    left_arm_ctrl_obs_B.n = body->JointInternal.PositionNumber;
    left_arm_ctrl_obs_B.n += left_arm_ctrl_obs_B.k;
    if (left_arm_ctrl_obs_B.k > left_arm_ctrl_obs_B.n - 1.0) {
      left_arm_ctrl_obs_B.e = 0;
      left_arm_ctrl_obs_B.d = 0;
    } else {
      left_arm_ctrl_obs_B.e = static_cast<int32_T>(left_arm_ctrl_obs_B.k) - 1;
      left_arm_ctrl_obs_B.d = static_cast<int32_T>(left_arm_ctrl_obs_B.n - 1.0);
    }

    left_arm_ctrl_obs_B.b_kstr_a = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    left_emxEnsureCapacity_char_T_e(switch_expression,
      left_arm_ctrl_obs_B.b_kstr_a);
    left_arm_ctrl_obs_B.loop_ub = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a <=
         left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.b_kstr_a++) {
      switch_expression->data[left_arm_ctrl_obs_B.b_kstr_a] =
        body->JointInternal.Type->data[left_arm_ctrl_obs_B.b_kstr_a];
    }

    left_arm_ctrl_obs_B.b_bool_n = false;
    if (switch_expression->size[1] == 5) {
      left_arm_ctrl_obs_B.b_kstr_a = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_a - 1 < 5) {
          left_arm_ctrl_obs_B.loop_ub = left_arm_ctrl_obs_B.b_kstr_a - 1;
          if (switch_expression->data[left_arm_ctrl_obs_B.loop_ub] !=
              left_arm_ctrl_obs_B.b_me[left_arm_ctrl_obs_B.loop_ub]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_a++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_n = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_n) {
      left_arm_ctrl_obs_B.b_kstr_a = 0;
    } else {
      for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 8;
           left_arm_ctrl_obs_B.b_kstr_a++) {
        left_arm_ctrl_obs_B.b_p[left_arm_ctrl_obs_B.b_kstr_a] =
          tmp_1[left_arm_ctrl_obs_B.b_kstr_a];
      }

      left_arm_ctrl_obs_B.b_bool_n = false;
      if (switch_expression->size[1] == 8) {
        left_arm_ctrl_obs_B.b_kstr_a = 1;
        do {
          exitg1 = 0;
          if (left_arm_ctrl_obs_B.b_kstr_a - 1 < 8) {
            left_arm_ctrl_obs_B.loop_ub = left_arm_ctrl_obs_B.b_kstr_a - 1;
            if (switch_expression->data[left_arm_ctrl_obs_B.loop_ub] !=
                left_arm_ctrl_obs_B.b_p[left_arm_ctrl_obs_B.loop_ub]) {
              exitg1 = 1;
            } else {
              left_arm_ctrl_obs_B.b_kstr_a++;
            }
          } else {
            left_arm_ctrl_obs_B.b_bool_n = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (left_arm_ctrl_obs_B.b_bool_n) {
        left_arm_ctrl_obs_B.b_kstr_a = 1;
      } else {
        left_arm_ctrl_obs_B.b_kstr_a = -1;
      }
    }

    switch (left_arm_ctrl_obs_B.b_kstr_a) {
     case 0:
      memset(&left_arm_ctrl_obs_B.c_f1[0], 0, sizeof(real_T) << 4U);
      left_arm_ctrl_obs_B.c_f1[0] = 1.0;
      left_arm_ctrl_obs_B.c_f1[5] = 1.0;
      left_arm_ctrl_obs_B.c_f1[10] = 1.0;
      left_arm_ctrl_obs_B.c_f1[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_e(&body->JointInternal,
        left_arm_ctrl_obs_B.v_jz);
      left_arm_ctrl_obs_B.d -= left_arm_ctrl_obs_B.e;
      for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a <
           left_arm_ctrl_obs_B.d; left_arm_ctrl_obs_B.b_kstr_a++) {
        left_arm_ctrl_obs_B.e_data[left_arm_ctrl_obs_B.b_kstr_a] =
          left_arm_ctrl_obs_B.e + left_arm_ctrl_obs_B.b_kstr_a;
      }

      left_arm_ctrl_obs_B.result_data_j[0] = left_arm_ctrl_obs_B.v_jz[0];
      left_arm_ctrl_obs_B.result_data_j[1] = left_arm_ctrl_obs_B.v_jz[1];
      left_arm_ctrl_obs_B.result_data_j[2] = left_arm_ctrl_obs_B.v_jz[2];
      if (0 <= (left_arm_ctrl_obs_B.d != 0) - 1) {
        left_arm_ctrl_obs_B.result_data_j[3] = qvec[left_arm_ctrl_obs_B.e_data[0]];
      }

      left_arm_ctrl_obs_B.k = 1.0 / sqrt((left_arm_ctrl_obs_B.result_data_j[0] *
        left_arm_ctrl_obs_B.result_data_j[0] +
        left_arm_ctrl_obs_B.result_data_j[1] *
        left_arm_ctrl_obs_B.result_data_j[1]) +
        left_arm_ctrl_obs_B.result_data_j[2] *
        left_arm_ctrl_obs_B.result_data_j[2]);
      left_arm_ctrl_obs_B.v_jz[0] = left_arm_ctrl_obs_B.result_data_j[0] *
        left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.v_jz[1] = left_arm_ctrl_obs_B.result_data_j[1] *
        left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.v_jz[2] = left_arm_ctrl_obs_B.result_data_j[2] *
        left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.k = cos(left_arm_ctrl_obs_B.result_data_j[3]);
      left_arm_ctrl_obs_B.sth_c = sin(left_arm_ctrl_obs_B.result_data_j[3]);
      left_arm_ctrl_obs_B.tempR_n[0] = left_arm_ctrl_obs_B.v_jz[0] *
        left_arm_ctrl_obs_B.v_jz[0] * (1.0 - left_arm_ctrl_obs_B.k) +
        left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.tempR_tmp_f = left_arm_ctrl_obs_B.v_jz[1] *
        left_arm_ctrl_obs_B.v_jz[0] * (1.0 - left_arm_ctrl_obs_B.k);
      left_arm_ctrl_obs_B.tempR_tmp_p = left_arm_ctrl_obs_B.v_jz[2] *
        left_arm_ctrl_obs_B.sth_c;
      left_arm_ctrl_obs_B.tempR_n[1] = left_arm_ctrl_obs_B.tempR_tmp_f -
        left_arm_ctrl_obs_B.tempR_tmp_p;
      left_arm_ctrl_obs_B.tempR_tmp_e = left_arm_ctrl_obs_B.v_jz[2] *
        left_arm_ctrl_obs_B.v_jz[0] * (1.0 - left_arm_ctrl_obs_B.k);
      left_arm_ctrl_obs_B.tempR_tmp_o4 = left_arm_ctrl_obs_B.v_jz[1] *
        left_arm_ctrl_obs_B.sth_c;
      left_arm_ctrl_obs_B.tempR_n[2] = left_arm_ctrl_obs_B.tempR_tmp_e +
        left_arm_ctrl_obs_B.tempR_tmp_o4;
      left_arm_ctrl_obs_B.tempR_n[3] = left_arm_ctrl_obs_B.tempR_tmp_f +
        left_arm_ctrl_obs_B.tempR_tmp_p;
      left_arm_ctrl_obs_B.tempR_n[4] = left_arm_ctrl_obs_B.v_jz[1] *
        left_arm_ctrl_obs_B.v_jz[1] * (1.0 - left_arm_ctrl_obs_B.k) +
        left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.tempR_tmp_f = left_arm_ctrl_obs_B.v_jz[2] *
        left_arm_ctrl_obs_B.v_jz[1] * (1.0 - left_arm_ctrl_obs_B.k);
      left_arm_ctrl_obs_B.tempR_tmp_p = left_arm_ctrl_obs_B.v_jz[0] *
        left_arm_ctrl_obs_B.sth_c;
      left_arm_ctrl_obs_B.tempR_n[5] = left_arm_ctrl_obs_B.tempR_tmp_f -
        left_arm_ctrl_obs_B.tempR_tmp_p;
      left_arm_ctrl_obs_B.tempR_n[6] = left_arm_ctrl_obs_B.tempR_tmp_e -
        left_arm_ctrl_obs_B.tempR_tmp_o4;
      left_arm_ctrl_obs_B.tempR_n[7] = left_arm_ctrl_obs_B.tempR_tmp_f +
        left_arm_ctrl_obs_B.tempR_tmp_p;
      left_arm_ctrl_obs_B.tempR_n[8] = left_arm_ctrl_obs_B.v_jz[2] *
        left_arm_ctrl_obs_B.v_jz[2] * (1.0 - left_arm_ctrl_obs_B.k) +
        left_arm_ctrl_obs_B.k;
      for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 3;
           left_arm_ctrl_obs_B.b_kstr_a++) {
        left_arm_ctrl_obs_B.e = left_arm_ctrl_obs_B.b_kstr_a + 1;
        left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.e - 1] =
          left_arm_ctrl_obs_B.tempR_n[(left_arm_ctrl_obs_B.e - 1) * 3];
        left_arm_ctrl_obs_B.e = left_arm_ctrl_obs_B.b_kstr_a + 1;
        left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.e + 2] =
          left_arm_ctrl_obs_B.tempR_n[(left_arm_ctrl_obs_B.e - 1) * 3 + 1];
        left_arm_ctrl_obs_B.e = left_arm_ctrl_obs_B.b_kstr_a + 1;
        left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.e + 5] =
          left_arm_ctrl_obs_B.tempR_n[(left_arm_ctrl_obs_B.e - 1) * 3 + 2];
      }

      memset(&left_arm_ctrl_obs_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 3;
           left_arm_ctrl_obs_B.b_kstr_a++) {
        left_arm_ctrl_obs_B.d = left_arm_ctrl_obs_B.b_kstr_a << 2;
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d] =
          left_arm_ctrl_obs_B.R_b[3 * left_arm_ctrl_obs_B.b_kstr_a];
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 1] =
          left_arm_ctrl_obs_B.R_b[3 * left_arm_ctrl_obs_B.b_kstr_a + 1];
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 2] =
          left_arm_ctrl_obs_B.R_b[3 * left_arm_ctrl_obs_B.b_kstr_a + 2];
      }

      left_arm_ctrl_obs_B.c_f1[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_e(&body->JointInternal,
        left_arm_ctrl_obs_B.v_jz);
      memset(&left_arm_ctrl_obs_B.tempR_n[0], 0, 9U * sizeof(real_T));
      left_arm_ctrl_obs_B.tempR_n[0] = 1.0;
      left_arm_ctrl_obs_B.tempR_n[4] = 1.0;
      left_arm_ctrl_obs_B.tempR_n[8] = 1.0;
      for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 3;
           left_arm_ctrl_obs_B.b_kstr_a++) {
        left_arm_ctrl_obs_B.d = left_arm_ctrl_obs_B.b_kstr_a << 2;
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d] =
          left_arm_ctrl_obs_B.tempR_n[3 * left_arm_ctrl_obs_B.b_kstr_a];
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 1] =
          left_arm_ctrl_obs_B.tempR_n[3 * left_arm_ctrl_obs_B.b_kstr_a + 1];
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 2] =
          left_arm_ctrl_obs_B.tempR_n[3 * left_arm_ctrl_obs_B.b_kstr_a + 2];
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.b_kstr_a + 12] =
          left_arm_ctrl_obs_B.v_jz[left_arm_ctrl_obs_B.b_kstr_a] *
          qvec[left_arm_ctrl_obs_B.e];
      }

      left_arm_ctrl_obs_B.c_f1[3] = 0.0;
      left_arm_ctrl_obs_B.c_f1[7] = 0.0;
      left_arm_ctrl_obs_B.c_f1[11] = 0.0;
      left_arm_ctrl_obs_B.c_f1[15] = 1.0;
      break;
    }

    for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 16;
         left_arm_ctrl_obs_B.b_kstr_a++) {
      left_arm_ctrl_obs_B.a_g[left_arm_ctrl_obs_B.b_kstr_a] =
        body->JointInternal.JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_a];
    }

    for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 16;
         left_arm_ctrl_obs_B.b_kstr_a++) {
      left_arm_ctrl_obs_B.b_m[left_arm_ctrl_obs_B.b_kstr_a] =
        body->JointInternal.ChildToJointTransform[left_arm_ctrl_obs_B.b_kstr_a];
    }

    for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 4;
         left_arm_ctrl_obs_B.b_kstr_a++) {
      for (left_arm_ctrl_obs_B.e = 0; left_arm_ctrl_obs_B.e < 4;
           left_arm_ctrl_obs_B.e++) {
        left_arm_ctrl_obs_B.d = left_arm_ctrl_obs_B.e << 2;
        left_arm_ctrl_obs_B.loop_ub = left_arm_ctrl_obs_B.b_kstr_a +
          left_arm_ctrl_obs_B.d;
        left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.loop_ub] = 0.0;
        left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.loop_ub] +=
          left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d] *
          left_arm_ctrl_obs_B.a_g[left_arm_ctrl_obs_B.b_kstr_a];
        left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.loop_ub] +=
          left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 1] *
          left_arm_ctrl_obs_B.a_g[left_arm_ctrl_obs_B.b_kstr_a + 4];
        left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.loop_ub] +=
          left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 2] *
          left_arm_ctrl_obs_B.a_g[left_arm_ctrl_obs_B.b_kstr_a + 8];
        left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.loop_ub] +=
          left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 3] *
          left_arm_ctrl_obs_B.a_g[left_arm_ctrl_obs_B.b_kstr_a + 12];
      }

      for (left_arm_ctrl_obs_B.e = 0; left_arm_ctrl_obs_B.e < 4;
           left_arm_ctrl_obs_B.e++) {
        left_arm_ctrl_obs_B.d = left_arm_ctrl_obs_B.e << 2;
        left_arm_ctrl_obs_B.loop_ub = left_arm_ctrl_obs_B.b_kstr_a +
          left_arm_ctrl_obs_B.d;
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.loop_ub] = 0.0;
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.loop_ub] +=
          left_arm_ctrl_obs_B.b_m[left_arm_ctrl_obs_B.d] *
          left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.b_kstr_a];
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.loop_ub] +=
          left_arm_ctrl_obs_B.b_m[left_arm_ctrl_obs_B.d + 1] *
          left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.b_kstr_a + 4];
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.loop_ub] +=
          left_arm_ctrl_obs_B.b_m[left_arm_ctrl_obs_B.d + 2] *
          left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.b_kstr_a + 8];
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.loop_ub] +=
          left_arm_ctrl_obs_B.b_m[left_arm_ctrl_obs_B.d + 3] *
          left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.b_kstr_a + 12];
      }
    }

    left_arm_ctrl_obs_B.k = left_arm_ctrl_obs_B.n;
    if (body->ParentIndex > 0.0) {
      for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 16;
           left_arm_ctrl_obs_B.b_kstr_a++) {
        left_arm_ctrl_obs_B.a_g[left_arm_ctrl_obs_B.b_kstr_a] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[left_arm_ctrl_obs_B.b_kstr_a];
      }

      for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 4;
           left_arm_ctrl_obs_B.b_kstr_a++) {
        for (left_arm_ctrl_obs_B.e = 0; left_arm_ctrl_obs_B.e < 4;
             left_arm_ctrl_obs_B.e++) {
          left_arm_ctrl_obs_B.d = left_arm_ctrl_obs_B.e << 2;
          left_arm_ctrl_obs_B.loop_ub = left_arm_ctrl_obs_B.b_kstr_a +
            left_arm_ctrl_obs_B.d;
          left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.loop_ub] = 0.0;
          left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.loop_ub] += Ttree->
            data[left_arm_ctrl_obs_B.b_jtilecol].f1[left_arm_ctrl_obs_B.d] *
            left_arm_ctrl_obs_B.a_g[left_arm_ctrl_obs_B.b_kstr_a];
          left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.loop_ub] += Ttree->
            data[left_arm_ctrl_obs_B.b_jtilecol].f1[left_arm_ctrl_obs_B.d + 1] *
            left_arm_ctrl_obs_B.a_g[left_arm_ctrl_obs_B.b_kstr_a + 4];
          left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.loop_ub] += Ttree->
            data[left_arm_ctrl_obs_B.b_jtilecol].f1[left_arm_ctrl_obs_B.d + 2] *
            left_arm_ctrl_obs_B.a_g[left_arm_ctrl_obs_B.b_kstr_a + 8];
          left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.loop_ub] += Ttree->
            data[left_arm_ctrl_obs_B.b_jtilecol].f1[left_arm_ctrl_obs_B.d + 3] *
            left_arm_ctrl_obs_B.a_g[left_arm_ctrl_obs_B.b_kstr_a + 12];
        }
      }

      for (left_arm_ctrl_obs_B.b_kstr_a = 0; left_arm_ctrl_obs_B.b_kstr_a < 16;
           left_arm_ctrl_obs_B.b_kstr_a++) {
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.b_kstr_a] =
          left_arm_ctrl_obs_B.a_n[left_arm_ctrl_obs_B.b_kstr_a];
      }
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
}

static void left_arm_ct_emxFree_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_e_cell_wrap_left_arm_T *)NULL) {
    if (((*pEmxArray)->data != (e_cell_wrap_left_arm_ctrl_obs_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_e_cell_wrap_left_arm_T *)NULL;
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static void left_arm_ctrl_obs_atan2(const real_T y_data[], const int32_T y_size
  [3], const real_T x_data[], const int32_T x_size[3], real_T r_data[], int32_T
  r_size[3])
{
  int32_T csz_idx_2;
  if (y_size[2] <= x_size[2]) {
    csz_idx_2 = y_size[2];
  } else {
    csz_idx_2 = 0;
  }

  r_size[2] = csz_idx_2;
  r_size[0] = 1;
  r_size[1] = 1;
  if (0 <= csz_idx_2 - 1) {
    r_data[0] = rt_atan2d_snf(y_data[0], x_data[0]);
  }
}

static void left_arm_ctrl_matlabCodegenHa_l(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCo_e0(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_rigidBodyJoint_e(rigidBodyJoint_left_arm_ctrl__T
  *pStruct)
{
  left_arm_ctrl__emxFree_char_T_e(&pStruct->Type);
  left_arm_ctrl__emxFree_real_T_e(&pStruct->MotionSubspace);
}

static void emxFreeStruct_i_robotics_mani_e(i_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_rigidBodyJoint_e(&pStruct->JointInternal);
}

static void emxFreeStruct_k_robotics_mani_e(k_robotics_manip_internal_e0h_T
  *pStruct)
{
  emxFreeStruct_i_robotics_mani_e(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_e(robotics_slmanip_internal_e0h_T
  *pStruct)
{
  emxFreeStruct_k_robotics_mani_e(&pStruct->TreeInternal);
}

static void emxFreeStruct_j_robotics_mani_e(j_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_rigidBodyJoint_e(&pStruct->JointInternal);
}

static void emxFreeStruct_k_robotics_man_e0(k_robotics_manip_internal__e0_T
  *pStruct)
{
  emxFreeStruct_i_robotics_mani_e(&pStruct->Base);
}

static void emxFreeStruct_robotics_slman_e0(robotics_slmanip_internal__e0_T
  *pStruct)
{
  emxFreeStruct_k_robotics_man_e0(&pStruct->TreeInternal);
}

static void l_emxFreeStruct_rigidBodyJoint1(rigidBodyJoint_left_arm_ctr_e_T
  *pStruct)
{
  left_arm_ctrl__emxFree_char_T_e(&pStruct->Type);
}

static void emxFreeStruct_i_robotics_man_e0(i_robotics_manip_internal_R_e_T
  *pStruct)
{
  left_arm_ctrl__emxFree_char_T_e(&pStruct->NameInternal);
  l_emxFreeStruct_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_k_robotics_ma_e0h(k_robotics_manip_internal_R_e_T
  *pStruct)
{
  emxFreeStruct_i_robotics_man_e0(&pStruct->Base);
}

static void emxFreeStruct_robotics_slma_e0h(robotics_slmanip_internal_b_e_T
  *pStruct)
{
  emxFreeStruct_k_robotics_ma_e0h(&pStruct->TreeInternal);
}

static void emxFreeStruct_j_robotics_man_e0(j_robotics_manip_internal_R_e_T
  *pStruct)
{
  left_arm_ctrl__emxFree_char_T_e(&pStruct->NameInternal);
  l_emxFreeStruct_rigidBodyJoint1(&pStruct->JointInternal);
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxInitStruct_rigidBodyJoint_e(rigidBodyJoint_left_arm_ctrl__T
  *pStruct)
{
  left_arm_ctrl__emxInit_char_T_e(&pStruct->Type, 2);
  left_arm_ctrl__emxInit_real_T_e(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_i_robotics_mani_e(i_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_rigidBodyJoint_e(&pStruct->JointInternal);
}

static void emxInitStruct_k_robotics_mani_e(k_robotics_manip_internal_e0h_T
  *pStruct)
{
  emxInitStruct_i_robotics_mani_e(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_e(robotics_slmanip_internal_e0h_T
  *pStruct)
{
  emxInitStruct_k_robotics_mani_e(&pStruct->TreeInternal);
}

static void emxInitStruct_j_robotics_mani_e(j_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_rigidBodyJoint_e(&pStruct->JointInternal);
}

static j_robotics_manip_internal_Rig_T *le_RigidBody_RigidBody_e0h4ewmd
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.30007875, 0.00021157499999999998, 0.0, 0.0,
    -0.0, -0.0052499999999999995, 0.00021157499999999998, 0.3005684315, 0.0, 0.0,
    0.0, -0.014105, 0.0, 0.0, 0.30064718149999997, 0.0052499999999999995,
    0.014105, 0.0, 0.0, 0.0, 0.0052499999999999995, 0.35, 0.0, 0.0, -0.0, 0.0,
    0.014105, 0.0, 0.35, 0.0, -0.0052499999999999995, -0.014105, 0.0, 0.0, 0.0,
    0.35 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.055, 0.09, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 0.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *l_RigidBody_RigidBody_e0h4ewmdi
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.302205,
    0.0, 0.0, 0.0, -0.021, 0.0, 0.0, 0.302205, -0.0, 0.021, 0.0, 0.0, 0.0, -0.0,
    0.2, 0.0, 0.0, -0.0, 0.0, 0.021, 0.0, 0.2, 0.0, 0.0, -0.021, 0.0, 0.0, 0.0,
    0.2 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0603, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 1.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *RigidBody_RigidBody_e0h4ewmdid
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.307938,
    0.0, 0.0, 0.0, -0.0378, 0.0, 0.0, 0.307938, -0.0, 0.0378, 0.0, 0.0, 0.0,
    -0.0, 0.18, 0.0, 0.0, -0.0, 0.0, 0.0378, 0.0, 0.18, 0.0, 0.0, -0.0378, 0.0,
    0.0, 0.0, 0.18 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 2.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *RigidBody_RigidBody_e0h4ewmdidb
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.301458,
    0.0, 0.0, 0.0, -0.0162, 0.0, 0.0, 0.301458, -0.0, 0.0162, 0.0, 0.0, 0.0,
    -0.0, 0.18, 0.0, 0.0, -0.0, 0.0, 0.0162, 0.0, 0.18, 0.0, 0.0, -0.0162, 0.0,
    0.0, 0.0, 0.18 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.27, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 3.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *RigidBody_RigidBod_e0h4ewmdidbj
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.305832,
    0.0, 0.0, 0.0, -0.0324, 0.0, 0.0, 0.305832, -0.0, 0.0324, 0.0, 0.0, 0.0,
    -0.0, 0.18, 0.0, 0.0, -0.0, 0.0, 0.0324, 0.0, 0.18, 0.0, 0.0, -0.0324, 0.0,
    0.0, 0.0, 0.18 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 4.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *RigidBody_RigidBo_e0h4ewmdidbjt
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.300375,
    0.0, 0.0, 0.0, -0.0075, 0.0, 0.0, 0.300375, -0.0, 0.0075, 0.0, 0.0, 0.0,
    -0.0, 0.15, 0.0, 0.0, -0.0, 0.0, 0.0075, 0.0, 0.15, 0.0, 0.0, -0.0075, 0.0,
    0.0, 0.0, 0.15 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.2126, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 5.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *RigidBody_RigidB_e0h4ewmdidbjtq
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.3015,
    0.0, 0.0, 0.0, -0.015, 0.0, 0.0, 0.3015, -0.0, 0.015, 0.0, 0.0, 0.0, -0.0,
    0.15, 0.0, 0.0, -0.0, 0.0, 0.015, 0.0, 0.15, 0.0, 0.0, -0.015, 0.0, 0.0, 0.0,
    0.15 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 6.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *RigidBody_Rigid_e0h4ewmdidbjtqt
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.13, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 7.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *l_RigidBody_Rigid_e
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, -0.04, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 8.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *l_RigidBody_Rigid_i
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  int8_T tmp[6];
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.04, 0.0, 1.0 };

  static const real_T tmp_5[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  obj->ParentIndex = 8.0;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_2[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_3[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_4[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static i_robotics_manip_internal_Rig_T *l_RigidBody_Rigid_n
  (i_robotics_manip_internal_Rig_T *obj)
{
  i_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  left_arm_ctrl_obs_B.b_kstr_l = obj->JointInternal.Type->size[0] *
    obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type,
    left_arm_ctrl_obs_B.b_kstr_l);
  for (left_arm_ctrl_obs_B.b_kstr_l = 0; left_arm_ctrl_obs_B.b_kstr_l < 5;
       left_arm_ctrl_obs_B.b_kstr_l++) {
    obj->JointInternal.Type->data[left_arm_ctrl_obs_B.b_kstr_l] =
      tmp[left_arm_ctrl_obs_B.b_kstr_l];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  left_arm_ctrl_obs_B.b_kstr_l = switch_expression->size[0] *
    switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression,
    left_arm_ctrl_obs_B.b_kstr_l);
  left_arm_ctrl_obs_B.loop_ub_p = obj->JointInternal.Type->size[0] *
    obj->JointInternal.Type->size[1] - 1;
  for (left_arm_ctrl_obs_B.b_kstr_l = 0; left_arm_ctrl_obs_B.b_kstr_l <=
       left_arm_ctrl_obs_B.loop_ub_p; left_arm_ctrl_obs_B.b_kstr_l++) {
    switch_expression->data[left_arm_ctrl_obs_B.b_kstr_l] =
      obj->JointInternal.Type->data[left_arm_ctrl_obs_B.b_kstr_l];
  }

  for (left_arm_ctrl_obs_B.b_kstr_l = 0; left_arm_ctrl_obs_B.b_kstr_l < 8;
       left_arm_ctrl_obs_B.b_kstr_l++) {
    left_arm_ctrl_obs_B.b_a[left_arm_ctrl_obs_B.b_kstr_l] =
      tmp_0[left_arm_ctrl_obs_B.b_kstr_l];
  }

  left_arm_ctrl_obs_B.b_bool_m = false;
  if (switch_expression->size[1] == 8) {
    left_arm_ctrl_obs_B.b_kstr_l = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_l - 1 < 8) {
        left_arm_ctrl_obs_B.loop_ub_p = left_arm_ctrl_obs_B.b_kstr_l - 1;
        if (switch_expression->data[left_arm_ctrl_obs_B.loop_ub_p] !=
            left_arm_ctrl_obs_B.b_a[left_arm_ctrl_obs_B.loop_ub_p]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_l++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_m = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (left_arm_ctrl_obs_B.b_bool_m) {
    left_arm_ctrl_obs_B.b_kstr_l = 0;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr_l = 0; left_arm_ctrl_obs_B.b_kstr_l < 9;
         left_arm_ctrl_obs_B.b_kstr_l++) {
      left_arm_ctrl_obs_B.b_m3[left_arm_ctrl_obs_B.b_kstr_l] =
        tmp_1[left_arm_ctrl_obs_B.b_kstr_l];
    }

    left_arm_ctrl_obs_B.b_bool_m = false;
    if (switch_expression->size[1] == 9) {
      left_arm_ctrl_obs_B.b_kstr_l = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_l - 1 < 9) {
          left_arm_ctrl_obs_B.loop_ub_p = left_arm_ctrl_obs_B.b_kstr_l - 1;
          if (switch_expression->data[left_arm_ctrl_obs_B.loop_ub_p] !=
              left_arm_ctrl_obs_B.b_m3[left_arm_ctrl_obs_B.loop_ub_p]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_l++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_m = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_m) {
      left_arm_ctrl_obs_B.b_kstr_l = 1;
    } else {
      left_arm_ctrl_obs_B.b_kstr_l = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (left_arm_ctrl_obs_B.b_kstr_l) {
   case 0:
    left_arm_ctrl_obs_B.iv[0] = 0;
    left_arm_ctrl_obs_B.iv[1] = 0;
    left_arm_ctrl_obs_B.iv[2] = 1;
    left_arm_ctrl_obs_B.iv[3] = 0;
    left_arm_ctrl_obs_B.iv[4] = 0;
    left_arm_ctrl_obs_B.iv[5] = 0;
    for (left_arm_ctrl_obs_B.b_kstr_l = 0; left_arm_ctrl_obs_B.b_kstr_l < 6;
         left_arm_ctrl_obs_B.b_kstr_l++) {
      left_arm_ctrl_obs_B.msubspace_data[left_arm_ctrl_obs_B.b_kstr_l] =
        left_arm_ctrl_obs_B.iv[left_arm_ctrl_obs_B.b_kstr_l];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    left_arm_ctrl_obs_B.iv[0] = 0;
    left_arm_ctrl_obs_B.iv[1] = 0;
    left_arm_ctrl_obs_B.iv[2] = 0;
    left_arm_ctrl_obs_B.iv[3] = 0;
    left_arm_ctrl_obs_B.iv[4] = 0;
    left_arm_ctrl_obs_B.iv[5] = 1;
    for (left_arm_ctrl_obs_B.b_kstr_l = 0; left_arm_ctrl_obs_B.b_kstr_l < 6;
         left_arm_ctrl_obs_B.b_kstr_l++) {
      left_arm_ctrl_obs_B.msubspace_data[left_arm_ctrl_obs_B.b_kstr_l] =
        left_arm_ctrl_obs_B.iv[left_arm_ctrl_obs_B.b_kstr_l];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (left_arm_ctrl_obs_B.b_kstr_l = 0; left_arm_ctrl_obs_B.b_kstr_l < 6;
         left_arm_ctrl_obs_B.b_kstr_l++) {
      left_arm_ctrl_obs_B.msubspace_data[left_arm_ctrl_obs_B.b_kstr_l] = 0;
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  left_arm_ctrl_obs_B.b_kstr_l = obj->JointInternal.MotionSubspace->size[0] *
    obj->JointInternal.MotionSubspace->size[1];
  obj->JointInternal.MotionSubspace->size[0] = 6;
  obj->JointInternal.MotionSubspace->size[1] = 1;
  left_emxEnsureCapacity_real_T_e(obj->JointInternal.MotionSubspace,
    left_arm_ctrl_obs_B.b_kstr_l);
  for (left_arm_ctrl_obs_B.b_kstr_l = 0; left_arm_ctrl_obs_B.b_kstr_l < 6;
       left_arm_ctrl_obs_B.b_kstr_l++) {
    obj->JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.b_kstr_l] =
      left_arm_ctrl_obs_B.msubspace_data[left_arm_ctrl_obs_B.b_kstr_l];
  }

  return b_obj;
}

static k_robotics_manip_internal_e0h_T *RigidBodyTree_RigidBodyTree_e0h
  (k_robotics_manip_internal_e0h_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9)
{
  k_robotics_manip_internal_e0h_T *b_obj;
  int32_T i;
  static const int8_T tmp[20] = { 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 1, 2, 3, 4, 5, 6,
    7, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = le_RigidBody_RigidBody_e0h4ewmd(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = l_RigidBody_RigidBody_e0h4ewmdi(iobj_1);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = RigidBody_RigidBody_e0h4ewmdid(iobj_2);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = RigidBody_RigidBody_e0h4ewmdidb(iobj_3);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = RigidBody_RigidBod_e0h4ewmdidbj(iobj_4);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = RigidBody_RigidBo_e0h4ewmdidbjt(iobj_5);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = RigidBody_RigidB_e0h4ewmdidbjtq(iobj_6);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = RigidBody_Rigid_e0h4ewmdidbjtqt(iobj_7);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = l_RigidBody_Rigid_e(iobj_8);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = l_RigidBody_Rigid_i(iobj_9);
  obj->Bodies[9]->Index = 10.0;
  obj->NumBodies = 10.0;
  obj->Gravity[0] = 9.81;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = 0.0;
  for (i = 0; i < 20; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  for (i = 0; i < 20; i++) {
    obj->VelocityDoFMap[i] = tmp[i];
  }

  l_RigidBody_Rigid_n(&obj->Base);
  return b_obj;
}

static void emxInitStruct_k_robotics_man_e0(k_robotics_manip_internal__e0_T
  *pStruct)
{
  emxInitStruct_i_robotics_mani_e(&pStruct->Base);
}

static void emxInitStruct_robotics_slman_e0(robotics_slmanip_internal__e0_T
  *pStruct)
{
  emxInitStruct_k_robotics_man_e0(&pStruct->TreeInternal);
}

static k_robotics_manip_internal__e0_T *RigidBodyTree_RigidBodyTree_e0
  (k_robotics_manip_internal__e0_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9)
{
  k_robotics_manip_internal__e0_T *b_obj;
  static const int8_T tmp[20] = { 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 1, 2, 3, 4, 5, 6,
    7, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = le_RigidBody_RigidBody_e0h4ewmd(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = l_RigidBody_RigidBody_e0h4ewmdi(iobj_1);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = RigidBody_RigidBody_e0h4ewmdid(iobj_2);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = RigidBody_RigidBody_e0h4ewmdidb(iobj_3);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = RigidBody_RigidBod_e0h4ewmdidbj(iobj_4);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = RigidBody_RigidBo_e0h4ewmdidbjt(iobj_5);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = RigidBody_RigidB_e0h4ewmdidbjtq(iobj_6);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = RigidBody_Rigid_e0h4ewmdidbjtqt(iobj_7);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = l_RigidBody_Rigid_e(iobj_8);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = l_RigidBody_Rigid_i(iobj_9);
  obj->Bodies[9]->Index = 10.0;
  obj->NumBodies = 10.0;
  obj->VelocityNumber = 7.0;
  for (left_arm_ctrl_obs_B.i3 = 0; left_arm_ctrl_obs_B.i3 < 20;
       left_arm_ctrl_obs_B.i3++) {
    obj->PositionDoFMap[left_arm_ctrl_obs_B.i3] = tmp[left_arm_ctrl_obs_B.i3];
  }

  for (left_arm_ctrl_obs_B.i3 = 0; left_arm_ctrl_obs_B.i3 < 20;
       left_arm_ctrl_obs_B.i3++) {
    obj->VelocityDoFMap[left_arm_ctrl_obs_B.i3] = tmp[left_arm_ctrl_obs_B.i3];
  }

  l_RigidBody_Rigid_n(&obj->Base);
  return b_obj;
}

static void l_emxInitStruct_rigidBodyJoint1(rigidBodyJoint_left_arm_ctr_e_T
  *pStruct)
{
  left_arm_ctrl__emxInit_char_T_e(&pStruct->Type, 2);
}

static void emxInitStruct_i_robotics_man_e0(i_robotics_manip_internal_R_e_T
  *pStruct)
{
  left_arm_ctrl__emxInit_char_T_e(&pStruct->NameInternal, 2);
  l_emxInitStruct_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_k_robotics_ma_e0h(k_robotics_manip_internal_R_e_T
  *pStruct)
{
  emxInitStruct_i_robotics_man_e0(&pStruct->Base);
}

static void emxInitStruct_robotics_slma_e0h(robotics_slmanip_internal_b_e_T
  *pStruct)
{
  emxInitStruct_k_robotics_ma_e0h(&pStruct->TreeInternal);
}

static void emxInitStruct_j_robotics_man_e0(j_robotics_manip_internal_R_e_T
  *pStruct)
{
  left_arm_ctrl__emxInit_char_T_e(&pStruct->NameInternal, 2);
  l_emxInitStruct_rigidBodyJoint1(&pStruct->JointInternal);
}

static j_robotics_manip_internal_R_e_T *left_arm__RigidBody_RigidBody_e
  (j_robotics_manip_internal_R_e_T *obj)
{
  j_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[14] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'l', 'i', 'n', 'k', '1' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.055, 0.09, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_emxEnsureCapacity_char_T_e(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *left_arm_RigidBody_RigidBody_e0
  (j_robotics_manip_internal_R_e_T *obj)
{
  j_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[14] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'l', 'i', 'n', 'k', '2' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0603, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_emxEnsureCapacity_char_T_e(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *left_ar_RigidBody_RigidBody_e0h
  (j_robotics_manip_internal_R_e_T *obj)
{
  j_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[14] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'l', 'i', 'n', 'k', '3' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_emxEnsureCapacity_char_T_e(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *left_a_RigidBody_RigidBody_e0h4
  (j_robotics_manip_internal_R_e_T *obj)
{
  j_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[14] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'l', 'i', 'n', 'k', '4' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.27, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_emxEnsureCapacity_char_T_e(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *left__RigidBody_RigidBody_e0h4e
  (j_robotics_manip_internal_R_e_T *obj)
{
  j_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[14] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'l', 'i', 'n', 'k', '5' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_emxEnsureCapacity_char_T_e(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *left_RigidBody_RigidBody_e0h4ew
  (j_robotics_manip_internal_R_e_T *obj)
{
  j_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[14] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'l', 'i', 'n', 'k', '6' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.2126, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_emxEnsureCapacity_char_T_e(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 5.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *lef_RigidBody_RigidBody_e0h4ewm
  (j_robotics_manip_internal_R_e_T *obj)
{
  j_robotics_manip_internal_R_e_T *b_obj;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[14] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'l', 'i', 'n', 'k', '7' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_emxEnsureCapacity_char_T_e(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 6.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_emxEnsureCapacity_char_T_e(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1]
    - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_0[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_1[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static k_robotics_manip_internal_R_e_T *l_RigidBodyTree_RigidBodyTree_e
  (k_robotics_manip_internal_R_e_T *obj, j_robotics_manip_internal_R_e_T *iobj_0,
   j_robotics_manip_internal_R_e_T *iobj_1, j_robotics_manip_internal_R_e_T
   *iobj_2, j_robotics_manip_internal_R_e_T *iobj_3,
   j_robotics_manip_internal_R_e_T *iobj_4, j_robotics_manip_internal_R_e_T
   *iobj_5, j_robotics_manip_internal_R_e_T *iobj_6,
   j_robotics_manip_internal_R_e_T *iobj_7, j_robotics_manip_internal_R_e_T
   *iobj_8, j_robotics_manip_internal_R_e_T *iobj_9)
{
  k_robotics_manip_internal_R_e_T *b_obj;
  i_robotics_manip_internal_R_e_T *obj_0;
  emxArray_char_T_left_arm_ctrl_T *switch_expression;
  boolean_T b_bool;
  int32_T b_kstr;
  char_T b[8];
  char_T b_0[9];
  int32_T loop_ub;
  static const char_T tmp[20] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'g', 'r', 'i', 'p', '_', 'c', 'e', 'n', 't', 'e', 'r' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.13, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_5[18] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'g', 'r', 'i', 'p', '_', 'l', 'e', 'f', 't' };

  static const real_T tmp_6[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, -0.04, 0.0, 1.0 };

  static const char_T tmp_7[19] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'g', 'r', 'i', 'p', '_', 'r', 'i', 'g', 'h', 't' };

  static const real_T tmp_8[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.04, 0.0, 1.0 };

  static const char_T tmp_9[19] = { 's', 'h', 'o', 'u', 'l', 'd', 'e', 'r', 's',
    '_', 'l', 'e', 'f', 't', '_', 'l', 'i', 'n', 'k' };

  int32_T exitg1;
  b_obj = obj;
  obj->Bodies[0] = left_arm__RigidBody_RigidBody_e(iobj_0);
  obj->Bodies[1] = left_arm_RigidBody_RigidBody_e0(iobj_1);
  obj->Bodies[2] = left_ar_RigidBody_RigidBody_e0h(iobj_2);
  obj->Bodies[3] = left_a_RigidBody_RigidBody_e0h4(iobj_3);
  obj->Bodies[4] = left__RigidBody_RigidBody_e0h4e(iobj_4);
  obj->Bodies[5] = left_RigidBody_RigidBody_e0h4ew(iobj_5);
  obj->Bodies[6] = lef_RigidBody_RigidBody_e0h4ewm(iobj_6);
  b_kstr = iobj_7->NameInternal->size[0] * iobj_7->NameInternal->size[1];
  iobj_7->NameInternal->size[0] = 1;
  iobj_7->NameInternal->size[1] = 20;
  left_emxEnsureCapacity_char_T_e(iobj_7->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 20; b_kstr++) {
    iobj_7->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  iobj_7->ParentIndex = 7.0;
  b_kstr = iobj_7->JointInternal.Type->size[0] * iobj_7->
    JointInternal.Type->size[1];
  iobj_7->JointInternal.Type->size[0] = 1;
  iobj_7->JointInternal.Type->size[1] = 5;
  left_emxEnsureCapacity_char_T_e(iobj_7->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_7->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl__emxInit_char_T_e(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_7->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = iobj_7->JointInternal.Type->size[0] * iobj_7->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_7->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_7->JointInternal.PositionNumber = 1.0;
    iobj_7->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_7->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_7->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_7->JointInternal.PositionNumber = 1.0;
    iobj_7->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_7->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_7->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_7->JointInternal.PositionNumber = 0.0;
    iobj_7->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_7->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_7->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_7->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_7->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  iobj_7->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_7->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_7->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[7] = iobj_7;
  b_kstr = iobj_8->NameInternal->size[0] * iobj_8->NameInternal->size[1];
  iobj_8->NameInternal->size[0] = 1;
  iobj_8->NameInternal->size[1] = 18;
  left_emxEnsureCapacity_char_T_e(iobj_8->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 18; b_kstr++) {
    iobj_8->NameInternal->data[b_kstr] = tmp_5[b_kstr];
  }

  iobj_8->ParentIndex = 8.0;
  b_kstr = iobj_8->JointInternal.Type->size[0] * iobj_8->
    JointInternal.Type->size[1];
  iobj_8->JointInternal.Type->size[0] = 1;
  iobj_8->JointInternal.Type->size[1] = 5;
  left_emxEnsureCapacity_char_T_e(iobj_8->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_8->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_8->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = iobj_8->JointInternal.Type->size[0] * iobj_8->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_8->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_8->JointInternal.PositionNumber = 1.0;
    iobj_8->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_8->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_8->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_8->JointInternal.PositionNumber = 1.0;
    iobj_8->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_8->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_8->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_8->JointInternal.PositionNumber = 0.0;
    iobj_8->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_8->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_8->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_8->JointInternal.JointToParentTransform[b_kstr] = tmp_6[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_8->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  iobj_8->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_8->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_8->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[8] = iobj_8;
  b_kstr = iobj_9->NameInternal->size[0] * iobj_9->NameInternal->size[1];
  iobj_9->NameInternal->size[0] = 1;
  iobj_9->NameInternal->size[1] = 19;
  left_emxEnsureCapacity_char_T_e(iobj_9->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 19; b_kstr++) {
    iobj_9->NameInternal->data[b_kstr] = tmp_7[b_kstr];
  }

  iobj_9->ParentIndex = 8.0;
  b_kstr = iobj_9->JointInternal.Type->size[0] * iobj_9->
    JointInternal.Type->size[1];
  iobj_9->JointInternal.Type->size[0] = 1;
  iobj_9->JointInternal.Type->size[1] = 5;
  left_emxEnsureCapacity_char_T_e(iobj_9->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_9->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_9->JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = iobj_9->JointInternal.Type->size[0] * iobj_9->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_9->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_9->JointInternal.PositionNumber = 1.0;
    iobj_9->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_9->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_9->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_9->JointInternal.PositionNumber = 1.0;
    iobj_9->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_9->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_9->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_9->JointInternal.PositionNumber = 0.0;
    iobj_9->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_9->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_9->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_9->JointInternal.JointToParentTransform[b_kstr] = tmp_8[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_9->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  iobj_9->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_9->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_9->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[9] = iobj_9;
  obj->NumBodies = 10.0;
  obj->PositionNumber = 7.0;
  obj_0 = &obj->Base;
  b_kstr = obj->Base.NameInternal->size[0] * obj->Base.NameInternal->size[1];
  obj->Base.NameInternal->size[0] = 1;
  obj->Base.NameInternal->size[1] = 19;
  left_emxEnsureCapacity_char_T_e(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 19; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  left_emxEnsureCapacity_char_T_e(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  left_emxEnsureCapacity_char_T_e(switch_expression, b_kstr);
  loop_ub = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj_0->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp_1[b_kstr];
  }

  b_bool = false;
  if (switch_expression->size[1] == 8) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        loop_ub = b_kstr - 1;
        if (switch_expression->data[loop_ub] != b[loop_ub]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_2[b_kstr];
    }

    b_bool = false;
    if (switch_expression->size[1] == 9) {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          loop_ub = b_kstr - 1;
          if (switch_expression->data[loop_ub] != b_0[loop_ub]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  left_arm_ctrl__emxFree_char_T_e(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->Base.JointInternal.PositionNumber = 1.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->Base.JointInternal.PositionNumber = 1.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->Base.JointInternal.PositionNumber = 0.0;
    obj->Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->Base.JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  return b_obj;
}

// Model step function
void left_arm_ctrl_obs_step(void)
{
  emxArray_real_T_left_arm_ctrl_T *b;
  robotics_slmanip_internal_b_e_T *obj;
  k_robotics_manip_internal_R_e_T *obj_0;
  emxArray_e_cell_wrap_left_arm_T *Ttree;
  emxArray_char_T_left_arm_ctrl_T *bname;
  j_robotics_manip_internal_R_e_T *obj_1;
  static const int8_T b_b[49] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 1 };

  static const int8_T f[98] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 1 };

  static const int8_T c_b[98] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0 };

  static const int8_T d_b[196] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const char_T tmp[19] = { 's', 'h', 'o', 'u', 'l', 'd', 'e', 'r', 's',
    '_', 'l', 'e', 'f', 't', '_', 'l', 'i', 'n', 'k' };

  static const char_T tmp_0[14] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'l', 'i', 'n', 'k', '6' };

  int32_T exitg1;
  boolean_T exitg2;
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&left_arm_ctrl_obs_M->solverInfo,
                          ((left_arm_ctrl_obs_M->Timing.clockTick0+1)*
      left_arm_ctrl_obs_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(left_arm_ctrl_obs_M)) {
    left_arm_ctrl_obs_M->Timing.t[0] = rtsiGetT(&left_arm_ctrl_obs_M->solverInfo);
  }

  // Integrator: '<S1>/Integrator'
  memcpy(&left_arm_ctrl_obs_B.Integrator[0],
         &left_arm_ctrl_obs_X.Integrator_CSTATE[0], 14U * sizeof(real_T));
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Atomic SubSystem: '<S10>/Subscribe'
    // MATLABSystem: '<S29>/SourceBlock' incorporates:
    //   Inport: '<S31>/In1'

    left_arm_ctrl_o_SystemCore_step(&left_arm_ctrl_obs_B.b_varargout_1,
      left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Curr,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Rece,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_DataOffset,
      left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_Inf,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_k);

    // Outputs for Enabled SubSystem: '<S29>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S31>/Enable'

    if (left_arm_ctrl_obs_B.b_varargout_1) {
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
           left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.In1.Data[left_arm_ctrl_obs_B.i] =
          left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[left_arm_ctrl_obs_B.i];
      }

      left_arm_ctrl_obs_B.In1.Data_SL_Info.CurrentLength =
        left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Curr;
      left_arm_ctrl_obs_B.In1.Data_SL_Info.ReceivedLength =
        left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Rece;
      left_arm_ctrl_obs_B.In1.Layout.DataOffset =
        left_arm_ctrl_obs_B.b_varargout_2_Layout_DataOffset;
      memcpy(&left_arm_ctrl_obs_B.In1.Layout.Dim[0],
             &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension) << 4U);
      left_arm_ctrl_obs_B.In1.Layout.Dim_SL_Info.CurrentLength =
        left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_Inf;
      left_arm_ctrl_obs_B.In1.Layout.Dim_SL_Info.ReceivedLength =
        left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_k;
    }

    // End of MATLABSystem: '<S29>/SourceBlock'
    // End of Outputs for SubSystem: '<S29>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<S10>/Subscribe'

    // Outputs for Atomic SubSystem: '<S10>/Subscribe1'
    // MATLABSystem: '<S30>/SourceBlock' incorporates:
    //   Inport: '<S32>/In1'

    left_arm_ctrl_SystemCore_step_e(&left_arm_ctrl_obs_B.b_varargout_1,
      left_arm_ctrl_obs_B.b_varargout_2_Data,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Curr,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Rece,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_DataOffset,
      left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_Inf,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_k);

    // Outputs for Enabled SubSystem: '<S30>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S32>/Enable'

    if (left_arm_ctrl_obs_B.b_varargout_1) {
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 14;
           left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.In1_e.Data[left_arm_ctrl_obs_B.i] =
          left_arm_ctrl_obs_B.b_varargout_2_Data[left_arm_ctrl_obs_B.i];
      }

      left_arm_ctrl_obs_B.In1_e.Data_SL_Info.CurrentLength =
        left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Curr;
      left_arm_ctrl_obs_B.In1_e.Data_SL_Info.ReceivedLength =
        left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Rece;
      left_arm_ctrl_obs_B.In1_e.Layout.DataOffset =
        left_arm_ctrl_obs_B.b_varargout_2_Layout_DataOffset;
      memcpy(&left_arm_ctrl_obs_B.In1_e.Layout.Dim[0],
             &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension) << 4U);
      left_arm_ctrl_obs_B.In1_e.Layout.Dim_SL_Info.CurrentLength =
        left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_Inf;
      left_arm_ctrl_obs_B.In1_e.Layout.Dim_SL_Info.ReceivedLength =
        left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_k;
    }

    // End of MATLABSystem: '<S30>/SourceBlock'
    // End of Outputs for SubSystem: '<S30>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<S10>/Subscribe1'
  }

  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[3] == 0) {
    // MATLABSystem: '<S6>/Get Parameter'
    ParamGet_left_arm_ctrl_obs_368.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter_o1);

    // MATLABSystem: '<S6>/Get Parameter1'
    ParamGet_left_arm_ctrl_obs_370.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter1_o1);

    // MATLABSystem: '<S6>/Get Parameter2'
    ParamGet_left_arm_ctrl_obs_372.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter2_o1);

    // MATLABSystem: '<S6>/Get Parameter3'
    ParamGet_left_arm_ctrl_obs_374.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter3_o1);

    // MATLABSystem: '<S6>/Get Parameter4'
    ParamGet_left_arm_ctrl_obs_376.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter4_o1);

    // MATLABSystem: '<S6>/Get Parameter5'
    ParamGet_left_arm_ctrl_obs_378.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter5_o1);

    // MATLABSystem: '<S6>/Get Parameter6'
    ParamGet_left_arm_ctrl_obs_380.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter6_o1);

    // MATLABSystem: '<S6>/Get Parameter7'
    ParamGet_left_arm_ctrl_obs_400.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter7_o1_n);

    // MATLABSystem: '<S6>/Get Parameter8'
    ParamGet_left_arm_ctrl_obs_401.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter8_o1_k);

    // MATLABSystem: '<S6>/Get Parameter9'
    ParamGet_left_arm_ctrl_obs_402.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter9_o1_o);

    // MATLABSystem: '<S6>/Get Parameter10'
    ParamGet_left_arm_ctrl_obs_403.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter10_o1);

    // MATLABSystem: '<S6>/Get Parameter11'
    ParamGet_left_arm_ctrl_obs_404.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter11_o1);

    // MATLABSystem: '<S6>/Get Parameter12'
    ParamGet_left_arm_ctrl_obs_405.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter12_o1);

    // MATLABSystem: '<S6>/Get Parameter13'
    ParamGet_left_arm_ctrl_obs_406.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter13_o1);
  }

  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) {
    // MATLABSystem: '<S17>/MATLAB System'
    lef_GravityTorqueBlock_stepImpl(&left_arm_ctrl_obs_DW.obj,
      &left_arm_ctrl_obs_B.In1_e.Data[0], left_arm_ctrl_obs_B.MATLABSystem_n);
  }

  // SignalConversion generated from: '<S18>/ SFunction ' incorporates:
  //   MATLAB Function: '<S4>/MATLAB Function1'

  left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[0] =
    left_arm_ctrl_obs_B.GetParameter_o1;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[1] =
    left_arm_ctrl_obs_B.GetParameter1_o1;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[2] =
    left_arm_ctrl_obs_B.GetParameter2_o1;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[3] =
    left_arm_ctrl_obs_B.GetParameter3_o1;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[4] =
    left_arm_ctrl_obs_B.GetParameter4_o1;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[5] =
    left_arm_ctrl_obs_B.GetParameter5_o1;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[6] =
    left_arm_ctrl_obs_B.GetParameter6_o1;

  // SignalConversion generated from: '<S18>/ SFunction ' incorporates:
  //   MATLAB Function: '<S4>/MATLAB Function1'

  left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[0] =
    left_arm_ctrl_obs_B.GetParameter7_o1_n;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[1] =
    left_arm_ctrl_obs_B.GetParameter8_o1_k;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[2] =
    left_arm_ctrl_obs_B.GetParameter9_o1_o;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[3] =
    left_arm_ctrl_obs_B.GetParameter10_o1;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[4] =
    left_arm_ctrl_obs_B.GetParameter11_o1;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[5] =
    left_arm_ctrl_obs_B.GetParameter12_o1;
  left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[6] =
    left_arm_ctrl_obs_B.GetParameter13_o1;

  // MATLAB Function: '<S4>/MATLAB Function1'
  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
       left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.i] = (static_cast<real32_T>
      ((left_arm_ctrl_obs_B.In1_e.Data[left_arm_ctrl_obs_B.i] -
        static_cast<real32_T>
        (left_arm_ctrl_obs_B.Integrator[left_arm_ctrl_obs_B.i])) *
       left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[left_arm_ctrl_obs_B.i]) +
      left_arm_ctrl_obs_B.MATLABSystem_n[left_arm_ctrl_obs_B.i]) +
      static_cast<real32_T>
      ((left_arm_ctrl_obs_B.In1_e.Data[left_arm_ctrl_obs_B.i + 7] -
        static_cast<real32_T>
        (left_arm_ctrl_obs_B.Integrator[left_arm_ctrl_obs_B.i + 7])) *
       left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[left_arm_ctrl_obs_B.i]);
    if (left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.i] > 2.5) {
      left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.i] = 2.5;
    } else {
      if (left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.i] < -2.5) {
        left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.i] = -2.5;
      }
    }
  }

  // Constant: '<S1>/Constant2'
  left_arm_ctrl_obs_MATLABSystem(&left_arm_ctrl_obs_B.Integrator[0],
    &left_arm_ctrl_obs_B.Integrator[7], left_arm_ctrl_obs_B.torque,
    left_arm_ctrl_obs_P.Constant2_Value, &left_arm_ctrl_obs_B.MATLABSystem,
    &left_arm_ctrl_obs_DW.MATLABSystem);
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[3] == 0) {
    // MATLABSystem: '<S8>/Get Parameter7'
    ParamGet_left_arm_ctrl_obs_608.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter7_o1);

    // MATLABSystem: '<S8>/Get Parameter8'
    ParamGet_left_arm_ctrl_obs_609.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter8_o1);
  }

  // MATLAB Function: '<S1>/EKF' incorporates:
  //   Constant: '<S1>/delta_t'
  //   Integrator: '<S1>/Integrator1'

  memset(&left_arm_ctrl_obs_B.xp[0], 0, 14U * sizeof(real_T));
  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 49;
       left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.R[left_arm_ctrl_obs_B.i] =
      left_arm_ctrl_obs_B.GetParameter8_o1 * static_cast<real_T>
      (b_b[left_arm_ctrl_obs_B.i]);
    left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.i] = static_cast<real_T>
      (b_b[left_arm_ctrl_obs_B.i]) * left_arm_ctrl_obs_P.delta_t_Value;
  }

  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 14;
       left_arm_ctrl_obs_B.i++) {
    for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 7;
         left_arm_ctrl_obs_B.b_ix++) {
      left_arm_ctrl_obs_B.F[left_arm_ctrl_obs_B.b_ix + 14 *
        left_arm_ctrl_obs_B.i] = f[7 * left_arm_ctrl_obs_B.i +
        left_arm_ctrl_obs_B.b_ix];
    }
  }

  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
       left_arm_ctrl_obs_B.i++) {
    for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 7;
         left_arm_ctrl_obs_B.b_ix++) {
      left_arm_ctrl_obs_B.smax = left_arm_ctrl_obs_B.R_tmp[7 *
        left_arm_ctrl_obs_B.i + left_arm_ctrl_obs_B.b_ix];
      left_arm_ctrl_obs_B.F[(left_arm_ctrl_obs_B.b_ix + 14 *
        left_arm_ctrl_obs_B.i) + 7] = left_arm_ctrl_obs_B.smax;
      left_arm_ctrl_obs_B.F[(left_arm_ctrl_obs_B.b_ix + 14 *
        (left_arm_ctrl_obs_B.i + 7)) + 7] = left_arm_ctrl_obs_B.smax;
    }
  }

  memcpy(&left_arm_ctrl_obs_B.R_tmp[0], &left_arm_ctrl_obs_B.R[0], 49U * sizeof
         (real_T));
  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
       left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.ipiv[left_arm_ctrl_obs_B.i] = static_cast<int8_T>
      (left_arm_ctrl_obs_B.i + 1);
  }

  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 6;
       left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.i << 3;
    left_arm_ctrl_obs_B.b_ix = 0;
    left_arm_ctrl_obs_B.ix = left_arm_ctrl_obs_B.c;
    left_arm_ctrl_obs_B.smax = fabs
      (left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.c]);
    left_arm_ctrl_obs_B.jAcol = 2;
    while (left_arm_ctrl_obs_B.jAcol <= 7 - left_arm_ctrl_obs_B.i) {
      left_arm_ctrl_obs_B.ix++;
      left_arm_ctrl_obs_B.bid1 = fabs
        (left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.ix]);
      if (left_arm_ctrl_obs_B.bid1 > left_arm_ctrl_obs_B.smax) {
        left_arm_ctrl_obs_B.b_ix = left_arm_ctrl_obs_B.jAcol - 1;
        left_arm_ctrl_obs_B.smax = left_arm_ctrl_obs_B.bid1;
      }

      left_arm_ctrl_obs_B.jAcol++;
    }

    if (left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.c +
        left_arm_ctrl_obs_B.b_ix] != 0.0) {
      if (left_arm_ctrl_obs_B.b_ix != 0) {
        left_arm_ctrl_obs_B.ix = left_arm_ctrl_obs_B.i +
          left_arm_ctrl_obs_B.b_ix;
        left_arm_ctrl_obs_B.ipiv[left_arm_ctrl_obs_B.i] = static_cast<int8_T>
          (left_arm_ctrl_obs_B.ix + 1);
        left_arm_ctrl_obs_B.b_ix = left_arm_ctrl_obs_B.i;
        for (left_arm_ctrl_obs_B.jAcol = 0; left_arm_ctrl_obs_B.jAcol < 7;
             left_arm_ctrl_obs_B.jAcol++) {
          left_arm_ctrl_obs_B.smax =
            left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.b_ix];
          left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.b_ix] =
            left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.ix];
          left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.ix] =
            left_arm_ctrl_obs_B.smax;
          left_arm_ctrl_obs_B.b_ix += 7;
          left_arm_ctrl_obs_B.ix += 7;
        }
      }

      left_arm_ctrl_obs_B.b_ix = (left_arm_ctrl_obs_B.c - left_arm_ctrl_obs_B.i)
        + 7;
      left_arm_ctrl_obs_B.ix = left_arm_ctrl_obs_B.c + 1;
      while (left_arm_ctrl_obs_B.ix + 1 <= left_arm_ctrl_obs_B.b_ix) {
        left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.ix] /=
          left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.c];
        left_arm_ctrl_obs_B.ix++;
      }
    }

    left_arm_ctrl_obs_B.b_ix = left_arm_ctrl_obs_B.c;
    left_arm_ctrl_obs_B.ix = left_arm_ctrl_obs_B.c + 7;
    left_arm_ctrl_obs_B.jAcol = 0;
    while (left_arm_ctrl_obs_B.jAcol <= 5 - left_arm_ctrl_obs_B.i) {
      if (left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.ix] != 0.0) {
        left_arm_ctrl_obs_B.smax =
          -left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.ix];
        left_arm_ctrl_obs_B.c_ix = left_arm_ctrl_obs_B.c + 1;
        left_arm_ctrl_obs_B.kBcol = (left_arm_ctrl_obs_B.b_ix -
          left_arm_ctrl_obs_B.i) + 14;
        left_arm_ctrl_obs_B.ijA = left_arm_ctrl_obs_B.b_ix + 8;
        while (left_arm_ctrl_obs_B.ijA + 1 <= left_arm_ctrl_obs_B.kBcol) {
          left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.ijA] +=
            left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.c_ix] *
            left_arm_ctrl_obs_B.smax;
          left_arm_ctrl_obs_B.c_ix++;
          left_arm_ctrl_obs_B.ijA++;
        }
      }

      left_arm_ctrl_obs_B.ix += 7;
      left_arm_ctrl_obs_B.b_ix += 7;
      left_arm_ctrl_obs_B.jAcol++;
    }
  }

  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
       left_arm_ctrl_obs_B.i++) {
    for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 14;
         left_arm_ctrl_obs_B.b_ix++) {
      left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.b_ix + 14 *
        left_arm_ctrl_obs_B.i;
      left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.c] = 0.0;
      for (left_arm_ctrl_obs_B.ix = 0; left_arm_ctrl_obs_B.ix < 14;
           left_arm_ctrl_obs_B.ix++) {
        left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.c] +=
          left_arm_ctrl_obs_X.Integrator1_CSTATE[14 * left_arm_ctrl_obs_B.ix +
          left_arm_ctrl_obs_B.b_ix] * static_cast<real_T>(c_b[14 *
          left_arm_ctrl_obs_B.i + left_arm_ctrl_obs_B.ix]);
      }
    }
  }

  for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 7;
       left_arm_ctrl_obs_B.b_ix++) {
    left_arm_ctrl_obs_B.ix = 14 * left_arm_ctrl_obs_B.b_ix;
    left_arm_ctrl_obs_B.jAcol = 7 * left_arm_ctrl_obs_B.b_ix;
    left_arm_ctrl_obs_B.c_ix = 0;
    while (left_arm_ctrl_obs_B.c_ix <= left_arm_ctrl_obs_B.b_ix - 1) {
      left_arm_ctrl_obs_B.kBcol = 14 * left_arm_ctrl_obs_B.c_ix;
      left_arm_ctrl_obs_B.i = left_arm_ctrl_obs_B.c_ix +
        left_arm_ctrl_obs_B.jAcol;
      if (left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.i] != 0.0) {
        for (left_arm_ctrl_obs_B.ijA = 0; left_arm_ctrl_obs_B.ijA < 14;
             left_arm_ctrl_obs_B.ijA++) {
          left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.ijA +
            left_arm_ctrl_obs_B.ix;
          left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.c] -=
            left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.i] *
            left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.ijA +
            left_arm_ctrl_obs_B.kBcol];
        }
      }

      left_arm_ctrl_obs_B.c_ix++;
    }

    left_arm_ctrl_obs_B.smax = 1.0 /
      left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.b_ix +
      left_arm_ctrl_obs_B.jAcol];
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 14;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.i + left_arm_ctrl_obs_B.ix;
      left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.c] *= left_arm_ctrl_obs_B.smax;
    }
  }

  for (left_arm_ctrl_obs_B.b_ix = 6; left_arm_ctrl_obs_B.b_ix >= 0;
       left_arm_ctrl_obs_B.b_ix--) {
    left_arm_ctrl_obs_B.ix = 14 * left_arm_ctrl_obs_B.b_ix;
    left_arm_ctrl_obs_B.jAcol = 7 * left_arm_ctrl_obs_B.b_ix - 1;
    left_arm_ctrl_obs_B.c_ix = left_arm_ctrl_obs_B.b_ix + 2;
    while (left_arm_ctrl_obs_B.c_ix < 8) {
      left_arm_ctrl_obs_B.kBcol = (left_arm_ctrl_obs_B.c_ix - 1) * 14;
      left_arm_ctrl_obs_B.i = left_arm_ctrl_obs_B.c_ix +
        left_arm_ctrl_obs_B.jAcol;
      if (left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.i] != 0.0) {
        for (left_arm_ctrl_obs_B.ijA = 0; left_arm_ctrl_obs_B.ijA < 14;
             left_arm_ctrl_obs_B.ijA++) {
          left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.ijA +
            left_arm_ctrl_obs_B.ix;
          left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.c] -=
            left_arm_ctrl_obs_B.R_tmp[left_arm_ctrl_obs_B.i] *
            left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.ijA +
            left_arm_ctrl_obs_B.kBcol];
        }
      }

      left_arm_ctrl_obs_B.c_ix++;
    }
  }

  for (left_arm_ctrl_obs_B.i = 5; left_arm_ctrl_obs_B.i >= 0;
       left_arm_ctrl_obs_B.i--) {
    if (left_arm_ctrl_obs_B.i + 1 !=
        left_arm_ctrl_obs_B.ipiv[left_arm_ctrl_obs_B.i]) {
      left_arm_ctrl_obs_B.b_ix = left_arm_ctrl_obs_B.ipiv[left_arm_ctrl_obs_B.i]
        - 1;
      for (left_arm_ctrl_obs_B.ix = 0; left_arm_ctrl_obs_B.ix < 14;
           left_arm_ctrl_obs_B.ix++) {
        left_arm_ctrl_obs_B.jAcol = 14 * left_arm_ctrl_obs_B.i +
          left_arm_ctrl_obs_B.ix;
        left_arm_ctrl_obs_B.smax =
          left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.jAcol];
        left_arm_ctrl_obs_B.c = 14 * left_arm_ctrl_obs_B.b_ix +
          left_arm_ctrl_obs_B.ix;
        left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.jAcol] =
          left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.c];
        left_arm_ctrl_obs_B.K[left_arm_ctrl_obs_B.c] = left_arm_ctrl_obs_B.smax;
      }
    }
  }

  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
       left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.xp[left_arm_ctrl_obs_B.i] =
      left_arm_ctrl_obs_B.Integrator[left_arm_ctrl_obs_B.i + 7];
    left_arm_ctrl_obs_B.xp[left_arm_ctrl_obs_B.i + 7] =
      left_arm_ctrl_obs_B.MATLABSystem.MATLABSystem[left_arm_ctrl_obs_B.i];
    left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[left_arm_ctrl_obs_B.i] =
      left_arm_ctrl_obs_B.In1.Data[left_arm_ctrl_obs_B.i] -
      left_arm_ctrl_obs_B.Integrator[left_arm_ctrl_obs_B.i];
  }

  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 14;
       left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.smax = 0.0;
    for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 7;
         left_arm_ctrl_obs_B.b_ix++) {
      left_arm_ctrl_obs_B.smax += left_arm_ctrl_obs_B.K[14 *
        left_arm_ctrl_obs_B.b_ix + left_arm_ctrl_obs_B.i] *
        left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[left_arm_ctrl_obs_B.b_ix];
    }

    left_arm_ctrl_obs_B.xp[left_arm_ctrl_obs_B.i] += left_arm_ctrl_obs_B.smax;
    for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 14;
         left_arm_ctrl_obs_B.b_ix++) {
      left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.i + 14 *
        left_arm_ctrl_obs_B.b_ix;
      left_arm_ctrl_obs_B.F_c[left_arm_ctrl_obs_B.c] = 0.0;
      left_arm_ctrl_obs_B.dv[left_arm_ctrl_obs_B.c] = 0.0;
      for (left_arm_ctrl_obs_B.ix = 0; left_arm_ctrl_obs_B.ix < 14;
           left_arm_ctrl_obs_B.ix++) {
        left_arm_ctrl_obs_B.jAcol = 14 * left_arm_ctrl_obs_B.ix +
          left_arm_ctrl_obs_B.i;
        left_arm_ctrl_obs_B.F_c[left_arm_ctrl_obs_B.c] +=
          left_arm_ctrl_obs_B.F[left_arm_ctrl_obs_B.jAcol] *
          left_arm_ctrl_obs_X.Integrator1_CSTATE[14 * left_arm_ctrl_obs_B.b_ix +
          left_arm_ctrl_obs_B.ix];
        left_arm_ctrl_obs_B.dv[left_arm_ctrl_obs_B.c] +=
          left_arm_ctrl_obs_X.Integrator1_CSTATE[left_arm_ctrl_obs_B.jAcol] *
          left_arm_ctrl_obs_B.F[14 * left_arm_ctrl_obs_B.ix +
          left_arm_ctrl_obs_B.b_ix];
      }
    }

    for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 7;
         left_arm_ctrl_obs_B.b_ix++) {
      left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.i + 14 *
        left_arm_ctrl_obs_B.b_ix;
      left_arm_ctrl_obs_B.K_k[left_arm_ctrl_obs_B.c] = 0.0;
      for (left_arm_ctrl_obs_B.ix = 0; left_arm_ctrl_obs_B.ix < 7;
           left_arm_ctrl_obs_B.ix++) {
        left_arm_ctrl_obs_B.K_k[left_arm_ctrl_obs_B.c] += left_arm_ctrl_obs_B.K
          [14 * left_arm_ctrl_obs_B.ix + left_arm_ctrl_obs_B.i] *
          left_arm_ctrl_obs_B.R[7 * left_arm_ctrl_obs_B.b_ix +
          left_arm_ctrl_obs_B.ix];
      }
    }

    for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 14;
         left_arm_ctrl_obs_B.b_ix++) {
      left_arm_ctrl_obs_B.smax = 0.0;
      for (left_arm_ctrl_obs_B.ix = 0; left_arm_ctrl_obs_B.ix < 7;
           left_arm_ctrl_obs_B.ix++) {
        left_arm_ctrl_obs_B.smax += left_arm_ctrl_obs_B.K_k[14 *
          left_arm_ctrl_obs_B.ix + left_arm_ctrl_obs_B.i] *
          left_arm_ctrl_obs_B.K[14 * left_arm_ctrl_obs_B.ix +
          left_arm_ctrl_obs_B.b_ix];
      }

      left_arm_ctrl_obs_B.c = 14 * left_arm_ctrl_obs_B.b_ix +
        left_arm_ctrl_obs_B.i;
      left_arm_ctrl_obs_B.Pp[left_arm_ctrl_obs_B.c] =
        ((left_arm_ctrl_obs_B.F_c[left_arm_ctrl_obs_B.c] +
          left_arm_ctrl_obs_B.dv[left_arm_ctrl_obs_B.c]) -
         left_arm_ctrl_obs_B.smax) + static_cast<real_T>
        (d_b[left_arm_ctrl_obs_B.c]) * left_arm_ctrl_obs_B.GetParameter7_o1;
    }
  }

  // End of MATLAB Function: '<S1>/EKF'
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) {
    left_arm_ctrl__emxInit_real_T_e(&b, 2);

    // MATLABSystem: '<S15>/MATLAB System'
    RigidBodyTreeDynamics_massMat_e(&left_arm_ctrl_obs_DW.obj_h.TreeInternal,
      left_arm_ctrl_obs_B.In1.Data, b);
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 49;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.MATLABSystem_k[left_arm_ctrl_obs_B.i] = b->
        data[left_arm_ctrl_obs_B.i];
    }

    // End of MATLABSystem: '<S15>/MATLAB System'
    left_arm_ctrl__emxFree_real_T_e(&b);
    left_arm_ct_emxInit_e_cell_wrap(&Ttree, 2);
    left_arm_ctrl__emxInit_char_T_e(&bname, 2);

    // MATLABSystem: '<S14>/MATLAB System'
    obj = &left_arm_ctrl_obs_DW.obj_o;
    obj_0 = &left_arm_ctrl_obs_DW.obj_o.TreeInternal;
    RigidBodyTree_forwardKinematics(&obj->TreeInternal,
      left_arm_ctrl_obs_B.In1.Data, Ttree);
    left_arm_ctrl_obs_B.bid1 = -1.0;
    left_arm_ctrl_obs_B.i = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj_0->Base.NameInternal->size[1];
    left_emxEnsureCapacity_char_T_e(bname, left_arm_ctrl_obs_B.i);
    left_arm_ctrl_obs_B.c = obj_0->Base.NameInternal->size[0] *
      obj_0->Base.NameInternal->size[1] - 1;
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i <=
         left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.i++) {
      bname->data[left_arm_ctrl_obs_B.i] = obj_0->Base.NameInternal->
        data[left_arm_ctrl_obs_B.i];
    }

    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 19;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.b_i[left_arm_ctrl_obs_B.i] = tmp[left_arm_ctrl_obs_B.i];
    }

    left_arm_ctrl_obs_B.b_varargout_1 = false;
    if (bname->size[1] == 19) {
      left_arm_ctrl_obs_B.c = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.c - 1 < 19) {
          left_arm_ctrl_obs_B.b_ix = left_arm_ctrl_obs_B.c - 1;
          if (bname->data[left_arm_ctrl_obs_B.b_ix] !=
              left_arm_ctrl_obs_B.b_i[left_arm_ctrl_obs_B.b_ix]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.c++;
          }
        } else {
          left_arm_ctrl_obs_B.b_varargout_1 = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_varargout_1) {
      left_arm_ctrl_obs_B.bid1 = 0.0;
    } else {
      left_arm_ctrl_obs_B.smax = obj->TreeInternal.NumBodies;
      left_arm_ctrl_obs_B.i = static_cast<int32_T>(left_arm_ctrl_obs_B.smax) - 1;
      if (0 <= left_arm_ctrl_obs_B.i) {
        for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 19;
             left_arm_ctrl_obs_B.b_ix++) {
          left_arm_ctrl_obs_B.b_i[left_arm_ctrl_obs_B.b_ix] =
            tmp[left_arm_ctrl_obs_B.b_ix];
        }
      }

      left_arm_ctrl_obs_B.ix = 0;
      exitg2 = false;
      while ((!exitg2) && (left_arm_ctrl_obs_B.ix <= left_arm_ctrl_obs_B.i)) {
        obj_1 = obj_0->Bodies[left_arm_ctrl_obs_B.ix];
        left_arm_ctrl_obs_B.b_ix = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = obj_1->NameInternal->size[1];
        left_emxEnsureCapacity_char_T_e(bname, left_arm_ctrl_obs_B.b_ix);
        left_arm_ctrl_obs_B.c = obj_1->NameInternal->size[0] *
          obj_1->NameInternal->size[1] - 1;
        for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix <=
             left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.b_ix++) {
          bname->data[left_arm_ctrl_obs_B.b_ix] = obj_1->NameInternal->
            data[left_arm_ctrl_obs_B.b_ix];
        }

        left_arm_ctrl_obs_B.b_varargout_1 = false;
        if (bname->size[1] == 19) {
          left_arm_ctrl_obs_B.c = 1;
          do {
            exitg1 = 0;
            if (left_arm_ctrl_obs_B.c - 1 < 19) {
              left_arm_ctrl_obs_B.b_ix = left_arm_ctrl_obs_B.c - 1;
              if (bname->data[left_arm_ctrl_obs_B.b_ix] !=
                  left_arm_ctrl_obs_B.b_i[left_arm_ctrl_obs_B.b_ix]) {
                exitg1 = 1;
              } else {
                left_arm_ctrl_obs_B.c++;
              }
            } else {
              left_arm_ctrl_obs_B.b_varargout_1 = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (left_arm_ctrl_obs_B.b_varargout_1) {
          left_arm_ctrl_obs_B.bid1 = static_cast<real_T>(left_arm_ctrl_obs_B.ix)
            + 1.0;
          exitg2 = true;
        } else {
          left_arm_ctrl_obs_B.ix++;
        }
      }
    }

    if (left_arm_ctrl_obs_B.bid1 == 0.0) {
      memset(&left_arm_ctrl_obs_B.T1[0], 0, sizeof(real_T) << 4U);
      left_arm_ctrl_obs_B.T1[0] = 1.0;
      left_arm_ctrl_obs_B.T1[5] = 1.0;
      left_arm_ctrl_obs_B.T1[10] = 1.0;
      left_arm_ctrl_obs_B.T1[15] = 1.0;
    } else {
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 16;
           left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.T1[left_arm_ctrl_obs_B.i] = Ttree->data
          [static_cast<int32_T>(left_arm_ctrl_obs_B.bid1) - 1]
          .f1[left_arm_ctrl_obs_B.i];
      }
    }

    left_arm_ctrl_obs_B.bid1 = -1.0;
    left_arm_ctrl_obs_B.i = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj_0->Base.NameInternal->size[1];
    left_emxEnsureCapacity_char_T_e(bname, left_arm_ctrl_obs_B.i);
    left_arm_ctrl_obs_B.c = obj_0->Base.NameInternal->size[0] *
      obj_0->Base.NameInternal->size[1] - 1;
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i <=
         left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.i++) {
      bname->data[left_arm_ctrl_obs_B.i] = obj_0->Base.NameInternal->
        data[left_arm_ctrl_obs_B.i];
    }

    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 14;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.b_o[left_arm_ctrl_obs_B.i] =
        tmp_0[left_arm_ctrl_obs_B.i];
    }

    left_arm_ctrl_obs_B.b_varargout_1 = false;
    if (bname->size[1] == 14) {
      left_arm_ctrl_obs_B.c = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.c - 1 < 14) {
          left_arm_ctrl_obs_B.b_ix = left_arm_ctrl_obs_B.c - 1;
          if (bname->data[left_arm_ctrl_obs_B.b_ix] !=
              left_arm_ctrl_obs_B.b_o[left_arm_ctrl_obs_B.b_ix]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.c++;
          }
        } else {
          left_arm_ctrl_obs_B.b_varargout_1 = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_varargout_1) {
      left_arm_ctrl_obs_B.bid1 = 0.0;
    } else {
      left_arm_ctrl_obs_B.smax = obj->TreeInternal.NumBodies;
      left_arm_ctrl_obs_B.i = static_cast<int32_T>(left_arm_ctrl_obs_B.smax) - 1;
      if (0 <= left_arm_ctrl_obs_B.i) {
        for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 14;
             left_arm_ctrl_obs_B.b_ix++) {
          left_arm_ctrl_obs_B.b_o[left_arm_ctrl_obs_B.b_ix] =
            tmp_0[left_arm_ctrl_obs_B.b_ix];
        }
      }

      left_arm_ctrl_obs_B.ix = 0;
      exitg2 = false;
      while ((!exitg2) && (left_arm_ctrl_obs_B.ix <= left_arm_ctrl_obs_B.i)) {
        obj_1 = obj_0->Bodies[left_arm_ctrl_obs_B.ix];
        left_arm_ctrl_obs_B.b_ix = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = obj_1->NameInternal->size[1];
        left_emxEnsureCapacity_char_T_e(bname, left_arm_ctrl_obs_B.b_ix);
        left_arm_ctrl_obs_B.c = obj_1->NameInternal->size[0] *
          obj_1->NameInternal->size[1] - 1;
        for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix <=
             left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.b_ix++) {
          bname->data[left_arm_ctrl_obs_B.b_ix] = obj_1->NameInternal->
            data[left_arm_ctrl_obs_B.b_ix];
        }

        left_arm_ctrl_obs_B.b_varargout_1 = false;
        if (bname->size[1] == 14) {
          left_arm_ctrl_obs_B.c = 1;
          do {
            exitg1 = 0;
            if (left_arm_ctrl_obs_B.c - 1 < 14) {
              left_arm_ctrl_obs_B.b_ix = left_arm_ctrl_obs_B.c - 1;
              if (bname->data[left_arm_ctrl_obs_B.b_ix] !=
                  left_arm_ctrl_obs_B.b_o[left_arm_ctrl_obs_B.b_ix]) {
                exitg1 = 1;
              } else {
                left_arm_ctrl_obs_B.c++;
              }
            } else {
              left_arm_ctrl_obs_B.b_varargout_1 = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (left_arm_ctrl_obs_B.b_varargout_1) {
          left_arm_ctrl_obs_B.bid1 = static_cast<real_T>(left_arm_ctrl_obs_B.ix)
            + 1.0;
          exitg2 = true;
        } else {
          left_arm_ctrl_obs_B.ix++;
        }
      }
    }

    left_arm_ctrl__emxFree_char_T_e(&bname);

    // MATLABSystem: '<S14>/MATLAB System'
    if (left_arm_ctrl_obs_B.bid1 == 0.0) {
      memset(&left_arm_ctrl_obs_B.T2[0], 0, sizeof(real_T) << 4U);
      left_arm_ctrl_obs_B.T2[0] = 1.0;
      left_arm_ctrl_obs_B.T2[5] = 1.0;
      left_arm_ctrl_obs_B.T2[10] = 1.0;
      left_arm_ctrl_obs_B.T2[15] = 1.0;
    } else {
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 16;
           left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.i] = Ttree->data
          [static_cast<int32_T>(left_arm_ctrl_obs_B.bid1) - 1]
          .f1[left_arm_ctrl_obs_B.i];
      }
    }

    left_arm_ct_emxFree_e_cell_wrap(&Ttree);

    // MATLABSystem: '<S14>/MATLAB System'
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 3;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.R_j[3 * left_arm_ctrl_obs_B.i] =
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.i];
      left_arm_ctrl_obs_B.R_j[3 * left_arm_ctrl_obs_B.i + 1] =
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.i + 4];
      left_arm_ctrl_obs_B.R_j[3 * left_arm_ctrl_obs_B.i + 2] =
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.i + 8];
    }

    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 9;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.R_d[left_arm_ctrl_obs_B.i] =
        -left_arm_ctrl_obs_B.R_j[left_arm_ctrl_obs_B.i];
    }

    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 3;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.i << 2;
      left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.c] = left_arm_ctrl_obs_B.R_j[3
        * left_arm_ctrl_obs_B.i];
      left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.c + 1] =
        left_arm_ctrl_obs_B.R_j[3 * left_arm_ctrl_obs_B.i + 1];
      left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.c + 2] =
        left_arm_ctrl_obs_B.R_j[3 * left_arm_ctrl_obs_B.i + 2];
      left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.i + 12] =
        left_arm_ctrl_obs_B.R_d[left_arm_ctrl_obs_B.i + 6] *
        left_arm_ctrl_obs_B.T2[14] +
        (left_arm_ctrl_obs_B.R_d[left_arm_ctrl_obs_B.i + 3] *
         left_arm_ctrl_obs_B.T2[13] +
         left_arm_ctrl_obs_B.R_d[left_arm_ctrl_obs_B.i] *
         left_arm_ctrl_obs_B.T2[12]);
    }

    left_arm_ctrl_obs_B.R_c[3] = 0.0;
    left_arm_ctrl_obs_B.R_c[7] = 0.0;
    left_arm_ctrl_obs_B.R_c[11] = 0.0;
    left_arm_ctrl_obs_B.R_c[15] = 1.0;
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 4;
         left_arm_ctrl_obs_B.i++) {
      for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 4;
           left_arm_ctrl_obs_B.b_ix++) {
        left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.i << 2;
        left_arm_ctrl_obs_B.ix = left_arm_ctrl_obs_B.b_ix +
          left_arm_ctrl_obs_B.c;
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.ix] = 0.0;
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.ix] +=
          left_arm_ctrl_obs_B.T1[left_arm_ctrl_obs_B.c] *
          left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.b_ix];
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.ix] +=
          left_arm_ctrl_obs_B.T1[left_arm_ctrl_obs_B.c + 1] *
          left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.b_ix + 4];
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.ix] +=
          left_arm_ctrl_obs_B.T1[left_arm_ctrl_obs_B.c + 2] *
          left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.b_ix + 8];
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.ix] +=
          left_arm_ctrl_obs_B.T1[left_arm_ctrl_obs_B.c + 3] *
          left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.b_ix + 12];
      }
    }
  }

  // Integrator: '<S11>/Integrator'
  memcpy(&left_arm_ctrl_obs_B.Integrator_f[0],
         &left_arm_ctrl_obs_X.Integrator_CSTATE_p[0], 14U * sizeof(real_T));

  // Constant: '<S11>/Constant2'
  left_arm_ctrl_obs_MATLABSystem(left_arm_ctrl_obs_B.In1.Data,
    &left_arm_ctrl_obs_B.Integrator_f[7], left_arm_ctrl_obs_B.torque,
    left_arm_ctrl_obs_P.Constant2_Value_k, &left_arm_ctrl_obs_B.MATLABSystem_me,
    &left_arm_ctrl_obs_DW.MATLABSystem_me);
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[3] == 0) {
    // MATLABSystem: '<S7>/Get Parameter7'
    ParamGet_left_arm_ctrl_obs_383.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter7_o1_j);

    // MATLABSystem: '<S7>/Get Parameter8'
    ParamGet_left_arm_ctrl_obs_384.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter8_o1_m);

    // MATLABSystem: '<S7>/Get Parameter9'
    ParamGet_left_arm_ctrl_obs_385.get_parameter
      (&left_arm_ctrl_obs_B.GetParameter9_o1);
  }

  // MATLAB Function: '<S11>/Observer'
  memset(&left_arm_ctrl_obs_B.xp_est[0], 0, 14U * sizeof(real_T));
  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
       left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.smax = 2.0 / (exp
      ((left_arm_ctrl_obs_B.Integrator_f[left_arm_ctrl_obs_B.i] -
        left_arm_ctrl_obs_B.In1.Data[left_arm_ctrl_obs_B.i]) /
       left_arm_ctrl_obs_B.GetParameter9_o1) + 1.0) - 1.0;
    left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[left_arm_ctrl_obs_B.i] =
      left_arm_ctrl_obs_B.smax * (sqrt(fabs
      (left_arm_ctrl_obs_B.In1.Data[left_arm_ctrl_obs_B.i] -
       left_arm_ctrl_obs_B.Integrator_f[left_arm_ctrl_obs_B.i])) *
      left_arm_ctrl_obs_B.GetParameter7_o1_j);
    left_arm_ctrl_obs_B.z[left_arm_ctrl_obs_B.i] = left_arm_ctrl_obs_B.smax *
      left_arm_ctrl_obs_B.GetParameter8_o1_m;
  }

  for (left_arm_ctrl_obs_B.ix = 0; left_arm_ctrl_obs_B.ix < 7;
       left_arm_ctrl_obs_B.ix++) {
    left_arm_ctrl_obs_B.xp_est[left_arm_ctrl_obs_B.ix] =
      left_arm_ctrl_obs_B.Integrator_f[left_arm_ctrl_obs_B.ix + 7] +
      left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[left_arm_ctrl_obs_B.ix];
    left_arm_ctrl_obs_B.xp_est[left_arm_ctrl_obs_B.ix + 7] =
      left_arm_ctrl_obs_B.MATLABSystem_me.MATLABSystem[left_arm_ctrl_obs_B.ix] +
      left_arm_ctrl_obs_B.z[left_arm_ctrl_obs_B.ix];
  }

  // End of MATLAB Function: '<S11>/Observer'
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) {
    // MATLABSystem: '<S3>/Coordinate Transformation Conversion1'
    left_arm_ctrl_obs_B.smax = sqrt(left_arm_ctrl_obs_B.T2[0] *
      left_arm_ctrl_obs_B.T2[0] + left_arm_ctrl_obs_B.T2[1] *
      left_arm_ctrl_obs_B.T2[1]);
    left_arm_ctrl_obs_B.CoordinateTransformationConvers[0] = rt_atan2d_snf
      (left_arm_ctrl_obs_B.T2[6], left_arm_ctrl_obs_B.T2[10]);
    left_arm_ctrl_obs_B.CoordinateTransformationConvers[1] = rt_atan2d_snf
      (-left_arm_ctrl_obs_B.T2[2], left_arm_ctrl_obs_B.smax);
    left_arm_ctrl_obs_B.CoordinateTransformationConvers[2] = rt_atan2d_snf
      (left_arm_ctrl_obs_B.T2[1], left_arm_ctrl_obs_B.T2[0]);
    if (left_arm_ctrl_obs_B.smax < 2.2204460492503131E-15) {
      left_arm_ctrl_obs_B.c = 0;
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 1;
           left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.c++;
      }

      left_arm_ctrl_obs_B.rtb_MATLABSystem_size[0] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size[1] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size[2] = left_arm_ctrl_obs_B.c;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_n[0] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_n[1] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_n[2] = left_arm_ctrl_obs_B.c;
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i <
           left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.rtb_MATLABSystem_data = -left_arm_ctrl_obs_B.T2[9];
        left_arm_ctrl_obs_B.rtb_MATLABSystem_data_e = left_arm_ctrl_obs_B.T2[5];
        left_arm_ctrl_obs_B.rtb_MATLABSystem_data_a = -left_arm_ctrl_obs_B.T2[2];
      }

      left_arm_ctrl_obs_atan2(&left_arm_ctrl_obs_B.rtb_MATLABSystem_data,
        left_arm_ctrl_obs_B.rtb_MATLABSystem_size,
        &left_arm_ctrl_obs_B.rtb_MATLABSystem_data_e,
        left_arm_ctrl_obs_B.rtb_MATLABSystem_size_n,
        &left_arm_ctrl_obs_B.tmp_data, left_arm_ctrl_obs_B.tmp_size);
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_m[0] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_m[1] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_m[2] = left_arm_ctrl_obs_B.c;
      left_arm_ctrl_obs_B.sy_size[0] = 1;
      left_arm_ctrl_obs_B.sy_size[1] = 1;
      left_arm_ctrl_obs_B.sy_size[2] = left_arm_ctrl_obs_B.c;
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i <
           left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.rtb_MATLABSystem_data = left_arm_ctrl_obs_B.smax;
      }

      left_arm_ctrl_obs_atan2(&left_arm_ctrl_obs_B.rtb_MATLABSystem_data_a,
        left_arm_ctrl_obs_B.rtb_MATLABSystem_size_m,
        &left_arm_ctrl_obs_B.rtb_MATLABSystem_data, left_arm_ctrl_obs_B.sy_size,
        &left_arm_ctrl_obs_B.rtb_MATLABSystem_data_e,
        left_arm_ctrl_obs_B.rtb_MATLABSystem_size);
      left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.tmp_size[2];
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i <
           left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.CoordinateTransformationConvers[0] =
          left_arm_ctrl_obs_B.tmp_data;
      }

      left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.rtb_MATLABSystem_size[2];
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i <
           left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.CoordinateTransformationConvers[1] =
          left_arm_ctrl_obs_B.rtb_MATLABSystem_data_e;
      }

      left_arm_ctrl_obs_B.CoordinateTransformationConvers[2] = 0.0;
    }

    left_arm_ctrl_obs_B.smax =
      left_arm_ctrl_obs_B.CoordinateTransformationConvers[0];
    left_arm_ctrl_obs_B.CoordinateTransformationConvers[0] =
      left_arm_ctrl_obs_B.CoordinateTransformationConvers[2];
    left_arm_ctrl_obs_B.CoordinateTransformationConvers[2] =
      left_arm_ctrl_obs_B.smax;

    // End of MATLABSystem: '<S3>/Coordinate Transformation Conversion1'
  }

  // MATLAB Function: '<S3>/mass estimator'
  left_arm_ctrl_obs_B.bid1 = 0.0;
  left_arm_ctrl_obs_B.smax = 3.3121686421112381E-170;
  for (left_arm_ctrl_obs_B.c_ix = 0; left_arm_ctrl_obs_B.c_ix < 7;
       left_arm_ctrl_obs_B.c_ix++) {
    left_arm_ctrl_obs_B.absxk = fabs
      (left_arm_ctrl_obs_B.Integrator_f[left_arm_ctrl_obs_B.c_ix + 7]);
    if (left_arm_ctrl_obs_B.absxk > left_arm_ctrl_obs_B.smax) {
      left_arm_ctrl_obs_B.t = left_arm_ctrl_obs_B.smax /
        left_arm_ctrl_obs_B.absxk;
      left_arm_ctrl_obs_B.bid1 = left_arm_ctrl_obs_B.bid1 *
        left_arm_ctrl_obs_B.t * left_arm_ctrl_obs_B.t + 1.0;
      left_arm_ctrl_obs_B.smax = left_arm_ctrl_obs_B.absxk;
    } else {
      left_arm_ctrl_obs_B.t = left_arm_ctrl_obs_B.absxk /
        left_arm_ctrl_obs_B.smax;
      left_arm_ctrl_obs_B.bid1 += left_arm_ctrl_obs_B.t * left_arm_ctrl_obs_B.t;
    }
  }

  left_arm_ctrl_obs_B.bid1 = left_arm_ctrl_obs_B.smax * sqrt
    (left_arm_ctrl_obs_B.bid1);
  left_arm_ctrl_obs_B.smax = sin
    (left_arm_ctrl_obs_B.CoordinateTransformationConvers[1]);
  if (fabs(left_arm_ctrl_obs_B.smax) > 0.2) {
    if (left_arm_ctrl_obs_B.bid1 < 0.15) {
      // TransferFcn: '<S2>/Low Pass (z1)' incorporates:
      //   TransferFcn: '<S2>/Low Pass (z1)1'
      //   TransferFcn: '<S2>/Low Pass (z1)2'
      //   TransferFcn: '<S2>/Low Pass (z1)3'
      //   TransferFcn: '<S2>/Low Pass (z1)4'
      //   TransferFcn: '<S2>/Low Pass (z1)5'
      //   TransferFcn: '<S2>/Low Pass (z1)6'

      left_arm_ctrl_obs_B.bid1 = left_arm_ctrl_obs_P.lowpass_B[0] /
        left_arm_ctrl_obs_P.lowpass_A[0];
      left_arm_ctrl_obs_B.absxk = left_arm_ctrl_obs_P.lowpass_B[1] /
        left_arm_ctrl_obs_P.lowpass_A[0] - left_arm_ctrl_obs_B.bid1 *
        (left_arm_ctrl_obs_P.lowpass_A[1] / left_arm_ctrl_obs_P.lowpass_A[0]);
      left_arm_ctrl_obs_B.t = left_arm_ctrl_obs_P.lowpass_B[0] /
        left_arm_ctrl_obs_P.lowpass_A[0];
      left_arm_ctrl_obs_B.rtb_LowPassz1_tmp = left_arm_ctrl_obs_P.lowpass_B[2] /
        left_arm_ctrl_obs_P.lowpass_A[0] - left_arm_ctrl_obs_B.bid1 *
        (left_arm_ctrl_obs_P.lowpass_A[2] / left_arm_ctrl_obs_P.lowpass_A[0]);
      left_arm_ctrl_obs_B.rtb_LowPassz1_tmp_a = left_arm_ctrl_obs_P.lowpass_B[3]
        / left_arm_ctrl_obs_P.lowpass_A[0] - left_arm_ctrl_obs_B.bid1 *
        (left_arm_ctrl_obs_P.lowpass_A[3] / left_arm_ctrl_obs_P.lowpass_A[0]);
      left_arm_ctrl_obs_B.bid1 = left_arm_ctrl_obs_P.lowpass_B[4] /
        left_arm_ctrl_obs_P.lowpass_A[0] - left_arm_ctrl_obs_B.bid1 *
        (left_arm_ctrl_obs_P.lowpass_A[4] / left_arm_ctrl_obs_P.lowpass_A[0]);

      // SignalConversion generated from: '<S16>/ SFunction ' incorporates:
      //   TransferFcn: '<S2>/Low Pass (z1)'
      //   TransferFcn: '<S2>/Low Pass (z1)1'
      //   TransferFcn: '<S2>/Low Pass (z1)2'
      //   TransferFcn: '<S2>/Low Pass (z1)3'
      //   TransferFcn: '<S2>/Low Pass (z1)4'
      //   TransferFcn: '<S2>/Low Pass (z1)5'
      //   TransferFcn: '<S2>/Low Pass (z1)6'

      left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[0] =
        (((left_arm_ctrl_obs_B.absxk * left_arm_ctrl_obs_X.LowPassz1_CSTATE[0] +
           left_arm_ctrl_obs_B.t * left_arm_ctrl_obs_B.z[0]) +
          left_arm_ctrl_obs_B.rtb_LowPassz1_tmp *
          left_arm_ctrl_obs_X.LowPassz1_CSTATE[1]) +
         left_arm_ctrl_obs_B.rtb_LowPassz1_tmp_a *
         left_arm_ctrl_obs_X.LowPassz1_CSTATE[2]) + left_arm_ctrl_obs_B.bid1 *
        left_arm_ctrl_obs_X.LowPassz1_CSTATE[3];
      left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[1] =
        (((left_arm_ctrl_obs_B.absxk * left_arm_ctrl_obs_X.LowPassz11_CSTATE[0]
           + left_arm_ctrl_obs_B.t * left_arm_ctrl_obs_B.z[1]) +
          left_arm_ctrl_obs_B.rtb_LowPassz1_tmp *
          left_arm_ctrl_obs_X.LowPassz11_CSTATE[1]) +
         left_arm_ctrl_obs_B.rtb_LowPassz1_tmp_a *
         left_arm_ctrl_obs_X.LowPassz11_CSTATE[2]) + left_arm_ctrl_obs_B.bid1 *
        left_arm_ctrl_obs_X.LowPassz11_CSTATE[3];
      left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[2] =
        (((left_arm_ctrl_obs_B.absxk * left_arm_ctrl_obs_X.LowPassz12_CSTATE[0]
           + left_arm_ctrl_obs_B.t * left_arm_ctrl_obs_B.z[2]) +
          left_arm_ctrl_obs_B.rtb_LowPassz1_tmp *
          left_arm_ctrl_obs_X.LowPassz12_CSTATE[1]) +
         left_arm_ctrl_obs_B.rtb_LowPassz1_tmp_a *
         left_arm_ctrl_obs_X.LowPassz12_CSTATE[2]) + left_arm_ctrl_obs_B.bid1 *
        left_arm_ctrl_obs_X.LowPassz12_CSTATE[3];
      left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[3] =
        (((left_arm_ctrl_obs_B.absxk * left_arm_ctrl_obs_X.LowPassz13_CSTATE[0]
           + left_arm_ctrl_obs_B.t * left_arm_ctrl_obs_B.z[3]) +
          left_arm_ctrl_obs_B.rtb_LowPassz1_tmp *
          left_arm_ctrl_obs_X.LowPassz13_CSTATE[1]) +
         left_arm_ctrl_obs_B.rtb_LowPassz1_tmp_a *
         left_arm_ctrl_obs_X.LowPassz13_CSTATE[2]) + left_arm_ctrl_obs_B.bid1 *
        left_arm_ctrl_obs_X.LowPassz13_CSTATE[3];
      left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[4] =
        (((left_arm_ctrl_obs_B.absxk * left_arm_ctrl_obs_X.LowPassz14_CSTATE[0]
           + left_arm_ctrl_obs_B.t * left_arm_ctrl_obs_B.z[4]) +
          left_arm_ctrl_obs_B.rtb_LowPassz1_tmp *
          left_arm_ctrl_obs_X.LowPassz14_CSTATE[1]) +
         left_arm_ctrl_obs_B.rtb_LowPassz1_tmp_a *
         left_arm_ctrl_obs_X.LowPassz14_CSTATE[2]) + left_arm_ctrl_obs_B.bid1 *
        left_arm_ctrl_obs_X.LowPassz14_CSTATE[3];
      left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[5] =
        (((left_arm_ctrl_obs_B.absxk * left_arm_ctrl_obs_X.LowPassz15_CSTATE[0]
           + left_arm_ctrl_obs_B.t * left_arm_ctrl_obs_B.z[5]) +
          left_arm_ctrl_obs_B.rtb_LowPassz1_tmp *
          left_arm_ctrl_obs_X.LowPassz15_CSTATE[1]) +
         left_arm_ctrl_obs_B.rtb_LowPassz1_tmp_a *
         left_arm_ctrl_obs_X.LowPassz15_CSTATE[2]) + left_arm_ctrl_obs_B.bid1 *
        left_arm_ctrl_obs_X.LowPassz15_CSTATE[3];
      left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[6] =
        (((left_arm_ctrl_obs_B.absxk * left_arm_ctrl_obs_X.LowPassz16_CSTATE[0]
           + left_arm_ctrl_obs_B.t * left_arm_ctrl_obs_B.z[6]) +
          left_arm_ctrl_obs_B.rtb_LowPassz1_tmp *
          left_arm_ctrl_obs_X.LowPassz16_CSTATE[1]) +
         left_arm_ctrl_obs_B.rtb_LowPassz1_tmp_a *
         left_arm_ctrl_obs_X.LowPassz16_CSTATE[2]) + left_arm_ctrl_obs_B.bid1 *
        left_arm_ctrl_obs_X.LowPassz16_CSTATE[3];
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
           left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[left_arm_ctrl_obs_B.i] =
          0.0;
        for (left_arm_ctrl_obs_B.b_ix = 0; left_arm_ctrl_obs_B.b_ix < 7;
             left_arm_ctrl_obs_B.b_ix++) {
          left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[left_arm_ctrl_obs_B.i]
            += left_arm_ctrl_obs_B.MATLABSystem_k[7 * left_arm_ctrl_obs_B.b_ix +
            left_arm_ctrl_obs_B.i] *
            left_arm_ctrl_obs_B.TmpSignalConversionAtSFunct[left_arm_ctrl_obs_B.b_ix];
        }
      }

      left_arm_ctrl_obs_B.smax =
        -left_arm_ctrl_obs_B.TmpSignalConversionAtSFun_j[5] /
        (1.9129500000000002 * left_arm_ctrl_obs_B.smax);
    } else {
      left_arm_ctrl_obs_B.smax = 0.0;
    }
  } else {
    left_arm_ctrl_obs_B.smax = 0.0;
  }

  // End of MATLAB Function: '<S3>/mass estimator'

  // RateTransition: '<S9>/Rate Transition'
  if ((rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
       left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) &&
      (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
       left_arm_ctrl_obs_M->Timing.TaskCounters.TID[2] == 0)) {
    left_arm_ctrl_obs_DW.RateTransition_Buffer = left_arm_ctrl_obs_B.smax;
  }

  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[2] == 0) {
    // BusAssignment: '<S9>/Bus Assignment2'
    left_arm_ctrl_obs_B.BusAssignment2.Data =
      left_arm_ctrl_obs_DW.RateTransition_Buffer;

    // Outputs for Atomic SubSystem: '<S9>/Publish2'
    // MATLABSystem: '<S27>/SinkBlock'
    Pub_left_arm_ctrl_obs_311.publish(&left_arm_ctrl_obs_B.BusAssignment2);

    // End of Outputs for SubSystem: '<S9>/Publish2'
  }

  // End of RateTransition: '<S9>/Rate Transition'
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) {
    // BusAssignment: '<S9>/Bus Assignment3' incorporates:
    //   Constant: '<S25>/Constant'
    //   Constant: '<S9>/Constant1'

    left_arm_ctrl_obs_B.BusAssignment3 = left_arm_ctrl_obs_P.Constant_Value_i;
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.BusAssignment3.Data[left_arm_ctrl_obs_B.i] =
        left_arm_ctrl_obs_B.Integrator_f[left_arm_ctrl_obs_B.i + 7];
    }

    left_arm_ctrl_obs_B.BusAssignment3.Data_SL_Info.CurrentLength =
      left_arm_ctrl_obs_P.Constant1_Value;
    left_arm_ctrl_obs_B.BusAssignment3.Data_SL_Info.ReceivedLength =
      left_arm_ctrl_obs_P.Constant1_Value;

    // End of BusAssignment: '<S9>/Bus Assignment3'

    // Outputs for Atomic SubSystem: '<S9>/Publish3'
    // MATLABSystem: '<S28>/SinkBlock'
    Pub_left_arm_ctrl_obs_331.publish(&left_arm_ctrl_obs_B.BusAssignment3);

    // End of Outputs for SubSystem: '<S9>/Publish3'

    // BusAssignment: '<S9>/Bus Assignment1' incorporates:
    //   Constant: '<S23>/Constant'
    //   Constant: '<S9>/Constant'

    left_arm_ctrl_obs_B.BusAssignment3 = left_arm_ctrl_obs_P.Constant_Value_e;
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.BusAssignment3.Data[left_arm_ctrl_obs_B.i] =
        left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.i];
    }

    left_arm_ctrl_obs_B.BusAssignment3.Data_SL_Info.CurrentLength =
      left_arm_ctrl_obs_P.Constant_Value_d;
    left_arm_ctrl_obs_B.BusAssignment3.Data_SL_Info.ReceivedLength =
      left_arm_ctrl_obs_P.Constant_Value_d;

    // End of BusAssignment: '<S9>/Bus Assignment1'

    // Outputs for Atomic SubSystem: '<S9>/Publish1'
    // MATLABSystem: '<S26>/SinkBlock'
    Pub_left_arm_ctrl_obs_304.publish(&left_arm_ctrl_obs_B.BusAssignment3);

    // End of Outputs for SubSystem: '<S9>/Publish1'

    // BusAssignment: '<S5>/Bus Assignment1' incorporates:
    //   Constant: '<S19>/Constant'
    //   Constant: '<S5>/Constant'

    left_arm_ctrl_obs_B.BusAssignment3 = left_arm_ctrl_obs_P.Constant_Value_b;
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.BusAssignment3.Data[left_arm_ctrl_obs_B.i] =
        left_arm_ctrl_obs_B.Integrator[left_arm_ctrl_obs_B.i];
    }

    left_arm_ctrl_obs_B.BusAssignment3.Data_SL_Info.CurrentLength =
      left_arm_ctrl_obs_P.Constant_Value_kl;
    left_arm_ctrl_obs_B.BusAssignment3.Data_SL_Info.ReceivedLength =
      left_arm_ctrl_obs_P.Constant_Value_kl;

    // End of BusAssignment: '<S5>/Bus Assignment1'

    // Outputs for Atomic SubSystem: '<S5>/Publish1'
    // MATLABSystem: '<S21>/SinkBlock'
    Pub_left_arm_ctrl_obs_630.publish(&left_arm_ctrl_obs_B.BusAssignment3);

    // End of Outputs for SubSystem: '<S5>/Publish1'

    // BusAssignment: '<S5>/Bus Assignment3' incorporates:
    //   Constant: '<S20>/Constant'
    //   Constant: '<S5>/Constant1'

    left_arm_ctrl_obs_B.BusAssignment3 = left_arm_ctrl_obs_P.Constant_Value_n;
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.BusAssignment3.Data[left_arm_ctrl_obs_B.i] =
        left_arm_ctrl_obs_B.Integrator[left_arm_ctrl_obs_B.i + 7];
    }

    left_arm_ctrl_obs_B.BusAssignment3.Data_SL_Info.CurrentLength =
      left_arm_ctrl_obs_P.Constant1_Value_i;
    left_arm_ctrl_obs_B.BusAssignment3.Data_SL_Info.ReceivedLength =
      left_arm_ctrl_obs_P.Constant1_Value_i;

    // End of BusAssignment: '<S5>/Bus Assignment3'

    // Outputs for Atomic SubSystem: '<S5>/Publish3'
    // MATLABSystem: '<S22>/SinkBlock'
    Pub_left_arm_ctrl_obs_632.publish(&left_arm_ctrl_obs_B.BusAssignment3);

    // End of Outputs for SubSystem: '<S5>/Publish3'
  }

  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M)) {
    rt_ertODEUpdateContinuousStates(&left_arm_ctrl_obs_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++left_arm_ctrl_obs_M->Timing.clockTick0;
    left_arm_ctrl_obs_M->Timing.t[0] = rtsiGetSolverStopTime
      (&left_arm_ctrl_obs_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.004s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.004, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      left_arm_ctrl_obs_M->Timing.clockTick1++;
    }

    rate_scheduler();
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void left_arm_ctrl_obs_derivatives(void)
{
  real_T LowPassz1_CSTATE_tmp;
  real_T LowPassz1_CSTATE_tmp_0;
  real_T LowPassz1_CSTATE_tmp_1;
  real_T LowPassz1_CSTATE_tmp_2;
  XDot_left_arm_ctrl_obs_T *_rtXdot;
  _rtXdot = ((XDot_left_arm_ctrl_obs_T *) left_arm_ctrl_obs_M->derivs);

  // Derivatives for Integrator: '<S1>/Integrator1'
  memcpy(&_rtXdot->Integrator1_CSTATE[0], &left_arm_ctrl_obs_B.Pp[0], 196U *
         sizeof(real_T));

  // Derivatives for Integrator: '<S1>/Integrator'
  memcpy(&_rtXdot->Integrator_CSTATE[0], &left_arm_ctrl_obs_B.xp[0], 14U *
         sizeof(real_T));

  // Derivatives for Integrator: '<S11>/Integrator'
  memcpy(&_rtXdot->Integrator_CSTATE_p[0], &left_arm_ctrl_obs_B.xp_est[0], 14U *
         sizeof(real_T));

  // Derivatives for TransferFcn: '<S2>/Low Pass (z1)' incorporates:
  //   TransferFcn: '<S2>/Low Pass (z1)1'
  //   TransferFcn: '<S2>/Low Pass (z1)2'
  //   TransferFcn: '<S2>/Low Pass (z1)3'
  //   TransferFcn: '<S2>/Low Pass (z1)4'
  //   TransferFcn: '<S2>/Low Pass (z1)5'
  //   TransferFcn: '<S2>/Low Pass (z1)6'

  _rtXdot->LowPassz1_CSTATE[0] = 0.0;
  LowPassz1_CSTATE_tmp = -left_arm_ctrl_obs_P.lowpass_A[1] /
    left_arm_ctrl_obs_P.lowpass_A[0];
  _rtXdot->LowPassz1_CSTATE[0] += LowPassz1_CSTATE_tmp *
    left_arm_ctrl_obs_X.LowPassz1_CSTATE[0];
  _rtXdot->LowPassz1_CSTATE[1] = 0.0;
  LowPassz1_CSTATE_tmp_0 = -left_arm_ctrl_obs_P.lowpass_A[2] /
    left_arm_ctrl_obs_P.lowpass_A[0];
  _rtXdot->LowPassz1_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    left_arm_ctrl_obs_X.LowPassz1_CSTATE[1];
  _rtXdot->LowPassz1_CSTATE[2] = 0.0;
  LowPassz1_CSTATE_tmp_1 = -left_arm_ctrl_obs_P.lowpass_A[3] /
    left_arm_ctrl_obs_P.lowpass_A[0];
  _rtXdot->LowPassz1_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    left_arm_ctrl_obs_X.LowPassz1_CSTATE[2];
  _rtXdot->LowPassz1_CSTATE[3] = 0.0;
  LowPassz1_CSTATE_tmp_2 = -left_arm_ctrl_obs_P.lowpass_A[4] /
    left_arm_ctrl_obs_P.lowpass_A[0];
  _rtXdot->LowPassz1_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    left_arm_ctrl_obs_X.LowPassz1_CSTATE[3];
  _rtXdot->LowPassz1_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz1_CSTATE[0];
  _rtXdot->LowPassz1_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz1_CSTATE[1];
  _rtXdot->LowPassz1_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz1_CSTATE[2];
  _rtXdot->LowPassz1_CSTATE[0] += left_arm_ctrl_obs_B.z[0];

  // Derivatives for TransferFcn: '<S2>/Low Pass (z1)1'
  _rtXdot->LowPassz11_CSTATE[0] = 0.0;
  _rtXdot->LowPassz11_CSTATE[0] += LowPassz1_CSTATE_tmp *
    left_arm_ctrl_obs_X.LowPassz11_CSTATE[0];
  _rtXdot->LowPassz11_CSTATE[1] = 0.0;
  _rtXdot->LowPassz11_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    left_arm_ctrl_obs_X.LowPassz11_CSTATE[1];
  _rtXdot->LowPassz11_CSTATE[2] = 0.0;
  _rtXdot->LowPassz11_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    left_arm_ctrl_obs_X.LowPassz11_CSTATE[2];
  _rtXdot->LowPassz11_CSTATE[3] = 0.0;
  _rtXdot->LowPassz11_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    left_arm_ctrl_obs_X.LowPassz11_CSTATE[3];
  _rtXdot->LowPassz11_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz11_CSTATE[0];
  _rtXdot->LowPassz11_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz11_CSTATE[1];
  _rtXdot->LowPassz11_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz11_CSTATE[2];
  _rtXdot->LowPassz11_CSTATE[0] += left_arm_ctrl_obs_B.z[1];

  // Derivatives for TransferFcn: '<S2>/Low Pass (z1)2'
  _rtXdot->LowPassz12_CSTATE[0] = 0.0;
  _rtXdot->LowPassz12_CSTATE[0] += LowPassz1_CSTATE_tmp *
    left_arm_ctrl_obs_X.LowPassz12_CSTATE[0];
  _rtXdot->LowPassz12_CSTATE[1] = 0.0;
  _rtXdot->LowPassz12_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    left_arm_ctrl_obs_X.LowPassz12_CSTATE[1];
  _rtXdot->LowPassz12_CSTATE[2] = 0.0;
  _rtXdot->LowPassz12_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    left_arm_ctrl_obs_X.LowPassz12_CSTATE[2];
  _rtXdot->LowPassz12_CSTATE[3] = 0.0;
  _rtXdot->LowPassz12_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    left_arm_ctrl_obs_X.LowPassz12_CSTATE[3];
  _rtXdot->LowPassz12_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz12_CSTATE[0];
  _rtXdot->LowPassz12_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz12_CSTATE[1];
  _rtXdot->LowPassz12_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz12_CSTATE[2];
  _rtXdot->LowPassz12_CSTATE[0] += left_arm_ctrl_obs_B.z[2];

  // Derivatives for TransferFcn: '<S2>/Low Pass (z1)3'
  _rtXdot->LowPassz13_CSTATE[0] = 0.0;
  _rtXdot->LowPassz13_CSTATE[0] += LowPassz1_CSTATE_tmp *
    left_arm_ctrl_obs_X.LowPassz13_CSTATE[0];
  _rtXdot->LowPassz13_CSTATE[1] = 0.0;
  _rtXdot->LowPassz13_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    left_arm_ctrl_obs_X.LowPassz13_CSTATE[1];
  _rtXdot->LowPassz13_CSTATE[2] = 0.0;
  _rtXdot->LowPassz13_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    left_arm_ctrl_obs_X.LowPassz13_CSTATE[2];
  _rtXdot->LowPassz13_CSTATE[3] = 0.0;
  _rtXdot->LowPassz13_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    left_arm_ctrl_obs_X.LowPassz13_CSTATE[3];
  _rtXdot->LowPassz13_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz13_CSTATE[0];
  _rtXdot->LowPassz13_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz13_CSTATE[1];
  _rtXdot->LowPassz13_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz13_CSTATE[2];
  _rtXdot->LowPassz13_CSTATE[0] += left_arm_ctrl_obs_B.z[3];

  // Derivatives for TransferFcn: '<S2>/Low Pass (z1)4'
  _rtXdot->LowPassz14_CSTATE[0] = 0.0;
  _rtXdot->LowPassz14_CSTATE[0] += LowPassz1_CSTATE_tmp *
    left_arm_ctrl_obs_X.LowPassz14_CSTATE[0];
  _rtXdot->LowPassz14_CSTATE[1] = 0.0;
  _rtXdot->LowPassz14_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    left_arm_ctrl_obs_X.LowPassz14_CSTATE[1];
  _rtXdot->LowPassz14_CSTATE[2] = 0.0;
  _rtXdot->LowPassz14_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    left_arm_ctrl_obs_X.LowPassz14_CSTATE[2];
  _rtXdot->LowPassz14_CSTATE[3] = 0.0;
  _rtXdot->LowPassz14_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    left_arm_ctrl_obs_X.LowPassz14_CSTATE[3];
  _rtXdot->LowPassz14_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz14_CSTATE[0];
  _rtXdot->LowPassz14_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz14_CSTATE[1];
  _rtXdot->LowPassz14_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz14_CSTATE[2];
  _rtXdot->LowPassz14_CSTATE[0] += left_arm_ctrl_obs_B.z[4];

  // Derivatives for TransferFcn: '<S2>/Low Pass (z1)5'
  _rtXdot->LowPassz15_CSTATE[0] = 0.0;
  _rtXdot->LowPassz15_CSTATE[0] += LowPassz1_CSTATE_tmp *
    left_arm_ctrl_obs_X.LowPassz15_CSTATE[0];
  _rtXdot->LowPassz15_CSTATE[1] = 0.0;
  _rtXdot->LowPassz15_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    left_arm_ctrl_obs_X.LowPassz15_CSTATE[1];
  _rtXdot->LowPassz15_CSTATE[2] = 0.0;
  _rtXdot->LowPassz15_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    left_arm_ctrl_obs_X.LowPassz15_CSTATE[2];
  _rtXdot->LowPassz15_CSTATE[3] = 0.0;
  _rtXdot->LowPassz15_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    left_arm_ctrl_obs_X.LowPassz15_CSTATE[3];
  _rtXdot->LowPassz15_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz15_CSTATE[0];
  _rtXdot->LowPassz15_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz15_CSTATE[1];
  _rtXdot->LowPassz15_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz15_CSTATE[2];
  _rtXdot->LowPassz15_CSTATE[0] += left_arm_ctrl_obs_B.z[5];

  // Derivatives for TransferFcn: '<S2>/Low Pass (z1)6'
  _rtXdot->LowPassz16_CSTATE[0] = 0.0;
  _rtXdot->LowPassz16_CSTATE[0] += LowPassz1_CSTATE_tmp *
    left_arm_ctrl_obs_X.LowPassz16_CSTATE[0];
  _rtXdot->LowPassz16_CSTATE[1] = 0.0;
  _rtXdot->LowPassz16_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    left_arm_ctrl_obs_X.LowPassz16_CSTATE[1];
  _rtXdot->LowPassz16_CSTATE[2] = 0.0;
  _rtXdot->LowPassz16_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    left_arm_ctrl_obs_X.LowPassz16_CSTATE[2];
  _rtXdot->LowPassz16_CSTATE[3] = 0.0;
  _rtXdot->LowPassz16_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    left_arm_ctrl_obs_X.LowPassz16_CSTATE[3];
  _rtXdot->LowPassz16_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz16_CSTATE[0];
  _rtXdot->LowPassz16_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz16_CSTATE[1];
  _rtXdot->LowPassz16_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz16_CSTATE[2];
  _rtXdot->LowPassz16_CSTATE[0] += left_arm_ctrl_obs_B.z[6];
}

// Model initialize function
void left_arm_ctrl_obs_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&left_arm_ctrl_obs_M->solverInfo,
                          &left_arm_ctrl_obs_M->Timing.simTimeStep);
    rtsiSetTPtr(&left_arm_ctrl_obs_M->solverInfo, &rtmGetTPtr
                (left_arm_ctrl_obs_M));
    rtsiSetStepSizePtr(&left_arm_ctrl_obs_M->solverInfo,
                       &left_arm_ctrl_obs_M->Timing.stepSize0);
    rtsiSetdXPtr(&left_arm_ctrl_obs_M->solverInfo, &left_arm_ctrl_obs_M->derivs);
    rtsiSetContStatesPtr(&left_arm_ctrl_obs_M->solverInfo, (real_T **)
                         &left_arm_ctrl_obs_M->contStates);
    rtsiSetNumContStatesPtr(&left_arm_ctrl_obs_M->solverInfo,
      &left_arm_ctrl_obs_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&left_arm_ctrl_obs_M->solverInfo,
      &left_arm_ctrl_obs_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&left_arm_ctrl_obs_M->solverInfo,
      &left_arm_ctrl_obs_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&left_arm_ctrl_obs_M->solverInfo,
      &left_arm_ctrl_obs_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&left_arm_ctrl_obs_M->solverInfo, (&rtmGetErrorStatus
      (left_arm_ctrl_obs_M)));
    rtsiSetRTModelPtr(&left_arm_ctrl_obs_M->solverInfo, left_arm_ctrl_obs_M);
  }

  rtsiSetSimTimeStep(&left_arm_ctrl_obs_M->solverInfo, MAJOR_TIME_STEP);
  left_arm_ctrl_obs_M->intgData.y = left_arm_ctrl_obs_M->odeY;
  left_arm_ctrl_obs_M->intgData.f[0] = left_arm_ctrl_obs_M->odeF[0];
  left_arm_ctrl_obs_M->intgData.f[1] = left_arm_ctrl_obs_M->odeF[1];
  left_arm_ctrl_obs_M->intgData.f[2] = left_arm_ctrl_obs_M->odeF[2];
  left_arm_ctrl_obs_M->intgData.f[3] = left_arm_ctrl_obs_M->odeF[3];
  left_arm_ctrl_obs_M->contStates = ((X_left_arm_ctrl_obs_T *)
    &left_arm_ctrl_obs_X);
  rtsiSetSolverData(&left_arm_ctrl_obs_M->solverInfo, static_cast<void *>
                    (&left_arm_ctrl_obs_M->intgData));
  rtsiSetSolverName(&left_arm_ctrl_obs_M->solverInfo,"ode4");
  rtmSetTPtr(left_arm_ctrl_obs_M, &left_arm_ctrl_obs_M->Timing.tArray[0]);
  left_arm_ctrl_obs_M->Timing.stepSize0 = 0.004;

  {
    static const char_T tmp[25] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r', 'e',
      '/', 'l', 'a', '_', 'c', 'u', 'r', 'r', 'e', 'n', 't', '_', 'p', 'o', 's',
      'e' };

    static const char_T tmp_0[28] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'g', 'o', 'a', 'l', '_', 't', 'r', 'a', 'j', 'e',
      'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_1[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'P', '1' };

    static const char_T tmp_2[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'P', '2' };

    static const char_T tmp_3[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'P', '3' };

    static const char_T tmp_4[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'P', '4' };

    static const char_T tmp_5[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'P', '5' };

    static const char_T tmp_6[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'P', '6' };

    static const char_T tmp_7[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'P', '7' };

    static const char_T tmp_8[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'D', '1' };

    static const char_T tmp_9[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'D', '2' };

    static const char_T tmp_a[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'D', '3' };

    static const char_T tmp_b[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'D', '4' };

    static const char_T tmp_c[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'D', '5' };

    static const char_T tmp_d[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'D', '6' };

    static const char_T tmp_e[11] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'D', '7' };

    static const char_T tmp_f[6] = { '/', 'e', 'k', 'f', '/', 'Q' };

    static const char_T tmp_g[6] = { '/', 'e', 'k', 'f', '/', 'R' };

    static const char_T tmp_h[11] = { '/', 's', 'm', 'o', '/', 'l', 'a', 'm',
      'b', 'd', 'a' };

    static const char_T tmp_i[10] = { '/', 's', 'm', 'o', '/', 'a', 'l', 'p',
      'h', 'a' };

    static const char_T tmp_j[10] = { '/', 's', 'm', 'o', '/', 'g', 'a', 'm',
      'm', 'a' };

    static const char_T tmp_k[15] = { '/', 'e', 's', 't', 'i', 'm', 'a', 't',
      'e', 'd', '_', 'm', 'a', 's', 's' };

    static const char_T tmp_l[21] = { '/', 's', 'm', 'o', '/', 'e', 's', 't',
      'i', 'm', 'a', 't', 'e', 'd', '_', 's', 'p', 'e', 'e', 'd', 's' };

    static const char_T tmp_m[19] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 't', 'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_n[16] = { '/', 'e', 'k', 'f', '/', 'e', 's', 't',
      'i', 'm', 'a', 't', 'e', 'd', '_', 'q' };

    static const char_T tmp_o[21] = { '/', 'e', 'k', 'f', '/', 'e', 's', 't',
      'i', 'm', 'a', 't', 'e', 'd', '_', 's', 'p', 'e', 'e', 'd', 's' };

    // Start for Atomic SubSystem: '<S10>/Subscribe'
    // Start for MATLABSystem: '<S29>/SourceBlock'
    left_arm_ctrl_obs_DW.obj_m.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_m.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_m.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_m.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_m.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 25;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv1[left_arm_ctrl_obs_B.i_n] =
        tmp[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv1[25] = '\x00';
    Sub_left_arm_ctrl_obs_299.createSubscriber(left_arm_ctrl_obs_B.cv1, 1);
    left_arm_ctrl_obs_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S29>/SourceBlock'
    // End of Start for SubSystem: '<S10>/Subscribe'

    // Start for Atomic SubSystem: '<S10>/Subscribe1'
    // Start for MATLABSystem: '<S30>/SourceBlock'
    left_arm_ctrl_obs_DW.obj_gx.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_gx.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_gx.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_gx.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_gx.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 28;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv[left_arm_ctrl_obs_B.i_n] =
        tmp_0[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv[28] = '\x00';
    Sub_left_arm_ctrl_obs_318.createSubscriber(left_arm_ctrl_obs_B.cv, 1);
    left_arm_ctrl_obs_DW.obj_gx.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S30>/SourceBlock'
    // End of Start for SubSystem: '<S10>/Subscribe1'

    // Start for MATLABSystem: '<S6>/Get Parameter'
    left_arm_ctrl_obs_DW.obj_l.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_l.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_l.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_l.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_l.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_1[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_368.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_368.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_368.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter'

    // Start for MATLABSystem: '<S6>/Get Parameter1'
    left_arm_ctrl_obs_DW.obj_oi.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_oi.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_oi.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_oi.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_oi.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_2[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_370.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_370.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_370.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_oi.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter1'

    // Start for MATLABSystem: '<S6>/Get Parameter2'
    left_arm_ctrl_obs_DW.obj_o1.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_o1.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_o1.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_o1.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_o1.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_3[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_372.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_372.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_372.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_o1.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter2'

    // Start for MATLABSystem: '<S6>/Get Parameter3'
    left_arm_ctrl_obs_DW.obj_k.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_k.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_k.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_k.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_k.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_4[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_374.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_374.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_374.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_k.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter3'

    // Start for MATLABSystem: '<S6>/Get Parameter4'
    left_arm_ctrl_obs_DW.obj_d.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_d.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_d.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_d.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_d.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_5[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_376.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_376.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_376.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter4'

    // Start for MATLABSystem: '<S6>/Get Parameter5'
    left_arm_ctrl_obs_DW.obj_i.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_i.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_i.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_i.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_i.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_6[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_378.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_378.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_378.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_i.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter5'

    // Start for MATLABSystem: '<S6>/Get Parameter6'
    left_arm_ctrl_obs_DW.obj_fn.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_fn.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_fn.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_fn.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_fn.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_7[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_380.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_380.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_380.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_fn.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter6'

    // Start for MATLABSystem: '<S6>/Get Parameter7'
    left_arm_ctrl_obs_DW.obj_hn.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_hn.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_hn.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_hn.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_hn.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_8[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_400.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_400.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_400.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_hn.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter7'

    // Start for MATLABSystem: '<S6>/Get Parameter8'
    left_arm_ctrl_obs_DW.obj_og.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_og.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_og.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_og.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_og.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_9[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_401.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_401.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_401.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_og.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter8'

    // Start for MATLABSystem: '<S6>/Get Parameter9'
    left_arm_ctrl_obs_DW.obj_fy.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_fy.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_fy.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_fy.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_fy.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_a[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_402.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_402.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_402.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_fy.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter9'

    // Start for MATLABSystem: '<S6>/Get Parameter10'
    left_arm_ctrl_obs_DW.obj_if.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_if.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_if.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_if.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_if.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_b[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_403.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_403.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_403.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_if.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter10'

    // Start for MATLABSystem: '<S6>/Get Parameter11'
    left_arm_ctrl_obs_DW.obj_g.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_g.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_g.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_g.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_g.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_c[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_404.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_404.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_404.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter11'

    // Start for MATLABSystem: '<S6>/Get Parameter12'
    left_arm_ctrl_obs_DW.obj_c.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_c.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_c.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_c.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_c.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_d[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_405.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_405.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_405.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter12'

    // Start for MATLABSystem: '<S6>/Get Parameter13'
    left_arm_ctrl_obs_DW.obj_p.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_p.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_p.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_p.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_p.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_e[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_406.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_406.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_406.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_p.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/Get Parameter13'
    emxInitStruct_robotics_slmani_e(&left_arm_ctrl_obs_DW.obj);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_0);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_19);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_18);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_17);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_16);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_15);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_14);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_13);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_12);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_11);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_10);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_9);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_8);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_7);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_6);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_5);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_4);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_3);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_2);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_1);

    // Start for MATLABSystem: '<S17>/MATLAB System'
    left_arm_ctrl_obs_DW.obj.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_e0h(&left_arm_ctrl_obs_DW.obj.TreeInternal,
      &left_arm_ctrl_obs_DW.gobj_0, &left_arm_ctrl_obs_DW.gobj_19,
      &left_arm_ctrl_obs_DW.gobj_18, &left_arm_ctrl_obs_DW.gobj_17,
      &left_arm_ctrl_obs_DW.gobj_16, &left_arm_ctrl_obs_DW.gobj_15,
      &left_arm_ctrl_obs_DW.gobj_14, &left_arm_ctrl_obs_DW.gobj_13,
      &left_arm_ctrl_obs_DW.gobj_12, &left_arm_ctrl_obs_DW.gobj_11);

    // Constant: '<S1>/Constant2'
    left_arm_ctr_MATLABSystem_Start(&left_arm_ctrl_obs_B.MATLABSystem,
      &left_arm_ctrl_obs_DW.MATLABSystem);

    // Start for MATLABSystem: '<S8>/Get Parameter7'
    left_arm_ctrl_obs_DW.obj_f.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_f.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_f.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_f.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_f.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 6;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv8[left_arm_ctrl_obs_B.i_n] =
        tmp_f[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv8[6] = '\x00';
    ParamGet_left_arm_ctrl_obs_608.initialize(left_arm_ctrl_obs_B.cv8);
    ParamGet_left_arm_ctrl_obs_608.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_608.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/Get Parameter7'

    // Start for MATLABSystem: '<S8>/Get Parameter8'
    left_arm_ctrl_obs_DW.obj_hh.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_hh.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_hh.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_hh.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_hh.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 6;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv8[left_arm_ctrl_obs_B.i_n] =
        tmp_g[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv8[6] = '\x00';
    ParamGet_left_arm_ctrl_obs_609.initialize(left_arm_ctrl_obs_B.cv8);
    ParamGet_left_arm_ctrl_obs_609.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_609.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_hh.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/Get Parameter8'
    emxInitStruct_robotics_slman_e0(&left_arm_ctrl_obs_DW.obj_h);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_0_h);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_19_g);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_18_i);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_17_a);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_16_f);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_15_j);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_14_m);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_13_a);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_12_f);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_11_g);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_10_j);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_9_b);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_8_j);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_7_a);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_6_c);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_5_n);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_4_m);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_3_d);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_2_e);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_1_p);

    // Start for MATLABSystem: '<S15>/MATLAB System'
    left_arm_ctrl_obs_DW.obj_h.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_h.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_e0(&left_arm_ctrl_obs_DW.obj_h.TreeInternal,
      &left_arm_ctrl_obs_DW.gobj_0_h, &left_arm_ctrl_obs_DW.gobj_19_g,
      &left_arm_ctrl_obs_DW.gobj_18_i, &left_arm_ctrl_obs_DW.gobj_17_a,
      &left_arm_ctrl_obs_DW.gobj_16_f, &left_arm_ctrl_obs_DW.gobj_15_j,
      &left_arm_ctrl_obs_DW.gobj_14_m, &left_arm_ctrl_obs_DW.gobj_13_a,
      &left_arm_ctrl_obs_DW.gobj_12_f, &left_arm_ctrl_obs_DW.gobj_11_g);
    emxInitStruct_robotics_slma_e0h(&left_arm_ctrl_obs_DW.obj_o);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_0_h4);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_19_e);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_18_d);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_17_p);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_16_e);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_15_i);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_14_b);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_13_g);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_12_c);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_11_d);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_10_l);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_9_a);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_8_h);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_7_e);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_6_k);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_5_nh);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_4_n);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_3_l);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_2_h);
    emxInitStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_1_h);

    // Start for MATLABSystem: '<S14>/MATLAB System'
    left_arm_ctrl_obs_DW.obj_o.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_o.isInitialized = 1;
    l_RigidBodyTree_RigidBodyTree_e(&left_arm_ctrl_obs_DW.obj_o.TreeInternal,
      &left_arm_ctrl_obs_DW.gobj_0_h4, &left_arm_ctrl_obs_DW.gobj_19_e,
      &left_arm_ctrl_obs_DW.gobj_18_d, &left_arm_ctrl_obs_DW.gobj_17_p,
      &left_arm_ctrl_obs_DW.gobj_16_e, &left_arm_ctrl_obs_DW.gobj_15_i,
      &left_arm_ctrl_obs_DW.gobj_14_b, &left_arm_ctrl_obs_DW.gobj_13_g,
      &left_arm_ctrl_obs_DW.gobj_12_c, &left_arm_ctrl_obs_DW.gobj_11_d);

    // Constant: '<S11>/Constant2'
    left_arm_ctr_MATLABSystem_Start(&left_arm_ctrl_obs_B.MATLABSystem_me,
      &left_arm_ctrl_obs_DW.MATLABSystem_me);

    // Start for MATLABSystem: '<S7>/Get Parameter7'
    left_arm_ctrl_obs_DW.obj_a.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_a.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_a.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_a.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_a.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 11;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv6[left_arm_ctrl_obs_B.i_n] =
        tmp_h[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv6[11] = '\x00';
    ParamGet_left_arm_ctrl_obs_383.initialize(left_arm_ctrl_obs_B.cv6);
    ParamGet_left_arm_ctrl_obs_383.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_383.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter7'

    // Start for MATLABSystem: '<S7>/Get Parameter8'
    left_arm_ctrl_obs_DW.obj_oo.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_oo.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_oo.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_oo.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_oo.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 10;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv7[left_arm_ctrl_obs_B.i_n] =
        tmp_i[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv7[10] = '\x00';
    ParamGet_left_arm_ctrl_obs_384.initialize(left_arm_ctrl_obs_B.cv7);
    ParamGet_left_arm_ctrl_obs_384.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_384.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_oo.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter8'

    // Start for MATLABSystem: '<S7>/Get Parameter9'
    left_arm_ctrl_obs_DW.obj_ok.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_ok.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_ok.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_ok.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_ok.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 10;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv7[left_arm_ctrl_obs_B.i_n] =
        tmp_j[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv7[10] = '\x00';
    ParamGet_left_arm_ctrl_obs_385.initialize(left_arm_ctrl_obs_B.cv7);
    ParamGet_left_arm_ctrl_obs_385.initialize_error_codes(0, 1, 2, 3);
    ParamGet_left_arm_ctrl_obs_385.set_initial_value(0.0);
    left_arm_ctrl_obs_DW.obj_ok.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S7>/Get Parameter9'

    // Start for Atomic SubSystem: '<S9>/Publish2'
    // Start for MATLABSystem: '<S27>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_ap.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_ap.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_ap.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_ap.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_ap.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 15;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv5[left_arm_ctrl_obs_B.i_n] =
        tmp_k[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv5[15] = '\x00';
    Pub_left_arm_ctrl_obs_311.createPublisher(left_arm_ctrl_obs_B.cv5, 1);
    left_arm_ctrl_obs_DW.obj_ap.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S27>/SinkBlock'
    // End of Start for SubSystem: '<S9>/Publish2'

    // Start for Atomic SubSystem: '<S9>/Publish3'
    // Start for MATLABSystem: '<S28>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_b.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_b.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_b.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_b.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_b.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 21;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv2[left_arm_ctrl_obs_B.i_n] =
        tmp_l[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv2[21] = '\x00';
    Pub_left_arm_ctrl_obs_331.createPublisher(left_arm_ctrl_obs_B.cv2, 1);
    left_arm_ctrl_obs_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S28>/SinkBlock'
    // End of Start for SubSystem: '<S9>/Publish3'

    // Start for Atomic SubSystem: '<S9>/Publish1'
    // Start for MATLABSystem: '<S26>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_d1.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_d1.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_d1.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_d1.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_d1.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 19;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv3[left_arm_ctrl_obs_B.i_n] =
        tmp_m[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv3[19] = '\x00';
    Pub_left_arm_ctrl_obs_304.createPublisher(left_arm_ctrl_obs_B.cv3, 1);
    left_arm_ctrl_obs_DW.obj_d1.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/SinkBlock'
    // End of Start for SubSystem: '<S9>/Publish1'

    // Start for Atomic SubSystem: '<S5>/Publish1'
    // Start for MATLABSystem: '<S21>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_j.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_j.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_j.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_j.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_j.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 16;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv4[left_arm_ctrl_obs_B.i_n] =
        tmp_n[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv4[16] = '\x00';
    Pub_left_arm_ctrl_obs_630.createPublisher(left_arm_ctrl_obs_B.cv4, 1);
    left_arm_ctrl_obs_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/SinkBlock'
    // End of Start for SubSystem: '<S5>/Publish1'

    // Start for Atomic SubSystem: '<S5>/Publish3'
    // Start for MATLABSystem: '<S22>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_kp.matlabCodegenIsDeleted = true;
    left_arm_ctrl_obs_DW.obj_kp.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_kp.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_kp.isSetupComplete = false;
    left_arm_ctrl_obs_DW.obj_kp.isInitialized = 1;
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 21;
         left_arm_ctrl_obs_B.i_n++) {
      left_arm_ctrl_obs_B.cv2[left_arm_ctrl_obs_B.i_n] =
        tmp_o[left_arm_ctrl_obs_B.i_n];
    }

    left_arm_ctrl_obs_B.cv2[21] = '\x00';
    Pub_left_arm_ctrl_obs_632.createPublisher(left_arm_ctrl_obs_B.cv2, 1);
    left_arm_ctrl_obs_DW.obj_kp.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S22>/SinkBlock'
    // End of Start for SubSystem: '<S5>/Publish3'

    // InitializeConditions for Integrator: '<S1>/Integrator1'
    memcpy(&left_arm_ctrl_obs_X.Integrator1_CSTATE[0],
           &left_arm_ctrl_obs_P.Integrator1_IC[0], 196U * sizeof(real_T));
    for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 14;
         left_arm_ctrl_obs_B.i_n++) {
      // InitializeConditions for Integrator: '<S1>/Integrator'
      left_arm_ctrl_obs_X.Integrator_CSTATE[left_arm_ctrl_obs_B.i_n] =
        left_arm_ctrl_obs_P.Integrator_IC;

      // InitializeConditions for Integrator: '<S11>/Integrator'
      left_arm_ctrl_obs_X.Integrator_CSTATE_p[left_arm_ctrl_obs_B.i_n] =
        left_arm_ctrl_obs_P.Integrator_IC_c[left_arm_ctrl_obs_B.i_n];
    }

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)'
    left_arm_ctrl_obs_X.LowPassz1_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)1'
    left_arm_ctrl_obs_X.LowPassz11_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)2'
    left_arm_ctrl_obs_X.LowPassz12_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)3'
    left_arm_ctrl_obs_X.LowPassz13_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)4'
    left_arm_ctrl_obs_X.LowPassz14_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)5'
    left_arm_ctrl_obs_X.LowPassz15_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)6'
    left_arm_ctrl_obs_X.LowPassz16_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)'
    left_arm_ctrl_obs_X.LowPassz1_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)1'
    left_arm_ctrl_obs_X.LowPassz11_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)2'
    left_arm_ctrl_obs_X.LowPassz12_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)3'
    left_arm_ctrl_obs_X.LowPassz13_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)4'
    left_arm_ctrl_obs_X.LowPassz14_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)5'
    left_arm_ctrl_obs_X.LowPassz15_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)6'
    left_arm_ctrl_obs_X.LowPassz16_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)'
    left_arm_ctrl_obs_X.LowPassz1_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)1'
    left_arm_ctrl_obs_X.LowPassz11_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)2'
    left_arm_ctrl_obs_X.LowPassz12_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)3'
    left_arm_ctrl_obs_X.LowPassz13_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)4'
    left_arm_ctrl_obs_X.LowPassz14_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)5'
    left_arm_ctrl_obs_X.LowPassz15_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)6'
    left_arm_ctrl_obs_X.LowPassz16_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)'
    left_arm_ctrl_obs_X.LowPassz1_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)1'
    left_arm_ctrl_obs_X.LowPassz11_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)2'
    left_arm_ctrl_obs_X.LowPassz12_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)3'
    left_arm_ctrl_obs_X.LowPassz13_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)4'
    left_arm_ctrl_obs_X.LowPassz14_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)5'
    left_arm_ctrl_obs_X.LowPassz15_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S2>/Low Pass (z1)6'
    left_arm_ctrl_obs_X.LowPassz16_CSTATE[3] = 0.0;

    // SystemInitialize for Atomic SubSystem: '<S10>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S29>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S31>/Out1'
    left_arm_ctrl_obs_B.In1 = left_arm_ctrl_obs_P.Out1_Y0_a;

    // End of SystemInitialize for SubSystem: '<S29>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S10>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<S10>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S30>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S32>/Out1'
    left_arm_ctrl_obs_B.In1_e = left_arm_ctrl_obs_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S30>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S10>/Subscribe1'
  }
}

// Model terminate function
void left_arm_ctrl_obs_terminate(void)
{
  // Terminate for Atomic SubSystem: '<S10>/Subscribe'
  // Terminate for MATLABSystem: '<S29>/SourceBlock'
  left_arm_ctrl_matlabCodegenHa_l(&left_arm_ctrl_obs_DW.obj_m);

  // End of Terminate for SubSystem: '<S10>/Subscribe'

  // Terminate for Atomic SubSystem: '<S10>/Subscribe1'
  // Terminate for MATLABSystem: '<S30>/SourceBlock'
  left_arm_ctrl_matlabCodegenHa_l(&left_arm_ctrl_obs_DW.obj_gx);

  // End of Terminate for SubSystem: '<S10>/Subscribe1'

  // Terminate for MATLABSystem: '<S6>/Get Parameter'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_l);

  // Terminate for MATLABSystem: '<S6>/Get Parameter1'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_oi);

  // Terminate for MATLABSystem: '<S6>/Get Parameter2'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_o1);

  // Terminate for MATLABSystem: '<S6>/Get Parameter3'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_k);

  // Terminate for MATLABSystem: '<S6>/Get Parameter4'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_d);

  // Terminate for MATLABSystem: '<S6>/Get Parameter5'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_i);

  // Terminate for MATLABSystem: '<S6>/Get Parameter6'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_fn);

  // Terminate for MATLABSystem: '<S6>/Get Parameter7'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_hn);

  // Terminate for MATLABSystem: '<S6>/Get Parameter8'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_og);

  // Terminate for MATLABSystem: '<S6>/Get Parameter9'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_fy);

  // Terminate for MATLABSystem: '<S6>/Get Parameter10'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_if);

  // Terminate for MATLABSystem: '<S6>/Get Parameter11'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_g);

  // Terminate for MATLABSystem: '<S6>/Get Parameter12'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_c);

  // Terminate for MATLABSystem: '<S6>/Get Parameter13'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_p);
  emxFreeStruct_robotics_slmani_e(&left_arm_ctrl_obs_DW.obj);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_0);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_19);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_18);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_17);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_16);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_15);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_14);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_13);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_12);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_11);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_10);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_9);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_8);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_7);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_6);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_5);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_4);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_3);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_2);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_1);
  left_arm_ctrl_MATLABSystem_Term(&left_arm_ctrl_obs_DW.MATLABSystem);

  // Terminate for MATLABSystem: '<S8>/Get Parameter7'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_f);

  // Terminate for MATLABSystem: '<S8>/Get Parameter8'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_hh);
  emxFreeStruct_robotics_slman_e0(&left_arm_ctrl_obs_DW.obj_h);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_0_h);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_19_g);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_18_i);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_17_a);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_16_f);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_15_j);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_14_m);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_13_a);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_12_f);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_11_g);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_10_j);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_9_b);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_8_j);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_7_a);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_6_c);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_5_n);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_4_m);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_3_d);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_2_e);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_1_p);
  emxFreeStruct_robotics_slma_e0h(&left_arm_ctrl_obs_DW.obj_o);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_0_h4);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_19_e);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_18_d);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_17_p);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_16_e);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_15_i);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_14_b);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_13_g);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_12_c);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_11_d);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_10_l);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_9_a);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_8_h);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_7_e);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_6_k);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_5_nh);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_4_n);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_3_l);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_2_h);
  emxFreeStruct_j_robotics_man_e0(&left_arm_ctrl_obs_DW.gobj_1_h);
  left_arm_ctrl_MATLABSystem_Term(&left_arm_ctrl_obs_DW.MATLABSystem_me);

  // Terminate for MATLABSystem: '<S7>/Get Parameter7'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_a);

  // Terminate for MATLABSystem: '<S7>/Get Parameter8'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_oo);

  // Terminate for MATLABSystem: '<S7>/Get Parameter9'
  matlabCodegenHandle_matlabCo_e0(&left_arm_ctrl_obs_DW.obj_ok);

  // Terminate for Atomic SubSystem: '<S9>/Publish2'
  // Terminate for MATLABSystem: '<S27>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_ap);

  // End of Terminate for SubSystem: '<S9>/Publish2'

  // Terminate for Atomic SubSystem: '<S9>/Publish3'
  // Terminate for MATLABSystem: '<S28>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_b);

  // End of Terminate for SubSystem: '<S9>/Publish3'

  // Terminate for Atomic SubSystem: '<S9>/Publish1'
  // Terminate for MATLABSystem: '<S26>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_d1);

  // End of Terminate for SubSystem: '<S9>/Publish1'

  // Terminate for Atomic SubSystem: '<S5>/Publish1'
  // Terminate for MATLABSystem: '<S21>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_j);

  // End of Terminate for SubSystem: '<S5>/Publish1'

  // Terminate for Atomic SubSystem: '<S5>/Publish3'
  // Terminate for MATLABSystem: '<S22>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_kp);

  // End of Terminate for SubSystem: '<S5>/Publish3'
}

//
// File trailer for generated code.
//
// [EOF]
//
