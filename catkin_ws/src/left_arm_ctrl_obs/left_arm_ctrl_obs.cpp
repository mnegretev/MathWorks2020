//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_ctrl_obs.cpp
//
// Code generated for Simulink model 'left_arm_ctrl_obs'.
//
// Model version                  : 1.203
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Mon Jul 13 12:03:31 2020
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
static void left_arm_ctrl_ob_emxInit_real_T(emxArray_real_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions);
static void left_arm_ctrl_o_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void left_a_emxEnsureCapacity_real_T(emxArray_real_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel);
static void left_arm_c_emxInit_e_cell_wrap1(emxArray_e_cell_wrap_left_a_e_T
  **pEmxArray, int32_T numDimensions);
static void emxEnsureCapacity_e_cell_wrap1(emxArray_e_cell_wrap_left_a_e_T
  *emxArray, int32_T oldNumel);
static void le_rigidBodyJoint_get_JointAxis(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T ax[3]);
static void left_arm_ctrl_obs_normalizeRows(const real_T matrix[3], real_T
  normRowMatrix[3]);
static void left_arm_ctrl_obs_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_e(const
  rigidBodyJoint_left_arm_ctrl__T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T T[16]);
static void left_arm_ctrl_obs_tforminv(const real_T T[16], real_T Tinv[16]);
static void left_arm_ct_tformToSpatialXform(const real_T T[16], real_T X[36]);
static void left_arm_ctrl_ob_emxFree_real_T(emxArray_real_T_left_arm_ctrl_T
  **pEmxArray);
static void left_arm_c_emxFree_e_cell_wrap1(emxArray_e_cell_wrap_left_a_e_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMat_e(k_robotics_manip_internal_e0h_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_ctrl_T *H);
static void left_arm_ct_emxInit_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions);
static void left_arm_ctrl_ob_emxInit_char_T(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions);
static void l_emxEnsureCapacity_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel);
static void left_a_emxEnsureCapacity_char_T(emxArray_char_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel);
static void rigidBodyJoint_get_JointAxis_e(const rigidBodyJoint_left_arm_ctr_e_T
  *obj, real_T ax[3]);
static void left_arm_ctrl_ob_emxFree_char_T(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray);
static void RigidBodyTree_forwardKinematics(k_robotics_manip_internal_R_e_T *obj,
  const real_T qvec[7], emxArray_e_cell_wrap_left_arm_T *Ttree);
static void left_arm_ct_emxFree_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray);
static void left_arm_ctrl_SystemCore_step_e(boolean_T *varargout_1, real32_T
  varargout_2_Data[14], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void left_arm_ctrl_obs_eye(real_T b_I[36]);
static boolean_T left_arm_ctrl_obs_strcmp(const emxArray_char_T_left_arm_ctrl_T *
  a);
static void lef_GravityTorqueBlock_stepImpl(robotics_slmanip_internal__e0_T *obj,
  const real32_T q[7], real32_T jointTorq[7]);
static void left_arm_ct_emxInit_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions);
static void l_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel);
static void left_arm_ct_emxFree_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMatri(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_ctrl_T *H,
  emxArray_real_T_left_arm_ctrl_T *lambda);
static void RigidBodyTreeDynamics_inverseDy(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], const real_T qdot[7], const
  emxArray_real_T_left_arm_ctrl_T *qddot, const real_T fext[60], real_T tau[7]);
static void left_arm_ctrl_obs_atan2(const real_T y_data[], const int32_T y_size
  [3], const real_T x_data[], const int32_T x_size[3], real_T r_data[], int32_T
  r_size[3]);
static void matlabCodegenHandle_matlabC_e0h(ros_slros_internal_block_Subs_T *obj);
static void le_emxFreeStruct_rigidBodyJoint(rigidBodyJoint_left_arm_ctrl__T
  *pStruct);
static void emxFreeStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal_e0h_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_e0h_T
  *pStruct);
static void emxFreeStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct);
static void l_emxFreeStruct_rigidBodyJoint1(rigidBodyJoint_left_arm_ctr_e_T
  *pStruct);
static void emxFreeStruct_i_robotics_mani_e(i_robotics_manip_internal_R_e_T
  *pStruct);
static void emxFreeStruct_k_robotics_mani_e(k_robotics_manip_internal_R_e_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct);
static void emxFreeStruct_j_robotics_mani_e(j_robotics_manip_internal_R_e_T
  *pStruct);
static void emxFreeStruct_k_robotics_man_e0(k_robotics_manip_internal__e0_T
  *pStruct);
static void emxFreeStruct_robotics_slman_e0(robotics_slmanip_internal__e0_T
  *pStruct);
static void emxFreeStruct_k_robotics_ma_e0h(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slma_e0h(robotics_slmanip_internal_blo_T
  *pStruct);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void le_emxInitStruct_rigidBodyJoint(rigidBodyJoint_left_arm_ctrl__T
  *pStruct);
static void emxInitStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal_e0h_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_e0h_T
  *pStruct);
static void emxInitStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct);
static j_robotics_manip_internal_Rig_T *left_arm_ct_RigidBody_RigidBody
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left_arm__RigidBody_RigidBody_e
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left_arm_RigidBody_RigidBody_e0
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left_ar_RigidBody_RigidBody_e0h
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left_a_RigidBody_RigidBody_e0h4
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left__RigidBody_RigidBody_e0h4e
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left_RigidBody_RigidBody_e0h4ew
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *lef_RigidBody_RigidBody_e0h4ewm
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *le_RigidBody_RigidBody_e0h4ewmd
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *l_RigidBody_RigidBody_e0h4ewmdi
  (j_robotics_manip_internal_Rig_T *obj);
static i_robotics_manip_internal_Rig_T *RigidBody_RigidBody_e0h4ewmdid
  (i_robotics_manip_internal_Rig_T *obj);
static k_robotics_manip_internal_e0h_T *RigidBodyTree_RigidBodyTree_e0h
  (k_robotics_manip_internal_e0h_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9);
static void l_emxInitStruct_rigidBodyJoint1(rigidBodyJoint_left_arm_ctr_e_T
  *pStruct);
static void emxInitStruct_i_robotics_mani_e(i_robotics_manip_internal_R_e_T
  *pStruct);
static void emxInitStruct_k_robotics_mani_e(k_robotics_manip_internal_R_e_T
  *pStruct);
static void emxInitStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct);
static void emxInitStruct_j_robotics_mani_e(j_robotics_manip_internal_R_e_T
  *pStruct);
static j_robotics_manip_internal_R_e_T *RigidBody_RigidBody_e0h4ewmdidb
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *RigidBody_RigidBod_e0h4ewmdidbj
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *RigidBody_RigidBo_e0h4ewmdidbjt
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *RigidBody_RigidB_e0h4ewmdidbjtq
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *RigidBody_Rigid_e0h4ewmdidbjtqt
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *l_RigidBody_Rigid_e
  (j_robotics_manip_internal_R_e_T *obj);
static j_robotics_manip_internal_R_e_T *l_RigidBody_Rigid_i
  (j_robotics_manip_internal_R_e_T *obj);
static k_robotics_manip_internal_R_e_T *l_RigidBodyTree_RigidBodyTree_e
  (k_robotics_manip_internal_R_e_T *obj, j_robotics_manip_internal_R_e_T *iobj_0,
   j_robotics_manip_internal_R_e_T *iobj_1, j_robotics_manip_internal_R_e_T
   *iobj_2, j_robotics_manip_internal_R_e_T *iobj_3,
   j_robotics_manip_internal_R_e_T *iobj_4, j_robotics_manip_internal_R_e_T
   *iobj_5, j_robotics_manip_internal_R_e_T *iobj_6,
   j_robotics_manip_internal_R_e_T *iobj_7, j_robotics_manip_internal_R_e_T
   *iobj_8, j_robotics_manip_internal_R_e_T *iobj_9);
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
static void emxInitStruct_k_robotics_ma_e0h(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slma_e0h(robotics_slmanip_internal_blo_T
  *pStruct);
static k_robotics_manip_internal_Rig_T *lef_RigidBodyTree_RigidBodyTree
  (k_robotics_manip_internal_Rig_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9);
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
  int_T nXc = 49;
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

static void left_arm_ctrl_ob_emxInit_real_T(emxArray_real_T_left_arm_ctrl_T
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
  for (left_arm_ctrl_obs_B.i_nn = 0; left_arm_ctrl_obs_B.i_nn < numDimensions;
       left_arm_ctrl_obs_B.i_nn++) {
    emxArray->size[left_arm_ctrl_obs_B.i_nn] = 0;
  }
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
  for (left_arm_ctrl_obs_B.i_c = 0; left_arm_ctrl_obs_B.i_c < 7;
       left_arm_ctrl_obs_B.i_c++) {
    varargout_2_Data[left_arm_ctrl_obs_B.i_c] =
      left_arm_ctrl_obs_B.b_varargout_2.Data[left_arm_ctrl_obs_B.i_c];
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

static void left_a_emxEnsureCapacity_real_T(emxArray_real_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_ctrl_obs_B.newNumel = 1;
  for (left_arm_ctrl_obs_B.i_h = 0; left_arm_ctrl_obs_B.i_h <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_h++) {
    left_arm_ctrl_obs_B.newNumel *= emxArray->size[left_arm_ctrl_obs_B.i_h];
  }

  if (left_arm_ctrl_obs_B.newNumel > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_h = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_h < 16) {
      left_arm_ctrl_obs_B.i_h = 16;
    }

    while (left_arm_ctrl_obs_B.i_h < left_arm_ctrl_obs_B.newNumel) {
      if (left_arm_ctrl_obs_B.i_h > 1073741823) {
        left_arm_ctrl_obs_B.i_h = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_h <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_h), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_h;
    emxArray->canFreeData = true;
  }
}

static void left_arm_c_emxInit_e_cell_wrap1(emxArray_e_cell_wrap_left_a_e_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_e_cell_wrap_left_a_e_T *emxArray;
  *pEmxArray = (emxArray_e_cell_wrap_left_a_e_T *)malloc(sizeof
    (emxArray_e_cell_wrap_left_a_e_T));
  emxArray = *pEmxArray;
  emxArray->data = (e_cell_wrap_left_arm_ctrl_o_e_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (left_arm_ctrl_obs_B.i_nc = 0; left_arm_ctrl_obs_B.i_nc < numDimensions;
       left_arm_ctrl_obs_B.i_nc++) {
    emxArray->size[left_arm_ctrl_obs_B.i_nc] = 0;
  }
}

static void emxEnsureCapacity_e_cell_wrap1(emxArray_e_cell_wrap_left_a_e_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_ctrl_obs_B.newNumel_e = 1;
  for (left_arm_ctrl_obs_B.i_d = 0; left_arm_ctrl_obs_B.i_d <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_d++) {
    left_arm_ctrl_obs_B.newNumel_e *= emxArray->size[left_arm_ctrl_obs_B.i_d];
  }

  if (left_arm_ctrl_obs_B.newNumel_e > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_d = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_d < 16) {
      left_arm_ctrl_obs_B.i_d = 16;
    }

    while (left_arm_ctrl_obs_B.i_d < left_arm_ctrl_obs_B.newNumel_e) {
      if (left_arm_ctrl_obs_B.i_d > 1073741823) {
        left_arm_ctrl_obs_B.i_d = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_d <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_d), sizeof
                     (e_cell_wrap_left_arm_ctrl_o_e_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(e_cell_wrap_left_arm_ctrl_o_e_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (e_cell_wrap_left_arm_ctrl_o_e_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_d;
    emxArray->canFreeData = true;
  }
}

static void le_rigidBodyJoint_get_JointAxis(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (left_arm_ctrl_obs_B.b_kstr_m = 0; left_arm_ctrl_obs_B.b_kstr_m < 8;
       left_arm_ctrl_obs_B.b_kstr_m++) {
    left_arm_ctrl_obs_B.b_c[left_arm_ctrl_obs_B.b_kstr_m] =
      tmp[left_arm_ctrl_obs_B.b_kstr_m];
  }

  left_arm_ctrl_obs_B.b_bool_h = false;
  if (obj->Type->size[1] == 8) {
    left_arm_ctrl_obs_B.b_kstr_m = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_m - 1 < 8) {
        left_arm_ctrl_obs_B.kstr_i = left_arm_ctrl_obs_B.b_kstr_m - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr_i] !=
            left_arm_ctrl_obs_B.b_c[left_arm_ctrl_obs_B.kstr_i]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_m++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_h = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (left_arm_ctrl_obs_B.b_bool_h) {
    guard1 = true;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr_m = 0; left_arm_ctrl_obs_B.b_kstr_m < 9;
         left_arm_ctrl_obs_B.b_kstr_m++) {
      left_arm_ctrl_obs_B.b_o[left_arm_ctrl_obs_B.b_kstr_m] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr_m];
    }

    left_arm_ctrl_obs_B.b_bool_h = false;
    if (obj->Type->size[1] == 9) {
      left_arm_ctrl_obs_B.b_kstr_m = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_m - 1 < 9) {
          left_arm_ctrl_obs_B.kstr_i = left_arm_ctrl_obs_B.b_kstr_m - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr_i] !=
              left_arm_ctrl_obs_B.b_o[left_arm_ctrl_obs_B.kstr_i]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_m++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_h = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_h) {
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
  normRowMatrix[3])
{
  left_arm_ctrl_obs_B.b_g = 1.0 / sqrt((matrix[0] * matrix[0] + matrix[1] *
    matrix[1]) + matrix[2] * matrix[2]);
  normRowMatrix[0] = matrix[0] * left_arm_ctrl_obs_B.b_g;
  normRowMatrix[1] = matrix[1] * left_arm_ctrl_obs_B.b_g;
  normRowMatrix[2] = matrix[2] * left_arm_ctrl_obs_B.b_g;
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

static void rigidBodyJoint_transformBodyT_e(const
  rigidBodyJoint_left_arm_ctrl__T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (left_arm_ctrl_obs_B.b_kstr_n = 0; left_arm_ctrl_obs_B.b_kstr_n < 5;
       left_arm_ctrl_obs_B.b_kstr_n++) {
    left_arm_ctrl_obs_B.b_m4[left_arm_ctrl_obs_B.b_kstr_n] =
      tmp[left_arm_ctrl_obs_B.b_kstr_n];
  }

  left_arm_ctrl_obs_B.b_bool_c5 = false;
  if (obj->Type->size[1] == 5) {
    left_arm_ctrl_obs_B.b_kstr_n = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_n - 1 < 5) {
        left_arm_ctrl_obs_B.kstr_g = left_arm_ctrl_obs_B.b_kstr_n - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr_g] !=
            left_arm_ctrl_obs_B.b_m4[left_arm_ctrl_obs_B.kstr_g]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_n++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_c5 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (left_arm_ctrl_obs_B.b_bool_c5) {
    left_arm_ctrl_obs_B.b_kstr_n = 0;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr_n = 0; left_arm_ctrl_obs_B.b_kstr_n < 8;
         left_arm_ctrl_obs_B.b_kstr_n++) {
      left_arm_ctrl_obs_B.b_fm[left_arm_ctrl_obs_B.b_kstr_n] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr_n];
    }

    left_arm_ctrl_obs_B.b_bool_c5 = false;
    if (obj->Type->size[1] == 8) {
      left_arm_ctrl_obs_B.b_kstr_n = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_n - 1 < 8) {
          left_arm_ctrl_obs_B.kstr_g = left_arm_ctrl_obs_B.b_kstr_n - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr_g] !=
              left_arm_ctrl_obs_B.b_fm[left_arm_ctrl_obs_B.kstr_g]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_n++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_c5 = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_c5) {
      left_arm_ctrl_obs_B.b_kstr_n = 1;
    } else {
      left_arm_ctrl_obs_B.b_kstr_n = -1;
    }
  }

  switch (left_arm_ctrl_obs_B.b_kstr_n) {
   case 0:
    memset(&left_arm_ctrl_obs_B.TJ_f[0], 0, sizeof(real_T) << 4U);
    left_arm_ctrl_obs_B.TJ_f[0] = 1.0;
    left_arm_ctrl_obs_B.TJ_f[5] = 1.0;
    left_arm_ctrl_obs_B.TJ_f[10] = 1.0;
    left_arm_ctrl_obs_B.TJ_f[15] = 1.0;
    break;

   case 1:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_ctrl_obs_B.v_as);
    left_arm_ctrl_obs_B.result_data_p[0] = left_arm_ctrl_obs_B.v_as[0];
    left_arm_ctrl_obs_B.result_data_p[1] = left_arm_ctrl_obs_B.v_as[1];
    left_arm_ctrl_obs_B.result_data_p[2] = left_arm_ctrl_obs_B.v_as[2];
    if (0 <= (*q_size != 0) - 1) {
      left_arm_ctrl_obs_B.result_data_p[3] = q_data[0];
    }

    left_arm_ctrl_obs_normalizeRows(&left_arm_ctrl_obs_B.result_data_p[0],
      left_arm_ctrl_obs_B.v_as);
    left_arm_ctrl_obs_B.cth = cos(left_arm_ctrl_obs_B.result_data_p[3]);
    left_arm_ctrl_obs_B.sth_a = sin(left_arm_ctrl_obs_B.result_data_p[3]);
    left_arm_ctrl_obs_B.tempR_tmp_j = left_arm_ctrl_obs_B.v_as[1] *
      left_arm_ctrl_obs_B.v_as[0] * (1.0 - left_arm_ctrl_obs_B.cth);
    left_arm_ctrl_obs_B.tempR_tmp_e = left_arm_ctrl_obs_B.v_as[2] *
      left_arm_ctrl_obs_B.sth_a;
    left_arm_ctrl_obs_B.tempR_tmp_o = left_arm_ctrl_obs_B.v_as[2] *
      left_arm_ctrl_obs_B.v_as[0] * (1.0 - left_arm_ctrl_obs_B.cth);
    left_arm_ctrl_obs_B.tempR_tmp_b = left_arm_ctrl_obs_B.v_as[1] *
      left_arm_ctrl_obs_B.sth_a;
    left_arm_ctrl_obs_B.tempR_tmp_a = left_arm_ctrl_obs_B.v_as[2] *
      left_arm_ctrl_obs_B.v_as[1] * (1.0 - left_arm_ctrl_obs_B.cth);
    left_arm_ctrl_obs_B.sth_a *= left_arm_ctrl_obs_B.v_as[0];
    left_arm_ctrl_obs_cat(left_arm_ctrl_obs_B.v_as[0] *
                          left_arm_ctrl_obs_B.v_as[0] * (1.0 -
      left_arm_ctrl_obs_B.cth) + left_arm_ctrl_obs_B.cth,
                          left_arm_ctrl_obs_B.tempR_tmp_j -
                          left_arm_ctrl_obs_B.tempR_tmp_e,
                          left_arm_ctrl_obs_B.tempR_tmp_o +
                          left_arm_ctrl_obs_B.tempR_tmp_b,
                          left_arm_ctrl_obs_B.tempR_tmp_j +
                          left_arm_ctrl_obs_B.tempR_tmp_e,
                          left_arm_ctrl_obs_B.v_as[1] *
                          left_arm_ctrl_obs_B.v_as[1] * (1.0 -
      left_arm_ctrl_obs_B.cth) + left_arm_ctrl_obs_B.cth,
                          left_arm_ctrl_obs_B.tempR_tmp_a -
                          left_arm_ctrl_obs_B.sth_a,
                          left_arm_ctrl_obs_B.tempR_tmp_o -
                          left_arm_ctrl_obs_B.tempR_tmp_b,
                          left_arm_ctrl_obs_B.tempR_tmp_a +
                          left_arm_ctrl_obs_B.sth_a, left_arm_ctrl_obs_B.v_as[2]
                          * left_arm_ctrl_obs_B.v_as[2] * (1.0 -
      left_arm_ctrl_obs_B.cth) + left_arm_ctrl_obs_B.cth,
                          left_arm_ctrl_obs_B.tempR_e);
    for (left_arm_ctrl_obs_B.b_kstr_n = 0; left_arm_ctrl_obs_B.b_kstr_n < 3;
         left_arm_ctrl_obs_B.b_kstr_n++) {
      left_arm_ctrl_obs_B.kstr_g = left_arm_ctrl_obs_B.b_kstr_n + 1;
      left_arm_ctrl_obs_B.R_d[left_arm_ctrl_obs_B.kstr_g - 1] =
        left_arm_ctrl_obs_B.tempR_e[(left_arm_ctrl_obs_B.kstr_g - 1) * 3];
      left_arm_ctrl_obs_B.kstr_g = left_arm_ctrl_obs_B.b_kstr_n + 1;
      left_arm_ctrl_obs_B.R_d[left_arm_ctrl_obs_B.kstr_g + 2] =
        left_arm_ctrl_obs_B.tempR_e[(left_arm_ctrl_obs_B.kstr_g - 1) * 3 + 1];
      left_arm_ctrl_obs_B.kstr_g = left_arm_ctrl_obs_B.b_kstr_n + 1;
      left_arm_ctrl_obs_B.R_d[left_arm_ctrl_obs_B.kstr_g + 5] =
        left_arm_ctrl_obs_B.tempR_e[(left_arm_ctrl_obs_B.kstr_g - 1) * 3 + 2];
    }

    memset(&left_arm_ctrl_obs_B.TJ_f[0], 0, sizeof(real_T) << 4U);
    for (left_arm_ctrl_obs_B.b_kstr_n = 0; left_arm_ctrl_obs_B.b_kstr_n < 3;
         left_arm_ctrl_obs_B.b_kstr_n++) {
      left_arm_ctrl_obs_B.kstr_g = left_arm_ctrl_obs_B.b_kstr_n << 2;
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_g] =
        left_arm_ctrl_obs_B.R_d[3 * left_arm_ctrl_obs_B.b_kstr_n];
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_g + 1] =
        left_arm_ctrl_obs_B.R_d[3 * left_arm_ctrl_obs_B.b_kstr_n + 1];
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_g + 2] =
        left_arm_ctrl_obs_B.R_d[3 * left_arm_ctrl_obs_B.b_kstr_n + 2];
    }

    left_arm_ctrl_obs_B.TJ_f[15] = 1.0;
    break;

   default:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_ctrl_obs_B.v_as);
    memset(&left_arm_ctrl_obs_B.tempR_e[0], 0, 9U * sizeof(real_T));
    left_arm_ctrl_obs_B.tempR_e[0] = 1.0;
    left_arm_ctrl_obs_B.tempR_e[4] = 1.0;
    left_arm_ctrl_obs_B.tempR_e[8] = 1.0;
    for (left_arm_ctrl_obs_B.b_kstr_n = 0; left_arm_ctrl_obs_B.b_kstr_n < 3;
         left_arm_ctrl_obs_B.b_kstr_n++) {
      left_arm_ctrl_obs_B.kstr_g = left_arm_ctrl_obs_B.b_kstr_n << 2;
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_g] =
        left_arm_ctrl_obs_B.tempR_e[3 * left_arm_ctrl_obs_B.b_kstr_n];
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_g + 1] =
        left_arm_ctrl_obs_B.tempR_e[3 * left_arm_ctrl_obs_B.b_kstr_n + 1];
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.kstr_g + 2] =
        left_arm_ctrl_obs_B.tempR_e[3 * left_arm_ctrl_obs_B.b_kstr_n + 2];
      left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.b_kstr_n + 12] =
        left_arm_ctrl_obs_B.v_as[left_arm_ctrl_obs_B.b_kstr_n] * q_data[0];
    }

    left_arm_ctrl_obs_B.TJ_f[3] = 0.0;
    left_arm_ctrl_obs_B.TJ_f[7] = 0.0;
    left_arm_ctrl_obs_B.TJ_f[11] = 0.0;
    left_arm_ctrl_obs_B.TJ_f[15] = 1.0;
    break;
  }

  for (left_arm_ctrl_obs_B.b_kstr_n = 0; left_arm_ctrl_obs_B.b_kstr_n < 4;
       left_arm_ctrl_obs_B.b_kstr_n++) {
    for (left_arm_ctrl_obs_B.kstr_g = 0; left_arm_ctrl_obs_B.kstr_g < 4;
         left_arm_ctrl_obs_B.kstr_g++) {
      left_arm_ctrl_obs_B.obj_tmp_tmp_n = left_arm_ctrl_obs_B.kstr_g << 2;
      left_arm_ctrl_obs_B.obj_tmp_d = left_arm_ctrl_obs_B.b_kstr_n +
        left_arm_ctrl_obs_B.obj_tmp_tmp_n;
      left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.obj_tmp_d] = 0.0;
      left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.obj_tmp_d] +=
        left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.obj_tmp_tmp_n] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_n];
      left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.obj_tmp_d] +=
        left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.obj_tmp_tmp_n + 1] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_n + 4];
      left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.obj_tmp_d] +=
        left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.obj_tmp_tmp_n + 2] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_n + 8];
      left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.obj_tmp_d] +=
        left_arm_ctrl_obs_B.TJ_f[left_arm_ctrl_obs_B.obj_tmp_tmp_n + 3] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_n + 12];
    }

    for (left_arm_ctrl_obs_B.kstr_g = 0; left_arm_ctrl_obs_B.kstr_g < 4;
         left_arm_ctrl_obs_B.kstr_g++) {
      left_arm_ctrl_obs_B.obj_tmp_tmp_n = left_arm_ctrl_obs_B.kstr_g << 2;
      left_arm_ctrl_obs_B.obj_tmp_d = left_arm_ctrl_obs_B.b_kstr_n +
        left_arm_ctrl_obs_B.obj_tmp_tmp_n;
      T[left_arm_ctrl_obs_B.obj_tmp_d] = 0.0;
      T[left_arm_ctrl_obs_B.obj_tmp_d] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_n] *
        left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.b_kstr_n];
      T[left_arm_ctrl_obs_B.obj_tmp_d] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_n + 1] *
        left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.b_kstr_n + 4];
      T[left_arm_ctrl_obs_B.obj_tmp_d] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_n + 2] *
        left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.b_kstr_n + 8];
      T[left_arm_ctrl_obs_B.obj_tmp_d] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_n + 3] *
        left_arm_ctrl_obs_B.obj_g[left_arm_ctrl_obs_B.b_kstr_n + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const
  rigidBodyJoint_left_arm_ctrl__T *obj, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 5;
       left_arm_ctrl_obs_B.b_kstr++) {
    left_arm_ctrl_obs_B.b_ie[left_arm_ctrl_obs_B.b_kstr] =
      tmp[left_arm_ctrl_obs_B.b_kstr];
  }

  left_arm_ctrl_obs_B.b_bool_c = false;
  if (obj->Type->size[1] == 5) {
    left_arm_ctrl_obs_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr - 1 < 5) {
        left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr] !=
            left_arm_ctrl_obs_B.b_ie[left_arm_ctrl_obs_B.kstr]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_c = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (left_arm_ctrl_obs_B.b_bool_c) {
    left_arm_ctrl_obs_B.b_kstr = 0;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 8;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.b_mj[left_arm_ctrl_obs_B.b_kstr] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr];
    }

    left_arm_ctrl_obs_B.b_bool_c = false;
    if (obj->Type->size[1] == 8) {
      left_arm_ctrl_obs_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr - 1 < 8) {
          left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr] !=
              left_arm_ctrl_obs_B.b_mj[left_arm_ctrl_obs_B.kstr]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_c) {
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
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_ctrl_obs_B.v_e);
    left_arm_ctrl_obs_B.v_a[0] = left_arm_ctrl_obs_B.v_e[0];
    left_arm_ctrl_obs_B.v_a[1] = left_arm_ctrl_obs_B.v_e[1];
    left_arm_ctrl_obs_B.v_a[2] = left_arm_ctrl_obs_B.v_e[2];
    left_arm_ctrl_obs_normalizeRows(left_arm_ctrl_obs_B.v_a,
      left_arm_ctrl_obs_B.v_e);
    left_arm_ctrl_obs_B.tempR_tmp_m = left_arm_ctrl_obs_B.v_e[1] *
      left_arm_ctrl_obs_B.v_e[0] * 0.0;
    left_arm_ctrl_obs_B.tempR_tmp_mc = left_arm_ctrl_obs_B.v_e[2] *
      left_arm_ctrl_obs_B.v_e[0] * 0.0;
    left_arm_ctrl_obs_B.tempR_tmp_h3 = left_arm_ctrl_obs_B.v_e[2] *
      left_arm_ctrl_obs_B.v_e[1] * 0.0;
    left_arm_ctrl_obs_cat(left_arm_ctrl_obs_B.v_e[0] * left_arm_ctrl_obs_B.v_e[0]
                          * 0.0 + 1.0, left_arm_ctrl_obs_B.tempR_tmp_m -
                          left_arm_ctrl_obs_B.v_e[2] * 0.0,
                          left_arm_ctrl_obs_B.tempR_tmp_mc +
                          left_arm_ctrl_obs_B.v_e[1] * 0.0,
                          left_arm_ctrl_obs_B.tempR_tmp_m +
                          left_arm_ctrl_obs_B.v_e[2] * 0.0,
                          left_arm_ctrl_obs_B.v_e[1] * left_arm_ctrl_obs_B.v_e[1]
                          * 0.0 + 1.0, left_arm_ctrl_obs_B.tempR_tmp_h3 -
                          left_arm_ctrl_obs_B.v_e[0] * 0.0,
                          left_arm_ctrl_obs_B.tempR_tmp_mc -
                          left_arm_ctrl_obs_B.v_e[1] * 0.0,
                          left_arm_ctrl_obs_B.tempR_tmp_h3 +
                          left_arm_ctrl_obs_B.v_e[0] * 0.0,
                          left_arm_ctrl_obs_B.v_e[2] * left_arm_ctrl_obs_B.v_e[2]
                          * 0.0 + 1.0, left_arm_ctrl_obs_B.tempR_l);
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 3;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr + 1;
      left_arm_ctrl_obs_B.R_bs[left_arm_ctrl_obs_B.kstr - 1] =
        left_arm_ctrl_obs_B.tempR_l[(left_arm_ctrl_obs_B.kstr - 1) * 3];
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr + 1;
      left_arm_ctrl_obs_B.R_bs[left_arm_ctrl_obs_B.kstr + 2] =
        left_arm_ctrl_obs_B.tempR_l[(left_arm_ctrl_obs_B.kstr - 1) * 3 + 1];
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr + 1;
      left_arm_ctrl_obs_B.R_bs[left_arm_ctrl_obs_B.kstr + 5] =
        left_arm_ctrl_obs_B.tempR_l[(left_arm_ctrl_obs_B.kstr - 1) * 3 + 2];
    }

    memset(&left_arm_ctrl_obs_B.TJ_p[0], 0, sizeof(real_T) << 4U);
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 3;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr << 2;
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr] =
        left_arm_ctrl_obs_B.R_bs[3 * left_arm_ctrl_obs_B.b_kstr];
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr + 1] =
        left_arm_ctrl_obs_B.R_bs[3 * left_arm_ctrl_obs_B.b_kstr + 1];
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr + 2] =
        left_arm_ctrl_obs_B.R_bs[3 * left_arm_ctrl_obs_B.b_kstr + 2];
    }

    left_arm_ctrl_obs_B.TJ_p[15] = 1.0;
    break;

   default:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_ctrl_obs_B.v_e);
    memset(&left_arm_ctrl_obs_B.tempR_l[0], 0, 9U * sizeof(real_T));
    left_arm_ctrl_obs_B.tempR_l[0] = 1.0;
    left_arm_ctrl_obs_B.tempR_l[4] = 1.0;
    left_arm_ctrl_obs_B.tempR_l[8] = 1.0;
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 3;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr << 2;
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr] =
        left_arm_ctrl_obs_B.tempR_l[3 * left_arm_ctrl_obs_B.b_kstr];
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr + 1] =
        left_arm_ctrl_obs_B.tempR_l[3 * left_arm_ctrl_obs_B.b_kstr + 1];
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.kstr + 2] =
        left_arm_ctrl_obs_B.tempR_l[3 * left_arm_ctrl_obs_B.b_kstr + 2];
      left_arm_ctrl_obs_B.TJ_p[left_arm_ctrl_obs_B.b_kstr + 12] =
        left_arm_ctrl_obs_B.v_e[left_arm_ctrl_obs_B.b_kstr] * 0.0;
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

static void left_arm_ctrl_obs_tforminv(const real_T T[16], real_T Tinv[16])
{
  for (left_arm_ctrl_obs_B.i3 = 0; left_arm_ctrl_obs_B.i3 < 3;
       left_arm_ctrl_obs_B.i3++) {
    left_arm_ctrl_obs_B.R_f[3 * left_arm_ctrl_obs_B.i3] =
      T[left_arm_ctrl_obs_B.i3];
    left_arm_ctrl_obs_B.R_f[3 * left_arm_ctrl_obs_B.i3 + 1] =
      T[left_arm_ctrl_obs_B.i3 + 4];
    left_arm_ctrl_obs_B.R_f[3 * left_arm_ctrl_obs_B.i3 + 2] =
      T[left_arm_ctrl_obs_B.i3 + 8];
  }

  for (left_arm_ctrl_obs_B.i3 = 0; left_arm_ctrl_obs_B.i3 < 9;
       left_arm_ctrl_obs_B.i3++) {
    left_arm_ctrl_obs_B.R_a[left_arm_ctrl_obs_B.i3] =
      -left_arm_ctrl_obs_B.R_f[left_arm_ctrl_obs_B.i3];
  }

  for (left_arm_ctrl_obs_B.i3 = 0; left_arm_ctrl_obs_B.i3 < 3;
       left_arm_ctrl_obs_B.i3++) {
    left_arm_ctrl_obs_B.Tinv_tmp = left_arm_ctrl_obs_B.i3 << 2;
    Tinv[left_arm_ctrl_obs_B.Tinv_tmp] = left_arm_ctrl_obs_B.R_f[3 *
      left_arm_ctrl_obs_B.i3];
    Tinv[left_arm_ctrl_obs_B.Tinv_tmp + 1] = left_arm_ctrl_obs_B.R_f[3 *
      left_arm_ctrl_obs_B.i3 + 1];
    Tinv[left_arm_ctrl_obs_B.Tinv_tmp + 2] = left_arm_ctrl_obs_B.R_f[3 *
      left_arm_ctrl_obs_B.i3 + 2];
    Tinv[left_arm_ctrl_obs_B.i3 + 12] =
      left_arm_ctrl_obs_B.R_a[left_arm_ctrl_obs_B.i3 + 6] * T[14] +
      (left_arm_ctrl_obs_B.R_a[left_arm_ctrl_obs_B.i3 + 3] * T[13] +
       left_arm_ctrl_obs_B.R_a[left_arm_ctrl_obs_B.i3] * T[12]);
  }

  Tinv[3] = 0.0;
  Tinv[7] = 0.0;
  Tinv[11] = 0.0;
  Tinv[15] = 1.0;
}

static void left_arm_ct_tformToSpatialXform(const real_T T[16], real_T X[36])
{
  left_arm_ctrl_obs_B.dv6[0] = 0.0;
  left_arm_ctrl_obs_B.dv6[3] = -T[14];
  left_arm_ctrl_obs_B.dv6[6] = T[13];
  left_arm_ctrl_obs_B.dv6[1] = T[14];
  left_arm_ctrl_obs_B.dv6[4] = 0.0;
  left_arm_ctrl_obs_B.dv6[7] = -T[12];
  left_arm_ctrl_obs_B.dv6[2] = -T[13];
  left_arm_ctrl_obs_B.dv6[5] = T[12];
  left_arm_ctrl_obs_B.dv6[8] = 0.0;
  for (left_arm_ctrl_obs_B.i1 = 0; left_arm_ctrl_obs_B.i1 < 3;
       left_arm_ctrl_obs_B.i1++) {
    for (left_arm_ctrl_obs_B.X_tmp = 0; left_arm_ctrl_obs_B.X_tmp < 3;
         left_arm_ctrl_obs_B.X_tmp++) {
      left_arm_ctrl_obs_B.X_tmp_c = left_arm_ctrl_obs_B.i1 + 3 *
        left_arm_ctrl_obs_B.X_tmp;
      left_arm_ctrl_obs_B.dv7[left_arm_ctrl_obs_B.X_tmp_c] = 0.0;
      left_arm_ctrl_obs_B.i2 = left_arm_ctrl_obs_B.X_tmp << 2;
      left_arm_ctrl_obs_B.dv7[left_arm_ctrl_obs_B.X_tmp_c] +=
        T[left_arm_ctrl_obs_B.i2] *
        left_arm_ctrl_obs_B.dv6[left_arm_ctrl_obs_B.i1];
      left_arm_ctrl_obs_B.dv7[left_arm_ctrl_obs_B.X_tmp_c] +=
        T[left_arm_ctrl_obs_B.i2 + 1] *
        left_arm_ctrl_obs_B.dv6[left_arm_ctrl_obs_B.i1 + 3];
      left_arm_ctrl_obs_B.dv7[left_arm_ctrl_obs_B.X_tmp_c] +=
        T[left_arm_ctrl_obs_B.i2 + 2] *
        left_arm_ctrl_obs_B.dv6[left_arm_ctrl_obs_B.i1 + 6];
      X[left_arm_ctrl_obs_B.X_tmp + 6 * left_arm_ctrl_obs_B.i1] = T
        [(left_arm_ctrl_obs_B.i1 << 2) + left_arm_ctrl_obs_B.X_tmp];
      X[left_arm_ctrl_obs_B.X_tmp + 6 * (left_arm_ctrl_obs_B.i1 + 3)] = 0.0;
    }
  }

  for (left_arm_ctrl_obs_B.i1 = 0; left_arm_ctrl_obs_B.i1 < 3;
       left_arm_ctrl_obs_B.i1++) {
    X[6 * left_arm_ctrl_obs_B.i1 + 3] = left_arm_ctrl_obs_B.dv7[3 *
      left_arm_ctrl_obs_B.i1];
    left_arm_ctrl_obs_B.X_tmp = left_arm_ctrl_obs_B.i1 << 2;
    left_arm_ctrl_obs_B.X_tmp_c = 6 * (left_arm_ctrl_obs_B.i1 + 3);
    X[left_arm_ctrl_obs_B.X_tmp_c + 3] = T[left_arm_ctrl_obs_B.X_tmp];
    X[6 * left_arm_ctrl_obs_B.i1 + 4] = left_arm_ctrl_obs_B.dv7[3 *
      left_arm_ctrl_obs_B.i1 + 1];
    X[left_arm_ctrl_obs_B.X_tmp_c + 4] = T[left_arm_ctrl_obs_B.X_tmp + 1];
    X[6 * left_arm_ctrl_obs_B.i1 + 5] = left_arm_ctrl_obs_B.dv7[3 *
      left_arm_ctrl_obs_B.i1 + 2];
    X[left_arm_ctrl_obs_B.X_tmp_c + 5] = T[left_arm_ctrl_obs_B.X_tmp + 2];
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

static void left_arm_c_emxFree_e_cell_wrap1(emxArray_e_cell_wrap_left_a_e_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_e_cell_wrap_left_a_e_T *)NULL) {
    if (((*pEmxArray)->data != (e_cell_wrap_left_arm_ctrl_o_e_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_e_cell_wrap_left_a_e_T *)NULL;
  }
}

static void RigidBodyTreeDynamics_massMat_e(k_robotics_manip_internal_e0h_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_ctrl_T *H)
{
  emxArray_e_cell_wrap_left_a_e_T *Ic;
  emxArray_e_cell_wrap_left_a_e_T *X;
  emxArray_real_T_left_arm_ctrl_T *Si;
  emxArray_real_T_left_arm_ctrl_T *Fi;
  emxArray_real_T_left_arm_ctrl_T *Sj;
  emxArray_real_T_left_arm_ctrl_T *Hji;
  j_robotics_manip_internal_Rig_T *obj;
  emxArray_real_T_left_arm_ctrl_T *a;
  emxArray_real_T_left_arm_ctrl_T *B;
  left_arm_ctrl_obs_B.nb_m = robot->NumBodies;
  left_arm_ctrl_obs_B.vNum_o = robot->VelocityNumber;
  left_arm_ctrl_obs_B.f = H->size[0] * H->size[1];
  left_arm_ctrl_obs_B.b_i_pr = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum_o);
  H->size[0] = left_arm_ctrl_obs_B.b_i_pr;
  H->size[1] = left_arm_ctrl_obs_B.b_i_pr;
  left_a_emxEnsureCapacity_real_T(H, left_arm_ctrl_obs_B.f);
  left_arm_ctrl_obs_B.n_d = left_arm_ctrl_obs_B.b_i_pr *
    left_arm_ctrl_obs_B.b_i_pr - 1;
  for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
       left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.f++) {
    H->data[left_arm_ctrl_obs_B.f] = 0.0;
  }

  left_arm_c_emxInit_e_cell_wrap1(&Ic, 2);
  left_arm_c_emxInit_e_cell_wrap1(&X, 2);
  left_arm_ctrl_obs_B.c_tmp = static_cast<int32_T>(left_arm_ctrl_obs_B.nb_m);
  left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.c_tmp - 1;
  left_arm_ctrl_obs_B.f = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = left_arm_ctrl_obs_B.c_tmp;
  emxEnsureCapacity_e_cell_wrap1(Ic, left_arm_ctrl_obs_B.f);
  left_arm_ctrl_obs_B.f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_ctrl_obs_B.c_tmp;
  emxEnsureCapacity_e_cell_wrap1(X, left_arm_ctrl_obs_B.f);
  for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr <=
       left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.b_i_pr++) {
    for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 36;
         left_arm_ctrl_obs_B.f++) {
      Ic->data[left_arm_ctrl_obs_B.b_i_pr].f1[left_arm_ctrl_obs_B.f] =
        robot->Bodies[left_arm_ctrl_obs_B.b_i_pr]->
        SpatialInertia[left_arm_ctrl_obs_B.f];
    }

    left_arm_ctrl_obs_B.vNum_o = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.b_i_pr];
    left_arm_ctrl_obs_B.p_idx_1_p = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.b_i_pr + 10];
    if (left_arm_ctrl_obs_B.p_idx_1_p < left_arm_ctrl_obs_B.vNum_o) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.b_i_pr];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        left_arm_ctrl_obs_B.T_l);
    } else {
      if (left_arm_ctrl_obs_B.vNum_o > left_arm_ctrl_obs_B.p_idx_1_p) {
        left_arm_ctrl_obs_B.c_tmp = 0;
        left_arm_ctrl_obs_B.f = -1;
      } else {
        left_arm_ctrl_obs_B.c_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.vNum_o) - 1;
        left_arm_ctrl_obs_B.f = static_cast<int32_T>
          (left_arm_ctrl_obs_B.p_idx_1_p) - 1;
      }

      obj = robot->Bodies[left_arm_ctrl_obs_B.b_i_pr];
      left_arm_ctrl_obs_B.q_size_tmp_j = left_arm_ctrl_obs_B.f -
        left_arm_ctrl_obs_B.c_tmp;
      left_arm_ctrl_obs_B.q_size_f = left_arm_ctrl_obs_B.q_size_tmp_j + 1;
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
           left_arm_ctrl_obs_B.q_size_tmp_j; left_arm_ctrl_obs_B.f++) {
        left_arm_ctrl_obs_B.q_data_m[left_arm_ctrl_obs_B.f] =
          q[left_arm_ctrl_obs_B.c_tmp + left_arm_ctrl_obs_B.f];
      }

      rigidBodyJoint_transformBodyT_e(&obj->JointInternal,
        left_arm_ctrl_obs_B.q_data_m, &left_arm_ctrl_obs_B.q_size_f,
        left_arm_ctrl_obs_B.T_l);
    }

    left_arm_ctrl_obs_tforminv(left_arm_ctrl_obs_B.T_l, left_arm_ctrl_obs_B.dv1);
    left_arm_ct_tformToSpatialXform(left_arm_ctrl_obs_B.dv1, X->
      data[left_arm_ctrl_obs_B.b_i_pr].f1);
  }

  left_arm_ctrl_obs_B.c = static_cast<int32_T>(((-1.0 - left_arm_ctrl_obs_B.nb_m)
    + 1.0) / -1.0) - 1;
  left_arm_ctrl_ob_emxInit_real_T(&Si, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Fi, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Sj, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Hji, 2);
  left_arm_ctrl_ob_emxInit_real_T(&a, 2);
  left_arm_ctrl_ob_emxInit_real_T(&B, 2);
  for (left_arm_ctrl_obs_B.c_tmp = 0; left_arm_ctrl_obs_B.c_tmp <=
       left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.c_tmp++) {
    left_arm_ctrl_obs_B.pid_tmp_d = static_cast<int32_T>
      (left_arm_ctrl_obs_B.nb_m + -static_cast<real_T>(left_arm_ctrl_obs_B.c_tmp));
    left_arm_ctrl_obs_B.q_size_tmp_j = left_arm_ctrl_obs_B.pid_tmp_d - 1;
    left_arm_ctrl_obs_B.pid_n = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp_j
      ]->ParentIndex;
    left_arm_ctrl_obs_B.vNum_o = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_d - 1];
    left_arm_ctrl_obs_B.p_idx_1_p = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_d + 9];
    if (left_arm_ctrl_obs_B.pid_n > 0.0) {
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
             left_arm_ctrl_obs_B.b_i_pr++) {
          left_arm_ctrl_obs_B.X_tmp_i = left_arm_ctrl_obs_B.f + 6 *
            left_arm_ctrl_obs_B.b_i_pr;
          left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.X_tmp_i] = 0.0;
          for (left_arm_ctrl_obs_B.n_d = 0; left_arm_ctrl_obs_B.n_d < 6;
               left_arm_ctrl_obs_B.n_d++) {
            left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.X_tmp_i] += X->
              data[left_arm_ctrl_obs_B.q_size_tmp_j].f1[6 *
              left_arm_ctrl_obs_B.f + left_arm_ctrl_obs_B.n_d] * Ic->
              data[left_arm_ctrl_obs_B.q_size_tmp_j].f1[6 *
              left_arm_ctrl_obs_B.b_i_pr + left_arm_ctrl_obs_B.n_d];
          }
        }
      }

      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
             left_arm_ctrl_obs_B.b_i_pr++) {
          left_arm_ctrl_obs_B.b_idx_0_pt = 0.0;
          for (left_arm_ctrl_obs_B.n_d = 0; left_arm_ctrl_obs_B.n_d < 6;
               left_arm_ctrl_obs_B.n_d++) {
            left_arm_ctrl_obs_B.b_idx_0_pt += left_arm_ctrl_obs_B.X_k[6 *
              left_arm_ctrl_obs_B.n_d + left_arm_ctrl_obs_B.f] * X->
              data[left_arm_ctrl_obs_B.q_size_tmp_j].f1[6 *
              left_arm_ctrl_obs_B.b_i_pr + left_arm_ctrl_obs_B.n_d];
          }

          left_arm_ctrl_obs_B.n_d = 6 * left_arm_ctrl_obs_B.b_i_pr +
            left_arm_ctrl_obs_B.f;
          Ic->data[static_cast<int32_T>(left_arm_ctrl_obs_B.pid_n) - 1]
            .f1[left_arm_ctrl_obs_B.n_d] += left_arm_ctrl_obs_B.b_idx_0_pt;
        }
      }
    }

    left_arm_ctrl_obs_B.b_idx_0_pt = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_d - 1];
    left_arm_ctrl_obs_B.b_idx_1_f = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_d + 9];
    if (left_arm_ctrl_obs_B.b_idx_0_pt <= left_arm_ctrl_obs_B.b_idx_1_f) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp_j];
      left_arm_ctrl_obs_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(Si, left_arm_ctrl_obs_B.f);
      left_arm_ctrl_obs_B.n_d = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
           left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.f++) {
        Si->data[left_arm_ctrl_obs_B.f] = obj->
          JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.f];
      }

      left_arm_ctrl_obs_B.n_d = Si->size[1] - 1;
      left_arm_ctrl_obs_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.b_j_c = 0; left_arm_ctrl_obs_B.b_j_c <=
           left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.b_j_c++) {
        left_arm_ctrl_obs_B.pid_tmp_d = left_arm_ctrl_obs_B.b_j_c * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
             left_arm_ctrl_obs_B.b_i_pr++) {
          Fi->data[(left_arm_ctrl_obs_B.pid_tmp_d + left_arm_ctrl_obs_B.b_i_pr)
            + 1] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
             left_arm_ctrl_obs_B.b_i_pr++) {
          left_arm_ctrl_obs_B.aoffset_g = left_arm_ctrl_obs_B.b_i_pr * 6 - 1;
          left_arm_ctrl_obs_B.temp_l = Si->data[(left_arm_ctrl_obs_B.pid_tmp_d +
            left_arm_ctrl_obs_B.b_i_pr) + 1];
          for (left_arm_ctrl_obs_B.c_i_i = 0; left_arm_ctrl_obs_B.c_i_i < 6;
               left_arm_ctrl_obs_B.c_i_i++) {
            left_arm_ctrl_obs_B.i_c3 = left_arm_ctrl_obs_B.c_i_i + 1;
            left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.pid_tmp_d +
              left_arm_ctrl_obs_B.i_c3;
            Fi->data[left_arm_ctrl_obs_B.f] += Ic->
              data[left_arm_ctrl_obs_B.q_size_tmp_j]
              .f1[left_arm_ctrl_obs_B.aoffset_g + left_arm_ctrl_obs_B.i_c3] *
              left_arm_ctrl_obs_B.temp_l;
          }
        }
      }

      if (left_arm_ctrl_obs_B.vNum_o > left_arm_ctrl_obs_B.p_idx_1_p) {
        left_arm_ctrl_obs_B.pid_tmp_d = 0;
        left_arm_ctrl_obs_B.X_tmp_i = 0;
      } else {
        left_arm_ctrl_obs_B.pid_tmp_d = static_cast<int32_T>
          (left_arm_ctrl_obs_B.vNum_o) - 1;
        left_arm_ctrl_obs_B.X_tmp_i = left_arm_ctrl_obs_B.pid_tmp_d;
      }

      left_arm_ctrl_obs_B.f = a->size[0] * a->size[1];
      a->size[0] = Si->size[1];
      a->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        left_arm_ctrl_obs_B.n_d = Si->size[1];
        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr <
             left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.b_i_pr++) {
          a->data[left_arm_ctrl_obs_B.b_i_pr + a->size[0] *
            left_arm_ctrl_obs_B.f] = Si->data[6 * left_arm_ctrl_obs_B.b_i_pr +
            left_arm_ctrl_obs_B.f];
        }
      }

      left_arm_ctrl_obs_B.m_d = a->size[0];
      left_arm_ctrl_obs_B.n_d = Fi->size[1] - 1;
      left_arm_ctrl_obs_B.f = Hji->size[0] * Hji->size[1];
      Hji->size[0] = a->size[0];
      Hji->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(Hji, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.b_j_c = 0; left_arm_ctrl_obs_B.b_j_c <=
           left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.b_j_c++) {
        left_arm_ctrl_obs_B.coffset_g = left_arm_ctrl_obs_B.b_j_c *
          left_arm_ctrl_obs_B.m_d - 1;
        left_arm_ctrl_obs_B.boffset_l = left_arm_ctrl_obs_B.b_j_c * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr <
             left_arm_ctrl_obs_B.m_d; left_arm_ctrl_obs_B.b_i_pr++) {
          Hji->data[(left_arm_ctrl_obs_B.coffset_g + left_arm_ctrl_obs_B.b_i_pr)
            + 1] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
             left_arm_ctrl_obs_B.b_i_pr++) {
          left_arm_ctrl_obs_B.aoffset_g = left_arm_ctrl_obs_B.b_i_pr *
            left_arm_ctrl_obs_B.m_d - 1;
          left_arm_ctrl_obs_B.temp_l = Fi->data[(left_arm_ctrl_obs_B.boffset_l +
            left_arm_ctrl_obs_B.b_i_pr) + 1];
          for (left_arm_ctrl_obs_B.c_i_i = 0; left_arm_ctrl_obs_B.c_i_i <
               left_arm_ctrl_obs_B.m_d; left_arm_ctrl_obs_B.c_i_i++) {
            left_arm_ctrl_obs_B.i_c3 = left_arm_ctrl_obs_B.c_i_i + 1;
            left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.coffset_g +
              left_arm_ctrl_obs_B.i_c3;
            Hji->data[left_arm_ctrl_obs_B.f] += a->
              data[left_arm_ctrl_obs_B.aoffset_g + left_arm_ctrl_obs_B.i_c3] *
              left_arm_ctrl_obs_B.temp_l;
          }
        }
      }

      left_arm_ctrl_obs_B.n_d = Hji->size[1];
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <
           left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.f++) {
        left_arm_ctrl_obs_B.b_j_c = Hji->size[0];
        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr <
             left_arm_ctrl_obs_B.b_j_c; left_arm_ctrl_obs_B.b_i_pr++) {
          H->data[(left_arm_ctrl_obs_B.pid_tmp_d + left_arm_ctrl_obs_B.b_i_pr) +
            H->size[0] * (left_arm_ctrl_obs_B.X_tmp_i + left_arm_ctrl_obs_B.f)] =
            Hji->data[Hji->size[0] * left_arm_ctrl_obs_B.f +
            left_arm_ctrl_obs_B.b_i_pr];
        }
      }

      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
             left_arm_ctrl_obs_B.b_i_pr++) {
          left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.b_i_pr + 6 *
            left_arm_ctrl_obs_B.f] = X->data[left_arm_ctrl_obs_B.q_size_tmp_j].
            f1[6 * left_arm_ctrl_obs_B.b_i_pr + left_arm_ctrl_obs_B.f];
        }
      }

      left_arm_ctrl_obs_B.f = B->size[0] * B->size[1];
      B->size[0] = 6;
      B->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(B, left_arm_ctrl_obs_B.f);
      left_arm_ctrl_obs_B.n_d = Fi->size[0] * Fi->size[1] - 1;
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
           left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.f++) {
        B->data[left_arm_ctrl_obs_B.f] = Fi->data[left_arm_ctrl_obs_B.f];
      }

      left_arm_ctrl_obs_B.n_d = Fi->size[1];
      left_arm_ctrl_obs_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = left_arm_ctrl_obs_B.n_d;
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.b_j_c = 0; left_arm_ctrl_obs_B.b_j_c <
           left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.b_j_c++) {
        left_arm_ctrl_obs_B.pid_tmp_d = left_arm_ctrl_obs_B.b_j_c * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
             left_arm_ctrl_obs_B.b_i_pr++) {
          Fi->data[(left_arm_ctrl_obs_B.pid_tmp_d + left_arm_ctrl_obs_B.b_i_pr)
            + 1] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
             left_arm_ctrl_obs_B.b_i_pr++) {
          left_arm_ctrl_obs_B.aoffset_g = left_arm_ctrl_obs_B.b_i_pr * 6 - 1;
          left_arm_ctrl_obs_B.temp_l = B->data[(left_arm_ctrl_obs_B.pid_tmp_d +
            left_arm_ctrl_obs_B.b_i_pr) + 1];
          for (left_arm_ctrl_obs_B.c_i_i = 0; left_arm_ctrl_obs_B.c_i_i < 6;
               left_arm_ctrl_obs_B.c_i_i++) {
            left_arm_ctrl_obs_B.i_c3 = left_arm_ctrl_obs_B.c_i_i + 1;
            left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.pid_tmp_d +
              left_arm_ctrl_obs_B.i_c3;
            Fi->data[left_arm_ctrl_obs_B.f] +=
              left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.aoffset_g +
              left_arm_ctrl_obs_B.i_c3] * left_arm_ctrl_obs_B.temp_l;
          }
        }
      }

      while (left_arm_ctrl_obs_B.pid_n > 0.0) {
        left_arm_ctrl_obs_B.pid_tmp_d = static_cast<int32_T>
          (left_arm_ctrl_obs_B.pid_n);
        left_arm_ctrl_obs_B.q_size_tmp_j = left_arm_ctrl_obs_B.pid_tmp_d - 1;
        obj = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp_j];
        left_arm_ctrl_obs_B.f = Sj->size[0] * Sj->size[1];
        Sj->size[0] = 6;
        Sj->size[1] = obj->JointInternal.MotionSubspace->size[1];
        left_a_emxEnsureCapacity_real_T(Sj, left_arm_ctrl_obs_B.f);
        left_arm_ctrl_obs_B.n_d = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
             left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.f++) {
          Sj->data[left_arm_ctrl_obs_B.f] = obj->
            JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.f];
        }

        left_arm_ctrl_obs_B.b_idx_0_pt = robot->
          VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_d - 1];
        left_arm_ctrl_obs_B.b_idx_1_f = robot->
          VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_d + 9];
        if (left_arm_ctrl_obs_B.b_idx_0_pt <= left_arm_ctrl_obs_B.b_idx_1_f) {
          left_arm_ctrl_obs_B.f = a->size[0] * a->size[1];
          a->size[0] = Sj->size[1];
          a->size[1] = 6;
          left_a_emxEnsureCapacity_real_T(a, left_arm_ctrl_obs_B.f);
          for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
               left_arm_ctrl_obs_B.f++) {
            left_arm_ctrl_obs_B.n_d = Sj->size[1];
            for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr <
                 left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.b_i_pr++) {
              a->data[left_arm_ctrl_obs_B.b_i_pr + a->size[0] *
                left_arm_ctrl_obs_B.f] = Sj->data[6 * left_arm_ctrl_obs_B.b_i_pr
                + left_arm_ctrl_obs_B.f];
            }
          }

          left_arm_ctrl_obs_B.m_d = a->size[0];
          left_arm_ctrl_obs_B.n_d = Fi->size[1] - 1;
          left_arm_ctrl_obs_B.f = Hji->size[0] * Hji->size[1];
          Hji->size[0] = a->size[0];
          Hji->size[1] = Fi->size[1];
          left_a_emxEnsureCapacity_real_T(Hji, left_arm_ctrl_obs_B.f);
          for (left_arm_ctrl_obs_B.b_j_c = 0; left_arm_ctrl_obs_B.b_j_c <=
               left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.b_j_c++) {
            left_arm_ctrl_obs_B.coffset_g = left_arm_ctrl_obs_B.b_j_c *
              left_arm_ctrl_obs_B.m_d - 1;
            left_arm_ctrl_obs_B.boffset_l = left_arm_ctrl_obs_B.b_j_c * 6 - 1;
            for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr <
                 left_arm_ctrl_obs_B.m_d; left_arm_ctrl_obs_B.b_i_pr++) {
              Hji->data[(left_arm_ctrl_obs_B.coffset_g +
                         left_arm_ctrl_obs_B.b_i_pr) + 1] = 0.0;
            }

            for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
                 left_arm_ctrl_obs_B.b_i_pr++) {
              left_arm_ctrl_obs_B.aoffset_g = left_arm_ctrl_obs_B.b_i_pr *
                left_arm_ctrl_obs_B.m_d - 1;
              left_arm_ctrl_obs_B.temp_l = Fi->data
                [(left_arm_ctrl_obs_B.boffset_l + left_arm_ctrl_obs_B.b_i_pr) +
                1];
              for (left_arm_ctrl_obs_B.c_i_i = 0; left_arm_ctrl_obs_B.c_i_i <
                   left_arm_ctrl_obs_B.m_d; left_arm_ctrl_obs_B.c_i_i++) {
                left_arm_ctrl_obs_B.i_c3 = left_arm_ctrl_obs_B.c_i_i + 1;
                left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.coffset_g +
                  left_arm_ctrl_obs_B.i_c3;
                Hji->data[left_arm_ctrl_obs_B.f] += a->
                  data[left_arm_ctrl_obs_B.aoffset_g + left_arm_ctrl_obs_B.i_c3]
                  * left_arm_ctrl_obs_B.temp_l;
              }
            }
          }

          if (left_arm_ctrl_obs_B.b_idx_0_pt > left_arm_ctrl_obs_B.b_idx_1_f) {
            left_arm_ctrl_obs_B.pid_tmp_d = 0;
          } else {
            left_arm_ctrl_obs_B.pid_tmp_d = static_cast<int32_T>
              (left_arm_ctrl_obs_B.b_idx_0_pt) - 1;
          }

          if (left_arm_ctrl_obs_B.vNum_o > left_arm_ctrl_obs_B.p_idx_1_p) {
            left_arm_ctrl_obs_B.X_tmp_i = 0;
          } else {
            left_arm_ctrl_obs_B.X_tmp_i = static_cast<int32_T>
              (left_arm_ctrl_obs_B.vNum_o) - 1;
          }

          left_arm_ctrl_obs_B.n_d = Hji->size[1];
          for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <
               left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.f++) {
            left_arm_ctrl_obs_B.b_j_c = Hji->size[0];
            for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr <
                 left_arm_ctrl_obs_B.b_j_c; left_arm_ctrl_obs_B.b_i_pr++) {
              H->data[(left_arm_ctrl_obs_B.pid_tmp_d +
                       left_arm_ctrl_obs_B.b_i_pr) + H->size[0] *
                (left_arm_ctrl_obs_B.X_tmp_i + left_arm_ctrl_obs_B.f)] =
                Hji->data[Hji->size[0] * left_arm_ctrl_obs_B.f +
                left_arm_ctrl_obs_B.b_i_pr];
            }
          }

          if (left_arm_ctrl_obs_B.vNum_o > left_arm_ctrl_obs_B.p_idx_1_p) {
            left_arm_ctrl_obs_B.pid_tmp_d = 0;
          } else {
            left_arm_ctrl_obs_B.pid_tmp_d = static_cast<int32_T>
              (left_arm_ctrl_obs_B.vNum_o) - 1;
          }

          if (left_arm_ctrl_obs_B.b_idx_0_pt > left_arm_ctrl_obs_B.b_idx_1_f) {
            left_arm_ctrl_obs_B.X_tmp_i = 0;
          } else {
            left_arm_ctrl_obs_B.X_tmp_i = static_cast<int32_T>
              (left_arm_ctrl_obs_B.b_idx_0_pt) - 1;
          }

          left_arm_ctrl_obs_B.n_d = Hji->size[0];
          for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <
               left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.f++) {
            left_arm_ctrl_obs_B.b_j_c = Hji->size[1];
            for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr <
                 left_arm_ctrl_obs_B.b_j_c; left_arm_ctrl_obs_B.b_i_pr++) {
              H->data[(left_arm_ctrl_obs_B.pid_tmp_d +
                       left_arm_ctrl_obs_B.b_i_pr) + H->size[0] *
                (left_arm_ctrl_obs_B.X_tmp_i + left_arm_ctrl_obs_B.f)] =
                Hji->data[Hji->size[0] * left_arm_ctrl_obs_B.b_i_pr +
                left_arm_ctrl_obs_B.f];
            }
          }
        }

        for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
             left_arm_ctrl_obs_B.f++) {
          for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
               left_arm_ctrl_obs_B.b_i_pr++) {
            left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.b_i_pr + 6 *
              left_arm_ctrl_obs_B.f] = X->data[left_arm_ctrl_obs_B.q_size_tmp_j]
              .f1[6 * left_arm_ctrl_obs_B.b_i_pr + left_arm_ctrl_obs_B.f];
          }
        }

        left_arm_ctrl_obs_B.f = B->size[0] * B->size[1];
        B->size[0] = 6;
        B->size[1] = Fi->size[1];
        left_a_emxEnsureCapacity_real_T(B, left_arm_ctrl_obs_B.f);
        left_arm_ctrl_obs_B.n_d = Fi->size[0] * Fi->size[1] - 1;
        for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
             left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.f++) {
          B->data[left_arm_ctrl_obs_B.f] = Fi->data[left_arm_ctrl_obs_B.f];
        }

        left_arm_ctrl_obs_B.n_d = Fi->size[1];
        left_arm_ctrl_obs_B.f = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = left_arm_ctrl_obs_B.n_d;
        left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.f);
        for (left_arm_ctrl_obs_B.b_j_c = 0; left_arm_ctrl_obs_B.b_j_c <
             left_arm_ctrl_obs_B.n_d; left_arm_ctrl_obs_B.b_j_c++) {
          left_arm_ctrl_obs_B.pid_tmp_d = left_arm_ctrl_obs_B.b_j_c * 6 - 1;
          for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
               left_arm_ctrl_obs_B.b_i_pr++) {
            Fi->data[(left_arm_ctrl_obs_B.pid_tmp_d + left_arm_ctrl_obs_B.b_i_pr)
              + 1] = 0.0;
          }

          for (left_arm_ctrl_obs_B.b_i_pr = 0; left_arm_ctrl_obs_B.b_i_pr < 6;
               left_arm_ctrl_obs_B.b_i_pr++) {
            left_arm_ctrl_obs_B.aoffset_g = left_arm_ctrl_obs_B.b_i_pr * 6 - 1;
            left_arm_ctrl_obs_B.temp_l = B->data[(left_arm_ctrl_obs_B.pid_tmp_d
              + left_arm_ctrl_obs_B.b_i_pr) + 1];
            for (left_arm_ctrl_obs_B.c_i_i = 0; left_arm_ctrl_obs_B.c_i_i < 6;
                 left_arm_ctrl_obs_B.c_i_i++) {
              left_arm_ctrl_obs_B.i_c3 = left_arm_ctrl_obs_B.c_i_i + 1;
              left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.pid_tmp_d +
                left_arm_ctrl_obs_B.i_c3;
              Fi->data[left_arm_ctrl_obs_B.f] +=
                left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.aoffset_g +
                left_arm_ctrl_obs_B.i_c3] * left_arm_ctrl_obs_B.temp_l;
            }
          }
        }

        left_arm_ctrl_obs_B.pid_n = robot->
          Bodies[left_arm_ctrl_obs_B.q_size_tmp_j]->ParentIndex;
      }
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&B);
  left_arm_ctrl_ob_emxFree_real_T(&a);
  left_arm_ctrl_ob_emxFree_real_T(&Hji);
  left_arm_ctrl_ob_emxFree_real_T(&Sj);
  left_arm_ctrl_ob_emxFree_real_T(&Fi);
  left_arm_ctrl_ob_emxFree_real_T(&Si);
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

static void left_arm_ctrl_ob_emxInit_char_T(emxArray_char_T_left_arm_ctrl_T
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
  for (left_arm_ctrl_obs_B.i_l = 0; left_arm_ctrl_obs_B.i_l < numDimensions;
       left_arm_ctrl_obs_B.i_l++) {
    emxArray->size[left_arm_ctrl_obs_B.i_l] = 0;
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

static void left_a_emxEnsureCapacity_char_T(emxArray_char_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_ctrl_obs_B.newNumel_n = 1;
  for (left_arm_ctrl_obs_B.i_o = 0; left_arm_ctrl_obs_B.i_o <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_o++) {
    left_arm_ctrl_obs_B.newNumel_n *= emxArray->size[left_arm_ctrl_obs_B.i_o];
  }

  if (left_arm_ctrl_obs_B.newNumel_n > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_o = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_o < 16) {
      left_arm_ctrl_obs_B.i_o = 16;
    }

    while (left_arm_ctrl_obs_B.i_o < left_arm_ctrl_obs_B.newNumel_n) {
      if (left_arm_ctrl_obs_B.i_o > 1073741823) {
        left_arm_ctrl_obs_B.i_o = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_o <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_o), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_o;
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
  for (left_arm_ctrl_obs_B.b_kstr_mx = 0; left_arm_ctrl_obs_B.b_kstr_mx < 8;
       left_arm_ctrl_obs_B.b_kstr_mx++) {
    left_arm_ctrl_obs_B.b_e[left_arm_ctrl_obs_B.b_kstr_mx] =
      tmp[left_arm_ctrl_obs_B.b_kstr_mx];
  }

  left_arm_ctrl_obs_B.b_bool_a = false;
  if (obj->Type->size[1] == 8) {
    left_arm_ctrl_obs_B.b_kstr_mx = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_mx - 1 < 8) {
        left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_mx - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr_k] !=
            left_arm_ctrl_obs_B.b_e[left_arm_ctrl_obs_B.kstr_k]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_mx++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_a = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (left_arm_ctrl_obs_B.b_bool_a) {
    guard1 = true;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr_mx = 0; left_arm_ctrl_obs_B.b_kstr_mx < 9;
         left_arm_ctrl_obs_B.b_kstr_mx++) {
      left_arm_ctrl_obs_B.b_l[left_arm_ctrl_obs_B.b_kstr_mx] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr_mx];
    }

    left_arm_ctrl_obs_B.b_bool_a = false;
    if (obj->Type->size[1] == 9) {
      left_arm_ctrl_obs_B.b_kstr_mx = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_mx - 1 < 9) {
          left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_mx - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr_k] !=
              left_arm_ctrl_obs_B.b_l[left_arm_ctrl_obs_B.kstr_k]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_mx++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_a = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_a) {
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
  for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 16;
       left_arm_ctrl_obs_B.b_kstr_m1++) {
    left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.b_kstr_m1] =
      tmp[left_arm_ctrl_obs_B.b_kstr_m1];
  }

  left_arm_ctrl_obs_B.ntilecols = static_cast<int32_T>(left_arm_ctrl_obs_B.n);
  left_arm_ctrl_obs_B.b_kstr_m1 = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  Ttree->size[1] = left_arm_ctrl_obs_B.ntilecols;
  l_emxEnsureCapacity_e_cell_wrap(Ttree, left_arm_ctrl_obs_B.b_kstr_m1);
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
  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  if (0 <= left_arm_ctrl_obs_B.ntilecols) {
    for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 5;
         left_arm_ctrl_obs_B.b_kstr_m1++) {
      left_arm_ctrl_obs_B.b_fb[left_arm_ctrl_obs_B.b_kstr_m1] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr_m1];
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

    left_arm_ctrl_obs_B.b_kstr_m1 = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    left_a_emxEnsureCapacity_char_T(switch_expression,
      left_arm_ctrl_obs_B.b_kstr_m1);
    left_arm_ctrl_obs_B.loop_ub_j = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 <=
         left_arm_ctrl_obs_B.loop_ub_j; left_arm_ctrl_obs_B.b_kstr_m1++) {
      switch_expression->data[left_arm_ctrl_obs_B.b_kstr_m1] =
        body->JointInternal.Type->data[left_arm_ctrl_obs_B.b_kstr_m1];
    }

    left_arm_ctrl_obs_B.b_bool_pi = false;
    if (switch_expression->size[1] == 5) {
      left_arm_ctrl_obs_B.b_kstr_m1 = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_m1 - 1 < 5) {
          left_arm_ctrl_obs_B.loop_ub_j = left_arm_ctrl_obs_B.b_kstr_m1 - 1;
          if (switch_expression->data[left_arm_ctrl_obs_B.loop_ub_j] !=
              left_arm_ctrl_obs_B.b_fb[left_arm_ctrl_obs_B.loop_ub_j]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_m1++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_pi = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_pi) {
      left_arm_ctrl_obs_B.b_kstr_m1 = 0;
    } else {
      for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 8;
           left_arm_ctrl_obs_B.b_kstr_m1++) {
        left_arm_ctrl_obs_B.b_p[left_arm_ctrl_obs_B.b_kstr_m1] =
          tmp_1[left_arm_ctrl_obs_B.b_kstr_m1];
      }

      left_arm_ctrl_obs_B.b_bool_pi = false;
      if (switch_expression->size[1] == 8) {
        left_arm_ctrl_obs_B.b_kstr_m1 = 1;
        do {
          exitg1 = 0;
          if (left_arm_ctrl_obs_B.b_kstr_m1 - 1 < 8) {
            left_arm_ctrl_obs_B.loop_ub_j = left_arm_ctrl_obs_B.b_kstr_m1 - 1;
            if (switch_expression->data[left_arm_ctrl_obs_B.loop_ub_j] !=
                left_arm_ctrl_obs_B.b_p[left_arm_ctrl_obs_B.loop_ub_j]) {
              exitg1 = 1;
            } else {
              left_arm_ctrl_obs_B.b_kstr_m1++;
            }
          } else {
            left_arm_ctrl_obs_B.b_bool_pi = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (left_arm_ctrl_obs_B.b_bool_pi) {
        left_arm_ctrl_obs_B.b_kstr_m1 = 1;
      } else {
        left_arm_ctrl_obs_B.b_kstr_m1 = -1;
      }
    }

    switch (left_arm_ctrl_obs_B.b_kstr_m1) {
     case 0:
      memset(&left_arm_ctrl_obs_B.c_f1[0], 0, sizeof(real_T) << 4U);
      left_arm_ctrl_obs_B.c_f1[0] = 1.0;
      left_arm_ctrl_obs_B.c_f1[5] = 1.0;
      left_arm_ctrl_obs_B.c_f1[10] = 1.0;
      left_arm_ctrl_obs_B.c_f1[15] = 1.0;
      break;

     case 1:
      rigidBodyJoint_get_JointAxis_e(&body->JointInternal,
        left_arm_ctrl_obs_B.v_i);
      left_arm_ctrl_obs_B.d -= left_arm_ctrl_obs_B.e;
      for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 <
           left_arm_ctrl_obs_B.d; left_arm_ctrl_obs_B.b_kstr_m1++) {
        left_arm_ctrl_obs_B.e_data[left_arm_ctrl_obs_B.b_kstr_m1] =
          left_arm_ctrl_obs_B.e + left_arm_ctrl_obs_B.b_kstr_m1;
      }

      left_arm_ctrl_obs_B.result_data_p5[0] = left_arm_ctrl_obs_B.v_i[0];
      left_arm_ctrl_obs_B.result_data_p5[1] = left_arm_ctrl_obs_B.v_i[1];
      left_arm_ctrl_obs_B.result_data_p5[2] = left_arm_ctrl_obs_B.v_i[2];
      if (0 <= (left_arm_ctrl_obs_B.d != 0) - 1) {
        left_arm_ctrl_obs_B.result_data_p5[3] = qvec[left_arm_ctrl_obs_B.e_data
          [0]];
      }

      left_arm_ctrl_obs_B.k = 1.0 / sqrt((left_arm_ctrl_obs_B.result_data_p5[0] *
        left_arm_ctrl_obs_B.result_data_p5[0] +
        left_arm_ctrl_obs_B.result_data_p5[1] *
        left_arm_ctrl_obs_B.result_data_p5[1]) +
        left_arm_ctrl_obs_B.result_data_p5[2] *
        left_arm_ctrl_obs_B.result_data_p5[2]);
      left_arm_ctrl_obs_B.v_i[0] = left_arm_ctrl_obs_B.result_data_p5[0] *
        left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.v_i[1] = left_arm_ctrl_obs_B.result_data_p5[1] *
        left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.v_i[2] = left_arm_ctrl_obs_B.result_data_p5[2] *
        left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.k = cos(left_arm_ctrl_obs_B.result_data_p5[3]);
      left_arm_ctrl_obs_B.sth_c = sin(left_arm_ctrl_obs_B.result_data_p5[3]);
      left_arm_ctrl_obs_B.tempR_j[0] = left_arm_ctrl_obs_B.v_i[0] *
        left_arm_ctrl_obs_B.v_i[0] * (1.0 - left_arm_ctrl_obs_B.k) +
        left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.tempR_tmp_ax = left_arm_ctrl_obs_B.v_i[1] *
        left_arm_ctrl_obs_B.v_i[0] * (1.0 - left_arm_ctrl_obs_B.k);
      left_arm_ctrl_obs_B.tempR_tmp_d = left_arm_ctrl_obs_B.v_i[2] *
        left_arm_ctrl_obs_B.sth_c;
      left_arm_ctrl_obs_B.tempR_j[1] = left_arm_ctrl_obs_B.tempR_tmp_ax -
        left_arm_ctrl_obs_B.tempR_tmp_d;
      left_arm_ctrl_obs_B.tempR_tmp_af = left_arm_ctrl_obs_B.v_i[2] *
        left_arm_ctrl_obs_B.v_i[0] * (1.0 - left_arm_ctrl_obs_B.k);
      left_arm_ctrl_obs_B.tempR_tmp_p = left_arm_ctrl_obs_B.v_i[1] *
        left_arm_ctrl_obs_B.sth_c;
      left_arm_ctrl_obs_B.tempR_j[2] = left_arm_ctrl_obs_B.tempR_tmp_af +
        left_arm_ctrl_obs_B.tempR_tmp_p;
      left_arm_ctrl_obs_B.tempR_j[3] = left_arm_ctrl_obs_B.tempR_tmp_ax +
        left_arm_ctrl_obs_B.tempR_tmp_d;
      left_arm_ctrl_obs_B.tempR_j[4] = left_arm_ctrl_obs_B.v_i[1] *
        left_arm_ctrl_obs_B.v_i[1] * (1.0 - left_arm_ctrl_obs_B.k) +
        left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.tempR_tmp_ax = left_arm_ctrl_obs_B.v_i[2] *
        left_arm_ctrl_obs_B.v_i[1] * (1.0 - left_arm_ctrl_obs_B.k);
      left_arm_ctrl_obs_B.tempR_tmp_d = left_arm_ctrl_obs_B.v_i[0] *
        left_arm_ctrl_obs_B.sth_c;
      left_arm_ctrl_obs_B.tempR_j[5] = left_arm_ctrl_obs_B.tempR_tmp_ax -
        left_arm_ctrl_obs_B.tempR_tmp_d;
      left_arm_ctrl_obs_B.tempR_j[6] = left_arm_ctrl_obs_B.tempR_tmp_af -
        left_arm_ctrl_obs_B.tempR_tmp_p;
      left_arm_ctrl_obs_B.tempR_j[7] = left_arm_ctrl_obs_B.tempR_tmp_ax +
        left_arm_ctrl_obs_B.tempR_tmp_d;
      left_arm_ctrl_obs_B.tempR_j[8] = left_arm_ctrl_obs_B.v_i[2] *
        left_arm_ctrl_obs_B.v_i[2] * (1.0 - left_arm_ctrl_obs_B.k) +
        left_arm_ctrl_obs_B.k;
      for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 3;
           left_arm_ctrl_obs_B.b_kstr_m1++) {
        left_arm_ctrl_obs_B.e = left_arm_ctrl_obs_B.b_kstr_m1 + 1;
        left_arm_ctrl_obs_B.R_bj[left_arm_ctrl_obs_B.e - 1] =
          left_arm_ctrl_obs_B.tempR_j[(left_arm_ctrl_obs_B.e - 1) * 3];
        left_arm_ctrl_obs_B.e = left_arm_ctrl_obs_B.b_kstr_m1 + 1;
        left_arm_ctrl_obs_B.R_bj[left_arm_ctrl_obs_B.e + 2] =
          left_arm_ctrl_obs_B.tempR_j[(left_arm_ctrl_obs_B.e - 1) * 3 + 1];
        left_arm_ctrl_obs_B.e = left_arm_ctrl_obs_B.b_kstr_m1 + 1;
        left_arm_ctrl_obs_B.R_bj[left_arm_ctrl_obs_B.e + 5] =
          left_arm_ctrl_obs_B.tempR_j[(left_arm_ctrl_obs_B.e - 1) * 3 + 2];
      }

      memset(&left_arm_ctrl_obs_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 3;
           left_arm_ctrl_obs_B.b_kstr_m1++) {
        left_arm_ctrl_obs_B.d = left_arm_ctrl_obs_B.b_kstr_m1 << 2;
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d] =
          left_arm_ctrl_obs_B.R_bj[3 * left_arm_ctrl_obs_B.b_kstr_m1];
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 1] =
          left_arm_ctrl_obs_B.R_bj[3 * left_arm_ctrl_obs_B.b_kstr_m1 + 1];
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 2] =
          left_arm_ctrl_obs_B.R_bj[3 * left_arm_ctrl_obs_B.b_kstr_m1 + 2];
      }

      left_arm_ctrl_obs_B.c_f1[15] = 1.0;
      break;

     default:
      rigidBodyJoint_get_JointAxis_e(&body->JointInternal,
        left_arm_ctrl_obs_B.v_i);
      memset(&left_arm_ctrl_obs_B.tempR_j[0], 0, 9U * sizeof(real_T));
      left_arm_ctrl_obs_B.tempR_j[0] = 1.0;
      left_arm_ctrl_obs_B.tempR_j[4] = 1.0;
      left_arm_ctrl_obs_B.tempR_j[8] = 1.0;
      for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 3;
           left_arm_ctrl_obs_B.b_kstr_m1++) {
        left_arm_ctrl_obs_B.d = left_arm_ctrl_obs_B.b_kstr_m1 << 2;
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d] =
          left_arm_ctrl_obs_B.tempR_j[3 * left_arm_ctrl_obs_B.b_kstr_m1];
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 1] =
          left_arm_ctrl_obs_B.tempR_j[3 * left_arm_ctrl_obs_B.b_kstr_m1 + 1];
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 2] =
          left_arm_ctrl_obs_B.tempR_j[3 * left_arm_ctrl_obs_B.b_kstr_m1 + 2];
        left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.b_kstr_m1 + 12] =
          left_arm_ctrl_obs_B.v_i[left_arm_ctrl_obs_B.b_kstr_m1] *
          qvec[left_arm_ctrl_obs_B.e];
      }

      left_arm_ctrl_obs_B.c_f1[3] = 0.0;
      left_arm_ctrl_obs_B.c_f1[7] = 0.0;
      left_arm_ctrl_obs_B.c_f1[11] = 0.0;
      left_arm_ctrl_obs_B.c_f1[15] = 1.0;
      break;
    }

    for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 16;
         left_arm_ctrl_obs_B.b_kstr_m1++) {
      left_arm_ctrl_obs_B.a_m[left_arm_ctrl_obs_B.b_kstr_m1] =
        body->JointInternal.JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_m1];
    }

    for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 16;
         left_arm_ctrl_obs_B.b_kstr_m1++) {
      left_arm_ctrl_obs_B.b_n[left_arm_ctrl_obs_B.b_kstr_m1] =
        body->JointInternal.ChildToJointTransform[left_arm_ctrl_obs_B.b_kstr_m1];
    }

    for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 4;
         left_arm_ctrl_obs_B.b_kstr_m1++) {
      for (left_arm_ctrl_obs_B.e = 0; left_arm_ctrl_obs_B.e < 4;
           left_arm_ctrl_obs_B.e++) {
        left_arm_ctrl_obs_B.d = left_arm_ctrl_obs_B.e << 2;
        left_arm_ctrl_obs_B.loop_ub_j = left_arm_ctrl_obs_B.b_kstr_m1 +
          left_arm_ctrl_obs_B.d;
        left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.loop_ub_j] = 0.0;
        left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.loop_ub_j] +=
          left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d] *
          left_arm_ctrl_obs_B.a_m[left_arm_ctrl_obs_B.b_kstr_m1];
        left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.loop_ub_j] +=
          left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 1] *
          left_arm_ctrl_obs_B.a_m[left_arm_ctrl_obs_B.b_kstr_m1 + 4];
        left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.loop_ub_j] +=
          left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 2] *
          left_arm_ctrl_obs_B.a_m[left_arm_ctrl_obs_B.b_kstr_m1 + 8];
        left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.loop_ub_j] +=
          left_arm_ctrl_obs_B.c_f1[left_arm_ctrl_obs_B.d + 3] *
          left_arm_ctrl_obs_B.a_m[left_arm_ctrl_obs_B.b_kstr_m1 + 12];
      }

      for (left_arm_ctrl_obs_B.e = 0; left_arm_ctrl_obs_B.e < 4;
           left_arm_ctrl_obs_B.e++) {
        left_arm_ctrl_obs_B.d = left_arm_ctrl_obs_B.e << 2;
        left_arm_ctrl_obs_B.loop_ub_j = left_arm_ctrl_obs_B.b_kstr_m1 +
          left_arm_ctrl_obs_B.d;
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.loop_ub_j] = 0.0;
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.loop_ub_j] +=
          left_arm_ctrl_obs_B.b_n[left_arm_ctrl_obs_B.d] *
          left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.b_kstr_m1];
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.loop_ub_j] +=
          left_arm_ctrl_obs_B.b_n[left_arm_ctrl_obs_B.d + 1] *
          left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.b_kstr_m1 + 4];
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.loop_ub_j] +=
          left_arm_ctrl_obs_B.b_n[left_arm_ctrl_obs_B.d + 2] *
          left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.b_kstr_m1 + 8];
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.loop_ub_j] +=
          left_arm_ctrl_obs_B.b_n[left_arm_ctrl_obs_B.d + 3] *
          left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.b_kstr_m1 + 12];
      }
    }

    left_arm_ctrl_obs_B.k = left_arm_ctrl_obs_B.n;
    if (body->ParentIndex > 0.0) {
      for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 16;
           left_arm_ctrl_obs_B.b_kstr_m1++) {
        left_arm_ctrl_obs_B.a_m[left_arm_ctrl_obs_B.b_kstr_m1] = Ttree->data[
          static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[left_arm_ctrl_obs_B.b_kstr_m1];
      }

      for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 4;
           left_arm_ctrl_obs_B.b_kstr_m1++) {
        for (left_arm_ctrl_obs_B.e = 0; left_arm_ctrl_obs_B.e < 4;
             left_arm_ctrl_obs_B.e++) {
          left_arm_ctrl_obs_B.d = left_arm_ctrl_obs_B.e << 2;
          left_arm_ctrl_obs_B.loop_ub_j = left_arm_ctrl_obs_B.b_kstr_m1 +
            left_arm_ctrl_obs_B.d;
          left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.loop_ub_j] = 0.0;
          left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.loop_ub_j] += Ttree->
            data[left_arm_ctrl_obs_B.b_jtilecol].f1[left_arm_ctrl_obs_B.d] *
            left_arm_ctrl_obs_B.a_m[left_arm_ctrl_obs_B.b_kstr_m1];
          left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.loop_ub_j] += Ttree->
            data[left_arm_ctrl_obs_B.b_jtilecol].f1[left_arm_ctrl_obs_B.d + 1] *
            left_arm_ctrl_obs_B.a_m[left_arm_ctrl_obs_B.b_kstr_m1 + 4];
          left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.loop_ub_j] += Ttree->
            data[left_arm_ctrl_obs_B.b_jtilecol].f1[left_arm_ctrl_obs_B.d + 2] *
            left_arm_ctrl_obs_B.a_m[left_arm_ctrl_obs_B.b_kstr_m1 + 8];
          left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.loop_ub_j] += Ttree->
            data[left_arm_ctrl_obs_B.b_jtilecol].f1[left_arm_ctrl_obs_B.d + 3] *
            left_arm_ctrl_obs_B.a_m[left_arm_ctrl_obs_B.b_kstr_m1 + 12];
        }
      }

      for (left_arm_ctrl_obs_B.b_kstr_m1 = 0; left_arm_ctrl_obs_B.b_kstr_m1 < 16;
           left_arm_ctrl_obs_B.b_kstr_m1++) {
        Ttree->data[left_arm_ctrl_obs_B.b_jtilecol]
          .f1[left_arm_ctrl_obs_B.b_kstr_m1] =
          left_arm_ctrl_obs_B.a_p[left_arm_ctrl_obs_B.b_kstr_m1];
      }
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
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

static void left_arm_ctrl_SystemCore_step_e(boolean_T *varargout_1, real32_T
  varargout_2_Data[14], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_left_arm_ctrl_obs_318.getLatestMessage
    (&left_arm_ctrl_obs_B.b_varargout_2_m);
  for (left_arm_ctrl_obs_B.i_fx = 0; left_arm_ctrl_obs_B.i_fx < 14;
       left_arm_ctrl_obs_B.i_fx++) {
    varargout_2_Data[left_arm_ctrl_obs_B.i_fx] =
      left_arm_ctrl_obs_B.b_varargout_2_m.Data[left_arm_ctrl_obs_B.i_fx];
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

static void left_arm_ctrl_obs_eye(real_T b_I[36])
{
  int32_T b_k;
  memset(&b_I[0], 0, 36U * sizeof(real_T));
  for (b_k = 0; b_k < 6; b_k++) {
    b_I[b_k + 6 * b_k] = 1.0;
  }
}

static boolean_T left_arm_ctrl_obs_strcmp(const emxArray_char_T_left_arm_ctrl_T *
  a)
{
  boolean_T b_bool;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  for (left_arm_ctrl_obs_B.b_kstr_g = 0; left_arm_ctrl_obs_B.b_kstr_g < 5;
       left_arm_ctrl_obs_B.b_kstr_g++) {
    left_arm_ctrl_obs_B.b_h[left_arm_ctrl_obs_B.b_kstr_g] =
      tmp[left_arm_ctrl_obs_B.b_kstr_g];
  }

  b_bool = false;
  if (a->size[1] == 5) {
    left_arm_ctrl_obs_B.b_kstr_g = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_g - 1 < 5) {
        left_arm_ctrl_obs_B.kstr_ik = left_arm_ctrl_obs_B.b_kstr_g - 1;
        if (a->data[left_arm_ctrl_obs_B.kstr_ik] !=
            left_arm_ctrl_obs_B.b_h[left_arm_ctrl_obs_B.kstr_ik]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_g++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

static void lef_GravityTorqueBlock_stepImpl(robotics_slmanip_internal__e0_T *obj,
  const real32_T q[7], real32_T jointTorq[7])
{
  k_robotics_manip_internal__e0_T *robot;
  emxArray_e_cell_wrap_left_a_e_T *X;
  emxArray_e_cell_wrap_left_a_e_T *Xtree;
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
  for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 7;
       left_arm_ctrl_obs_B.i_n++) {
    left_arm_ctrl_obs_B.q[left_arm_ctrl_obs_B.i_n] = q[left_arm_ctrl_obs_B.i_n];
  }

  left_arm_ctrl_obs_B.a0[0] = 0.0;
  left_arm_ctrl_obs_B.a0[1] = 0.0;
  left_arm_ctrl_obs_B.a0[2] = 0.0;
  left_arm_ctrl_obs_B.a0[3] = -obj->TreeInternal.Gravity[0];
  left_arm_ctrl_obs_B.a0[4] = -obj->TreeInternal.Gravity[1];
  left_arm_ctrl_obs_B.a0[5] = -obj->TreeInternal.Gravity[2];
  left_arm_ctrl_ob_emxInit_real_T(&vJ, 2);
  left_arm_ctrl_obs_B.nb = obj->TreeInternal.NumBodies;
  left_arm_ctrl_obs_B.u = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  left_arm_ctrl_obs_B.unnamed_idx_1 = static_cast<int32_T>
    (left_arm_ctrl_obs_B.nb);
  vJ->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(vJ, left_arm_ctrl_obs_B.u);
  left_arm_ctrl_obs_B.aoffset = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 - 1;
  for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <=
       left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
    vJ->data[left_arm_ctrl_obs_B.u] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&vB, 2);
  left_arm_ctrl_obs_B.u = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(vB, left_arm_ctrl_obs_B.u);
  for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <=
       left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
    vB->data[left_arm_ctrl_obs_B.u] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&aB, 2);
  left_arm_ctrl_obs_B.u = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(aB, left_arm_ctrl_obs_B.u);
  for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <=
       left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
    aB->data[left_arm_ctrl_obs_B.u] = 0.0;
  }

  for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 7;
       left_arm_ctrl_obs_B.i_n++) {
    left_arm_ctrl_obs_B.tau[left_arm_ctrl_obs_B.i_n] = 0.0;
  }

  left_arm_c_emxInit_e_cell_wrap1(&X, 2);
  left_arm_c_emxInit_e_cell_wrap1(&Xtree, 2);
  left_arm_ctrl_obs_B.i_n = left_arm_ctrl_obs_B.unnamed_idx_1 - 1;
  left_arm_ctrl_obs_B.u = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  emxEnsureCapacity_e_cell_wrap1(Xtree, left_arm_ctrl_obs_B.u);
  left_arm_ctrl_obs_B.u = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  emxEnsureCapacity_e_cell_wrap1(X, left_arm_ctrl_obs_B.u);
  if (0 <= left_arm_ctrl_obs_B.i_n) {
    left_arm_ctrl_obs_eye(left_arm_ctrl_obs_B.b_I);
  }

  for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p <=
       left_arm_ctrl_obs_B.i_n; left_arm_ctrl_obs_B.b_k_p++) {
    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 36;
         left_arm_ctrl_obs_B.u++) {
      Xtree->data[left_arm_ctrl_obs_B.b_k_p].f1[left_arm_ctrl_obs_B.u] =
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.u];
      X->data[left_arm_ctrl_obs_B.b_k_p].f1[left_arm_ctrl_obs_B.u] =
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.u];
    }
  }

  left_arm_ctrl_ob_emxInit_real_T(&f, 2);
  left_arm_ctrl_obs_B.u = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(f, left_arm_ctrl_obs_B.u);
  left_arm_ctrl_ob_emxInit_real_T(&S, 2);
  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  for (left_arm_ctrl_obs_B.unnamed_idx_1 = 0; left_arm_ctrl_obs_B.unnamed_idx_1 <=
       left_arm_ctrl_obs_B.i_n; left_arm_ctrl_obs_B.unnamed_idx_1++) {
    obj_0 = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1];
    left_arm_ctrl_obs_B.u = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj_0->JointInternal.MotionSubspace->size[1];
    left_a_emxEnsureCapacity_real_T(S, left_arm_ctrl_obs_B.u);
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
      rigidBodyJoint_transformBodyToP(&obj_0->JointInternal,
        left_arm_ctrl_obs_B.T);
      left_arm_ctrl_obs_B.t_c = 1;
      left_arm_ctrl_obs_B.qddoti_data[0] = 0.0;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        vJ->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
          0.0;
      }
    } else {
      if (left_arm_ctrl_obs_B.a_idx_0 > left_arm_ctrl_obs_B.a_idx_1) {
        left_arm_ctrl_obs_B.b_k_p = 0;
        left_arm_ctrl_obs_B.j_b = 0;
      } else {
        left_arm_ctrl_obs_B.b_k_p = static_cast<int32_T>
          (left_arm_ctrl_obs_B.a_idx_0) - 1;
        left_arm_ctrl_obs_B.j_b = static_cast<int32_T>
          (left_arm_ctrl_obs_B.a_idx_1);
      }

      if (left_arm_ctrl_obs_B.b_idx_0 > left_arm_ctrl_obs_B.b_idx_1) {
        left_arm_ctrl_obs_B.m = 0;
        left_arm_ctrl_obs_B.inner = 0;
        left_arm_ctrl_obs_B.u = 0;
        left_arm_ctrl_obs_B.t_c = -1;
      } else {
        left_arm_ctrl_obs_B.m = static_cast<int32_T>(left_arm_ctrl_obs_B.b_idx_0)
          - 1;
        left_arm_ctrl_obs_B.inner = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_1);
        left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.m;
        left_arm_ctrl_obs_B.t_c = left_arm_ctrl_obs_B.inner - 1;
      }

      left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.t_c - left_arm_ctrl_obs_B.u;
      left_arm_ctrl_obs_B.t_c = left_arm_ctrl_obs_B.u + 1;
      if (0 <= left_arm_ctrl_obs_B.u) {
        memset(&left_arm_ctrl_obs_B.qddoti_data[0], 0, (left_arm_ctrl_obs_B.u +
                1) * sizeof(real_T));
      }

      obj_0 = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1];
      left_arm_ctrl_obs_B.u = switch_expression->size[0] *
        switch_expression->size[1];
      switch_expression->size[0] = 1;
      switch_expression->size[1] = obj_0->JointInternal.Type->size[1];
      left_a_emxEnsureCapacity_char_T(switch_expression, left_arm_ctrl_obs_B.u);
      left_arm_ctrl_obs_B.aoffset = obj_0->JointInternal.Type->size[0] *
        obj_0->JointInternal.Type->size[1] - 1;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <=
           left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
        switch_expression->data[left_arm_ctrl_obs_B.u] =
          obj_0->JointInternal.Type->data[left_arm_ctrl_obs_B.u];
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 5;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.b_k[left_arm_ctrl_obs_B.u] =
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
                left_arm_ctrl_obs_B.b_k[left_arm_ctrl_obs_B.aoffset]) {
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
          left_arm_ctrl_obs_B.b_m[left_arm_ctrl_obs_B.u] =
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
                  left_arm_ctrl_obs_B.b_m[left_arm_ctrl_obs_B.aoffset]) {
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
        le_rigidBodyJoint_get_JointAxis(&obj_0->JointInternal,
          left_arm_ctrl_obs_B.v);
        left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.j_b -
          left_arm_ctrl_obs_B.b_k_p;
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u <
             left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.l_data[left_arm_ctrl_obs_B.u] =
            left_arm_ctrl_obs_B.b_k_p + left_arm_ctrl_obs_B.u;
        }

        left_arm_ctrl_obs_B.result_data[0] = left_arm_ctrl_obs_B.v[0];
        left_arm_ctrl_obs_B.result_data[1] = left_arm_ctrl_obs_B.v[1];
        left_arm_ctrl_obs_B.result_data[2] = left_arm_ctrl_obs_B.v[2];
        if (0 <= (left_arm_ctrl_obs_B.aoffset != 0) - 1) {
          left_arm_ctrl_obs_B.result_data[3] =
            left_arm_ctrl_obs_B.q[left_arm_ctrl_obs_B.l_data[0]];
        }

        left_arm_ctrl_obs_normalizeRows(&left_arm_ctrl_obs_B.result_data[0],
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
        left_arm_ctrl_obs_B.tempR_tmp_h = left_arm_ctrl_obs_B.v[2] *
          left_arm_ctrl_obs_B.v[1] * (1.0 - left_arm_ctrl_obs_B.a_idx_0);
        left_arm_ctrl_obs_B.sth *= left_arm_ctrl_obs_B.v[0];
        left_arm_ctrl_obs_cat(left_arm_ctrl_obs_B.v[0] * left_arm_ctrl_obs_B.v[0]
                              * (1.0 - left_arm_ctrl_obs_B.a_idx_0) +
                              left_arm_ctrl_obs_B.a_idx_0,
                              left_arm_ctrl_obs_B.a_idx_1 -
                              left_arm_ctrl_obs_B.b_idx_0,
                              left_arm_ctrl_obs_B.b_idx_1 +
                              left_arm_ctrl_obs_B.tempR_tmp,
                              left_arm_ctrl_obs_B.a_idx_1 +
                              left_arm_ctrl_obs_B.b_idx_0,
                              left_arm_ctrl_obs_B.v[1] * left_arm_ctrl_obs_B.v[1]
                              * (1.0 - left_arm_ctrl_obs_B.a_idx_0) +
                              left_arm_ctrl_obs_B.a_idx_0,
                              left_arm_ctrl_obs_B.tempR_tmp_h -
                              left_arm_ctrl_obs_B.sth,
                              left_arm_ctrl_obs_B.b_idx_1 -
                              left_arm_ctrl_obs_B.tempR_tmp,
                              left_arm_ctrl_obs_B.tempR_tmp_h +
                              left_arm_ctrl_obs_B.sth, left_arm_ctrl_obs_B.v[2] *
                              left_arm_ctrl_obs_B.v[2] * (1.0 -
          left_arm_ctrl_obs_B.a_idx_0) + left_arm_ctrl_obs_B.a_idx_0,
                              left_arm_ctrl_obs_B.tempR_n);
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 3;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.b_k_p + 1;
          left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.u - 1] =
            left_arm_ctrl_obs_B.tempR_n[(left_arm_ctrl_obs_B.u - 1) * 3];
          left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.b_k_p + 1;
          left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.u + 2] =
            left_arm_ctrl_obs_B.tempR_n[(left_arm_ctrl_obs_B.u - 1) * 3 + 1];
          left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.b_k_p + 1;
          left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.u + 5] =
            left_arm_ctrl_obs_B.tempR_n[(left_arm_ctrl_obs_B.u - 1) * 3 + 2];
        }

        memset(&left_arm_ctrl_obs_B.TJ[0], 0, sizeof(real_T) << 4U);
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u << 2;
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset] =
            left_arm_ctrl_obs_B.R_b[3 * left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 1] =
            left_arm_ctrl_obs_B.R_b[3 * left_arm_ctrl_obs_B.u + 1];
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 2] =
            left_arm_ctrl_obs_B.R_b[3 * left_arm_ctrl_obs_B.u + 2];
        }

        left_arm_ctrl_obs_B.TJ[15] = 1.0;
        break;

       default:
        le_rigidBodyJoint_get_JointAxis(&obj_0->JointInternal,
          left_arm_ctrl_obs_B.v);
        memset(&left_arm_ctrl_obs_B.tempR_n[0], 0, 9U * sizeof(real_T));
        left_arm_ctrl_obs_B.tempR_n[0] = 1.0;
        left_arm_ctrl_obs_B.tempR_n[4] = 1.0;
        left_arm_ctrl_obs_B.tempR_n[8] = 1.0;
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u << 2;
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset] =
            left_arm_ctrl_obs_B.tempR_n[3 * left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 1] =
            left_arm_ctrl_obs_B.tempR_n[3 * left_arm_ctrl_obs_B.u + 1];
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 2] =
            left_arm_ctrl_obs_B.tempR_n[3 * left_arm_ctrl_obs_B.u + 2];
          left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.u + 12] =
            left_arm_ctrl_obs_B.v[left_arm_ctrl_obs_B.u] *
            left_arm_ctrl_obs_B.q[left_arm_ctrl_obs_B.b_k_p];
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
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 4;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.b_k_p << 2;
          left_arm_ctrl_obs_B.j_b = left_arm_ctrl_obs_B.u +
            left_arm_ctrl_obs_B.aoffset;
          left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.j_b] = 0.0;
          left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.j_b] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.j_b] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 1] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u + 4];
          left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.j_b] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 2] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u + 8];
          left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.j_b] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 3] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u + 12];
        }

        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 4;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.j_b = left_arm_ctrl_obs_B.b_k_p << 2;
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u +
            left_arm_ctrl_obs_B.j_b;
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] = 0.0;
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j_b] *
            left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j_b + 1] *
            left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.u + 4];
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j_b + 2] *
            left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.u + 8];
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j_b + 3] *
            left_arm_ctrl_obs_B.a_b[left_arm_ctrl_obs_B.u + 12];
        }
      }

      if ((S->size[1] == 1) || (left_arm_ctrl_obs_B.inner -
           left_arm_ctrl_obs_B.m == 1)) {
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.b_k_p = left_arm_ctrl_obs_B.u + 6 *
            left_arm_ctrl_obs_B.unnamed_idx_1;
          vJ->data[left_arm_ctrl_obs_B.b_k_p] = 0.0;
          left_arm_ctrl_obs_B.aoffset = S->size[1];
          for (left_arm_ctrl_obs_B.m = 0; left_arm_ctrl_obs_B.m <
               left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.m++) {
            vJ->data[left_arm_ctrl_obs_B.b_k_p] += S->data[6 *
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

        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p <=
             left_arm_ctrl_obs_B.inner; left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.b_k_p * 6 - 1;
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

    left_arm_ctrl_obs_tforminv(left_arm_ctrl_obs_B.T, left_arm_ctrl_obs_B.TJ);
    left_arm_ct_tformToSpatialXform(left_arm_ctrl_obs_B.TJ, X->
      data[left_arm_ctrl_obs_B.unnamed_idx_1].f1);
    left_arm_ctrl_obs_B.a_idx_0 = robot->
      Bodies[left_arm_ctrl_obs_B.unnamed_idx_1]->ParentIndex;
    if (left_arm_ctrl_obs_B.a_idx_0 > 0.0) {
      left_arm_ctrl_obs_B.m = static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0);
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = 0.0;
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 6;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.a_idx_1 += vB->data[(left_arm_ctrl_obs_B.m - 1) *
            6 + left_arm_ctrl_obs_B.b_k_p] * X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
            left_arm_ctrl_obs_B.b_k_p + left_arm_ctrl_obs_B.u];
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

      if ((S->size[1] == 1) || (left_arm_ctrl_obs_B.t_c == 1)) {
        left_arm_ctrl_obs_B.aoffset = S->size[1];
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = 0.0;
          for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p <
               left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.b_k_p++) {
            left_arm_ctrl_obs_B.a_idx_1 = S->data[6 * left_arm_ctrl_obs_B.b_k_p
              + left_arm_ctrl_obs_B.u] *
              left_arm_ctrl_obs_B.qddoti_data[left_arm_ctrl_obs_B.b_k_p] +
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

        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p <=
             left_arm_ctrl_obs_B.inner; left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.b_k_p * 6 - 1;
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

      left_arm_ctrl_obs_B.tempR_n[0] = 0.0;
      left_arm_ctrl_obs_B.t_c = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 2;
      left_arm_ctrl_obs_B.tempR_n[3] = -vB->data[left_arm_ctrl_obs_B.t_c];
      left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 1;
      left_arm_ctrl_obs_B.tempR_n[6] = vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR_n[1] = vB->data[left_arm_ctrl_obs_B.t_c];
      left_arm_ctrl_obs_B.tempR_n[4] = 0.0;
      left_arm_ctrl_obs_B.tempR_n[7] = -vB->data[6 *
        left_arm_ctrl_obs_B.unnamed_idx_1];
      left_arm_ctrl_obs_B.tempR_n[2] = -vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR_n[5] = vB->data[6 *
        left_arm_ctrl_obs_B.unnamed_idx_1];
      left_arm_ctrl_obs_B.tempR_n[8] = 0.0;
      left_arm_ctrl_obs_B.tempR[3] = 0.0;
      left_arm_ctrl_obs_B.t_c = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 5;
      left_arm_ctrl_obs_B.tempR[9] = -vB->data[left_arm_ctrl_obs_B.t_c];
      left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 4;
      left_arm_ctrl_obs_B.tempR[15] = vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR[4] = vB->data[left_arm_ctrl_obs_B.t_c];
      left_arm_ctrl_obs_B.tempR[10] = 0.0;
      left_arm_ctrl_obs_B.t_c = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 3;
      left_arm_ctrl_obs_B.tempR[16] = -vB->data[left_arm_ctrl_obs_B.t_c];
      left_arm_ctrl_obs_B.tempR[5] = -vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR[11] = vB->data[left_arm_ctrl_obs_B.t_c];
      left_arm_ctrl_obs_B.tempR[17] = 0.0;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_n[3 *
          left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.t_c = 6 * (left_arm_ctrl_obs_B.u + 3);
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c] = 0.0;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c + 3] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_n[3 *
          left_arm_ctrl_obs_B.u + 1];
        left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 1] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c + 1] = 0.0;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c + 4] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_n[3 *
          left_arm_ctrl_obs_B.u + 2];
        left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 2] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c + 2] = 0.0;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c + 5] =
          left_arm_ctrl_obs_B.a_idx_1;
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = 0.0;
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 6;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.a_idx_1 += aB->data[(left_arm_ctrl_obs_B.m - 1) *
            6 + left_arm_ctrl_obs_B.b_k_p] * X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
            left_arm_ctrl_obs_B.b_k_p + left_arm_ctrl_obs_B.u];
        }

        left_arm_ctrl_obs_B.X_c[left_arm_ctrl_obs_B.u] =
          left_arm_ctrl_obs_B.a_idx_1 +
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = 0.0;
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 6;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR[6 *
            left_arm_ctrl_obs_B.b_k_p + left_arm_ctrl_obs_B.u] * vJ->data[6 *
            left_arm_ctrl_obs_B.unnamed_idx_1 + left_arm_ctrl_obs_B.b_k_p] +
            left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] =
            left_arm_ctrl_obs_B.a_idx_1;
        }

        aB->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
          left_arm_ctrl_obs_B.X_c[left_arm_ctrl_obs_B.u] +
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
      }

      left_arm_ctrl_obs_B.R_b[0] = 0.0;
      left_arm_ctrl_obs_B.R_b[3] = -left_arm_ctrl_obs_B.T[14];
      left_arm_ctrl_obs_B.R_b[6] = left_arm_ctrl_obs_B.T[13];
      left_arm_ctrl_obs_B.R_b[1] = left_arm_ctrl_obs_B.T[14];
      left_arm_ctrl_obs_B.R_b[4] = 0.0;
      left_arm_ctrl_obs_B.R_b[7] = -left_arm_ctrl_obs_B.T[12];
      left_arm_ctrl_obs_B.R_b[2] = -left_arm_ctrl_obs_B.T[13];
      left_arm_ctrl_obs_B.R_b[5] = left_arm_ctrl_obs_B.T[12];
      left_arm_ctrl_obs_B.R_b[8] = 0.0;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 3;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.m = left_arm_ctrl_obs_B.u + 3 *
            left_arm_ctrl_obs_B.b_k_p;
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] = 0.0;
          left_arm_ctrl_obs_B.t_c = left_arm_ctrl_obs_B.b_k_p << 2;
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_c] *
            left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_c + 1] *
            left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.u + 3];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_c + 2] *
            left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.u + 6];
          left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.b_k_p + 6 *
            left_arm_ctrl_obs_B.u] = left_arm_ctrl_obs_B.T
            [(left_arm_ctrl_obs_B.u << 2) + left_arm_ctrl_obs_B.b_k_p];
          left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.b_k_p + 6 *
            (left_arm_ctrl_obs_B.u + 3)] = 0.0;
        }
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.b_I[6 * left_arm_ctrl_obs_B.u + 3] =
          left_arm_ctrl_obs_B.dv2[3 * left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u << 2;
        left_arm_ctrl_obs_B.t_c = 6 * (left_arm_ctrl_obs_B.u + 3);
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.t_c + 3] =
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset];
        left_arm_ctrl_obs_B.b_I[6 * left_arm_ctrl_obs_B.u + 4] =
          left_arm_ctrl_obs_B.dv2[3 * left_arm_ctrl_obs_B.u + 1];
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.t_c + 4] =
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset + 1];
        left_arm_ctrl_obs_B.b_I[6 * left_arm_ctrl_obs_B.u + 5] =
          left_arm_ctrl_obs_B.dv2[3 * left_arm_ctrl_obs_B.u + 2];
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.t_c + 5] =
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset + 2];
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 6;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.t_c = left_arm_ctrl_obs_B.u + 6 *
            left_arm_ctrl_obs_B.b_k_p;
          left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c] = 0.0;
          for (left_arm_ctrl_obs_B.m = 0; left_arm_ctrl_obs_B.m < 6;
               left_arm_ctrl_obs_B.m++) {
            left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c] += Xtree->data[
              static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0) - 1].f1[6 *
              left_arm_ctrl_obs_B.m + left_arm_ctrl_obs_B.u] *
              left_arm_ctrl_obs_B.b_I[6 * left_arm_ctrl_obs_B.b_k_p +
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
        left_arm_ctrl_obs_B.b_k_p = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 +
          left_arm_ctrl_obs_B.u;
        vB->data[left_arm_ctrl_obs_B.b_k_p] = vJ->data[left_arm_ctrl_obs_B.b_k_p];
      }

      if ((S->size[1] == 1) || (left_arm_ctrl_obs_B.t_c == 1)) {
        left_arm_ctrl_obs_B.aoffset = S->size[1];
        for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
             left_arm_ctrl_obs_B.u++) {
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = 0.0;
          for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p <
               left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.b_k_p++) {
            left_arm_ctrl_obs_B.a_idx_1 = S->data[6 * left_arm_ctrl_obs_B.b_k_p
              + left_arm_ctrl_obs_B.u] *
              left_arm_ctrl_obs_B.qddoti_data[left_arm_ctrl_obs_B.b_k_p] +
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

        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p <=
             left_arm_ctrl_obs_B.inner; left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.b_k_p * 6 - 1;
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
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 6;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.a_idx_1 += X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
            left_arm_ctrl_obs_B.b_k_p + left_arm_ctrl_obs_B.u] *
            left_arm_ctrl_obs_B.a0[left_arm_ctrl_obs_B.b_k_p];
        }

        aB->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
          left_arm_ctrl_obs_B.a_idx_1 +
          left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u];
      }

      left_arm_ctrl_obs_B.R_b[0] = 0.0;
      left_arm_ctrl_obs_B.R_b[3] = -left_arm_ctrl_obs_B.T[14];
      left_arm_ctrl_obs_B.R_b[6] = left_arm_ctrl_obs_B.T[13];
      left_arm_ctrl_obs_B.R_b[1] = left_arm_ctrl_obs_B.T[14];
      left_arm_ctrl_obs_B.R_b[4] = 0.0;
      left_arm_ctrl_obs_B.R_b[7] = -left_arm_ctrl_obs_B.T[12];
      left_arm_ctrl_obs_B.R_b[2] = -left_arm_ctrl_obs_B.T[13];
      left_arm_ctrl_obs_B.R_b[5] = left_arm_ctrl_obs_B.T[12];
      left_arm_ctrl_obs_B.R_b[8] = 0.0;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 3;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.m = left_arm_ctrl_obs_B.u + 3 *
            left_arm_ctrl_obs_B.b_k_p;
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] = 0.0;
          left_arm_ctrl_obs_B.t_c = left_arm_ctrl_obs_B.b_k_p << 2;
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_c] *
            left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_c + 1] *
            left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.u + 3];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_c + 2] *
            left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.u + 6];
          Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1]
            .f1[left_arm_ctrl_obs_B.b_k_p + 6 * left_arm_ctrl_obs_B.u] =
            left_arm_ctrl_obs_B.T[(left_arm_ctrl_obs_B.u << 2) +
            left_arm_ctrl_obs_B.b_k_p];
          Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1]
            .f1[left_arm_ctrl_obs_B.b_k_p + 6 * (left_arm_ctrl_obs_B.u + 3)] =
            0.0;
        }
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
          left_arm_ctrl_obs_B.u + 3] = left_arm_ctrl_obs_B.dv2[3 *
          left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.b_k_p = left_arm_ctrl_obs_B.u << 2;
        left_arm_ctrl_obs_B.m = 6 * (left_arm_ctrl_obs_B.u + 3);
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[left_arm_ctrl_obs_B.m
          + 3] = left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.b_k_p];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
          left_arm_ctrl_obs_B.u + 4] = left_arm_ctrl_obs_B.dv2[3 *
          left_arm_ctrl_obs_B.u + 1];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[left_arm_ctrl_obs_B.m
          + 4] = left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.b_k_p + 1];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 *
          left_arm_ctrl_obs_B.u + 5] = left_arm_ctrl_obs_B.dv2[3 *
          left_arm_ctrl_obs_B.u + 2];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[left_arm_ctrl_obs_B.m
          + 5] = left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.b_k_p + 2];
      }
    }

    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 36;
         left_arm_ctrl_obs_B.u++) {
      left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.u] = robot->
        Bodies[left_arm_ctrl_obs_B.unnamed_idx_1]->
        SpatialInertia[left_arm_ctrl_obs_B.u];
    }

    left_arm_ctrl_obs_B.tempR_n[0] = 0.0;
    left_arm_ctrl_obs_B.t_c = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 2;
    left_arm_ctrl_obs_B.tempR_n[3] = -vB->data[left_arm_ctrl_obs_B.t_c];
    left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 1;
    left_arm_ctrl_obs_B.tempR_n[6] = vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR_n[1] = vB->data[left_arm_ctrl_obs_B.t_c];
    left_arm_ctrl_obs_B.tempR_n[4] = 0.0;
    left_arm_ctrl_obs_B.tempR_n[7] = -vB->data[6 *
      left_arm_ctrl_obs_B.unnamed_idx_1];
    left_arm_ctrl_obs_B.tempR_n[2] = -vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR_n[5] = vB->data[6 *
      left_arm_ctrl_obs_B.unnamed_idx_1];
    left_arm_ctrl_obs_B.tempR_n[8] = 0.0;
    left_arm_ctrl_obs_B.tempR[18] = 0.0;
    left_arm_ctrl_obs_B.t_c = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 5;
    left_arm_ctrl_obs_B.tempR[24] = -vB->data[left_arm_ctrl_obs_B.t_c];
    left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 4;
    left_arm_ctrl_obs_B.tempR[30] = vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR[19] = vB->data[left_arm_ctrl_obs_B.t_c];
    left_arm_ctrl_obs_B.tempR[25] = 0.0;
    left_arm_ctrl_obs_B.t_c = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 3;
    left_arm_ctrl_obs_B.tempR[31] = -vB->data[left_arm_ctrl_obs_B.t_c];
    left_arm_ctrl_obs_B.tempR[20] = -vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR[26] = vB->data[left_arm_ctrl_obs_B.t_c];
    left_arm_ctrl_obs_B.tempR[32] = 0.0;
    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
         left_arm_ctrl_obs_B.u++) {
      left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_n[3 *
        left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 3] = 0.0;
      left_arm_ctrl_obs_B.t_c = 6 * (left_arm_ctrl_obs_B.u + 3);
      left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c + 3] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_n[3 *
        left_arm_ctrl_obs_B.u + 1];
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 1] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 4] = 0.0;
      left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c + 4] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_n[3 *
        left_arm_ctrl_obs_B.u + 2];
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 2] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 5] = 0.0;
      left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_c + 5] =
        left_arm_ctrl_obs_B.a_idx_1;
    }

    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
         left_arm_ctrl_obs_B.u++) {
      left_arm_ctrl_obs_B.X_c[left_arm_ctrl_obs_B.u] = 0.0;
      left_arm_ctrl_obs_B.b_I_m[left_arm_ctrl_obs_B.u] = 0.0;
      for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 6;
           left_arm_ctrl_obs_B.b_k_p++) {
        left_arm_ctrl_obs_B.a_idx_0 = left_arm_ctrl_obs_B.b_I[6 *
          left_arm_ctrl_obs_B.b_k_p + left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.t_c = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 +
          left_arm_ctrl_obs_B.b_k_p;
        left_arm_ctrl_obs_B.a_idx_1 = vB->data[left_arm_ctrl_obs_B.t_c] *
          left_arm_ctrl_obs_B.a_idx_0 +
          left_arm_ctrl_obs_B.X_c[left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.a_idx_0 = aB->data[left_arm_ctrl_obs_B.t_c] *
          left_arm_ctrl_obs_B.a_idx_0 +
          left_arm_ctrl_obs_B.b_I_m[left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.X_c[left_arm_ctrl_obs_B.u] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.b_I_m[left_arm_ctrl_obs_B.u] =
          left_arm_ctrl_obs_B.a_idx_0;
      }
    }

    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
         left_arm_ctrl_obs_B.u++) {
      left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] = 0.0;
      left_arm_ctrl_obs_B.a_idx_1 = 0.0;
      for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 6;
           left_arm_ctrl_obs_B.b_k_p++) {
        left_arm_ctrl_obs_B.a_idx_1 += Xtree->
          data[left_arm_ctrl_obs_B.unnamed_idx_1].f1[6 * left_arm_ctrl_obs_B.u +
          left_arm_ctrl_obs_B.b_k_p] * 0.0;
        left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u] +=
          left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.b_k_p +
          left_arm_ctrl_obs_B.u] *
          left_arm_ctrl_obs_B.X_c[left_arm_ctrl_obs_B.b_k_p];
      }

      f->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
        (left_arm_ctrl_obs_B.b_I_m[left_arm_ctrl_obs_B.u] +
         left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u]) -
        left_arm_ctrl_obs_B.a_idx_1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  left_arm_ctrl_ob_emxFree_real_T(&aB);
  left_arm_ctrl_ob_emxFree_real_T(&vB);
  left_arm_ctrl_ob_emxFree_real_T(&vJ);
  left_arm_c_emxFree_e_cell_wrap1(&Xtree);
  left_arm_ctrl_obs_B.i_n = static_cast<int32_T>(((-1.0 - left_arm_ctrl_obs_B.nb)
    + 1.0) / -1.0) - 1;
  left_arm_ctrl_ob_emxInit_real_T(&taui, 1);
  left_arm_ctrl_ob_emxInit_real_T(&a, 2);
  for (left_arm_ctrl_obs_B.t_c = 0; left_arm_ctrl_obs_B.t_c <=
       left_arm_ctrl_obs_B.i_n; left_arm_ctrl_obs_B.t_c++) {
    left_arm_ctrl_obs_B.a_idx_0 = left_arm_ctrl_obs_B.nb + -static_cast<real_T>
      (left_arm_ctrl_obs_B.t_c);
    left_arm_ctrl_obs_B.inner = static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0);
    left_arm_ctrl_obs_B.j_b = left_arm_ctrl_obs_B.inner - 1;
    obj_0 = robot->Bodies[left_arm_ctrl_obs_B.j_b];
    if (!left_arm_ctrl_obs_strcmp(obj_0->JointInternal.Type)) {
      obj_0 = robot->Bodies[left_arm_ctrl_obs_B.j_b];
      left_arm_ctrl_obs_B.u = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj_0->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(S, left_arm_ctrl_obs_B.u);
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
      left_a_emxEnsureCapacity_real_T(a, left_arm_ctrl_obs_B.u);
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.aoffset = S->size[1];
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p <
             left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.b_k_p++) {
          a->data[left_arm_ctrl_obs_B.b_k_p + a->size[0] * left_arm_ctrl_obs_B.u]
            = S->data[6 * left_arm_ctrl_obs_B.b_k_p + left_arm_ctrl_obs_B.u];
        }
      }

      left_arm_ctrl_obs_B.m = a->size[0] - 1;
      left_arm_ctrl_obs_B.u = taui->size[0];
      taui->size[0] = a->size[0];
      left_a_emxEnsureCapacity_real_T(taui, left_arm_ctrl_obs_B.u);
      for (left_arm_ctrl_obs_B.unnamed_idx_1 = 0;
           left_arm_ctrl_obs_B.unnamed_idx_1 <= left_arm_ctrl_obs_B.m;
           left_arm_ctrl_obs_B.unnamed_idx_1++) {
        taui->data[left_arm_ctrl_obs_B.unnamed_idx_1] = 0.0;
      }

      for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 6;
           left_arm_ctrl_obs_B.b_k_p++) {
        left_arm_ctrl_obs_B.aoffset = (left_arm_ctrl_obs_B.m + 1) *
          left_arm_ctrl_obs_B.b_k_p - 1;
        for (left_arm_ctrl_obs_B.c_i = 0; left_arm_ctrl_obs_B.c_i <=
             left_arm_ctrl_obs_B.m; left_arm_ctrl_obs_B.c_i++) {
          taui->data[left_arm_ctrl_obs_B.c_i] += f->data[(static_cast<int32_T>
            (left_arm_ctrl_obs_B.a_idx_0) - 1) * 6 + left_arm_ctrl_obs_B.b_k_p] *
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

    left_arm_ctrl_obs_B.a_idx_0 = robot->Bodies[left_arm_ctrl_obs_B.j_b]
      ->ParentIndex;
    if (left_arm_ctrl_obs_B.a_idx_0 > 0.0) {
      left_arm_ctrl_obs_B.m = static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0);
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = 0.0;
        for (left_arm_ctrl_obs_B.b_k_p = 0; left_arm_ctrl_obs_B.b_k_p < 6;
             left_arm_ctrl_obs_B.b_k_p++) {
          left_arm_ctrl_obs_B.a_idx_1 += f->data[(left_arm_ctrl_obs_B.inner - 1)
            * 6 + left_arm_ctrl_obs_B.b_k_p] * X->data[left_arm_ctrl_obs_B.j_b].
            f1[6 * left_arm_ctrl_obs_B.u + left_arm_ctrl_obs_B.b_k_p];
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

  left_arm_ctrl_ob_emxFree_real_T(&a);
  left_arm_ctrl_ob_emxFree_real_T(&taui);
  left_arm_ctrl_ob_emxFree_real_T(&S);
  left_arm_ctrl_ob_emxFree_real_T(&f);
  left_arm_c_emxFree_e_cell_wrap1(&X);
  for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n < 7;
       left_arm_ctrl_obs_B.i_n++) {
    jointTorq[left_arm_ctrl_obs_B.i_n] = static_cast<real32_T>
      (left_arm_ctrl_obs_B.tau[left_arm_ctrl_obs_B.i_n]);
  }
}

static void left_arm_ct_emxInit_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
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
  for (left_arm_ctrl_obs_B.i_ce = 0; left_arm_ctrl_obs_B.i_ce < numDimensions;
       left_arm_ctrl_obs_B.i_ce++) {
    emxArray->size[left_arm_ctrl_obs_B.i_ce] = 0;
  }
}

static void l_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_ctrl_obs_B.newNumel_c = 1;
  for (left_arm_ctrl_obs_B.i_b = 0; left_arm_ctrl_obs_B.i_b <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_b++) {
    left_arm_ctrl_obs_B.newNumel_c *= emxArray->size[left_arm_ctrl_obs_B.i_b];
  }

  if (left_arm_ctrl_obs_B.newNumel_c > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_b = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_b < 16) {
      left_arm_ctrl_obs_B.i_b = 16;
    }

    while (left_arm_ctrl_obs_B.i_b < left_arm_ctrl_obs_B.newNumel_c) {
      if (left_arm_ctrl_obs_B.i_b > 1073741823) {
        left_arm_ctrl_obs_B.i_b = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_b <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_b), sizeof
                     (f_cell_wrap_left_arm_ctrl_obs_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_left_arm_ctrl_obs_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_left_arm_ctrl_obs_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_b;
    emxArray->canFreeData = true;
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

static void RigidBodyTreeDynamics_massMatri(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_ctrl_T *H,
  emxArray_real_T_left_arm_ctrl_T *lambda)
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
  left_arm_ctrl_obs_B.nb_e = robot->NumBodies;
  left_arm_ctrl_obs_B.vNum_f = robot->VelocityNumber;
  left_arm_ctrl_obs_B.nm1d2 = H->size[0] * H->size[1];
  left_arm_ctrl_obs_B.b_i_p = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum_f);
  H->size[0] = left_arm_ctrl_obs_B.b_i_p;
  H->size[1] = left_arm_ctrl_obs_B.b_i_p;
  left_a_emxEnsureCapacity_real_T(H, left_arm_ctrl_obs_B.nm1d2);
  left_arm_ctrl_obs_B.n_p = left_arm_ctrl_obs_B.b_i_p *
    left_arm_ctrl_obs_B.b_i_p - 1;
  for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
       left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
    H->data[left_arm_ctrl_obs_B.nm1d2] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&lambda_, 2);
  left_arm_ctrl_obs_B.nm1d2 = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  left_arm_ctrl_obs_B.unnamed_idx_1_c = static_cast<int32_T>
    (left_arm_ctrl_obs_B.nb_e);
  lambda_->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_c;
  left_a_emxEnsureCapacity_real_T(lambda_, left_arm_ctrl_obs_B.nm1d2);
  left_arm_ctrl_obs_B.idx = left_arm_ctrl_obs_B.unnamed_idx_1_c - 1;
  for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
       left_arm_ctrl_obs_B.idx; left_arm_ctrl_obs_B.nm1d2++) {
    lambda_->data[left_arm_ctrl_obs_B.nm1d2] = 0.0;
  }

  left_arm_ctrl_obs_B.nm1d2 = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = left_arm_ctrl_obs_B.b_i_p;
  left_a_emxEnsureCapacity_real_T(lambda, left_arm_ctrl_obs_B.nm1d2);
  left_arm_ctrl_obs_B.n_p = left_arm_ctrl_obs_B.b_i_p - 1;
  for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
       left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
    lambda->data[left_arm_ctrl_obs_B.nm1d2] = 0.0;
  }

  left_arm_ct_emxInit_f_cell_wrap(&Ic, 2);
  left_arm_ct_emxInit_f_cell_wrap(&X, 2);
  left_arm_ctrl_obs_B.nm1d2 = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_c;
  l_emxEnsureCapacity_f_cell_wrap(Ic, left_arm_ctrl_obs_B.nm1d2);
  left_arm_ctrl_obs_B.nm1d2 = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_c;
  l_emxEnsureCapacity_f_cell_wrap(X, left_arm_ctrl_obs_B.nm1d2);
  for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p <=
       left_arm_ctrl_obs_B.idx; left_arm_ctrl_obs_B.b_i_p++) {
    for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 36;
         left_arm_ctrl_obs_B.nm1d2++) {
      Ic->data[left_arm_ctrl_obs_B.b_i_p].f1[left_arm_ctrl_obs_B.nm1d2] =
        robot->Bodies[left_arm_ctrl_obs_B.b_i_p]->
        SpatialInertia[left_arm_ctrl_obs_B.nm1d2];
    }

    left_arm_ctrl_obs_B.vNum_f = robot->PositionDoFMap[left_arm_ctrl_obs_B.b_i_p];
    left_arm_ctrl_obs_B.p_idx_1 = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.b_i_p + 10];
    if (left_arm_ctrl_obs_B.p_idx_1 < left_arm_ctrl_obs_B.vNum_f) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.b_i_p];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        left_arm_ctrl_obs_B.T_g);
    } else {
      if (left_arm_ctrl_obs_B.vNum_f > left_arm_ctrl_obs_B.p_idx_1) {
        left_arm_ctrl_obs_B.unnamed_idx_1_c = 0;
        left_arm_ctrl_obs_B.nm1d2 = -1;
      } else {
        left_arm_ctrl_obs_B.unnamed_idx_1_c = static_cast<int32_T>
          (left_arm_ctrl_obs_B.vNum_f) - 1;
        left_arm_ctrl_obs_B.nm1d2 = static_cast<int32_T>
          (left_arm_ctrl_obs_B.p_idx_1) - 1;
      }

      obj = robot->Bodies[left_arm_ctrl_obs_B.b_i_p];
      left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.nm1d2 -
        left_arm_ctrl_obs_B.unnamed_idx_1_c;
      left_arm_ctrl_obs_B.q_size_g = left_arm_ctrl_obs_B.q_size_tmp + 1;
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
           left_arm_ctrl_obs_B.q_size_tmp; left_arm_ctrl_obs_B.nm1d2++) {
        left_arm_ctrl_obs_B.q_data_n[left_arm_ctrl_obs_B.nm1d2] =
          q[left_arm_ctrl_obs_B.unnamed_idx_1_c + left_arm_ctrl_obs_B.nm1d2];
      }

      rigidBodyJoint_transformBodyT_e(&obj->JointInternal,
        left_arm_ctrl_obs_B.q_data_n, &left_arm_ctrl_obs_B.q_size_g,
        left_arm_ctrl_obs_B.T_g);
    }

    left_arm_ctrl_obs_tforminv(left_arm_ctrl_obs_B.T_g, left_arm_ctrl_obs_B.dv);
    left_arm_ct_tformToSpatialXform(left_arm_ctrl_obs_B.dv, X->
      data[left_arm_ctrl_obs_B.b_i_p].f1);
  }

  left_arm_ctrl_obs_B.idx = static_cast<int32_T>(((-1.0 -
    left_arm_ctrl_obs_B.nb_e) + 1.0) / -1.0) - 1;
  left_arm_ctrl_ob_emxInit_real_T(&Si, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Fi, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Sj, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Hji, 2);
  left_arm_ctrl_ob_emxInit_char_T(&a, 2);
  left_arm_ctrl_ob_emxInit_real_T(&a_0, 2);
  left_arm_ctrl_ob_emxInit_real_T(&B, 2);
  for (left_arm_ctrl_obs_B.unnamed_idx_1_c = 0;
       left_arm_ctrl_obs_B.unnamed_idx_1_c <= left_arm_ctrl_obs_B.idx;
       left_arm_ctrl_obs_B.unnamed_idx_1_c++) {
    left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>(left_arm_ctrl_obs_B.nb_e
      + -static_cast<real_T>(left_arm_ctrl_obs_B.unnamed_idx_1_c));
    left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.pid_tmp - 1;
    left_arm_ctrl_obs_B.pid = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp]
      ->ParentIndex;
    left_arm_ctrl_obs_B.vNum_f = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp - 1];
    left_arm_ctrl_obs_B.p_idx_1 = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp + 9];
    if (left_arm_ctrl_obs_B.pid > 0.0) {
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
           left_arm_ctrl_obs_B.nm1d2++) {
        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
             left_arm_ctrl_obs_B.b_i_p++) {
          left_arm_ctrl_obs_B.n_p = left_arm_ctrl_obs_B.nm1d2 + 6 *
            left_arm_ctrl_obs_B.b_i_p;
          left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.n_p] = 0.0;
          for (left_arm_ctrl_obs_B.cb = 0; left_arm_ctrl_obs_B.cb < 6;
               left_arm_ctrl_obs_B.cb++) {
            left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.n_p] += X->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 *
              left_arm_ctrl_obs_B.nm1d2 + left_arm_ctrl_obs_B.cb] * Ic->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 *
              left_arm_ctrl_obs_B.b_i_p + left_arm_ctrl_obs_B.cb];
          }
        }
      }

      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
           left_arm_ctrl_obs_B.nm1d2++) {
        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
             left_arm_ctrl_obs_B.b_i_p++) {
          left_arm_ctrl_obs_B.b_idx_0_h = 0.0;
          for (left_arm_ctrl_obs_B.cb = 0; left_arm_ctrl_obs_B.cb < 6;
               left_arm_ctrl_obs_B.cb++) {
            left_arm_ctrl_obs_B.b_idx_0_h += left_arm_ctrl_obs_B.X[6 *
              left_arm_ctrl_obs_B.cb + left_arm_ctrl_obs_B.nm1d2] * X->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 *
              left_arm_ctrl_obs_B.b_i_p + left_arm_ctrl_obs_B.cb];
          }

          left_arm_ctrl_obs_B.cb = 6 * left_arm_ctrl_obs_B.b_i_p +
            left_arm_ctrl_obs_B.nm1d2;
          Ic->data[static_cast<int32_T>(left_arm_ctrl_obs_B.pid) - 1]
            .f1[left_arm_ctrl_obs_B.cb] += left_arm_ctrl_obs_B.b_idx_0_h;
        }
      }

      lambda_->data[left_arm_ctrl_obs_B.q_size_tmp] = left_arm_ctrl_obs_B.pid;
      if (lambda_->data[left_arm_ctrl_obs_B.q_size_tmp] > 0.0) {
        for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 5;
             left_arm_ctrl_obs_B.nm1d2++) {
          left_arm_ctrl_obs_B.b_cu[left_arm_ctrl_obs_B.nm1d2] =
            tmp[left_arm_ctrl_obs_B.nm1d2];
        }
      }

      exitg1 = false;
      while ((!exitg1) && (lambda_->data[left_arm_ctrl_obs_B.q_size_tmp] > 0.0))
      {
        obj = robot->Bodies[static_cast<int32_T>(lambda_->
          data[left_arm_ctrl_obs_B.q_size_tmp]) - 1];
        left_arm_ctrl_obs_B.nm1d2 = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = obj->JointInternal.Type->size[1];
        left_a_emxEnsureCapacity_char_T(a, left_arm_ctrl_obs_B.nm1d2);
        left_arm_ctrl_obs_B.n_p = obj->JointInternal.Type->size[0] *
          obj->JointInternal.Type->size[1] - 1;
        for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
             left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
          a->data[left_arm_ctrl_obs_B.nm1d2] = obj->JointInternal.Type->
            data[left_arm_ctrl_obs_B.nm1d2];
        }

        left_arm_ctrl_obs_B.b_bool_p = false;
        if (a->size[1] == 5) {
          left_arm_ctrl_obs_B.nm1d2 = 1;
          do {
            exitg2 = 0;
            if (left_arm_ctrl_obs_B.nm1d2 - 1 < 5) {
              left_arm_ctrl_obs_B.n_p = left_arm_ctrl_obs_B.nm1d2 - 1;
              if (a->data[left_arm_ctrl_obs_B.n_p] !=
                  left_arm_ctrl_obs_B.b_cu[left_arm_ctrl_obs_B.n_p]) {
                exitg2 = 1;
              } else {
                left_arm_ctrl_obs_B.nm1d2++;
              }
            } else {
              left_arm_ctrl_obs_B.b_bool_p = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (left_arm_ctrl_obs_B.b_bool_p) {
          lambda_->data[left_arm_ctrl_obs_B.q_size_tmp] = robot->Bodies[
            static_cast<int32_T>(lambda_->data[left_arm_ctrl_obs_B.q_size_tmp])
            - 1]->ParentIndex;
        } else {
          exitg1 = true;
        }
      }
    }

    left_arm_ctrl_obs_B.b_idx_0_h = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp - 1];
    left_arm_ctrl_obs_B.b_idx_1_e = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp + 9];
    if (left_arm_ctrl_obs_B.b_idx_0_h <= left_arm_ctrl_obs_B.b_idx_1_e) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp];
      left_arm_ctrl_obs_B.nm1d2 = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(Si, left_arm_ctrl_obs_B.nm1d2);
      left_arm_ctrl_obs_B.n_p = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
           left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
        Si->data[left_arm_ctrl_obs_B.nm1d2] = obj->
          JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.nm1d2];
      }

      left_arm_ctrl_obs_B.n_p = Si->size[1] - 1;
      left_arm_ctrl_obs_B.nm1d2 = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.nm1d2);
      for (left_arm_ctrl_obs_B.b_j = 0; left_arm_ctrl_obs_B.b_j <=
           left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.b_j++) {
        left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_j * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
             left_arm_ctrl_obs_B.b_i_p++) {
          Fi->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_p) + 1]
            = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
             left_arm_ctrl_obs_B.b_i_p++) {
          left_arm_ctrl_obs_B.aoffset_n = left_arm_ctrl_obs_B.b_i_p * 6 - 1;
          left_arm_ctrl_obs_B.temp = Si->data[(left_arm_ctrl_obs_B.pid_tmp +
            left_arm_ctrl_obs_B.b_i_p) + 1];
          for (left_arm_ctrl_obs_B.c_i_n = 0; left_arm_ctrl_obs_B.c_i_n < 6;
               left_arm_ctrl_obs_B.c_i_n++) {
            left_arm_ctrl_obs_B.i_kt = left_arm_ctrl_obs_B.c_i_n + 1;
            left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.pid_tmp +
              left_arm_ctrl_obs_B.i_kt;
            Fi->data[left_arm_ctrl_obs_B.nm1d2] += Ic->
              data[left_arm_ctrl_obs_B.q_size_tmp]
              .f1[left_arm_ctrl_obs_B.aoffset_n + left_arm_ctrl_obs_B.i_kt] *
              left_arm_ctrl_obs_B.temp;
          }
        }
      }

      if (left_arm_ctrl_obs_B.vNum_f > left_arm_ctrl_obs_B.p_idx_1) {
        left_arm_ctrl_obs_B.pid_tmp = 0;
        left_arm_ctrl_obs_B.cb = 0;
      } else {
        left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.vNum_f) - 1;
        left_arm_ctrl_obs_B.cb = left_arm_ctrl_obs_B.pid_tmp;
      }

      left_arm_ctrl_obs_B.nm1d2 = a_0->size[0] * a_0->size[1];
      a_0->size[0] = Si->size[1];
      a_0->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a_0, left_arm_ctrl_obs_B.nm1d2);
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
           left_arm_ctrl_obs_B.nm1d2++) {
        left_arm_ctrl_obs_B.n_p = Si->size[1];
        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p <
             left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.b_i_p++) {
          a_0->data[left_arm_ctrl_obs_B.b_i_p + a_0->size[0] *
            left_arm_ctrl_obs_B.nm1d2] = Si->data[6 * left_arm_ctrl_obs_B.b_i_p
            + left_arm_ctrl_obs_B.nm1d2];
        }
      }

      left_arm_ctrl_obs_B.m_o = a_0->size[0];
      left_arm_ctrl_obs_B.n_p = Fi->size[1] - 1;
      left_arm_ctrl_obs_B.nm1d2 = Hji->size[0] * Hji->size[1];
      Hji->size[0] = a_0->size[0];
      Hji->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(Hji, left_arm_ctrl_obs_B.nm1d2);
      for (left_arm_ctrl_obs_B.b_j = 0; left_arm_ctrl_obs_B.b_j <=
           left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.b_j++) {
        left_arm_ctrl_obs_B.coffset = left_arm_ctrl_obs_B.b_j *
          left_arm_ctrl_obs_B.m_o - 1;
        left_arm_ctrl_obs_B.boffset = left_arm_ctrl_obs_B.b_j * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p <
             left_arm_ctrl_obs_B.m_o; left_arm_ctrl_obs_B.b_i_p++) {
          Hji->data[(left_arm_ctrl_obs_B.coffset + left_arm_ctrl_obs_B.b_i_p) +
            1] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
             left_arm_ctrl_obs_B.b_i_p++) {
          left_arm_ctrl_obs_B.aoffset_n = left_arm_ctrl_obs_B.b_i_p *
            left_arm_ctrl_obs_B.m_o - 1;
          left_arm_ctrl_obs_B.temp = Fi->data[(left_arm_ctrl_obs_B.boffset +
            left_arm_ctrl_obs_B.b_i_p) + 1];
          for (left_arm_ctrl_obs_B.c_i_n = 0; left_arm_ctrl_obs_B.c_i_n <
               left_arm_ctrl_obs_B.m_o; left_arm_ctrl_obs_B.c_i_n++) {
            left_arm_ctrl_obs_B.i_kt = left_arm_ctrl_obs_B.c_i_n + 1;
            left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.coffset +
              left_arm_ctrl_obs_B.i_kt;
            Hji->data[left_arm_ctrl_obs_B.nm1d2] += a_0->
              data[left_arm_ctrl_obs_B.aoffset_n + left_arm_ctrl_obs_B.i_kt] *
              left_arm_ctrl_obs_B.temp;
          }
        }
      }

      left_arm_ctrl_obs_B.n_p = Hji->size[1];
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <
           left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
        left_arm_ctrl_obs_B.b_j = Hji->size[0];
        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p <
             left_arm_ctrl_obs_B.b_j; left_arm_ctrl_obs_B.b_i_p++) {
          H->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_p) +
            H->size[0] * (left_arm_ctrl_obs_B.cb + left_arm_ctrl_obs_B.nm1d2)] =
            Hji->data[Hji->size[0] * left_arm_ctrl_obs_B.nm1d2 +
            left_arm_ctrl_obs_B.b_i_p];
        }
      }

      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
           left_arm_ctrl_obs_B.nm1d2++) {
        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
             left_arm_ctrl_obs_B.b_i_p++) {
          left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.b_i_p + 6 *
            left_arm_ctrl_obs_B.nm1d2] = X->data[left_arm_ctrl_obs_B.q_size_tmp]
            .f1[6 * left_arm_ctrl_obs_B.b_i_p + left_arm_ctrl_obs_B.nm1d2];
        }
      }

      left_arm_ctrl_obs_B.nm1d2 = B->size[0] * B->size[1];
      B->size[0] = 6;
      B->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(B, left_arm_ctrl_obs_B.nm1d2);
      left_arm_ctrl_obs_B.n_p = Fi->size[0] * Fi->size[1] - 1;
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
           left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
        B->data[left_arm_ctrl_obs_B.nm1d2] = Fi->data[left_arm_ctrl_obs_B.nm1d2];
      }

      left_arm_ctrl_obs_B.n_p = Fi->size[1];
      left_arm_ctrl_obs_B.nm1d2 = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = left_arm_ctrl_obs_B.n_p;
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.nm1d2);
      for (left_arm_ctrl_obs_B.b_j = 0; left_arm_ctrl_obs_B.b_j <
           left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.b_j++) {
        left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_j * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
             left_arm_ctrl_obs_B.b_i_p++) {
          Fi->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_p) + 1]
            = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
             left_arm_ctrl_obs_B.b_i_p++) {
          left_arm_ctrl_obs_B.aoffset_n = left_arm_ctrl_obs_B.b_i_p * 6 - 1;
          left_arm_ctrl_obs_B.temp = B->data[(left_arm_ctrl_obs_B.pid_tmp +
            left_arm_ctrl_obs_B.b_i_p) + 1];
          for (left_arm_ctrl_obs_B.c_i_n = 0; left_arm_ctrl_obs_B.c_i_n < 6;
               left_arm_ctrl_obs_B.c_i_n++) {
            left_arm_ctrl_obs_B.i_kt = left_arm_ctrl_obs_B.c_i_n + 1;
            left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.pid_tmp +
              left_arm_ctrl_obs_B.i_kt;
            Fi->data[left_arm_ctrl_obs_B.nm1d2] +=
              left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.aoffset_n +
              left_arm_ctrl_obs_B.i_kt] * left_arm_ctrl_obs_B.temp;
          }
        }
      }

      while (left_arm_ctrl_obs_B.pid > 0.0) {
        left_arm_ctrl_obs_B.b_i_p = static_cast<int32_T>(left_arm_ctrl_obs_B.pid);
        left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.b_i_p - 1;
        obj = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp];
        left_arm_ctrl_obs_B.nm1d2 = Sj->size[0] * Sj->size[1];
        Sj->size[0] = 6;
        Sj->size[1] = obj->JointInternal.MotionSubspace->size[1];
        left_a_emxEnsureCapacity_real_T(Sj, left_arm_ctrl_obs_B.nm1d2);
        left_arm_ctrl_obs_B.n_p = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
             left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
          Sj->data[left_arm_ctrl_obs_B.nm1d2] =
            obj->JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.nm1d2];
        }

        left_arm_ctrl_obs_B.b_idx_0_h = robot->
          VelocityDoFMap[left_arm_ctrl_obs_B.b_i_p - 1];
        left_arm_ctrl_obs_B.b_idx_1_e = robot->
          VelocityDoFMap[left_arm_ctrl_obs_B.b_i_p + 9];
        if (left_arm_ctrl_obs_B.b_idx_0_h <= left_arm_ctrl_obs_B.b_idx_1_e) {
          left_arm_ctrl_obs_B.nm1d2 = a_0->size[0] * a_0->size[1];
          a_0->size[0] = Sj->size[1];
          a_0->size[1] = 6;
          left_a_emxEnsureCapacity_real_T(a_0, left_arm_ctrl_obs_B.nm1d2);
          for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
               left_arm_ctrl_obs_B.nm1d2++) {
            left_arm_ctrl_obs_B.n_p = Sj->size[1];
            for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p <
                 left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.b_i_p++) {
              a_0->data[left_arm_ctrl_obs_B.b_i_p + a_0->size[0] *
                left_arm_ctrl_obs_B.nm1d2] = Sj->data[6 *
                left_arm_ctrl_obs_B.b_i_p + left_arm_ctrl_obs_B.nm1d2];
            }
          }

          left_arm_ctrl_obs_B.m_o = a_0->size[0];
          left_arm_ctrl_obs_B.n_p = Fi->size[1] - 1;
          left_arm_ctrl_obs_B.nm1d2 = Hji->size[0] * Hji->size[1];
          Hji->size[0] = a_0->size[0];
          Hji->size[1] = Fi->size[1];
          left_a_emxEnsureCapacity_real_T(Hji, left_arm_ctrl_obs_B.nm1d2);
          for (left_arm_ctrl_obs_B.b_j = 0; left_arm_ctrl_obs_B.b_j <=
               left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.b_j++) {
            left_arm_ctrl_obs_B.coffset = left_arm_ctrl_obs_B.b_j *
              left_arm_ctrl_obs_B.m_o - 1;
            left_arm_ctrl_obs_B.boffset = left_arm_ctrl_obs_B.b_j * 6 - 1;
            for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p <
                 left_arm_ctrl_obs_B.m_o; left_arm_ctrl_obs_B.b_i_p++) {
              Hji->data[(left_arm_ctrl_obs_B.coffset + left_arm_ctrl_obs_B.b_i_p)
                + 1] = 0.0;
            }

            for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
                 left_arm_ctrl_obs_B.b_i_p++) {
              left_arm_ctrl_obs_B.aoffset_n = left_arm_ctrl_obs_B.b_i_p *
                left_arm_ctrl_obs_B.m_o - 1;
              left_arm_ctrl_obs_B.temp = Fi->data[(left_arm_ctrl_obs_B.boffset +
                left_arm_ctrl_obs_B.b_i_p) + 1];
              for (left_arm_ctrl_obs_B.c_i_n = 0; left_arm_ctrl_obs_B.c_i_n <
                   left_arm_ctrl_obs_B.m_o; left_arm_ctrl_obs_B.c_i_n++) {
                left_arm_ctrl_obs_B.i_kt = left_arm_ctrl_obs_B.c_i_n + 1;
                left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.coffset +
                  left_arm_ctrl_obs_B.i_kt;
                Hji->data[left_arm_ctrl_obs_B.nm1d2] += a_0->
                  data[left_arm_ctrl_obs_B.aoffset_n + left_arm_ctrl_obs_B.i_kt]
                  * left_arm_ctrl_obs_B.temp;
              }
            }
          }

          if (left_arm_ctrl_obs_B.b_idx_0_h > left_arm_ctrl_obs_B.b_idx_1_e) {
            left_arm_ctrl_obs_B.pid_tmp = 0;
          } else {
            left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>
              (left_arm_ctrl_obs_B.b_idx_0_h) - 1;
          }

          if (left_arm_ctrl_obs_B.vNum_f > left_arm_ctrl_obs_B.p_idx_1) {
            left_arm_ctrl_obs_B.cb = 0;
          } else {
            left_arm_ctrl_obs_B.cb = static_cast<int32_T>
              (left_arm_ctrl_obs_B.vNum_f) - 1;
          }

          left_arm_ctrl_obs_B.n_p = Hji->size[1];
          for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <
               left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
            left_arm_ctrl_obs_B.b_j = Hji->size[0];
            for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p <
                 left_arm_ctrl_obs_B.b_j; left_arm_ctrl_obs_B.b_i_p++) {
              H->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_p)
                + H->size[0] * (left_arm_ctrl_obs_B.cb +
                                left_arm_ctrl_obs_B.nm1d2)] = Hji->data
                [Hji->size[0] * left_arm_ctrl_obs_B.nm1d2 +
                left_arm_ctrl_obs_B.b_i_p];
            }
          }

          if (left_arm_ctrl_obs_B.vNum_f > left_arm_ctrl_obs_B.p_idx_1) {
            left_arm_ctrl_obs_B.pid_tmp = 0;
          } else {
            left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>
              (left_arm_ctrl_obs_B.vNum_f) - 1;
          }

          if (left_arm_ctrl_obs_B.b_idx_0_h > left_arm_ctrl_obs_B.b_idx_1_e) {
            left_arm_ctrl_obs_B.cb = 0;
          } else {
            left_arm_ctrl_obs_B.cb = static_cast<int32_T>
              (left_arm_ctrl_obs_B.b_idx_0_h) - 1;
          }

          left_arm_ctrl_obs_B.n_p = Hji->size[0];
          for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <
               left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
            left_arm_ctrl_obs_B.b_j = Hji->size[1];
            for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p <
                 left_arm_ctrl_obs_B.b_j; left_arm_ctrl_obs_B.b_i_p++) {
              H->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_p)
                + H->size[0] * (left_arm_ctrl_obs_B.cb +
                                left_arm_ctrl_obs_B.nm1d2)] = Hji->data
                [Hji->size[0] * left_arm_ctrl_obs_B.b_i_p +
                left_arm_ctrl_obs_B.nm1d2];
            }
          }
        }

        for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
             left_arm_ctrl_obs_B.nm1d2++) {
          for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
               left_arm_ctrl_obs_B.b_i_p++) {
            left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.b_i_p + 6 *
              left_arm_ctrl_obs_B.nm1d2] = X->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 *
              left_arm_ctrl_obs_B.b_i_p + left_arm_ctrl_obs_B.nm1d2];
          }
        }

        left_arm_ctrl_obs_B.nm1d2 = B->size[0] * B->size[1];
        B->size[0] = 6;
        B->size[1] = Fi->size[1];
        left_a_emxEnsureCapacity_real_T(B, left_arm_ctrl_obs_B.nm1d2);
        left_arm_ctrl_obs_B.n_p = Fi->size[0] * Fi->size[1] - 1;
        for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
             left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
          B->data[left_arm_ctrl_obs_B.nm1d2] = Fi->
            data[left_arm_ctrl_obs_B.nm1d2];
        }

        left_arm_ctrl_obs_B.n_p = Fi->size[1];
        left_arm_ctrl_obs_B.nm1d2 = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = left_arm_ctrl_obs_B.n_p;
        left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.nm1d2);
        for (left_arm_ctrl_obs_B.b_j = 0; left_arm_ctrl_obs_B.b_j <
             left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.b_j++) {
          left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_j * 6 - 1;
          for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
               left_arm_ctrl_obs_B.b_i_p++) {
            Fi->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_p) +
              1] = 0.0;
          }

          for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p < 6;
               left_arm_ctrl_obs_B.b_i_p++) {
            left_arm_ctrl_obs_B.aoffset_n = left_arm_ctrl_obs_B.b_i_p * 6 - 1;
            left_arm_ctrl_obs_B.temp = B->data[(left_arm_ctrl_obs_B.pid_tmp +
              left_arm_ctrl_obs_B.b_i_p) + 1];
            for (left_arm_ctrl_obs_B.c_i_n = 0; left_arm_ctrl_obs_B.c_i_n < 6;
                 left_arm_ctrl_obs_B.c_i_n++) {
              left_arm_ctrl_obs_B.i_kt = left_arm_ctrl_obs_B.c_i_n + 1;
              left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.pid_tmp +
                left_arm_ctrl_obs_B.i_kt;
              Fi->data[left_arm_ctrl_obs_B.nm1d2] +=
                left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.aoffset_n +
                left_arm_ctrl_obs_B.i_kt] * left_arm_ctrl_obs_B.temp;
            }
          }
        }

        left_arm_ctrl_obs_B.pid = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp
          ]->ParentIndex;
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
  for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 10;
       left_arm_ctrl_obs_B.nm1d2++) {
    left_arm_ctrl_obs_B.mask[left_arm_ctrl_obs_B.nm1d2] = (robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.nm1d2] <= robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.nm1d2 + 10]);
  }

  left_arm_ctrl_obs_B.idx = 0;
  left_arm_ctrl_obs_B.nm1d2 = 1;
  exitg1 = false;
  while ((!exitg1) && (left_arm_ctrl_obs_B.nm1d2 - 1 < 10)) {
    if (left_arm_ctrl_obs_B.mask[left_arm_ctrl_obs_B.nm1d2 - 1]) {
      left_arm_ctrl_obs_B.idx++;
      left_arm_ctrl_obs_B.ii_data[left_arm_ctrl_obs_B.idx - 1] =
        left_arm_ctrl_obs_B.nm1d2;
      if (left_arm_ctrl_obs_B.idx >= 10) {
        exitg1 = true;
      } else {
        left_arm_ctrl_obs_B.nm1d2++;
      }
    } else {
      left_arm_ctrl_obs_B.nm1d2++;
    }
  }

  if (1 > left_arm_ctrl_obs_B.idx) {
    left_arm_ctrl_obs_B.idx = 0;
  }

  for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <
       left_arm_ctrl_obs_B.idx; left_arm_ctrl_obs_B.nm1d2++) {
    left_arm_ctrl_obs_B.nonFixedIndices_data[left_arm_ctrl_obs_B.nm1d2] =
      left_arm_ctrl_obs_B.ii_data[left_arm_ctrl_obs_B.nm1d2];
  }

  left_arm_ctrl_obs_B.idx--;
  left_arm_ctrl_ob_emxInit_real_T(&s, 2);
  for (left_arm_ctrl_obs_B.unnamed_idx_1_c = 0;
       left_arm_ctrl_obs_B.unnamed_idx_1_c <= left_arm_ctrl_obs_B.idx;
       left_arm_ctrl_obs_B.unnamed_idx_1_c++) {
    left_arm_ctrl_obs_B.vNum_f = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.nonFixedIndices_data[left_arm_ctrl_obs_B.unnamed_idx_1_c]
      - 1];
    left_arm_ctrl_obs_B.p_idx_1 = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.nonFixedIndices_data[left_arm_ctrl_obs_B.unnamed_idx_1_c]
      + 9];
    if (rtIsNaN(left_arm_ctrl_obs_B.vNum_f) || rtIsNaN
        (left_arm_ctrl_obs_B.p_idx_1)) {
      left_arm_ctrl_obs_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_ctrl_obs_B.nm1d2);
      s->data[0] = (rtNaN);
    } else if (left_arm_ctrl_obs_B.p_idx_1 < left_arm_ctrl_obs_B.vNum_f) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(left_arm_ctrl_obs_B.vNum_f) || rtIsInf
                (left_arm_ctrl_obs_B.p_idx_1)) && (left_arm_ctrl_obs_B.vNum_f ==
                left_arm_ctrl_obs_B.p_idx_1)) {
      left_arm_ctrl_obs_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_ctrl_obs_B.nm1d2);
      s->data[0] = (rtNaN);
    } else if (floor(left_arm_ctrl_obs_B.vNum_f) == left_arm_ctrl_obs_B.vNum_f)
    {
      left_arm_ctrl_obs_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      left_arm_ctrl_obs_B.n_p = static_cast<int32_T>(floor
        (left_arm_ctrl_obs_B.p_idx_1 - left_arm_ctrl_obs_B.vNum_f));
      s->size[1] = left_arm_ctrl_obs_B.n_p + 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_ctrl_obs_B.nm1d2);
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
           left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
        s->data[left_arm_ctrl_obs_B.nm1d2] = left_arm_ctrl_obs_B.vNum_f +
          static_cast<real_T>(left_arm_ctrl_obs_B.nm1d2);
      }
    } else {
      left_arm_ctrl_obs_B.nb_e = floor((left_arm_ctrl_obs_B.p_idx_1 -
        left_arm_ctrl_obs_B.vNum_f) + 0.5);
      left_arm_ctrl_obs_B.pid = left_arm_ctrl_obs_B.vNum_f +
        left_arm_ctrl_obs_B.nb_e;
      left_arm_ctrl_obs_B.b_idx_0_h = left_arm_ctrl_obs_B.pid -
        left_arm_ctrl_obs_B.p_idx_1;
      left_arm_ctrl_obs_B.b_idx_1_e = fabs(left_arm_ctrl_obs_B.vNum_f);
      left_arm_ctrl_obs_B.temp = fabs(left_arm_ctrl_obs_B.p_idx_1);
      if ((left_arm_ctrl_obs_B.b_idx_1_e > left_arm_ctrl_obs_B.temp) || rtIsNaN
          (left_arm_ctrl_obs_B.temp)) {
        left_arm_ctrl_obs_B.temp = left_arm_ctrl_obs_B.b_idx_1_e;
      }

      if (fabs(left_arm_ctrl_obs_B.b_idx_0_h) < 4.4408920985006262E-16 *
          left_arm_ctrl_obs_B.temp) {
        left_arm_ctrl_obs_B.nb_e++;
        left_arm_ctrl_obs_B.pid = left_arm_ctrl_obs_B.p_idx_1;
      } else if (left_arm_ctrl_obs_B.b_idx_0_h > 0.0) {
        left_arm_ctrl_obs_B.pid = (left_arm_ctrl_obs_B.nb_e - 1.0) +
          left_arm_ctrl_obs_B.vNum_f;
      } else {
        left_arm_ctrl_obs_B.nb_e++;
      }

      if (left_arm_ctrl_obs_B.nb_e >= 0.0) {
        left_arm_ctrl_obs_B.nm1d2 = static_cast<int32_T>
          (left_arm_ctrl_obs_B.nb_e);
      } else {
        left_arm_ctrl_obs_B.nm1d2 = 0;
      }

      left_arm_ctrl_obs_B.n_p = left_arm_ctrl_obs_B.nm1d2 - 1;
      left_arm_ctrl_obs_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = left_arm_ctrl_obs_B.n_p + 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_ctrl_obs_B.nm1d2);
      if (left_arm_ctrl_obs_B.n_p + 1 > 0) {
        s->data[0] = left_arm_ctrl_obs_B.vNum_f;
        if (left_arm_ctrl_obs_B.n_p + 1 > 1) {
          s->data[left_arm_ctrl_obs_B.n_p] = left_arm_ctrl_obs_B.pid;
          left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.n_p / 2;
          left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.nm1d2 - 2;
          for (left_arm_ctrl_obs_B.b_i_p = 0; left_arm_ctrl_obs_B.b_i_p <=
               left_arm_ctrl_obs_B.q_size_tmp; left_arm_ctrl_obs_B.b_i_p++) {
            left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_i_p + 1;
            s->data[left_arm_ctrl_obs_B.pid_tmp] = left_arm_ctrl_obs_B.vNum_f +
              static_cast<real_T>(left_arm_ctrl_obs_B.pid_tmp);
            s->data[left_arm_ctrl_obs_B.n_p - left_arm_ctrl_obs_B.pid_tmp] =
              left_arm_ctrl_obs_B.pid - static_cast<real_T>
              (left_arm_ctrl_obs_B.pid_tmp);
          }

          if (left_arm_ctrl_obs_B.nm1d2 << 1 == left_arm_ctrl_obs_B.n_p) {
            s->data[left_arm_ctrl_obs_B.nm1d2] = (left_arm_ctrl_obs_B.vNum_f +
              left_arm_ctrl_obs_B.pid) / 2.0;
          } else {
            s->data[left_arm_ctrl_obs_B.nm1d2] = left_arm_ctrl_obs_B.vNum_f +
              static_cast<real_T>(left_arm_ctrl_obs_B.nm1d2);
            s->data[left_arm_ctrl_obs_B.nm1d2 + 1] = left_arm_ctrl_obs_B.pid -
              static_cast<real_T>(left_arm_ctrl_obs_B.nm1d2);
          }
        }
      }
    }

    if (left_arm_ctrl_obs_B.vNum_f > left_arm_ctrl_obs_B.p_idx_1) {
      left_arm_ctrl_obs_B.q_size_tmp = 0;
    } else {
      left_arm_ctrl_obs_B.q_size_tmp = static_cast<int32_T>
        (left_arm_ctrl_obs_B.vNum_f) - 1;
    }

    left_arm_ctrl_obs_B.n_p = s->size[1];
    for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <
         left_arm_ctrl_obs_B.n_p; left_arm_ctrl_obs_B.nm1d2++) {
      lambda->data[left_arm_ctrl_obs_B.q_size_tmp + left_arm_ctrl_obs_B.nm1d2] =
        s->data[left_arm_ctrl_obs_B.nm1d2] - 1.0;
    }

    if (lambda_->
        data[left_arm_ctrl_obs_B.nonFixedIndices_data[left_arm_ctrl_obs_B.unnamed_idx_1_c]
        - 1] == 0.0) {
      lambda->data[static_cast<int32_T>(left_arm_ctrl_obs_B.vNum_f) - 1] = 0.0;
    } else {
      left_arm_ctrl_obs_B.nm1d2 = static_cast<int32_T>(lambda_->
        data[left_arm_ctrl_obs_B.nonFixedIndices_data[left_arm_ctrl_obs_B.unnamed_idx_1_c]
        - 1]);
      left_arm_ctrl_obs_B.b_idx_1_e = robot->
        VelocityDoFMap[left_arm_ctrl_obs_B.nm1d2 + 9];
      lambda->data[static_cast<int32_T>(left_arm_ctrl_obs_B.vNum_f) - 1] =
        left_arm_ctrl_obs_B.b_idx_1_e;
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&s);
  left_arm_ctrl_ob_emxFree_real_T(&lambda_);
}

static void RigidBodyTreeDynamics_inverseDy(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], const real_T qdot[7], const
  emxArray_real_T_left_arm_ctrl_T *qddot, const real_T fext[60], real_T tau[7])
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
  left_arm_ctrl_obs_B.a0_m[0] = 0.0;
  left_arm_ctrl_obs_B.a0_m[1] = 0.0;
  left_arm_ctrl_obs_B.a0_m[2] = 0.0;
  left_arm_ctrl_obs_B.a0_m[3] = -robot->Gravity[0];
  left_arm_ctrl_obs_B.a0_m[4] = -robot->Gravity[1];
  left_arm_ctrl_obs_B.a0_m[5] = -robot->Gravity[2];
  left_arm_ctrl_ob_emxInit_real_T(&vJ, 2);
  left_arm_ctrl_obs_B.nb_c = robot->NumBodies;
  left_arm_ctrl_obs_B.i_f = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  left_arm_ctrl_obs_B.unnamed_idx_1_a = static_cast<int32_T>
    (left_arm_ctrl_obs_B.nb_c);
  vJ->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_a;
  left_a_emxEnsureCapacity_real_T(vJ, left_arm_ctrl_obs_B.i_f);
  left_arm_ctrl_obs_B.loop_ub_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a - 1;
  for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f <=
       left_arm_ctrl_obs_B.loop_ub_tmp; left_arm_ctrl_obs_B.i_f++) {
    vJ->data[left_arm_ctrl_obs_B.i_f] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&vB, 2);
  left_arm_ctrl_obs_B.i_f = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_a;
  left_a_emxEnsureCapacity_real_T(vB, left_arm_ctrl_obs_B.i_f);
  for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f <=
       left_arm_ctrl_obs_B.loop_ub_tmp; left_arm_ctrl_obs_B.i_f++) {
    vB->data[left_arm_ctrl_obs_B.i_f] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&aB, 2);
  left_arm_ctrl_obs_B.i_f = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_a;
  left_a_emxEnsureCapacity_real_T(aB, left_arm_ctrl_obs_B.i_f);
  for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f <=
       left_arm_ctrl_obs_B.loop_ub_tmp; left_arm_ctrl_obs_B.i_f++) {
    aB->data[left_arm_ctrl_obs_B.i_f] = 0.0;
  }

  for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 7;
       left_arm_ctrl_obs_B.i_f++) {
    tau[left_arm_ctrl_obs_B.i_f] = 0.0;
  }

  left_arm_ct_emxInit_f_cell_wrap(&X, 2);
  left_arm_ct_emxInit_f_cell_wrap(&Xtree, 2);
  left_arm_ctrl_obs_B.loop_ub_tmp = left_arm_ctrl_obs_B.unnamed_idx_1_a - 1;
  left_arm_ctrl_obs_B.i_f = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_a;
  l_emxEnsureCapacity_f_cell_wrap(Xtree, left_arm_ctrl_obs_B.i_f);
  left_arm_ctrl_obs_B.i_f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_a;
  l_emxEnsureCapacity_f_cell_wrap(X, left_arm_ctrl_obs_B.i_f);
  for (left_arm_ctrl_obs_B.b_k_j = 0; left_arm_ctrl_obs_B.b_k_j <=
       left_arm_ctrl_obs_B.loop_ub_tmp; left_arm_ctrl_obs_B.b_k_j++) {
    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 36;
         left_arm_ctrl_obs_B.i_f++) {
      Xtree->data[left_arm_ctrl_obs_B.b_k_j].f1[left_arm_ctrl_obs_B.i_f] = 0.0;
    }

    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
         left_arm_ctrl_obs_B.i_f++) {
      Xtree->data[left_arm_ctrl_obs_B.b_k_j].f1[left_arm_ctrl_obs_B.i_f + 6 *
        left_arm_ctrl_obs_B.i_f] = 1.0;
    }

    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 36;
         left_arm_ctrl_obs_B.i_f++) {
      X->data[left_arm_ctrl_obs_B.b_k_j].f1[left_arm_ctrl_obs_B.i_f] = 0.0;
    }

    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
         left_arm_ctrl_obs_B.i_f++) {
      X->data[left_arm_ctrl_obs_B.b_k_j].f1[left_arm_ctrl_obs_B.i_f + 6 *
        left_arm_ctrl_obs_B.i_f] = 1.0;
    }
  }

  left_arm_ctrl_ob_emxInit_real_T(&f, 2);
  left_arm_ctrl_obs_B.i_f = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_a;
  left_a_emxEnsureCapacity_real_T(f, left_arm_ctrl_obs_B.i_f);
  left_arm_ctrl_ob_emxInit_real_T(&S, 2);
  left_arm_ctrl_ob_emxInit_real_T(&qddoti, 1);
  if (0 <= left_arm_ctrl_obs_B.loop_ub_tmp) {
    left_arm_ctrl_obs_B.dv3[0] = 0.0;
    left_arm_ctrl_obs_B.dv3[4] = 0.0;
    left_arm_ctrl_obs_B.dv3[8] = 0.0;
  }

  for (left_arm_ctrl_obs_B.unnamed_idx_1_a = 0;
       left_arm_ctrl_obs_B.unnamed_idx_1_a <= left_arm_ctrl_obs_B.loop_ub_tmp;
       left_arm_ctrl_obs_B.unnamed_idx_1_a++) {
    obj = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1_a];
    left_arm_ctrl_obs_B.i_f = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    left_a_emxEnsureCapacity_real_T(S, left_arm_ctrl_obs_B.i_f);
    left_arm_ctrl_obs_B.b_k_j = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f <=
         left_arm_ctrl_obs_B.b_k_j; left_arm_ctrl_obs_B.i_f++) {
      S->data[left_arm_ctrl_obs_B.i_f] = obj->JointInternal.MotionSubspace->
        data[left_arm_ctrl_obs_B.i_f];
    }

    left_arm_ctrl_obs_B.a_idx_0_p = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1_a];
    left_arm_ctrl_obs_B.a_idx_1_k = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1_a + 10];
    left_arm_ctrl_obs_B.b_idx_0_p = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1_a];
    left_arm_ctrl_obs_B.b_idx_1_p = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1_a + 10];
    if (left_arm_ctrl_obs_B.a_idx_1_k < left_arm_ctrl_obs_B.a_idx_0_p) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1_a];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        left_arm_ctrl_obs_B.T_c);
      left_arm_ctrl_obs_B.i_f = qddoti->size[0];
      qddoti->size[0] = 1;
      left_a_emxEnsureCapacity_real_T(qddoti, left_arm_ctrl_obs_B.i_f);
      qddoti->data[0] = 0.0;
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        vJ->data[left_arm_ctrl_obs_B.i_f + 6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_a] = 0.0;
      }
    } else {
      if (left_arm_ctrl_obs_B.a_idx_0_p > left_arm_ctrl_obs_B.a_idx_1_k) {
        left_arm_ctrl_obs_B.inner_m = 0;
        left_arm_ctrl_obs_B.m_e = -1;
      } else {
        left_arm_ctrl_obs_B.inner_m = static_cast<int32_T>
          (left_arm_ctrl_obs_B.a_idx_0_p) - 1;
        left_arm_ctrl_obs_B.m_e = static_cast<int32_T>
          (left_arm_ctrl_obs_B.a_idx_1_k) - 1;
      }

      if (left_arm_ctrl_obs_B.b_idx_0_p > left_arm_ctrl_obs_B.b_idx_1_p) {
        left_arm_ctrl_obs_B.p_tmp = 0;
        left_arm_ctrl_obs_B.o_tmp = 0;
        left_arm_ctrl_obs_B.aoffset_m = 0;
        left_arm_ctrl_obs_B.b_k_j = -1;
      } else {
        left_arm_ctrl_obs_B.p_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_0_p) - 1;
        left_arm_ctrl_obs_B.o_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_1_p);
        left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.p_tmp;
        left_arm_ctrl_obs_B.b_k_j = left_arm_ctrl_obs_B.o_tmp - 1;
      }

      left_arm_ctrl_obs_B.i_f = qddoti->size[0];
      left_arm_ctrl_obs_B.b_k_j -= left_arm_ctrl_obs_B.aoffset_m;
      qddoti->size[0] = left_arm_ctrl_obs_B.b_k_j + 1;
      left_a_emxEnsureCapacity_real_T(qddoti, left_arm_ctrl_obs_B.i_f);
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f <=
           left_arm_ctrl_obs_B.b_k_j; left_arm_ctrl_obs_B.i_f++) {
        qddoti->data[left_arm_ctrl_obs_B.i_f] = qddot->
          data[left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_f];
      }

      obj = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1_a];
      left_arm_ctrl_obs_B.m_e -= left_arm_ctrl_obs_B.inner_m;
      left_arm_ctrl_obs_B.q_size = left_arm_ctrl_obs_B.m_e + 1;
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f <=
           left_arm_ctrl_obs_B.m_e; left_arm_ctrl_obs_B.i_f++) {
        left_arm_ctrl_obs_B.q_data[left_arm_ctrl_obs_B.i_f] =
          q[left_arm_ctrl_obs_B.inner_m + left_arm_ctrl_obs_B.i_f];
      }

      rigidBodyJoint_transformBodyT_e(&obj->JointInternal,
        left_arm_ctrl_obs_B.q_data, &left_arm_ctrl_obs_B.q_size,
        left_arm_ctrl_obs_B.T_c);
      if ((S->size[1] == 1) || (left_arm_ctrl_obs_B.o_tmp -
           left_arm_ctrl_obs_B.p_tmp == 1)) {
        for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
             left_arm_ctrl_obs_B.i_f++) {
          left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.i_f + 6 *
            left_arm_ctrl_obs_B.unnamed_idx_1_a;
          vJ->data[left_arm_ctrl_obs_B.aoffset_m] = 0.0;
          left_arm_ctrl_obs_B.b_k_j = S->size[1];
          for (left_arm_ctrl_obs_B.inner_m = 0; left_arm_ctrl_obs_B.inner_m <
               left_arm_ctrl_obs_B.b_k_j; left_arm_ctrl_obs_B.inner_m++) {
            vJ->data[left_arm_ctrl_obs_B.aoffset_m] += S->data[6 *
              left_arm_ctrl_obs_B.inner_m + left_arm_ctrl_obs_B.i_f] *
              qdot[left_arm_ctrl_obs_B.p_tmp + left_arm_ctrl_obs_B.inner_m];
          }
        }
      } else {
        left_arm_ctrl_obs_B.inner_m = S->size[1] - 1;
        for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
             left_arm_ctrl_obs_B.i_f++) {
          vJ->data[left_arm_ctrl_obs_B.i_f + 6 *
            left_arm_ctrl_obs_B.unnamed_idx_1_a] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_k_j = 0; left_arm_ctrl_obs_B.b_k_j <=
             left_arm_ctrl_obs_B.inner_m; left_arm_ctrl_obs_B.b_k_j++) {
          left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.b_k_j * 6 - 1;
          for (left_arm_ctrl_obs_B.c_i_j = 0; left_arm_ctrl_obs_B.c_i_j < 6;
               left_arm_ctrl_obs_B.c_i_j++) {
            left_arm_ctrl_obs_B.i_f = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a +
              left_arm_ctrl_obs_B.c_i_j;
            vJ->data[left_arm_ctrl_obs_B.i_f] += S->data
              [(left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.c_i_j) + 1] *
              qdot[left_arm_ctrl_obs_B.p_tmp + left_arm_ctrl_obs_B.b_k_j];
          }
        }
      }
    }

    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 3;
         left_arm_ctrl_obs_B.i_f++) {
      left_arm_ctrl_obs_B.R_h[3 * left_arm_ctrl_obs_B.i_f] =
        left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.i_f];
      left_arm_ctrl_obs_B.R_h[3 * left_arm_ctrl_obs_B.i_f + 1] =
        left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.i_f + 4];
      left_arm_ctrl_obs_B.R_h[3 * left_arm_ctrl_obs_B.i_f + 2] =
        left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.i_f + 8];
    }

    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 9;
         left_arm_ctrl_obs_B.i_f++) {
      left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i_f] =
        -left_arm_ctrl_obs_B.R_h[left_arm_ctrl_obs_B.i_f];
    }

    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 3;
         left_arm_ctrl_obs_B.i_f++) {
      left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.i_f << 2;
      left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp] =
        left_arm_ctrl_obs_B.R_h[3 * left_arm_ctrl_obs_B.i_f];
      left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp + 1] =
        left_arm_ctrl_obs_B.R_h[3 * left_arm_ctrl_obs_B.i_f + 1];
      left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp + 2] =
        left_arm_ctrl_obs_B.R_h[3 * left_arm_ctrl_obs_B.i_f + 2];
      left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.i_f + 12] =
        left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i_f + 6] *
        left_arm_ctrl_obs_B.T_c[14] +
        (left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i_f + 3] *
         left_arm_ctrl_obs_B.T_c[13] +
         left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i_f] *
         left_arm_ctrl_obs_B.T_c[12]);
    }

    left_arm_ctrl_obs_B.Tinv[3] = 0.0;
    left_arm_ctrl_obs_B.Tinv[7] = 0.0;
    left_arm_ctrl_obs_B.Tinv[11] = 0.0;
    left_arm_ctrl_obs_B.Tinv[15] = 1.0;
    left_arm_ctrl_obs_B.dv3[3] = -left_arm_ctrl_obs_B.Tinv[14];
    left_arm_ctrl_obs_B.dv3[6] = left_arm_ctrl_obs_B.Tinv[13];
    left_arm_ctrl_obs_B.dv3[1] = left_arm_ctrl_obs_B.Tinv[14];
    left_arm_ctrl_obs_B.dv3[7] = -left_arm_ctrl_obs_B.Tinv[12];
    left_arm_ctrl_obs_B.dv3[2] = -left_arm_ctrl_obs_B.Tinv[13];
    left_arm_ctrl_obs_B.dv3[5] = left_arm_ctrl_obs_B.Tinv[12];
    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 3;
         left_arm_ctrl_obs_B.i_f++) {
      for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m < 3;
           left_arm_ctrl_obs_B.aoffset_m++) {
        left_arm_ctrl_obs_B.inner_m = left_arm_ctrl_obs_B.i_f + 3 *
          left_arm_ctrl_obs_B.aoffset_m;
        left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.inner_m] = 0.0;
        left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.aoffset_m << 2;
        left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.inner_m] +=
          left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp] *
          left_arm_ctrl_obs_B.dv3[left_arm_ctrl_obs_B.i_f];
        left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.inner_m] +=
          left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp + 1] *
          left_arm_ctrl_obs_B.dv3[left_arm_ctrl_obs_B.i_f + 3];
        left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.inner_m] +=
          left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp + 2] *
          left_arm_ctrl_obs_B.dv3[left_arm_ctrl_obs_B.i_f + 6];
        X->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
          .f1[left_arm_ctrl_obs_B.aoffset_m + 6 * left_arm_ctrl_obs_B.i_f] =
          left_arm_ctrl_obs_B.Tinv[(left_arm_ctrl_obs_B.i_f << 2) +
          left_arm_ctrl_obs_B.aoffset_m];
        X->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
          .f1[left_arm_ctrl_obs_B.aoffset_m + 6 * (left_arm_ctrl_obs_B.i_f + 3)]
          = 0.0;
      }
    }

    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 3;
         left_arm_ctrl_obs_B.i_f++) {
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_a].f1[6 *
        left_arm_ctrl_obs_B.i_f + 3] = left_arm_ctrl_obs_B.dv4[3 *
        left_arm_ctrl_obs_B.i_f];
      left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.i_f << 2;
      left_arm_ctrl_obs_B.inner_m = 6 * (left_arm_ctrl_obs_B.i_f + 3);
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
        .f1[left_arm_ctrl_obs_B.inner_m + 3] =
        left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.aoffset_m];
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_a].f1[6 *
        left_arm_ctrl_obs_B.i_f + 4] = left_arm_ctrl_obs_B.dv4[3 *
        left_arm_ctrl_obs_B.i_f + 1];
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
        .f1[left_arm_ctrl_obs_B.inner_m + 4] =
        left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.aoffset_m + 1];
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_a].f1[6 *
        left_arm_ctrl_obs_B.i_f + 5] = left_arm_ctrl_obs_B.dv4[3 *
        left_arm_ctrl_obs_B.i_f + 2];
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
        .f1[left_arm_ctrl_obs_B.inner_m + 5] =
        left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.aoffset_m + 2];
    }

    left_arm_ctrl_obs_B.a_idx_0_p = robot->
      Bodies[left_arm_ctrl_obs_B.unnamed_idx_1_a]->ParentIndex;
    if (left_arm_ctrl_obs_B.a_idx_0_p > 0.0) {
      left_arm_ctrl_obs_B.m_e = static_cast<int32_T>
        (left_arm_ctrl_obs_B.a_idx_0_p);
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        left_arm_ctrl_obs_B.a_idx_1_k = 0.0;
        for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
             6; left_arm_ctrl_obs_B.aoffset_m++) {
          left_arm_ctrl_obs_B.a_idx_1_k += vB->data[(left_arm_ctrl_obs_B.m_e - 1)
            * 6 + left_arm_ctrl_obs_B.aoffset_m] * X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1_a].f1[6 *
            left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_f];
        }

        left_arm_ctrl_obs_B.vJ[left_arm_ctrl_obs_B.i_f] = vJ->data[6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_a + left_arm_ctrl_obs_B.i_f] +
          left_arm_ctrl_obs_B.a_idx_1_k;
      }

      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        vB->data[left_arm_ctrl_obs_B.i_f + 6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_a] =
          left_arm_ctrl_obs_B.vJ[left_arm_ctrl_obs_B.i_f];
      }

      if ((S->size[1] == 1) || (qddoti->size[0] == 1)) {
        left_arm_ctrl_obs_B.b_k_j = S->size[1];
        for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
             left_arm_ctrl_obs_B.i_f++) {
          left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f] = 0.0;
          for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
               left_arm_ctrl_obs_B.b_k_j; left_arm_ctrl_obs_B.aoffset_m++) {
            left_arm_ctrl_obs_B.a_idx_1_k = S->data[6 *
              left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_f] *
              qddoti->data[left_arm_ctrl_obs_B.aoffset_m] +
              left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f];
            left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f] =
              left_arm_ctrl_obs_B.a_idx_1_k;
          }
        }
      } else {
        left_arm_ctrl_obs_B.inner_m = S->size[1] - 1;
        for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
             left_arm_ctrl_obs_B.i_f++) {
          left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_k_j = 0; left_arm_ctrl_obs_B.b_k_j <=
             left_arm_ctrl_obs_B.inner_m; left_arm_ctrl_obs_B.b_k_j++) {
          left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.b_k_j * 6 - 1;
          for (left_arm_ctrl_obs_B.c_i_j = 0; left_arm_ctrl_obs_B.c_i_j < 6;
               left_arm_ctrl_obs_B.c_i_j++) {
            left_arm_ctrl_obs_B.a_idx_1_k = S->data
              [(left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.c_i_j) + 1] *
              qddoti->data[left_arm_ctrl_obs_B.b_k_j] +
              left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.c_i_j];
            left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.c_i_j] =
              left_arm_ctrl_obs_B.a_idx_1_k;
          }
        }
      }

      left_arm_ctrl_obs_B.R_h[0] = 0.0;
      left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a + 2;
      left_arm_ctrl_obs_B.R_h[3] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.i_f = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a + 1;
      left_arm_ctrl_obs_B.R_h[6] = vB->data[left_arm_ctrl_obs_B.i_f];
      left_arm_ctrl_obs_B.R_h[1] = vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.R_h[4] = 0.0;
      left_arm_ctrl_obs_B.R_h[7] = -vB->data[6 *
        left_arm_ctrl_obs_B.unnamed_idx_1_a];
      left_arm_ctrl_obs_B.R_h[2] = -vB->data[left_arm_ctrl_obs_B.i_f];
      left_arm_ctrl_obs_B.R_h[5] = vB->data[6 *
        left_arm_ctrl_obs_B.unnamed_idx_1_a];
      left_arm_ctrl_obs_B.R_h[8] = 0.0;
      left_arm_ctrl_obs_B.b_I_c[3] = 0.0;
      left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a + 5;
      left_arm_ctrl_obs_B.b_I_c[9] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.i_f = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a + 4;
      left_arm_ctrl_obs_B.b_I_c[15] = vB->data[left_arm_ctrl_obs_B.i_f];
      left_arm_ctrl_obs_B.b_I_c[4] = vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.b_I_c[10] = 0.0;
      left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a + 3;
      left_arm_ctrl_obs_B.b_I_c[16] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.b_I_c[5] = -vB->data[left_arm_ctrl_obs_B.i_f];
      left_arm_ctrl_obs_B.b_I_c[11] = vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.b_I_c[17] = 0.0;
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 3;
           left_arm_ctrl_obs_B.i_f++) {
        left_arm_ctrl_obs_B.a_idx_1_k = left_arm_ctrl_obs_B.R_h[3 *
          left_arm_ctrl_obs_B.i_f];
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_f] =
          left_arm_ctrl_obs_B.a_idx_1_k;
        left_arm_ctrl_obs_B.p_tmp = 6 * (left_arm_ctrl_obs_B.i_f + 3);
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp] = 0.0;
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp + 3] =
          left_arm_ctrl_obs_B.a_idx_1_k;
        left_arm_ctrl_obs_B.a_idx_1_k = left_arm_ctrl_obs_B.R_h[3 *
          left_arm_ctrl_obs_B.i_f + 1];
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_f + 1] =
          left_arm_ctrl_obs_B.a_idx_1_k;
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp + 1] = 0.0;
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp + 4] =
          left_arm_ctrl_obs_B.a_idx_1_k;
        left_arm_ctrl_obs_B.a_idx_1_k = left_arm_ctrl_obs_B.R_h[3 *
          left_arm_ctrl_obs_B.i_f + 2];
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_f + 2] =
          left_arm_ctrl_obs_B.a_idx_1_k;
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp + 2] = 0.0;
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp + 5] =
          left_arm_ctrl_obs_B.a_idx_1_k;
      }

      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        left_arm_ctrl_obs_B.a_idx_1_k = 0.0;
        for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
             6; left_arm_ctrl_obs_B.aoffset_m++) {
          left_arm_ctrl_obs_B.a_idx_1_k += aB->data[(left_arm_ctrl_obs_B.m_e - 1)
            * 6 + left_arm_ctrl_obs_B.aoffset_m] * X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1_a].f1[6 *
            left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_f];
        }

        left_arm_ctrl_obs_B.vJ[left_arm_ctrl_obs_B.i_f] =
          left_arm_ctrl_obs_B.a_idx_1_k +
          left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f];
      }

      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f] = 0.0;
        for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
             6; left_arm_ctrl_obs_B.aoffset_m++) {
          left_arm_ctrl_obs_B.a_idx_1_k = left_arm_ctrl_obs_B.b_I_c[6 *
            left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_f] * vJ->data
            [6 * left_arm_ctrl_obs_B.unnamed_idx_1_a +
            left_arm_ctrl_obs_B.aoffset_m] +
            left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f];
          left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f] =
            left_arm_ctrl_obs_B.a_idx_1_k;
        }

        aB->data[left_arm_ctrl_obs_B.i_f + 6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_a] =
          left_arm_ctrl_obs_B.vJ[left_arm_ctrl_obs_B.i_f] +
          left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f];
      }

      left_arm_ctrl_obs_B.R_bn[0] = 0.0;
      left_arm_ctrl_obs_B.R_bn[3] = -left_arm_ctrl_obs_B.T_c[14];
      left_arm_ctrl_obs_B.R_bn[6] = left_arm_ctrl_obs_B.T_c[13];
      left_arm_ctrl_obs_B.R_bn[1] = left_arm_ctrl_obs_B.T_c[14];
      left_arm_ctrl_obs_B.R_bn[4] = 0.0;
      left_arm_ctrl_obs_B.R_bn[7] = -left_arm_ctrl_obs_B.T_c[12];
      left_arm_ctrl_obs_B.R_bn[2] = -left_arm_ctrl_obs_B.T_c[13];
      left_arm_ctrl_obs_B.R_bn[5] = left_arm_ctrl_obs_B.T_c[12];
      left_arm_ctrl_obs_B.R_bn[8] = 0.0;
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 3;
           left_arm_ctrl_obs_B.i_f++) {
        for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
             3; left_arm_ctrl_obs_B.aoffset_m++) {
          left_arm_ctrl_obs_B.inner_m = left_arm_ctrl_obs_B.i_f + 3 *
            left_arm_ctrl_obs_B.aoffset_m;
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_m] = 0.0;
          left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.aoffset_m << 2;
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_m] +=
            left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.p_tmp] *
            left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i_f];
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_m] +=
            left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.p_tmp + 1] *
            left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i_f + 3];
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_m] +=
            left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.p_tmp + 2] *
            left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i_f + 6];
          left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.aoffset_m + 6 *
            left_arm_ctrl_obs_B.i_f] = left_arm_ctrl_obs_B.T_c
            [(left_arm_ctrl_obs_B.i_f << 2) + left_arm_ctrl_obs_B.aoffset_m];
          left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.aoffset_m + 6 *
            (left_arm_ctrl_obs_B.i_f + 3)] = 0.0;
        }
      }

      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 3;
           left_arm_ctrl_obs_B.i_f++) {
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_f + 3] =
          left_arm_ctrl_obs_B.dv5[3 * left_arm_ctrl_obs_B.i_f];
        left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.i_f << 2;
        left_arm_ctrl_obs_B.inner_m = 6 * (left_arm_ctrl_obs_B.i_f + 3);
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.inner_m + 3] =
          left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.p_tmp];
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_f + 4] =
          left_arm_ctrl_obs_B.dv5[3 * left_arm_ctrl_obs_B.i_f + 1];
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.inner_m + 4] =
          left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.p_tmp + 1];
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_f + 5] =
          left_arm_ctrl_obs_B.dv5[3 * left_arm_ctrl_obs_B.i_f + 2];
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.inner_m + 5] =
          left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.p_tmp + 2];
      }

      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
             6; left_arm_ctrl_obs_B.aoffset_m++) {
          left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.i_f + 6 *
            left_arm_ctrl_obs_B.aoffset_m;
          left_arm_ctrl_obs_B.Xtree[left_arm_ctrl_obs_B.p_tmp] = 0.0;
          for (left_arm_ctrl_obs_B.inner_m = 0; left_arm_ctrl_obs_B.inner_m < 6;
               left_arm_ctrl_obs_B.inner_m++) {
            left_arm_ctrl_obs_B.Xtree[left_arm_ctrl_obs_B.p_tmp] += Xtree->data[
              static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0_p) - 1].f1[6 *
              left_arm_ctrl_obs_B.inner_m + left_arm_ctrl_obs_B.i_f] *
              left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.aoffset_m +
              left_arm_ctrl_obs_B.inner_m];
          }
        }
      }

      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 36;
           left_arm_ctrl_obs_B.i_f++) {
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
          .f1[left_arm_ctrl_obs_B.i_f] =
          left_arm_ctrl_obs_B.Xtree[left_arm_ctrl_obs_B.i_f];
      }
    } else {
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        left_arm_ctrl_obs_B.aoffset_m = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a
          + left_arm_ctrl_obs_B.i_f;
        vB->data[left_arm_ctrl_obs_B.aoffset_m] = vJ->
          data[left_arm_ctrl_obs_B.aoffset_m];
      }

      if ((S->size[1] == 1) || (qddoti->size[0] == 1)) {
        left_arm_ctrl_obs_B.b_k_j = S->size[1];
        for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
             left_arm_ctrl_obs_B.i_f++) {
          left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f] = 0.0;
          for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
               left_arm_ctrl_obs_B.b_k_j; left_arm_ctrl_obs_B.aoffset_m++) {
            left_arm_ctrl_obs_B.a_idx_1_k = S->data[6 *
              left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_f] *
              qddoti->data[left_arm_ctrl_obs_B.aoffset_m] +
              left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f];
            left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f] =
              left_arm_ctrl_obs_B.a_idx_1_k;
          }
        }
      } else {
        left_arm_ctrl_obs_B.inner_m = S->size[1] - 1;
        for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
             left_arm_ctrl_obs_B.i_f++) {
          left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_k_j = 0; left_arm_ctrl_obs_B.b_k_j <=
             left_arm_ctrl_obs_B.inner_m; left_arm_ctrl_obs_B.b_k_j++) {
          left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.b_k_j * 6 - 1;
          for (left_arm_ctrl_obs_B.c_i_j = 0; left_arm_ctrl_obs_B.c_i_j < 6;
               left_arm_ctrl_obs_B.c_i_j++) {
            left_arm_ctrl_obs_B.a_idx_1_k = S->data
              [(left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.c_i_j) + 1] *
              qddoti->data[left_arm_ctrl_obs_B.b_k_j] +
              left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.c_i_j];
            left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.c_i_j] =
              left_arm_ctrl_obs_B.a_idx_1_k;
          }
        }
      }

      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        left_arm_ctrl_obs_B.a_idx_1_k = 0.0;
        for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
             6; left_arm_ctrl_obs_B.aoffset_m++) {
          left_arm_ctrl_obs_B.a_idx_1_k += X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1_a].f1[6 *
            left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_f] *
            left_arm_ctrl_obs_B.a0_m[left_arm_ctrl_obs_B.aoffset_m];
        }

        aB->data[left_arm_ctrl_obs_B.i_f + 6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_a] = left_arm_ctrl_obs_B.a_idx_1_k +
          left_arm_ctrl_obs_B.y_j[left_arm_ctrl_obs_B.i_f];
      }

      left_arm_ctrl_obs_B.R_bn[0] = 0.0;
      left_arm_ctrl_obs_B.R_bn[3] = -left_arm_ctrl_obs_B.T_c[14];
      left_arm_ctrl_obs_B.R_bn[6] = left_arm_ctrl_obs_B.T_c[13];
      left_arm_ctrl_obs_B.R_bn[1] = left_arm_ctrl_obs_B.T_c[14];
      left_arm_ctrl_obs_B.R_bn[4] = 0.0;
      left_arm_ctrl_obs_B.R_bn[7] = -left_arm_ctrl_obs_B.T_c[12];
      left_arm_ctrl_obs_B.R_bn[2] = -left_arm_ctrl_obs_B.T_c[13];
      left_arm_ctrl_obs_B.R_bn[5] = left_arm_ctrl_obs_B.T_c[12];
      left_arm_ctrl_obs_B.R_bn[8] = 0.0;
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 3;
           left_arm_ctrl_obs_B.i_f++) {
        for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
             3; left_arm_ctrl_obs_B.aoffset_m++) {
          left_arm_ctrl_obs_B.inner_m = left_arm_ctrl_obs_B.i_f + 3 *
            left_arm_ctrl_obs_B.aoffset_m;
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_m] = 0.0;
          left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.aoffset_m << 2;
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_m] +=
            left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.p_tmp] *
            left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i_f];
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_m] +=
            left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.p_tmp + 1] *
            left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i_f + 3];
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_m] +=
            left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.p_tmp + 2] *
            left_arm_ctrl_obs_B.R_bn[left_arm_ctrl_obs_B.i_f + 6];
          Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
            .f1[left_arm_ctrl_obs_B.aoffset_m + 6 * left_arm_ctrl_obs_B.i_f] =
            left_arm_ctrl_obs_B.T_c[(left_arm_ctrl_obs_B.i_f << 2) +
            left_arm_ctrl_obs_B.aoffset_m];
          Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
            .f1[left_arm_ctrl_obs_B.aoffset_m + 6 * (left_arm_ctrl_obs_B.i_f + 3)]
            = 0.0;
        }
      }

      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 3;
           left_arm_ctrl_obs_B.i_f++) {
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_a].f1[6 *
          left_arm_ctrl_obs_B.i_f + 3] = left_arm_ctrl_obs_B.dv5[3 *
          left_arm_ctrl_obs_B.i_f];
        left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.i_f << 2;
        left_arm_ctrl_obs_B.inner_m = 6 * (left_arm_ctrl_obs_B.i_f + 3);
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
          .f1[left_arm_ctrl_obs_B.inner_m + 3] =
          left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.aoffset_m];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_a].f1[6 *
          left_arm_ctrl_obs_B.i_f + 4] = left_arm_ctrl_obs_B.dv5[3 *
          left_arm_ctrl_obs_B.i_f + 1];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
          .f1[left_arm_ctrl_obs_B.inner_m + 4] =
          left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.aoffset_m + 1];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_a].f1[6 *
          left_arm_ctrl_obs_B.i_f + 5] = left_arm_ctrl_obs_B.dv5[3 *
          left_arm_ctrl_obs_B.i_f + 2];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_a]
          .f1[left_arm_ctrl_obs_B.inner_m + 5] =
          left_arm_ctrl_obs_B.T_c[left_arm_ctrl_obs_B.aoffset_m + 2];
      }
    }

    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 36;
         left_arm_ctrl_obs_B.i_f++) {
      left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.i_f] = robot->
        Bodies[left_arm_ctrl_obs_B.unnamed_idx_1_a]->
        SpatialInertia[left_arm_ctrl_obs_B.i_f];
    }

    left_arm_ctrl_obs_B.R_h[0] = 0.0;
    left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a + 2;
    left_arm_ctrl_obs_B.R_h[3] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.i_f = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a + 1;
    left_arm_ctrl_obs_B.R_h[6] = vB->data[left_arm_ctrl_obs_B.i_f];
    left_arm_ctrl_obs_B.R_h[1] = vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.R_h[4] = 0.0;
    left_arm_ctrl_obs_B.R_h[7] = -vB->data[6 *
      left_arm_ctrl_obs_B.unnamed_idx_1_a];
    left_arm_ctrl_obs_B.R_h[2] = -vB->data[left_arm_ctrl_obs_B.i_f];
    left_arm_ctrl_obs_B.R_h[5] = vB->data[6 *
      left_arm_ctrl_obs_B.unnamed_idx_1_a];
    left_arm_ctrl_obs_B.R_h[8] = 0.0;
    left_arm_ctrl_obs_B.R[18] = 0.0;
    left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a + 5;
    left_arm_ctrl_obs_B.R[24] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.i_f = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a + 4;
    left_arm_ctrl_obs_B.R[30] = vB->data[left_arm_ctrl_obs_B.i_f];
    left_arm_ctrl_obs_B.R[19] = vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.R[25] = 0.0;
    left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a + 3;
    left_arm_ctrl_obs_B.R[31] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.R[20] = -vB->data[left_arm_ctrl_obs_B.i_f];
    left_arm_ctrl_obs_B.R[26] = vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.R[32] = 0.0;
    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 3;
         left_arm_ctrl_obs_B.i_f++) {
      left_arm_ctrl_obs_B.a_idx_1_k = left_arm_ctrl_obs_B.R_h[3 *
        left_arm_ctrl_obs_B.i_f];
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_f] =
        left_arm_ctrl_obs_B.a_idx_1_k;
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_f + 3] = 0.0;
      left_arm_ctrl_obs_B.p_tmp = 6 * (left_arm_ctrl_obs_B.i_f + 3);
      left_arm_ctrl_obs_B.R[left_arm_ctrl_obs_B.p_tmp + 3] =
        left_arm_ctrl_obs_B.a_idx_1_k;
      left_arm_ctrl_obs_B.a_idx_1_k = left_arm_ctrl_obs_B.R_h[3 *
        left_arm_ctrl_obs_B.i_f + 1];
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_f + 1] =
        left_arm_ctrl_obs_B.a_idx_1_k;
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_f + 4] = 0.0;
      left_arm_ctrl_obs_B.R[left_arm_ctrl_obs_B.p_tmp + 4] =
        left_arm_ctrl_obs_B.a_idx_1_k;
      left_arm_ctrl_obs_B.a_idx_1_k = left_arm_ctrl_obs_B.R_h[3 *
        left_arm_ctrl_obs_B.i_f + 2];
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_f + 2] =
        left_arm_ctrl_obs_B.a_idx_1_k;
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_f + 5] = 0.0;
      left_arm_ctrl_obs_B.R[left_arm_ctrl_obs_B.p_tmp + 5] =
        left_arm_ctrl_obs_B.a_idx_1_k;
    }

    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
         left_arm_ctrl_obs_B.i_f++) {
      left_arm_ctrl_obs_B.b_I_h[left_arm_ctrl_obs_B.i_f] = 0.0;
      left_arm_ctrl_obs_B.b_I_c0[left_arm_ctrl_obs_B.i_f] = 0.0;
      for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m < 6;
           left_arm_ctrl_obs_B.aoffset_m++) {
        left_arm_ctrl_obs_B.a_idx_0_p = left_arm_ctrl_obs_B.b_I_c[6 *
          left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_f];
        left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a +
          left_arm_ctrl_obs_B.aoffset_m;
        left_arm_ctrl_obs_B.a_idx_1_k = vB->data[left_arm_ctrl_obs_B.p_tmp] *
          left_arm_ctrl_obs_B.a_idx_0_p +
          left_arm_ctrl_obs_B.b_I_h[left_arm_ctrl_obs_B.i_f];
        left_arm_ctrl_obs_B.a_idx_0_p = aB->data[left_arm_ctrl_obs_B.p_tmp] *
          left_arm_ctrl_obs_B.a_idx_0_p +
          left_arm_ctrl_obs_B.b_I_c0[left_arm_ctrl_obs_B.i_f];
        left_arm_ctrl_obs_B.b_I_h[left_arm_ctrl_obs_B.i_f] =
          left_arm_ctrl_obs_B.a_idx_1_k;
        left_arm_ctrl_obs_B.b_I_c0[left_arm_ctrl_obs_B.i_f] =
          left_arm_ctrl_obs_B.a_idx_0_p;
      }
    }

    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
         left_arm_ctrl_obs_B.i_f++) {
      left_arm_ctrl_obs_B.R_ct[left_arm_ctrl_obs_B.i_f] = 0.0;
      left_arm_ctrl_obs_B.a_idx_1_k = 0.0;
      for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m < 6;
           left_arm_ctrl_obs_B.aoffset_m++) {
        left_arm_ctrl_obs_B.a_idx_1_k += Xtree->
          data[left_arm_ctrl_obs_B.unnamed_idx_1_a].f1[6 *
          left_arm_ctrl_obs_B.i_f + left_arm_ctrl_obs_B.aoffset_m] * fext[6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_a + left_arm_ctrl_obs_B.aoffset_m];
        left_arm_ctrl_obs_B.R_ct[left_arm_ctrl_obs_B.i_f] +=
          left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.aoffset_m +
          left_arm_ctrl_obs_B.i_f] *
          left_arm_ctrl_obs_B.b_I_h[left_arm_ctrl_obs_B.aoffset_m];
      }

      f->data[left_arm_ctrl_obs_B.i_f + 6 * left_arm_ctrl_obs_B.unnamed_idx_1_a]
        = (left_arm_ctrl_obs_B.b_I_c0[left_arm_ctrl_obs_B.i_f] +
           left_arm_ctrl_obs_B.R_ct[left_arm_ctrl_obs_B.i_f]) -
        left_arm_ctrl_obs_B.a_idx_1_k;
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&aB);
  left_arm_ctrl_ob_emxFree_real_T(&vB);
  left_arm_ctrl_ob_emxFree_real_T(&vJ);
  left_arm_ct_emxFree_f_cell_wrap(&Xtree);
  left_arm_ctrl_obs_B.loop_ub_tmp = static_cast<int32_T>(((-1.0 -
    left_arm_ctrl_obs_B.nb_c) + 1.0) / -1.0) - 1;
  left_arm_ctrl_ob_emxInit_char_T(&a, 2);
  left_arm_ctrl_ob_emxInit_real_T(&a_0, 2);
  if (0 <= left_arm_ctrl_obs_B.loop_ub_tmp) {
    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 5;
         left_arm_ctrl_obs_B.i_f++) {
      left_arm_ctrl_obs_B.b_oc[left_arm_ctrl_obs_B.i_f] =
        tmp[left_arm_ctrl_obs_B.i_f];
    }
  }

  for (left_arm_ctrl_obs_B.p_tmp = 0; left_arm_ctrl_obs_B.p_tmp <=
       left_arm_ctrl_obs_B.loop_ub_tmp; left_arm_ctrl_obs_B.p_tmp++) {
    left_arm_ctrl_obs_B.a_idx_0_p = left_arm_ctrl_obs_B.nb_c +
      -static_cast<real_T>(left_arm_ctrl_obs_B.p_tmp);
    left_arm_ctrl_obs_B.inner_m = static_cast<int32_T>
      (left_arm_ctrl_obs_B.a_idx_0_p);
    left_arm_ctrl_obs_B.o_tmp = left_arm_ctrl_obs_B.inner_m - 1;
    obj = robot->Bodies[left_arm_ctrl_obs_B.o_tmp];
    left_arm_ctrl_obs_B.i_f = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    left_a_emxEnsureCapacity_char_T(a, left_arm_ctrl_obs_B.i_f);
    left_arm_ctrl_obs_B.b_k_j = obj->JointInternal.Type->size[0] *
      obj->JointInternal.Type->size[1] - 1;
    for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f <=
         left_arm_ctrl_obs_B.b_k_j; left_arm_ctrl_obs_B.i_f++) {
      a->data[left_arm_ctrl_obs_B.i_f] = obj->JointInternal.Type->
        data[left_arm_ctrl_obs_B.i_f];
    }

    left_arm_ctrl_obs_B.b_bool_d = false;
    if (a->size[1] == 5) {
      left_arm_ctrl_obs_B.i_f = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.i_f - 1 < 5) {
          left_arm_ctrl_obs_B.unnamed_idx_1_a = left_arm_ctrl_obs_B.i_f - 1;
          if (a->data[left_arm_ctrl_obs_B.unnamed_idx_1_a] !=
              left_arm_ctrl_obs_B.b_oc[left_arm_ctrl_obs_B.unnamed_idx_1_a]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.i_f++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_d = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!left_arm_ctrl_obs_B.b_bool_d) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.o_tmp];
      left_arm_ctrl_obs_B.i_f = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(S, left_arm_ctrl_obs_B.i_f);
      left_arm_ctrl_obs_B.b_k_j = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f <=
           left_arm_ctrl_obs_B.b_k_j; left_arm_ctrl_obs_B.i_f++) {
        S->data[left_arm_ctrl_obs_B.i_f] = obj->
          JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.i_f];
      }

      left_arm_ctrl_obs_B.i_f = a_0->size[0] * a_0->size[1];
      a_0->size[0] = S->size[1];
      a_0->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a_0, left_arm_ctrl_obs_B.i_f);
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        left_arm_ctrl_obs_B.b_k_j = S->size[1];
        for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
             left_arm_ctrl_obs_B.b_k_j; left_arm_ctrl_obs_B.aoffset_m++) {
          a_0->data[left_arm_ctrl_obs_B.aoffset_m + a_0->size[0] *
            left_arm_ctrl_obs_B.i_f] = S->data[6 * left_arm_ctrl_obs_B.aoffset_m
            + left_arm_ctrl_obs_B.i_f];
        }
      }

      left_arm_ctrl_obs_B.m_e = a_0->size[0] - 1;
      left_arm_ctrl_obs_B.i_f = qddoti->size[0];
      qddoti->size[0] = a_0->size[0];
      left_a_emxEnsureCapacity_real_T(qddoti, left_arm_ctrl_obs_B.i_f);
      for (left_arm_ctrl_obs_B.unnamed_idx_1_a = 0;
           left_arm_ctrl_obs_B.unnamed_idx_1_a <= left_arm_ctrl_obs_B.m_e;
           left_arm_ctrl_obs_B.unnamed_idx_1_a++) {
        qddoti->data[left_arm_ctrl_obs_B.unnamed_idx_1_a] = 0.0;
      }

      for (left_arm_ctrl_obs_B.b_k_j = 0; left_arm_ctrl_obs_B.b_k_j < 6;
           left_arm_ctrl_obs_B.b_k_j++) {
        left_arm_ctrl_obs_B.aoffset_m = (left_arm_ctrl_obs_B.m_e + 1) *
          left_arm_ctrl_obs_B.b_k_j - 1;
        for (left_arm_ctrl_obs_B.c_i_j = 0; left_arm_ctrl_obs_B.c_i_j <=
             left_arm_ctrl_obs_B.m_e; left_arm_ctrl_obs_B.c_i_j++) {
          qddoti->data[left_arm_ctrl_obs_B.c_i_j] += f->data
            [(static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0_p) - 1) * 6 +
            left_arm_ctrl_obs_B.b_k_j] * a_0->data
            [(left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.c_i_j) + 1];
        }
      }

      left_arm_ctrl_obs_B.b_idx_0_p = robot->
        VelocityDoFMap[left_arm_ctrl_obs_B.inner_m - 1];
      left_arm_ctrl_obs_B.b_idx_1_p = robot->
        VelocityDoFMap[left_arm_ctrl_obs_B.inner_m + 9];
      if (left_arm_ctrl_obs_B.b_idx_0_p > left_arm_ctrl_obs_B.b_idx_1_p) {
        left_arm_ctrl_obs_B.m_e = 0;
        left_arm_ctrl_obs_B.i_f = 0;
      } else {
        left_arm_ctrl_obs_B.m_e = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_0_p) - 1;
        left_arm_ctrl_obs_B.i_f = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_1_p);
      }

      left_arm_ctrl_obs_B.unnamed_idx_1_a = left_arm_ctrl_obs_B.i_f -
        left_arm_ctrl_obs_B.m_e;
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f <
           left_arm_ctrl_obs_B.unnamed_idx_1_a; left_arm_ctrl_obs_B.i_f++) {
        tau[left_arm_ctrl_obs_B.m_e + left_arm_ctrl_obs_B.i_f] = qddoti->
          data[left_arm_ctrl_obs_B.i_f];
      }
    }

    left_arm_ctrl_obs_B.a_idx_0_p = robot->Bodies[left_arm_ctrl_obs_B.o_tmp]
      ->ParentIndex;
    if (left_arm_ctrl_obs_B.a_idx_0_p > 0.0) {
      left_arm_ctrl_obs_B.m_e = static_cast<int32_T>
        (left_arm_ctrl_obs_B.a_idx_0_p);
      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        left_arm_ctrl_obs_B.a_idx_1_k = 0.0;
        for (left_arm_ctrl_obs_B.aoffset_m = 0; left_arm_ctrl_obs_B.aoffset_m <
             6; left_arm_ctrl_obs_B.aoffset_m++) {
          left_arm_ctrl_obs_B.a_idx_1_k += f->data[(left_arm_ctrl_obs_B.inner_m
            - 1) * 6 + left_arm_ctrl_obs_B.aoffset_m] * X->
            data[left_arm_ctrl_obs_B.o_tmp].f1[6 * left_arm_ctrl_obs_B.i_f +
            left_arm_ctrl_obs_B.aoffset_m];
        }

        left_arm_ctrl_obs_B.a0_m[left_arm_ctrl_obs_B.i_f] = f->data
          [(left_arm_ctrl_obs_B.m_e - 1) * 6 + left_arm_ctrl_obs_B.i_f] +
          left_arm_ctrl_obs_B.a_idx_1_k;
      }

      for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f < 6;
           left_arm_ctrl_obs_B.i_f++) {
        f->data[left_arm_ctrl_obs_B.i_f + 6 * (left_arm_ctrl_obs_B.m_e - 1)] =
          left_arm_ctrl_obs_B.a0_m[left_arm_ctrl_obs_B.i_f];
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

static void matlabCodegenHandle_matlabC_e0h(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
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

static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal_e0h_T
  *pStruct)
{
  emxFreeStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_e0h_T
  *pStruct)
{
  emxFreeStruct_k_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct)
{
  le_emxFreeStruct_rigidBodyJoint(&pStruct->JointInternal);
}

static void l_emxFreeStruct_rigidBodyJoint1(rigidBodyJoint_left_arm_ctr_e_T
  *pStruct)
{
  left_arm_ctrl_ob_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_i_robotics_mani_e(i_robotics_manip_internal_R_e_T
  *pStruct)
{
  left_arm_ctrl_ob_emxFree_char_T(&pStruct->NameInternal);
  l_emxFreeStruct_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_k_robotics_mani_e(k_robotics_manip_internal_R_e_T
  *pStruct)
{
  emxFreeStruct_i_robotics_mani_e(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct)
{
  emxFreeStruct_k_robotics_mani_e(&pStruct->TreeInternal);
}

static void emxFreeStruct_j_robotics_mani_e(j_robotics_manip_internal_R_e_T
  *pStruct)
{
  left_arm_ctrl_ob_emxFree_char_T(&pStruct->NameInternal);
  l_emxFreeStruct_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_k_robotics_man_e0(k_robotics_manip_internal__e0_T
  *pStruct)
{
  emxFreeStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slman_e0(robotics_slmanip_internal__e0_T
  *pStruct)
{
  emxFreeStruct_k_robotics_man_e0(&pStruct->TreeInternal);
}

static void emxFreeStruct_k_robotics_ma_e0h(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slma_e0h(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_k_robotics_ma_e0h(&pStruct->TreeInternal);
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void le_emxInitStruct_rigidBodyJoint(rigidBodyJoint_left_arm_ctrl__T
  *pStruct)
{
  left_arm_ctrl_ob_emxInit_char_T(&pStruct->Type, 2);
  left_arm_ctrl_ob_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct)
{
  le_emxInitStruct_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal_e0h_T
  *pStruct)
{
  emxInitStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_e0h_T
  *pStruct)
{
  emxInitStruct_k_robotics_manip_(&pStruct->TreeInternal);
}

static void emxInitStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct)
{
  le_emxInitStruct_rigidBodyJoint(&pStruct->JointInternal);
}

static j_robotics_manip_internal_Rig_T *left_arm_ct_RigidBody_RigidBody
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
  static const real_T tmp_0[36] = { 0.3000675, 0.0, -0.00018135, 0.0, -0.0045,
    0.0, 0.0, 0.30055472699999997, 0.0, 0.0045, 0.0, -0.01209, -0.00018135, 0.0,
    0.30048722699999997, -0.0, 0.01209, 0.0, 0.0, 0.0045, -0.0, 0.3, 0.0, 0.0,
    -0.0045, 0.0, 0.01209, 0.0, 0.3, 0.0, 0.0, -0.01209, 0.0, 0.0, 0.0, 0.3 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, 0.99999999999325373, 0.0, 0.0, -0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.055, 0.09, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left_arm__RigidBody_RigidBody_e
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
  static const real_T tmp_0[36] = { 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0,
    0.3033075, 0.0, 0.0, 0.0, -0.0315, 0.0, 0.0, 0.3033075, -0.0, 0.0315, 0.0,
    0.0, 0.0, -0.0, 0.3, 0.0, 0.0, -0.0, 0.0, 0.0315, 0.0, 0.3, 0.0, 0.0,
    -0.0315, 0.0, 0.0, 0.0, 0.3 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, 0.99999999999325373, 0.0, 0.0, -0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0603, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left_arm_RigidBody_RigidBody_e0
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
  static const real_T tmp_0[36] = { 0.31323, 0.0, 0.0, 0.0, -0.063, 0.0, 0.0,
    0.31323, 0.0, 0.063, 0.0, -0.0, 0.0, 0.0, 0.3, -0.0, 0.0, 0.0, 0.0, 0.063,
    -0.0, 0.3, 0.0, 0.0, -0.063, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, -0.0, 0.0, 0.0,
    0.0, 0.3 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -3.6732051033465739E-6, 0.99999999999325373,
    -0.0, 0.0, 3.6732051033217936E-6, 1.3492435731251315E-11,
    0.99999999999325373, 0.0, 0.99999999998650746, 3.6732051033217936E-6,
    -3.6732051033465739E-6, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left_ar_RigidBody_RigidBody_e0h
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
  static const real_T tmp_0[36] = { 0.30243, 0.0, 0.0, 0.0, -0.0, -0.027, 0.0,
    0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.30243, 0.027, 0.0, 0.0, 0.0, 0.0,
    0.027, 0.3, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.3, 0.0, -0.027, -0.0, 0.0, 0.0,
    0.0, 0.3 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { -3.6732051033465739E-6, -0.99999999999325373,
    -0.0, 0.0, -3.6732051033217936E-6, 1.3492435731251315E-11,
    -0.99999999999325373, 0.0, 0.99999999998650746, -3.6732051033217936E-6,
    -3.6732051033465739E-6, 0.0, 0.0, 0.0, 0.27, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left_a_RigidBody_RigidBody_e0h4
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
  static const real_T tmp_0[36] = { 0.30972, 0.0, 0.0, 0.0, -0.054, 0.0, 0.0,
    0.30972, 0.0, 0.054, 0.0, -0.0, 0.0, 0.0, 0.3, -0.0, 0.0, 0.0, 0.0, 0.054,
    -0.0, 0.3, 0.0, 0.0, -0.054, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, -0.0, 0.0, 0.0,
    0.0, 0.3 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, 0.99999999999325373, 0.0, 0.0, -0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left__RigidBody_RigidBody_e0h4e
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
  static const real_T tmp_0[36] = { 0.30074999999999996, 0.0, 0.0, 0.0, -0.0,
    -0.015, 0.0, 0.3, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.30074999999999996, 0.015,
    0.0, 0.0, 0.0, 0.0, 0.015, 0.3, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.3, 0.0,
    -0.015, -0.0, 0.0, 0.0, 0.0, 0.3 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, -0.99999999999325373, 0.0, -0.0, 0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0, 0.0, 0.2126, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *left_RigidBody_RigidBody_e0h4ew
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
  static const real_T tmp_0[36] = { 0.303, 0.0, 0.0, 0.0, -0.03, 0.0, 0.0, 0.303,
    0.0, 0.03, 0.0, -0.0, 0.0, 0.0, 0.3, -0.0, 0.0, 0.0, 0.0, 0.03, -0.0, 0.3,
    0.0, 0.0, -0.03, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.3 };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, 0.99999999999325373, 0.0, 0.0, -0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_5[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_Rig_T *lef_RigidBody_RigidBody_e0h4ewm
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
    0.0, 1.0, 0.0, 0.0, 0.0, 0.13, 1.0 };

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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
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
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { -3.6732051033465739E-6, -0.0,
    -0.99999999999325373, 0.0, 0.0, 1.0, -0.0, 0.0, 0.99999999999325373, 0.0,
    -3.6732051033465739E-6, 0.0, 0.0, -0.04, 0.0, 1.0 };

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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
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
  static const real_T tmp_0[36] = { 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_1[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_2[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_3[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_4[16] = { -3.6732051033465739E-6, -0.0,
    0.99999999999325373, 0.0, -0.0, 1.0, -0.0, 0.0, -0.99999999999325373, -0.0,
    -3.6732051033465739E-6, 0.0, 0.0, 0.04, 0.0, 1.0 };

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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static i_robotics_manip_internal_Rig_T *RigidBody_RigidBody_e0h4ewmdid
  (i_robotics_manip_internal_Rig_T *obj)
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
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
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
  obj->Bodies[0] = left_arm_ct_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = left_arm__RigidBody_RigidBody_e(iobj_1);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = left_arm_RigidBody_RigidBody_e0(iobj_2);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = left_ar_RigidBody_RigidBody_e0h(iobj_3);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = left_a_RigidBody_RigidBody_e0h4(iobj_4);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = left__RigidBody_RigidBody_e0h4e(iobj_5);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = left_RigidBody_RigidBody_e0h4ew(iobj_6);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = lef_RigidBody_RigidBody_e0h4ewm(iobj_7);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = le_RigidBody_RigidBody_e0h4ewmd(iobj_8);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = l_RigidBody_RigidBody_e0h4ewmdi(iobj_9);
  obj->Bodies[9]->Index = 10.0;
  obj->NumBodies = 10.0;
  obj->VelocityNumber = 7.0;
  for (i = 0; i < 20; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  for (i = 0; i < 20; i++) {
    obj->VelocityDoFMap[i] = tmp[i];
  }

  RigidBody_RigidBody_e0h4ewmdid(&obj->Base);
  return b_obj;
}

static void l_emxInitStruct_rigidBodyJoint1(rigidBodyJoint_left_arm_ctr_e_T
  *pStruct)
{
  left_arm_ctrl_ob_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInitStruct_i_robotics_mani_e(i_robotics_manip_internal_R_e_T
  *pStruct)
{
  left_arm_ctrl_ob_emxInit_char_T(&pStruct->NameInternal, 2);
  l_emxInitStruct_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_k_robotics_mani_e(k_robotics_manip_internal_R_e_T
  *pStruct)
{
  emxInitStruct_i_robotics_mani_e(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct)
{
  emxInitStruct_k_robotics_mani_e(&pStruct->TreeInternal);
}

static void emxInitStruct_j_robotics_mani_e(j_robotics_manip_internal_R_e_T
  *pStruct)
{
  left_arm_ctrl_ob_emxInit_char_T(&pStruct->NameInternal, 2);
  l_emxInitStruct_rigidBodyJoint1(&pStruct->JointInternal);
}

static j_robotics_manip_internal_R_e_T *RigidBody_RigidBody_e0h4ewmdidb
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

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, 0.99999999999325373, 0.0, 0.0, -0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.055, 0.09, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_a_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *RigidBody_RigidBod_e0h4ewmdidbj
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

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, 0.99999999999325373, 0.0, 0.0, -0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0603, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_a_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *RigidBody_RigidBo_e0h4ewmdidbjt
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

  static const real_T tmp_2[16] = { -3.6732051033465739E-6, 0.99999999999325373,
    -0.0, 0.0, 3.6732051033217936E-6, 1.3492435731251315E-11,
    0.99999999999325373, 0.0, 0.99999999998650746, 3.6732051033217936E-6,
    -3.6732051033465739E-6, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_a_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *RigidBody_RigidB_e0h4ewmdidbjtq
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

  static const real_T tmp_2[16] = { -3.6732051033465739E-6, -0.99999999999325373,
    -0.0, 0.0, -3.6732051033217936E-6, 1.3492435731251315E-11,
    -0.99999999999325373, 0.0, 0.99999999998650746, -3.6732051033217936E-6,
    -3.6732051033465739E-6, 0.0, 0.0, 0.0, 0.27, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_a_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *RigidBody_Rigid_e0h4ewmdidbjtqt
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

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, 0.99999999999325373, 0.0, 0.0, -0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_a_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *l_RigidBody_Rigid_e
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

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, -0.99999999999325373, 0.0, -0.0, 0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0, 0.0, 0.2126, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_a_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 5.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  return b_obj;
}

static j_robotics_manip_internal_R_e_T *l_RigidBody_Rigid_i
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

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0,
    -3.6732051033465739E-6, 0.99999999999325373, 0.0, 0.0, -0.99999999999325373,
    -3.6732051033465739E-6, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 14;
  left_a_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 6.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  left_a_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
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
  obj->JointInternal.JointAxisInternal[2] = 1.0;
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
    0.0, 1.0, 0.0, 0.0, 0.0, 0.13, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_5[18] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'g', 'r', 'i', 'p', '_', 'l', 'e', 'f', 't' };

  static const real_T tmp_6[16] = { -3.6732051033465739E-6, -0.0,
    -0.99999999999325373, 0.0, 0.0, 1.0, -0.0, 0.0, 0.99999999999325373, 0.0,
    -3.6732051033465739E-6, 0.0, 0.0, -0.04, 0.0, 1.0 };

  static const char_T tmp_7[19] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'g', 'r', 'i', 'p', '_', 'r', 'i', 'g', 'h', 't' };

  static const real_T tmp_8[16] = { -3.6732051033465739E-6, -0.0,
    0.99999999999325373, 0.0, -0.0, 1.0, -0.0, 0.0, -0.99999999999325373, -0.0,
    -3.6732051033465739E-6, 0.0, 0.0, 0.04, 0.0, 1.0 };

  static const char_T tmp_9[19] = { 's', 'h', 'o', 'u', 'l', 'd', 'e', 'r', 's',
    '_', 'l', 'e', 'f', 't', '_', 'l', 'i', 'n', 'k' };

  int32_T exitg1;
  b_obj = obj;
  obj->Bodies[0] = RigidBody_RigidBody_e0h4ewmdidb(iobj_0);
  obj->Bodies[1] = RigidBody_RigidBod_e0h4ewmdidbj(iobj_1);
  obj->Bodies[2] = RigidBody_RigidBo_e0h4ewmdidbjt(iobj_2);
  obj->Bodies[3] = RigidBody_RigidB_e0h4ewmdidbjtq(iobj_3);
  obj->Bodies[4] = RigidBody_Rigid_e0h4ewmdidbjtqt(iobj_4);
  obj->Bodies[5] = l_RigidBody_Rigid_e(iobj_5);
  obj->Bodies[6] = l_RigidBody_Rigid_i(iobj_6);
  b_kstr = iobj_7->NameInternal->size[0] * iobj_7->NameInternal->size[1];
  iobj_7->NameInternal->size[0] = 1;
  iobj_7->NameInternal->size[1] = 20;
  left_a_emxEnsureCapacity_char_T(iobj_7->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 20; b_kstr++) {
    iobj_7->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  iobj_7->ParentIndex = 7.0;
  b_kstr = iobj_7->JointInternal.Type->size[0] * iobj_7->
    JointInternal.Type->size[1];
  iobj_7->JointInternal.Type->size[0] = 1;
  iobj_7->JointInternal.Type->size[1] = 5;
  left_a_emxEnsureCapacity_char_T(iobj_7->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_7->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  left_arm_ctrl_ob_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_7->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_char_T(iobj_8->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 18; b_kstr++) {
    iobj_8->NameInternal->data[b_kstr] = tmp_5[b_kstr];
  }

  iobj_8->ParentIndex = 8.0;
  b_kstr = iobj_8->JointInternal.Type->size[0] * iobj_8->
    JointInternal.Type->size[1];
  iobj_8->JointInternal.Type->size[0] = 1;
  iobj_8->JointInternal.Type->size[1] = 5;
  left_a_emxEnsureCapacity_char_T(iobj_8->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_8->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_8->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_char_T(iobj_9->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 19; b_kstr++) {
    iobj_9->NameInternal->data[b_kstr] = tmp_7[b_kstr];
  }

  iobj_9->ParentIndex = 8.0;
  b_kstr = iobj_9->JointInternal.Type->size[0] * iobj_9->
    JointInternal.Type->size[1];
  iobj_9->JointInternal.Type->size[0] = 1;
  iobj_9->JointInternal.Type->size[1] = 5;
  left_a_emxEnsureCapacity_char_T(iobj_9->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_9->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_9->JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
  left_a_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 19; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  left_a_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  left_a_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
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

static void emxInitStruct_k_robotics_man_e0(k_robotics_manip_internal__e0_T
  *pStruct)
{
  emxInitStruct_i_robotics_manip_(&pStruct->Base);
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
  int32_T i;
  static const int8_T tmp[20] = { 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 1, 2, 3, 4, 5, 6,
    7, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = left_arm_ct_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = left_arm__RigidBody_RigidBody_e(iobj_1);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = left_arm_RigidBody_RigidBody_e0(iobj_2);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = left_ar_RigidBody_RigidBody_e0h(iobj_3);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = left_a_RigidBody_RigidBody_e0h4(iobj_4);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = left__RigidBody_RigidBody_e0h4e(iobj_5);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = left_RigidBody_RigidBody_e0h4ew(iobj_6);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = lef_RigidBody_RigidBody_e0h4ewm(iobj_7);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = le_RigidBody_RigidBody_e0h4ewmd(iobj_8);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = l_RigidBody_RigidBody_e0h4ewmdi(iobj_9);
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

  RigidBody_RigidBody_e0h4ewmdid(&obj->Base);
  return b_obj;
}

static void emxInitStruct_k_robotics_ma_e0h(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slma_e0h(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_k_robotics_ma_e0h(&pStruct->TreeInternal);
}

static k_robotics_manip_internal_Rig_T *lef_RigidBodyTree_RigidBodyTree
  (k_robotics_manip_internal_Rig_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9)
{
  k_robotics_manip_internal_Rig_T *b_obj;
  int32_T i;
  static const int8_T tmp[20] = { 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 1, 2, 3, 4, 5, 6,
    7, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = left_arm_ct_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = left_arm__RigidBody_RigidBody_e(iobj_1);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = left_arm_RigidBody_RigidBody_e0(iobj_2);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = left_ar_RigidBody_RigidBody_e0h(iobj_3);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = left_a_RigidBody_RigidBody_e0h4(iobj_4);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = left__RigidBody_RigidBody_e0h4e(iobj_5);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = left_RigidBody_RigidBody_e0h4ew(iobj_6);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = lef_RigidBody_RigidBody_e0h4ewm(iobj_7);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = le_RigidBody_RigidBody_e0h4ewmd(iobj_8);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = l_RigidBody_RigidBody_e0h4ewmdi(iobj_9);
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

  RigidBody_RigidBody_e0h4ewmdid(&obj->Base);
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
  robotics_slmanip_internal_blo_T *obj_2;
  emxArray_real_T_left_arm_ctrl_T *L;
  emxArray_real_T_left_arm_ctrl_T *lambda;
  emxArray_real_T_left_arm_ctrl_T *H;
  emxArray_real_T_left_arm_ctrl_T *tmp;
  static const real_T kp[7] = { 20.5, 1.0, 1.0, 14.5, 1.0, 10.5, 1.0 };

  static const int8_T kd[7] = { 7, 1, 1, 5, 1, 5, 1 };

  static const char_T tmp_0[19] = { 's', 'h', 'o', 'u', 'l', 'd', 'e', 'r', 's',
    '_', 'l', 'e', 'f', 't', '_', 'l', 'i', 'n', 'k' };

  static const char_T tmp_1[14] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
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

  left_arm_ctrl_ob_emxInit_real_T(&b, 2);
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S13>/SourceBlock' incorporates:
    //   Inport: '<S16>/In1'

    left_arm_ctrl_o_SystemCore_step(&left_arm_ctrl_obs_B.b_varargout_1,
      left_arm_ctrl_obs_B.qe,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Curr,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Rece,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_DataOffset,
      left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_Inf,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_j);

    // Outputs for Enabled SubSystem: '<S13>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S16>/Enable'

    if (left_arm_ctrl_obs_B.b_varargout_1) {
      for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 7;
           left_arm_ctrl_obs_B.n_m++) {
        left_arm_ctrl_obs_B.In1.Data[left_arm_ctrl_obs_B.n_m] =
          left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.n_m];
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
        left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_j;
    }

    // End of MATLABSystem: '<S13>/SourceBlock'
    // End of Outputs for SubSystem: '<S13>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'

    // MATLABSystem: '<S7>/MATLAB System'
    RigidBodyTreeDynamics_massMat_e(&left_arm_ctrl_obs_DW.obj_jz.TreeInternal,
      left_arm_ctrl_obs_B.In1.Data, b);
    for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 49;
         left_arm_ctrl_obs_B.n_m++) {
      left_arm_ctrl_obs_B.MATLABSystem[left_arm_ctrl_obs_B.n_m] = b->
        data[left_arm_ctrl_obs_B.n_m];
    }

    // End of MATLABSystem: '<S7>/MATLAB System'
    left_arm_ct_emxInit_e_cell_wrap(&Ttree, 2);
    left_arm_ctrl_ob_emxInit_char_T(&bname, 2);

    // MATLABSystem: '<S5>/MATLAB System'
    obj = &left_arm_ctrl_obs_DW.obj_f;
    obj_0 = &left_arm_ctrl_obs_DW.obj_f.TreeInternal;
    RigidBodyTree_forwardKinematics(&obj->TreeInternal,
      left_arm_ctrl_obs_B.In1.Data, Ttree);
    left_arm_ctrl_obs_B.bid1 = -1.0;
    left_arm_ctrl_obs_B.n_m = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj_0->Base.NameInternal->size[1];
    left_a_emxEnsureCapacity_char_T(bname, left_arm_ctrl_obs_B.n_m);
    left_arm_ctrl_obs_B.loop_ub = obj_0->Base.NameInternal->size[0] *
      obj_0->Base.NameInternal->size[1] - 1;
    for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m <=
         left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.n_m++) {
      bname->data[left_arm_ctrl_obs_B.n_m] = obj_0->Base.NameInternal->
        data[left_arm_ctrl_obs_B.n_m];
    }

    for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 19;
         left_arm_ctrl_obs_B.n_m++) {
      left_arm_ctrl_obs_B.b_i[left_arm_ctrl_obs_B.n_m] =
        tmp_0[left_arm_ctrl_obs_B.n_m];
    }

    left_arm_ctrl_obs_B.b_varargout_1 = false;
    if (bname->size[1] == 19) {
      left_arm_ctrl_obs_B.loop_ub = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.loop_ub - 1 < 19) {
          left_arm_ctrl_obs_B.iend = left_arm_ctrl_obs_B.loop_ub - 1;
          if (bname->data[left_arm_ctrl_obs_B.iend] !=
              left_arm_ctrl_obs_B.b_i[left_arm_ctrl_obs_B.iend]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.loop_ub++;
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
      left_arm_ctrl_obs_B.vNum = obj->TreeInternal.NumBodies;
      left_arm_ctrl_obs_B.n_m = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum) -
        1;
      if (0 <= left_arm_ctrl_obs_B.n_m) {
        for (left_arm_ctrl_obs_B.j_a = 0; left_arm_ctrl_obs_B.j_a < 19;
             left_arm_ctrl_obs_B.j_a++) {
          left_arm_ctrl_obs_B.b_i[left_arm_ctrl_obs_B.j_a] =
            tmp_0[left_arm_ctrl_obs_B.j_a];
        }
      }

      left_arm_ctrl_obs_B.i = 0;
      exitg2 = false;
      while ((!exitg2) && (left_arm_ctrl_obs_B.i <= left_arm_ctrl_obs_B.n_m)) {
        obj_1 = obj_0->Bodies[left_arm_ctrl_obs_B.i];
        left_arm_ctrl_obs_B.j_a = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = obj_1->NameInternal->size[1];
        left_a_emxEnsureCapacity_char_T(bname, left_arm_ctrl_obs_B.j_a);
        left_arm_ctrl_obs_B.loop_ub = obj_1->NameInternal->size[0] *
          obj_1->NameInternal->size[1] - 1;
        for (left_arm_ctrl_obs_B.j_a = 0; left_arm_ctrl_obs_B.j_a <=
             left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.j_a++) {
          bname->data[left_arm_ctrl_obs_B.j_a] = obj_1->NameInternal->
            data[left_arm_ctrl_obs_B.j_a];
        }

        left_arm_ctrl_obs_B.b_varargout_1 = false;
        if (bname->size[1] == 19) {
          left_arm_ctrl_obs_B.loop_ub = 1;
          do {
            exitg1 = 0;
            if (left_arm_ctrl_obs_B.loop_ub - 1 < 19) {
              left_arm_ctrl_obs_B.iend = left_arm_ctrl_obs_B.loop_ub - 1;
              if (bname->data[left_arm_ctrl_obs_B.iend] !=
                  left_arm_ctrl_obs_B.b_i[left_arm_ctrl_obs_B.iend]) {
                exitg1 = 1;
              } else {
                left_arm_ctrl_obs_B.loop_ub++;
              }
            } else {
              left_arm_ctrl_obs_B.b_varargout_1 = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (left_arm_ctrl_obs_B.b_varargout_1) {
          left_arm_ctrl_obs_B.bid1 = static_cast<real_T>(left_arm_ctrl_obs_B.i)
            + 1.0;
          exitg2 = true;
        } else {
          left_arm_ctrl_obs_B.i++;
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
      for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 16;
           left_arm_ctrl_obs_B.n_m++) {
        left_arm_ctrl_obs_B.T1[left_arm_ctrl_obs_B.n_m] = Ttree->data[
          static_cast<int32_T>(left_arm_ctrl_obs_B.bid1) - 1]
          .f1[left_arm_ctrl_obs_B.n_m];
      }
    }

    left_arm_ctrl_obs_B.bid1 = -1.0;
    left_arm_ctrl_obs_B.n_m = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj_0->Base.NameInternal->size[1];
    left_a_emxEnsureCapacity_char_T(bname, left_arm_ctrl_obs_B.n_m);
    left_arm_ctrl_obs_B.loop_ub = obj_0->Base.NameInternal->size[0] *
      obj_0->Base.NameInternal->size[1] - 1;
    for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m <=
         left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.n_m++) {
      bname->data[left_arm_ctrl_obs_B.n_m] = obj_0->Base.NameInternal->
        data[left_arm_ctrl_obs_B.n_m];
    }

    for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 14;
         left_arm_ctrl_obs_B.n_m++) {
      left_arm_ctrl_obs_B.b_f[left_arm_ctrl_obs_B.n_m] =
        tmp_1[left_arm_ctrl_obs_B.n_m];
    }

    left_arm_ctrl_obs_B.b_varargout_1 = false;
    if (bname->size[1] == 14) {
      left_arm_ctrl_obs_B.loop_ub = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.loop_ub - 1 < 14) {
          left_arm_ctrl_obs_B.iend = left_arm_ctrl_obs_B.loop_ub - 1;
          if (bname->data[left_arm_ctrl_obs_B.iend] !=
              left_arm_ctrl_obs_B.b_f[left_arm_ctrl_obs_B.iend]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.loop_ub++;
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
      left_arm_ctrl_obs_B.vNum = obj->TreeInternal.NumBodies;
      left_arm_ctrl_obs_B.n_m = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum) -
        1;
      if (0 <= left_arm_ctrl_obs_B.n_m) {
        for (left_arm_ctrl_obs_B.j_a = 0; left_arm_ctrl_obs_B.j_a < 14;
             left_arm_ctrl_obs_B.j_a++) {
          left_arm_ctrl_obs_B.b_f[left_arm_ctrl_obs_B.j_a] =
            tmp_1[left_arm_ctrl_obs_B.j_a];
        }
      }

      left_arm_ctrl_obs_B.i = 0;
      exitg2 = false;
      while ((!exitg2) && (left_arm_ctrl_obs_B.i <= left_arm_ctrl_obs_B.n_m)) {
        obj_1 = obj_0->Bodies[left_arm_ctrl_obs_B.i];
        left_arm_ctrl_obs_B.j_a = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = obj_1->NameInternal->size[1];
        left_a_emxEnsureCapacity_char_T(bname, left_arm_ctrl_obs_B.j_a);
        left_arm_ctrl_obs_B.loop_ub = obj_1->NameInternal->size[0] *
          obj_1->NameInternal->size[1] - 1;
        for (left_arm_ctrl_obs_B.j_a = 0; left_arm_ctrl_obs_B.j_a <=
             left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.j_a++) {
          bname->data[left_arm_ctrl_obs_B.j_a] = obj_1->NameInternal->
            data[left_arm_ctrl_obs_B.j_a];
        }

        left_arm_ctrl_obs_B.b_varargout_1 = false;
        if (bname->size[1] == 14) {
          left_arm_ctrl_obs_B.loop_ub = 1;
          do {
            exitg1 = 0;
            if (left_arm_ctrl_obs_B.loop_ub - 1 < 14) {
              left_arm_ctrl_obs_B.iend = left_arm_ctrl_obs_B.loop_ub - 1;
              if (bname->data[left_arm_ctrl_obs_B.iend] !=
                  left_arm_ctrl_obs_B.b_f[left_arm_ctrl_obs_B.iend]) {
                exitg1 = 1;
              } else {
                left_arm_ctrl_obs_B.loop_ub++;
              }
            } else {
              left_arm_ctrl_obs_B.b_varargout_1 = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (left_arm_ctrl_obs_B.b_varargout_1) {
          left_arm_ctrl_obs_B.bid1 = static_cast<real_T>(left_arm_ctrl_obs_B.i)
            + 1.0;
          exitg2 = true;
        } else {
          left_arm_ctrl_obs_B.i++;
        }
      }
    }

    left_arm_ctrl_ob_emxFree_char_T(&bname);

    // MATLABSystem: '<S5>/MATLAB System'
    if (left_arm_ctrl_obs_B.bid1 == 0.0) {
      memset(&left_arm_ctrl_obs_B.T2[0], 0, sizeof(real_T) << 4U);
      left_arm_ctrl_obs_B.T2[0] = 1.0;
      left_arm_ctrl_obs_B.T2[5] = 1.0;
      left_arm_ctrl_obs_B.T2[10] = 1.0;
      left_arm_ctrl_obs_B.T2[15] = 1.0;
    } else {
      for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 16;
           left_arm_ctrl_obs_B.n_m++) {
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.n_m] = Ttree->data[
          static_cast<int32_T>(left_arm_ctrl_obs_B.bid1) - 1]
          .f1[left_arm_ctrl_obs_B.n_m];
      }
    }

    left_arm_ct_emxFree_e_cell_wrap(&Ttree);

    // MATLABSystem: '<S5>/MATLAB System'
    for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 3;
         left_arm_ctrl_obs_B.n_m++) {
      left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.n_m] =
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.n_m];
      left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.n_m + 1] =
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.n_m + 4];
      left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.n_m + 2] =
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.n_m + 8];
    }

    for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 9;
         left_arm_ctrl_obs_B.n_m++) {
      left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.n_m] =
        -left_arm_ctrl_obs_B.R_l[left_arm_ctrl_obs_B.n_m];
    }

    for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 3;
         left_arm_ctrl_obs_B.n_m++) {
      left_arm_ctrl_obs_B.loop_ub = left_arm_ctrl_obs_B.n_m << 2;
      left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.loop_ub] =
        left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.n_m];
      left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.loop_ub + 1] =
        left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.n_m + 1];
      left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.loop_ub + 2] =
        left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.n_m + 2];
      left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.n_m + 12] =
        left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.n_m + 6] *
        left_arm_ctrl_obs_B.T2[14] +
        (left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.n_m + 3] *
         left_arm_ctrl_obs_B.T2[13] +
         left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.n_m] *
         left_arm_ctrl_obs_B.T2[12]);
    }

    left_arm_ctrl_obs_B.R_c[3] = 0.0;
    left_arm_ctrl_obs_B.R_c[7] = 0.0;
    left_arm_ctrl_obs_B.R_c[11] = 0.0;
    left_arm_ctrl_obs_B.R_c[15] = 1.0;
    for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 4;
         left_arm_ctrl_obs_B.n_m++) {
      for (left_arm_ctrl_obs_B.j_a = 0; left_arm_ctrl_obs_B.j_a < 4;
           left_arm_ctrl_obs_B.j_a++) {
        left_arm_ctrl_obs_B.loop_ub = left_arm_ctrl_obs_B.n_m << 2;
        left_arm_ctrl_obs_B.i = left_arm_ctrl_obs_B.j_a +
          left_arm_ctrl_obs_B.loop_ub;
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.i] = 0.0;
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.i] +=
          left_arm_ctrl_obs_B.T1[left_arm_ctrl_obs_B.loop_ub] *
          left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.j_a];
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.i] +=
          left_arm_ctrl_obs_B.T1[left_arm_ctrl_obs_B.loop_ub + 1] *
          left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.j_a + 4];
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.i] +=
          left_arm_ctrl_obs_B.T1[left_arm_ctrl_obs_B.loop_ub + 2] *
          left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.j_a + 8];
        left_arm_ctrl_obs_B.T2[left_arm_ctrl_obs_B.i] +=
          left_arm_ctrl_obs_B.T1[left_arm_ctrl_obs_B.loop_ub + 3] *
          left_arm_ctrl_obs_B.R_c[left_arm_ctrl_obs_B.j_a + 12];
      }
    }

    // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
    // MATLABSystem: '<S14>/SourceBlock' incorporates:
    //   Inport: '<S17>/In1'

    left_arm_ctrl_SystemCore_step_e(&left_arm_ctrl_obs_B.b_varargout_1,
      left_arm_ctrl_obs_B.b_varargout_2_Data,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Curr,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Rece,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_DataOffset,
      left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_Inf,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_j);

    // Outputs for Enabled SubSystem: '<S14>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S17>/Enable'

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
        left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_j;
    }

    // End of MATLABSystem: '<S14>/SourceBlock'
    // End of Outputs for SubSystem: '<S14>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe1'

    // MATLABSystem: '<S6>/MATLAB System'
    lef_GravityTorqueBlock_stepImpl(&left_arm_ctrl_obs_DW.obj_j,
      &left_arm_ctrl_obs_B.In1_e.Data[0], left_arm_ctrl_obs_B.MATLABSystem_p);
  }

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   Integrator: '<Root>/Integrator'

  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
       left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.i] = (static_cast<real32_T>
      ((left_arm_ctrl_obs_B.In1_e.Data[left_arm_ctrl_obs_B.i] -
        static_cast<real32_T>(left_arm_ctrl_obs_B.In1.Data[left_arm_ctrl_obs_B.i]))
       * kp[left_arm_ctrl_obs_B.i]) +
      left_arm_ctrl_obs_B.MATLABSystem_p[left_arm_ctrl_obs_B.i]) +
      static_cast<real32_T>
      ((left_arm_ctrl_obs_B.In1_e.Data[left_arm_ctrl_obs_B.i + 7] -
        static_cast<real32_T>
        (left_arm_ctrl_obs_X.Integrator_CSTATE[left_arm_ctrl_obs_B.i + 7])) *
       static_cast<real_T>(kd[left_arm_ctrl_obs_B.i]));
  }

  // End of MATLAB Function: '<Root>/MATLAB Function1'
  left_arm_ctrl_ob_emxInit_real_T(&lambda, 2);
  left_arm_ctrl_ob_emxInit_real_T(&tmp, 1);

  // MATLABSystem: '<S4>/MATLAB System' incorporates:
  //   Constant: '<Root>/Constant2'
  //   Integrator: '<Root>/Integrator'

  obj_2 = &left_arm_ctrl_obs_DW.obj;
  RigidBodyTreeDynamics_massMatri(&left_arm_ctrl_obs_DW.obj.TreeInternal,
    left_arm_ctrl_obs_B.In1.Data, b, lambda);
  left_arm_ctrl_obs_B.vNum = obj_2->TreeInternal.VelocityNumber;
  left_arm_ctrl_obs_B.n_m = tmp->size[0];
  left_arm_ctrl_obs_B.loop_ub = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum);
  tmp->size[0] = left_arm_ctrl_obs_B.loop_ub;
  left_a_emxEnsureCapacity_real_T(tmp, left_arm_ctrl_obs_B.n_m);
  for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m <
       left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.n_m++) {
    tmp->data[left_arm_ctrl_obs_B.n_m] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj_2->TreeInternal,
    left_arm_ctrl_obs_B.In1.Data, &left_arm_ctrl_obs_X.Integrator_CSTATE[7], tmp,
    left_arm_ctrl_obs_P.Constant2_Value, left_arm_ctrl_obs_B.qe);
  left_arm_ctrl_ob_emxFree_real_T(&tmp);

  // MATLABSystem: '<S4>/MATLAB System'
  for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 7;
       left_arm_ctrl_obs_B.n_m++) {
    left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.n_m] =
      left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.n_m] -
      left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.n_m];
  }

  if ((b->size[0] == 0) || (b->size[1] == 0)) {
    left_arm_ctrl_obs_B.u1 = 0;
  } else {
    left_arm_ctrl_obs_B.i = b->size[0];
    left_arm_ctrl_obs_B.u1 = b->size[1];
    if (left_arm_ctrl_obs_B.i > left_arm_ctrl_obs_B.u1) {
      left_arm_ctrl_obs_B.u1 = left_arm_ctrl_obs_B.i;
    }
  }

  left_arm_ctrl_ob_emxInit_real_T(&H, 2);

  // MATLABSystem: '<S4>/MATLAB System'
  left_arm_ctrl_obs_B.n_m = H->size[0] * H->size[1];
  H->size[0] = b->size[0];
  H->size[1] = b->size[1];
  left_a_emxEnsureCapacity_real_T(H, left_arm_ctrl_obs_B.n_m);
  left_arm_ctrl_obs_B.i = b->size[0] * b->size[1] - 1;
  for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m <=
       left_arm_ctrl_obs_B.i; left_arm_ctrl_obs_B.n_m++) {
    H->data[left_arm_ctrl_obs_B.n_m] = b->data[left_arm_ctrl_obs_B.n_m];
  }

  left_arm_ctrl_ob_emxFree_real_T(&b);

  // MATLABSystem: '<S4>/MATLAB System'
  left_arm_ctrl_obs_B.iend = static_cast<int32_T>(((-1.0 - static_cast<real_T>
    (left_arm_ctrl_obs_B.u1)) + 1.0) / -1.0) - 1;
  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i <=
       left_arm_ctrl_obs_B.iend; left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.j = static_cast<real_T>(left_arm_ctrl_obs_B.u1) + -
      static_cast<real_T>(left_arm_ctrl_obs_B.i);
    left_arm_ctrl_obs_B.n_m = static_cast<int32_T>(left_arm_ctrl_obs_B.j);
    left_arm_ctrl_obs_B.j_a = left_arm_ctrl_obs_B.n_m - 1;
    H->data[(static_cast<int32_T>(left_arm_ctrl_obs_B.j) + H->size[0] * (
              static_cast<int32_T>(left_arm_ctrl_obs_B.j) - 1)) - 1] = sqrt
      (H->data[(left_arm_ctrl_obs_B.j_a * H->size[0] + left_arm_ctrl_obs_B.n_m)
       - 1]);
    left_arm_ctrl_obs_B.bid1 = lambda->data[left_arm_ctrl_obs_B.j_a];
    while (left_arm_ctrl_obs_B.bid1 > 0.0) {
      left_arm_ctrl_obs_B.i_k = static_cast<int32_T>(left_arm_ctrl_obs_B.bid1) -
        1;
      H->data[(static_cast<int32_T>(left_arm_ctrl_obs_B.j) + H->size[0] * (
                static_cast<int32_T>(left_arm_ctrl_obs_B.bid1) - 1)) - 1] =
        H->data[(left_arm_ctrl_obs_B.i_k * H->size[0] + left_arm_ctrl_obs_B.n_m)
        - 1] / H->data[((static_cast<int32_T>(left_arm_ctrl_obs_B.j) - 1) *
                        H->size[0] + static_cast<int32_T>(left_arm_ctrl_obs_B.j))
        - 1];
      left_arm_ctrl_obs_B.bid1 = lambda->data[left_arm_ctrl_obs_B.i_k];
    }

    left_arm_ctrl_obs_B.bid1 = lambda->data[left_arm_ctrl_obs_B.j_a];
    while (left_arm_ctrl_obs_B.bid1 > 0.0) {
      left_arm_ctrl_obs_B.j = left_arm_ctrl_obs_B.bid1;
      while (left_arm_ctrl_obs_B.j > 0.0) {
        left_arm_ctrl_obs_B.j_a = static_cast<int32_T>(left_arm_ctrl_obs_B.j) -
          1;
        H->data[(static_cast<int32_T>(left_arm_ctrl_obs_B.bid1) + H->size[0] * (
                  static_cast<int32_T>(left_arm_ctrl_obs_B.j) - 1)) - 1] =
          H->data[(left_arm_ctrl_obs_B.j_a * H->size[0] + static_cast<int32_T>
                   (left_arm_ctrl_obs_B.bid1)) - 1] - H->data
          [((static_cast<int32_T>(left_arm_ctrl_obs_B.bid1) - 1) * H->size[0] +
            left_arm_ctrl_obs_B.n_m) - 1] * H->data[((static_cast<int32_T>
          (left_arm_ctrl_obs_B.j) - 1) * H->size[0] + left_arm_ctrl_obs_B.n_m) -
          1];
        left_arm_ctrl_obs_B.j = lambda->data[left_arm_ctrl_obs_B.j_a];
      }

      left_arm_ctrl_obs_B.bid1 = lambda->data[static_cast<int32_T>
        (left_arm_ctrl_obs_B.bid1) - 1];
    }
  }

  left_arm_ctrl_ob_emxInit_real_T(&L, 2);

  // MATLABSystem: '<S4>/MATLAB System'
  left_arm_ctrl_obs_B.n_m = L->size[0] * L->size[1];
  L->size[0] = H->size[0];
  L->size[1] = H->size[1];
  left_a_emxEnsureCapacity_real_T(L, left_arm_ctrl_obs_B.n_m);
  left_arm_ctrl_obs_B.i = H->size[0] * H->size[1] - 1;
  for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m <=
       left_arm_ctrl_obs_B.i; left_arm_ctrl_obs_B.n_m++) {
    L->data[left_arm_ctrl_obs_B.n_m] = H->data[left_arm_ctrl_obs_B.n_m];
  }

  left_arm_ctrl_obs_B.n_m = H->size[1];
  if ((H->size[0] == 0) || (H->size[1] == 0) || (1 >= H->size[1])) {
  } else {
    left_arm_ctrl_obs_B.iend = 0;
    for (left_arm_ctrl_obs_B.j_a = 2; left_arm_ctrl_obs_B.j_a <=
         left_arm_ctrl_obs_B.n_m; left_arm_ctrl_obs_B.j_a++) {
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i <=
           left_arm_ctrl_obs_B.iend; left_arm_ctrl_obs_B.i++) {
        L->data[left_arm_ctrl_obs_B.i + L->size[0] * (left_arm_ctrl_obs_B.j_a -
          1)] = 0.0;
      }

      if (left_arm_ctrl_obs_B.iend + 1 < H->size[0]) {
        left_arm_ctrl_obs_B.iend++;
      }
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&H);

  // MATLABSystem: '<S4>/MATLAB System'
  left_arm_ctrl_obs_B.iend = static_cast<int32_T>(((-1.0 -
    left_arm_ctrl_obs_B.vNum) + 1.0) / -1.0) - 1;
  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i <=
       left_arm_ctrl_obs_B.iend; left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.j_a = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum + -
      static_cast<real_T>(left_arm_ctrl_obs_B.i));
    left_arm_ctrl_obs_B.n_m = left_arm_ctrl_obs_B.j_a - 1;
    left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.n_m] /= L->data
      [(left_arm_ctrl_obs_B.n_m * L->size[0] + left_arm_ctrl_obs_B.j_a) - 1];
    left_arm_ctrl_obs_B.j = lambda->data[left_arm_ctrl_obs_B.n_m];
    while (left_arm_ctrl_obs_B.j > 0.0) {
      left_arm_ctrl_obs_B.u1 = static_cast<int32_T>(left_arm_ctrl_obs_B.j) - 1;
      left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.u1] -= L->data
        [(left_arm_ctrl_obs_B.u1 * L->size[0] + left_arm_ctrl_obs_B.j_a) - 1] *
        left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.n_m];
      left_arm_ctrl_obs_B.j = lambda->data[left_arm_ctrl_obs_B.u1];
    }
  }

  left_arm_ctrl_obs_B.loop_ub--;
  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i <=
       left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.j = lambda->data[left_arm_ctrl_obs_B.i];
    while (left_arm_ctrl_obs_B.j > 0.0) {
      left_arm_ctrl_obs_B.n_m = static_cast<int32_T>(left_arm_ctrl_obs_B.j) - 1;
      left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.i] -= L->
        data[left_arm_ctrl_obs_B.n_m * L->size[0] + left_arm_ctrl_obs_B.i] *
        left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.n_m];
      left_arm_ctrl_obs_B.j = lambda->data[left_arm_ctrl_obs_B.n_m];
    }

    left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.i] /= L->data[L->size[0] *
      left_arm_ctrl_obs_B.i + left_arm_ctrl_obs_B.i];
  }

  left_arm_ctrl_ob_emxFree_real_T(&lambda);
  left_arm_ctrl_ob_emxFree_real_T(&L);

  // MATLAB Function: '<Root>/Observer' incorporates:
  //   Integrator: '<Root>/Integrator'
  //   MATLABSystem: '<S4>/MATLAB System'

  memset(&left_arm_ctrl_obs_B.xp_est[0], 0, 14U * sizeof(real_T));
  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
       left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.vNum = 2.0 / (exp
      ((left_arm_ctrl_obs_X.Integrator_CSTATE[left_arm_ctrl_obs_B.i] -
        left_arm_ctrl_obs_B.In1.Data[left_arm_ctrl_obs_B.i]) / 0.0001) + 1.0) -
      1.0;
    left_arm_ctrl_obs_B.z[left_arm_ctrl_obs_B.i] = left_arm_ctrl_obs_B.vNum *
      6.0;
    left_arm_ctrl_obs_B.xp_est[left_arm_ctrl_obs_B.i] = left_arm_ctrl_obs_B.vNum
      * (sqrt(fabs(left_arm_ctrl_obs_B.In1.Data[left_arm_ctrl_obs_B.i] -
                   left_arm_ctrl_obs_X.Integrator_CSTATE[left_arm_ctrl_obs_B.i]))
         * 6.0) + left_arm_ctrl_obs_X.Integrator_CSTATE[left_arm_ctrl_obs_B.i +
      7];
    left_arm_ctrl_obs_B.xp_est[left_arm_ctrl_obs_B.i + 7] =
      left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.i] +
      left_arm_ctrl_obs_B.z[left_arm_ctrl_obs_B.i];
  }

  // End of MATLAB Function: '<Root>/Observer'
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) {
    // MATLABSystem: '<Root>/Coordinate Transformation Conversion'
    left_arm_ctrl_obs_B.vNum = sqrt(left_arm_ctrl_obs_B.T2[0] *
      left_arm_ctrl_obs_B.T2[0] + left_arm_ctrl_obs_B.T2[1] *
      left_arm_ctrl_obs_B.T2[1]);
    left_arm_ctrl_obs_B.CoordinateTransformationConvers[0] = rt_atan2d_snf
      (left_arm_ctrl_obs_B.T2[6], left_arm_ctrl_obs_B.T2[10]);
    left_arm_ctrl_obs_B.CoordinateTransformationConvers[1] = rt_atan2d_snf
      (-left_arm_ctrl_obs_B.T2[2], left_arm_ctrl_obs_B.vNum);
    left_arm_ctrl_obs_B.CoordinateTransformationConvers[2] = rt_atan2d_snf
      (left_arm_ctrl_obs_B.T2[1], left_arm_ctrl_obs_B.T2[0]);
    if (left_arm_ctrl_obs_B.vNum < 2.2204460492503131E-15) {
      left_arm_ctrl_obs_B.loop_ub = 0;
      for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 1;
           left_arm_ctrl_obs_B.i++) {
        left_arm_ctrl_obs_B.loop_ub++;
      }

      left_arm_ctrl_obs_B.rtb_MATLABSystem_size[0] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size[1] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size[2] = left_arm_ctrl_obs_B.loop_ub;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_g[0] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_g[1] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_g[2] =
        left_arm_ctrl_obs_B.loop_ub;
      for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m <
           left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.n_m++) {
        left_arm_ctrl_obs_B.rtb_MATLABSystem_data = -left_arm_ctrl_obs_B.T2[9];
        left_arm_ctrl_obs_B.rtb_MATLABSystem_data_h = left_arm_ctrl_obs_B.T2[5];
        left_arm_ctrl_obs_B.rtb_MATLABSystem_data_l = -left_arm_ctrl_obs_B.T2[2];
      }

      left_arm_ctrl_obs_atan2(&left_arm_ctrl_obs_B.rtb_MATLABSystem_data,
        left_arm_ctrl_obs_B.rtb_MATLABSystem_size,
        &left_arm_ctrl_obs_B.rtb_MATLABSystem_data_h,
        left_arm_ctrl_obs_B.rtb_MATLABSystem_size_g,
        &left_arm_ctrl_obs_B.tmp_data, left_arm_ctrl_obs_B.tmp_size);
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_c[0] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_c[1] = 1;
      left_arm_ctrl_obs_B.rtb_MATLABSystem_size_c[2] =
        left_arm_ctrl_obs_B.loop_ub;
      left_arm_ctrl_obs_B.sy_size[0] = 1;
      left_arm_ctrl_obs_B.sy_size[1] = 1;
      left_arm_ctrl_obs_B.sy_size[2] = left_arm_ctrl_obs_B.loop_ub;
      for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m <
           left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.n_m++) {
        left_arm_ctrl_obs_B.rtb_MATLABSystem_data = left_arm_ctrl_obs_B.vNum;
      }

      left_arm_ctrl_obs_atan2(&left_arm_ctrl_obs_B.rtb_MATLABSystem_data_l,
        left_arm_ctrl_obs_B.rtb_MATLABSystem_size_c,
        &left_arm_ctrl_obs_B.rtb_MATLABSystem_data, left_arm_ctrl_obs_B.sy_size,
        &left_arm_ctrl_obs_B.rtb_MATLABSystem_data_h,
        left_arm_ctrl_obs_B.rtb_MATLABSystem_size);
      left_arm_ctrl_obs_B.loop_ub = left_arm_ctrl_obs_B.tmp_size[2];
      for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m <
           left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.n_m++) {
        left_arm_ctrl_obs_B.CoordinateTransformationConvers[0] =
          left_arm_ctrl_obs_B.tmp_data;
      }

      left_arm_ctrl_obs_B.loop_ub = left_arm_ctrl_obs_B.rtb_MATLABSystem_size[2];
      for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m <
           left_arm_ctrl_obs_B.loop_ub; left_arm_ctrl_obs_B.n_m++) {
        left_arm_ctrl_obs_B.CoordinateTransformationConvers[1] =
          left_arm_ctrl_obs_B.rtb_MATLABSystem_data_h;
      }

      left_arm_ctrl_obs_B.CoordinateTransformationConvers[2] = 0.0;
    }

    left_arm_ctrl_obs_B.vNum =
      left_arm_ctrl_obs_B.CoordinateTransformationConvers[0];
    left_arm_ctrl_obs_B.CoordinateTransformationConvers[0] =
      left_arm_ctrl_obs_B.CoordinateTransformationConvers[2];
    left_arm_ctrl_obs_B.CoordinateTransformationConvers[2] =
      left_arm_ctrl_obs_B.vNum;

    // End of MATLABSystem: '<Root>/Coordinate Transformation Conversion'
  }

  // TransferFcn: '<Root>/Low Pass (z1)'
  left_arm_ctrl_obs_B.vNum = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)'
  left_arm_ctrl_obs_B.bid1 = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)1'
  left_arm_ctrl_obs_B.j = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)2'
  left_arm_ctrl_obs_B.LowPassz22 = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)3'
  left_arm_ctrl_obs_B.LowPassz23 = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)4'
  left_arm_ctrl_obs_B.LowPassz24 = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)5'
  left_arm_ctrl_obs_B.LowPassz25 = 0.0;
  for (left_arm_ctrl_obs_B.loop_ub = 0; left_arm_ctrl_obs_B.loop_ub < 5;
       left_arm_ctrl_obs_B.loop_ub++) {
    // TransferFcn: '<Root>/Low Pass (z1)'
    left_arm_ctrl_obs_B.vNum +=
      left_arm_ctrl_obs_P.LowPassz1_C[left_arm_ctrl_obs_B.loop_ub] *
      left_arm_ctrl_obs_X.LowPassz1_CSTATE[left_arm_ctrl_obs_B.loop_ub];

    // TransferFcn: '<Root>/Low Pass (z2)'
    left_arm_ctrl_obs_B.bid1 +=
      left_arm_ctrl_obs_P.LowPassz2_C[left_arm_ctrl_obs_B.loop_ub] *
      left_arm_ctrl_obs_X.LowPassz2_CSTATE[left_arm_ctrl_obs_B.loop_ub];

    // TransferFcn: '<Root>/Low Pass (z2)1'
    left_arm_ctrl_obs_B.j +=
      left_arm_ctrl_obs_P.LowPassz21_C[left_arm_ctrl_obs_B.loop_ub] *
      left_arm_ctrl_obs_X.LowPassz21_CSTATE[left_arm_ctrl_obs_B.loop_ub];

    // TransferFcn: '<Root>/Low Pass (z2)2'
    left_arm_ctrl_obs_B.LowPassz22 +=
      left_arm_ctrl_obs_P.LowPassz22_C[left_arm_ctrl_obs_B.loop_ub] *
      left_arm_ctrl_obs_X.LowPassz22_CSTATE[left_arm_ctrl_obs_B.loop_ub];

    // TransferFcn: '<Root>/Low Pass (z2)3'
    left_arm_ctrl_obs_B.LowPassz23 +=
      left_arm_ctrl_obs_P.LowPassz23_C[left_arm_ctrl_obs_B.loop_ub] *
      left_arm_ctrl_obs_X.LowPassz23_CSTATE[left_arm_ctrl_obs_B.loop_ub];

    // TransferFcn: '<Root>/Low Pass (z2)4'
    left_arm_ctrl_obs_B.LowPassz24 +=
      left_arm_ctrl_obs_P.LowPassz24_C[left_arm_ctrl_obs_B.loop_ub] *
      left_arm_ctrl_obs_X.LowPassz24_CSTATE[left_arm_ctrl_obs_B.loop_ub];

    // TransferFcn: '<Root>/Low Pass (z2)5'
    left_arm_ctrl_obs_B.LowPassz25 +=
      left_arm_ctrl_obs_P.LowPassz25_C[left_arm_ctrl_obs_B.loop_ub] *
      left_arm_ctrl_obs_X.LowPassz25_CSTATE[left_arm_ctrl_obs_B.loop_ub];
  }

  // MATLAB Function: '<Root>/mass estimator' incorporates:
  //   Integrator: '<Root>/Integrator'

  left_arm_ctrl_obs_B.vel = 0.0;
  left_arm_ctrl_obs_B.scale = 3.3121686421112381E-170;
  for (left_arm_ctrl_obs_B.loop_ub = 0; left_arm_ctrl_obs_B.loop_ub < 7;
       left_arm_ctrl_obs_B.loop_ub++) {
    left_arm_ctrl_obs_B.absxk = fabs
      (left_arm_ctrl_obs_X.Integrator_CSTATE[left_arm_ctrl_obs_B.loop_ub + 7]);
    if (left_arm_ctrl_obs_B.absxk > left_arm_ctrl_obs_B.scale) {
      left_arm_ctrl_obs_B.t = left_arm_ctrl_obs_B.scale /
        left_arm_ctrl_obs_B.absxk;
      left_arm_ctrl_obs_B.vel = left_arm_ctrl_obs_B.vel * left_arm_ctrl_obs_B.t *
        left_arm_ctrl_obs_B.t + 1.0;
      left_arm_ctrl_obs_B.scale = left_arm_ctrl_obs_B.absxk;
    } else {
      left_arm_ctrl_obs_B.t = left_arm_ctrl_obs_B.absxk /
        left_arm_ctrl_obs_B.scale;
      left_arm_ctrl_obs_B.vel += left_arm_ctrl_obs_B.t * left_arm_ctrl_obs_B.t;
    }
  }

  left_arm_ctrl_obs_B.vel = left_arm_ctrl_obs_B.scale * sqrt
    (left_arm_ctrl_obs_B.vel);
  if (fabs(-left_arm_ctrl_obs_B.CoordinateTransformationConvers[0] -
           1.5707963267948966) > 0.2) {
    if (left_arm_ctrl_obs_B.vel < 0.05) {
      // SignalConversion generated from: '<S15>/ SFunction '
      left_arm_ctrl_obs_B.qe[0] = left_arm_ctrl_obs_B.vNum;
      left_arm_ctrl_obs_B.qe[1] = left_arm_ctrl_obs_B.bid1;
      left_arm_ctrl_obs_B.qe[2] = left_arm_ctrl_obs_B.j;
      left_arm_ctrl_obs_B.qe[3] = left_arm_ctrl_obs_B.LowPassz22;
      left_arm_ctrl_obs_B.qe[4] = left_arm_ctrl_obs_B.LowPassz23;
      left_arm_ctrl_obs_B.qe[5] = left_arm_ctrl_obs_B.LowPassz24;
      left_arm_ctrl_obs_B.qe[6] = left_arm_ctrl_obs_B.LowPassz25;
      for (left_arm_ctrl_obs_B.n_m = 0; left_arm_ctrl_obs_B.n_m < 7;
           left_arm_ctrl_obs_B.n_m++) {
        left_arm_ctrl_obs_B.dv8[left_arm_ctrl_obs_B.n_m] = 0.0;
        for (left_arm_ctrl_obs_B.j_a = 0; left_arm_ctrl_obs_B.j_a < 7;
             left_arm_ctrl_obs_B.j_a++) {
          left_arm_ctrl_obs_B.dv8[left_arm_ctrl_obs_B.n_m] +=
            left_arm_ctrl_obs_B.MATLABSystem[7 * left_arm_ctrl_obs_B.j_a +
            left_arm_ctrl_obs_B.n_m] *
            left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.j_a];
        }
      }

      left_arm_ctrl_obs_B.vNum = -left_arm_ctrl_obs_B.dv8[5] / (sin
        (-left_arm_ctrl_obs_B.CoordinateTransformationConvers[0] -
         1.5707963267948966) * 1.9129500000000002);
    } else {
      left_arm_ctrl_obs_B.vNum = 0.0;
    }
  } else {
    left_arm_ctrl_obs_B.vNum = 0.0;
  }

  // End of MATLAB Function: '<Root>/mass estimator'

  // RateTransition: '<Root>/Rate Transition'
  if ((rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
       left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) &&
      (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
       left_arm_ctrl_obs_M->Timing.TaskCounters.TID[2] == 0)) {
    left_arm_ctrl_obs_DW.RateTransition_Buffer = left_arm_ctrl_obs_B.vNum;
  }

  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[2] == 0) {
    // BusAssignment: '<Root>/Bus Assignment2'
    left_arm_ctrl_obs_B.BusAssignment2.Data =
      left_arm_ctrl_obs_DW.RateTransition_Buffer;

    // Outputs for Atomic SubSystem: '<Root>/Publish2'
    // MATLABSystem: '<S11>/SinkBlock'
    Pub_left_arm_ctrl_obs_311.publish(&left_arm_ctrl_obs_B.BusAssignment2);

    // End of Outputs for SubSystem: '<Root>/Publish2'
  }

  // End of RateTransition: '<Root>/Rate Transition'
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) {
    // BusAssignment: '<Root>/Bus Assignment1' incorporates:
    //   Constant: '<Root>/Constant'
    //   Constant: '<S1>/Constant'

    left_arm_ctrl_obs_B.BusAssignment1 = left_arm_ctrl_obs_P.Constant_Value_e;
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.BusAssignment1.Data[left_arm_ctrl_obs_B.i] =
        left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.i];
    }

    left_arm_ctrl_obs_B.BusAssignment1.Data_SL_Info.CurrentLength =
      left_arm_ctrl_obs_P.Constant_Value_d;
    left_arm_ctrl_obs_B.BusAssignment1.Data_SL_Info.ReceivedLength =
      left_arm_ctrl_obs_P.Constant_Value_d;

    // End of BusAssignment: '<Root>/Bus Assignment1'

    // Outputs for Atomic SubSystem: '<Root>/Publish1'
    // MATLABSystem: '<S10>/SinkBlock'
    Pub_left_arm_ctrl_obs_304.publish(&left_arm_ctrl_obs_B.BusAssignment1);

    // End of Outputs for SubSystem: '<Root>/Publish1'

    // BusAssignment: '<Root>/Bus Assignment3' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S3>/Constant'
    //   Integrator: '<Root>/Integrator'

    left_arm_ctrl_obs_B.BusAssignment1 = left_arm_ctrl_obs_P.Constant_Value_i;
    for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
         left_arm_ctrl_obs_B.i++) {
      left_arm_ctrl_obs_B.BusAssignment1.Data[left_arm_ctrl_obs_B.i] =
        left_arm_ctrl_obs_X.Integrator_CSTATE[left_arm_ctrl_obs_B.i + 7];
    }

    left_arm_ctrl_obs_B.BusAssignment1.Data_SL_Info.CurrentLength =
      left_arm_ctrl_obs_P.Constant1_Value;
    left_arm_ctrl_obs_B.BusAssignment1.Data_SL_Info.ReceivedLength =
      left_arm_ctrl_obs_P.Constant1_Value;

    // End of BusAssignment: '<Root>/Bus Assignment3'

    // Outputs for Atomic SubSystem: '<Root>/Publish3'
    // MATLABSystem: '<S12>/SinkBlock'
    Pub_left_arm_ctrl_obs_331.publish(&left_arm_ctrl_obs_B.BusAssignment1);

    // End of Outputs for SubSystem: '<Root>/Publish3'
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
  int_T is;
  XDot_left_arm_ctrl_obs_T *_rtXdot;
  _rtXdot = ((XDot_left_arm_ctrl_obs_T *) left_arm_ctrl_obs_M->derivs);

  // Derivatives for Integrator: '<Root>/Integrator'
  memcpy(&_rtXdot->Integrator_CSTATE[0], &left_arm_ctrl_obs_B.xp_est[0], 14U *
         sizeof(real_T));
  for (is = 0; is < 5; is++) {
    // Derivatives for TransferFcn: '<Root>/Low Pass (z1)'
    _rtXdot->LowPassz1_CSTATE[is] = 0.0;
    _rtXdot->LowPassz1_CSTATE[0] += left_arm_ctrl_obs_P.LowPassz1_A[is] *
      left_arm_ctrl_obs_X.LowPassz1_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)'
    _rtXdot->LowPassz2_CSTATE[is] = 0.0;
    _rtXdot->LowPassz2_CSTATE[0] += left_arm_ctrl_obs_P.LowPassz2_A[is] *
      left_arm_ctrl_obs_X.LowPassz2_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)1'
    _rtXdot->LowPassz21_CSTATE[is] = 0.0;
    _rtXdot->LowPassz21_CSTATE[0] += left_arm_ctrl_obs_P.LowPassz21_A[is] *
      left_arm_ctrl_obs_X.LowPassz21_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)2'
    _rtXdot->LowPassz22_CSTATE[is] = 0.0;
    _rtXdot->LowPassz22_CSTATE[0] += left_arm_ctrl_obs_P.LowPassz22_A[is] *
      left_arm_ctrl_obs_X.LowPassz22_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)3'
    _rtXdot->LowPassz23_CSTATE[is] = 0.0;
    _rtXdot->LowPassz23_CSTATE[0] += left_arm_ctrl_obs_P.LowPassz23_A[is] *
      left_arm_ctrl_obs_X.LowPassz23_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)4'
    _rtXdot->LowPassz24_CSTATE[is] = 0.0;
    _rtXdot->LowPassz24_CSTATE[0] += left_arm_ctrl_obs_P.LowPassz24_A[is] *
      left_arm_ctrl_obs_X.LowPassz24_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)5'
    _rtXdot->LowPassz25_CSTATE[is] = 0.0;
    _rtXdot->LowPassz25_CSTATE[0] += left_arm_ctrl_obs_P.LowPassz25_A[is] *
      left_arm_ctrl_obs_X.LowPassz25_CSTATE[is];
  }

  // Derivatives for TransferFcn: '<Root>/Low Pass (z1)'
  _rtXdot->LowPassz1_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz1_CSTATE[0];
  _rtXdot->LowPassz1_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz1_CSTATE[1];
  _rtXdot->LowPassz1_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz1_CSTATE[2];
  _rtXdot->LowPassz1_CSTATE[4] += left_arm_ctrl_obs_X.LowPassz1_CSTATE[3];
  _rtXdot->LowPassz1_CSTATE[0] += left_arm_ctrl_obs_B.z[0];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)'
  _rtXdot->LowPassz2_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz2_CSTATE[0];
  _rtXdot->LowPassz2_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz2_CSTATE[1];
  _rtXdot->LowPassz2_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz2_CSTATE[2];
  _rtXdot->LowPassz2_CSTATE[4] += left_arm_ctrl_obs_X.LowPassz2_CSTATE[3];
  _rtXdot->LowPassz2_CSTATE[0] += left_arm_ctrl_obs_B.z[1];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)1'
  _rtXdot->LowPassz21_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz21_CSTATE[0];
  _rtXdot->LowPassz21_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz21_CSTATE[1];
  _rtXdot->LowPassz21_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz21_CSTATE[2];
  _rtXdot->LowPassz21_CSTATE[4] += left_arm_ctrl_obs_X.LowPassz21_CSTATE[3];
  _rtXdot->LowPassz21_CSTATE[0] += left_arm_ctrl_obs_B.z[2];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)2'
  _rtXdot->LowPassz22_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz22_CSTATE[0];
  _rtXdot->LowPassz22_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz22_CSTATE[1];
  _rtXdot->LowPassz22_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz22_CSTATE[2];
  _rtXdot->LowPassz22_CSTATE[4] += left_arm_ctrl_obs_X.LowPassz22_CSTATE[3];
  _rtXdot->LowPassz22_CSTATE[0] += left_arm_ctrl_obs_B.z[3];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)3'
  _rtXdot->LowPassz23_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz23_CSTATE[0];
  _rtXdot->LowPassz23_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz23_CSTATE[1];
  _rtXdot->LowPassz23_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz23_CSTATE[2];
  _rtXdot->LowPassz23_CSTATE[4] += left_arm_ctrl_obs_X.LowPassz23_CSTATE[3];
  _rtXdot->LowPassz23_CSTATE[0] += left_arm_ctrl_obs_B.z[4];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)4'
  _rtXdot->LowPassz24_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz24_CSTATE[0];
  _rtXdot->LowPassz24_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz24_CSTATE[1];
  _rtXdot->LowPassz24_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz24_CSTATE[2];
  _rtXdot->LowPassz24_CSTATE[4] += left_arm_ctrl_obs_X.LowPassz24_CSTATE[3];
  _rtXdot->LowPassz24_CSTATE[0] += left_arm_ctrl_obs_B.z[5];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)5'
  _rtXdot->LowPassz25_CSTATE[1] += left_arm_ctrl_obs_X.LowPassz25_CSTATE[0];
  _rtXdot->LowPassz25_CSTATE[2] += left_arm_ctrl_obs_X.LowPassz25_CSTATE[1];
  _rtXdot->LowPassz25_CSTATE[3] += left_arm_ctrl_obs_X.LowPassz25_CSTATE[2];
  _rtXdot->LowPassz25_CSTATE[4] += left_arm_ctrl_obs_X.LowPassz25_CSTATE[3];
  _rtXdot->LowPassz25_CSTATE[0] += left_arm_ctrl_obs_B.z[6];
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
    int_T is;
    char_T tmp[20];
    char_T tmp_0[16];
    char_T tmp_1[29];
    char_T tmp_2[26];
    static const char_T tmp_3[25] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'c', 'u', 'r', 'r', 'e', 'n', 't', '_', 'p', 'o',
      's', 'e' };

    static const char_T tmp_4[28] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'g', 'o', 'a', 'l', '_', 't', 'r', 'a', 'j', 'e',
      'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_5[15] = { '/', 'e', 's', 't', 'i', 'm', 'a', 't',
      'e', 'd', '_', 'm', 'a', 's', 's' };

    static const char_T tmp_6[19] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 't', 'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_7[28] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'e', 's', 't', 'i', 'm', 'a', 't', 'e', 'd', '_',
      's', 'p', 'e', 'e', 'd' };

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S13>/SourceBlock'
    left_arm_ctrl_obs_DW.obj_m.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_m.isInitialized = 1;
    for (is = 0; is < 25; is++) {
      tmp_2[is] = tmp_3[is];
    }

    tmp_2[25] = '\x00';
    Sub_left_arm_ctrl_obs_299.createSubscriber(tmp_2, 1);
    left_arm_ctrl_obs_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'
    emxInitStruct_robotics_slmanip_(&left_arm_ctrl_obs_DW.obj_jz);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_0);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_19);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_18);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_17);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_16);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_15);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_14);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_13);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_12);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_11);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_10);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_9);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_8);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_7);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_6);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_5);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_4);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_3);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_2);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_1);

    // Start for MATLABSystem: '<S7>/MATLAB System'
    left_arm_ctrl_obs_DW.obj_jz.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_jz.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_e0h(&left_arm_ctrl_obs_DW.obj_jz.TreeInternal,
      &left_arm_ctrl_obs_DW.gobj_0, &left_arm_ctrl_obs_DW.gobj_19,
      &left_arm_ctrl_obs_DW.gobj_18, &left_arm_ctrl_obs_DW.gobj_17,
      &left_arm_ctrl_obs_DW.gobj_16, &left_arm_ctrl_obs_DW.gobj_15,
      &left_arm_ctrl_obs_DW.gobj_14, &left_arm_ctrl_obs_DW.gobj_13,
      &left_arm_ctrl_obs_DW.gobj_12, &left_arm_ctrl_obs_DW.gobj_11);
    emxInitStruct_robotics_slmani_e(&left_arm_ctrl_obs_DW.obj_f);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_0_eq);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_19_b);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_18_j);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_17_m);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_16_i);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_15_h);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_14_n);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_13_d);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_12_h);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_11_h);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_10_h);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_9_e);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_8_o);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_7_ia);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_6_a);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_5_h);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_4_h);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_3_o);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_2_f);
    emxInitStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_1_m);

    // Start for MATLABSystem: '<S5>/MATLAB System'
    left_arm_ctrl_obs_DW.obj_f.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_f.isInitialized = 1;
    l_RigidBodyTree_RigidBodyTree_e(&left_arm_ctrl_obs_DW.obj_f.TreeInternal,
      &left_arm_ctrl_obs_DW.gobj_0_eq, &left_arm_ctrl_obs_DW.gobj_19_b,
      &left_arm_ctrl_obs_DW.gobj_18_j, &left_arm_ctrl_obs_DW.gobj_17_m,
      &left_arm_ctrl_obs_DW.gobj_16_i, &left_arm_ctrl_obs_DW.gobj_15_h,
      &left_arm_ctrl_obs_DW.gobj_14_n, &left_arm_ctrl_obs_DW.gobj_13_d,
      &left_arm_ctrl_obs_DW.gobj_12_h, &left_arm_ctrl_obs_DW.gobj_11_h);

    // Start for Atomic SubSystem: '<Root>/Subscribe1'
    // Start for MATLABSystem: '<S14>/SourceBlock'
    left_arm_ctrl_obs_DW.obj_g.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_g.isInitialized = 1;
    for (is = 0; is < 28; is++) {
      tmp_1[is] = tmp_4[is];
    }

    tmp_1[28] = '\x00';
    Sub_left_arm_ctrl_obs_318.createSubscriber(tmp_1, 1);
    left_arm_ctrl_obs_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe1'
    emxInitStruct_robotics_slman_e0(&left_arm_ctrl_obs_DW.obj_j);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_0_f);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_19_h);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_18_b);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_17_d);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_16_a);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_15_k);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_14_a);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_13_l);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_12_g);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_11_b);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_10_n);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_9_k);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_8_g);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_7_a);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_6_k);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_5_g);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_4_p);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_3_j);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_2_p);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_1_o);

    // Start for MATLABSystem: '<S6>/MATLAB System'
    left_arm_ctrl_obs_DW.obj_j.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_j.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_e0(&left_arm_ctrl_obs_DW.obj_j.TreeInternal,
      &left_arm_ctrl_obs_DW.gobj_0_f, &left_arm_ctrl_obs_DW.gobj_19_h,
      &left_arm_ctrl_obs_DW.gobj_18_b, &left_arm_ctrl_obs_DW.gobj_17_d,
      &left_arm_ctrl_obs_DW.gobj_16_a, &left_arm_ctrl_obs_DW.gobj_15_k,
      &left_arm_ctrl_obs_DW.gobj_14_a, &left_arm_ctrl_obs_DW.gobj_13_l,
      &left_arm_ctrl_obs_DW.gobj_12_g, &left_arm_ctrl_obs_DW.gobj_11_b);
    emxInitStruct_robotics_slma_e0h(&left_arm_ctrl_obs_DW.obj);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_0_e);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_19_i);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_18_m);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_17_e);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_16_e);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_15_d);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_14_f);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_13_n);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_12_m);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_11_p);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_10_e);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_9_l);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_8_j);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_7_i);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_6_ke);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_5_e);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_4_f);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_3_f);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_2_b);
    emxInitStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_1_f);

    // Start for MATLABSystem: '<S4>/MATLAB System'
    left_arm_ctrl_obs_DW.obj.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj.isInitialized = 1;
    lef_RigidBodyTree_RigidBodyTree(&left_arm_ctrl_obs_DW.obj.TreeInternal,
      &left_arm_ctrl_obs_DW.gobj_0_e, &left_arm_ctrl_obs_DW.gobj_19_i,
      &left_arm_ctrl_obs_DW.gobj_18_m, &left_arm_ctrl_obs_DW.gobj_17_e,
      &left_arm_ctrl_obs_DW.gobj_16_e, &left_arm_ctrl_obs_DW.gobj_15_d,
      &left_arm_ctrl_obs_DW.gobj_14_f, &left_arm_ctrl_obs_DW.gobj_13_n,
      &left_arm_ctrl_obs_DW.gobj_12_m, &left_arm_ctrl_obs_DW.gobj_11_p);

    // Start for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S11>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_a.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_a.isInitialized = 1;
    for (is = 0; is < 15; is++) {
      tmp_0[is] = tmp_5[is];
    }

    tmp_0[15] = '\x00';
    Pub_left_arm_ctrl_obs_311.createPublisher(tmp_0, 1);
    left_arm_ctrl_obs_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish2'

    // Start for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S10>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_d.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_d.isInitialized = 1;
    for (is = 0; is < 19; is++) {
      tmp[is] = tmp_6[is];
    }

    tmp[19] = '\x00';
    Pub_left_arm_ctrl_obs_304.createPublisher(tmp, 1);
    left_arm_ctrl_obs_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish1'

    // Start for Atomic SubSystem: '<Root>/Publish3'
    // Start for MATLABSystem: '<S12>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_b.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_b.isInitialized = 1;
    for (is = 0; is < 28; is++) {
      tmp_1[is] = tmp_7[is];
    }

    tmp_1[28] = '\x00';
    Pub_left_arm_ctrl_obs_331.createPublisher(tmp_1, 1);
    left_arm_ctrl_obs_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish3'

    // InitializeConditions for Integrator: '<Root>/Integrator'
    memcpy(&left_arm_ctrl_obs_X.Integrator_CSTATE[0],
           &left_arm_ctrl_obs_P.Integrator_IC[0], 14U * sizeof(real_T));
    for (is = 0; is < 5; is++) {
      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z1)'
      left_arm_ctrl_obs_X.LowPassz1_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)'
      left_arm_ctrl_obs_X.LowPassz2_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)1'
      left_arm_ctrl_obs_X.LowPassz21_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)2'
      left_arm_ctrl_obs_X.LowPassz22_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)3'
      left_arm_ctrl_obs_X.LowPassz23_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)4'
      left_arm_ctrl_obs_X.LowPassz24_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)5'
      left_arm_ctrl_obs_X.LowPassz25_CSTATE[is] = 0.0;
    }

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S13>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S16>/Out1'
    left_arm_ctrl_obs_B.In1 = left_arm_ctrl_obs_P.Out1_Y0_a;

    // End of SystemInitialize for SubSystem: '<S13>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S14>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S17>/Out1'
    left_arm_ctrl_obs_B.In1_e = left_arm_ctrl_obs_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S14>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'
  }
}

// Model terminate function
void left_arm_ctrl_obs_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S13>/SourceBlock'
  matlabCodegenHandle_matlabC_e0h(&left_arm_ctrl_obs_DW.obj_m);

  // End of Terminate for SubSystem: '<Root>/Subscribe'
  emxFreeStruct_robotics_slmanip_(&left_arm_ctrl_obs_DW.obj_jz);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_0);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_19);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_18);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_17);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_16);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_15);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_14);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_13);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_12);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_11);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_10);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_9);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_8);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_7);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_6);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_5);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_4);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_3);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_2);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_1);
  emxFreeStruct_robotics_slmani_e(&left_arm_ctrl_obs_DW.obj_f);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_0_eq);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_19_b);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_18_j);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_17_m);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_16_i);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_15_h);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_14_n);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_13_d);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_12_h);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_11_h);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_10_h);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_9_e);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_8_o);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_7_ia);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_6_a);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_5_h);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_4_h);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_3_o);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_2_f);
  emxFreeStruct_j_robotics_mani_e(&left_arm_ctrl_obs_DW.gobj_1_m);

  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S14>/SourceBlock'
  matlabCodegenHandle_matlabC_e0h(&left_arm_ctrl_obs_DW.obj_g);

  // End of Terminate for SubSystem: '<Root>/Subscribe1'
  emxFreeStruct_robotics_slman_e0(&left_arm_ctrl_obs_DW.obj_j);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_0_f);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_19_h);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_18_b);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_17_d);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_16_a);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_15_k);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_14_a);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_13_l);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_12_g);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_11_b);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_10_n);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_9_k);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_8_g);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_7_a);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_6_k);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_5_g);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_4_p);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_3_j);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_2_p);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_1_o);
  emxFreeStruct_robotics_slma_e0h(&left_arm_ctrl_obs_DW.obj);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_0_e);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_19_i);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_18_m);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_17_e);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_16_e);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_15_d);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_14_f);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_13_n);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_12_m);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_11_p);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_10_e);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_9_l);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_8_j);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_7_i);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_6_ke);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_5_e);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_4_f);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_3_f);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_2_b);
  emxFreeStruct_j_robotics_manip_(&left_arm_ctrl_obs_DW.gobj_1_f);

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S11>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_a);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S10>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_d);

  // End of Terminate for SubSystem: '<Root>/Publish1'

  // Terminate for Atomic SubSystem: '<Root>/Publish3'
  // Terminate for MATLABSystem: '<S12>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_b);

  // End of Terminate for SubSystem: '<Root>/Publish3'
}

//
// File trailer for generated code.
//
// [EOF]
//
