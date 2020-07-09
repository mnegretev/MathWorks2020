//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_ctrl_obs.cpp
//
// Code generated for Simulink model 'left_arm_ctrl_obs'.
//
// Model version                  : 1.194
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Thu Jul  9 16:04:37 2020
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
static void left_arm_ct_emxInit_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions);
static void l_emxEnsureCapacity_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
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
static void left_arm_ct_emxFree_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMat_e(k_robotics_manip_internal__e0_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_ctrl_T *H);
static void left_arm_ctrl_SystemCore_step_e(boolean_T *varargout_1, real32_T
  varargout_2_Data[14], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void left_arm_ctrl_obs_eye(real_T b_I[36]);
static void left_arm_ctrl_ob_emxInit_char_T(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray, int32_T numDimensions);
static void left_a_emxEnsureCapacity_char_T(emxArray_char_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel);
static void left_arm_ctrl_ob_emxFree_char_T(emxArray_char_T_left_arm_ctrl_T
  **pEmxArray);
static boolean_T left_arm_ctrl_obs_strcmp(const emxArray_char_T_left_arm_ctrl_T *
  a);
static void lef_GravityTorqueBlock_stepImpl(robotics_slmanip_internal_b_e_T *obj,
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
static void matlabCodegenHandle_matlabC_e0h(ros_slros_internal_block_Subs_T *obj);
static void le_emxFreeStruct_rigidBodyJoint(rigidBodyJoint_left_arm_ctrl__T
  *pStruct);
static void emxFreeStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal__e0_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal__e0_T
  *pStruct);
static void emxFreeStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_k_robotics_mani_e(k_robotics_manip_internal_R_e_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct);
static void emxFreeStruct_k_robotics_man_e0(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slman_e0(robotics_slmanip_internal_blo_T
  *pStruct);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void le_emxInitStruct_rigidBodyJoint(rigidBodyJoint_left_arm_ctrl__T
  *pStruct);
static void emxInitStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal__e0_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal__e0_T
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
static k_robotics_manip_internal__e0_T *RigidBodyTree_RigidBodyTree_e0
  (k_robotics_manip_internal__e0_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9);
static void emxInitStruct_k_robotics_mani_e(k_robotics_manip_internal_R_e_T
  *pStruct);
static void emxInitStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct);
static k_robotics_manip_internal_R_e_T *l_RigidBodyTree_RigidBodyTree_e
  (k_robotics_manip_internal_R_e_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9);
static void emxInitStruct_k_robotics_man_e0(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slman_e0(robotics_slmanip_internal_blo_T
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
  for (left_arm_ctrl_obs_B.i_g = 0; left_arm_ctrl_obs_B.i_g < numDimensions;
       left_arm_ctrl_obs_B.i_g++) {
    emxArray->size[left_arm_ctrl_obs_B.i_g] = 0;
  }
}

static void left_arm_ctrl_o_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  int32_T i;
  *varargout_1 = Sub_left_arm_ctrl_obs_299.getLatestMessage
    (&left_arm_ctrl_obs_B.b_varargout_2);
  for (i = 0; i < 7; i++) {
    varargout_2_Data[i] = left_arm_ctrl_obs_B.b_varargout_2.Data[i];
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
  for (left_arm_ctrl_obs_B.i_n = 0; left_arm_ctrl_obs_B.i_n <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_n++) {
    left_arm_ctrl_obs_B.newNumel *= emxArray->size[left_arm_ctrl_obs_B.i_n];
  }

  if (left_arm_ctrl_obs_B.newNumel > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_n = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_n < 16) {
      left_arm_ctrl_obs_B.i_n = 16;
    }

    while (left_arm_ctrl_obs_B.i_n < left_arm_ctrl_obs_B.newNumel) {
      if (left_arm_ctrl_obs_B.i_n > 1073741823) {
        left_arm_ctrl_obs_B.i_n = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_n <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_n), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_n;
    emxArray->canFreeData = true;
  }
}

static void left_arm_ct_emxInit_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_e_cell_wrap_left_arm_T *emxArray;
  *pEmxArray = (emxArray_e_cell_wrap_left_arm_T *)malloc(sizeof
    (emxArray_e_cell_wrap_left_arm_T));
  emxArray = *pEmxArray;
  emxArray->data = (e_cell_wrap_left_arm_ctrl_obs_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (left_arm_ctrl_obs_B.i_m1 = 0; left_arm_ctrl_obs_B.i_m1 < numDimensions;
       left_arm_ctrl_obs_B.i_m1++) {
    emxArray->size[left_arm_ctrl_obs_B.i_m1] = 0;
  }
}

static void l_emxEnsureCapacity_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_ctrl_obs_B.newNumel_n = 1;
  for (left_arm_ctrl_obs_B.i_k = 0; left_arm_ctrl_obs_B.i_k <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_k++) {
    left_arm_ctrl_obs_B.newNumel_n *= emxArray->size[left_arm_ctrl_obs_B.i_k];
  }

  if (left_arm_ctrl_obs_B.newNumel_n > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_k = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_k < 16) {
      left_arm_ctrl_obs_B.i_k = 16;
    }

    while (left_arm_ctrl_obs_B.i_k < left_arm_ctrl_obs_B.newNumel_n) {
      if (left_arm_ctrl_obs_B.i_k > 1073741823) {
        left_arm_ctrl_obs_B.i_k = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_k <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_k), sizeof
                     (e_cell_wrap_left_arm_ctrl_obs_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(e_cell_wrap_left_arm_ctrl_obs_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (e_cell_wrap_left_arm_ctrl_obs_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_k;
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
  for (left_arm_ctrl_obs_B.b_kstr_o = 0; left_arm_ctrl_obs_B.b_kstr_o < 8;
       left_arm_ctrl_obs_B.b_kstr_o++) {
    left_arm_ctrl_obs_B.b_o2[left_arm_ctrl_obs_B.b_kstr_o] =
      tmp[left_arm_ctrl_obs_B.b_kstr_o];
  }

  left_arm_ctrl_obs_B.b_bool_g = false;
  if (obj->Type->size[1] == 8) {
    left_arm_ctrl_obs_B.b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_o - 1 < 8) {
        left_arm_ctrl_obs_B.kstr_m = left_arm_ctrl_obs_B.b_kstr_o - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr_m] !=
            left_arm_ctrl_obs_B.b_o2[left_arm_ctrl_obs_B.kstr_m]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_o++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_g = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (left_arm_ctrl_obs_B.b_bool_g) {
    guard1 = true;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr_o = 0; left_arm_ctrl_obs_B.b_kstr_o < 9;
         left_arm_ctrl_obs_B.b_kstr_o++) {
      left_arm_ctrl_obs_B.b_i[left_arm_ctrl_obs_B.b_kstr_o] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr_o];
    }

    left_arm_ctrl_obs_B.b_bool_g = false;
    if (obj->Type->size[1] == 9) {
      left_arm_ctrl_obs_B.b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_o - 1 < 9) {
          left_arm_ctrl_obs_B.kstr_m = left_arm_ctrl_obs_B.b_kstr_o - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr_m] !=
              left_arm_ctrl_obs_B.b_i[left_arm_ctrl_obs_B.kstr_m]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_o++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_g = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_g) {
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
  left_arm_ctrl_obs_B.b_h = 1.0 / sqrt((matrix[0] * matrix[0] + matrix[1] *
    matrix[1]) + matrix[2] * matrix[2]);
  normRowMatrix[0] = matrix[0] * left_arm_ctrl_obs_B.b_h;
  normRowMatrix[1] = matrix[1] * left_arm_ctrl_obs_B.b_h;
  normRowMatrix[2] = matrix[2] * left_arm_ctrl_obs_B.b_h;
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
  for (left_arm_ctrl_obs_B.b_kstr_i = 0; left_arm_ctrl_obs_B.b_kstr_i < 5;
       left_arm_ctrl_obs_B.b_kstr_i++) {
    left_arm_ctrl_obs_B.b_fi[left_arm_ctrl_obs_B.b_kstr_i] =
      tmp[left_arm_ctrl_obs_B.b_kstr_i];
  }

  left_arm_ctrl_obs_B.b_bool_cx = false;
  if (obj->Type->size[1] == 5) {
    left_arm_ctrl_obs_B.b_kstr_i = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_i - 1 < 5) {
        left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_i - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr_k] !=
            left_arm_ctrl_obs_B.b_fi[left_arm_ctrl_obs_B.kstr_k]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_i++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_cx = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (left_arm_ctrl_obs_B.b_bool_cx) {
    left_arm_ctrl_obs_B.b_kstr_i = 0;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr_i = 0; left_arm_ctrl_obs_B.b_kstr_i < 8;
         left_arm_ctrl_obs_B.b_kstr_i++) {
      left_arm_ctrl_obs_B.b_ip[left_arm_ctrl_obs_B.b_kstr_i] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr_i];
    }

    left_arm_ctrl_obs_B.b_bool_cx = false;
    if (obj->Type->size[1] == 8) {
      left_arm_ctrl_obs_B.b_kstr_i = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr_i - 1 < 8) {
          left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_i - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr_k] !=
              left_arm_ctrl_obs_B.b_ip[left_arm_ctrl_obs_B.kstr_k]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr_i++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_cx = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_cx) {
      left_arm_ctrl_obs_B.b_kstr_i = 1;
    } else {
      left_arm_ctrl_obs_B.b_kstr_i = -1;
    }
  }

  switch (left_arm_ctrl_obs_B.b_kstr_i) {
   case 0:
    memset(&left_arm_ctrl_obs_B.TJ_c[0], 0, sizeof(real_T) << 4U);
    left_arm_ctrl_obs_B.TJ_c[0] = 1.0;
    left_arm_ctrl_obs_B.TJ_c[5] = 1.0;
    left_arm_ctrl_obs_B.TJ_c[10] = 1.0;
    left_arm_ctrl_obs_B.TJ_c[15] = 1.0;
    break;

   case 1:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_ctrl_obs_B.v_p);
    left_arm_ctrl_obs_B.result_data_j[0] = left_arm_ctrl_obs_B.v_p[0];
    left_arm_ctrl_obs_B.result_data_j[1] = left_arm_ctrl_obs_B.v_p[1];
    left_arm_ctrl_obs_B.result_data_j[2] = left_arm_ctrl_obs_B.v_p[2];
    if (0 <= (*q_size != 0) - 1) {
      left_arm_ctrl_obs_B.result_data_j[3] = q_data[0];
    }

    left_arm_ctrl_obs_normalizeRows(&left_arm_ctrl_obs_B.result_data_j[0],
      left_arm_ctrl_obs_B.v_p);
    left_arm_ctrl_obs_B.cth = cos(left_arm_ctrl_obs_B.result_data_j[3]);
    left_arm_ctrl_obs_B.sth_f = sin(left_arm_ctrl_obs_B.result_data_j[3]);
    left_arm_ctrl_obs_B.tempR_tmp_p = left_arm_ctrl_obs_B.v_p[1] *
      left_arm_ctrl_obs_B.v_p[0] * (1.0 - left_arm_ctrl_obs_B.cth);
    left_arm_ctrl_obs_B.tempR_tmp_e = left_arm_ctrl_obs_B.v_p[2] *
      left_arm_ctrl_obs_B.sth_f;
    left_arm_ctrl_obs_B.tempR_tmp_o = left_arm_ctrl_obs_B.v_p[2] *
      left_arm_ctrl_obs_B.v_p[0] * (1.0 - left_arm_ctrl_obs_B.cth);
    left_arm_ctrl_obs_B.tempR_tmp_h = left_arm_ctrl_obs_B.v_p[1] *
      left_arm_ctrl_obs_B.sth_f;
    left_arm_ctrl_obs_B.tempR_tmp_l = left_arm_ctrl_obs_B.v_p[2] *
      left_arm_ctrl_obs_B.v_p[1] * (1.0 - left_arm_ctrl_obs_B.cth);
    left_arm_ctrl_obs_B.sth_f *= left_arm_ctrl_obs_B.v_p[0];
    left_arm_ctrl_obs_cat(left_arm_ctrl_obs_B.v_p[0] * left_arm_ctrl_obs_B.v_p[0]
                          * (1.0 - left_arm_ctrl_obs_B.cth) +
                          left_arm_ctrl_obs_B.cth,
                          left_arm_ctrl_obs_B.tempR_tmp_p -
                          left_arm_ctrl_obs_B.tempR_tmp_e,
                          left_arm_ctrl_obs_B.tempR_tmp_o +
                          left_arm_ctrl_obs_B.tempR_tmp_h,
                          left_arm_ctrl_obs_B.tempR_tmp_p +
                          left_arm_ctrl_obs_B.tempR_tmp_e,
                          left_arm_ctrl_obs_B.v_p[1] * left_arm_ctrl_obs_B.v_p[1]
                          * (1.0 - left_arm_ctrl_obs_B.cth) +
                          left_arm_ctrl_obs_B.cth,
                          left_arm_ctrl_obs_B.tempR_tmp_l -
                          left_arm_ctrl_obs_B.sth_f,
                          left_arm_ctrl_obs_B.tempR_tmp_o -
                          left_arm_ctrl_obs_B.tempR_tmp_h,
                          left_arm_ctrl_obs_B.tempR_tmp_l +
                          left_arm_ctrl_obs_B.sth_f, left_arm_ctrl_obs_B.v_p[2] *
                          left_arm_ctrl_obs_B.v_p[2] * (1.0 -
      left_arm_ctrl_obs_B.cth) + left_arm_ctrl_obs_B.cth,
                          left_arm_ctrl_obs_B.tempR_n);
    for (left_arm_ctrl_obs_B.b_kstr_i = 0; left_arm_ctrl_obs_B.b_kstr_i < 3;
         left_arm_ctrl_obs_B.b_kstr_i++) {
      left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_i + 1;
      left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.kstr_k - 1] =
        left_arm_ctrl_obs_B.tempR_n[(left_arm_ctrl_obs_B.kstr_k - 1) * 3];
      left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_i + 1;
      left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.kstr_k + 2] =
        left_arm_ctrl_obs_B.tempR_n[(left_arm_ctrl_obs_B.kstr_k - 1) * 3 + 1];
      left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_i + 1;
      left_arm_ctrl_obs_B.R_b[left_arm_ctrl_obs_B.kstr_k + 5] =
        left_arm_ctrl_obs_B.tempR_n[(left_arm_ctrl_obs_B.kstr_k - 1) * 3 + 2];
    }

    memset(&left_arm_ctrl_obs_B.TJ_c[0], 0, sizeof(real_T) << 4U);
    for (left_arm_ctrl_obs_B.b_kstr_i = 0; left_arm_ctrl_obs_B.b_kstr_i < 3;
         left_arm_ctrl_obs_B.b_kstr_i++) {
      left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_i << 2;
      left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.kstr_k] =
        left_arm_ctrl_obs_B.R_b[3 * left_arm_ctrl_obs_B.b_kstr_i];
      left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.kstr_k + 1] =
        left_arm_ctrl_obs_B.R_b[3 * left_arm_ctrl_obs_B.b_kstr_i + 1];
      left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.kstr_k + 2] =
        left_arm_ctrl_obs_B.R_b[3 * left_arm_ctrl_obs_B.b_kstr_i + 2];
    }

    left_arm_ctrl_obs_B.TJ_c[15] = 1.0;
    break;

   default:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_ctrl_obs_B.v_p);
    memset(&left_arm_ctrl_obs_B.tempR_n[0], 0, 9U * sizeof(real_T));
    left_arm_ctrl_obs_B.tempR_n[0] = 1.0;
    left_arm_ctrl_obs_B.tempR_n[4] = 1.0;
    left_arm_ctrl_obs_B.tempR_n[8] = 1.0;
    for (left_arm_ctrl_obs_B.b_kstr_i = 0; left_arm_ctrl_obs_B.b_kstr_i < 3;
         left_arm_ctrl_obs_B.b_kstr_i++) {
      left_arm_ctrl_obs_B.kstr_k = left_arm_ctrl_obs_B.b_kstr_i << 2;
      left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.kstr_k] =
        left_arm_ctrl_obs_B.tempR_n[3 * left_arm_ctrl_obs_B.b_kstr_i];
      left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.kstr_k + 1] =
        left_arm_ctrl_obs_B.tempR_n[3 * left_arm_ctrl_obs_B.b_kstr_i + 1];
      left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.kstr_k + 2] =
        left_arm_ctrl_obs_B.tempR_n[3 * left_arm_ctrl_obs_B.b_kstr_i + 2];
      left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.b_kstr_i + 12] =
        left_arm_ctrl_obs_B.v_p[left_arm_ctrl_obs_B.b_kstr_i] * q_data[0];
    }

    left_arm_ctrl_obs_B.TJ_c[3] = 0.0;
    left_arm_ctrl_obs_B.TJ_c[7] = 0.0;
    left_arm_ctrl_obs_B.TJ_c[11] = 0.0;
    left_arm_ctrl_obs_B.TJ_c[15] = 1.0;
    break;
  }

  for (left_arm_ctrl_obs_B.b_kstr_i = 0; left_arm_ctrl_obs_B.b_kstr_i < 4;
       left_arm_ctrl_obs_B.b_kstr_i++) {
    for (left_arm_ctrl_obs_B.kstr_k = 0; left_arm_ctrl_obs_B.kstr_k < 4;
         left_arm_ctrl_obs_B.kstr_k++) {
      left_arm_ctrl_obs_B.obj_tmp_tmp_m = left_arm_ctrl_obs_B.kstr_k << 2;
      left_arm_ctrl_obs_B.obj_tmp_o = left_arm_ctrl_obs_B.b_kstr_i +
        left_arm_ctrl_obs_B.obj_tmp_tmp_m;
      left_arm_ctrl_obs_B.obj_f[left_arm_ctrl_obs_B.obj_tmp_o] = 0.0;
      left_arm_ctrl_obs_B.obj_f[left_arm_ctrl_obs_B.obj_tmp_o] +=
        left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.obj_tmp_tmp_m] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_i];
      left_arm_ctrl_obs_B.obj_f[left_arm_ctrl_obs_B.obj_tmp_o] +=
        left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.obj_tmp_tmp_m + 1] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_i + 4];
      left_arm_ctrl_obs_B.obj_f[left_arm_ctrl_obs_B.obj_tmp_o] +=
        left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.obj_tmp_tmp_m + 2] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_i + 8];
      left_arm_ctrl_obs_B.obj_f[left_arm_ctrl_obs_B.obj_tmp_o] +=
        left_arm_ctrl_obs_B.TJ_c[left_arm_ctrl_obs_B.obj_tmp_tmp_m + 3] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr_i + 12];
    }

    for (left_arm_ctrl_obs_B.kstr_k = 0; left_arm_ctrl_obs_B.kstr_k < 4;
         left_arm_ctrl_obs_B.kstr_k++) {
      left_arm_ctrl_obs_B.obj_tmp_tmp_m = left_arm_ctrl_obs_B.kstr_k << 2;
      left_arm_ctrl_obs_B.obj_tmp_o = left_arm_ctrl_obs_B.b_kstr_i +
        left_arm_ctrl_obs_B.obj_tmp_tmp_m;
      T[left_arm_ctrl_obs_B.obj_tmp_o] = 0.0;
      T[left_arm_ctrl_obs_B.obj_tmp_o] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_m] *
        left_arm_ctrl_obs_B.obj_f[left_arm_ctrl_obs_B.b_kstr_i];
      T[left_arm_ctrl_obs_B.obj_tmp_o] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_m + 1] *
        left_arm_ctrl_obs_B.obj_f[left_arm_ctrl_obs_B.b_kstr_i + 4];
      T[left_arm_ctrl_obs_B.obj_tmp_o] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_m + 2] *
        left_arm_ctrl_obs_B.obj_f[left_arm_ctrl_obs_B.b_kstr_i + 8];
      T[left_arm_ctrl_obs_B.obj_tmp_o] += obj->
        ChildToJointTransform[left_arm_ctrl_obs_B.obj_tmp_tmp_m + 3] *
        left_arm_ctrl_obs_B.obj_f[left_arm_ctrl_obs_B.b_kstr_i + 12];
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
    left_arm_ctrl_obs_B.b_g[left_arm_ctrl_obs_B.b_kstr] =
      tmp[left_arm_ctrl_obs_B.b_kstr];
  }

  left_arm_ctrl_obs_B.b_bool_d = false;
  if (obj->Type->size[1] == 5) {
    left_arm_ctrl_obs_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr - 1 < 5) {
        left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr - 1;
        if (obj->Type->data[left_arm_ctrl_obs_B.kstr] !=
            left_arm_ctrl_obs_B.b_g[left_arm_ctrl_obs_B.kstr]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr++;
        }
      } else {
        left_arm_ctrl_obs_B.b_bool_d = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (left_arm_ctrl_obs_B.b_bool_d) {
    left_arm_ctrl_obs_B.b_kstr = 0;
  } else {
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 8;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.b_o[left_arm_ctrl_obs_B.b_kstr] =
        tmp_0[left_arm_ctrl_obs_B.b_kstr];
    }

    left_arm_ctrl_obs_B.b_bool_d = false;
    if (obj->Type->size[1] == 8) {
      left_arm_ctrl_obs_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.b_kstr - 1 < 8) {
          left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr - 1;
          if (obj->Type->data[left_arm_ctrl_obs_B.kstr] !=
              left_arm_ctrl_obs_B.b_o[left_arm_ctrl_obs_B.kstr]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.b_kstr++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_d = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_ctrl_obs_B.b_bool_d) {
      left_arm_ctrl_obs_B.b_kstr = 1;
    } else {
      left_arm_ctrl_obs_B.b_kstr = -1;
    }
  }

  switch (left_arm_ctrl_obs_B.b_kstr) {
   case 0:
    memset(&left_arm_ctrl_obs_B.TJ_b[0], 0, sizeof(real_T) << 4U);
    left_arm_ctrl_obs_B.TJ_b[0] = 1.0;
    left_arm_ctrl_obs_B.TJ_b[5] = 1.0;
    left_arm_ctrl_obs_B.TJ_b[10] = 1.0;
    left_arm_ctrl_obs_B.TJ_b[15] = 1.0;
    break;

   case 1:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_ctrl_obs_B.v_c);
    left_arm_ctrl_obs_B.v_ct[0] = left_arm_ctrl_obs_B.v_c[0];
    left_arm_ctrl_obs_B.v_ct[1] = left_arm_ctrl_obs_B.v_c[1];
    left_arm_ctrl_obs_B.v_ct[2] = left_arm_ctrl_obs_B.v_c[2];
    left_arm_ctrl_obs_normalizeRows(left_arm_ctrl_obs_B.v_ct,
      left_arm_ctrl_obs_B.v_c);
    left_arm_ctrl_obs_B.tempR_tmp_f = left_arm_ctrl_obs_B.v_c[1] *
      left_arm_ctrl_obs_B.v_c[0] * 0.0;
    left_arm_ctrl_obs_B.tempR_tmp_g = left_arm_ctrl_obs_B.v_c[2] *
      left_arm_ctrl_obs_B.v_c[0] * 0.0;
    left_arm_ctrl_obs_B.tempR_tmp_c = left_arm_ctrl_obs_B.v_c[2] *
      left_arm_ctrl_obs_B.v_c[1] * 0.0;
    left_arm_ctrl_obs_cat(left_arm_ctrl_obs_B.v_c[0] * left_arm_ctrl_obs_B.v_c[0]
                          * 0.0 + 1.0, left_arm_ctrl_obs_B.tempR_tmp_f -
                          left_arm_ctrl_obs_B.v_c[2] * 0.0,
                          left_arm_ctrl_obs_B.tempR_tmp_g +
                          left_arm_ctrl_obs_B.v_c[1] * 0.0,
                          left_arm_ctrl_obs_B.tempR_tmp_f +
                          left_arm_ctrl_obs_B.v_c[2] * 0.0,
                          left_arm_ctrl_obs_B.v_c[1] * left_arm_ctrl_obs_B.v_c[1]
                          * 0.0 + 1.0, left_arm_ctrl_obs_B.tempR_tmp_c -
                          left_arm_ctrl_obs_B.v_c[0] * 0.0,
                          left_arm_ctrl_obs_B.tempR_tmp_g -
                          left_arm_ctrl_obs_B.v_c[1] * 0.0,
                          left_arm_ctrl_obs_B.tempR_tmp_c +
                          left_arm_ctrl_obs_B.v_c[0] * 0.0,
                          left_arm_ctrl_obs_B.v_c[2] * left_arm_ctrl_obs_B.v_c[2]
                          * 0.0 + 1.0, left_arm_ctrl_obs_B.tempR_d);
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 3;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr + 1;
      left_arm_ctrl_obs_B.R_d[left_arm_ctrl_obs_B.kstr - 1] =
        left_arm_ctrl_obs_B.tempR_d[(left_arm_ctrl_obs_B.kstr - 1) * 3];
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr + 1;
      left_arm_ctrl_obs_B.R_d[left_arm_ctrl_obs_B.kstr + 2] =
        left_arm_ctrl_obs_B.tempR_d[(left_arm_ctrl_obs_B.kstr - 1) * 3 + 1];
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr + 1;
      left_arm_ctrl_obs_B.R_d[left_arm_ctrl_obs_B.kstr + 5] =
        left_arm_ctrl_obs_B.tempR_d[(left_arm_ctrl_obs_B.kstr - 1) * 3 + 2];
    }

    memset(&left_arm_ctrl_obs_B.TJ_b[0], 0, sizeof(real_T) << 4U);
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 3;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr << 2;
      left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.kstr] =
        left_arm_ctrl_obs_B.R_d[3 * left_arm_ctrl_obs_B.b_kstr];
      left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.kstr + 1] =
        left_arm_ctrl_obs_B.R_d[3 * left_arm_ctrl_obs_B.b_kstr + 1];
      left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.kstr + 2] =
        left_arm_ctrl_obs_B.R_d[3 * left_arm_ctrl_obs_B.b_kstr + 2];
    }

    left_arm_ctrl_obs_B.TJ_b[15] = 1.0;
    break;

   default:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_ctrl_obs_B.v_c);
    memset(&left_arm_ctrl_obs_B.tempR_d[0], 0, 9U * sizeof(real_T));
    left_arm_ctrl_obs_B.tempR_d[0] = 1.0;
    left_arm_ctrl_obs_B.tempR_d[4] = 1.0;
    left_arm_ctrl_obs_B.tempR_d[8] = 1.0;
    for (left_arm_ctrl_obs_B.b_kstr = 0; left_arm_ctrl_obs_B.b_kstr < 3;
         left_arm_ctrl_obs_B.b_kstr++) {
      left_arm_ctrl_obs_B.kstr = left_arm_ctrl_obs_B.b_kstr << 2;
      left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.kstr] =
        left_arm_ctrl_obs_B.tempR_d[3 * left_arm_ctrl_obs_B.b_kstr];
      left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.kstr + 1] =
        left_arm_ctrl_obs_B.tempR_d[3 * left_arm_ctrl_obs_B.b_kstr + 1];
      left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.kstr + 2] =
        left_arm_ctrl_obs_B.tempR_d[3 * left_arm_ctrl_obs_B.b_kstr + 2];
      left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.b_kstr + 12] =
        left_arm_ctrl_obs_B.v_c[left_arm_ctrl_obs_B.b_kstr] * 0.0;
    }

    left_arm_ctrl_obs_B.TJ_b[3] = 0.0;
    left_arm_ctrl_obs_B.TJ_b[7] = 0.0;
    left_arm_ctrl_obs_B.TJ_b[11] = 0.0;
    left_arm_ctrl_obs_B.TJ_b[15] = 1.0;
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
        left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.obj_tmp_tmp] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr];
      left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.obj_tmp] +=
        left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.obj_tmp_tmp + 1] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr + 4];
      left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.obj_tmp] +=
        left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.obj_tmp_tmp + 2] *
        obj->JointToParentTransform[left_arm_ctrl_obs_B.b_kstr + 8];
      left_arm_ctrl_obs_B.obj[left_arm_ctrl_obs_B.obj_tmp] +=
        left_arm_ctrl_obs_B.TJ_b[left_arm_ctrl_obs_B.obj_tmp_tmp + 3] *
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
    left_arm_ctrl_obs_B.R_bs[3 * left_arm_ctrl_obs_B.i3] =
      T[left_arm_ctrl_obs_B.i3];
    left_arm_ctrl_obs_B.R_bs[3 * left_arm_ctrl_obs_B.i3 + 1] =
      T[left_arm_ctrl_obs_B.i3 + 4];
    left_arm_ctrl_obs_B.R_bs[3 * left_arm_ctrl_obs_B.i3 + 2] =
      T[left_arm_ctrl_obs_B.i3 + 8];
  }

  for (left_arm_ctrl_obs_B.i3 = 0; left_arm_ctrl_obs_B.i3 < 9;
       left_arm_ctrl_obs_B.i3++) {
    left_arm_ctrl_obs_B.R_ln[left_arm_ctrl_obs_B.i3] =
      -left_arm_ctrl_obs_B.R_bs[left_arm_ctrl_obs_B.i3];
  }

  for (left_arm_ctrl_obs_B.i3 = 0; left_arm_ctrl_obs_B.i3 < 3;
       left_arm_ctrl_obs_B.i3++) {
    left_arm_ctrl_obs_B.Tinv_tmp = left_arm_ctrl_obs_B.i3 << 2;
    Tinv[left_arm_ctrl_obs_B.Tinv_tmp] = left_arm_ctrl_obs_B.R_bs[3 *
      left_arm_ctrl_obs_B.i3];
    Tinv[left_arm_ctrl_obs_B.Tinv_tmp + 1] = left_arm_ctrl_obs_B.R_bs[3 *
      left_arm_ctrl_obs_B.i3 + 1];
    Tinv[left_arm_ctrl_obs_B.Tinv_tmp + 2] = left_arm_ctrl_obs_B.R_bs[3 *
      left_arm_ctrl_obs_B.i3 + 2];
    Tinv[left_arm_ctrl_obs_B.i3 + 12] =
      left_arm_ctrl_obs_B.R_ln[left_arm_ctrl_obs_B.i3 + 6] * T[14] +
      (left_arm_ctrl_obs_B.R_ln[left_arm_ctrl_obs_B.i3 + 3] * T[13] +
       left_arm_ctrl_obs_B.R_ln[left_arm_ctrl_obs_B.i3] * T[12]);
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

static void RigidBodyTreeDynamics_massMat_e(k_robotics_manip_internal__e0_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_ctrl_T *H)
{
  emxArray_e_cell_wrap_left_arm_T *Ic;
  emxArray_e_cell_wrap_left_arm_T *X;
  emxArray_real_T_left_arm_ctrl_T *Si;
  emxArray_real_T_left_arm_ctrl_T *Fi;
  emxArray_real_T_left_arm_ctrl_T *Sj;
  emxArray_real_T_left_arm_ctrl_T *Hji;
  j_robotics_manip_internal_Rig_T *obj;
  emxArray_real_T_left_arm_ctrl_T *a;
  emxArray_real_T_left_arm_ctrl_T *B;
  left_arm_ctrl_obs_B.nb_k = robot->NumBodies;
  left_arm_ctrl_obs_B.vNum_p = robot->VelocityNumber;
  left_arm_ctrl_obs_B.f = H->size[0] * H->size[1];
  left_arm_ctrl_obs_B.b_i_n = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum_p);
  H->size[0] = left_arm_ctrl_obs_B.b_i_n;
  H->size[1] = left_arm_ctrl_obs_B.b_i_n;
  left_a_emxEnsureCapacity_real_T(H, left_arm_ctrl_obs_B.f);
  left_arm_ctrl_obs_B.n_i = left_arm_ctrl_obs_B.b_i_n *
    left_arm_ctrl_obs_B.b_i_n - 1;
  for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
       left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.f++) {
    H->data[left_arm_ctrl_obs_B.f] = 0.0;
  }

  left_arm_ct_emxInit_e_cell_wrap(&Ic, 2);
  left_arm_ct_emxInit_e_cell_wrap(&X, 2);
  left_arm_ctrl_obs_B.c_tmp = static_cast<int32_T>(left_arm_ctrl_obs_B.nb_k);
  left_arm_ctrl_obs_B.c = left_arm_ctrl_obs_B.c_tmp - 1;
  left_arm_ctrl_obs_B.f = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = left_arm_ctrl_obs_B.c_tmp;
  l_emxEnsureCapacity_e_cell_wrap(Ic, left_arm_ctrl_obs_B.f);
  left_arm_ctrl_obs_B.f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_ctrl_obs_B.c_tmp;
  l_emxEnsureCapacity_e_cell_wrap(X, left_arm_ctrl_obs_B.f);
  for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n <=
       left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.b_i_n++) {
    for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 36;
         left_arm_ctrl_obs_B.f++) {
      Ic->data[left_arm_ctrl_obs_B.b_i_n].f1[left_arm_ctrl_obs_B.f] =
        robot->Bodies[left_arm_ctrl_obs_B.b_i_n]->
        SpatialInertia[left_arm_ctrl_obs_B.f];
    }

    left_arm_ctrl_obs_B.vNum_p = robot->PositionDoFMap[left_arm_ctrl_obs_B.b_i_n];
    left_arm_ctrl_obs_B.p_idx_1_a = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.b_i_n + 10];
    if (left_arm_ctrl_obs_B.p_idx_1_a < left_arm_ctrl_obs_B.vNum_p) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.b_i_n];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        left_arm_ctrl_obs_B.T_g1);
    } else {
      if (left_arm_ctrl_obs_B.vNum_p > left_arm_ctrl_obs_B.p_idx_1_a) {
        left_arm_ctrl_obs_B.c_tmp = 0;
        left_arm_ctrl_obs_B.f = -1;
      } else {
        left_arm_ctrl_obs_B.c_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.vNum_p) - 1;
        left_arm_ctrl_obs_B.f = static_cast<int32_T>
          (left_arm_ctrl_obs_B.p_idx_1_a) - 1;
      }

      obj = robot->Bodies[left_arm_ctrl_obs_B.b_i_n];
      left_arm_ctrl_obs_B.q_size_tmp_n = left_arm_ctrl_obs_B.f -
        left_arm_ctrl_obs_B.c_tmp;
      left_arm_ctrl_obs_B.q_size_a = left_arm_ctrl_obs_B.q_size_tmp_n + 1;
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
           left_arm_ctrl_obs_B.q_size_tmp_n; left_arm_ctrl_obs_B.f++) {
        left_arm_ctrl_obs_B.q_data_n[left_arm_ctrl_obs_B.f] =
          q[left_arm_ctrl_obs_B.c_tmp + left_arm_ctrl_obs_B.f];
      }

      rigidBodyJoint_transformBodyT_e(&obj->JointInternal,
        left_arm_ctrl_obs_B.q_data_n, &left_arm_ctrl_obs_B.q_size_a,
        left_arm_ctrl_obs_B.T_g1);
    }

    left_arm_ctrl_obs_tforminv(left_arm_ctrl_obs_B.T_g1, left_arm_ctrl_obs_B.dv1);
    left_arm_ct_tformToSpatialXform(left_arm_ctrl_obs_B.dv1, X->
      data[left_arm_ctrl_obs_B.b_i_n].f1);
  }

  left_arm_ctrl_obs_B.c = static_cast<int32_T>(((-1.0 - left_arm_ctrl_obs_B.nb_k)
    + 1.0) / -1.0) - 1;
  left_arm_ctrl_ob_emxInit_real_T(&Si, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Fi, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Sj, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Hji, 2);
  left_arm_ctrl_ob_emxInit_real_T(&a, 2);
  left_arm_ctrl_ob_emxInit_real_T(&B, 2);
  for (left_arm_ctrl_obs_B.c_tmp = 0; left_arm_ctrl_obs_B.c_tmp <=
       left_arm_ctrl_obs_B.c; left_arm_ctrl_obs_B.c_tmp++) {
    left_arm_ctrl_obs_B.pid_tmp_g = static_cast<int32_T>
      (left_arm_ctrl_obs_B.nb_k + -static_cast<real_T>(left_arm_ctrl_obs_B.c_tmp));
    left_arm_ctrl_obs_B.q_size_tmp_n = left_arm_ctrl_obs_B.pid_tmp_g - 1;
    left_arm_ctrl_obs_B.pid_p = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp_n
      ]->ParentIndex;
    left_arm_ctrl_obs_B.vNum_p = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_g - 1];
    left_arm_ctrl_obs_B.p_idx_1_a = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_g + 9];
    if (left_arm_ctrl_obs_B.pid_p > 0.0) {
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
             left_arm_ctrl_obs_B.b_i_n++) {
          left_arm_ctrl_obs_B.X_tmp_d = left_arm_ctrl_obs_B.f + 6 *
            left_arm_ctrl_obs_B.b_i_n;
          left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.X_tmp_d] = 0.0;
          for (left_arm_ctrl_obs_B.n_i = 0; left_arm_ctrl_obs_B.n_i < 6;
               left_arm_ctrl_obs_B.n_i++) {
            left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.X_tmp_d] += X->
              data[left_arm_ctrl_obs_B.q_size_tmp_n].f1[6 *
              left_arm_ctrl_obs_B.f + left_arm_ctrl_obs_B.n_i] * Ic->
              data[left_arm_ctrl_obs_B.q_size_tmp_n].f1[6 *
              left_arm_ctrl_obs_B.b_i_n + left_arm_ctrl_obs_B.n_i];
          }
        }
      }

      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
             left_arm_ctrl_obs_B.b_i_n++) {
          left_arm_ctrl_obs_B.b_idx_0_j = 0.0;
          for (left_arm_ctrl_obs_B.n_i = 0; left_arm_ctrl_obs_B.n_i < 6;
               left_arm_ctrl_obs_B.n_i++) {
            left_arm_ctrl_obs_B.b_idx_0_j += left_arm_ctrl_obs_B.X_k[6 *
              left_arm_ctrl_obs_B.n_i + left_arm_ctrl_obs_B.f] * X->
              data[left_arm_ctrl_obs_B.q_size_tmp_n].f1[6 *
              left_arm_ctrl_obs_B.b_i_n + left_arm_ctrl_obs_B.n_i];
          }

          left_arm_ctrl_obs_B.n_i = 6 * left_arm_ctrl_obs_B.b_i_n +
            left_arm_ctrl_obs_B.f;
          Ic->data[static_cast<int32_T>(left_arm_ctrl_obs_B.pid_p) - 1]
            .f1[left_arm_ctrl_obs_B.n_i] += left_arm_ctrl_obs_B.b_idx_0_j;
        }
      }
    }

    left_arm_ctrl_obs_B.b_idx_0_j = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_g - 1];
    left_arm_ctrl_obs_B.b_idx_1_e = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_g + 9];
    if (left_arm_ctrl_obs_B.b_idx_0_j <= left_arm_ctrl_obs_B.b_idx_1_e) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp_n];
      left_arm_ctrl_obs_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(Si, left_arm_ctrl_obs_B.f);
      left_arm_ctrl_obs_B.n_i = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
           left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.f++) {
        Si->data[left_arm_ctrl_obs_B.f] = obj->
          JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.f];
      }

      left_arm_ctrl_obs_B.n_i = Si->size[1] - 1;
      left_arm_ctrl_obs_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.b_j_e = 0; left_arm_ctrl_obs_B.b_j_e <=
           left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.b_j_e++) {
        left_arm_ctrl_obs_B.pid_tmp_g = left_arm_ctrl_obs_B.b_j_e * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
             left_arm_ctrl_obs_B.b_i_n++) {
          Fi->data[(left_arm_ctrl_obs_B.pid_tmp_g + left_arm_ctrl_obs_B.b_i_n) +
            1] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
             left_arm_ctrl_obs_B.b_i_n++) {
          left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.b_i_n * 6 - 1;
          left_arm_ctrl_obs_B.temp_p = Si->data[(left_arm_ctrl_obs_B.pid_tmp_g +
            left_arm_ctrl_obs_B.b_i_n) + 1];
          for (left_arm_ctrl_obs_B.c_i_m = 0; left_arm_ctrl_obs_B.c_i_m < 6;
               left_arm_ctrl_obs_B.c_i_m++) {
            left_arm_ctrl_obs_B.i_j = left_arm_ctrl_obs_B.c_i_m + 1;
            left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.pid_tmp_g +
              left_arm_ctrl_obs_B.i_j;
            Fi->data[left_arm_ctrl_obs_B.f] += Ic->
              data[left_arm_ctrl_obs_B.q_size_tmp_n]
              .f1[left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_j] *
              left_arm_ctrl_obs_B.temp_p;
          }
        }
      }

      if (left_arm_ctrl_obs_B.vNum_p > left_arm_ctrl_obs_B.p_idx_1_a) {
        left_arm_ctrl_obs_B.pid_tmp_g = 0;
        left_arm_ctrl_obs_B.X_tmp_d = 0;
      } else {
        left_arm_ctrl_obs_B.pid_tmp_g = static_cast<int32_T>
          (left_arm_ctrl_obs_B.vNum_p) - 1;
        left_arm_ctrl_obs_B.X_tmp_d = left_arm_ctrl_obs_B.pid_tmp_g;
      }

      left_arm_ctrl_obs_B.f = a->size[0] * a->size[1];
      a->size[0] = Si->size[1];
      a->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        left_arm_ctrl_obs_B.n_i = Si->size[1];
        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n <
             left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.b_i_n++) {
          a->data[left_arm_ctrl_obs_B.b_i_n + a->size[0] * left_arm_ctrl_obs_B.f]
            = Si->data[6 * left_arm_ctrl_obs_B.b_i_n + left_arm_ctrl_obs_B.f];
        }
      }

      left_arm_ctrl_obs_B.m_m = a->size[0];
      left_arm_ctrl_obs_B.n_i = Fi->size[1] - 1;
      left_arm_ctrl_obs_B.f = Hji->size[0] * Hji->size[1];
      Hji->size[0] = a->size[0];
      Hji->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(Hji, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.b_j_e = 0; left_arm_ctrl_obs_B.b_j_e <=
           left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.b_j_e++) {
        left_arm_ctrl_obs_B.coffset_j = left_arm_ctrl_obs_B.b_j_e *
          left_arm_ctrl_obs_B.m_m - 1;
        left_arm_ctrl_obs_B.boffset_f = left_arm_ctrl_obs_B.b_j_e * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n <
             left_arm_ctrl_obs_B.m_m; left_arm_ctrl_obs_B.b_i_n++) {
          Hji->data[(left_arm_ctrl_obs_B.coffset_j + left_arm_ctrl_obs_B.b_i_n)
            + 1] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
             left_arm_ctrl_obs_B.b_i_n++) {
          left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.b_i_n *
            left_arm_ctrl_obs_B.m_m - 1;
          left_arm_ctrl_obs_B.temp_p = Fi->data[(left_arm_ctrl_obs_B.boffset_f +
            left_arm_ctrl_obs_B.b_i_n) + 1];
          for (left_arm_ctrl_obs_B.c_i_m = 0; left_arm_ctrl_obs_B.c_i_m <
               left_arm_ctrl_obs_B.m_m; left_arm_ctrl_obs_B.c_i_m++) {
            left_arm_ctrl_obs_B.i_j = left_arm_ctrl_obs_B.c_i_m + 1;
            left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.coffset_j +
              left_arm_ctrl_obs_B.i_j;
            Hji->data[left_arm_ctrl_obs_B.f] += a->
              data[left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_j] *
              left_arm_ctrl_obs_B.temp_p;
          }
        }
      }

      left_arm_ctrl_obs_B.n_i = Hji->size[1];
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <
           left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.f++) {
        left_arm_ctrl_obs_B.b_j_e = Hji->size[0];
        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n <
             left_arm_ctrl_obs_B.b_j_e; left_arm_ctrl_obs_B.b_i_n++) {
          H->data[(left_arm_ctrl_obs_B.pid_tmp_g + left_arm_ctrl_obs_B.b_i_n) +
            H->size[0] * (left_arm_ctrl_obs_B.X_tmp_d + left_arm_ctrl_obs_B.f)] =
            Hji->data[Hji->size[0] * left_arm_ctrl_obs_B.f +
            left_arm_ctrl_obs_B.b_i_n];
        }
      }

      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
           left_arm_ctrl_obs_B.f++) {
        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
             left_arm_ctrl_obs_B.b_i_n++) {
          left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.b_i_n + 6 *
            left_arm_ctrl_obs_B.f] = X->data[left_arm_ctrl_obs_B.q_size_tmp_n].
            f1[6 * left_arm_ctrl_obs_B.b_i_n + left_arm_ctrl_obs_B.f];
        }
      }

      left_arm_ctrl_obs_B.f = B->size[0] * B->size[1];
      B->size[0] = 6;
      B->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(B, left_arm_ctrl_obs_B.f);
      left_arm_ctrl_obs_B.n_i = Fi->size[0] * Fi->size[1] - 1;
      for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
           left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.f++) {
        B->data[left_arm_ctrl_obs_B.f] = Fi->data[left_arm_ctrl_obs_B.f];
      }

      left_arm_ctrl_obs_B.n_i = Fi->size[1];
      left_arm_ctrl_obs_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = left_arm_ctrl_obs_B.n_i;
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.f);
      for (left_arm_ctrl_obs_B.b_j_e = 0; left_arm_ctrl_obs_B.b_j_e <
           left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.b_j_e++) {
        left_arm_ctrl_obs_B.pid_tmp_g = left_arm_ctrl_obs_B.b_j_e * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
             left_arm_ctrl_obs_B.b_i_n++) {
          Fi->data[(left_arm_ctrl_obs_B.pid_tmp_g + left_arm_ctrl_obs_B.b_i_n) +
            1] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
             left_arm_ctrl_obs_B.b_i_n++) {
          left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.b_i_n * 6 - 1;
          left_arm_ctrl_obs_B.temp_p = B->data[(left_arm_ctrl_obs_B.pid_tmp_g +
            left_arm_ctrl_obs_B.b_i_n) + 1];
          for (left_arm_ctrl_obs_B.c_i_m = 0; left_arm_ctrl_obs_B.c_i_m < 6;
               left_arm_ctrl_obs_B.c_i_m++) {
            left_arm_ctrl_obs_B.i_j = left_arm_ctrl_obs_B.c_i_m + 1;
            left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.pid_tmp_g +
              left_arm_ctrl_obs_B.i_j;
            Fi->data[left_arm_ctrl_obs_B.f] +=
              left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.aoffset_m +
              left_arm_ctrl_obs_B.i_j] * left_arm_ctrl_obs_B.temp_p;
          }
        }
      }

      while (left_arm_ctrl_obs_B.pid_p > 0.0) {
        left_arm_ctrl_obs_B.pid_tmp_g = static_cast<int32_T>
          (left_arm_ctrl_obs_B.pid_p);
        left_arm_ctrl_obs_B.q_size_tmp_n = left_arm_ctrl_obs_B.pid_tmp_g - 1;
        obj = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp_n];
        left_arm_ctrl_obs_B.f = Sj->size[0] * Sj->size[1];
        Sj->size[0] = 6;
        Sj->size[1] = obj->JointInternal.MotionSubspace->size[1];
        left_a_emxEnsureCapacity_real_T(Sj, left_arm_ctrl_obs_B.f);
        left_arm_ctrl_obs_B.n_i = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
             left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.f++) {
          Sj->data[left_arm_ctrl_obs_B.f] = obj->
            JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.f];
        }

        left_arm_ctrl_obs_B.b_idx_0_j = robot->
          VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_g - 1];
        left_arm_ctrl_obs_B.b_idx_1_e = robot->
          VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp_g + 9];
        if (left_arm_ctrl_obs_B.b_idx_0_j <= left_arm_ctrl_obs_B.b_idx_1_e) {
          left_arm_ctrl_obs_B.f = a->size[0] * a->size[1];
          a->size[0] = Sj->size[1];
          a->size[1] = 6;
          left_a_emxEnsureCapacity_real_T(a, left_arm_ctrl_obs_B.f);
          for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
               left_arm_ctrl_obs_B.f++) {
            left_arm_ctrl_obs_B.n_i = Sj->size[1];
            for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n <
                 left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.b_i_n++) {
              a->data[left_arm_ctrl_obs_B.b_i_n + a->size[0] *
                left_arm_ctrl_obs_B.f] = Sj->data[6 * left_arm_ctrl_obs_B.b_i_n
                + left_arm_ctrl_obs_B.f];
            }
          }

          left_arm_ctrl_obs_B.m_m = a->size[0];
          left_arm_ctrl_obs_B.n_i = Fi->size[1] - 1;
          left_arm_ctrl_obs_B.f = Hji->size[0] * Hji->size[1];
          Hji->size[0] = a->size[0];
          Hji->size[1] = Fi->size[1];
          left_a_emxEnsureCapacity_real_T(Hji, left_arm_ctrl_obs_B.f);
          for (left_arm_ctrl_obs_B.b_j_e = 0; left_arm_ctrl_obs_B.b_j_e <=
               left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.b_j_e++) {
            left_arm_ctrl_obs_B.coffset_j = left_arm_ctrl_obs_B.b_j_e *
              left_arm_ctrl_obs_B.m_m - 1;
            left_arm_ctrl_obs_B.boffset_f = left_arm_ctrl_obs_B.b_j_e * 6 - 1;
            for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n <
                 left_arm_ctrl_obs_B.m_m; left_arm_ctrl_obs_B.b_i_n++) {
              Hji->data[(left_arm_ctrl_obs_B.coffset_j +
                         left_arm_ctrl_obs_B.b_i_n) + 1] = 0.0;
            }

            for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
                 left_arm_ctrl_obs_B.b_i_n++) {
              left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.b_i_n *
                left_arm_ctrl_obs_B.m_m - 1;
              left_arm_ctrl_obs_B.temp_p = Fi->data
                [(left_arm_ctrl_obs_B.boffset_f + left_arm_ctrl_obs_B.b_i_n) + 1];
              for (left_arm_ctrl_obs_B.c_i_m = 0; left_arm_ctrl_obs_B.c_i_m <
                   left_arm_ctrl_obs_B.m_m; left_arm_ctrl_obs_B.c_i_m++) {
                left_arm_ctrl_obs_B.i_j = left_arm_ctrl_obs_B.c_i_m + 1;
                left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.coffset_j +
                  left_arm_ctrl_obs_B.i_j;
                Hji->data[left_arm_ctrl_obs_B.f] += a->
                  data[left_arm_ctrl_obs_B.aoffset_m + left_arm_ctrl_obs_B.i_j] *
                  left_arm_ctrl_obs_B.temp_p;
              }
            }
          }

          if (left_arm_ctrl_obs_B.b_idx_0_j > left_arm_ctrl_obs_B.b_idx_1_e) {
            left_arm_ctrl_obs_B.pid_tmp_g = 0;
          } else {
            left_arm_ctrl_obs_B.pid_tmp_g = static_cast<int32_T>
              (left_arm_ctrl_obs_B.b_idx_0_j) - 1;
          }

          if (left_arm_ctrl_obs_B.vNum_p > left_arm_ctrl_obs_B.p_idx_1_a) {
            left_arm_ctrl_obs_B.X_tmp_d = 0;
          } else {
            left_arm_ctrl_obs_B.X_tmp_d = static_cast<int32_T>
              (left_arm_ctrl_obs_B.vNum_p) - 1;
          }

          left_arm_ctrl_obs_B.n_i = Hji->size[1];
          for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <
               left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.f++) {
            left_arm_ctrl_obs_B.b_j_e = Hji->size[0];
            for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n <
                 left_arm_ctrl_obs_B.b_j_e; left_arm_ctrl_obs_B.b_i_n++) {
              H->data[(left_arm_ctrl_obs_B.pid_tmp_g + left_arm_ctrl_obs_B.b_i_n)
                + H->size[0] * (left_arm_ctrl_obs_B.X_tmp_d +
                                left_arm_ctrl_obs_B.f)] = Hji->data[Hji->size[0]
                * left_arm_ctrl_obs_B.f + left_arm_ctrl_obs_B.b_i_n];
            }
          }

          if (left_arm_ctrl_obs_B.vNum_p > left_arm_ctrl_obs_B.p_idx_1_a) {
            left_arm_ctrl_obs_B.pid_tmp_g = 0;
          } else {
            left_arm_ctrl_obs_B.pid_tmp_g = static_cast<int32_T>
              (left_arm_ctrl_obs_B.vNum_p) - 1;
          }

          if (left_arm_ctrl_obs_B.b_idx_0_j > left_arm_ctrl_obs_B.b_idx_1_e) {
            left_arm_ctrl_obs_B.X_tmp_d = 0;
          } else {
            left_arm_ctrl_obs_B.X_tmp_d = static_cast<int32_T>
              (left_arm_ctrl_obs_B.b_idx_0_j) - 1;
          }

          left_arm_ctrl_obs_B.n_i = Hji->size[0];
          for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <
               left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.f++) {
            left_arm_ctrl_obs_B.b_j_e = Hji->size[1];
            for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n <
                 left_arm_ctrl_obs_B.b_j_e; left_arm_ctrl_obs_B.b_i_n++) {
              H->data[(left_arm_ctrl_obs_B.pid_tmp_g + left_arm_ctrl_obs_B.b_i_n)
                + H->size[0] * (left_arm_ctrl_obs_B.X_tmp_d +
                                left_arm_ctrl_obs_B.f)] = Hji->data[Hji->size[0]
                * left_arm_ctrl_obs_B.b_i_n + left_arm_ctrl_obs_B.f];
            }
          }
        }

        for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f < 6;
             left_arm_ctrl_obs_B.f++) {
          for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
               left_arm_ctrl_obs_B.b_i_n++) {
            left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.b_i_n + 6 *
              left_arm_ctrl_obs_B.f] = X->data[left_arm_ctrl_obs_B.q_size_tmp_n]
              .f1[6 * left_arm_ctrl_obs_B.b_i_n + left_arm_ctrl_obs_B.f];
          }
        }

        left_arm_ctrl_obs_B.f = B->size[0] * B->size[1];
        B->size[0] = 6;
        B->size[1] = Fi->size[1];
        left_a_emxEnsureCapacity_real_T(B, left_arm_ctrl_obs_B.f);
        left_arm_ctrl_obs_B.n_i = Fi->size[0] * Fi->size[1] - 1;
        for (left_arm_ctrl_obs_B.f = 0; left_arm_ctrl_obs_B.f <=
             left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.f++) {
          B->data[left_arm_ctrl_obs_B.f] = Fi->data[left_arm_ctrl_obs_B.f];
        }

        left_arm_ctrl_obs_B.n_i = Fi->size[1];
        left_arm_ctrl_obs_B.f = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = left_arm_ctrl_obs_B.n_i;
        left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.f);
        for (left_arm_ctrl_obs_B.b_j_e = 0; left_arm_ctrl_obs_B.b_j_e <
             left_arm_ctrl_obs_B.n_i; left_arm_ctrl_obs_B.b_j_e++) {
          left_arm_ctrl_obs_B.pid_tmp_g = left_arm_ctrl_obs_B.b_j_e * 6 - 1;
          for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
               left_arm_ctrl_obs_B.b_i_n++) {
            Fi->data[(left_arm_ctrl_obs_B.pid_tmp_g + left_arm_ctrl_obs_B.b_i_n)
              + 1] = 0.0;
          }

          for (left_arm_ctrl_obs_B.b_i_n = 0; left_arm_ctrl_obs_B.b_i_n < 6;
               left_arm_ctrl_obs_B.b_i_n++) {
            left_arm_ctrl_obs_B.aoffset_m = left_arm_ctrl_obs_B.b_i_n * 6 - 1;
            left_arm_ctrl_obs_B.temp_p = B->data[(left_arm_ctrl_obs_B.pid_tmp_g
              + left_arm_ctrl_obs_B.b_i_n) + 1];
            for (left_arm_ctrl_obs_B.c_i_m = 0; left_arm_ctrl_obs_B.c_i_m < 6;
                 left_arm_ctrl_obs_B.c_i_m++) {
              left_arm_ctrl_obs_B.i_j = left_arm_ctrl_obs_B.c_i_m + 1;
              left_arm_ctrl_obs_B.f = left_arm_ctrl_obs_B.pid_tmp_g +
                left_arm_ctrl_obs_B.i_j;
              Fi->data[left_arm_ctrl_obs_B.f] +=
                left_arm_ctrl_obs_B.X_k[left_arm_ctrl_obs_B.aoffset_m +
                left_arm_ctrl_obs_B.i_j] * left_arm_ctrl_obs_B.temp_p;
            }
          }
        }

        left_arm_ctrl_obs_B.pid_p = robot->
          Bodies[left_arm_ctrl_obs_B.q_size_tmp_n]->ParentIndex;
      }
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&B);
  left_arm_ctrl_ob_emxFree_real_T(&a);
  left_arm_ctrl_ob_emxFree_real_T(&Hji);
  left_arm_ctrl_ob_emxFree_real_T(&Sj);
  left_arm_ctrl_ob_emxFree_real_T(&Fi);
  left_arm_ctrl_ob_emxFree_real_T(&Si);
  left_arm_ct_emxFree_e_cell_wrap(&X);
  left_arm_ct_emxFree_e_cell_wrap(&Ic);
}

static void left_arm_ctrl_SystemCore_step_e(boolean_T *varargout_1, real32_T
  varargout_2_Data[14], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_ctrl_obs_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  int32_T i;
  *varargout_1 = Sub_left_arm_ctrl_obs_318.getLatestMessage
    (&left_arm_ctrl_obs_B.b_varargout_2_m);
  for (i = 0; i < 14; i++) {
    varargout_2_Data[i] = left_arm_ctrl_obs_B.b_varargout_2_m.Data[i];
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
  for (left_arm_ctrl_obs_B.i_c = 0; left_arm_ctrl_obs_B.i_c < numDimensions;
       left_arm_ctrl_obs_B.i_c++) {
    emxArray->size[left_arm_ctrl_obs_B.i_c] = 0;
  }
}

static void left_a_emxEnsureCapacity_char_T(emxArray_char_T_left_arm_ctrl_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_ctrl_obs_B.newNumel_c = 1;
  for (left_arm_ctrl_obs_B.i_f = 0; left_arm_ctrl_obs_B.i_f <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_f++) {
    left_arm_ctrl_obs_B.newNumel_c *= emxArray->size[left_arm_ctrl_obs_B.i_f];
  }

  if (left_arm_ctrl_obs_B.newNumel_c > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_f = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_f < 16) {
      left_arm_ctrl_obs_B.i_f = 16;
    }

    while (left_arm_ctrl_obs_B.i_f < left_arm_ctrl_obs_B.newNumel_c) {
      if (left_arm_ctrl_obs_B.i_f > 1073741823) {
        left_arm_ctrl_obs_B.i_f = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_f <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_f), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_f;
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

static boolean_T left_arm_ctrl_obs_strcmp(const emxArray_char_T_left_arm_ctrl_T *
  a)
{
  boolean_T b_bool;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  for (left_arm_ctrl_obs_B.b_kstr_oy = 0; left_arm_ctrl_obs_B.b_kstr_oy < 5;
       left_arm_ctrl_obs_B.b_kstr_oy++) {
    left_arm_ctrl_obs_B.b_ei[left_arm_ctrl_obs_B.b_kstr_oy] =
      tmp[left_arm_ctrl_obs_B.b_kstr_oy];
  }

  b_bool = false;
  if (a->size[1] == 5) {
    left_arm_ctrl_obs_B.b_kstr_oy = 1;
    do {
      exitg1 = 0;
      if (left_arm_ctrl_obs_B.b_kstr_oy - 1 < 5) {
        left_arm_ctrl_obs_B.kstr_n = left_arm_ctrl_obs_B.b_kstr_oy - 1;
        if (a->data[left_arm_ctrl_obs_B.kstr_n] !=
            left_arm_ctrl_obs_B.b_ei[left_arm_ctrl_obs_B.kstr_n]) {
          exitg1 = 1;
        } else {
          left_arm_ctrl_obs_B.b_kstr_oy++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

static void lef_GravityTorqueBlock_stepImpl(robotics_slmanip_internal_b_e_T *obj,
  const real32_T q[7], real32_T jointTorq[7])
{
  k_robotics_manip_internal_R_e_T *robot;
  emxArray_e_cell_wrap_left_arm_T *X;
  emxArray_e_cell_wrap_left_arm_T *Xtree;
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
  for (left_arm_ctrl_obs_B.i_p = 0; left_arm_ctrl_obs_B.i_p < 7;
       left_arm_ctrl_obs_B.i_p++) {
    left_arm_ctrl_obs_B.q[left_arm_ctrl_obs_B.i_p] = q[left_arm_ctrl_obs_B.i_p];
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

  for (left_arm_ctrl_obs_B.i_p = 0; left_arm_ctrl_obs_B.i_p < 7;
       left_arm_ctrl_obs_B.i_p++) {
    left_arm_ctrl_obs_B.tau[left_arm_ctrl_obs_B.i_p] = 0.0;
  }

  left_arm_ct_emxInit_e_cell_wrap(&X, 2);
  left_arm_ct_emxInit_e_cell_wrap(&Xtree, 2);
  left_arm_ctrl_obs_B.i_p = left_arm_ctrl_obs_B.unnamed_idx_1 - 1;
  left_arm_ctrl_obs_B.u = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  l_emxEnsureCapacity_e_cell_wrap(Xtree, left_arm_ctrl_obs_B.u);
  left_arm_ctrl_obs_B.u = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1;
  l_emxEnsureCapacity_e_cell_wrap(X, left_arm_ctrl_obs_B.u);
  if (0 <= left_arm_ctrl_obs_B.i_p) {
    left_arm_ctrl_obs_eye(left_arm_ctrl_obs_B.b_I);
  }

  for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k <=
       left_arm_ctrl_obs_B.i_p; left_arm_ctrl_obs_B.b_k++) {
    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 36;
         left_arm_ctrl_obs_B.u++) {
      Xtree->data[left_arm_ctrl_obs_B.b_k].f1[left_arm_ctrl_obs_B.u] =
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.u];
      X->data[left_arm_ctrl_obs_B.b_k].f1[left_arm_ctrl_obs_B.u] =
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
       left_arm_ctrl_obs_B.i_p; left_arm_ctrl_obs_B.unnamed_idx_1++) {
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
      left_arm_ctrl_obs_B.t_a = 1;
      left_arm_ctrl_obs_B.qddoti_data[0] = 0.0;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        vJ->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
          0.0;
      }
    } else {
      if (left_arm_ctrl_obs_B.a_idx_0 > left_arm_ctrl_obs_B.a_idx_1) {
        left_arm_ctrl_obs_B.b_k = 0;
        left_arm_ctrl_obs_B.j_d = 0;
      } else {
        left_arm_ctrl_obs_B.b_k = static_cast<int32_T>
          (left_arm_ctrl_obs_B.a_idx_0) - 1;
        left_arm_ctrl_obs_B.j_d = static_cast<int32_T>
          (left_arm_ctrl_obs_B.a_idx_1);
      }

      if (left_arm_ctrl_obs_B.b_idx_0 > left_arm_ctrl_obs_B.b_idx_1) {
        left_arm_ctrl_obs_B.m = 0;
        left_arm_ctrl_obs_B.inner = 0;
        left_arm_ctrl_obs_B.u = 0;
        left_arm_ctrl_obs_B.t_a = -1;
      } else {
        left_arm_ctrl_obs_B.m = static_cast<int32_T>(left_arm_ctrl_obs_B.b_idx_0)
          - 1;
        left_arm_ctrl_obs_B.inner = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_1);
        left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.m;
        left_arm_ctrl_obs_B.t_a = left_arm_ctrl_obs_B.inner - 1;
      }

      left_arm_ctrl_obs_B.u = left_arm_ctrl_obs_B.t_a - left_arm_ctrl_obs_B.u;
      left_arm_ctrl_obs_B.t_a = left_arm_ctrl_obs_B.u + 1;
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
        left_arm_ctrl_obs_B.b_a[left_arm_ctrl_obs_B.u] =
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
                left_arm_ctrl_obs_B.b_a[left_arm_ctrl_obs_B.aoffset]) {
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
          left_arm_ctrl_obs_B.b_l[left_arm_ctrl_obs_B.u] =
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
                  left_arm_ctrl_obs_B.b_l[left_arm_ctrl_obs_B.aoffset]) {
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
        left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.j_d -
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
        left_arm_ctrl_obs_B.tempR_tmp_i = left_arm_ctrl_obs_B.v[2] *
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
                              left_arm_ctrl_obs_B.tempR_tmp_i -
                              left_arm_ctrl_obs_B.sth,
                              left_arm_ctrl_obs_B.b_idx_1 -
                              left_arm_ctrl_obs_B.tempR_tmp,
                              left_arm_ctrl_obs_B.tempR_tmp_i +
                              left_arm_ctrl_obs_B.sth, left_arm_ctrl_obs_B.v[2] *
                              left_arm_ctrl_obs_B.v[2] * (1.0 -
          left_arm_ctrl_obs_B.a_idx_0) + left_arm_ctrl_obs_B.a_idx_0,
                              left_arm_ctrl_obs_B.tempR_l);
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
        le_rigidBodyJoint_get_JointAxis(&obj_0->JointInternal,
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
          left_arm_ctrl_obs_B.j_d = left_arm_ctrl_obs_B.u +
            left_arm_ctrl_obs_B.aoffset;
          left_arm_ctrl_obs_B.a_c[left_arm_ctrl_obs_B.j_d] = 0.0;
          left_arm_ctrl_obs_B.a_c[left_arm_ctrl_obs_B.j_d] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.a_c[left_arm_ctrl_obs_B.j_d] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 1] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u + 4];
          left_arm_ctrl_obs_B.a_c[left_arm_ctrl_obs_B.j_d] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 2] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u + 8];
          left_arm_ctrl_obs_B.a_c[left_arm_ctrl_obs_B.j_d] +=
            left_arm_ctrl_obs_B.TJ[left_arm_ctrl_obs_B.aoffset + 3] *
            left_arm_ctrl_obs_B.a[left_arm_ctrl_obs_B.u + 12];
        }

        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 4;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.j_d = left_arm_ctrl_obs_B.b_k << 2;
          left_arm_ctrl_obs_B.aoffset = left_arm_ctrl_obs_B.u +
            left_arm_ctrl_obs_B.j_d;
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] = 0.0;
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j_d] *
            left_arm_ctrl_obs_B.a_c[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j_d + 1] *
            left_arm_ctrl_obs_B.a_c[left_arm_ctrl_obs_B.u + 4];
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j_d + 2] *
            left_arm_ctrl_obs_B.a_c[left_arm_ctrl_obs_B.u + 8];
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset] +=
            left_arm_ctrl_obs_B.b[left_arm_ctrl_obs_B.j_d + 3] *
            left_arm_ctrl_obs_B.a_c[left_arm_ctrl_obs_B.u + 12];
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

      if ((S->size[1] == 1) || (left_arm_ctrl_obs_B.t_a == 1)) {
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
      left_arm_ctrl_obs_B.t_a = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 2;
      left_arm_ctrl_obs_B.tempR_l[3] = -vB->data[left_arm_ctrl_obs_B.t_a];
      left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 1;
      left_arm_ctrl_obs_B.tempR_l[6] = vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR_l[1] = vB->data[left_arm_ctrl_obs_B.t_a];
      left_arm_ctrl_obs_B.tempR_l[4] = 0.0;
      left_arm_ctrl_obs_B.tempR_l[7] = -vB->data[6 *
        left_arm_ctrl_obs_B.unnamed_idx_1];
      left_arm_ctrl_obs_B.tempR_l[2] = -vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR_l[5] = vB->data[6 *
        left_arm_ctrl_obs_B.unnamed_idx_1];
      left_arm_ctrl_obs_B.tempR_l[8] = 0.0;
      left_arm_ctrl_obs_B.tempR[3] = 0.0;
      left_arm_ctrl_obs_B.t_a = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 5;
      left_arm_ctrl_obs_B.tempR[9] = -vB->data[left_arm_ctrl_obs_B.t_a];
      left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 4;
      left_arm_ctrl_obs_B.tempR[15] = vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR[4] = vB->data[left_arm_ctrl_obs_B.t_a];
      left_arm_ctrl_obs_B.tempR[10] = 0.0;
      left_arm_ctrl_obs_B.t_a = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 3;
      left_arm_ctrl_obs_B.tempR[16] = -vB->data[left_arm_ctrl_obs_B.t_a];
      left_arm_ctrl_obs_B.tempR[5] = -vB->data[left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR[11] = vB->data[left_arm_ctrl_obs_B.t_a];
      left_arm_ctrl_obs_B.tempR[17] = 0.0;
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
          left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.t_a = 6 * (left_arm_ctrl_obs_B.u + 3);
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a] = 0.0;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a + 3] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
          left_arm_ctrl_obs_B.u + 1];
        left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 1] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a + 1] = 0.0;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a + 4] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
          left_arm_ctrl_obs_B.u + 2];
        left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 2] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a + 2] = 0.0;
        left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a + 5] =
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

        left_arm_ctrl_obs_B.X_i[left_arm_ctrl_obs_B.u] =
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
          left_arm_ctrl_obs_B.X_i[left_arm_ctrl_obs_B.u] +
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
          left_arm_ctrl_obs_B.t_a = left_arm_ctrl_obs_B.b_k << 2;
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_a] *
            left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_a + 1] *
            left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u + 3];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_a + 2] *
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
        left_arm_ctrl_obs_B.t_a = 6 * (left_arm_ctrl_obs_B.u + 3);
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.t_a + 3] =
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset];
        left_arm_ctrl_obs_B.b_I[6 * left_arm_ctrl_obs_B.u + 4] =
          left_arm_ctrl_obs_B.dv2[3 * left_arm_ctrl_obs_B.u + 1];
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.t_a + 4] =
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset + 1];
        left_arm_ctrl_obs_B.b_I[6 * left_arm_ctrl_obs_B.u + 5] =
          left_arm_ctrl_obs_B.dv2[3 * left_arm_ctrl_obs_B.u + 2];
        left_arm_ctrl_obs_B.b_I[left_arm_ctrl_obs_B.t_a + 5] =
          left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.aoffset + 2];
      }

      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.t_a = left_arm_ctrl_obs_B.u + 6 *
            left_arm_ctrl_obs_B.b_k;
          left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a] = 0.0;
          for (left_arm_ctrl_obs_B.m = 0; left_arm_ctrl_obs_B.m < 6;
               left_arm_ctrl_obs_B.m++) {
            left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a] += Xtree->data[
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

      if ((S->size[1] == 1) || (left_arm_ctrl_obs_B.t_a == 1)) {
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
          left_arm_ctrl_obs_B.t_a = left_arm_ctrl_obs_B.b_k << 2;
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_a] *
            left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_a + 1] *
            left_arm_ctrl_obs_B.R_g[left_arm_ctrl_obs_B.u + 3];
          left_arm_ctrl_obs_B.dv2[left_arm_ctrl_obs_B.m] +=
            left_arm_ctrl_obs_B.T[left_arm_ctrl_obs_B.t_a + 2] *
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
    left_arm_ctrl_obs_B.t_a = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 2;
    left_arm_ctrl_obs_B.tempR_l[3] = -vB->data[left_arm_ctrl_obs_B.t_a];
    left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 1;
    left_arm_ctrl_obs_B.tempR_l[6] = vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR_l[1] = vB->data[left_arm_ctrl_obs_B.t_a];
    left_arm_ctrl_obs_B.tempR_l[4] = 0.0;
    left_arm_ctrl_obs_B.tempR_l[7] = -vB->data[6 *
      left_arm_ctrl_obs_B.unnamed_idx_1];
    left_arm_ctrl_obs_B.tempR_l[2] = -vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR_l[5] = vB->data[6 *
      left_arm_ctrl_obs_B.unnamed_idx_1];
    left_arm_ctrl_obs_B.tempR_l[8] = 0.0;
    left_arm_ctrl_obs_B.tempR[18] = 0.0;
    left_arm_ctrl_obs_B.t_a = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 5;
    left_arm_ctrl_obs_B.tempR[24] = -vB->data[left_arm_ctrl_obs_B.t_a];
    left_arm_ctrl_obs_B.u = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 4;
    left_arm_ctrl_obs_B.tempR[30] = vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR[19] = vB->data[left_arm_ctrl_obs_B.t_a];
    left_arm_ctrl_obs_B.tempR[25] = 0.0;
    left_arm_ctrl_obs_B.t_a = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 + 3;
    left_arm_ctrl_obs_B.tempR[31] = -vB->data[left_arm_ctrl_obs_B.t_a];
    left_arm_ctrl_obs_B.tempR[20] = -vB->data[left_arm_ctrl_obs_B.u];
    left_arm_ctrl_obs_B.tempR[26] = vB->data[left_arm_ctrl_obs_B.t_a];
    left_arm_ctrl_obs_B.tempR[32] = 0.0;
    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 3;
         left_arm_ctrl_obs_B.u++) {
      left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
        left_arm_ctrl_obs_B.u];
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 3] = 0.0;
      left_arm_ctrl_obs_B.t_a = 6 * (left_arm_ctrl_obs_B.u + 3);
      left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a + 3] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
        left_arm_ctrl_obs_B.u + 1];
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 1] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 4] = 0.0;
      left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a + 4] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.a_idx_1 = left_arm_ctrl_obs_B.tempR_l[3 *
        left_arm_ctrl_obs_B.u + 2];
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 2] =
        left_arm_ctrl_obs_B.a_idx_1;
      left_arm_ctrl_obs_B.tempR[6 * left_arm_ctrl_obs_B.u + 5] = 0.0;
      left_arm_ctrl_obs_B.tempR[left_arm_ctrl_obs_B.t_a + 5] =
        left_arm_ctrl_obs_B.a_idx_1;
    }

    for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
         left_arm_ctrl_obs_B.u++) {
      left_arm_ctrl_obs_B.X_i[left_arm_ctrl_obs_B.u] = 0.0;
      left_arm_ctrl_obs_B.b_I_o[left_arm_ctrl_obs_B.u] = 0.0;
      for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
           left_arm_ctrl_obs_B.b_k++) {
        left_arm_ctrl_obs_B.a_idx_0 = left_arm_ctrl_obs_B.b_I[6 *
          left_arm_ctrl_obs_B.b_k + left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.t_a = 6 * left_arm_ctrl_obs_B.unnamed_idx_1 +
          left_arm_ctrl_obs_B.b_k;
        left_arm_ctrl_obs_B.a_idx_1 = vB->data[left_arm_ctrl_obs_B.t_a] *
          left_arm_ctrl_obs_B.a_idx_0 +
          left_arm_ctrl_obs_B.X_i[left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.a_idx_0 = aB->data[left_arm_ctrl_obs_B.t_a] *
          left_arm_ctrl_obs_B.a_idx_0 +
          left_arm_ctrl_obs_B.b_I_o[left_arm_ctrl_obs_B.u];
        left_arm_ctrl_obs_B.X_i[left_arm_ctrl_obs_B.u] =
          left_arm_ctrl_obs_B.a_idx_1;
        left_arm_ctrl_obs_B.b_I_o[left_arm_ctrl_obs_B.u] =
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
          left_arm_ctrl_obs_B.X_i[left_arm_ctrl_obs_B.b_k];
      }

      f->data[left_arm_ctrl_obs_B.u + 6 * left_arm_ctrl_obs_B.unnamed_idx_1] =
        (left_arm_ctrl_obs_B.b_I_o[left_arm_ctrl_obs_B.u] +
         left_arm_ctrl_obs_B.y[left_arm_ctrl_obs_B.u]) -
        left_arm_ctrl_obs_B.a_idx_1;
    }
  }

  left_arm_ctrl_ob_emxFree_char_T(&switch_expression);
  left_arm_ctrl_ob_emxFree_real_T(&aB);
  left_arm_ctrl_ob_emxFree_real_T(&vB);
  left_arm_ctrl_ob_emxFree_real_T(&vJ);
  left_arm_ct_emxFree_e_cell_wrap(&Xtree);
  left_arm_ctrl_obs_B.i_p = static_cast<int32_T>(((-1.0 - left_arm_ctrl_obs_B.nb)
    + 1.0) / -1.0) - 1;
  left_arm_ctrl_ob_emxInit_real_T(&taui, 1);
  left_arm_ctrl_ob_emxInit_real_T(&a, 2);
  for (left_arm_ctrl_obs_B.t_a = 0; left_arm_ctrl_obs_B.t_a <=
       left_arm_ctrl_obs_B.i_p; left_arm_ctrl_obs_B.t_a++) {
    left_arm_ctrl_obs_B.a_idx_0 = left_arm_ctrl_obs_B.nb + -static_cast<real_T>
      (left_arm_ctrl_obs_B.t_a);
    left_arm_ctrl_obs_B.inner = static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0);
    left_arm_ctrl_obs_B.j_d = left_arm_ctrl_obs_B.inner - 1;
    obj_0 = robot->Bodies[left_arm_ctrl_obs_B.j_d];
    if (!left_arm_ctrl_obs_strcmp(obj_0->JointInternal.Type)) {
      obj_0 = robot->Bodies[left_arm_ctrl_obs_B.j_d];
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
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k <
             left_arm_ctrl_obs_B.aoffset; left_arm_ctrl_obs_B.b_k++) {
          a->data[left_arm_ctrl_obs_B.b_k + a->size[0] * left_arm_ctrl_obs_B.u] =
            S->data[6 * left_arm_ctrl_obs_B.b_k + left_arm_ctrl_obs_B.u];
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

    left_arm_ctrl_obs_B.a_idx_0 = robot->Bodies[left_arm_ctrl_obs_B.j_d]
      ->ParentIndex;
    if (left_arm_ctrl_obs_B.a_idx_0 > 0.0) {
      left_arm_ctrl_obs_B.m = static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0);
      for (left_arm_ctrl_obs_B.u = 0; left_arm_ctrl_obs_B.u < 6;
           left_arm_ctrl_obs_B.u++) {
        left_arm_ctrl_obs_B.a_idx_1 = 0.0;
        for (left_arm_ctrl_obs_B.b_k = 0; left_arm_ctrl_obs_B.b_k < 6;
             left_arm_ctrl_obs_B.b_k++) {
          left_arm_ctrl_obs_B.a_idx_1 += f->data[(left_arm_ctrl_obs_B.inner - 1)
            * 6 + left_arm_ctrl_obs_B.b_k] * X->data[left_arm_ctrl_obs_B.j_d]
            .f1[6 * left_arm_ctrl_obs_B.u + left_arm_ctrl_obs_B.b_k];
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
  left_arm_ct_emxFree_e_cell_wrap(&X);
  for (left_arm_ctrl_obs_B.i_p = 0; left_arm_ctrl_obs_B.i_p < 7;
       left_arm_ctrl_obs_B.i_p++) {
    jointTorq[left_arm_ctrl_obs_B.i_p] = static_cast<real32_T>
      (left_arm_ctrl_obs_B.tau[left_arm_ctrl_obs_B.i_p]);
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
  for (left_arm_ctrl_obs_B.i_cj = 0; left_arm_ctrl_obs_B.i_cj < numDimensions;
       left_arm_ctrl_obs_B.i_cj++) {
    emxArray->size[left_arm_ctrl_obs_B.i_cj] = 0;
  }
}

static void l_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_ctrl_obs_B.newNumel_p = 1;
  for (left_arm_ctrl_obs_B.i_p2 = 0; left_arm_ctrl_obs_B.i_p2 <
       emxArray->numDimensions; left_arm_ctrl_obs_B.i_p2++) {
    left_arm_ctrl_obs_B.newNumel_p *= emxArray->size[left_arm_ctrl_obs_B.i_p2];
  }

  if (left_arm_ctrl_obs_B.newNumel_p > emxArray->allocatedSize) {
    left_arm_ctrl_obs_B.i_p2 = emxArray->allocatedSize;
    if (left_arm_ctrl_obs_B.i_p2 < 16) {
      left_arm_ctrl_obs_B.i_p2 = 16;
    }

    while (left_arm_ctrl_obs_B.i_p2 < left_arm_ctrl_obs_B.newNumel_p) {
      if (left_arm_ctrl_obs_B.i_p2 > 1073741823) {
        left_arm_ctrl_obs_B.i_p2 = MAX_int32_T;
      } else {
        left_arm_ctrl_obs_B.i_p2 <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_ctrl_obs_B.i_p2), sizeof
                     (f_cell_wrap_left_arm_ctrl_obs_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_left_arm_ctrl_obs_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_left_arm_ctrl_obs_T *)newData;
    emxArray->allocatedSize = left_arm_ctrl_obs_B.i_p2;
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
  left_arm_ctrl_obs_B.nb_m = robot->NumBodies;
  left_arm_ctrl_obs_B.vNum_m = robot->VelocityNumber;
  left_arm_ctrl_obs_B.nm1d2 = H->size[0] * H->size[1];
  left_arm_ctrl_obs_B.b_i_c = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum_m);
  H->size[0] = left_arm_ctrl_obs_B.b_i_c;
  H->size[1] = left_arm_ctrl_obs_B.b_i_c;
  left_a_emxEnsureCapacity_real_T(H, left_arm_ctrl_obs_B.nm1d2);
  left_arm_ctrl_obs_B.n_f = left_arm_ctrl_obs_B.b_i_c *
    left_arm_ctrl_obs_B.b_i_c - 1;
  for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
       left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
    H->data[left_arm_ctrl_obs_B.nm1d2] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&lambda_, 2);
  left_arm_ctrl_obs_B.nm1d2 = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  left_arm_ctrl_obs_B.unnamed_idx_1_b = static_cast<int32_T>
    (left_arm_ctrl_obs_B.nb_m);
  lambda_->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_b;
  left_a_emxEnsureCapacity_real_T(lambda_, left_arm_ctrl_obs_B.nm1d2);
  left_arm_ctrl_obs_B.idx = left_arm_ctrl_obs_B.unnamed_idx_1_b - 1;
  for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
       left_arm_ctrl_obs_B.idx; left_arm_ctrl_obs_B.nm1d2++) {
    lambda_->data[left_arm_ctrl_obs_B.nm1d2] = 0.0;
  }

  left_arm_ctrl_obs_B.nm1d2 = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = left_arm_ctrl_obs_B.b_i_c;
  left_a_emxEnsureCapacity_real_T(lambda, left_arm_ctrl_obs_B.nm1d2);
  left_arm_ctrl_obs_B.n_f = left_arm_ctrl_obs_B.b_i_c - 1;
  for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
       left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
    lambda->data[left_arm_ctrl_obs_B.nm1d2] = 0.0;
  }

  left_arm_ct_emxInit_f_cell_wrap(&Ic, 2);
  left_arm_ct_emxInit_f_cell_wrap(&X, 2);
  left_arm_ctrl_obs_B.nm1d2 = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_b;
  l_emxEnsureCapacity_f_cell_wrap(Ic, left_arm_ctrl_obs_B.nm1d2);
  left_arm_ctrl_obs_B.nm1d2 = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_b;
  l_emxEnsureCapacity_f_cell_wrap(X, left_arm_ctrl_obs_B.nm1d2);
  for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c <=
       left_arm_ctrl_obs_B.idx; left_arm_ctrl_obs_B.b_i_c++) {
    for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 36;
         left_arm_ctrl_obs_B.nm1d2++) {
      Ic->data[left_arm_ctrl_obs_B.b_i_c].f1[left_arm_ctrl_obs_B.nm1d2] =
        robot->Bodies[left_arm_ctrl_obs_B.b_i_c]->
        SpatialInertia[left_arm_ctrl_obs_B.nm1d2];
    }

    left_arm_ctrl_obs_B.vNum_m = robot->PositionDoFMap[left_arm_ctrl_obs_B.b_i_c];
    left_arm_ctrl_obs_B.p_idx_1 = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.b_i_c + 10];
    if (left_arm_ctrl_obs_B.p_idx_1 < left_arm_ctrl_obs_B.vNum_m) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.b_i_c];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        left_arm_ctrl_obs_B.T_g);
    } else {
      if (left_arm_ctrl_obs_B.vNum_m > left_arm_ctrl_obs_B.p_idx_1) {
        left_arm_ctrl_obs_B.unnamed_idx_1_b = 0;
        left_arm_ctrl_obs_B.nm1d2 = -1;
      } else {
        left_arm_ctrl_obs_B.unnamed_idx_1_b = static_cast<int32_T>
          (left_arm_ctrl_obs_B.vNum_m) - 1;
        left_arm_ctrl_obs_B.nm1d2 = static_cast<int32_T>
          (left_arm_ctrl_obs_B.p_idx_1) - 1;
      }

      obj = robot->Bodies[left_arm_ctrl_obs_B.b_i_c];
      left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.nm1d2 -
        left_arm_ctrl_obs_B.unnamed_idx_1_b;
      left_arm_ctrl_obs_B.q_size_p = left_arm_ctrl_obs_B.q_size_tmp + 1;
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
           left_arm_ctrl_obs_B.q_size_tmp; left_arm_ctrl_obs_B.nm1d2++) {
        left_arm_ctrl_obs_B.q_data_o[left_arm_ctrl_obs_B.nm1d2] =
          q[left_arm_ctrl_obs_B.unnamed_idx_1_b + left_arm_ctrl_obs_B.nm1d2];
      }

      rigidBodyJoint_transformBodyT_e(&obj->JointInternal,
        left_arm_ctrl_obs_B.q_data_o, &left_arm_ctrl_obs_B.q_size_p,
        left_arm_ctrl_obs_B.T_g);
    }

    left_arm_ctrl_obs_tforminv(left_arm_ctrl_obs_B.T_g, left_arm_ctrl_obs_B.dv);
    left_arm_ct_tformToSpatialXform(left_arm_ctrl_obs_B.dv, X->
      data[left_arm_ctrl_obs_B.b_i_c].f1);
  }

  left_arm_ctrl_obs_B.idx = static_cast<int32_T>(((-1.0 -
    left_arm_ctrl_obs_B.nb_m) + 1.0) / -1.0) - 1;
  left_arm_ctrl_ob_emxInit_real_T(&Si, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Fi, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Sj, 2);
  left_arm_ctrl_ob_emxInit_real_T(&Hji, 2);
  left_arm_ctrl_ob_emxInit_char_T(&a, 2);
  left_arm_ctrl_ob_emxInit_real_T(&a_0, 2);
  left_arm_ctrl_ob_emxInit_real_T(&B, 2);
  for (left_arm_ctrl_obs_B.unnamed_idx_1_b = 0;
       left_arm_ctrl_obs_B.unnamed_idx_1_b <= left_arm_ctrl_obs_B.idx;
       left_arm_ctrl_obs_B.unnamed_idx_1_b++) {
    left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>(left_arm_ctrl_obs_B.nb_m
      + -static_cast<real_T>(left_arm_ctrl_obs_B.unnamed_idx_1_b));
    left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.pid_tmp - 1;
    left_arm_ctrl_obs_B.pid = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp]
      ->ParentIndex;
    left_arm_ctrl_obs_B.vNum_m = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp - 1];
    left_arm_ctrl_obs_B.p_idx_1 = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp + 9];
    if (left_arm_ctrl_obs_B.pid > 0.0) {
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
           left_arm_ctrl_obs_B.nm1d2++) {
        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
             left_arm_ctrl_obs_B.b_i_c++) {
          left_arm_ctrl_obs_B.n_f = left_arm_ctrl_obs_B.nm1d2 + 6 *
            left_arm_ctrl_obs_B.b_i_c;
          left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.n_f] = 0.0;
          for (left_arm_ctrl_obs_B.cb = 0; left_arm_ctrl_obs_B.cb < 6;
               left_arm_ctrl_obs_B.cb++) {
            left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.n_f] += X->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 *
              left_arm_ctrl_obs_B.nm1d2 + left_arm_ctrl_obs_B.cb] * Ic->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 *
              left_arm_ctrl_obs_B.b_i_c + left_arm_ctrl_obs_B.cb];
          }
        }
      }

      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
           left_arm_ctrl_obs_B.nm1d2++) {
        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
             left_arm_ctrl_obs_B.b_i_c++) {
          left_arm_ctrl_obs_B.b_idx_0_h = 0.0;
          for (left_arm_ctrl_obs_B.cb = 0; left_arm_ctrl_obs_B.cb < 6;
               left_arm_ctrl_obs_B.cb++) {
            left_arm_ctrl_obs_B.b_idx_0_h += left_arm_ctrl_obs_B.X[6 *
              left_arm_ctrl_obs_B.cb + left_arm_ctrl_obs_B.nm1d2] * X->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 *
              left_arm_ctrl_obs_B.b_i_c + left_arm_ctrl_obs_B.cb];
          }

          left_arm_ctrl_obs_B.cb = 6 * left_arm_ctrl_obs_B.b_i_c +
            left_arm_ctrl_obs_B.nm1d2;
          Ic->data[static_cast<int32_T>(left_arm_ctrl_obs_B.pid) - 1]
            .f1[left_arm_ctrl_obs_B.cb] += left_arm_ctrl_obs_B.b_idx_0_h;
        }
      }

      lambda_->data[left_arm_ctrl_obs_B.q_size_tmp] = left_arm_ctrl_obs_B.pid;
      if (lambda_->data[left_arm_ctrl_obs_B.q_size_tmp] > 0.0) {
        for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 5;
             left_arm_ctrl_obs_B.nm1d2++) {
          left_arm_ctrl_obs_B.b_h2[left_arm_ctrl_obs_B.nm1d2] =
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
        left_arm_ctrl_obs_B.n_f = obj->JointInternal.Type->size[0] *
          obj->JointInternal.Type->size[1] - 1;
        for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
             left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
          a->data[left_arm_ctrl_obs_B.nm1d2] = obj->JointInternal.Type->
            data[left_arm_ctrl_obs_B.nm1d2];
        }

        left_arm_ctrl_obs_B.b_bool_i = false;
        if (a->size[1] == 5) {
          left_arm_ctrl_obs_B.nm1d2 = 1;
          do {
            exitg2 = 0;
            if (left_arm_ctrl_obs_B.nm1d2 - 1 < 5) {
              left_arm_ctrl_obs_B.n_f = left_arm_ctrl_obs_B.nm1d2 - 1;
              if (a->data[left_arm_ctrl_obs_B.n_f] !=
                  left_arm_ctrl_obs_B.b_h2[left_arm_ctrl_obs_B.n_f]) {
                exitg2 = 1;
              } else {
                left_arm_ctrl_obs_B.nm1d2++;
              }
            } else {
              left_arm_ctrl_obs_B.b_bool_i = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (left_arm_ctrl_obs_B.b_bool_i) {
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
    left_arm_ctrl_obs_B.b_idx_1_cs = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.pid_tmp + 9];
    if (left_arm_ctrl_obs_B.b_idx_0_h <= left_arm_ctrl_obs_B.b_idx_1_cs) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp];
      left_arm_ctrl_obs_B.nm1d2 = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(Si, left_arm_ctrl_obs_B.nm1d2);
      left_arm_ctrl_obs_B.n_f = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
           left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
        Si->data[left_arm_ctrl_obs_B.nm1d2] = obj->
          JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.nm1d2];
      }

      left_arm_ctrl_obs_B.n_f = Si->size[1] - 1;
      left_arm_ctrl_obs_B.nm1d2 = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.nm1d2);
      for (left_arm_ctrl_obs_B.b_j = 0; left_arm_ctrl_obs_B.b_j <=
           left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.b_j++) {
        left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_j * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
             left_arm_ctrl_obs_B.b_i_c++) {
          Fi->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_c) + 1]
            = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
             left_arm_ctrl_obs_B.b_i_c++) {
          left_arm_ctrl_obs_B.aoffset_h = left_arm_ctrl_obs_B.b_i_c * 6 - 1;
          left_arm_ctrl_obs_B.temp = Si->data[(left_arm_ctrl_obs_B.pid_tmp +
            left_arm_ctrl_obs_B.b_i_c) + 1];
          for (left_arm_ctrl_obs_B.c_i_a = 0; left_arm_ctrl_obs_B.c_i_a < 6;
               left_arm_ctrl_obs_B.c_i_a++) {
            left_arm_ctrl_obs_B.i_m = left_arm_ctrl_obs_B.c_i_a + 1;
            left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.pid_tmp +
              left_arm_ctrl_obs_B.i_m;
            Fi->data[left_arm_ctrl_obs_B.nm1d2] += Ic->
              data[left_arm_ctrl_obs_B.q_size_tmp]
              .f1[left_arm_ctrl_obs_B.aoffset_h + left_arm_ctrl_obs_B.i_m] *
              left_arm_ctrl_obs_B.temp;
          }
        }
      }

      if (left_arm_ctrl_obs_B.vNum_m > left_arm_ctrl_obs_B.p_idx_1) {
        left_arm_ctrl_obs_B.pid_tmp = 0;
        left_arm_ctrl_obs_B.cb = 0;
      } else {
        left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.vNum_m) - 1;
        left_arm_ctrl_obs_B.cb = left_arm_ctrl_obs_B.pid_tmp;
      }

      left_arm_ctrl_obs_B.nm1d2 = a_0->size[0] * a_0->size[1];
      a_0->size[0] = Si->size[1];
      a_0->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a_0, left_arm_ctrl_obs_B.nm1d2);
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
           left_arm_ctrl_obs_B.nm1d2++) {
        left_arm_ctrl_obs_B.n_f = Si->size[1];
        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c <
             left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.b_i_c++) {
          a_0->data[left_arm_ctrl_obs_B.b_i_c + a_0->size[0] *
            left_arm_ctrl_obs_B.nm1d2] = Si->data[6 * left_arm_ctrl_obs_B.b_i_c
            + left_arm_ctrl_obs_B.nm1d2];
        }
      }

      left_arm_ctrl_obs_B.m_k = a_0->size[0];
      left_arm_ctrl_obs_B.n_f = Fi->size[1] - 1;
      left_arm_ctrl_obs_B.nm1d2 = Hji->size[0] * Hji->size[1];
      Hji->size[0] = a_0->size[0];
      Hji->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(Hji, left_arm_ctrl_obs_B.nm1d2);
      for (left_arm_ctrl_obs_B.b_j = 0; left_arm_ctrl_obs_B.b_j <=
           left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.b_j++) {
        left_arm_ctrl_obs_B.coffset = left_arm_ctrl_obs_B.b_j *
          left_arm_ctrl_obs_B.m_k - 1;
        left_arm_ctrl_obs_B.boffset = left_arm_ctrl_obs_B.b_j * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c <
             left_arm_ctrl_obs_B.m_k; left_arm_ctrl_obs_B.b_i_c++) {
          Hji->data[(left_arm_ctrl_obs_B.coffset + left_arm_ctrl_obs_B.b_i_c) +
            1] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
             left_arm_ctrl_obs_B.b_i_c++) {
          left_arm_ctrl_obs_B.aoffset_h = left_arm_ctrl_obs_B.b_i_c *
            left_arm_ctrl_obs_B.m_k - 1;
          left_arm_ctrl_obs_B.temp = Fi->data[(left_arm_ctrl_obs_B.boffset +
            left_arm_ctrl_obs_B.b_i_c) + 1];
          for (left_arm_ctrl_obs_B.c_i_a = 0; left_arm_ctrl_obs_B.c_i_a <
               left_arm_ctrl_obs_B.m_k; left_arm_ctrl_obs_B.c_i_a++) {
            left_arm_ctrl_obs_B.i_m = left_arm_ctrl_obs_B.c_i_a + 1;
            left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.coffset +
              left_arm_ctrl_obs_B.i_m;
            Hji->data[left_arm_ctrl_obs_B.nm1d2] += a_0->
              data[left_arm_ctrl_obs_B.aoffset_h + left_arm_ctrl_obs_B.i_m] *
              left_arm_ctrl_obs_B.temp;
          }
        }
      }

      left_arm_ctrl_obs_B.n_f = Hji->size[1];
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <
           left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
        left_arm_ctrl_obs_B.b_j = Hji->size[0];
        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c <
             left_arm_ctrl_obs_B.b_j; left_arm_ctrl_obs_B.b_i_c++) {
          H->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_c) +
            H->size[0] * (left_arm_ctrl_obs_B.cb + left_arm_ctrl_obs_B.nm1d2)] =
            Hji->data[Hji->size[0] * left_arm_ctrl_obs_B.nm1d2 +
            left_arm_ctrl_obs_B.b_i_c];
        }
      }

      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
           left_arm_ctrl_obs_B.nm1d2++) {
        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
             left_arm_ctrl_obs_B.b_i_c++) {
          left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.b_i_c + 6 *
            left_arm_ctrl_obs_B.nm1d2] = X->data[left_arm_ctrl_obs_B.q_size_tmp]
            .f1[6 * left_arm_ctrl_obs_B.b_i_c + left_arm_ctrl_obs_B.nm1d2];
        }
      }

      left_arm_ctrl_obs_B.nm1d2 = B->size[0] * B->size[1];
      B->size[0] = 6;
      B->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(B, left_arm_ctrl_obs_B.nm1d2);
      left_arm_ctrl_obs_B.n_f = Fi->size[0] * Fi->size[1] - 1;
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
           left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
        B->data[left_arm_ctrl_obs_B.nm1d2] = Fi->data[left_arm_ctrl_obs_B.nm1d2];
      }

      left_arm_ctrl_obs_B.n_f = Fi->size[1];
      left_arm_ctrl_obs_B.nm1d2 = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = left_arm_ctrl_obs_B.n_f;
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.nm1d2);
      for (left_arm_ctrl_obs_B.b_j = 0; left_arm_ctrl_obs_B.b_j <
           left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.b_j++) {
        left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_j * 6 - 1;
        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
             left_arm_ctrl_obs_B.b_i_c++) {
          Fi->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_c) + 1]
            = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
             left_arm_ctrl_obs_B.b_i_c++) {
          left_arm_ctrl_obs_B.aoffset_h = left_arm_ctrl_obs_B.b_i_c * 6 - 1;
          left_arm_ctrl_obs_B.temp = B->data[(left_arm_ctrl_obs_B.pid_tmp +
            left_arm_ctrl_obs_B.b_i_c) + 1];
          for (left_arm_ctrl_obs_B.c_i_a = 0; left_arm_ctrl_obs_B.c_i_a < 6;
               left_arm_ctrl_obs_B.c_i_a++) {
            left_arm_ctrl_obs_B.i_m = left_arm_ctrl_obs_B.c_i_a + 1;
            left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.pid_tmp +
              left_arm_ctrl_obs_B.i_m;
            Fi->data[left_arm_ctrl_obs_B.nm1d2] +=
              left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.aoffset_h +
              left_arm_ctrl_obs_B.i_m] * left_arm_ctrl_obs_B.temp;
          }
        }
      }

      while (left_arm_ctrl_obs_B.pid > 0.0) {
        left_arm_ctrl_obs_B.b_i_c = static_cast<int32_T>(left_arm_ctrl_obs_B.pid);
        left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.b_i_c - 1;
        obj = robot->Bodies[left_arm_ctrl_obs_B.q_size_tmp];
        left_arm_ctrl_obs_B.nm1d2 = Sj->size[0] * Sj->size[1];
        Sj->size[0] = 6;
        Sj->size[1] = obj->JointInternal.MotionSubspace->size[1];
        left_a_emxEnsureCapacity_real_T(Sj, left_arm_ctrl_obs_B.nm1d2);
        left_arm_ctrl_obs_B.n_f = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
             left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
          Sj->data[left_arm_ctrl_obs_B.nm1d2] =
            obj->JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.nm1d2];
        }

        left_arm_ctrl_obs_B.b_idx_0_h = robot->
          VelocityDoFMap[left_arm_ctrl_obs_B.b_i_c - 1];
        left_arm_ctrl_obs_B.b_idx_1_cs = robot->
          VelocityDoFMap[left_arm_ctrl_obs_B.b_i_c + 9];
        if (left_arm_ctrl_obs_B.b_idx_0_h <= left_arm_ctrl_obs_B.b_idx_1_cs) {
          left_arm_ctrl_obs_B.nm1d2 = a_0->size[0] * a_0->size[1];
          a_0->size[0] = Sj->size[1];
          a_0->size[1] = 6;
          left_a_emxEnsureCapacity_real_T(a_0, left_arm_ctrl_obs_B.nm1d2);
          for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
               left_arm_ctrl_obs_B.nm1d2++) {
            left_arm_ctrl_obs_B.n_f = Sj->size[1];
            for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c <
                 left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.b_i_c++) {
              a_0->data[left_arm_ctrl_obs_B.b_i_c + a_0->size[0] *
                left_arm_ctrl_obs_B.nm1d2] = Sj->data[6 *
                left_arm_ctrl_obs_B.b_i_c + left_arm_ctrl_obs_B.nm1d2];
            }
          }

          left_arm_ctrl_obs_B.m_k = a_0->size[0];
          left_arm_ctrl_obs_B.n_f = Fi->size[1] - 1;
          left_arm_ctrl_obs_B.nm1d2 = Hji->size[0] * Hji->size[1];
          Hji->size[0] = a_0->size[0];
          Hji->size[1] = Fi->size[1];
          left_a_emxEnsureCapacity_real_T(Hji, left_arm_ctrl_obs_B.nm1d2);
          for (left_arm_ctrl_obs_B.b_j = 0; left_arm_ctrl_obs_B.b_j <=
               left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.b_j++) {
            left_arm_ctrl_obs_B.coffset = left_arm_ctrl_obs_B.b_j *
              left_arm_ctrl_obs_B.m_k - 1;
            left_arm_ctrl_obs_B.boffset = left_arm_ctrl_obs_B.b_j * 6 - 1;
            for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c <
                 left_arm_ctrl_obs_B.m_k; left_arm_ctrl_obs_B.b_i_c++) {
              Hji->data[(left_arm_ctrl_obs_B.coffset + left_arm_ctrl_obs_B.b_i_c)
                + 1] = 0.0;
            }

            for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
                 left_arm_ctrl_obs_B.b_i_c++) {
              left_arm_ctrl_obs_B.aoffset_h = left_arm_ctrl_obs_B.b_i_c *
                left_arm_ctrl_obs_B.m_k - 1;
              left_arm_ctrl_obs_B.temp = Fi->data[(left_arm_ctrl_obs_B.boffset +
                left_arm_ctrl_obs_B.b_i_c) + 1];
              for (left_arm_ctrl_obs_B.c_i_a = 0; left_arm_ctrl_obs_B.c_i_a <
                   left_arm_ctrl_obs_B.m_k; left_arm_ctrl_obs_B.c_i_a++) {
                left_arm_ctrl_obs_B.i_m = left_arm_ctrl_obs_B.c_i_a + 1;
                left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.coffset +
                  left_arm_ctrl_obs_B.i_m;
                Hji->data[left_arm_ctrl_obs_B.nm1d2] += a_0->
                  data[left_arm_ctrl_obs_B.aoffset_h + left_arm_ctrl_obs_B.i_m] *
                  left_arm_ctrl_obs_B.temp;
              }
            }
          }

          if (left_arm_ctrl_obs_B.b_idx_0_h > left_arm_ctrl_obs_B.b_idx_1_cs) {
            left_arm_ctrl_obs_B.pid_tmp = 0;
          } else {
            left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>
              (left_arm_ctrl_obs_B.b_idx_0_h) - 1;
          }

          if (left_arm_ctrl_obs_B.vNum_m > left_arm_ctrl_obs_B.p_idx_1) {
            left_arm_ctrl_obs_B.cb = 0;
          } else {
            left_arm_ctrl_obs_B.cb = static_cast<int32_T>
              (left_arm_ctrl_obs_B.vNum_m) - 1;
          }

          left_arm_ctrl_obs_B.n_f = Hji->size[1];
          for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <
               left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
            left_arm_ctrl_obs_B.b_j = Hji->size[0];
            for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c <
                 left_arm_ctrl_obs_B.b_j; left_arm_ctrl_obs_B.b_i_c++) {
              H->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_c)
                + H->size[0] * (left_arm_ctrl_obs_B.cb +
                                left_arm_ctrl_obs_B.nm1d2)] = Hji->data
                [Hji->size[0] * left_arm_ctrl_obs_B.nm1d2 +
                left_arm_ctrl_obs_B.b_i_c];
            }
          }

          if (left_arm_ctrl_obs_B.vNum_m > left_arm_ctrl_obs_B.p_idx_1) {
            left_arm_ctrl_obs_B.pid_tmp = 0;
          } else {
            left_arm_ctrl_obs_B.pid_tmp = static_cast<int32_T>
              (left_arm_ctrl_obs_B.vNum_m) - 1;
          }

          if (left_arm_ctrl_obs_B.b_idx_0_h > left_arm_ctrl_obs_B.b_idx_1_cs) {
            left_arm_ctrl_obs_B.cb = 0;
          } else {
            left_arm_ctrl_obs_B.cb = static_cast<int32_T>
              (left_arm_ctrl_obs_B.b_idx_0_h) - 1;
          }

          left_arm_ctrl_obs_B.n_f = Hji->size[0];
          for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <
               left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
            left_arm_ctrl_obs_B.b_j = Hji->size[1];
            for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c <
                 left_arm_ctrl_obs_B.b_j; left_arm_ctrl_obs_B.b_i_c++) {
              H->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_c)
                + H->size[0] * (left_arm_ctrl_obs_B.cb +
                                left_arm_ctrl_obs_B.nm1d2)] = Hji->data
                [Hji->size[0] * left_arm_ctrl_obs_B.b_i_c +
                left_arm_ctrl_obs_B.nm1d2];
            }
          }
        }

        for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 < 6;
             left_arm_ctrl_obs_B.nm1d2++) {
          for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
               left_arm_ctrl_obs_B.b_i_c++) {
            left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.b_i_c + 6 *
              left_arm_ctrl_obs_B.nm1d2] = X->
              data[left_arm_ctrl_obs_B.q_size_tmp].f1[6 *
              left_arm_ctrl_obs_B.b_i_c + left_arm_ctrl_obs_B.nm1d2];
          }
        }

        left_arm_ctrl_obs_B.nm1d2 = B->size[0] * B->size[1];
        B->size[0] = 6;
        B->size[1] = Fi->size[1];
        left_a_emxEnsureCapacity_real_T(B, left_arm_ctrl_obs_B.nm1d2);
        left_arm_ctrl_obs_B.n_f = Fi->size[0] * Fi->size[1] - 1;
        for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
             left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
          B->data[left_arm_ctrl_obs_B.nm1d2] = Fi->
            data[left_arm_ctrl_obs_B.nm1d2];
        }

        left_arm_ctrl_obs_B.n_f = Fi->size[1];
        left_arm_ctrl_obs_B.nm1d2 = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = left_arm_ctrl_obs_B.n_f;
        left_a_emxEnsureCapacity_real_T(Fi, left_arm_ctrl_obs_B.nm1d2);
        for (left_arm_ctrl_obs_B.b_j = 0; left_arm_ctrl_obs_B.b_j <
             left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.b_j++) {
          left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_j * 6 - 1;
          for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
               left_arm_ctrl_obs_B.b_i_c++) {
            Fi->data[(left_arm_ctrl_obs_B.pid_tmp + left_arm_ctrl_obs_B.b_i_c) +
              1] = 0.0;
          }

          for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c < 6;
               left_arm_ctrl_obs_B.b_i_c++) {
            left_arm_ctrl_obs_B.aoffset_h = left_arm_ctrl_obs_B.b_i_c * 6 - 1;
            left_arm_ctrl_obs_B.temp = B->data[(left_arm_ctrl_obs_B.pid_tmp +
              left_arm_ctrl_obs_B.b_i_c) + 1];
            for (left_arm_ctrl_obs_B.c_i_a = 0; left_arm_ctrl_obs_B.c_i_a < 6;
                 left_arm_ctrl_obs_B.c_i_a++) {
              left_arm_ctrl_obs_B.i_m = left_arm_ctrl_obs_B.c_i_a + 1;
              left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.pid_tmp +
                left_arm_ctrl_obs_B.i_m;
              Fi->data[left_arm_ctrl_obs_B.nm1d2] +=
                left_arm_ctrl_obs_B.X[left_arm_ctrl_obs_B.aoffset_h +
                left_arm_ctrl_obs_B.i_m] * left_arm_ctrl_obs_B.temp;
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
  for (left_arm_ctrl_obs_B.unnamed_idx_1_b = 0;
       left_arm_ctrl_obs_B.unnamed_idx_1_b <= left_arm_ctrl_obs_B.idx;
       left_arm_ctrl_obs_B.unnamed_idx_1_b++) {
    left_arm_ctrl_obs_B.vNum_m = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.nonFixedIndices_data[left_arm_ctrl_obs_B.unnamed_idx_1_b]
      - 1];
    left_arm_ctrl_obs_B.p_idx_1 = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.nonFixedIndices_data[left_arm_ctrl_obs_B.unnamed_idx_1_b]
      + 9];
    if (rtIsNaN(left_arm_ctrl_obs_B.vNum_m) || rtIsNaN
        (left_arm_ctrl_obs_B.p_idx_1)) {
      left_arm_ctrl_obs_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_ctrl_obs_B.nm1d2);
      s->data[0] = (rtNaN);
    } else if (left_arm_ctrl_obs_B.p_idx_1 < left_arm_ctrl_obs_B.vNum_m) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(left_arm_ctrl_obs_B.vNum_m) || rtIsInf
                (left_arm_ctrl_obs_B.p_idx_1)) && (left_arm_ctrl_obs_B.vNum_m ==
                left_arm_ctrl_obs_B.p_idx_1)) {
      left_arm_ctrl_obs_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_ctrl_obs_B.nm1d2);
      s->data[0] = (rtNaN);
    } else if (floor(left_arm_ctrl_obs_B.vNum_m) == left_arm_ctrl_obs_B.vNum_m)
    {
      left_arm_ctrl_obs_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      left_arm_ctrl_obs_B.n_f = static_cast<int32_T>(floor
        (left_arm_ctrl_obs_B.p_idx_1 - left_arm_ctrl_obs_B.vNum_m));
      s->size[1] = left_arm_ctrl_obs_B.n_f + 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_ctrl_obs_B.nm1d2);
      for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <=
           left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
        s->data[left_arm_ctrl_obs_B.nm1d2] = left_arm_ctrl_obs_B.vNum_m +
          static_cast<real_T>(left_arm_ctrl_obs_B.nm1d2);
      }
    } else {
      left_arm_ctrl_obs_B.nb_m = floor((left_arm_ctrl_obs_B.p_idx_1 -
        left_arm_ctrl_obs_B.vNum_m) + 0.5);
      left_arm_ctrl_obs_B.pid = left_arm_ctrl_obs_B.vNum_m +
        left_arm_ctrl_obs_B.nb_m;
      left_arm_ctrl_obs_B.b_idx_0_h = left_arm_ctrl_obs_B.pid -
        left_arm_ctrl_obs_B.p_idx_1;
      left_arm_ctrl_obs_B.b_idx_1_cs = fabs(left_arm_ctrl_obs_B.vNum_m);
      left_arm_ctrl_obs_B.temp = fabs(left_arm_ctrl_obs_B.p_idx_1);
      if ((left_arm_ctrl_obs_B.b_idx_1_cs > left_arm_ctrl_obs_B.temp) || rtIsNaN
          (left_arm_ctrl_obs_B.temp)) {
        left_arm_ctrl_obs_B.temp = left_arm_ctrl_obs_B.b_idx_1_cs;
      }

      if (fabs(left_arm_ctrl_obs_B.b_idx_0_h) < 4.4408920985006262E-16 *
          left_arm_ctrl_obs_B.temp) {
        left_arm_ctrl_obs_B.nb_m++;
        left_arm_ctrl_obs_B.pid = left_arm_ctrl_obs_B.p_idx_1;
      } else if (left_arm_ctrl_obs_B.b_idx_0_h > 0.0) {
        left_arm_ctrl_obs_B.pid = (left_arm_ctrl_obs_B.nb_m - 1.0) +
          left_arm_ctrl_obs_B.vNum_m;
      } else {
        left_arm_ctrl_obs_B.nb_m++;
      }

      if (left_arm_ctrl_obs_B.nb_m >= 0.0) {
        left_arm_ctrl_obs_B.nm1d2 = static_cast<int32_T>
          (left_arm_ctrl_obs_B.nb_m);
      } else {
        left_arm_ctrl_obs_B.nm1d2 = 0;
      }

      left_arm_ctrl_obs_B.n_f = left_arm_ctrl_obs_B.nm1d2 - 1;
      left_arm_ctrl_obs_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = left_arm_ctrl_obs_B.n_f + 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_ctrl_obs_B.nm1d2);
      if (left_arm_ctrl_obs_B.n_f + 1 > 0) {
        s->data[0] = left_arm_ctrl_obs_B.vNum_m;
        if (left_arm_ctrl_obs_B.n_f + 1 > 1) {
          s->data[left_arm_ctrl_obs_B.n_f] = left_arm_ctrl_obs_B.pid;
          left_arm_ctrl_obs_B.nm1d2 = left_arm_ctrl_obs_B.n_f / 2;
          left_arm_ctrl_obs_B.q_size_tmp = left_arm_ctrl_obs_B.nm1d2 - 2;
          for (left_arm_ctrl_obs_B.b_i_c = 0; left_arm_ctrl_obs_B.b_i_c <=
               left_arm_ctrl_obs_B.q_size_tmp; left_arm_ctrl_obs_B.b_i_c++) {
            left_arm_ctrl_obs_B.pid_tmp = left_arm_ctrl_obs_B.b_i_c + 1;
            s->data[left_arm_ctrl_obs_B.pid_tmp] = left_arm_ctrl_obs_B.vNum_m +
              static_cast<real_T>(left_arm_ctrl_obs_B.pid_tmp);
            s->data[left_arm_ctrl_obs_B.n_f - left_arm_ctrl_obs_B.pid_tmp] =
              left_arm_ctrl_obs_B.pid - static_cast<real_T>
              (left_arm_ctrl_obs_B.pid_tmp);
          }

          if (left_arm_ctrl_obs_B.nm1d2 << 1 == left_arm_ctrl_obs_B.n_f) {
            s->data[left_arm_ctrl_obs_B.nm1d2] = (left_arm_ctrl_obs_B.vNum_m +
              left_arm_ctrl_obs_B.pid) / 2.0;
          } else {
            s->data[left_arm_ctrl_obs_B.nm1d2] = left_arm_ctrl_obs_B.vNum_m +
              static_cast<real_T>(left_arm_ctrl_obs_B.nm1d2);
            s->data[left_arm_ctrl_obs_B.nm1d2 + 1] = left_arm_ctrl_obs_B.pid -
              static_cast<real_T>(left_arm_ctrl_obs_B.nm1d2);
          }
        }
      }
    }

    if (left_arm_ctrl_obs_B.vNum_m > left_arm_ctrl_obs_B.p_idx_1) {
      left_arm_ctrl_obs_B.q_size_tmp = 0;
    } else {
      left_arm_ctrl_obs_B.q_size_tmp = static_cast<int32_T>
        (left_arm_ctrl_obs_B.vNum_m) - 1;
    }

    left_arm_ctrl_obs_B.n_f = s->size[1];
    for (left_arm_ctrl_obs_B.nm1d2 = 0; left_arm_ctrl_obs_B.nm1d2 <
         left_arm_ctrl_obs_B.n_f; left_arm_ctrl_obs_B.nm1d2++) {
      lambda->data[left_arm_ctrl_obs_B.q_size_tmp + left_arm_ctrl_obs_B.nm1d2] =
        s->data[left_arm_ctrl_obs_B.nm1d2] - 1.0;
    }

    if (lambda_->
        data[left_arm_ctrl_obs_B.nonFixedIndices_data[left_arm_ctrl_obs_B.unnamed_idx_1_b]
        - 1] == 0.0) {
      lambda->data[static_cast<int32_T>(left_arm_ctrl_obs_B.vNum_m) - 1] = 0.0;
    } else {
      left_arm_ctrl_obs_B.nm1d2 = static_cast<int32_T>(lambda_->
        data[left_arm_ctrl_obs_B.nonFixedIndices_data[left_arm_ctrl_obs_B.unnamed_idx_1_b]
        - 1]);
      left_arm_ctrl_obs_B.b_idx_1_cs = robot->
        VelocityDoFMap[left_arm_ctrl_obs_B.nm1d2 + 9];
      lambda->data[static_cast<int32_T>(left_arm_ctrl_obs_B.vNum_m) - 1] =
        left_arm_ctrl_obs_B.b_idx_1_cs;
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
  left_arm_ctrl_obs_B.a0_n[0] = 0.0;
  left_arm_ctrl_obs_B.a0_n[1] = 0.0;
  left_arm_ctrl_obs_B.a0_n[2] = 0.0;
  left_arm_ctrl_obs_B.a0_n[3] = -robot->Gravity[0];
  left_arm_ctrl_obs_B.a0_n[4] = -robot->Gravity[1];
  left_arm_ctrl_obs_B.a0_n[5] = -robot->Gravity[2];
  left_arm_ctrl_ob_emxInit_real_T(&vJ, 2);
  left_arm_ctrl_obs_B.nb_o = robot->NumBodies;
  left_arm_ctrl_obs_B.i_i = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  left_arm_ctrl_obs_B.unnamed_idx_1_o = static_cast<int32_T>
    (left_arm_ctrl_obs_B.nb_o);
  vJ->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_o;
  left_a_emxEnsureCapacity_real_T(vJ, left_arm_ctrl_obs_B.i_i);
  left_arm_ctrl_obs_B.loop_ub_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o - 1;
  for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i <=
       left_arm_ctrl_obs_B.loop_ub_tmp; left_arm_ctrl_obs_B.i_i++) {
    vJ->data[left_arm_ctrl_obs_B.i_i] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&vB, 2);
  left_arm_ctrl_obs_B.i_i = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_o;
  left_a_emxEnsureCapacity_real_T(vB, left_arm_ctrl_obs_B.i_i);
  for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i <=
       left_arm_ctrl_obs_B.loop_ub_tmp; left_arm_ctrl_obs_B.i_i++) {
    vB->data[left_arm_ctrl_obs_B.i_i] = 0.0;
  }

  left_arm_ctrl_ob_emxInit_real_T(&aB, 2);
  left_arm_ctrl_obs_B.i_i = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_o;
  left_a_emxEnsureCapacity_real_T(aB, left_arm_ctrl_obs_B.i_i);
  for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i <=
       left_arm_ctrl_obs_B.loop_ub_tmp; left_arm_ctrl_obs_B.i_i++) {
    aB->data[left_arm_ctrl_obs_B.i_i] = 0.0;
  }

  for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 7;
       left_arm_ctrl_obs_B.i_i++) {
    tau[left_arm_ctrl_obs_B.i_i] = 0.0;
  }

  left_arm_ct_emxInit_f_cell_wrap(&X, 2);
  left_arm_ct_emxInit_f_cell_wrap(&Xtree, 2);
  left_arm_ctrl_obs_B.loop_ub_tmp = left_arm_ctrl_obs_B.unnamed_idx_1_o - 1;
  left_arm_ctrl_obs_B.i_i = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_o;
  l_emxEnsureCapacity_f_cell_wrap(Xtree, left_arm_ctrl_obs_B.i_i);
  left_arm_ctrl_obs_B.i_i = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_o;
  l_emxEnsureCapacity_f_cell_wrap(X, left_arm_ctrl_obs_B.i_i);
  for (left_arm_ctrl_obs_B.b_k_n = 0; left_arm_ctrl_obs_B.b_k_n <=
       left_arm_ctrl_obs_B.loop_ub_tmp; left_arm_ctrl_obs_B.b_k_n++) {
    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 36;
         left_arm_ctrl_obs_B.i_i++) {
      Xtree->data[left_arm_ctrl_obs_B.b_k_n].f1[left_arm_ctrl_obs_B.i_i] = 0.0;
    }

    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
         left_arm_ctrl_obs_B.i_i++) {
      Xtree->data[left_arm_ctrl_obs_B.b_k_n].f1[left_arm_ctrl_obs_B.i_i + 6 *
        left_arm_ctrl_obs_B.i_i] = 1.0;
    }

    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 36;
         left_arm_ctrl_obs_B.i_i++) {
      X->data[left_arm_ctrl_obs_B.b_k_n].f1[left_arm_ctrl_obs_B.i_i] = 0.0;
    }

    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
         left_arm_ctrl_obs_B.i_i++) {
      X->data[left_arm_ctrl_obs_B.b_k_n].f1[left_arm_ctrl_obs_B.i_i + 6 *
        left_arm_ctrl_obs_B.i_i] = 1.0;
    }
  }

  left_arm_ctrl_ob_emxInit_real_T(&f, 2);
  left_arm_ctrl_obs_B.i_i = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = left_arm_ctrl_obs_B.unnamed_idx_1_o;
  left_a_emxEnsureCapacity_real_T(f, left_arm_ctrl_obs_B.i_i);
  left_arm_ctrl_ob_emxInit_real_T(&S, 2);
  left_arm_ctrl_ob_emxInit_real_T(&qddoti, 1);
  if (0 <= left_arm_ctrl_obs_B.loop_ub_tmp) {
    left_arm_ctrl_obs_B.dv3[0] = 0.0;
    left_arm_ctrl_obs_B.dv3[4] = 0.0;
    left_arm_ctrl_obs_B.dv3[8] = 0.0;
  }

  for (left_arm_ctrl_obs_B.unnamed_idx_1_o = 0;
       left_arm_ctrl_obs_B.unnamed_idx_1_o <= left_arm_ctrl_obs_B.loop_ub_tmp;
       left_arm_ctrl_obs_B.unnamed_idx_1_o++) {
    obj = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1_o];
    left_arm_ctrl_obs_B.i_i = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    left_a_emxEnsureCapacity_real_T(S, left_arm_ctrl_obs_B.i_i);
    left_arm_ctrl_obs_B.b_k_n = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i <=
         left_arm_ctrl_obs_B.b_k_n; left_arm_ctrl_obs_B.i_i++) {
      S->data[left_arm_ctrl_obs_B.i_i] = obj->JointInternal.MotionSubspace->
        data[left_arm_ctrl_obs_B.i_i];
    }

    left_arm_ctrl_obs_B.a_idx_0_m = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1_o];
    left_arm_ctrl_obs_B.a_idx_1_l = robot->
      PositionDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1_o + 10];
    left_arm_ctrl_obs_B.b_idx_0_m = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1_o];
    left_arm_ctrl_obs_B.b_idx_1_c = robot->
      VelocityDoFMap[left_arm_ctrl_obs_B.unnamed_idx_1_o + 10];
    if (left_arm_ctrl_obs_B.a_idx_1_l < left_arm_ctrl_obs_B.a_idx_0_m) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1_o];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        left_arm_ctrl_obs_B.T_p);
      left_arm_ctrl_obs_B.i_i = qddoti->size[0];
      qddoti->size[0] = 1;
      left_a_emxEnsureCapacity_real_T(qddoti, left_arm_ctrl_obs_B.i_i);
      qddoti->data[0] = 0.0;
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        vJ->data[left_arm_ctrl_obs_B.i_i + 6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_o] = 0.0;
      }
    } else {
      if (left_arm_ctrl_obs_B.a_idx_0_m > left_arm_ctrl_obs_B.a_idx_1_l) {
        left_arm_ctrl_obs_B.inner_p = 0;
        left_arm_ctrl_obs_B.m_l = -1;
      } else {
        left_arm_ctrl_obs_B.inner_p = static_cast<int32_T>
          (left_arm_ctrl_obs_B.a_idx_0_m) - 1;
        left_arm_ctrl_obs_B.m_l = static_cast<int32_T>
          (left_arm_ctrl_obs_B.a_idx_1_l) - 1;
      }

      if (left_arm_ctrl_obs_B.b_idx_0_m > left_arm_ctrl_obs_B.b_idx_1_c) {
        left_arm_ctrl_obs_B.p_tmp = 0;
        left_arm_ctrl_obs_B.o_tmp = 0;
        left_arm_ctrl_obs_B.aoffset_p = 0;
        left_arm_ctrl_obs_B.b_k_n = -1;
      } else {
        left_arm_ctrl_obs_B.p_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_0_m) - 1;
        left_arm_ctrl_obs_B.o_tmp = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_1_c);
        left_arm_ctrl_obs_B.aoffset_p = left_arm_ctrl_obs_B.p_tmp;
        left_arm_ctrl_obs_B.b_k_n = left_arm_ctrl_obs_B.o_tmp - 1;
      }

      left_arm_ctrl_obs_B.i_i = qddoti->size[0];
      left_arm_ctrl_obs_B.b_k_n -= left_arm_ctrl_obs_B.aoffset_p;
      qddoti->size[0] = left_arm_ctrl_obs_B.b_k_n + 1;
      left_a_emxEnsureCapacity_real_T(qddoti, left_arm_ctrl_obs_B.i_i);
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i <=
           left_arm_ctrl_obs_B.b_k_n; left_arm_ctrl_obs_B.i_i++) {
        qddoti->data[left_arm_ctrl_obs_B.i_i] = qddot->
          data[left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.i_i];
      }

      obj = robot->Bodies[left_arm_ctrl_obs_B.unnamed_idx_1_o];
      left_arm_ctrl_obs_B.m_l -= left_arm_ctrl_obs_B.inner_p;
      left_arm_ctrl_obs_B.q_size = left_arm_ctrl_obs_B.m_l + 1;
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i <=
           left_arm_ctrl_obs_B.m_l; left_arm_ctrl_obs_B.i_i++) {
        left_arm_ctrl_obs_B.q_data[left_arm_ctrl_obs_B.i_i] =
          q[left_arm_ctrl_obs_B.inner_p + left_arm_ctrl_obs_B.i_i];
      }

      rigidBodyJoint_transformBodyT_e(&obj->JointInternal,
        left_arm_ctrl_obs_B.q_data, &left_arm_ctrl_obs_B.q_size,
        left_arm_ctrl_obs_B.T_p);
      if ((S->size[1] == 1) || (left_arm_ctrl_obs_B.o_tmp -
           left_arm_ctrl_obs_B.p_tmp == 1)) {
        for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
             left_arm_ctrl_obs_B.i_i++) {
          left_arm_ctrl_obs_B.aoffset_p = left_arm_ctrl_obs_B.i_i + 6 *
            left_arm_ctrl_obs_B.unnamed_idx_1_o;
          vJ->data[left_arm_ctrl_obs_B.aoffset_p] = 0.0;
          left_arm_ctrl_obs_B.b_k_n = S->size[1];
          for (left_arm_ctrl_obs_B.inner_p = 0; left_arm_ctrl_obs_B.inner_p <
               left_arm_ctrl_obs_B.b_k_n; left_arm_ctrl_obs_B.inner_p++) {
            vJ->data[left_arm_ctrl_obs_B.aoffset_p] += S->data[6 *
              left_arm_ctrl_obs_B.inner_p + left_arm_ctrl_obs_B.i_i] *
              qdot[left_arm_ctrl_obs_B.p_tmp + left_arm_ctrl_obs_B.inner_p];
          }
        }
      } else {
        left_arm_ctrl_obs_B.inner_p = S->size[1] - 1;
        for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
             left_arm_ctrl_obs_B.i_i++) {
          vJ->data[left_arm_ctrl_obs_B.i_i + 6 *
            left_arm_ctrl_obs_B.unnamed_idx_1_o] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_k_n = 0; left_arm_ctrl_obs_B.b_k_n <=
             left_arm_ctrl_obs_B.inner_p; left_arm_ctrl_obs_B.b_k_n++) {
          left_arm_ctrl_obs_B.aoffset_p = left_arm_ctrl_obs_B.b_k_n * 6 - 1;
          for (left_arm_ctrl_obs_B.c_i_f = 0; left_arm_ctrl_obs_B.c_i_f < 6;
               left_arm_ctrl_obs_B.c_i_f++) {
            left_arm_ctrl_obs_B.i_i = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o +
              left_arm_ctrl_obs_B.c_i_f;
            vJ->data[left_arm_ctrl_obs_B.i_i] += S->data
              [(left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.c_i_f) + 1] *
              qdot[left_arm_ctrl_obs_B.p_tmp + left_arm_ctrl_obs_B.b_k_n];
          }
        }
      }
    }

    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 3;
         left_arm_ctrl_obs_B.i_i++) {
      left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.i_i] =
        left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.i_i];
      left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.i_i + 1] =
        left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.i_i + 4];
      left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.i_i + 2] =
        left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.i_i + 8];
    }

    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 9;
         left_arm_ctrl_obs_B.i_i++) {
      left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.i_i] =
        -left_arm_ctrl_obs_B.R_l[left_arm_ctrl_obs_B.i_i];
    }

    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 3;
         left_arm_ctrl_obs_B.i_i++) {
      left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.i_i << 2;
      left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp] =
        left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.i_i];
      left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp + 1] =
        left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.i_i + 1];
      left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp + 2] =
        left_arm_ctrl_obs_B.R_l[3 * left_arm_ctrl_obs_B.i_i + 2];
      left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.i_i + 12] =
        left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.i_i + 6] *
        left_arm_ctrl_obs_B.T_p[14] +
        (left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.i_i + 3] *
         left_arm_ctrl_obs_B.T_p[13] +
         left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.i_i] *
         left_arm_ctrl_obs_B.T_p[12]);
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
    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 3;
         left_arm_ctrl_obs_B.i_i++) {
      for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p < 3;
           left_arm_ctrl_obs_B.aoffset_p++) {
        left_arm_ctrl_obs_B.inner_p = left_arm_ctrl_obs_B.i_i + 3 *
          left_arm_ctrl_obs_B.aoffset_p;
        left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.inner_p] = 0.0;
        left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.aoffset_p << 2;
        left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.inner_p] +=
          left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp] *
          left_arm_ctrl_obs_B.dv3[left_arm_ctrl_obs_B.i_i];
        left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.inner_p] +=
          left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp + 1] *
          left_arm_ctrl_obs_B.dv3[left_arm_ctrl_obs_B.i_i + 3];
        left_arm_ctrl_obs_B.dv4[left_arm_ctrl_obs_B.inner_p] +=
          left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.p_tmp + 2] *
          left_arm_ctrl_obs_B.dv3[left_arm_ctrl_obs_B.i_i + 6];
        X->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
          .f1[left_arm_ctrl_obs_B.aoffset_p + 6 * left_arm_ctrl_obs_B.i_i] =
          left_arm_ctrl_obs_B.Tinv[(left_arm_ctrl_obs_B.i_i << 2) +
          left_arm_ctrl_obs_B.aoffset_p];
        X->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
          .f1[left_arm_ctrl_obs_B.aoffset_p + 6 * (left_arm_ctrl_obs_B.i_i + 3)]
          = 0.0;
      }
    }

    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 3;
         left_arm_ctrl_obs_B.i_i++) {
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_o].f1[6 *
        left_arm_ctrl_obs_B.i_i + 3] = left_arm_ctrl_obs_B.dv4[3 *
        left_arm_ctrl_obs_B.i_i];
      left_arm_ctrl_obs_B.aoffset_p = left_arm_ctrl_obs_B.i_i << 2;
      left_arm_ctrl_obs_B.inner_p = 6 * (left_arm_ctrl_obs_B.i_i + 3);
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
        .f1[left_arm_ctrl_obs_B.inner_p + 3] =
        left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.aoffset_p];
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_o].f1[6 *
        left_arm_ctrl_obs_B.i_i + 4] = left_arm_ctrl_obs_B.dv4[3 *
        left_arm_ctrl_obs_B.i_i + 1];
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
        .f1[left_arm_ctrl_obs_B.inner_p + 4] =
        left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.aoffset_p + 1];
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_o].f1[6 *
        left_arm_ctrl_obs_B.i_i + 5] = left_arm_ctrl_obs_B.dv4[3 *
        left_arm_ctrl_obs_B.i_i + 2];
      X->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
        .f1[left_arm_ctrl_obs_B.inner_p + 5] =
        left_arm_ctrl_obs_B.Tinv[left_arm_ctrl_obs_B.aoffset_p + 2];
    }

    left_arm_ctrl_obs_B.a_idx_0_m = robot->
      Bodies[left_arm_ctrl_obs_B.unnamed_idx_1_o]->ParentIndex;
    if (left_arm_ctrl_obs_B.a_idx_0_m > 0.0) {
      left_arm_ctrl_obs_B.m_l = static_cast<int32_T>
        (left_arm_ctrl_obs_B.a_idx_0_m);
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        left_arm_ctrl_obs_B.a_idx_1_l = 0.0;
        for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
             6; left_arm_ctrl_obs_B.aoffset_p++) {
          left_arm_ctrl_obs_B.a_idx_1_l += vB->data[(left_arm_ctrl_obs_B.m_l - 1)
            * 6 + left_arm_ctrl_obs_B.aoffset_p] * X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1_o].f1[6 *
            left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.i_i];
        }

        left_arm_ctrl_obs_B.vJ[left_arm_ctrl_obs_B.i_i] = vJ->data[6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_o + left_arm_ctrl_obs_B.i_i] +
          left_arm_ctrl_obs_B.a_idx_1_l;
      }

      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        vB->data[left_arm_ctrl_obs_B.i_i + 6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_o] =
          left_arm_ctrl_obs_B.vJ[left_arm_ctrl_obs_B.i_i];
      }

      if ((S->size[1] == 1) || (qddoti->size[0] == 1)) {
        left_arm_ctrl_obs_B.b_k_n = S->size[1];
        for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
             left_arm_ctrl_obs_B.i_i++) {
          left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i] = 0.0;
          for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
               left_arm_ctrl_obs_B.b_k_n; left_arm_ctrl_obs_B.aoffset_p++) {
            left_arm_ctrl_obs_B.a_idx_1_l = S->data[6 *
              left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.i_i] *
              qddoti->data[left_arm_ctrl_obs_B.aoffset_p] +
              left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i];
            left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i] =
              left_arm_ctrl_obs_B.a_idx_1_l;
          }
        }
      } else {
        left_arm_ctrl_obs_B.inner_p = S->size[1] - 1;
        for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
             left_arm_ctrl_obs_B.i_i++) {
          left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_k_n = 0; left_arm_ctrl_obs_B.b_k_n <=
             left_arm_ctrl_obs_B.inner_p; left_arm_ctrl_obs_B.b_k_n++) {
          left_arm_ctrl_obs_B.aoffset_p = left_arm_ctrl_obs_B.b_k_n * 6 - 1;
          for (left_arm_ctrl_obs_B.c_i_f = 0; left_arm_ctrl_obs_B.c_i_f < 6;
               left_arm_ctrl_obs_B.c_i_f++) {
            left_arm_ctrl_obs_B.a_idx_1_l = S->data
              [(left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.c_i_f) + 1] *
              qddoti->data[left_arm_ctrl_obs_B.b_k_n] +
              left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.c_i_f];
            left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.c_i_f] =
              left_arm_ctrl_obs_B.a_idx_1_l;
          }
        }
      }

      left_arm_ctrl_obs_B.R_l[0] = 0.0;
      left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o + 2;
      left_arm_ctrl_obs_B.R_l[3] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.i_i = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o + 1;
      left_arm_ctrl_obs_B.R_l[6] = vB->data[left_arm_ctrl_obs_B.i_i];
      left_arm_ctrl_obs_B.R_l[1] = vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.R_l[4] = 0.0;
      left_arm_ctrl_obs_B.R_l[7] = -vB->data[6 *
        left_arm_ctrl_obs_B.unnamed_idx_1_o];
      left_arm_ctrl_obs_B.R_l[2] = -vB->data[left_arm_ctrl_obs_B.i_i];
      left_arm_ctrl_obs_B.R_l[5] = vB->data[6 *
        left_arm_ctrl_obs_B.unnamed_idx_1_o];
      left_arm_ctrl_obs_B.R_l[8] = 0.0;
      left_arm_ctrl_obs_B.b_I_c[3] = 0.0;
      left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o + 5;
      left_arm_ctrl_obs_B.b_I_c[9] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.i_i = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o + 4;
      left_arm_ctrl_obs_B.b_I_c[15] = vB->data[left_arm_ctrl_obs_B.i_i];
      left_arm_ctrl_obs_B.b_I_c[4] = vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.b_I_c[10] = 0.0;
      left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o + 3;
      left_arm_ctrl_obs_B.b_I_c[16] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.b_I_c[5] = -vB->data[left_arm_ctrl_obs_B.i_i];
      left_arm_ctrl_obs_B.b_I_c[11] = vB->data[left_arm_ctrl_obs_B.p_tmp];
      left_arm_ctrl_obs_B.b_I_c[17] = 0.0;
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 3;
           left_arm_ctrl_obs_B.i_i++) {
        left_arm_ctrl_obs_B.a_idx_1_l = left_arm_ctrl_obs_B.R_l[3 *
          left_arm_ctrl_obs_B.i_i];
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_i] =
          left_arm_ctrl_obs_B.a_idx_1_l;
        left_arm_ctrl_obs_B.p_tmp = 6 * (left_arm_ctrl_obs_B.i_i + 3);
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp] = 0.0;
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp + 3] =
          left_arm_ctrl_obs_B.a_idx_1_l;
        left_arm_ctrl_obs_B.a_idx_1_l = left_arm_ctrl_obs_B.R_l[3 *
          left_arm_ctrl_obs_B.i_i + 1];
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_i + 1] =
          left_arm_ctrl_obs_B.a_idx_1_l;
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp + 1] = 0.0;
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp + 4] =
          left_arm_ctrl_obs_B.a_idx_1_l;
        left_arm_ctrl_obs_B.a_idx_1_l = left_arm_ctrl_obs_B.R_l[3 *
          left_arm_ctrl_obs_B.i_i + 2];
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_i + 2] =
          left_arm_ctrl_obs_B.a_idx_1_l;
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp + 2] = 0.0;
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.p_tmp + 5] =
          left_arm_ctrl_obs_B.a_idx_1_l;
      }

      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        left_arm_ctrl_obs_B.a_idx_1_l = 0.0;
        for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
             6; left_arm_ctrl_obs_B.aoffset_p++) {
          left_arm_ctrl_obs_B.a_idx_1_l += aB->data[(left_arm_ctrl_obs_B.m_l - 1)
            * 6 + left_arm_ctrl_obs_B.aoffset_p] * X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1_o].f1[6 *
            left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.i_i];
        }

        left_arm_ctrl_obs_B.vJ[left_arm_ctrl_obs_B.i_i] =
          left_arm_ctrl_obs_B.a_idx_1_l +
          left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i];
      }

      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i] = 0.0;
        for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
             6; left_arm_ctrl_obs_B.aoffset_p++) {
          left_arm_ctrl_obs_B.a_idx_1_l = left_arm_ctrl_obs_B.b_I_c[6 *
            left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.i_i] * vJ->data
            [6 * left_arm_ctrl_obs_B.unnamed_idx_1_o +
            left_arm_ctrl_obs_B.aoffset_p] +
            left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i];
          left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i] =
            left_arm_ctrl_obs_B.a_idx_1_l;
        }

        aB->data[left_arm_ctrl_obs_B.i_i + 6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_o] =
          left_arm_ctrl_obs_B.vJ[left_arm_ctrl_obs_B.i_i] +
          left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i];
      }

      left_arm_ctrl_obs_B.R_o[0] = 0.0;
      left_arm_ctrl_obs_B.R_o[3] = -left_arm_ctrl_obs_B.T_p[14];
      left_arm_ctrl_obs_B.R_o[6] = left_arm_ctrl_obs_B.T_p[13];
      left_arm_ctrl_obs_B.R_o[1] = left_arm_ctrl_obs_B.T_p[14];
      left_arm_ctrl_obs_B.R_o[4] = 0.0;
      left_arm_ctrl_obs_B.R_o[7] = -left_arm_ctrl_obs_B.T_p[12];
      left_arm_ctrl_obs_B.R_o[2] = -left_arm_ctrl_obs_B.T_p[13];
      left_arm_ctrl_obs_B.R_o[5] = left_arm_ctrl_obs_B.T_p[12];
      left_arm_ctrl_obs_B.R_o[8] = 0.0;
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 3;
           left_arm_ctrl_obs_B.i_i++) {
        for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
             3; left_arm_ctrl_obs_B.aoffset_p++) {
          left_arm_ctrl_obs_B.inner_p = left_arm_ctrl_obs_B.i_i + 3 *
            left_arm_ctrl_obs_B.aoffset_p;
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_p] = 0.0;
          left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.aoffset_p << 2;
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_p] +=
            left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.p_tmp] *
            left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.i_i];
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_p] +=
            left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.p_tmp + 1] *
            left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.i_i + 3];
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_p] +=
            left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.p_tmp + 2] *
            left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.i_i + 6];
          left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.aoffset_p + 6 *
            left_arm_ctrl_obs_B.i_i] = left_arm_ctrl_obs_B.T_p
            [(left_arm_ctrl_obs_B.i_i << 2) + left_arm_ctrl_obs_B.aoffset_p];
          left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.aoffset_p + 6 *
            (left_arm_ctrl_obs_B.i_i + 3)] = 0.0;
        }
      }

      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 3;
           left_arm_ctrl_obs_B.i_i++) {
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_i + 3] =
          left_arm_ctrl_obs_B.dv5[3 * left_arm_ctrl_obs_B.i_i];
        left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.i_i << 2;
        left_arm_ctrl_obs_B.inner_p = 6 * (left_arm_ctrl_obs_B.i_i + 3);
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.inner_p + 3] =
          left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.p_tmp];
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_i + 4] =
          left_arm_ctrl_obs_B.dv5[3 * left_arm_ctrl_obs_B.i_i + 1];
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.inner_p + 4] =
          left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.p_tmp + 1];
        left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.i_i + 5] =
          left_arm_ctrl_obs_B.dv5[3 * left_arm_ctrl_obs_B.i_i + 2];
        left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.inner_p + 5] =
          left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.p_tmp + 2];
      }

      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
             6; left_arm_ctrl_obs_B.aoffset_p++) {
          left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.i_i + 6 *
            left_arm_ctrl_obs_B.aoffset_p;
          left_arm_ctrl_obs_B.Xtree[left_arm_ctrl_obs_B.p_tmp] = 0.0;
          for (left_arm_ctrl_obs_B.inner_p = 0; left_arm_ctrl_obs_B.inner_p < 6;
               left_arm_ctrl_obs_B.inner_p++) {
            left_arm_ctrl_obs_B.Xtree[left_arm_ctrl_obs_B.p_tmp] += Xtree->data[
              static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0_m) - 1].f1[6 *
              left_arm_ctrl_obs_B.inner_p + left_arm_ctrl_obs_B.i_i] *
              left_arm_ctrl_obs_B.b_I_c[6 * left_arm_ctrl_obs_B.aoffset_p +
              left_arm_ctrl_obs_B.inner_p];
          }
        }
      }

      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 36;
           left_arm_ctrl_obs_B.i_i++) {
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
          .f1[left_arm_ctrl_obs_B.i_i] =
          left_arm_ctrl_obs_B.Xtree[left_arm_ctrl_obs_B.i_i];
      }
    } else {
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        left_arm_ctrl_obs_B.aoffset_p = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o
          + left_arm_ctrl_obs_B.i_i;
        vB->data[left_arm_ctrl_obs_B.aoffset_p] = vJ->
          data[left_arm_ctrl_obs_B.aoffset_p];
      }

      if ((S->size[1] == 1) || (qddoti->size[0] == 1)) {
        left_arm_ctrl_obs_B.b_k_n = S->size[1];
        for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
             left_arm_ctrl_obs_B.i_i++) {
          left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i] = 0.0;
          for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
               left_arm_ctrl_obs_B.b_k_n; left_arm_ctrl_obs_B.aoffset_p++) {
            left_arm_ctrl_obs_B.a_idx_1_l = S->data[6 *
              left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.i_i] *
              qddoti->data[left_arm_ctrl_obs_B.aoffset_p] +
              left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i];
            left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i] =
              left_arm_ctrl_obs_B.a_idx_1_l;
          }
        }
      } else {
        left_arm_ctrl_obs_B.inner_p = S->size[1] - 1;
        for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
             left_arm_ctrl_obs_B.i_i++) {
          left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i] = 0.0;
        }

        for (left_arm_ctrl_obs_B.b_k_n = 0; left_arm_ctrl_obs_B.b_k_n <=
             left_arm_ctrl_obs_B.inner_p; left_arm_ctrl_obs_B.b_k_n++) {
          left_arm_ctrl_obs_B.aoffset_p = left_arm_ctrl_obs_B.b_k_n * 6 - 1;
          for (left_arm_ctrl_obs_B.c_i_f = 0; left_arm_ctrl_obs_B.c_i_f < 6;
               left_arm_ctrl_obs_B.c_i_f++) {
            left_arm_ctrl_obs_B.a_idx_1_l = S->data
              [(left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.c_i_f) + 1] *
              qddoti->data[left_arm_ctrl_obs_B.b_k_n] +
              left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.c_i_f];
            left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.c_i_f] =
              left_arm_ctrl_obs_B.a_idx_1_l;
          }
        }
      }

      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        left_arm_ctrl_obs_B.a_idx_1_l = 0.0;
        for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
             6; left_arm_ctrl_obs_B.aoffset_p++) {
          left_arm_ctrl_obs_B.a_idx_1_l += X->
            data[left_arm_ctrl_obs_B.unnamed_idx_1_o].f1[6 *
            left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.i_i] *
            left_arm_ctrl_obs_B.a0_n[left_arm_ctrl_obs_B.aoffset_p];
        }

        aB->data[left_arm_ctrl_obs_B.i_i + 6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_o] = left_arm_ctrl_obs_B.a_idx_1_l +
          left_arm_ctrl_obs_B.y_m[left_arm_ctrl_obs_B.i_i];
      }

      left_arm_ctrl_obs_B.R_o[0] = 0.0;
      left_arm_ctrl_obs_B.R_o[3] = -left_arm_ctrl_obs_B.T_p[14];
      left_arm_ctrl_obs_B.R_o[6] = left_arm_ctrl_obs_B.T_p[13];
      left_arm_ctrl_obs_B.R_o[1] = left_arm_ctrl_obs_B.T_p[14];
      left_arm_ctrl_obs_B.R_o[4] = 0.0;
      left_arm_ctrl_obs_B.R_o[7] = -left_arm_ctrl_obs_B.T_p[12];
      left_arm_ctrl_obs_B.R_o[2] = -left_arm_ctrl_obs_B.T_p[13];
      left_arm_ctrl_obs_B.R_o[5] = left_arm_ctrl_obs_B.T_p[12];
      left_arm_ctrl_obs_B.R_o[8] = 0.0;
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 3;
           left_arm_ctrl_obs_B.i_i++) {
        for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
             3; left_arm_ctrl_obs_B.aoffset_p++) {
          left_arm_ctrl_obs_B.inner_p = left_arm_ctrl_obs_B.i_i + 3 *
            left_arm_ctrl_obs_B.aoffset_p;
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_p] = 0.0;
          left_arm_ctrl_obs_B.p_tmp = left_arm_ctrl_obs_B.aoffset_p << 2;
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_p] +=
            left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.p_tmp] *
            left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.i_i];
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_p] +=
            left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.p_tmp + 1] *
            left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.i_i + 3];
          left_arm_ctrl_obs_B.dv5[left_arm_ctrl_obs_B.inner_p] +=
            left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.p_tmp + 2] *
            left_arm_ctrl_obs_B.R_o[left_arm_ctrl_obs_B.i_i + 6];
          Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
            .f1[left_arm_ctrl_obs_B.aoffset_p + 6 * left_arm_ctrl_obs_B.i_i] =
            left_arm_ctrl_obs_B.T_p[(left_arm_ctrl_obs_B.i_i << 2) +
            left_arm_ctrl_obs_B.aoffset_p];
          Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
            .f1[left_arm_ctrl_obs_B.aoffset_p + 6 * (left_arm_ctrl_obs_B.i_i + 3)]
            = 0.0;
        }
      }

      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 3;
           left_arm_ctrl_obs_B.i_i++) {
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_o].f1[6 *
          left_arm_ctrl_obs_B.i_i + 3] = left_arm_ctrl_obs_B.dv5[3 *
          left_arm_ctrl_obs_B.i_i];
        left_arm_ctrl_obs_B.aoffset_p = left_arm_ctrl_obs_B.i_i << 2;
        left_arm_ctrl_obs_B.inner_p = 6 * (left_arm_ctrl_obs_B.i_i + 3);
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
          .f1[left_arm_ctrl_obs_B.inner_p + 3] =
          left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.aoffset_p];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_o].f1[6 *
          left_arm_ctrl_obs_B.i_i + 4] = left_arm_ctrl_obs_B.dv5[3 *
          left_arm_ctrl_obs_B.i_i + 1];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
          .f1[left_arm_ctrl_obs_B.inner_p + 4] =
          left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.aoffset_p + 1];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_o].f1[6 *
          left_arm_ctrl_obs_B.i_i + 5] = left_arm_ctrl_obs_B.dv5[3 *
          left_arm_ctrl_obs_B.i_i + 2];
        Xtree->data[left_arm_ctrl_obs_B.unnamed_idx_1_o]
          .f1[left_arm_ctrl_obs_B.inner_p + 5] =
          left_arm_ctrl_obs_B.T_p[left_arm_ctrl_obs_B.aoffset_p + 2];
      }
    }

    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 36;
         left_arm_ctrl_obs_B.i_i++) {
      left_arm_ctrl_obs_B.b_I_c[left_arm_ctrl_obs_B.i_i] = robot->
        Bodies[left_arm_ctrl_obs_B.unnamed_idx_1_o]->
        SpatialInertia[left_arm_ctrl_obs_B.i_i];
    }

    left_arm_ctrl_obs_B.R_l[0] = 0.0;
    left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o + 2;
    left_arm_ctrl_obs_B.R_l[3] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.i_i = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o + 1;
    left_arm_ctrl_obs_B.R_l[6] = vB->data[left_arm_ctrl_obs_B.i_i];
    left_arm_ctrl_obs_B.R_l[1] = vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.R_l[4] = 0.0;
    left_arm_ctrl_obs_B.R_l[7] = -vB->data[6 *
      left_arm_ctrl_obs_B.unnamed_idx_1_o];
    left_arm_ctrl_obs_B.R_l[2] = -vB->data[left_arm_ctrl_obs_B.i_i];
    left_arm_ctrl_obs_B.R_l[5] = vB->data[6 *
      left_arm_ctrl_obs_B.unnamed_idx_1_o];
    left_arm_ctrl_obs_B.R_l[8] = 0.0;
    left_arm_ctrl_obs_B.R[18] = 0.0;
    left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o + 5;
    left_arm_ctrl_obs_B.R[24] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.i_i = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o + 4;
    left_arm_ctrl_obs_B.R[30] = vB->data[left_arm_ctrl_obs_B.i_i];
    left_arm_ctrl_obs_B.R[19] = vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.R[25] = 0.0;
    left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o + 3;
    left_arm_ctrl_obs_B.R[31] = -vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.R[20] = -vB->data[left_arm_ctrl_obs_B.i_i];
    left_arm_ctrl_obs_B.R[26] = vB->data[left_arm_ctrl_obs_B.p_tmp];
    left_arm_ctrl_obs_B.R[32] = 0.0;
    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 3;
         left_arm_ctrl_obs_B.i_i++) {
      left_arm_ctrl_obs_B.a_idx_1_l = left_arm_ctrl_obs_B.R_l[3 *
        left_arm_ctrl_obs_B.i_i];
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_i] =
        left_arm_ctrl_obs_B.a_idx_1_l;
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_i + 3] = 0.0;
      left_arm_ctrl_obs_B.p_tmp = 6 * (left_arm_ctrl_obs_B.i_i + 3);
      left_arm_ctrl_obs_B.R[left_arm_ctrl_obs_B.p_tmp + 3] =
        left_arm_ctrl_obs_B.a_idx_1_l;
      left_arm_ctrl_obs_B.a_idx_1_l = left_arm_ctrl_obs_B.R_l[3 *
        left_arm_ctrl_obs_B.i_i + 1];
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_i + 1] =
        left_arm_ctrl_obs_B.a_idx_1_l;
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_i + 4] = 0.0;
      left_arm_ctrl_obs_B.R[left_arm_ctrl_obs_B.p_tmp + 4] =
        left_arm_ctrl_obs_B.a_idx_1_l;
      left_arm_ctrl_obs_B.a_idx_1_l = left_arm_ctrl_obs_B.R_l[3 *
        left_arm_ctrl_obs_B.i_i + 2];
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_i + 2] =
        left_arm_ctrl_obs_B.a_idx_1_l;
      left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.i_i + 5] = 0.0;
      left_arm_ctrl_obs_B.R[left_arm_ctrl_obs_B.p_tmp + 5] =
        left_arm_ctrl_obs_B.a_idx_1_l;
    }

    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
         left_arm_ctrl_obs_B.i_i++) {
      left_arm_ctrl_obs_B.b_I_cz[left_arm_ctrl_obs_B.i_i] = 0.0;
      left_arm_ctrl_obs_B.b_I_m[left_arm_ctrl_obs_B.i_i] = 0.0;
      for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p < 6;
           left_arm_ctrl_obs_B.aoffset_p++) {
        left_arm_ctrl_obs_B.a_idx_0_m = left_arm_ctrl_obs_B.b_I_c[6 *
          left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.i_i];
        left_arm_ctrl_obs_B.p_tmp = 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o +
          left_arm_ctrl_obs_B.aoffset_p;
        left_arm_ctrl_obs_B.a_idx_1_l = vB->data[left_arm_ctrl_obs_B.p_tmp] *
          left_arm_ctrl_obs_B.a_idx_0_m +
          left_arm_ctrl_obs_B.b_I_cz[left_arm_ctrl_obs_B.i_i];
        left_arm_ctrl_obs_B.a_idx_0_m = aB->data[left_arm_ctrl_obs_B.p_tmp] *
          left_arm_ctrl_obs_B.a_idx_0_m +
          left_arm_ctrl_obs_B.b_I_m[left_arm_ctrl_obs_B.i_i];
        left_arm_ctrl_obs_B.b_I_cz[left_arm_ctrl_obs_B.i_i] =
          left_arm_ctrl_obs_B.a_idx_1_l;
        left_arm_ctrl_obs_B.b_I_m[left_arm_ctrl_obs_B.i_i] =
          left_arm_ctrl_obs_B.a_idx_0_m;
      }
    }

    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
         left_arm_ctrl_obs_B.i_i++) {
      left_arm_ctrl_obs_B.R_m[left_arm_ctrl_obs_B.i_i] = 0.0;
      left_arm_ctrl_obs_B.a_idx_1_l = 0.0;
      for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p < 6;
           left_arm_ctrl_obs_B.aoffset_p++) {
        left_arm_ctrl_obs_B.a_idx_1_l += Xtree->
          data[left_arm_ctrl_obs_B.unnamed_idx_1_o].f1[6 *
          left_arm_ctrl_obs_B.i_i + left_arm_ctrl_obs_B.aoffset_p] * fext[6 *
          left_arm_ctrl_obs_B.unnamed_idx_1_o + left_arm_ctrl_obs_B.aoffset_p];
        left_arm_ctrl_obs_B.R_m[left_arm_ctrl_obs_B.i_i] +=
          left_arm_ctrl_obs_B.R[6 * left_arm_ctrl_obs_B.aoffset_p +
          left_arm_ctrl_obs_B.i_i] *
          left_arm_ctrl_obs_B.b_I_cz[left_arm_ctrl_obs_B.aoffset_p];
      }

      f->data[left_arm_ctrl_obs_B.i_i + 6 * left_arm_ctrl_obs_B.unnamed_idx_1_o]
        = (left_arm_ctrl_obs_B.b_I_m[left_arm_ctrl_obs_B.i_i] +
           left_arm_ctrl_obs_B.R_m[left_arm_ctrl_obs_B.i_i]) -
        left_arm_ctrl_obs_B.a_idx_1_l;
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&aB);
  left_arm_ctrl_ob_emxFree_real_T(&vB);
  left_arm_ctrl_ob_emxFree_real_T(&vJ);
  left_arm_ct_emxFree_f_cell_wrap(&Xtree);
  left_arm_ctrl_obs_B.loop_ub_tmp = static_cast<int32_T>(((-1.0 -
    left_arm_ctrl_obs_B.nb_o) + 1.0) / -1.0) - 1;
  left_arm_ctrl_ob_emxInit_char_T(&a, 2);
  left_arm_ctrl_ob_emxInit_real_T(&a_0, 2);
  if (0 <= left_arm_ctrl_obs_B.loop_ub_tmp) {
    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 5;
         left_arm_ctrl_obs_B.i_i++) {
      left_arm_ctrl_obs_B.b_e[left_arm_ctrl_obs_B.i_i] =
        tmp[left_arm_ctrl_obs_B.i_i];
    }
  }

  for (left_arm_ctrl_obs_B.p_tmp = 0; left_arm_ctrl_obs_B.p_tmp <=
       left_arm_ctrl_obs_B.loop_ub_tmp; left_arm_ctrl_obs_B.p_tmp++) {
    left_arm_ctrl_obs_B.a_idx_0_m = left_arm_ctrl_obs_B.nb_o +
      -static_cast<real_T>(left_arm_ctrl_obs_B.p_tmp);
    left_arm_ctrl_obs_B.inner_p = static_cast<int32_T>
      (left_arm_ctrl_obs_B.a_idx_0_m);
    left_arm_ctrl_obs_B.o_tmp = left_arm_ctrl_obs_B.inner_p - 1;
    obj = robot->Bodies[left_arm_ctrl_obs_B.o_tmp];
    left_arm_ctrl_obs_B.i_i = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    left_a_emxEnsureCapacity_char_T(a, left_arm_ctrl_obs_B.i_i);
    left_arm_ctrl_obs_B.b_k_n = obj->JointInternal.Type->size[0] *
      obj->JointInternal.Type->size[1] - 1;
    for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i <=
         left_arm_ctrl_obs_B.b_k_n; left_arm_ctrl_obs_B.i_i++) {
      a->data[left_arm_ctrl_obs_B.i_i] = obj->JointInternal.Type->
        data[left_arm_ctrl_obs_B.i_i];
    }

    left_arm_ctrl_obs_B.b_bool_c = false;
    if (a->size[1] == 5) {
      left_arm_ctrl_obs_B.i_i = 1;
      do {
        exitg1 = 0;
        if (left_arm_ctrl_obs_B.i_i - 1 < 5) {
          left_arm_ctrl_obs_B.unnamed_idx_1_o = left_arm_ctrl_obs_B.i_i - 1;
          if (a->data[left_arm_ctrl_obs_B.unnamed_idx_1_o] !=
              left_arm_ctrl_obs_B.b_e[left_arm_ctrl_obs_B.unnamed_idx_1_o]) {
            exitg1 = 1;
          } else {
            left_arm_ctrl_obs_B.i_i++;
          }
        } else {
          left_arm_ctrl_obs_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!left_arm_ctrl_obs_B.b_bool_c) {
      obj = robot->Bodies[left_arm_ctrl_obs_B.o_tmp];
      left_arm_ctrl_obs_B.i_i = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(S, left_arm_ctrl_obs_B.i_i);
      left_arm_ctrl_obs_B.b_k_n = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i <=
           left_arm_ctrl_obs_B.b_k_n; left_arm_ctrl_obs_B.i_i++) {
        S->data[left_arm_ctrl_obs_B.i_i] = obj->
          JointInternal.MotionSubspace->data[left_arm_ctrl_obs_B.i_i];
      }

      left_arm_ctrl_obs_B.i_i = a_0->size[0] * a_0->size[1];
      a_0->size[0] = S->size[1];
      a_0->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a_0, left_arm_ctrl_obs_B.i_i);
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        left_arm_ctrl_obs_B.b_k_n = S->size[1];
        for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
             left_arm_ctrl_obs_B.b_k_n; left_arm_ctrl_obs_B.aoffset_p++) {
          a_0->data[left_arm_ctrl_obs_B.aoffset_p + a_0->size[0] *
            left_arm_ctrl_obs_B.i_i] = S->data[6 * left_arm_ctrl_obs_B.aoffset_p
            + left_arm_ctrl_obs_B.i_i];
        }
      }

      left_arm_ctrl_obs_B.m_l = a_0->size[0] - 1;
      left_arm_ctrl_obs_B.i_i = qddoti->size[0];
      qddoti->size[0] = a_0->size[0];
      left_a_emxEnsureCapacity_real_T(qddoti, left_arm_ctrl_obs_B.i_i);
      for (left_arm_ctrl_obs_B.unnamed_idx_1_o = 0;
           left_arm_ctrl_obs_B.unnamed_idx_1_o <= left_arm_ctrl_obs_B.m_l;
           left_arm_ctrl_obs_B.unnamed_idx_1_o++) {
        qddoti->data[left_arm_ctrl_obs_B.unnamed_idx_1_o] = 0.0;
      }

      for (left_arm_ctrl_obs_B.b_k_n = 0; left_arm_ctrl_obs_B.b_k_n < 6;
           left_arm_ctrl_obs_B.b_k_n++) {
        left_arm_ctrl_obs_B.aoffset_p = (left_arm_ctrl_obs_B.m_l + 1) *
          left_arm_ctrl_obs_B.b_k_n - 1;
        for (left_arm_ctrl_obs_B.c_i_f = 0; left_arm_ctrl_obs_B.c_i_f <=
             left_arm_ctrl_obs_B.m_l; left_arm_ctrl_obs_B.c_i_f++) {
          qddoti->data[left_arm_ctrl_obs_B.c_i_f] += f->data
            [(static_cast<int32_T>(left_arm_ctrl_obs_B.a_idx_0_m) - 1) * 6 +
            left_arm_ctrl_obs_B.b_k_n] * a_0->data
            [(left_arm_ctrl_obs_B.aoffset_p + left_arm_ctrl_obs_B.c_i_f) + 1];
        }
      }

      left_arm_ctrl_obs_B.b_idx_0_m = robot->
        VelocityDoFMap[left_arm_ctrl_obs_B.inner_p - 1];
      left_arm_ctrl_obs_B.b_idx_1_c = robot->
        VelocityDoFMap[left_arm_ctrl_obs_B.inner_p + 9];
      if (left_arm_ctrl_obs_B.b_idx_0_m > left_arm_ctrl_obs_B.b_idx_1_c) {
        left_arm_ctrl_obs_B.m_l = 0;
        left_arm_ctrl_obs_B.i_i = 0;
      } else {
        left_arm_ctrl_obs_B.m_l = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_0_m) - 1;
        left_arm_ctrl_obs_B.i_i = static_cast<int32_T>
          (left_arm_ctrl_obs_B.b_idx_1_c);
      }

      left_arm_ctrl_obs_B.unnamed_idx_1_o = left_arm_ctrl_obs_B.i_i -
        left_arm_ctrl_obs_B.m_l;
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i <
           left_arm_ctrl_obs_B.unnamed_idx_1_o; left_arm_ctrl_obs_B.i_i++) {
        tau[left_arm_ctrl_obs_B.m_l + left_arm_ctrl_obs_B.i_i] = qddoti->
          data[left_arm_ctrl_obs_B.i_i];
      }
    }

    left_arm_ctrl_obs_B.a_idx_0_m = robot->Bodies[left_arm_ctrl_obs_B.o_tmp]
      ->ParentIndex;
    if (left_arm_ctrl_obs_B.a_idx_0_m > 0.0) {
      left_arm_ctrl_obs_B.m_l = static_cast<int32_T>
        (left_arm_ctrl_obs_B.a_idx_0_m);
      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        left_arm_ctrl_obs_B.a_idx_1_l = 0.0;
        for (left_arm_ctrl_obs_B.aoffset_p = 0; left_arm_ctrl_obs_B.aoffset_p <
             6; left_arm_ctrl_obs_B.aoffset_p++) {
          left_arm_ctrl_obs_B.a_idx_1_l += f->data[(left_arm_ctrl_obs_B.inner_p
            - 1) * 6 + left_arm_ctrl_obs_B.aoffset_p] * X->
            data[left_arm_ctrl_obs_B.o_tmp].f1[6 * left_arm_ctrl_obs_B.i_i +
            left_arm_ctrl_obs_B.aoffset_p];
        }

        left_arm_ctrl_obs_B.a0_n[left_arm_ctrl_obs_B.i_i] = f->data
          [(left_arm_ctrl_obs_B.m_l - 1) * 6 + left_arm_ctrl_obs_B.i_i] +
          left_arm_ctrl_obs_B.a_idx_1_l;
      }

      for (left_arm_ctrl_obs_B.i_i = 0; left_arm_ctrl_obs_B.i_i < 6;
           left_arm_ctrl_obs_B.i_i++) {
        f->data[left_arm_ctrl_obs_B.i_i + 6 * (left_arm_ctrl_obs_B.m_l - 1)] =
          left_arm_ctrl_obs_B.a0_n[left_arm_ctrl_obs_B.i_i];
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

static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal__e0_T
  *pStruct)
{
  emxFreeStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal__e0_T
  *pStruct)
{
  emxFreeStruct_k_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct)
{
  le_emxFreeStruct_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_k_robotics_mani_e(k_robotics_manip_internal_R_e_T
  *pStruct)
{
  emxFreeStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct)
{
  emxFreeStruct_k_robotics_mani_e(&pStruct->TreeInternal);
}

static void emxFreeStruct_k_robotics_man_e0(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slman_e0(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_k_robotics_man_e0(&pStruct->TreeInternal);
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

static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal__e0_T
  *pStruct)
{
  emxInitStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal__e0_T
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
      left_arm_ctrl_obs_B.msubspace_data[b_kstr] = tmp[b_kstr];
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
      left_arm_ctrl_obs_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      left_arm_ctrl_obs_B.msubspace_data[b_kstr] = 0;
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
    obj->JointInternal.MotionSubspace->data[b_kstr] =
      left_arm_ctrl_obs_B.msubspace_data[b_kstr];
  }

  return b_obj;
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

static void emxInitStruct_k_robotics_mani_e(k_robotics_manip_internal_R_e_T
  *pStruct)
{
  emxInitStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_e(robotics_slmanip_internal_b_e_T
  *pStruct)
{
  emxInitStruct_k_robotics_mani_e(&pStruct->TreeInternal);
}

static k_robotics_manip_internal_R_e_T *l_RigidBodyTree_RigidBodyTree_e
  (k_robotics_manip_internal_R_e_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9)
{
  k_robotics_manip_internal_R_e_T *b_obj;
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

static void emxInitStruct_k_robotics_man_e0(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slman_e0(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_k_robotics_man_e0(&pStruct->TreeInternal);
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
  robotics_slmanip_internal_blo_T *obj;
  emxArray_real_T_left_arm_ctrl_T *L;
  emxArray_real_T_left_arm_ctrl_T *lambda;
  emxArray_real_T_left_arm_ctrl_T *H;
  emxArray_real_T_left_arm_ctrl_T *tmp;
  static const real_T kp[7] = { 20.5, 1.0, 1.0, 14.5, 1.0, 10.5, 1.0 };

  static const int8_T kd[7] = { 7, 1, 1, 5, 1, 5, 1 };

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
    // MATLABSystem: '<S12>/SourceBlock' incorporates:
    //   Inport: '<S15>/In1'

    left_arm_ctrl_o_SystemCore_step(&left_arm_ctrl_obs_B.b_varargout_1,
      left_arm_ctrl_obs_B.qe,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Curr,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Rece,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_DataOffset,
      left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_Inf,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_p);

    // Outputs for Enabled SubSystem: '<S12>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S15>/Enable'

    if (left_arm_ctrl_obs_B.b_varargout_1) {
      for (left_arm_ctrl_obs_B.j_c = 0; left_arm_ctrl_obs_B.j_c < 7;
           left_arm_ctrl_obs_B.j_c++) {
        left_arm_ctrl_obs_B.In1.Data[left_arm_ctrl_obs_B.j_c] =
          left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.j_c];
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
        left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_p;
    }

    // End of MATLABSystem: '<S12>/SourceBlock'
    // End of Outputs for SubSystem: '<S12>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'

    // MATLABSystem: '<S6>/MATLAB System'
    RigidBodyTreeDynamics_massMat_e(&left_arm_ctrl_obs_DW.obj_jz.TreeInternal,
      left_arm_ctrl_obs_B.In1.Data, b);
    for (left_arm_ctrl_obs_B.j_c = 0; left_arm_ctrl_obs_B.j_c < 49;
         left_arm_ctrl_obs_B.j_c++) {
      left_arm_ctrl_obs_B.MATLABSystem[left_arm_ctrl_obs_B.j_c] = b->
        data[left_arm_ctrl_obs_B.j_c];
    }

    // End of MATLABSystem: '<S6>/MATLAB System'

    // Outputs for Atomic SubSystem: '<Root>/Subscribe1'
    // MATLABSystem: '<S13>/SourceBlock' incorporates:
    //   Inport: '<S16>/In1'

    left_arm_ctrl_SystemCore_step_e(&left_arm_ctrl_obs_B.b_varargout_1,
      left_arm_ctrl_obs_B.b_varargout_2_Data,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Curr,
      &left_arm_ctrl_obs_B.b_varargout_2_Data_SL_Info_Rece,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_DataOffset,
      left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_Inf,
      &left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_p);

    // Outputs for Enabled SubSystem: '<S13>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S16>/Enable'

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
        left_arm_ctrl_obs_B.b_varargout_2_Layout_Dim_SL_I_p;
    }

    // End of MATLABSystem: '<S13>/SourceBlock'
    // End of Outputs for SubSystem: '<S13>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe1'

    // MATLABSystem: '<S5>/MATLAB System'
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

  obj = &left_arm_ctrl_obs_DW.obj;
  RigidBodyTreeDynamics_massMatri(&left_arm_ctrl_obs_DW.obj.TreeInternal,
    left_arm_ctrl_obs_B.In1.Data, b, lambda);
  left_arm_ctrl_obs_B.vNum = obj->TreeInternal.VelocityNumber;
  left_arm_ctrl_obs_B.j_c = tmp->size[0];
  left_arm_ctrl_obs_B.i = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum);
  tmp->size[0] = left_arm_ctrl_obs_B.i;
  left_a_emxEnsureCapacity_real_T(tmp, left_arm_ctrl_obs_B.j_c);
  for (left_arm_ctrl_obs_B.j_c = 0; left_arm_ctrl_obs_B.j_c <
       left_arm_ctrl_obs_B.i; left_arm_ctrl_obs_B.j_c++) {
    tmp->data[left_arm_ctrl_obs_B.j_c] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj->TreeInternal,
    left_arm_ctrl_obs_B.In1.Data, &left_arm_ctrl_obs_X.Integrator_CSTATE[7], tmp,
    left_arm_ctrl_obs_P.Constant2_Value, left_arm_ctrl_obs_B.qe);
  left_arm_ctrl_ob_emxFree_real_T(&tmp);

  // MATLABSystem: '<S4>/MATLAB System'
  for (left_arm_ctrl_obs_B.j_c = 0; left_arm_ctrl_obs_B.j_c < 7;
       left_arm_ctrl_obs_B.j_c++) {
    left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.j_c] =
      left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.j_c] -
      left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.j_c];
  }

  if ((b->size[0] == 0) || (b->size[1] == 0)) {
    left_arm_ctrl_obs_B.iend = 0;
  } else {
    left_arm_ctrl_obs_B.u0 = b->size[0];
    left_arm_ctrl_obs_B.iend = b->size[1];
    if (left_arm_ctrl_obs_B.u0 > left_arm_ctrl_obs_B.iend) {
      left_arm_ctrl_obs_B.iend = left_arm_ctrl_obs_B.u0;
    }
  }

  left_arm_ctrl_ob_emxInit_real_T(&H, 2);

  // MATLABSystem: '<S4>/MATLAB System'
  left_arm_ctrl_obs_B.j_c = H->size[0] * H->size[1];
  H->size[0] = b->size[0];
  H->size[1] = b->size[1];
  left_a_emxEnsureCapacity_real_T(H, left_arm_ctrl_obs_B.j_c);
  left_arm_ctrl_obs_B.u0 = b->size[0] * b->size[1] - 1;
  for (left_arm_ctrl_obs_B.j_c = 0; left_arm_ctrl_obs_B.j_c <=
       left_arm_ctrl_obs_B.u0; left_arm_ctrl_obs_B.j_c++) {
    H->data[left_arm_ctrl_obs_B.j_c] = b->data[left_arm_ctrl_obs_B.j_c];
  }

  left_arm_ctrl_ob_emxFree_real_T(&b);

  // MATLABSystem: '<S4>/MATLAB System'
  left_arm_ctrl_obs_B.n = static_cast<int32_T>(((-1.0 - static_cast<real_T>
    (left_arm_ctrl_obs_B.iend)) + 1.0) / -1.0) - 1;
  for (left_arm_ctrl_obs_B.u0 = 0; left_arm_ctrl_obs_B.u0 <=
       left_arm_ctrl_obs_B.n; left_arm_ctrl_obs_B.u0++) {
    left_arm_ctrl_obs_B.j = static_cast<real_T>(left_arm_ctrl_obs_B.iend) + -
      static_cast<real_T>(left_arm_ctrl_obs_B.u0);
    left_arm_ctrl_obs_B.j_c = static_cast<int32_T>(left_arm_ctrl_obs_B.j);
    left_arm_ctrl_obs_B.qe_tmp = left_arm_ctrl_obs_B.j_c - 1;
    H->data[(static_cast<int32_T>(left_arm_ctrl_obs_B.j) + H->size[0] * (
              static_cast<int32_T>(left_arm_ctrl_obs_B.j) - 1)) - 1] = sqrt
      (H->data[(left_arm_ctrl_obs_B.qe_tmp * H->size[0] +
                left_arm_ctrl_obs_B.j_c) - 1]);
    left_arm_ctrl_obs_B.k = lambda->data[left_arm_ctrl_obs_B.qe_tmp];
    while (left_arm_ctrl_obs_B.k > 0.0) {
      left_arm_ctrl_obs_B.i_a = static_cast<int32_T>(left_arm_ctrl_obs_B.k) - 1;
      H->data[(static_cast<int32_T>(left_arm_ctrl_obs_B.j) + H->size[0] * (
                static_cast<int32_T>(left_arm_ctrl_obs_B.k) - 1)) - 1] = H->
        data[(left_arm_ctrl_obs_B.i_a * H->size[0] + left_arm_ctrl_obs_B.j_c) -
        1] / H->data[((static_cast<int32_T>(left_arm_ctrl_obs_B.j) - 1) *
                      H->size[0] + static_cast<int32_T>(left_arm_ctrl_obs_B.j))
        - 1];
      left_arm_ctrl_obs_B.k = lambda->data[left_arm_ctrl_obs_B.i_a];
    }

    left_arm_ctrl_obs_B.k = lambda->data[left_arm_ctrl_obs_B.qe_tmp];
    while (left_arm_ctrl_obs_B.k > 0.0) {
      left_arm_ctrl_obs_B.j = left_arm_ctrl_obs_B.k;
      while (left_arm_ctrl_obs_B.j > 0.0) {
        left_arm_ctrl_obs_B.qe_tmp = static_cast<int32_T>(left_arm_ctrl_obs_B.j)
          - 1;
        H->data[(static_cast<int32_T>(left_arm_ctrl_obs_B.k) + H->size[0] * (
                  static_cast<int32_T>(left_arm_ctrl_obs_B.j) - 1)) - 1] =
          H->data[(left_arm_ctrl_obs_B.qe_tmp * H->size[0] + static_cast<int32_T>
                   (left_arm_ctrl_obs_B.k)) - 1] - H->data[((static_cast<int32_T>
          (left_arm_ctrl_obs_B.k) - 1) * H->size[0] + left_arm_ctrl_obs_B.j_c) -
          1] * H->data[((static_cast<int32_T>(left_arm_ctrl_obs_B.j) - 1) *
                        H->size[0] + left_arm_ctrl_obs_B.j_c) - 1];
        left_arm_ctrl_obs_B.j = lambda->data[left_arm_ctrl_obs_B.qe_tmp];
      }

      left_arm_ctrl_obs_B.k = lambda->data[static_cast<int32_T>
        (left_arm_ctrl_obs_B.k) - 1];
    }
  }

  left_arm_ctrl_ob_emxInit_real_T(&L, 2);

  // MATLABSystem: '<S4>/MATLAB System'
  left_arm_ctrl_obs_B.j_c = L->size[0] * L->size[1];
  L->size[0] = H->size[0];
  L->size[1] = H->size[1];
  left_a_emxEnsureCapacity_real_T(L, left_arm_ctrl_obs_B.j_c);
  left_arm_ctrl_obs_B.u0 = H->size[0] * H->size[1] - 1;
  for (left_arm_ctrl_obs_B.j_c = 0; left_arm_ctrl_obs_B.j_c <=
       left_arm_ctrl_obs_B.u0; left_arm_ctrl_obs_B.j_c++) {
    L->data[left_arm_ctrl_obs_B.j_c] = H->data[left_arm_ctrl_obs_B.j_c];
  }

  left_arm_ctrl_obs_B.n = H->size[1];
  if ((H->size[0] == 0) || (H->size[1] == 0) || (1 >= H->size[1])) {
  } else {
    left_arm_ctrl_obs_B.iend = 0;
    for (left_arm_ctrl_obs_B.j_c = 2; left_arm_ctrl_obs_B.j_c <=
         left_arm_ctrl_obs_B.n; left_arm_ctrl_obs_B.j_c++) {
      for (left_arm_ctrl_obs_B.u0 = 0; left_arm_ctrl_obs_B.u0 <=
           left_arm_ctrl_obs_B.iend; left_arm_ctrl_obs_B.u0++) {
        L->data[left_arm_ctrl_obs_B.u0 + L->size[0] * (left_arm_ctrl_obs_B.j_c -
          1)] = 0.0;
      }

      if (left_arm_ctrl_obs_B.iend + 1 < H->size[0]) {
        left_arm_ctrl_obs_B.iend++;
      }
    }
  }

  left_arm_ctrl_ob_emxFree_real_T(&H);

  // MATLABSystem: '<S4>/MATLAB System'
  left_arm_ctrl_obs_B.n = static_cast<int32_T>(((-1.0 - left_arm_ctrl_obs_B.vNum)
    + 1.0) / -1.0) - 1;
  for (left_arm_ctrl_obs_B.u0 = 0; left_arm_ctrl_obs_B.u0 <=
       left_arm_ctrl_obs_B.n; left_arm_ctrl_obs_B.u0++) {
    left_arm_ctrl_obs_B.j_c = static_cast<int32_T>(left_arm_ctrl_obs_B.vNum + -
      static_cast<real_T>(left_arm_ctrl_obs_B.u0));
    left_arm_ctrl_obs_B.iend = left_arm_ctrl_obs_B.j_c - 1;
    left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.iend] /= L->data
      [(left_arm_ctrl_obs_B.iend * L->size[0] + left_arm_ctrl_obs_B.j_c) - 1];
    left_arm_ctrl_obs_B.j = lambda->data[left_arm_ctrl_obs_B.iend];
    while (left_arm_ctrl_obs_B.j > 0.0) {
      left_arm_ctrl_obs_B.qe_tmp = static_cast<int32_T>(left_arm_ctrl_obs_B.j) -
        1;
      left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.qe_tmp] -= L->data
        [(left_arm_ctrl_obs_B.qe_tmp * L->size[0] + left_arm_ctrl_obs_B.j_c) - 1]
        * left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.iend];
      left_arm_ctrl_obs_B.j = lambda->data[left_arm_ctrl_obs_B.qe_tmp];
    }
  }

  left_arm_ctrl_obs_B.i--;
  for (left_arm_ctrl_obs_B.u0 = 0; left_arm_ctrl_obs_B.u0 <=
       left_arm_ctrl_obs_B.i; left_arm_ctrl_obs_B.u0++) {
    left_arm_ctrl_obs_B.j = lambda->data[left_arm_ctrl_obs_B.u0];
    while (left_arm_ctrl_obs_B.j > 0.0) {
      left_arm_ctrl_obs_B.iend = static_cast<int32_T>(left_arm_ctrl_obs_B.j) - 1;
      left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.u0] -= L->
        data[left_arm_ctrl_obs_B.iend * L->size[0] + left_arm_ctrl_obs_B.u0] *
        left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.iend];
      left_arm_ctrl_obs_B.j = lambda->data[left_arm_ctrl_obs_B.iend];
    }

    left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.u0] /= L->data[L->size[0] *
      left_arm_ctrl_obs_B.u0 + left_arm_ctrl_obs_B.u0];
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
    // MATLABSystem: '<S9>/SinkBlock'
    Pub_left_arm_ctrl_obs_304.publish(&left_arm_ctrl_obs_B.BusAssignment1);

    // End of Outputs for SubSystem: '<Root>/Publish1'
  }

  // TransferFcn: '<Root>/Low Pass (z1)'
  left_arm_ctrl_obs_B.vNum = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)'
  left_arm_ctrl_obs_B.k = 0.0;

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
  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 5;
       left_arm_ctrl_obs_B.i++) {
    // TransferFcn: '<Root>/Low Pass (z1)'
    left_arm_ctrl_obs_B.vNum +=
      left_arm_ctrl_obs_P.LowPassz1_C[left_arm_ctrl_obs_B.i] *
      left_arm_ctrl_obs_X.LowPassz1_CSTATE[left_arm_ctrl_obs_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)'
    left_arm_ctrl_obs_B.k +=
      left_arm_ctrl_obs_P.LowPassz2_C[left_arm_ctrl_obs_B.i] *
      left_arm_ctrl_obs_X.LowPassz2_CSTATE[left_arm_ctrl_obs_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)1'
    left_arm_ctrl_obs_B.j +=
      left_arm_ctrl_obs_P.LowPassz21_C[left_arm_ctrl_obs_B.i] *
      left_arm_ctrl_obs_X.LowPassz21_CSTATE[left_arm_ctrl_obs_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)2'
    left_arm_ctrl_obs_B.LowPassz22 +=
      left_arm_ctrl_obs_P.LowPassz22_C[left_arm_ctrl_obs_B.i] *
      left_arm_ctrl_obs_X.LowPassz22_CSTATE[left_arm_ctrl_obs_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)3'
    left_arm_ctrl_obs_B.LowPassz23 +=
      left_arm_ctrl_obs_P.LowPassz23_C[left_arm_ctrl_obs_B.i] *
      left_arm_ctrl_obs_X.LowPassz23_CSTATE[left_arm_ctrl_obs_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)4'
    left_arm_ctrl_obs_B.LowPassz24 +=
      left_arm_ctrl_obs_P.LowPassz24_C[left_arm_ctrl_obs_B.i] *
      left_arm_ctrl_obs_X.LowPassz24_CSTATE[left_arm_ctrl_obs_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)5'
    left_arm_ctrl_obs_B.LowPassz25 +=
      left_arm_ctrl_obs_P.LowPassz25_C[left_arm_ctrl_obs_B.i] *
      left_arm_ctrl_obs_X.LowPassz25_CSTATE[left_arm_ctrl_obs_B.i];
  }

  // MATLAB Function: '<Root>/mass estimator' incorporates:
  //   Integrator: '<Root>/Integrator'

  left_arm_ctrl_obs_B.vel = 0.0;
  left_arm_ctrl_obs_B.scale = 3.3121686421112381E-170;
  for (left_arm_ctrl_obs_B.i = 0; left_arm_ctrl_obs_B.i < 7;
       left_arm_ctrl_obs_B.i++) {
    left_arm_ctrl_obs_B.absxk = fabs
      (left_arm_ctrl_obs_X.Integrator_CSTATE[left_arm_ctrl_obs_B.i + 7]);
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
  left_arm_ctrl_obs_B.scale = sin((left_arm_ctrl_obs_B.In1.Data[0] +
    left_arm_ctrl_obs_B.In1.Data[3]) + left_arm_ctrl_obs_B.In1.Data[5]);
  if (fabs(left_arm_ctrl_obs_B.scale) > 0.1) {
    if (left_arm_ctrl_obs_B.vel < 0.1) {
      // SignalConversion generated from: '<S14>/ SFunction '
      left_arm_ctrl_obs_B.qe[0] = left_arm_ctrl_obs_B.vNum;
      left_arm_ctrl_obs_B.qe[1] = left_arm_ctrl_obs_B.k;
      left_arm_ctrl_obs_B.qe[2] = left_arm_ctrl_obs_B.j;
      left_arm_ctrl_obs_B.qe[3] = left_arm_ctrl_obs_B.LowPassz22;
      left_arm_ctrl_obs_B.qe[4] = left_arm_ctrl_obs_B.LowPassz23;
      left_arm_ctrl_obs_B.qe[5] = left_arm_ctrl_obs_B.LowPassz24;
      left_arm_ctrl_obs_B.qe[6] = left_arm_ctrl_obs_B.LowPassz25;
      for (left_arm_ctrl_obs_B.j_c = 0; left_arm_ctrl_obs_B.j_c < 7;
           left_arm_ctrl_obs_B.j_c++) {
        left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.j_c] = 0.0;
        for (left_arm_ctrl_obs_B.qe_tmp = 0; left_arm_ctrl_obs_B.qe_tmp < 7;
             left_arm_ctrl_obs_B.qe_tmp++) {
          left_arm_ctrl_obs_B.torque[left_arm_ctrl_obs_B.j_c] +=
            left_arm_ctrl_obs_B.MATLABSystem[7 * left_arm_ctrl_obs_B.qe_tmp +
            left_arm_ctrl_obs_B.j_c] *
            left_arm_ctrl_obs_B.qe[left_arm_ctrl_obs_B.qe_tmp];
        }
      }

      left_arm_ctrl_obs_B.vNum = -left_arm_ctrl_obs_B.torque[5] /
        (1.9129500000000002 * left_arm_ctrl_obs_B.scale);
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
    // MATLABSystem: '<S10>/SinkBlock'
    Pub_left_arm_ctrl_obs_311.publish(&left_arm_ctrl_obs_B.BusAssignment2);

    // End of Outputs for SubSystem: '<Root>/Publish2'
  }

  // End of RateTransition: '<Root>/Rate Transition'
  if (rtmIsMajorTimeStep(left_arm_ctrl_obs_M) &&
      left_arm_ctrl_obs_M->Timing.TaskCounters.TID[1] == 0) {
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
    // MATLABSystem: '<S11>/SinkBlock'
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
    static const char_T tmp[25] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r', 'e',
      '/', 'l', 'a', '_', 'c', 'u', 'r', 'r', 'e', 'n', 't', '_', 'p', 'o', 's',
      'e' };

    static const char_T tmp_0[28] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'g', 'o', 'a', 'l', '_', 't', 'r', 'a', 'j', 'e',
      'c', 't', 'o', 'r', 'y' };

    static const char_T tmp_1[19] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 't', 'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_2[15] = { '/', 'e', 's', 't', 'i', 'm', 'a', 't',
      'e', 'd', '_', 'm', 'a', 's', 's' };

    static const char_T tmp_3[28] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 'e', 's', 't', 'i', 'm', 'a', 't', 'e', 'd', '_',
      's', 'p', 'e', 'e', 'd' };

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S12>/SourceBlock'
    left_arm_ctrl_obs_DW.obj_m.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_m.isInitialized = 1;
    for (is = 0; is < 25; is++) {
      left_arm_ctrl_obs_B.cv1[is] = tmp[is];
    }

    left_arm_ctrl_obs_B.cv1[25] = '\x00';
    Sub_left_arm_ctrl_obs_299.createSubscriber(left_arm_ctrl_obs_B.cv1, 1);
    left_arm_ctrl_obs_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/SourceBlock'
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

    // Start for MATLABSystem: '<S6>/MATLAB System'
    left_arm_ctrl_obs_DW.obj_jz.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_jz.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_e0(&left_arm_ctrl_obs_DW.obj_jz.TreeInternal,
      &left_arm_ctrl_obs_DW.gobj_0, &left_arm_ctrl_obs_DW.gobj_19,
      &left_arm_ctrl_obs_DW.gobj_18, &left_arm_ctrl_obs_DW.gobj_17,
      &left_arm_ctrl_obs_DW.gobj_16, &left_arm_ctrl_obs_DW.gobj_15,
      &left_arm_ctrl_obs_DW.gobj_14, &left_arm_ctrl_obs_DW.gobj_13,
      &left_arm_ctrl_obs_DW.gobj_12, &left_arm_ctrl_obs_DW.gobj_11);

    // Start for Atomic SubSystem: '<Root>/Subscribe1'
    // Start for MATLABSystem: '<S13>/SourceBlock'
    left_arm_ctrl_obs_DW.obj_g.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_g.isInitialized = 1;
    for (is = 0; is < 28; is++) {
      left_arm_ctrl_obs_B.cv[is] = tmp_0[is];
    }

    left_arm_ctrl_obs_B.cv[28] = '\x00';
    Sub_left_arm_ctrl_obs_318.createSubscriber(left_arm_ctrl_obs_B.cv, 1);
    left_arm_ctrl_obs_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe1'
    emxInitStruct_robotics_slmani_e(&left_arm_ctrl_obs_DW.obj_j);
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

    // Start for MATLABSystem: '<S5>/MATLAB System'
    left_arm_ctrl_obs_DW.obj_j.isInitialized = 0;
    left_arm_ctrl_obs_DW.obj_j.isInitialized = 1;
    l_RigidBodyTree_RigidBodyTree_e(&left_arm_ctrl_obs_DW.obj_j.TreeInternal,
      &left_arm_ctrl_obs_DW.gobj_0_f, &left_arm_ctrl_obs_DW.gobj_19_h,
      &left_arm_ctrl_obs_DW.gobj_18_b, &left_arm_ctrl_obs_DW.gobj_17_d,
      &left_arm_ctrl_obs_DW.gobj_16_a, &left_arm_ctrl_obs_DW.gobj_15_k,
      &left_arm_ctrl_obs_DW.gobj_14_a, &left_arm_ctrl_obs_DW.gobj_13_l,
      &left_arm_ctrl_obs_DW.gobj_12_g, &left_arm_ctrl_obs_DW.gobj_11_b);
    emxInitStruct_robotics_slman_e0(&left_arm_ctrl_obs_DW.obj);
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

    // Start for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_d.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_d.isInitialized = 1;
    for (is = 0; is < 19; is++) {
      left_arm_ctrl_obs_B.cv2[is] = tmp_1[is];
    }

    left_arm_ctrl_obs_B.cv2[19] = '\x00';
    Pub_left_arm_ctrl_obs_304.createPublisher(left_arm_ctrl_obs_B.cv2, 1);
    left_arm_ctrl_obs_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish1'

    // Start for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S10>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_a.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_a.isInitialized = 1;
    for (is = 0; is < 15; is++) {
      left_arm_ctrl_obs_B.cv3[is] = tmp_2[is];
    }

    left_arm_ctrl_obs_B.cv3[15] = '\x00';
    Pub_left_arm_ctrl_obs_311.createPublisher(left_arm_ctrl_obs_B.cv3, 1);
    left_arm_ctrl_obs_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish2'

    // Start for Atomic SubSystem: '<Root>/Publish3'
    // Start for MATLABSystem: '<S11>/SinkBlock'
    left_arm_ctrl_obs_DW.obj_b.matlabCodegenIsDeleted = false;
    left_arm_ctrl_obs_DW.obj_b.isInitialized = 1;
    for (is = 0; is < 28; is++) {
      left_arm_ctrl_obs_B.cv[is] = tmp_3[is];
    }

    left_arm_ctrl_obs_B.cv[28] = '\x00';
    Pub_left_arm_ctrl_obs_331.createPublisher(left_arm_ctrl_obs_B.cv, 1);
    left_arm_ctrl_obs_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S11>/SinkBlock'
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
    // SystemInitialize for Enabled SubSystem: '<S12>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S15>/Out1'
    left_arm_ctrl_obs_B.In1 = left_arm_ctrl_obs_P.Out1_Y0_a;

    // End of SystemInitialize for SubSystem: '<S12>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S13>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S16>/Out1'
    left_arm_ctrl_obs_B.In1_e = left_arm_ctrl_obs_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S13>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe1'
  }
}

// Model terminate function
void left_arm_ctrl_obs_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S12>/SourceBlock'
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

  // Terminate for Atomic SubSystem: '<Root>/Subscribe1'
  // Terminate for MATLABSystem: '<S13>/SourceBlock'
  matlabCodegenHandle_matlabC_e0h(&left_arm_ctrl_obs_DW.obj_g);

  // End of Terminate for SubSystem: '<Root>/Subscribe1'
  emxFreeStruct_robotics_slmani_e(&left_arm_ctrl_obs_DW.obj_j);
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
  emxFreeStruct_robotics_slman_e0(&left_arm_ctrl_obs_DW.obj);
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

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_d);

  // End of Terminate for SubSystem: '<Root>/Publish1'

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S10>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_a);

  // End of Terminate for SubSystem: '<Root>/Publish2'

  // Terminate for Atomic SubSystem: '<Root>/Publish3'
  // Terminate for MATLABSystem: '<S11>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_ctrl_obs_DW.obj_b);

  // End of Terminate for SubSystem: '<Root>/Publish3'
}

//
// File trailer for generated code.
//
// [EOF]
//
