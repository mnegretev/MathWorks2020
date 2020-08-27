//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: smo_estimator.cpp
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
#include "smo_estimator.h"
#include "smo_estimator_private.h"

// Block signals (default storage)
B_smo_estimator_T smo_estimator_B;

// Continuous states
X_smo_estimator_T smo_estimator_X;

// Block states (default storage)
DW_smo_estimator_T smo_estimator_DW;

// Real-time model
RT_MODEL_smo_estimator_T smo_estimator_M_ = RT_MODEL_smo_estimator_T();
RT_MODEL_smo_estimator_T *const smo_estimator_M = &smo_estimator_M_;

// Forward declaration for local functions
static void smo_estimator_emxInit_real_T(emxArray_real_T_smo_estimator_T
  **pEmxArray, int32_T numDimensions);
static void smo_estimator_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_smo_estimator_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void smo_es_emxEnsureCapacity_real_T(emxArray_real_T_smo_estimator_T
  *emxArray, int32_T oldNumel);
static void smo_estima_emxInit_f_cell_wrap1(emxArray_f_cell_wrap_smo_es_m_T
  **pEmxArray, int32_T numDimensions);
static void emxEnsureCapacity_f_cell_wrap1(emxArray_f_cell_wrap_smo_es_m_T
  *emxArray, int32_T oldNumel);
static void rigidBodyJoint_get_JointAxis_m(const c_rigidBodyJoint_smo_estima_m_T
  *obj, real_T ax[3]);
static void smo_estimator_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_m(const
  c_rigidBodyJoint_smo_estima_m_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_smo_estima_m_T *obj, real_T T[16]);
static void smo_estimator_tforminv(const real_T T[16], real_T Tinv[16]);
static void smo_estimat_tformToSpatialXform(const real_T T[16], real_T X[36]);
static void smo_estimator_emxFree_real_T(emxArray_real_T_smo_estimator_T
  **pEmxArray);
static void smo_estima_emxFree_f_cell_wrap1(emxArray_f_cell_wrap_smo_es_m_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMatri(q_robotics_manip_internal_R_m_T
  *robot, const real_T q[7], emxArray_real_T_smo_estimator_T *H);
static void smo_estimat_emxInit_f_cell_wrap(emxArray_f_cell_wrap_smo_esti_T
  **pEmxArray, int32_T numDimensions);
static void smo_estimator_emxInit_char_T(emxArray_char_T_smo_estimator_T
  **pEmxArray, int32_T numDimensions);
static void s_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_smo_esti_T
  *emxArray, int32_T oldNumel);
static void smo_es_emxEnsureCapacity_char_T(emxArray_char_T_smo_estimator_T
  *emxArray, int32_T oldNumel);
static void sm_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_smo_estimato_T *obj, real_T ax[3]);
static void smo_estimator_emxFree_char_T(emxArray_char_T_smo_estimator_T
  **pEmxArray);
static void RigidBodyTree_forwardKinematics(q_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[7], emxArray_f_cell_wrap_smo_esti_T *Ttree);
static void smo_estimat_emxFree_f_cell_wrap(emxArray_f_cell_wrap_smo_esti_T
  **pEmxArray);
static void smo_estimator_SystemCore_step_m(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_smo_estimator_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void RigidBodyTreeDynamics_massMat_m(q_robotics_manip_internal__mr_T
  *robot, const real_T q[7], emxArray_real_T_smo_estimator_T *H,
  emxArray_real_T_smo_estimator_T *lambda);
static void RigidBodyTreeDynamics_inverseDy(q_robotics_manip_internal__mr_T
  *robot, const real_T q[7], const real_T qdot[7], const real_T fext[60], real_T
  tau[7]);
static void smo_estimator_atan2(const real_T y_data[], const int32_T y_size[3],
  const real_T x_data[], const int32_T x_size[3], real_T r_data[], int32_T
  r_size[3]);
static void matlabCodegenHandle_matla_mrfu4(ros_slros_internal_block_Subs_T *obj);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_smo_estima_m_T
  *pStruct);
static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_R_m_T
  *pStruct);
static void emxFreeStruct_q_robotics_manip_(q_robotics_manip_internal_R_m_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_b_m_T
  *pStruct);
static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_R_m_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_smo_estimato_T
  *pStruct);
static void emxFreeStruct_p_robotics_mani_m(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_q_robotics_mani_m(q_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_m(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxFreeStruct_o_robotics_mani_m(o_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_q_robotics_man_mr(q_robotics_manip_internal__mr_T
  *pStruct);
static void emxFreeStruct_robotics_slman_mr(robotics_slmanip_internal__mr_T
  *pStruct);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHandle_matlabC_mrf(ros_slros_internal_block_Publ_T *obj);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_smo_estima_m_T
  *pStruct);
static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_R_m_T
  *pStruct);
static void emxInitStruct_q_robotics_manip_(q_robotics_manip_internal_R_m_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_b_m_T
  *pStruct);
static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_R_m_T
  *pStruct);
static o_robotics_manip_internal_R_m_T *smo_RigidBody_RigidBody_mrfu4kk
  (o_robotics_manip_internal_R_m_T *obj);
static o_robotics_manip_internal_R_m_T *sm_RigidBody_RigidBody_mrfu4kk1
  (o_robotics_manip_internal_R_m_T *obj);
static o_robotics_manip_internal_R_m_T *s_RigidBody_RigidBody_mrfu4kk1b
  (o_robotics_manip_internal_R_m_T *obj);
static o_robotics_manip_internal_R_m_T *RigidBody_RigidBody_mrfu4kk1bh
  (o_robotics_manip_internal_R_m_T *obj);
static o_robotics_manip_internal_R_m_T *RigidBody_RigidBody_mrfu4kk1bht
  (o_robotics_manip_internal_R_m_T *obj);
static o_robotics_manip_internal_R_m_T *RigidBody_RigidBod_mrfu4kk1bhtk
  (o_robotics_manip_internal_R_m_T *obj);
static o_robotics_manip_internal_R_m_T *RigidBody_RigidBo_mrfu4kk1bhtk3
  (o_robotics_manip_internal_R_m_T *obj);
static o_robotics_manip_internal_R_m_T *RigidBody_RigidB_mrfu4kk1bhtk3a
  (o_robotics_manip_internal_R_m_T *obj);
static o_robotics_manip_internal_R_m_T *RigidBody_Rigid_mrfu4kk1bhtk3a2
  (o_robotics_manip_internal_R_m_T *obj);
static o_robotics_manip_internal_R_m_T *s_RigidBody_Rigid_h
  (o_robotics_manip_internal_R_m_T *obj);
static p_robotics_manip_internal_R_m_T *s_RigidBody_Rigid_o
  (p_robotics_manip_internal_R_m_T *obj);
static q_robotics_manip_internal_R_m_T *s_RigidBodyTree_RigidBodyTree_m
  (q_robotics_manip_internal_R_m_T *obj, o_robotics_manip_internal_R_m_T *iobj_0,
   o_robotics_manip_internal_R_m_T *iobj_1, o_robotics_manip_internal_R_m_T
   *iobj_2, o_robotics_manip_internal_R_m_T *iobj_3,
   o_robotics_manip_internal_R_m_T *iobj_4, o_robotics_manip_internal_R_m_T
   *iobj_5, o_robotics_manip_internal_R_m_T *iobj_6,
   o_robotics_manip_internal_R_m_T *iobj_7, o_robotics_manip_internal_R_m_T
   *iobj_8, o_robotics_manip_internal_R_m_T *iobj_9);
static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_smo_estimato_T
  *pStruct);
static void emxInitStruct_p_robotics_mani_m(p_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_q_robotics_mani_m(q_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmani_m(robotics_slmanip_internal_blo_T
  *pStruct);
static void emxInitStruct_o_robotics_mani_m(o_robotics_manip_internal_Rig_T
  *pStruct);
static o_robotics_manip_internal_Rig_T *smo_estimat_RigidBody_RigidBody
  (o_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *smo_estim_RigidBody_RigidBody_m
  (o_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *smo_esti_RigidBody_RigidBody_mr
  (o_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *smo_est_RigidBody_RigidBody_mrf
  (o_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *smo_es_RigidBody_RigidBody_mrfu
  (o_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *smo_e_RigidBody_RigidBody_mrfu4
  (o_robotics_manip_internal_Rig_T *obj);
static o_robotics_manip_internal_Rig_T *smo__RigidBody_RigidBody_mrfu4k
  (o_robotics_manip_internal_Rig_T *obj);
static q_robotics_manip_internal_Rig_T *smo_RigidBodyTree_RigidBodyTree
  (q_robotics_manip_internal_Rig_T *obj, o_robotics_manip_internal_Rig_T *iobj_0,
   o_robotics_manip_internal_Rig_T *iobj_1, o_robotics_manip_internal_Rig_T
   *iobj_2, o_robotics_manip_internal_Rig_T *iobj_3,
   o_robotics_manip_internal_Rig_T *iobj_4, o_robotics_manip_internal_Rig_T
   *iobj_5, o_robotics_manip_internal_Rig_T *iobj_6,
   o_robotics_manip_internal_Rig_T *iobj_7, o_robotics_manip_internal_Rig_T
   *iobj_8, o_robotics_manip_internal_Rig_T *iobj_9);
static void emxInitStruct_q_robotics_man_mr(q_robotics_manip_internal__mr_T
  *pStruct);
static void emxInitStruct_robotics_slman_mr(robotics_slmanip_internal__mr_T
  *pStruct);
static q_robotics_manip_internal__mr_T *RigidBodyTree_RigidBodyTree_mr
  (q_robotics_manip_internal__mr_T *obj, o_robotics_manip_internal_R_m_T *iobj_0,
   o_robotics_manip_internal_R_m_T *iobj_1, o_robotics_manip_internal_R_m_T
   *iobj_2, o_robotics_manip_internal_R_m_T *iobj_3,
   o_robotics_manip_internal_R_m_T *iobj_4, o_robotics_manip_internal_R_m_T
   *iobj_5, o_robotics_manip_internal_R_m_T *iobj_6,
   o_robotics_manip_internal_R_m_T *iobj_7, o_robotics_manip_internal_R_m_T
   *iobj_8, o_robotics_manip_internal_R_m_T *iobj_9);
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

  (smo_estimator_M->Timing.TaskCounters.TID[2])++;
  if ((smo_estimator_M->Timing.TaskCounters.TID[2]) > 9) {// Sample time: [0.04s, 0.0s] 
    smo_estimator_M->Timing.TaskCounters.TID[2] = 0;
  }

  (smo_estimator_M->Timing.TaskCounters.TID[3])++;
  if ((smo_estimator_M->Timing.TaskCounters.TID[3]) > 249) {// Sample time: [1.0s, 0.0s] 
    smo_estimator_M->Timing.TaskCounters.TID[3] = 0;
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
  int_T nXc = 42;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  smo_estimator_derivatives();

  // f1 = f(t + (h/2), y + (h/2)*f0)
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  smo_estimator_step();
  smo_estimator_derivatives();

  // f2 = f(t + (h/2), y + (h/2)*f1)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  smo_estimator_step();
  smo_estimator_derivatives();

  // f3 = f(t + h, y + h*f2)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  smo_estimator_step();
  smo_estimator_derivatives();

  // tnew = t + h
  // ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3)
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void smo_estimator_emxInit_real_T(emxArray_real_T_smo_estimator_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_smo_estimator_T *emxArray;
  *pEmxArray = (emxArray_real_T_smo_estimator_T *)malloc(sizeof
    (emxArray_real_T_smo_estimator_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (smo_estimator_B.i_b = 0; smo_estimator_B.i_b < numDimensions;
       smo_estimator_B.i_b++) {
    emxArray->size[smo_estimator_B.i_b] = 0;
  }
}

static void smo_estimator_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_smo_estimator_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_smo_estimator_299.getLatestMessage
    (&smo_estimator_B.b_varargout_2_m);
  for (smo_estimator_B.i_px = 0; smo_estimator_B.i_px < 7; smo_estimator_B.i_px
       ++) {
    varargout_2_Data[smo_estimator_B.i_px] =
      smo_estimator_B.b_varargout_2_m.Data[smo_estimator_B.i_px];
  }

  *varargout_2_Data_SL_Info_Curren =
    smo_estimator_B.b_varargout_2_m.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    smo_estimator_B.b_varargout_2_m.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    smo_estimator_B.b_varargout_2_m.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &smo_estimator_B.b_varargout_2_m.Layout.Dim[0], sizeof
         (SL_Bus_smo_estimator_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    smo_estimator_B.b_varargout_2_m.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    smo_estimator_B.b_varargout_2_m.Layout.Dim_SL_Info.ReceivedLength;
}

static void smo_es_emxEnsureCapacity_real_T(emxArray_real_T_smo_estimator_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  smo_estimator_B.newNumel = 1;
  for (smo_estimator_B.i_f = 0; smo_estimator_B.i_f < emxArray->numDimensions;
       smo_estimator_B.i_f++) {
    smo_estimator_B.newNumel *= emxArray->size[smo_estimator_B.i_f];
  }

  if (smo_estimator_B.newNumel > emxArray->allocatedSize) {
    smo_estimator_B.i_f = emxArray->allocatedSize;
    if (smo_estimator_B.i_f < 16) {
      smo_estimator_B.i_f = 16;
    }

    while (smo_estimator_B.i_f < smo_estimator_B.newNumel) {
      if (smo_estimator_B.i_f > 1073741823) {
        smo_estimator_B.i_f = MAX_int32_T;
      } else {
        smo_estimator_B.i_f <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(smo_estimator_B.i_f), sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = smo_estimator_B.i_f;
    emxArray->canFreeData = true;
  }
}

static void smo_estima_emxInit_f_cell_wrap1(emxArray_f_cell_wrap_smo_es_m_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_smo_es_m_T *emxArray;
  *pEmxArray = (emxArray_f_cell_wrap_smo_es_m_T *)malloc(sizeof
    (emxArray_f_cell_wrap_smo_es_m_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_smo_estimator_m_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (smo_estimator_B.i_c = 0; smo_estimator_B.i_c < numDimensions;
       smo_estimator_B.i_c++) {
    emxArray->size[smo_estimator_B.i_c] = 0;
  }
}

static void emxEnsureCapacity_f_cell_wrap1(emxArray_f_cell_wrap_smo_es_m_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  smo_estimator_B.newNumel_a = 1;
  for (smo_estimator_B.i_kb = 0; smo_estimator_B.i_kb < emxArray->numDimensions;
       smo_estimator_B.i_kb++) {
    smo_estimator_B.newNumel_a *= emxArray->size[smo_estimator_B.i_kb];
  }

  if (smo_estimator_B.newNumel_a > emxArray->allocatedSize) {
    smo_estimator_B.i_kb = emxArray->allocatedSize;
    if (smo_estimator_B.i_kb < 16) {
      smo_estimator_B.i_kb = 16;
    }

    while (smo_estimator_B.i_kb < smo_estimator_B.newNumel_a) {
      if (smo_estimator_B.i_kb > 1073741823) {
        smo_estimator_B.i_kb = MAX_int32_T;
      } else {
        smo_estimator_B.i_kb <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(smo_estimator_B.i_kb), sizeof
                     (f_cell_wrap_smo_estimator_m_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_smo_estimator_m_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_smo_estimator_m_T *)newData;
    emxArray->allocatedSize = smo_estimator_B.i_kb;
    emxArray->canFreeData = true;
  }
}

static void rigidBodyJoint_get_JointAxis_m(const c_rigidBodyJoint_smo_estima_m_T
  *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (smo_estimator_B.b_kstr_e = 0; smo_estimator_B.b_kstr_e < 8;
       smo_estimator_B.b_kstr_e++) {
    smo_estimator_B.b_m3[smo_estimator_B.b_kstr_e] =
      tmp[smo_estimator_B.b_kstr_e];
  }

  smo_estimator_B.b_bool_m = false;
  if (obj->Type->size[1] == 8) {
    smo_estimator_B.b_kstr_e = 1;
    do {
      exitg1 = 0;
      if (smo_estimator_B.b_kstr_e - 1 < 8) {
        smo_estimator_B.kstr_j = smo_estimator_B.b_kstr_e - 1;
        if (obj->Type->data[smo_estimator_B.kstr_j] !=
            smo_estimator_B.b_m3[smo_estimator_B.kstr_j]) {
          exitg1 = 1;
        } else {
          smo_estimator_B.b_kstr_e++;
        }
      } else {
        smo_estimator_B.b_bool_m = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (smo_estimator_B.b_bool_m) {
    guard1 = true;
  } else {
    for (smo_estimator_B.b_kstr_e = 0; smo_estimator_B.b_kstr_e < 9;
         smo_estimator_B.b_kstr_e++) {
      smo_estimator_B.b_m[smo_estimator_B.b_kstr_e] =
        tmp_0[smo_estimator_B.b_kstr_e];
    }

    smo_estimator_B.b_bool_m = false;
    if (obj->Type->size[1] == 9) {
      smo_estimator_B.b_kstr_e = 1;
      do {
        exitg1 = 0;
        if (smo_estimator_B.b_kstr_e - 1 < 9) {
          smo_estimator_B.kstr_j = smo_estimator_B.b_kstr_e - 1;
          if (obj->Type->data[smo_estimator_B.kstr_j] !=
              smo_estimator_B.b_m[smo_estimator_B.kstr_j]) {
            exitg1 = 1;
          } else {
            smo_estimator_B.b_kstr_e++;
          }
        } else {
          smo_estimator_B.b_bool_m = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (smo_estimator_B.b_bool_m) {
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

static void smo_estimator_cat(real_T varargin_1, real_T varargin_2, real_T
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

static void rigidBodyJoint_transformBodyT_m(const
  c_rigidBodyJoint_smo_estima_m_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (smo_estimator_B.b_kstr_a = 0; smo_estimator_B.b_kstr_a < 5;
       smo_estimator_B.b_kstr_a++) {
    smo_estimator_B.b_h2[smo_estimator_B.b_kstr_a] =
      tmp[smo_estimator_B.b_kstr_a];
  }

  smo_estimator_B.b_bool_e = false;
  if (obj->Type->size[1] == 5) {
    smo_estimator_B.b_kstr_a = 1;
    do {
      exitg1 = 0;
      if (smo_estimator_B.b_kstr_a - 1 < 5) {
        smo_estimator_B.kstr = smo_estimator_B.b_kstr_a - 1;
        if (obj->Type->data[smo_estimator_B.kstr] !=
            smo_estimator_B.b_h2[smo_estimator_B.kstr]) {
          exitg1 = 1;
        } else {
          smo_estimator_B.b_kstr_a++;
        }
      } else {
        smo_estimator_B.b_bool_e = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (smo_estimator_B.b_bool_e) {
    smo_estimator_B.b_kstr_a = 0;
  } else {
    for (smo_estimator_B.b_kstr_a = 0; smo_estimator_B.b_kstr_a < 8;
         smo_estimator_B.b_kstr_a++) {
      smo_estimator_B.b_md[smo_estimator_B.b_kstr_a] =
        tmp_0[smo_estimator_B.b_kstr_a];
    }

    smo_estimator_B.b_bool_e = false;
    if (obj->Type->size[1] == 8) {
      smo_estimator_B.b_kstr_a = 1;
      do {
        exitg1 = 0;
        if (smo_estimator_B.b_kstr_a - 1 < 8) {
          smo_estimator_B.kstr = smo_estimator_B.b_kstr_a - 1;
          if (obj->Type->data[smo_estimator_B.kstr] !=
              smo_estimator_B.b_md[smo_estimator_B.kstr]) {
            exitg1 = 1;
          } else {
            smo_estimator_B.b_kstr_a++;
          }
        } else {
          smo_estimator_B.b_bool_e = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (smo_estimator_B.b_bool_e) {
      smo_estimator_B.b_kstr_a = 1;
    } else {
      smo_estimator_B.b_kstr_a = -1;
    }
  }

  switch (smo_estimator_B.b_kstr_a) {
   case 0:
    memset(&smo_estimator_B.TJ[0], 0, sizeof(real_T) << 4U);
    smo_estimator_B.TJ[0] = 1.0;
    smo_estimator_B.TJ[5] = 1.0;
    smo_estimator_B.TJ[10] = 1.0;
    smo_estimator_B.TJ[15] = 1.0;
    break;

   case 1:
    rigidBodyJoint_get_JointAxis_m(obj, smo_estimator_B.v);
    smo_estimator_B.result_data[0] = smo_estimator_B.v[0];
    smo_estimator_B.result_data[1] = smo_estimator_B.v[1];
    smo_estimator_B.result_data[2] = smo_estimator_B.v[2];
    if (0 <= (*q_size != 0) - 1) {
      smo_estimator_B.result_data[3] = q_data[0];
    }

    smo_estimator_B.cth = 1.0 / sqrt((smo_estimator_B.result_data[0] *
      smo_estimator_B.result_data[0] + smo_estimator_B.result_data[1] *
      smo_estimator_B.result_data[1]) + smo_estimator_B.result_data[2] *
      smo_estimator_B.result_data[2]);
    smo_estimator_B.v[0] = smo_estimator_B.result_data[0] * smo_estimator_B.cth;
    smo_estimator_B.v[1] = smo_estimator_B.result_data[1] * smo_estimator_B.cth;
    smo_estimator_B.v[2] = smo_estimator_B.result_data[2] * smo_estimator_B.cth;
    smo_estimator_B.cth = cos(smo_estimator_B.result_data[3]);
    smo_estimator_B.sth = sin(smo_estimator_B.result_data[3]);
    smo_estimator_B.tempR_tmp = smo_estimator_B.v[1] * smo_estimator_B.v[0] *
      (1.0 - smo_estimator_B.cth);
    smo_estimator_B.tempR_tmp_e = smo_estimator_B.v[2] * smo_estimator_B.sth;
    smo_estimator_B.tempR_tmp_a = smo_estimator_B.v[2] * smo_estimator_B.v[0] *
      (1.0 - smo_estimator_B.cth);
    smo_estimator_B.tempR_tmp_as = smo_estimator_B.v[1] * smo_estimator_B.sth;
    smo_estimator_B.tempR_tmp_i = smo_estimator_B.v[2] * smo_estimator_B.v[1] *
      (1.0 - smo_estimator_B.cth);
    smo_estimator_B.sth *= smo_estimator_B.v[0];
    smo_estimator_cat(smo_estimator_B.v[0] * smo_estimator_B.v[0] * (1.0 -
      smo_estimator_B.cth) + smo_estimator_B.cth, smo_estimator_B.tempR_tmp -
                      smo_estimator_B.tempR_tmp_e, smo_estimator_B.tempR_tmp_a +
                      smo_estimator_B.tempR_tmp_as, smo_estimator_B.tempR_tmp +
                      smo_estimator_B.tempR_tmp_e, smo_estimator_B.v[1] *
                      smo_estimator_B.v[1] * (1.0 - smo_estimator_B.cth) +
                      smo_estimator_B.cth, smo_estimator_B.tempR_tmp_i -
                      smo_estimator_B.sth, smo_estimator_B.tempR_tmp_a -
                      smo_estimator_B.tempR_tmp_as, smo_estimator_B.tempR_tmp_i
                      + smo_estimator_B.sth, smo_estimator_B.v[2] *
                      smo_estimator_B.v[2] * (1.0 - smo_estimator_B.cth) +
                      smo_estimator_B.cth, smo_estimator_B.tempR);
    for (smo_estimator_B.b_kstr_a = 0; smo_estimator_B.b_kstr_a < 3;
         smo_estimator_B.b_kstr_a++) {
      smo_estimator_B.kstr = smo_estimator_B.b_kstr_a + 1;
      smo_estimator_B.R_g[smo_estimator_B.kstr - 1] = smo_estimator_B.tempR
        [(smo_estimator_B.kstr - 1) * 3];
      smo_estimator_B.kstr = smo_estimator_B.b_kstr_a + 1;
      smo_estimator_B.R_g[smo_estimator_B.kstr + 2] = smo_estimator_B.tempR
        [(smo_estimator_B.kstr - 1) * 3 + 1];
      smo_estimator_B.kstr = smo_estimator_B.b_kstr_a + 1;
      smo_estimator_B.R_g[smo_estimator_B.kstr + 5] = smo_estimator_B.tempR
        [(smo_estimator_B.kstr - 1) * 3 + 2];
    }

    memset(&smo_estimator_B.TJ[0], 0, sizeof(real_T) << 4U);
    for (smo_estimator_B.b_kstr_a = 0; smo_estimator_B.b_kstr_a < 3;
         smo_estimator_B.b_kstr_a++) {
      smo_estimator_B.kstr = smo_estimator_B.b_kstr_a << 2;
      smo_estimator_B.TJ[smo_estimator_B.kstr] = smo_estimator_B.R_g[3 *
        smo_estimator_B.b_kstr_a];
      smo_estimator_B.TJ[smo_estimator_B.kstr + 1] = smo_estimator_B.R_g[3 *
        smo_estimator_B.b_kstr_a + 1];
      smo_estimator_B.TJ[smo_estimator_B.kstr + 2] = smo_estimator_B.R_g[3 *
        smo_estimator_B.b_kstr_a + 2];
    }

    smo_estimator_B.TJ[15] = 1.0;
    break;

   default:
    rigidBodyJoint_get_JointAxis_m(obj, smo_estimator_B.v);
    memset(&smo_estimator_B.tempR[0], 0, 9U * sizeof(real_T));
    smo_estimator_B.tempR[0] = 1.0;
    smo_estimator_B.tempR[4] = 1.0;
    smo_estimator_B.tempR[8] = 1.0;
    for (smo_estimator_B.b_kstr_a = 0; smo_estimator_B.b_kstr_a < 3;
         smo_estimator_B.b_kstr_a++) {
      smo_estimator_B.kstr = smo_estimator_B.b_kstr_a << 2;
      smo_estimator_B.TJ[smo_estimator_B.kstr] = smo_estimator_B.tempR[3 *
        smo_estimator_B.b_kstr_a];
      smo_estimator_B.TJ[smo_estimator_B.kstr + 1] = smo_estimator_B.tempR[3 *
        smo_estimator_B.b_kstr_a + 1];
      smo_estimator_B.TJ[smo_estimator_B.kstr + 2] = smo_estimator_B.tempR[3 *
        smo_estimator_B.b_kstr_a + 2];
      smo_estimator_B.TJ[smo_estimator_B.b_kstr_a + 12] =
        smo_estimator_B.v[smo_estimator_B.b_kstr_a] * q_data[0];
    }

    smo_estimator_B.TJ[3] = 0.0;
    smo_estimator_B.TJ[7] = 0.0;
    smo_estimator_B.TJ[11] = 0.0;
    smo_estimator_B.TJ[15] = 1.0;
    break;
  }

  for (smo_estimator_B.b_kstr_a = 0; smo_estimator_B.b_kstr_a < 4;
       smo_estimator_B.b_kstr_a++) {
    for (smo_estimator_B.kstr = 0; smo_estimator_B.kstr < 4;
         smo_estimator_B.kstr++) {
      smo_estimator_B.obj_tmp_tmp = smo_estimator_B.kstr << 2;
      smo_estimator_B.obj_tmp = smo_estimator_B.b_kstr_a +
        smo_estimator_B.obj_tmp_tmp;
      smo_estimator_B.obj[smo_estimator_B.obj_tmp] = 0.0;
      smo_estimator_B.obj[smo_estimator_B.obj_tmp] +=
        smo_estimator_B.TJ[smo_estimator_B.obj_tmp_tmp] *
        obj->JointToParentTransform[smo_estimator_B.b_kstr_a];
      smo_estimator_B.obj[smo_estimator_B.obj_tmp] +=
        smo_estimator_B.TJ[smo_estimator_B.obj_tmp_tmp + 1] *
        obj->JointToParentTransform[smo_estimator_B.b_kstr_a + 4];
      smo_estimator_B.obj[smo_estimator_B.obj_tmp] +=
        smo_estimator_B.TJ[smo_estimator_B.obj_tmp_tmp + 2] *
        obj->JointToParentTransform[smo_estimator_B.b_kstr_a + 8];
      smo_estimator_B.obj[smo_estimator_B.obj_tmp] +=
        smo_estimator_B.TJ[smo_estimator_B.obj_tmp_tmp + 3] *
        obj->JointToParentTransform[smo_estimator_B.b_kstr_a + 12];
    }

    for (smo_estimator_B.kstr = 0; smo_estimator_B.kstr < 4;
         smo_estimator_B.kstr++) {
      smo_estimator_B.obj_tmp_tmp = smo_estimator_B.kstr << 2;
      smo_estimator_B.obj_tmp = smo_estimator_B.b_kstr_a +
        smo_estimator_B.obj_tmp_tmp;
      T[smo_estimator_B.obj_tmp] = 0.0;
      T[smo_estimator_B.obj_tmp] += obj->
        ChildToJointTransform[smo_estimator_B.obj_tmp_tmp] *
        smo_estimator_B.obj[smo_estimator_B.b_kstr_a];
      T[smo_estimator_B.obj_tmp] += obj->
        ChildToJointTransform[smo_estimator_B.obj_tmp_tmp + 1] *
        smo_estimator_B.obj[smo_estimator_B.b_kstr_a + 4];
      T[smo_estimator_B.obj_tmp] += obj->
        ChildToJointTransform[smo_estimator_B.obj_tmp_tmp + 2] *
        smo_estimator_B.obj[smo_estimator_B.b_kstr_a + 8];
      T[smo_estimator_B.obj_tmp] += obj->
        ChildToJointTransform[smo_estimator_B.obj_tmp_tmp + 3] *
        smo_estimator_B.obj[smo_estimator_B.b_kstr_a + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const
  c_rigidBodyJoint_smo_estima_m_T *obj, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (smo_estimator_B.b_kstr_h = 0; smo_estimator_B.b_kstr_h < 5;
       smo_estimator_B.b_kstr_h++) {
    smo_estimator_B.b_mc[smo_estimator_B.b_kstr_h] =
      tmp[smo_estimator_B.b_kstr_h];
  }

  smo_estimator_B.b_bool_j = false;
  if (obj->Type->size[1] == 5) {
    smo_estimator_B.b_kstr_h = 1;
    do {
      exitg1 = 0;
      if (smo_estimator_B.b_kstr_h - 1 < 5) {
        smo_estimator_B.kstr_f = smo_estimator_B.b_kstr_h - 1;
        if (obj->Type->data[smo_estimator_B.kstr_f] !=
            smo_estimator_B.b_mc[smo_estimator_B.kstr_f]) {
          exitg1 = 1;
        } else {
          smo_estimator_B.b_kstr_h++;
        }
      } else {
        smo_estimator_B.b_bool_j = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (smo_estimator_B.b_bool_j) {
    smo_estimator_B.b_kstr_h = 0;
  } else {
    for (smo_estimator_B.b_kstr_h = 0; smo_estimator_B.b_kstr_h < 8;
         smo_estimator_B.b_kstr_h++) {
      smo_estimator_B.b_j[smo_estimator_B.b_kstr_h] =
        tmp_0[smo_estimator_B.b_kstr_h];
    }

    smo_estimator_B.b_bool_j = false;
    if (obj->Type->size[1] == 8) {
      smo_estimator_B.b_kstr_h = 1;
      do {
        exitg1 = 0;
        if (smo_estimator_B.b_kstr_h - 1 < 8) {
          smo_estimator_B.kstr_f = smo_estimator_B.b_kstr_h - 1;
          if (obj->Type->data[smo_estimator_B.kstr_f] !=
              smo_estimator_B.b_j[smo_estimator_B.kstr_f]) {
            exitg1 = 1;
          } else {
            smo_estimator_B.b_kstr_h++;
          }
        } else {
          smo_estimator_B.b_bool_j = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (smo_estimator_B.b_bool_j) {
      smo_estimator_B.b_kstr_h = 1;
    } else {
      smo_estimator_B.b_kstr_h = -1;
    }
  }

  switch (smo_estimator_B.b_kstr_h) {
   case 0:
    memset(&smo_estimator_B.TJ_b[0], 0, sizeof(real_T) << 4U);
    smo_estimator_B.TJ_b[0] = 1.0;
    smo_estimator_B.TJ_b[5] = 1.0;
    smo_estimator_B.TJ_b[10] = 1.0;
    smo_estimator_B.TJ_b[15] = 1.0;
    break;

   case 1:
    rigidBodyJoint_get_JointAxis_m(obj, smo_estimator_B.v_a);
    smo_estimator_B.axang_idx_0 = smo_estimator_B.v_a[0];
    smo_estimator_B.axang_idx_1 = smo_estimator_B.v_a[1];
    smo_estimator_B.axang_idx_2 = smo_estimator_B.v_a[2];
    smo_estimator_B.b_f = 1.0 / sqrt((smo_estimator_B.axang_idx_0 *
      smo_estimator_B.axang_idx_0 + smo_estimator_B.axang_idx_1 *
      smo_estimator_B.axang_idx_1) + smo_estimator_B.axang_idx_2 *
      smo_estimator_B.axang_idx_2);
    smo_estimator_B.v_a[0] = smo_estimator_B.axang_idx_0 * smo_estimator_B.b_f;
    smo_estimator_B.v_a[1] = smo_estimator_B.axang_idx_1 * smo_estimator_B.b_f;
    smo_estimator_B.v_a[2] = smo_estimator_B.axang_idx_2 * smo_estimator_B.b_f;
    smo_estimator_B.axang_idx_0 = smo_estimator_B.v_a[1] * smo_estimator_B.v_a[0]
      * 0.0;
    smo_estimator_B.axang_idx_1 = smo_estimator_B.v_a[2] * smo_estimator_B.v_a[0]
      * 0.0;
    smo_estimator_B.axang_idx_2 = smo_estimator_B.v_a[2] * smo_estimator_B.v_a[1]
      * 0.0;
    smo_estimator_cat(smo_estimator_B.v_a[0] * smo_estimator_B.v_a[0] * 0.0 +
                      1.0, smo_estimator_B.axang_idx_0 - smo_estimator_B.v_a[2] *
                      0.0, smo_estimator_B.axang_idx_1 + smo_estimator_B.v_a[1] *
                      0.0, smo_estimator_B.axang_idx_0 + smo_estimator_B.v_a[2] *
                      0.0, smo_estimator_B.v_a[1] * smo_estimator_B.v_a[1] * 0.0
                      + 1.0, smo_estimator_B.axang_idx_2 - smo_estimator_B.v_a[0]
                      * 0.0, smo_estimator_B.axang_idx_1 - smo_estimator_B.v_a[1]
                      * 0.0, smo_estimator_B.axang_idx_2 + smo_estimator_B.v_a[0]
                      * 0.0, smo_estimator_B.v_a[2] * smo_estimator_B.v_a[2] *
                      0.0 + 1.0, smo_estimator_B.tempR_d);
    for (smo_estimator_B.b_kstr_h = 0; smo_estimator_B.b_kstr_h < 3;
         smo_estimator_B.b_kstr_h++) {
      smo_estimator_B.kstr_f = smo_estimator_B.b_kstr_h + 1;
      smo_estimator_B.R_ld[smo_estimator_B.kstr_f - 1] =
        smo_estimator_B.tempR_d[(smo_estimator_B.kstr_f - 1) * 3];
      smo_estimator_B.kstr_f = smo_estimator_B.b_kstr_h + 1;
      smo_estimator_B.R_ld[smo_estimator_B.kstr_f + 2] =
        smo_estimator_B.tempR_d[(smo_estimator_B.kstr_f - 1) * 3 + 1];
      smo_estimator_B.kstr_f = smo_estimator_B.b_kstr_h + 1;
      smo_estimator_B.R_ld[smo_estimator_B.kstr_f + 5] =
        smo_estimator_B.tempR_d[(smo_estimator_B.kstr_f - 1) * 3 + 2];
    }

    memset(&smo_estimator_B.TJ_b[0], 0, sizeof(real_T) << 4U);
    for (smo_estimator_B.b_kstr_h = 0; smo_estimator_B.b_kstr_h < 3;
         smo_estimator_B.b_kstr_h++) {
      smo_estimator_B.kstr_f = smo_estimator_B.b_kstr_h << 2;
      smo_estimator_B.TJ_b[smo_estimator_B.kstr_f] = smo_estimator_B.R_ld[3 *
        smo_estimator_B.b_kstr_h];
      smo_estimator_B.TJ_b[smo_estimator_B.kstr_f + 1] = smo_estimator_B.R_ld[3 *
        smo_estimator_B.b_kstr_h + 1];
      smo_estimator_B.TJ_b[smo_estimator_B.kstr_f + 2] = smo_estimator_B.R_ld[3 *
        smo_estimator_B.b_kstr_h + 2];
    }

    smo_estimator_B.TJ_b[15] = 1.0;
    break;

   default:
    rigidBodyJoint_get_JointAxis_m(obj, smo_estimator_B.v_a);
    memset(&smo_estimator_B.tempR_d[0], 0, 9U * sizeof(real_T));
    smo_estimator_B.tempR_d[0] = 1.0;
    smo_estimator_B.tempR_d[4] = 1.0;
    smo_estimator_B.tempR_d[8] = 1.0;
    for (smo_estimator_B.b_kstr_h = 0; smo_estimator_B.b_kstr_h < 3;
         smo_estimator_B.b_kstr_h++) {
      smo_estimator_B.kstr_f = smo_estimator_B.b_kstr_h << 2;
      smo_estimator_B.TJ_b[smo_estimator_B.kstr_f] = smo_estimator_B.tempR_d[3 *
        smo_estimator_B.b_kstr_h];
      smo_estimator_B.TJ_b[smo_estimator_B.kstr_f + 1] =
        smo_estimator_B.tempR_d[3 * smo_estimator_B.b_kstr_h + 1];
      smo_estimator_B.TJ_b[smo_estimator_B.kstr_f + 2] =
        smo_estimator_B.tempR_d[3 * smo_estimator_B.b_kstr_h + 2];
      smo_estimator_B.TJ_b[smo_estimator_B.b_kstr_h + 12] =
        smo_estimator_B.v_a[smo_estimator_B.b_kstr_h] * 0.0;
    }

    smo_estimator_B.TJ_b[3] = 0.0;
    smo_estimator_B.TJ_b[7] = 0.0;
    smo_estimator_B.TJ_b[11] = 0.0;
    smo_estimator_B.TJ_b[15] = 1.0;
    break;
  }

  for (smo_estimator_B.b_kstr_h = 0; smo_estimator_B.b_kstr_h < 4;
       smo_estimator_B.b_kstr_h++) {
    for (smo_estimator_B.kstr_f = 0; smo_estimator_B.kstr_f < 4;
         smo_estimator_B.kstr_f++) {
      smo_estimator_B.obj_tmp_tmp_c = smo_estimator_B.kstr_f << 2;
      smo_estimator_B.obj_tmp_e = smo_estimator_B.b_kstr_h +
        smo_estimator_B.obj_tmp_tmp_c;
      smo_estimator_B.obj_p[smo_estimator_B.obj_tmp_e] = 0.0;
      smo_estimator_B.obj_p[smo_estimator_B.obj_tmp_e] +=
        smo_estimator_B.TJ_b[smo_estimator_B.obj_tmp_tmp_c] *
        obj->JointToParentTransform[smo_estimator_B.b_kstr_h];
      smo_estimator_B.obj_p[smo_estimator_B.obj_tmp_e] +=
        smo_estimator_B.TJ_b[smo_estimator_B.obj_tmp_tmp_c + 1] *
        obj->JointToParentTransform[smo_estimator_B.b_kstr_h + 4];
      smo_estimator_B.obj_p[smo_estimator_B.obj_tmp_e] +=
        smo_estimator_B.TJ_b[smo_estimator_B.obj_tmp_tmp_c + 2] *
        obj->JointToParentTransform[smo_estimator_B.b_kstr_h + 8];
      smo_estimator_B.obj_p[smo_estimator_B.obj_tmp_e] +=
        smo_estimator_B.TJ_b[smo_estimator_B.obj_tmp_tmp_c + 3] *
        obj->JointToParentTransform[smo_estimator_B.b_kstr_h + 12];
    }

    for (smo_estimator_B.kstr_f = 0; smo_estimator_B.kstr_f < 4;
         smo_estimator_B.kstr_f++) {
      smo_estimator_B.obj_tmp_tmp_c = smo_estimator_B.kstr_f << 2;
      smo_estimator_B.obj_tmp_e = smo_estimator_B.b_kstr_h +
        smo_estimator_B.obj_tmp_tmp_c;
      T[smo_estimator_B.obj_tmp_e] = 0.0;
      T[smo_estimator_B.obj_tmp_e] += obj->
        ChildToJointTransform[smo_estimator_B.obj_tmp_tmp_c] *
        smo_estimator_B.obj_p[smo_estimator_B.b_kstr_h];
      T[smo_estimator_B.obj_tmp_e] += obj->
        ChildToJointTransform[smo_estimator_B.obj_tmp_tmp_c + 1] *
        smo_estimator_B.obj_p[smo_estimator_B.b_kstr_h + 4];
      T[smo_estimator_B.obj_tmp_e] += obj->
        ChildToJointTransform[smo_estimator_B.obj_tmp_tmp_c + 2] *
        smo_estimator_B.obj_p[smo_estimator_B.b_kstr_h + 8];
      T[smo_estimator_B.obj_tmp_e] += obj->
        ChildToJointTransform[smo_estimator_B.obj_tmp_tmp_c + 3] *
        smo_estimator_B.obj_p[smo_estimator_B.b_kstr_h + 12];
    }
  }
}

static void smo_estimator_tforminv(const real_T T[16], real_T Tinv[16])
{
  for (smo_estimator_B.i3 = 0; smo_estimator_B.i3 < 3; smo_estimator_B.i3++) {
    smo_estimator_B.R_ln[3 * smo_estimator_B.i3] = T[smo_estimator_B.i3];
    smo_estimator_B.R_ln[3 * smo_estimator_B.i3 + 1] = T[smo_estimator_B.i3 + 4];
    smo_estimator_B.R_ln[3 * smo_estimator_B.i3 + 2] = T[smo_estimator_B.i3 + 8];
  }

  for (smo_estimator_B.i3 = 0; smo_estimator_B.i3 < 9; smo_estimator_B.i3++) {
    smo_estimator_B.R_h[smo_estimator_B.i3] =
      -smo_estimator_B.R_ln[smo_estimator_B.i3];
  }

  for (smo_estimator_B.i3 = 0; smo_estimator_B.i3 < 3; smo_estimator_B.i3++) {
    smo_estimator_B.Tinv_tmp = smo_estimator_B.i3 << 2;
    Tinv[smo_estimator_B.Tinv_tmp] = smo_estimator_B.R_ln[3 * smo_estimator_B.i3];
    Tinv[smo_estimator_B.Tinv_tmp + 1] = smo_estimator_B.R_ln[3 *
      smo_estimator_B.i3 + 1];
    Tinv[smo_estimator_B.Tinv_tmp + 2] = smo_estimator_B.R_ln[3 *
      smo_estimator_B.i3 + 2];
    Tinv[smo_estimator_B.i3 + 12] = smo_estimator_B.R_h[smo_estimator_B.i3 + 6] *
      T[14] + (smo_estimator_B.R_h[smo_estimator_B.i3 + 3] * T[13] +
               smo_estimator_B.R_h[smo_estimator_B.i3] * T[12]);
  }

  Tinv[3] = 0.0;
  Tinv[7] = 0.0;
  Tinv[11] = 0.0;
  Tinv[15] = 1.0;
}

static void smo_estimat_tformToSpatialXform(const real_T T[16], real_T X[36])
{
  smo_estimator_B.dv3[0] = 0.0;
  smo_estimator_B.dv3[3] = -T[14];
  smo_estimator_B.dv3[6] = T[13];
  smo_estimator_B.dv3[1] = T[14];
  smo_estimator_B.dv3[4] = 0.0;
  smo_estimator_B.dv3[7] = -T[12];
  smo_estimator_B.dv3[2] = -T[13];
  smo_estimator_B.dv3[5] = T[12];
  smo_estimator_B.dv3[8] = 0.0;
  for (smo_estimator_B.i1 = 0; smo_estimator_B.i1 < 3; smo_estimator_B.i1++) {
    for (smo_estimator_B.X_tmp_m = 0; smo_estimator_B.X_tmp_m < 3;
         smo_estimator_B.X_tmp_m++) {
      smo_estimator_B.X_tmp_c = smo_estimator_B.i1 + 3 * smo_estimator_B.X_tmp_m;
      smo_estimator_B.dv4[smo_estimator_B.X_tmp_c] = 0.0;
      smo_estimator_B.i2 = smo_estimator_B.X_tmp_m << 2;
      smo_estimator_B.dv4[smo_estimator_B.X_tmp_c] += T[smo_estimator_B.i2] *
        smo_estimator_B.dv3[smo_estimator_B.i1];
      smo_estimator_B.dv4[smo_estimator_B.X_tmp_c] += T[smo_estimator_B.i2 + 1] *
        smo_estimator_B.dv3[smo_estimator_B.i1 + 3];
      smo_estimator_B.dv4[smo_estimator_B.X_tmp_c] += T[smo_estimator_B.i2 + 2] *
        smo_estimator_B.dv3[smo_estimator_B.i1 + 6];
      X[smo_estimator_B.X_tmp_m + 6 * smo_estimator_B.i1] = T
        [(smo_estimator_B.i1 << 2) + smo_estimator_B.X_tmp_m];
      X[smo_estimator_B.X_tmp_m + 6 * (smo_estimator_B.i1 + 3)] = 0.0;
    }
  }

  for (smo_estimator_B.i1 = 0; smo_estimator_B.i1 < 3; smo_estimator_B.i1++) {
    X[6 * smo_estimator_B.i1 + 3] = smo_estimator_B.dv4[3 * smo_estimator_B.i1];
    smo_estimator_B.X_tmp_m = smo_estimator_B.i1 << 2;
    smo_estimator_B.X_tmp_c = 6 * (smo_estimator_B.i1 + 3);
    X[smo_estimator_B.X_tmp_c + 3] = T[smo_estimator_B.X_tmp_m];
    X[6 * smo_estimator_B.i1 + 4] = smo_estimator_B.dv4[3 * smo_estimator_B.i1 +
      1];
    X[smo_estimator_B.X_tmp_c + 4] = T[smo_estimator_B.X_tmp_m + 1];
    X[6 * smo_estimator_B.i1 + 5] = smo_estimator_B.dv4[3 * smo_estimator_B.i1 +
      2];
    X[smo_estimator_B.X_tmp_c + 5] = T[smo_estimator_B.X_tmp_m + 2];
  }
}

static void smo_estimator_emxFree_real_T(emxArray_real_T_smo_estimator_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_smo_estimator_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_smo_estimator_T *)NULL;
  }
}

static void smo_estima_emxFree_f_cell_wrap1(emxArray_f_cell_wrap_smo_es_m_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_smo_es_m_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_smo_estimator_m_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_smo_es_m_T *)NULL;
  }
}

static void RigidBodyTreeDynamics_massMatri(q_robotics_manip_internal_R_m_T
  *robot, const real_T q[7], emxArray_real_T_smo_estimator_T *H)
{
  emxArray_f_cell_wrap_smo_es_m_T *Ic;
  emxArray_f_cell_wrap_smo_es_m_T *X;
  emxArray_real_T_smo_estimator_T *Si;
  emxArray_real_T_smo_estimator_T *Fi;
  emxArray_real_T_smo_estimator_T *Hji;
  o_robotics_manip_internal_R_m_T *obj;
  smo_estimator_B.nb_lm = robot->NumBodies;
  smo_estimator_B.vNum_m = robot->VelocityNumber;
  smo_estimator_B.f_m = H->size[0] * H->size[1];
  smo_estimator_B.c_tmp = static_cast<int32_T>(smo_estimator_B.vNum_m);
  H->size[0] = smo_estimator_B.c_tmp;
  H->size[1] = smo_estimator_B.c_tmp;
  smo_es_emxEnsureCapacity_real_T(H, smo_estimator_B.f_m);
  smo_estimator_B.loop_ub_o = smo_estimator_B.c_tmp * smo_estimator_B.c_tmp - 1;
  for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <= smo_estimator_B.loop_ub_o;
       smo_estimator_B.f_m++) {
    H->data[smo_estimator_B.f_m] = 0.0;
  }

  smo_estima_emxInit_f_cell_wrap1(&Ic, 2);
  smo_estima_emxInit_f_cell_wrap1(&X, 2);
  smo_estimator_B.c_tmp = static_cast<int32_T>(smo_estimator_B.nb_lm);
  smo_estimator_B.c = smo_estimator_B.c_tmp - 1;
  smo_estimator_B.f_m = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = smo_estimator_B.c_tmp;
  emxEnsureCapacity_f_cell_wrap1(Ic, smo_estimator_B.f_m);
  smo_estimator_B.f_m = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = smo_estimator_B.c_tmp;
  emxEnsureCapacity_f_cell_wrap1(X, smo_estimator_B.f_m);
  for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp <= smo_estimator_B.c;
       smo_estimator_B.c_tmp++) {
    for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m < 36; smo_estimator_B.f_m
         ++) {
      Ic->data[smo_estimator_B.c_tmp].f1[smo_estimator_B.f_m] = robot->
        Bodies[smo_estimator_B.c_tmp]->SpatialInertia[smo_estimator_B.f_m];
    }

    smo_estimator_B.vNum_m = robot->PositionDoFMap[smo_estimator_B.c_tmp];
    smo_estimator_B.p_idx_1_f = robot->PositionDoFMap[smo_estimator_B.c_tmp + 10];
    if (smo_estimator_B.p_idx_1_f < smo_estimator_B.vNum_m) {
      obj = robot->Bodies[smo_estimator_B.c_tmp];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, smo_estimator_B.T_f);
    } else {
      if (smo_estimator_B.vNum_m > smo_estimator_B.p_idx_1_f) {
        smo_estimator_B.g = 0;
        smo_estimator_B.f_m = -1;
      } else {
        smo_estimator_B.g = static_cast<int32_T>(smo_estimator_B.vNum_m) - 1;
        smo_estimator_B.f_m = static_cast<int32_T>(smo_estimator_B.p_idx_1_f) -
          1;
      }

      obj = robot->Bodies[smo_estimator_B.c_tmp];
      smo_estimator_B.loop_ub_o = smo_estimator_B.f_m - smo_estimator_B.g;
      smo_estimator_B.q_size_k = smo_estimator_B.loop_ub_o + 1;
      for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <=
           smo_estimator_B.loop_ub_o; smo_estimator_B.f_m++) {
        smo_estimator_B.q_data_d[smo_estimator_B.f_m] = q[smo_estimator_B.g +
          smo_estimator_B.f_m];
      }

      rigidBodyJoint_transformBodyT_m(&obj->JointInternal,
        smo_estimator_B.q_data_d, &smo_estimator_B.q_size_k, smo_estimator_B.T_f);
    }

    smo_estimator_tforminv(smo_estimator_B.T_f, smo_estimator_B.dv1);
    smo_estimat_tformToSpatialXform(smo_estimator_B.dv1, X->
      data[smo_estimator_B.c_tmp].f1);
  }

  smo_estimator_B.g = static_cast<int32_T>(((-1.0 - smo_estimator_B.nb_lm) + 1.0)
    / -1.0) - 1;
  smo_estimator_emxInit_real_T(&Si, 2);
  smo_estimator_emxInit_real_T(&Fi, 2);
  smo_estimator_emxInit_real_T(&Hji, 2);
  for (smo_estimator_B.c = 0; smo_estimator_B.c <= smo_estimator_B.g;
       smo_estimator_B.c++) {
    smo_estimator_B.n_n = static_cast<int32_T>(smo_estimator_B.nb_lm + -
      static_cast<real_T>(smo_estimator_B.c));
    smo_estimator_B.pid_tmp_i = smo_estimator_B.n_n - 1;
    smo_estimator_B.pid_m = robot->Bodies[smo_estimator_B.pid_tmp_i]
      ->ParentIndex;
    smo_estimator_B.vNum_m = robot->VelocityDoFMap[smo_estimator_B.n_n - 1];
    smo_estimator_B.p_idx_1_f = robot->VelocityDoFMap[smo_estimator_B.n_n + 9];
    if (smo_estimator_B.pid_m > 0.0) {
      for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m < 6; smo_estimator_B.f_m
           ++) {
        for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp < 6;
             smo_estimator_B.c_tmp++) {
          smo_estimator_B.X_tmp_o = smo_estimator_B.f_m + 6 *
            smo_estimator_B.c_tmp;
          smo_estimator_B.X_c[smo_estimator_B.X_tmp_o] = 0.0;
          for (smo_estimator_B.loop_ub_o = 0; smo_estimator_B.loop_ub_o < 6;
               smo_estimator_B.loop_ub_o++) {
            smo_estimator_B.X_c[smo_estimator_B.X_tmp_o] += X->
              data[smo_estimator_B.pid_tmp_i].f1[6 * smo_estimator_B.f_m +
              smo_estimator_B.loop_ub_o] * Ic->data[smo_estimator_B.pid_tmp_i].
              f1[6 * smo_estimator_B.c_tmp + smo_estimator_B.loop_ub_o];
          }
        }
      }

      for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m < 6; smo_estimator_B.f_m
           ++) {
        for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp < 6;
             smo_estimator_B.c_tmp++) {
          smo_estimator_B.b_idx_0_p = 0.0;
          for (smo_estimator_B.loop_ub_o = 0; smo_estimator_B.loop_ub_o < 6;
               smo_estimator_B.loop_ub_o++) {
            smo_estimator_B.b_idx_0_p += smo_estimator_B.X_c[6 *
              smo_estimator_B.loop_ub_o + smo_estimator_B.f_m] * X->
              data[smo_estimator_B.pid_tmp_i].f1[6 * smo_estimator_B.c_tmp +
              smo_estimator_B.loop_ub_o];
          }

          smo_estimator_B.loop_ub_o = 6 * smo_estimator_B.c_tmp +
            smo_estimator_B.f_m;
          Ic->data[static_cast<int32_T>(smo_estimator_B.pid_m) - 1]
            .f1[smo_estimator_B.loop_ub_o] += smo_estimator_B.b_idx_0_p;
        }
      }
    }

    smo_estimator_B.b_idx_0_p = robot->VelocityDoFMap[smo_estimator_B.n_n - 1];
    smo_estimator_B.b_idx_1_e = robot->VelocityDoFMap[smo_estimator_B.n_n + 9];
    if (smo_estimator_B.b_idx_0_p <= smo_estimator_B.b_idx_1_e) {
      obj = robot->Bodies[smo_estimator_B.pid_tmp_i];
      smo_estimator_B.f_m = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      smo_es_emxEnsureCapacity_real_T(Si, smo_estimator_B.f_m);
      smo_estimator_B.loop_ub_o = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <=
           smo_estimator_B.loop_ub_o; smo_estimator_B.f_m++) {
        Si->data[smo_estimator_B.f_m] = obj->JointInternal.MotionSubspace->
          data[smo_estimator_B.f_m];
      }

      smo_estimator_B.n_n = Si->size[1] - 1;
      smo_estimator_B.f_m = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      smo_es_emxEnsureCapacity_real_T(Fi, smo_estimator_B.f_m);
      for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <= smo_estimator_B.n_n;
           smo_estimator_B.f_m++) {
        smo_estimator_B.X_tmp_o = smo_estimator_B.f_m * 6 - 1;
        for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp < 6;
             smo_estimator_B.c_tmp++) {
          smo_estimator_B.s_c = 0.0;
          for (smo_estimator_B.loop_ub_o = 0; smo_estimator_B.loop_ub_o < 6;
               smo_estimator_B.loop_ub_o++) {
            smo_estimator_B.s_c += Ic->data[smo_estimator_B.pid_tmp_i]
              .f1[smo_estimator_B.loop_ub_o * 6 + smo_estimator_B.c_tmp] *
              Si->data[(smo_estimator_B.X_tmp_o + smo_estimator_B.loop_ub_o) + 1];
          }

          Fi->data[(smo_estimator_B.X_tmp_o + smo_estimator_B.c_tmp) + 1] =
            smo_estimator_B.s_c;
        }
      }

      if (smo_estimator_B.vNum_m > smo_estimator_B.p_idx_1_f) {
        smo_estimator_B.X_tmp_o = 0;
        smo_estimator_B.cb_o = 0;
      } else {
        smo_estimator_B.X_tmp_o = static_cast<int32_T>(smo_estimator_B.vNum_m) -
          1;
        smo_estimator_B.cb_o = smo_estimator_B.X_tmp_o;
      }

      smo_estimator_B.m_l = Si->size[1];
      smo_estimator_B.n_n = Fi->size[1] - 1;
      smo_estimator_B.f_m = Hji->size[0] * Hji->size[1];
      Hji->size[0] = Si->size[1];
      Hji->size[1] = Fi->size[1];
      smo_es_emxEnsureCapacity_real_T(Hji, smo_estimator_B.f_m);
      for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <= smo_estimator_B.n_n;
           smo_estimator_B.f_m++) {
        smo_estimator_B.coffset_p = smo_estimator_B.f_m * smo_estimator_B.m_l -
          1;
        smo_estimator_B.boffset_p = smo_estimator_B.f_m * 6 - 1;
        for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp <
             smo_estimator_B.m_l; smo_estimator_B.c_tmp++) {
          smo_estimator_B.aoffset_f = smo_estimator_B.c_tmp * 6 - 1;
          smo_estimator_B.s_c = 0.0;
          for (smo_estimator_B.loop_ub_o = 0; smo_estimator_B.loop_ub_o < 6;
               smo_estimator_B.loop_ub_o++) {
            smo_estimator_B.k_i = smo_estimator_B.loop_ub_o + 1;
            smo_estimator_B.s_c += Si->data[smo_estimator_B.aoffset_f +
              smo_estimator_B.k_i] * Fi->data[smo_estimator_B.boffset_p +
              smo_estimator_B.k_i];
          }

          Hji->data[(smo_estimator_B.coffset_p + smo_estimator_B.c_tmp) + 1] =
            smo_estimator_B.s_c;
        }
      }

      smo_estimator_B.loop_ub_o = Hji->size[1];
      for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <
           smo_estimator_B.loop_ub_o; smo_estimator_B.f_m++) {
        smo_estimator_B.n_n = Hji->size[0];
        for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp <
             smo_estimator_B.n_n; smo_estimator_B.c_tmp++) {
          H->data[(smo_estimator_B.X_tmp_o + smo_estimator_B.c_tmp) + H->size[0]
            * (smo_estimator_B.cb_o + smo_estimator_B.f_m)] = Hji->data
            [Hji->size[0] * smo_estimator_B.f_m + smo_estimator_B.c_tmp];
        }
      }

      smo_estimator_B.n_n = Fi->size[1];
      smo_estimator_B.f_m = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = Fi->size[1];
      smo_es_emxEnsureCapacity_real_T(Si, smo_estimator_B.f_m);
      smo_estimator_B.loop_ub_o = Fi->size[0] * Fi->size[1] - 1;
      for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <=
           smo_estimator_B.loop_ub_o; smo_estimator_B.f_m++) {
        Si->data[smo_estimator_B.f_m] = Fi->data[smo_estimator_B.f_m];
      }

      smo_estimator_B.f_m = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = smo_estimator_B.n_n;
      smo_es_emxEnsureCapacity_real_T(Fi, smo_estimator_B.f_m);
      for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m < smo_estimator_B.n_n;
           smo_estimator_B.f_m++) {
        smo_estimator_B.X_tmp_o = smo_estimator_B.f_m * 6 - 1;
        for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp < 6;
             smo_estimator_B.c_tmp++) {
          smo_estimator_B.aoffset_f = smo_estimator_B.c_tmp * 6 - 1;
          smo_estimator_B.s_c = 0.0;
          for (smo_estimator_B.loop_ub_o = 0; smo_estimator_B.loop_ub_o < 6;
               smo_estimator_B.loop_ub_o++) {
            smo_estimator_B.k_i = smo_estimator_B.loop_ub_o + 1;
            smo_estimator_B.s_c += X->data[smo_estimator_B.pid_tmp_i]
              .f1[smo_estimator_B.aoffset_f + smo_estimator_B.k_i] * Si->
              data[smo_estimator_B.X_tmp_o + smo_estimator_B.k_i];
          }

          Fi->data[(smo_estimator_B.X_tmp_o + smo_estimator_B.c_tmp) + 1] =
            smo_estimator_B.s_c;
        }
      }

      while (smo_estimator_B.pid_m > 0.0) {
        smo_estimator_B.c_tmp = static_cast<int32_T>(smo_estimator_B.pid_m);
        smo_estimator_B.pid_tmp_i = smo_estimator_B.c_tmp - 1;
        obj = robot->Bodies[smo_estimator_B.pid_tmp_i];
        smo_estimator_B.f_m = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
        smo_es_emxEnsureCapacity_real_T(Si, smo_estimator_B.f_m);
        smo_estimator_B.loop_ub_o = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <=
             smo_estimator_B.loop_ub_o; smo_estimator_B.f_m++) {
          Si->data[smo_estimator_B.f_m] = obj->
            JointInternal.MotionSubspace->data[smo_estimator_B.f_m];
        }

        smo_estimator_B.b_idx_0_p = robot->VelocityDoFMap[smo_estimator_B.c_tmp
          - 1];
        smo_estimator_B.b_idx_1_e = robot->VelocityDoFMap[smo_estimator_B.c_tmp
          + 9];
        if (smo_estimator_B.b_idx_0_p <= smo_estimator_B.b_idx_1_e) {
          smo_estimator_B.m_l = Si->size[1];
          smo_estimator_B.n_n = Fi->size[1] - 1;
          smo_estimator_B.f_m = Hji->size[0] * Hji->size[1];
          Hji->size[0] = Si->size[1];
          Hji->size[1] = Fi->size[1];
          smo_es_emxEnsureCapacity_real_T(Hji, smo_estimator_B.f_m);
          for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <=
               smo_estimator_B.n_n; smo_estimator_B.f_m++) {
            smo_estimator_B.coffset_p = smo_estimator_B.f_m *
              smo_estimator_B.m_l - 1;
            smo_estimator_B.boffset_p = smo_estimator_B.f_m * 6 - 1;
            for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp <
                 smo_estimator_B.m_l; smo_estimator_B.c_tmp++) {
              smo_estimator_B.aoffset_f = smo_estimator_B.c_tmp * 6 - 1;
              smo_estimator_B.s_c = 0.0;
              for (smo_estimator_B.loop_ub_o = 0; smo_estimator_B.loop_ub_o < 6;
                   smo_estimator_B.loop_ub_o++) {
                smo_estimator_B.k_i = smo_estimator_B.loop_ub_o + 1;
                smo_estimator_B.s_c += Si->data[smo_estimator_B.aoffset_f +
                  smo_estimator_B.k_i] * Fi->data[smo_estimator_B.boffset_p +
                  smo_estimator_B.k_i];
              }

              Hji->data[(smo_estimator_B.coffset_p + smo_estimator_B.c_tmp) + 1]
                = smo_estimator_B.s_c;
            }
          }

          if (smo_estimator_B.b_idx_0_p > smo_estimator_B.b_idx_1_e) {
            smo_estimator_B.X_tmp_o = 0;
          } else {
            smo_estimator_B.X_tmp_o = static_cast<int32_T>
              (smo_estimator_B.b_idx_0_p) - 1;
          }

          if (smo_estimator_B.vNum_m > smo_estimator_B.p_idx_1_f) {
            smo_estimator_B.cb_o = 0;
          } else {
            smo_estimator_B.cb_o = static_cast<int32_T>(smo_estimator_B.vNum_m)
              - 1;
          }

          smo_estimator_B.loop_ub_o = Hji->size[1];
          for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <
               smo_estimator_B.loop_ub_o; smo_estimator_B.f_m++) {
            smo_estimator_B.n_n = Hji->size[0];
            for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp <
                 smo_estimator_B.n_n; smo_estimator_B.c_tmp++) {
              H->data[(smo_estimator_B.X_tmp_o + smo_estimator_B.c_tmp) +
                H->size[0] * (smo_estimator_B.cb_o + smo_estimator_B.f_m)] =
                Hji->data[Hji->size[0] * smo_estimator_B.f_m +
                smo_estimator_B.c_tmp];
            }
          }

          if (smo_estimator_B.vNum_m > smo_estimator_B.p_idx_1_f) {
            smo_estimator_B.X_tmp_o = 0;
          } else {
            smo_estimator_B.X_tmp_o = static_cast<int32_T>
              (smo_estimator_B.vNum_m) - 1;
          }

          if (smo_estimator_B.b_idx_0_p > smo_estimator_B.b_idx_1_e) {
            smo_estimator_B.cb_o = 0;
          } else {
            smo_estimator_B.cb_o = static_cast<int32_T>
              (smo_estimator_B.b_idx_0_p) - 1;
          }

          smo_estimator_B.loop_ub_o = Hji->size[0];
          for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <
               smo_estimator_B.loop_ub_o; smo_estimator_B.f_m++) {
            smo_estimator_B.n_n = Hji->size[1];
            for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp <
                 smo_estimator_B.n_n; smo_estimator_B.c_tmp++) {
              H->data[(smo_estimator_B.X_tmp_o + smo_estimator_B.c_tmp) +
                H->size[0] * (smo_estimator_B.cb_o + smo_estimator_B.f_m)] =
                Hji->data[Hji->size[0] * smo_estimator_B.c_tmp +
                smo_estimator_B.f_m];
            }
          }
        }

        smo_estimator_B.n_n = Fi->size[1];
        smo_estimator_B.f_m = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = Fi->size[1];
        smo_es_emxEnsureCapacity_real_T(Si, smo_estimator_B.f_m);
        smo_estimator_B.loop_ub_o = Fi->size[0] * Fi->size[1] - 1;
        for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m <=
             smo_estimator_B.loop_ub_o; smo_estimator_B.f_m++) {
          Si->data[smo_estimator_B.f_m] = Fi->data[smo_estimator_B.f_m];
        }

        smo_estimator_B.f_m = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = smo_estimator_B.n_n;
        smo_es_emxEnsureCapacity_real_T(Fi, smo_estimator_B.f_m);
        for (smo_estimator_B.f_m = 0; smo_estimator_B.f_m < smo_estimator_B.n_n;
             smo_estimator_B.f_m++) {
          smo_estimator_B.X_tmp_o = smo_estimator_B.f_m * 6 - 1;
          for (smo_estimator_B.c_tmp = 0; smo_estimator_B.c_tmp < 6;
               smo_estimator_B.c_tmp++) {
            smo_estimator_B.aoffset_f = smo_estimator_B.c_tmp * 6 - 1;
            smo_estimator_B.s_c = 0.0;
            for (smo_estimator_B.loop_ub_o = 0; smo_estimator_B.loop_ub_o < 6;
                 smo_estimator_B.loop_ub_o++) {
              smo_estimator_B.k_i = smo_estimator_B.loop_ub_o + 1;
              smo_estimator_B.s_c += X->data[smo_estimator_B.pid_tmp_i]
                .f1[smo_estimator_B.aoffset_f + smo_estimator_B.k_i] * Si->
                data[smo_estimator_B.X_tmp_o + smo_estimator_B.k_i];
            }

            Fi->data[(smo_estimator_B.X_tmp_o + smo_estimator_B.c_tmp) + 1] =
              smo_estimator_B.s_c;
          }
        }

        smo_estimator_B.pid_m = robot->Bodies[smo_estimator_B.pid_tmp_i]
          ->ParentIndex;
      }
    }
  }

  smo_estimator_emxFree_real_T(&Hji);
  smo_estimator_emxFree_real_T(&Fi);
  smo_estimator_emxFree_real_T(&Si);
  smo_estima_emxFree_f_cell_wrap1(&X);
  smo_estima_emxFree_f_cell_wrap1(&Ic);
}

static void smo_estimat_emxInit_f_cell_wrap(emxArray_f_cell_wrap_smo_esti_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_smo_esti_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_cell_wrap_smo_esti_T *)malloc(sizeof
    (emxArray_f_cell_wrap_smo_esti_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_smo_estimator_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void smo_estimator_emxInit_char_T(emxArray_char_T_smo_estimator_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_smo_estimator_T *emxArray;
  *pEmxArray = (emxArray_char_T_smo_estimator_T *)malloc(sizeof
    (emxArray_char_T_smo_estimator_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (smo_estimator_B.i_pc = 0; smo_estimator_B.i_pc < numDimensions;
       smo_estimator_B.i_pc++) {
    emxArray->size[smo_estimator_B.i_pc] = 0;
  }
}

static void s_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_smo_esti_T
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
                     (f_cell_wrap_smo_estimator_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_smo_estimator_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_smo_estimator_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void smo_es_emxEnsureCapacity_char_T(emxArray_char_T_smo_estimator_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  smo_estimator_B.newNumel_h = 1;
  for (smo_estimator_B.i_m = 0; smo_estimator_B.i_m < emxArray->numDimensions;
       smo_estimator_B.i_m++) {
    smo_estimator_B.newNumel_h *= emxArray->size[smo_estimator_B.i_m];
  }

  if (smo_estimator_B.newNumel_h > emxArray->allocatedSize) {
    smo_estimator_B.i_m = emxArray->allocatedSize;
    if (smo_estimator_B.i_m < 16) {
      smo_estimator_B.i_m = 16;
    }

    while (smo_estimator_B.i_m < smo_estimator_B.newNumel_h) {
      if (smo_estimator_B.i_m > 1073741823) {
        smo_estimator_B.i_m = MAX_int32_T;
      } else {
        smo_estimator_B.i_m <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(smo_estimator_B.i_m), sizeof(char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = smo_estimator_B.i_m;
    emxArray->canFreeData = true;
  }
}

static void sm_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_smo_estimato_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (smo_estimator_B.b_kstr_p = 0; smo_estimator_B.b_kstr_p < 8;
       smo_estimator_B.b_kstr_p++) {
    smo_estimator_B.b_c0[smo_estimator_B.b_kstr_p] =
      tmp[smo_estimator_B.b_kstr_p];
  }

  smo_estimator_B.b_bool_a = false;
  if (obj->Type->size[1] == 8) {
    smo_estimator_B.b_kstr_p = 1;
    do {
      exitg1 = 0;
      if (smo_estimator_B.b_kstr_p - 1 < 8) {
        smo_estimator_B.kstr_a = smo_estimator_B.b_kstr_p - 1;
        if (obj->Type->data[smo_estimator_B.kstr_a] !=
            smo_estimator_B.b_c0[smo_estimator_B.kstr_a]) {
          exitg1 = 1;
        } else {
          smo_estimator_B.b_kstr_p++;
        }
      } else {
        smo_estimator_B.b_bool_a = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (smo_estimator_B.b_bool_a) {
    guard1 = true;
  } else {
    for (smo_estimator_B.b_kstr_p = 0; smo_estimator_B.b_kstr_p < 9;
         smo_estimator_B.b_kstr_p++) {
      smo_estimator_B.b_c[smo_estimator_B.b_kstr_p] =
        tmp_0[smo_estimator_B.b_kstr_p];
    }

    smo_estimator_B.b_bool_a = false;
    if (obj->Type->size[1] == 9) {
      smo_estimator_B.b_kstr_p = 1;
      do {
        exitg1 = 0;
        if (smo_estimator_B.b_kstr_p - 1 < 9) {
          smo_estimator_B.kstr_a = smo_estimator_B.b_kstr_p - 1;
          if (obj->Type->data[smo_estimator_B.kstr_a] !=
              smo_estimator_B.b_c[smo_estimator_B.kstr_a]) {
            exitg1 = 1;
          } else {
            smo_estimator_B.b_kstr_p++;
          }
        } else {
          smo_estimator_B.b_bool_a = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (smo_estimator_B.b_bool_a) {
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

static void smo_estimator_emxFree_char_T(emxArray_char_T_smo_estimator_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_smo_estimator_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_smo_estimator_T *)NULL;
  }
}

static void RigidBodyTree_forwardKinematics(q_robotics_manip_internal_Rig_T *obj,
  const real_T qvec[7], emxArray_f_cell_wrap_smo_esti_T *Ttree)
{
  o_robotics_manip_internal_Rig_T *body;
  emxArray_char_T_smo_estimator_T *switch_expression;
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  smo_estimator_B.n = obj->NumBodies;
  for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 16;
       smo_estimator_B.b_kstr_ax++) {
    smo_estimator_B.c_f1[smo_estimator_B.b_kstr_ax] =
      tmp[smo_estimator_B.b_kstr_ax];
  }

  smo_estimator_B.b_kstr_ax = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  smo_estimator_B.e = static_cast<int32_T>(smo_estimator_B.n);
  Ttree->size[1] = smo_estimator_B.e;
  s_emxEnsureCapacity_f_cell_wrap(Ttree, smo_estimator_B.b_kstr_ax);
  if (smo_estimator_B.e != 0) {
    smo_estimator_B.ntilecols = smo_estimator_B.e - 1;
    if (0 <= smo_estimator_B.ntilecols) {
      memcpy(&smo_estimator_B.expl_temp.f1[0], &smo_estimator_B.c_f1[0], sizeof
             (real_T) << 4U);
    }

    for (smo_estimator_B.b_jtilecol = 0; smo_estimator_B.b_jtilecol <=
         smo_estimator_B.ntilecols; smo_estimator_B.b_jtilecol++) {
      Ttree->data[smo_estimator_B.b_jtilecol] = smo_estimator_B.expl_temp;
    }
  }

  smo_estimator_B.k = 1.0;
  smo_estimator_B.ntilecols = static_cast<int32_T>(smo_estimator_B.n) - 1;
  smo_estimator_emxInit_char_T(&switch_expression, 2);
  if (0 <= smo_estimator_B.ntilecols) {
    for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 5;
         smo_estimator_B.b_kstr_ax++) {
      smo_estimator_B.b_h3[smo_estimator_B.b_kstr_ax] =
        tmp_0[smo_estimator_B.b_kstr_ax];
    }
  }

  for (smo_estimator_B.b_jtilecol = 0; smo_estimator_B.b_jtilecol <=
       smo_estimator_B.ntilecols; smo_estimator_B.b_jtilecol++) {
    body = obj->Bodies[smo_estimator_B.b_jtilecol];
    smo_estimator_B.n = body->JointInternal.PositionNumber;
    smo_estimator_B.n += smo_estimator_B.k;
    if (smo_estimator_B.k > smo_estimator_B.n - 1.0) {
      smo_estimator_B.e = 0;
      smo_estimator_B.d = 0;
    } else {
      smo_estimator_B.e = static_cast<int32_T>(smo_estimator_B.k) - 1;
      smo_estimator_B.d = static_cast<int32_T>(smo_estimator_B.n - 1.0);
    }

    smo_estimator_B.b_kstr_ax = switch_expression->size[0] *
      switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    smo_es_emxEnsureCapacity_char_T(switch_expression, smo_estimator_B.b_kstr_ax);
    smo_estimator_B.loop_ub_d = body->JointInternal.Type->size[0] *
      body->JointInternal.Type->size[1] - 1;
    for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax <=
         smo_estimator_B.loop_ub_d; smo_estimator_B.b_kstr_ax++) {
      switch_expression->data[smo_estimator_B.b_kstr_ax] =
        body->JointInternal.Type->data[smo_estimator_B.b_kstr_ax];
    }

    smo_estimator_B.b_bool_f = false;
    if (switch_expression->size[1] == 5) {
      smo_estimator_B.b_kstr_ax = 1;
      do {
        exitg1 = 0;
        if (smo_estimator_B.b_kstr_ax - 1 < 5) {
          smo_estimator_B.loop_ub_d = smo_estimator_B.b_kstr_ax - 1;
          if (switch_expression->data[smo_estimator_B.loop_ub_d] !=
              smo_estimator_B.b_h3[smo_estimator_B.loop_ub_d]) {
            exitg1 = 1;
          } else {
            smo_estimator_B.b_kstr_ax++;
          }
        } else {
          smo_estimator_B.b_bool_f = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (smo_estimator_B.b_bool_f) {
      smo_estimator_B.b_kstr_ax = 0;
    } else {
      for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 8;
           smo_estimator_B.b_kstr_ax++) {
        smo_estimator_B.b_h[smo_estimator_B.b_kstr_ax] =
          tmp_1[smo_estimator_B.b_kstr_ax];
      }

      smo_estimator_B.b_bool_f = false;
      if (switch_expression->size[1] == 8) {
        smo_estimator_B.b_kstr_ax = 1;
        do {
          exitg1 = 0;
          if (smo_estimator_B.b_kstr_ax - 1 < 8) {
            smo_estimator_B.loop_ub_d = smo_estimator_B.b_kstr_ax - 1;
            if (switch_expression->data[smo_estimator_B.loop_ub_d] !=
                smo_estimator_B.b_h[smo_estimator_B.loop_ub_d]) {
              exitg1 = 1;
            } else {
              smo_estimator_B.b_kstr_ax++;
            }
          } else {
            smo_estimator_B.b_bool_f = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (smo_estimator_B.b_bool_f) {
        smo_estimator_B.b_kstr_ax = 1;
      } else {
        smo_estimator_B.b_kstr_ax = -1;
      }
    }

    switch (smo_estimator_B.b_kstr_ax) {
     case 0:
      memset(&smo_estimator_B.c_f1[0], 0, sizeof(real_T) << 4U);
      smo_estimator_B.c_f1[0] = 1.0;
      smo_estimator_B.c_f1[5] = 1.0;
      smo_estimator_B.c_f1[10] = 1.0;
      smo_estimator_B.c_f1[15] = 1.0;
      break;

     case 1:
      sm_rigidBodyJoint_get_JointAxis(&body->JointInternal, smo_estimator_B.v_j);
      smo_estimator_B.d -= smo_estimator_B.e;
      for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax <
           smo_estimator_B.d; smo_estimator_B.b_kstr_ax++) {
        smo_estimator_B.e_data[smo_estimator_B.b_kstr_ax] = smo_estimator_B.e +
          smo_estimator_B.b_kstr_ax;
      }

      smo_estimator_B.result_data_j[0] = smo_estimator_B.v_j[0];
      smo_estimator_B.result_data_j[1] = smo_estimator_B.v_j[1];
      smo_estimator_B.result_data_j[2] = smo_estimator_B.v_j[2];
      if (0 <= (smo_estimator_B.d != 0) - 1) {
        smo_estimator_B.result_data_j[3] = qvec[smo_estimator_B.e_data[0]];
      }

      smo_estimator_B.k = 1.0 / sqrt((smo_estimator_B.result_data_j[0] *
        smo_estimator_B.result_data_j[0] + smo_estimator_B.result_data_j[1] *
        smo_estimator_B.result_data_j[1]) + smo_estimator_B.result_data_j[2] *
        smo_estimator_B.result_data_j[2]);
      smo_estimator_B.v_j[0] = smo_estimator_B.result_data_j[0] *
        smo_estimator_B.k;
      smo_estimator_B.v_j[1] = smo_estimator_B.result_data_j[1] *
        smo_estimator_B.k;
      smo_estimator_B.v_j[2] = smo_estimator_B.result_data_j[2] *
        smo_estimator_B.k;
      smo_estimator_B.k = cos(smo_estimator_B.result_data_j[3]);
      smo_estimator_B.sth_i = sin(smo_estimator_B.result_data_j[3]);
      smo_estimator_B.tempR_l[0] = smo_estimator_B.v_j[0] * smo_estimator_B.v_j
        [0] * (1.0 - smo_estimator_B.k) + smo_estimator_B.k;
      smo_estimator_B.tempR_tmp_f = smo_estimator_B.v_j[1] *
        smo_estimator_B.v_j[0] * (1.0 - smo_estimator_B.k);
      smo_estimator_B.tempR_tmp_g = smo_estimator_B.v_j[2] *
        smo_estimator_B.sth_i;
      smo_estimator_B.tempR_l[1] = smo_estimator_B.tempR_tmp_f -
        smo_estimator_B.tempR_tmp_g;
      smo_estimator_B.tempR_tmp_c = smo_estimator_B.v_j[2] *
        smo_estimator_B.v_j[0] * (1.0 - smo_estimator_B.k);
      smo_estimator_B.tempR_tmp_o = smo_estimator_B.v_j[1] *
        smo_estimator_B.sth_i;
      smo_estimator_B.tempR_l[2] = smo_estimator_B.tempR_tmp_c +
        smo_estimator_B.tempR_tmp_o;
      smo_estimator_B.tempR_l[3] = smo_estimator_B.tempR_tmp_f +
        smo_estimator_B.tempR_tmp_g;
      smo_estimator_B.tempR_l[4] = smo_estimator_B.v_j[1] * smo_estimator_B.v_j
        [1] * (1.0 - smo_estimator_B.k) + smo_estimator_B.k;
      smo_estimator_B.tempR_tmp_f = smo_estimator_B.v_j[2] *
        smo_estimator_B.v_j[1] * (1.0 - smo_estimator_B.k);
      smo_estimator_B.tempR_tmp_g = smo_estimator_B.v_j[0] *
        smo_estimator_B.sth_i;
      smo_estimator_B.tempR_l[5] = smo_estimator_B.tempR_tmp_f -
        smo_estimator_B.tempR_tmp_g;
      smo_estimator_B.tempR_l[6] = smo_estimator_B.tempR_tmp_c -
        smo_estimator_B.tempR_tmp_o;
      smo_estimator_B.tempR_l[7] = smo_estimator_B.tempR_tmp_f +
        smo_estimator_B.tempR_tmp_g;
      smo_estimator_B.tempR_l[8] = smo_estimator_B.v_j[2] * smo_estimator_B.v_j
        [2] * (1.0 - smo_estimator_B.k) + smo_estimator_B.k;
      for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 3;
           smo_estimator_B.b_kstr_ax++) {
        smo_estimator_B.e = smo_estimator_B.b_kstr_ax + 1;
        smo_estimator_B.R_dy[smo_estimator_B.e - 1] = smo_estimator_B.tempR_l
          [(smo_estimator_B.e - 1) * 3];
        smo_estimator_B.e = smo_estimator_B.b_kstr_ax + 1;
        smo_estimator_B.R_dy[smo_estimator_B.e + 2] = smo_estimator_B.tempR_l
          [(smo_estimator_B.e - 1) * 3 + 1];
        smo_estimator_B.e = smo_estimator_B.b_kstr_ax + 1;
        smo_estimator_B.R_dy[smo_estimator_B.e + 5] = smo_estimator_B.tempR_l
          [(smo_estimator_B.e - 1) * 3 + 2];
      }

      memset(&smo_estimator_B.c_f1[0], 0, sizeof(real_T) << 4U);
      for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 3;
           smo_estimator_B.b_kstr_ax++) {
        smo_estimator_B.d = smo_estimator_B.b_kstr_ax << 2;
        smo_estimator_B.c_f1[smo_estimator_B.d] = smo_estimator_B.R_dy[3 *
          smo_estimator_B.b_kstr_ax];
        smo_estimator_B.c_f1[smo_estimator_B.d + 1] = smo_estimator_B.R_dy[3 *
          smo_estimator_B.b_kstr_ax + 1];
        smo_estimator_B.c_f1[smo_estimator_B.d + 2] = smo_estimator_B.R_dy[3 *
          smo_estimator_B.b_kstr_ax + 2];
      }

      smo_estimator_B.c_f1[15] = 1.0;
      break;

     default:
      sm_rigidBodyJoint_get_JointAxis(&body->JointInternal, smo_estimator_B.v_j);
      memset(&smo_estimator_B.tempR_l[0], 0, 9U * sizeof(real_T));
      smo_estimator_B.tempR_l[0] = 1.0;
      smo_estimator_B.tempR_l[4] = 1.0;
      smo_estimator_B.tempR_l[8] = 1.0;
      for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 3;
           smo_estimator_B.b_kstr_ax++) {
        smo_estimator_B.d = smo_estimator_B.b_kstr_ax << 2;
        smo_estimator_B.c_f1[smo_estimator_B.d] = smo_estimator_B.tempR_l[3 *
          smo_estimator_B.b_kstr_ax];
        smo_estimator_B.c_f1[smo_estimator_B.d + 1] = smo_estimator_B.tempR_l[3 *
          smo_estimator_B.b_kstr_ax + 1];
        smo_estimator_B.c_f1[smo_estimator_B.d + 2] = smo_estimator_B.tempR_l[3 *
          smo_estimator_B.b_kstr_ax + 2];
        smo_estimator_B.c_f1[smo_estimator_B.b_kstr_ax + 12] =
          smo_estimator_B.v_j[smo_estimator_B.b_kstr_ax] *
          qvec[smo_estimator_B.e];
      }

      smo_estimator_B.c_f1[3] = 0.0;
      smo_estimator_B.c_f1[7] = 0.0;
      smo_estimator_B.c_f1[11] = 0.0;
      smo_estimator_B.c_f1[15] = 1.0;
      break;
    }

    for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 16;
         smo_estimator_B.b_kstr_ax++) {
      smo_estimator_B.a[smo_estimator_B.b_kstr_ax] =
        body->JointInternal.JointToParentTransform[smo_estimator_B.b_kstr_ax];
    }

    for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 16;
         smo_estimator_B.b_kstr_ax++) {
      smo_estimator_B.b[smo_estimator_B.b_kstr_ax] =
        body->JointInternal.ChildToJointTransform[smo_estimator_B.b_kstr_ax];
    }

    for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 4;
         smo_estimator_B.b_kstr_ax++) {
      for (smo_estimator_B.e = 0; smo_estimator_B.e < 4; smo_estimator_B.e++) {
        smo_estimator_B.d = smo_estimator_B.e << 2;
        smo_estimator_B.loop_ub_d = smo_estimator_B.b_kstr_ax +
          smo_estimator_B.d;
        smo_estimator_B.a_c[smo_estimator_B.loop_ub_d] = 0.0;
        smo_estimator_B.a_c[smo_estimator_B.loop_ub_d] +=
          smo_estimator_B.c_f1[smo_estimator_B.d] *
          smo_estimator_B.a[smo_estimator_B.b_kstr_ax];
        smo_estimator_B.a_c[smo_estimator_B.loop_ub_d] +=
          smo_estimator_B.c_f1[smo_estimator_B.d + 1] *
          smo_estimator_B.a[smo_estimator_B.b_kstr_ax + 4];
        smo_estimator_B.a_c[smo_estimator_B.loop_ub_d] +=
          smo_estimator_B.c_f1[smo_estimator_B.d + 2] *
          smo_estimator_B.a[smo_estimator_B.b_kstr_ax + 8];
        smo_estimator_B.a_c[smo_estimator_B.loop_ub_d] +=
          smo_estimator_B.c_f1[smo_estimator_B.d + 3] *
          smo_estimator_B.a[smo_estimator_B.b_kstr_ax + 12];
      }

      for (smo_estimator_B.e = 0; smo_estimator_B.e < 4; smo_estimator_B.e++) {
        smo_estimator_B.d = smo_estimator_B.e << 2;
        smo_estimator_B.loop_ub_d = smo_estimator_B.b_kstr_ax +
          smo_estimator_B.d;
        Ttree->data[smo_estimator_B.b_jtilecol].f1[smo_estimator_B.loop_ub_d] =
          0.0;
        Ttree->data[smo_estimator_B.b_jtilecol].f1[smo_estimator_B.loop_ub_d] +=
          smo_estimator_B.b[smo_estimator_B.d] *
          smo_estimator_B.a_c[smo_estimator_B.b_kstr_ax];
        Ttree->data[smo_estimator_B.b_jtilecol].f1[smo_estimator_B.loop_ub_d] +=
          smo_estimator_B.b[smo_estimator_B.d + 1] *
          smo_estimator_B.a_c[smo_estimator_B.b_kstr_ax + 4];
        Ttree->data[smo_estimator_B.b_jtilecol].f1[smo_estimator_B.loop_ub_d] +=
          smo_estimator_B.b[smo_estimator_B.d + 2] *
          smo_estimator_B.a_c[smo_estimator_B.b_kstr_ax + 8];
        Ttree->data[smo_estimator_B.b_jtilecol].f1[smo_estimator_B.loop_ub_d] +=
          smo_estimator_B.b[smo_estimator_B.d + 3] *
          smo_estimator_B.a_c[smo_estimator_B.b_kstr_ax + 12];
      }
    }

    smo_estimator_B.k = smo_estimator_B.n;
    if (body->ParentIndex > 0.0) {
      for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 16;
           smo_estimator_B.b_kstr_ax++) {
        smo_estimator_B.a[smo_estimator_B.b_kstr_ax] = Ttree->data
          [static_cast<int32_T>(body->ParentIndex) - 1]
          .f1[smo_estimator_B.b_kstr_ax];
      }

      for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 4;
           smo_estimator_B.b_kstr_ax++) {
        for (smo_estimator_B.e = 0; smo_estimator_B.e < 4; smo_estimator_B.e++)
        {
          smo_estimator_B.d = smo_estimator_B.e << 2;
          smo_estimator_B.loop_ub_d = smo_estimator_B.b_kstr_ax +
            smo_estimator_B.d;
          smo_estimator_B.a_c[smo_estimator_B.loop_ub_d] = 0.0;
          smo_estimator_B.a_c[smo_estimator_B.loop_ub_d] += Ttree->
            data[smo_estimator_B.b_jtilecol].f1[smo_estimator_B.d] *
            smo_estimator_B.a[smo_estimator_B.b_kstr_ax];
          smo_estimator_B.a_c[smo_estimator_B.loop_ub_d] += Ttree->
            data[smo_estimator_B.b_jtilecol].f1[smo_estimator_B.d + 1] *
            smo_estimator_B.a[smo_estimator_B.b_kstr_ax + 4];
          smo_estimator_B.a_c[smo_estimator_B.loop_ub_d] += Ttree->
            data[smo_estimator_B.b_jtilecol].f1[smo_estimator_B.d + 2] *
            smo_estimator_B.a[smo_estimator_B.b_kstr_ax + 8];
          smo_estimator_B.a_c[smo_estimator_B.loop_ub_d] += Ttree->
            data[smo_estimator_B.b_jtilecol].f1[smo_estimator_B.d + 3] *
            smo_estimator_B.a[smo_estimator_B.b_kstr_ax + 12];
        }
      }

      for (smo_estimator_B.b_kstr_ax = 0; smo_estimator_B.b_kstr_ax < 16;
           smo_estimator_B.b_kstr_ax++) {
        Ttree->data[smo_estimator_B.b_jtilecol].f1[smo_estimator_B.b_kstr_ax] =
          smo_estimator_B.a_c[smo_estimator_B.b_kstr_ax];
      }
    }
  }

  smo_estimator_emxFree_char_T(&switch_expression);
}

static void smo_estimat_emxFree_f_cell_wrap(emxArray_f_cell_wrap_smo_esti_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_smo_esti_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_smo_estimator_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_smo_esti_T *)NULL;
  }
}

static void smo_estimator_SystemCore_step_m(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_smo_estimator_std_msgs_MultiArrayDimension varargout_2_Layout_Dim[16],
  uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  *varargout_1 = Sub_smo_estimator_638.getLatestMessage
    (&smo_estimator_B.b_varargout_2);
  for (smo_estimator_B.i_p = 0; smo_estimator_B.i_p < 7; smo_estimator_B.i_p++)
  {
    varargout_2_Data[smo_estimator_B.i_p] =
      smo_estimator_B.b_varargout_2.Data[smo_estimator_B.i_p];
  }

  *varargout_2_Data_SL_Info_Curren =
    smo_estimator_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    smo_estimator_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    smo_estimator_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0], &smo_estimator_B.b_varargout_2.Layout.Dim[0],
         sizeof(SL_Bus_smo_estimator_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    smo_estimator_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    smo_estimator_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void RigidBodyTreeDynamics_massMat_m(q_robotics_manip_internal__mr_T
  *robot, const real_T q[7], emxArray_real_T_smo_estimator_T *H,
  emxArray_real_T_smo_estimator_T *lambda)
{
  emxArray_f_cell_wrap_smo_es_m_T *Ic;
  emxArray_f_cell_wrap_smo_es_m_T *X;
  emxArray_real_T_smo_estimator_T *lambda_;
  emxArray_real_T_smo_estimator_T *Si;
  emxArray_real_T_smo_estimator_T *Fi;
  emxArray_real_T_smo_estimator_T *Hji;
  emxArray_real_T_smo_estimator_T *s;
  o_robotics_manip_internal_R_m_T *obj;
  emxArray_char_T_smo_estimator_T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  smo_estimator_B.nb_l = robot->NumBodies;
  smo_estimator_B.vNum_o = robot->VelocityNumber;
  smo_estimator_B.f = H->size[0] * H->size[1];
  smo_estimator_B.b_i_o = static_cast<int32_T>(smo_estimator_B.vNum_o);
  H->size[0] = smo_estimator_B.b_i_o;
  H->size[1] = smo_estimator_B.b_i_o;
  smo_es_emxEnsureCapacity_real_T(H, smo_estimator_B.f);
  smo_estimator_B.loop_ub = smo_estimator_B.b_i_o * smo_estimator_B.b_i_o - 1;
  for (smo_estimator_B.f = 0; smo_estimator_B.f <= smo_estimator_B.loop_ub;
       smo_estimator_B.f++) {
    H->data[smo_estimator_B.f] = 0.0;
  }

  smo_estimator_emxInit_real_T(&lambda_, 2);
  smo_estimator_B.f = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  smo_estimator_B.nm1d2 = static_cast<int32_T>(smo_estimator_B.nb_l);
  lambda_->size[1] = smo_estimator_B.nm1d2;
  smo_es_emxEnsureCapacity_real_T(lambda_, smo_estimator_B.f);
  smo_estimator_B.idx = smo_estimator_B.nm1d2 - 1;
  for (smo_estimator_B.f = 0; smo_estimator_B.f <= smo_estimator_B.idx;
       smo_estimator_B.f++) {
    lambda_->data[smo_estimator_B.f] = 0.0;
  }

  smo_estimator_B.f = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = smo_estimator_B.b_i_o;
  smo_es_emxEnsureCapacity_real_T(lambda, smo_estimator_B.f);
  smo_estimator_B.loop_ub = smo_estimator_B.b_i_o - 1;
  for (smo_estimator_B.f = 0; smo_estimator_B.f <= smo_estimator_B.loop_ub;
       smo_estimator_B.f++) {
    lambda->data[smo_estimator_B.f] = 0.0;
  }

  smo_estima_emxInit_f_cell_wrap1(&Ic, 2);
  smo_estima_emxInit_f_cell_wrap1(&X, 2);
  smo_estimator_B.f = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = smo_estimator_B.nm1d2;
  emxEnsureCapacity_f_cell_wrap1(Ic, smo_estimator_B.f);
  smo_estimator_B.f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = smo_estimator_B.nm1d2;
  emxEnsureCapacity_f_cell_wrap1(X, smo_estimator_B.f);
  for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o <= smo_estimator_B.idx;
       smo_estimator_B.b_i_o++) {
    for (smo_estimator_B.f = 0; smo_estimator_B.f < 36; smo_estimator_B.f++) {
      Ic->data[smo_estimator_B.b_i_o].f1[smo_estimator_B.f] = robot->
        Bodies[smo_estimator_B.b_i_o]->SpatialInertia[smo_estimator_B.f];
    }

    smo_estimator_B.vNum_o = robot->PositionDoFMap[smo_estimator_B.b_i_o];
    smo_estimator_B.p_idx_1 = robot->PositionDoFMap[smo_estimator_B.b_i_o + 10];
    if (smo_estimator_B.p_idx_1 < smo_estimator_B.vNum_o) {
      obj = robot->Bodies[smo_estimator_B.b_i_o];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, smo_estimator_B.T_c);
    } else {
      if (smo_estimator_B.vNum_o > smo_estimator_B.p_idx_1) {
        smo_estimator_B.nm1d2 = 0;
        smo_estimator_B.f = -1;
      } else {
        smo_estimator_B.nm1d2 = static_cast<int32_T>(smo_estimator_B.vNum_o) - 1;
        smo_estimator_B.f = static_cast<int32_T>(smo_estimator_B.p_idx_1) - 1;
      }

      obj = robot->Bodies[smo_estimator_B.b_i_o];
      smo_estimator_B.loop_ub = smo_estimator_B.f - smo_estimator_B.nm1d2;
      smo_estimator_B.q_size_e = smo_estimator_B.loop_ub + 1;
      for (smo_estimator_B.f = 0; smo_estimator_B.f <= smo_estimator_B.loop_ub;
           smo_estimator_B.f++) {
        smo_estimator_B.q_data_b[smo_estimator_B.f] = q[smo_estimator_B.nm1d2 +
          smo_estimator_B.f];
      }

      rigidBodyJoint_transformBodyT_m(&obj->JointInternal,
        smo_estimator_B.q_data_b, &smo_estimator_B.q_size_e, smo_estimator_B.T_c);
    }

    smo_estimator_tforminv(smo_estimator_B.T_c, smo_estimator_B.dv);
    smo_estimat_tformToSpatialXform(smo_estimator_B.dv, X->
      data[smo_estimator_B.b_i_o].f1);
  }

  smo_estimator_B.nm1d2 = static_cast<int32_T>(((-1.0 - smo_estimator_B.nb_l) +
    1.0) / -1.0) - 1;
  smo_estimator_emxInit_real_T(&Si, 2);
  smo_estimator_emxInit_real_T(&Fi, 2);
  smo_estimator_emxInit_real_T(&Hji, 2);
  smo_estimator_emxInit_char_T(&a, 2);
  for (smo_estimator_B.idx = 0; smo_estimator_B.idx <= smo_estimator_B.nm1d2;
       smo_estimator_B.idx++) {
    smo_estimator_B.n_b = static_cast<int32_T>(smo_estimator_B.nb_l + -
      static_cast<real_T>(smo_estimator_B.idx));
    smo_estimator_B.pid_tmp = smo_estimator_B.n_b - 1;
    smo_estimator_B.pid = robot->Bodies[smo_estimator_B.pid_tmp]->ParentIndex;
    smo_estimator_B.vNum_o = robot->VelocityDoFMap[smo_estimator_B.n_b - 1];
    smo_estimator_B.p_idx_1 = robot->VelocityDoFMap[smo_estimator_B.n_b + 9];
    if (smo_estimator_B.pid > 0.0) {
      for (smo_estimator_B.f = 0; smo_estimator_B.f < 6; smo_estimator_B.f++) {
        for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o < 6;
             smo_estimator_B.b_i_o++) {
          smo_estimator_B.X_tmp = smo_estimator_B.f + 6 * smo_estimator_B.b_i_o;
          smo_estimator_B.X[smo_estimator_B.X_tmp] = 0.0;
          for (smo_estimator_B.loop_ub = 0; smo_estimator_B.loop_ub < 6;
               smo_estimator_B.loop_ub++) {
            smo_estimator_B.X[smo_estimator_B.X_tmp] += X->
              data[smo_estimator_B.pid_tmp].f1[6 * smo_estimator_B.f +
              smo_estimator_B.loop_ub] * Ic->data[smo_estimator_B.pid_tmp].f1[6 *
              smo_estimator_B.b_i_o + smo_estimator_B.loop_ub];
          }
        }
      }

      for (smo_estimator_B.f = 0; smo_estimator_B.f < 6; smo_estimator_B.f++) {
        for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o < 6;
             smo_estimator_B.b_i_o++) {
          smo_estimator_B.b_idx_0_o = 0.0;
          for (smo_estimator_B.loop_ub = 0; smo_estimator_B.loop_ub < 6;
               smo_estimator_B.loop_ub++) {
            smo_estimator_B.b_idx_0_o += smo_estimator_B.X[6 *
              smo_estimator_B.loop_ub + smo_estimator_B.f] * X->
              data[smo_estimator_B.pid_tmp].f1[6 * smo_estimator_B.b_i_o +
              smo_estimator_B.loop_ub];
          }

          smo_estimator_B.loop_ub = 6 * smo_estimator_B.b_i_o +
            smo_estimator_B.f;
          Ic->data[static_cast<int32_T>(smo_estimator_B.pid) - 1]
            .f1[smo_estimator_B.loop_ub] += smo_estimator_B.b_idx_0_o;
        }
      }

      lambda_->data[smo_estimator_B.pid_tmp] = smo_estimator_B.pid;
      exitg1 = false;
      while ((!exitg1) && (lambda_->data[smo_estimator_B.pid_tmp] > 0.0)) {
        obj = robot->Bodies[static_cast<int32_T>(lambda_->
          data[smo_estimator_B.pid_tmp]) - 1];
        smo_estimator_B.f = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = obj->JointInternal.Type->size[1];
        smo_es_emxEnsureCapacity_char_T(a, smo_estimator_B.f);
        smo_estimator_B.loop_ub = obj->JointInternal.Type->size[0] *
          obj->JointInternal.Type->size[1] - 1;
        for (smo_estimator_B.f = 0; smo_estimator_B.f <= smo_estimator_B.loop_ub;
             smo_estimator_B.f++) {
          a->data[smo_estimator_B.f] = obj->JointInternal.Type->
            data[smo_estimator_B.f];
        }

        for (smo_estimator_B.f = 0; smo_estimator_B.f < 5; smo_estimator_B.f++)
        {
          smo_estimator_B.b_me[smo_estimator_B.f] = tmp[smo_estimator_B.f];
        }

        smo_estimator_B.b_bool_m0 = false;
        if (a->size[1] == 5) {
          smo_estimator_B.f = 1;
          do {
            exitg2 = 0;
            if (smo_estimator_B.f - 1 < 5) {
              smo_estimator_B.b_i_o = smo_estimator_B.f - 1;
              if (a->data[smo_estimator_B.b_i_o] !=
                  smo_estimator_B.b_me[smo_estimator_B.b_i_o]) {
                exitg2 = 1;
              } else {
                smo_estimator_B.f++;
              }
            } else {
              smo_estimator_B.b_bool_m0 = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (smo_estimator_B.b_bool_m0) {
          lambda_->data[smo_estimator_B.pid_tmp] = robot->Bodies
            [static_cast<int32_T>(lambda_->data[smo_estimator_B.pid_tmp]) - 1]
            ->ParentIndex;
        } else {
          exitg1 = true;
        }
      }
    }

    smo_estimator_B.b_idx_0_o = robot->VelocityDoFMap[smo_estimator_B.n_b - 1];
    smo_estimator_B.b_idx_1_i = robot->VelocityDoFMap[smo_estimator_B.n_b + 9];
    if (smo_estimator_B.b_idx_0_o <= smo_estimator_B.b_idx_1_i) {
      obj = robot->Bodies[smo_estimator_B.pid_tmp];
      smo_estimator_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      smo_es_emxEnsureCapacity_real_T(Si, smo_estimator_B.f);
      smo_estimator_B.loop_ub = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (smo_estimator_B.f = 0; smo_estimator_B.f <= smo_estimator_B.loop_ub;
           smo_estimator_B.f++) {
        Si->data[smo_estimator_B.f] = obj->JointInternal.MotionSubspace->
          data[smo_estimator_B.f];
      }

      smo_estimator_B.n_b = Si->size[1] - 1;
      smo_estimator_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      smo_es_emxEnsureCapacity_real_T(Fi, smo_estimator_B.f);
      for (smo_estimator_B.loop_ub = 0; smo_estimator_B.loop_ub <=
           smo_estimator_B.n_b; smo_estimator_B.loop_ub++) {
        smo_estimator_B.coffset_tmp = smo_estimator_B.loop_ub * 6 - 1;
        for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o < 6;
             smo_estimator_B.b_i_o++) {
          smo_estimator_B.s = 0.0;
          for (smo_estimator_B.f = 0; smo_estimator_B.f < 6; smo_estimator_B.f++)
          {
            smo_estimator_B.s += Ic->data[smo_estimator_B.pid_tmp]
              .f1[smo_estimator_B.f * 6 + smo_estimator_B.b_i_o] * Si->data
              [(smo_estimator_B.coffset_tmp + smo_estimator_B.f) + 1];
          }

          Fi->data[(smo_estimator_B.coffset_tmp + smo_estimator_B.b_i_o) + 1] =
            smo_estimator_B.s;
        }
      }

      if (smo_estimator_B.vNum_o > smo_estimator_B.p_idx_1) {
        smo_estimator_B.coffset_tmp = 0;
        smo_estimator_B.cb = 0;
      } else {
        smo_estimator_B.coffset_tmp = static_cast<int32_T>
          (smo_estimator_B.vNum_o) - 1;
        smo_estimator_B.cb = static_cast<int32_T>(smo_estimator_B.vNum_o) - 1;
      }

      smo_estimator_B.m_a = Si->size[1];
      smo_estimator_B.n_b = Fi->size[1] - 1;
      smo_estimator_B.f = Hji->size[0] * Hji->size[1];
      Hji->size[0] = Si->size[1];
      Hji->size[1] = Fi->size[1];
      smo_es_emxEnsureCapacity_real_T(Hji, smo_estimator_B.f);
      for (smo_estimator_B.loop_ub = 0; smo_estimator_B.loop_ub <=
           smo_estimator_B.n_b; smo_estimator_B.loop_ub++) {
        smo_estimator_B.coffset = smo_estimator_B.loop_ub * smo_estimator_B.m_a
          - 1;
        smo_estimator_B.boffset = smo_estimator_B.loop_ub * 6 - 1;
        for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o <
             smo_estimator_B.m_a; smo_estimator_B.b_i_o++) {
          smo_estimator_B.aoffset_g = smo_estimator_B.b_i_o * 6 - 1;
          smo_estimator_B.s = 0.0;
          for (smo_estimator_B.f = 0; smo_estimator_B.f < 6; smo_estimator_B.f++)
          {
            smo_estimator_B.X_tmp = smo_estimator_B.f + 1;
            smo_estimator_B.s += Si->data[smo_estimator_B.aoffset_g +
              smo_estimator_B.X_tmp] * Fi->data[smo_estimator_B.boffset +
              smo_estimator_B.X_tmp];
          }

          Hji->data[(smo_estimator_B.coffset + smo_estimator_B.b_i_o) + 1] =
            smo_estimator_B.s;
        }
      }

      smo_estimator_B.loop_ub = Hji->size[1];
      for (smo_estimator_B.f = 0; smo_estimator_B.f < smo_estimator_B.loop_ub;
           smo_estimator_B.f++) {
        smo_estimator_B.n_b = Hji->size[0];
        for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o <
             smo_estimator_B.n_b; smo_estimator_B.b_i_o++) {
          H->data[(smo_estimator_B.coffset_tmp + smo_estimator_B.b_i_o) +
            H->size[0] * (smo_estimator_B.cb + smo_estimator_B.f)] = Hji->
            data[Hji->size[0] * smo_estimator_B.f + smo_estimator_B.b_i_o];
        }
      }

      smo_estimator_B.n_b = Fi->size[1];
      smo_estimator_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = Fi->size[1];
      smo_es_emxEnsureCapacity_real_T(Si, smo_estimator_B.f);
      smo_estimator_B.loop_ub = Fi->size[0] * Fi->size[1] - 1;
      for (smo_estimator_B.f = 0; smo_estimator_B.f <= smo_estimator_B.loop_ub;
           smo_estimator_B.f++) {
        Si->data[smo_estimator_B.f] = Fi->data[smo_estimator_B.f];
      }

      smo_estimator_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = smo_estimator_B.n_b;
      smo_es_emxEnsureCapacity_real_T(Fi, smo_estimator_B.f);
      for (smo_estimator_B.loop_ub = 0; smo_estimator_B.loop_ub <
           smo_estimator_B.n_b; smo_estimator_B.loop_ub++) {
        smo_estimator_B.coffset_tmp = smo_estimator_B.loop_ub * 6 - 1;
        for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o < 6;
             smo_estimator_B.b_i_o++) {
          smo_estimator_B.aoffset_g = smo_estimator_B.b_i_o * 6 - 1;
          smo_estimator_B.s = 0.0;
          for (smo_estimator_B.f = 0; smo_estimator_B.f < 6; smo_estimator_B.f++)
          {
            smo_estimator_B.X_tmp = smo_estimator_B.f + 1;
            smo_estimator_B.s += X->data[smo_estimator_B.pid_tmp]
              .f1[smo_estimator_B.aoffset_g + smo_estimator_B.X_tmp] * Si->
              data[smo_estimator_B.coffset_tmp + smo_estimator_B.X_tmp];
          }

          Fi->data[(smo_estimator_B.coffset_tmp + smo_estimator_B.b_i_o) + 1] =
            smo_estimator_B.s;
        }
      }

      while (smo_estimator_B.pid > 0.0) {
        smo_estimator_B.b_i_o = static_cast<int32_T>(smo_estimator_B.pid);
        smo_estimator_B.pid_tmp = smo_estimator_B.b_i_o - 1;
        obj = robot->Bodies[smo_estimator_B.pid_tmp];
        smo_estimator_B.f = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
        smo_es_emxEnsureCapacity_real_T(Si, smo_estimator_B.f);
        smo_estimator_B.loop_ub = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (smo_estimator_B.f = 0; smo_estimator_B.f <= smo_estimator_B.loop_ub;
             smo_estimator_B.f++) {
          Si->data[smo_estimator_B.f] = obj->JointInternal.MotionSubspace->
            data[smo_estimator_B.f];
        }

        smo_estimator_B.b_idx_0_o = robot->VelocityDoFMap[smo_estimator_B.b_i_o
          - 1];
        smo_estimator_B.b_idx_1_i = robot->VelocityDoFMap[smo_estimator_B.b_i_o
          + 9];
        if (smo_estimator_B.b_idx_0_o <= smo_estimator_B.b_idx_1_i) {
          smo_estimator_B.m_a = Si->size[1];
          smo_estimator_B.n_b = Fi->size[1] - 1;
          smo_estimator_B.f = Hji->size[0] * Hji->size[1];
          Hji->size[0] = Si->size[1];
          Hji->size[1] = Fi->size[1];
          smo_es_emxEnsureCapacity_real_T(Hji, smo_estimator_B.f);
          for (smo_estimator_B.loop_ub = 0; smo_estimator_B.loop_ub <=
               smo_estimator_B.n_b; smo_estimator_B.loop_ub++) {
            smo_estimator_B.coffset = smo_estimator_B.loop_ub *
              smo_estimator_B.m_a - 1;
            smo_estimator_B.boffset = smo_estimator_B.loop_ub * 6 - 1;
            for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o <
                 smo_estimator_B.m_a; smo_estimator_B.b_i_o++) {
              smo_estimator_B.aoffset_g = smo_estimator_B.b_i_o * 6 - 1;
              smo_estimator_B.s = 0.0;
              for (smo_estimator_B.f = 0; smo_estimator_B.f < 6;
                   smo_estimator_B.f++) {
                smo_estimator_B.X_tmp = smo_estimator_B.f + 1;
                smo_estimator_B.s += Si->data[smo_estimator_B.aoffset_g +
                  smo_estimator_B.X_tmp] * Fi->data[smo_estimator_B.boffset +
                  smo_estimator_B.X_tmp];
              }

              Hji->data[(smo_estimator_B.coffset + smo_estimator_B.b_i_o) + 1] =
                smo_estimator_B.s;
            }
          }

          if (smo_estimator_B.b_idx_0_o > smo_estimator_B.b_idx_1_i) {
            smo_estimator_B.X_tmp = 0;
          } else {
            smo_estimator_B.X_tmp = static_cast<int32_T>
              (smo_estimator_B.b_idx_0_o) - 1;
          }

          if (smo_estimator_B.vNum_o > smo_estimator_B.p_idx_1) {
            smo_estimator_B.coffset_tmp = 0;
          } else {
            smo_estimator_B.coffset_tmp = static_cast<int32_T>
              (smo_estimator_B.vNum_o) - 1;
          }

          smo_estimator_B.loop_ub = Hji->size[1];
          for (smo_estimator_B.f = 0; smo_estimator_B.f <
               smo_estimator_B.loop_ub; smo_estimator_B.f++) {
            smo_estimator_B.n_b = Hji->size[0];
            for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o <
                 smo_estimator_B.n_b; smo_estimator_B.b_i_o++) {
              H->data[(smo_estimator_B.X_tmp + smo_estimator_B.b_i_o) + H->size
                [0] * (smo_estimator_B.coffset_tmp + smo_estimator_B.f)] =
                Hji->data[Hji->size[0] * smo_estimator_B.f +
                smo_estimator_B.b_i_o];
            }
          }

          if (smo_estimator_B.vNum_o > smo_estimator_B.p_idx_1) {
            smo_estimator_B.X_tmp = 0;
          } else {
            smo_estimator_B.X_tmp = static_cast<int32_T>(smo_estimator_B.vNum_o)
              - 1;
          }

          if (smo_estimator_B.b_idx_0_o > smo_estimator_B.b_idx_1_i) {
            smo_estimator_B.coffset_tmp = 0;
          } else {
            smo_estimator_B.coffset_tmp = static_cast<int32_T>
              (smo_estimator_B.b_idx_0_o) - 1;
          }

          smo_estimator_B.loop_ub = Hji->size[0];
          for (smo_estimator_B.f = 0; smo_estimator_B.f <
               smo_estimator_B.loop_ub; smo_estimator_B.f++) {
            smo_estimator_B.n_b = Hji->size[1];
            for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o <
                 smo_estimator_B.n_b; smo_estimator_B.b_i_o++) {
              H->data[(smo_estimator_B.X_tmp + smo_estimator_B.b_i_o) + H->size
                [0] * (smo_estimator_B.coffset_tmp + smo_estimator_B.f)] =
                Hji->data[Hji->size[0] * smo_estimator_B.b_i_o +
                smo_estimator_B.f];
            }
          }
        }

        smo_estimator_B.n_b = Fi->size[1];
        smo_estimator_B.f = Si->size[0] * Si->size[1];
        Si->size[0] = 6;
        Si->size[1] = Fi->size[1];
        smo_es_emxEnsureCapacity_real_T(Si, smo_estimator_B.f);
        smo_estimator_B.loop_ub = Fi->size[0] * Fi->size[1] - 1;
        for (smo_estimator_B.f = 0; smo_estimator_B.f <= smo_estimator_B.loop_ub;
             smo_estimator_B.f++) {
          Si->data[smo_estimator_B.f] = Fi->data[smo_estimator_B.f];
        }

        smo_estimator_B.f = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = smo_estimator_B.n_b;
        smo_es_emxEnsureCapacity_real_T(Fi, smo_estimator_B.f);
        for (smo_estimator_B.loop_ub = 0; smo_estimator_B.loop_ub <
             smo_estimator_B.n_b; smo_estimator_B.loop_ub++) {
          smo_estimator_B.coffset_tmp = smo_estimator_B.loop_ub * 6 - 1;
          for (smo_estimator_B.b_i_o = 0; smo_estimator_B.b_i_o < 6;
               smo_estimator_B.b_i_o++) {
            smo_estimator_B.aoffset_g = smo_estimator_B.b_i_o * 6 - 1;
            smo_estimator_B.s = 0.0;
            for (smo_estimator_B.f = 0; smo_estimator_B.f < 6; smo_estimator_B.f
                 ++) {
              smo_estimator_B.X_tmp = smo_estimator_B.f + 1;
              smo_estimator_B.s += X->data[smo_estimator_B.pid_tmp]
                .f1[smo_estimator_B.aoffset_g + smo_estimator_B.X_tmp] *
                Si->data[smo_estimator_B.coffset_tmp + smo_estimator_B.X_tmp];
            }

            Fi->data[(smo_estimator_B.coffset_tmp + smo_estimator_B.b_i_o) + 1] =
              smo_estimator_B.s;
          }
        }

        smo_estimator_B.pid = robot->Bodies[smo_estimator_B.pid_tmp]
          ->ParentIndex;
      }
    }
  }

  smo_estimator_emxFree_char_T(&a);
  smo_estimator_emxFree_real_T(&Hji);
  smo_estimator_emxFree_real_T(&Fi);
  smo_estimator_emxFree_real_T(&Si);
  smo_estima_emxFree_f_cell_wrap1(&X);
  smo_estima_emxFree_f_cell_wrap1(&Ic);
  for (smo_estimator_B.f = 0; smo_estimator_B.f < 10; smo_estimator_B.f++) {
    smo_estimator_B.mask[smo_estimator_B.f] = (robot->
      VelocityDoFMap[smo_estimator_B.f] <= robot->
      VelocityDoFMap[smo_estimator_B.f + 10]);
  }

  smo_estimator_B.idx = 0;
  smo_estimator_B.f = 1;
  exitg1 = false;
  while ((!exitg1) && (smo_estimator_B.f - 1 < 10)) {
    if (smo_estimator_B.mask[smo_estimator_B.f - 1]) {
      smo_estimator_B.idx++;
      smo_estimator_B.ii_data[smo_estimator_B.idx - 1] = smo_estimator_B.f;
      if (smo_estimator_B.idx >= 10) {
        exitg1 = true;
      } else {
        smo_estimator_B.f++;
      }
    } else {
      smo_estimator_B.f++;
    }
  }

  if (1 > smo_estimator_B.idx) {
    smo_estimator_B.idx = 0;
  }

  for (smo_estimator_B.f = 0; smo_estimator_B.f < smo_estimator_B.idx;
       smo_estimator_B.f++) {
    smo_estimator_B.nonFixedIndices_data[smo_estimator_B.f] =
      smo_estimator_B.ii_data[smo_estimator_B.f];
  }

  smo_estimator_B.b_i_o = smo_estimator_B.idx - 1;
  smo_estimator_emxInit_real_T(&s, 2);
  for (smo_estimator_B.idx = 0; smo_estimator_B.idx <= smo_estimator_B.b_i_o;
       smo_estimator_B.idx++) {
    smo_estimator_B.vNum_o = robot->
      VelocityDoFMap[smo_estimator_B.nonFixedIndices_data[smo_estimator_B.idx] -
      1];
    smo_estimator_B.p_idx_1 = robot->
      VelocityDoFMap[smo_estimator_B.nonFixedIndices_data[smo_estimator_B.idx] +
      9];
    if (rtIsNaN(smo_estimator_B.vNum_o) || rtIsNaN(smo_estimator_B.p_idx_1)) {
      smo_estimator_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      smo_es_emxEnsureCapacity_real_T(s, smo_estimator_B.f);
      s->data[0] = (rtNaN);
    } else if (smo_estimator_B.p_idx_1 < smo_estimator_B.vNum_o) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(smo_estimator_B.vNum_o) || rtIsInf
                (smo_estimator_B.p_idx_1)) && (smo_estimator_B.vNum_o ==
                smo_estimator_B.p_idx_1)) {
      smo_estimator_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      smo_es_emxEnsureCapacity_real_T(s, smo_estimator_B.f);
      s->data[0] = (rtNaN);
    } else if (floor(smo_estimator_B.vNum_o) == smo_estimator_B.vNum_o) {
      smo_estimator_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      smo_estimator_B.loop_ub = static_cast<int32_T>(floor
        (smo_estimator_B.p_idx_1 - smo_estimator_B.vNum_o));
      s->size[1] = smo_estimator_B.loop_ub + 1;
      smo_es_emxEnsureCapacity_real_T(s, smo_estimator_B.f);
      for (smo_estimator_B.f = 0; smo_estimator_B.f <= smo_estimator_B.loop_ub;
           smo_estimator_B.f++) {
        s->data[smo_estimator_B.f] = smo_estimator_B.vNum_o + static_cast<real_T>
          (smo_estimator_B.f);
      }
    } else {
      smo_estimator_B.nb_l = floor((smo_estimator_B.p_idx_1 -
        smo_estimator_B.vNum_o) + 0.5);
      smo_estimator_B.pid = smo_estimator_B.vNum_o + smo_estimator_B.nb_l;
      smo_estimator_B.b_idx_0_o = smo_estimator_B.pid - smo_estimator_B.p_idx_1;
      smo_estimator_B.b_idx_1_i = fabs(smo_estimator_B.vNum_o);
      smo_estimator_B.s = fabs(smo_estimator_B.p_idx_1);
      if ((smo_estimator_B.b_idx_1_i > smo_estimator_B.s) || rtIsNaN
          (smo_estimator_B.s)) {
        smo_estimator_B.s = smo_estimator_B.b_idx_1_i;
      }

      if (fabs(smo_estimator_B.b_idx_0_o) < 4.4408920985006262E-16 *
          smo_estimator_B.s) {
        smo_estimator_B.nb_l++;
        smo_estimator_B.pid = smo_estimator_B.p_idx_1;
      } else if (smo_estimator_B.b_idx_0_o > 0.0) {
        smo_estimator_B.pid = (smo_estimator_B.nb_l - 1.0) +
          smo_estimator_B.vNum_o;
      } else {
        smo_estimator_B.nb_l++;
      }

      if (smo_estimator_B.nb_l >= 0.0) {
        smo_estimator_B.f = static_cast<int32_T>(smo_estimator_B.nb_l);
      } else {
        smo_estimator_B.f = 0;
      }

      smo_estimator_B.n_b = smo_estimator_B.f - 1;
      smo_estimator_B.f = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = smo_estimator_B.n_b + 1;
      smo_es_emxEnsureCapacity_real_T(s, smo_estimator_B.f);
      if (smo_estimator_B.n_b + 1 > 0) {
        s->data[0] = smo_estimator_B.vNum_o;
        if (smo_estimator_B.n_b + 1 > 1) {
          s->data[smo_estimator_B.n_b] = smo_estimator_B.pid;
          smo_estimator_B.nm1d2 = smo_estimator_B.n_b / 2;
          smo_estimator_B.loop_ub = smo_estimator_B.nm1d2 - 2;
          for (smo_estimator_B.f = 0; smo_estimator_B.f <=
               smo_estimator_B.loop_ub; smo_estimator_B.f++) {
            smo_estimator_B.X_tmp = smo_estimator_B.f + 1;
            s->data[smo_estimator_B.X_tmp] = smo_estimator_B.vNum_o +
              static_cast<real_T>(smo_estimator_B.X_tmp);
            s->data[smo_estimator_B.n_b - smo_estimator_B.X_tmp] =
              smo_estimator_B.pid - static_cast<real_T>(smo_estimator_B.X_tmp);
          }

          if (smo_estimator_B.nm1d2 << 1 == smo_estimator_B.n_b) {
            s->data[smo_estimator_B.nm1d2] = (smo_estimator_B.vNum_o +
              smo_estimator_B.pid) / 2.0;
          } else {
            s->data[smo_estimator_B.nm1d2] = smo_estimator_B.vNum_o +
              static_cast<real_T>(smo_estimator_B.nm1d2);
            s->data[smo_estimator_B.nm1d2 + 1] = smo_estimator_B.pid -
              static_cast<real_T>(smo_estimator_B.nm1d2);
          }
        }
      }
    }

    if (smo_estimator_B.vNum_o > smo_estimator_B.p_idx_1) {
      smo_estimator_B.nm1d2 = 0;
    } else {
      smo_estimator_B.nm1d2 = static_cast<int32_T>(smo_estimator_B.vNum_o) - 1;
    }

    smo_estimator_B.loop_ub = s->size[1];
    for (smo_estimator_B.f = 0; smo_estimator_B.f < smo_estimator_B.loop_ub;
         smo_estimator_B.f++) {
      lambda->data[smo_estimator_B.nm1d2 + smo_estimator_B.f] = s->
        data[smo_estimator_B.f] - 1.0;
    }

    if (lambda_->data[smo_estimator_B.nonFixedIndices_data[smo_estimator_B.idx]
        - 1] == 0.0) {
      lambda->data[static_cast<int32_T>(smo_estimator_B.vNum_o) - 1] = 0.0;
    } else {
      smo_estimator_B.f = static_cast<int32_T>(lambda_->
        data[smo_estimator_B.nonFixedIndices_data[smo_estimator_B.idx] - 1]);
      smo_estimator_B.b_idx_1_i = robot->VelocityDoFMap[smo_estimator_B.f + 9];
      lambda->data[static_cast<int32_T>(smo_estimator_B.vNum_o) - 1] =
        smo_estimator_B.b_idx_1_i;
    }
  }

  smo_estimator_emxFree_real_T(&s);
  smo_estimator_emxFree_real_T(&lambda_);
}

static void RigidBodyTreeDynamics_inverseDy(q_robotics_manip_internal__mr_T
  *robot, const real_T q[7], const real_T qdot[7], const real_T fext[60], real_T
  tau[7])
{
  emxArray_f_cell_wrap_smo_es_m_T *X;
  emxArray_f_cell_wrap_smo_es_m_T *Xtree;
  emxArray_real_T_smo_estimator_T *vJ;
  emxArray_real_T_smo_estimator_T *vB;
  emxArray_real_T_smo_estimator_T *aB;
  emxArray_real_T_smo_estimator_T *f;
  emxArray_real_T_smo_estimator_T *S;
  emxArray_real_T_smo_estimator_T *taui;
  o_robotics_manip_internal_R_m_T *obj;
  emxArray_char_T_smo_estimator_T *a;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  smo_estimator_B.a0[0] = 0.0;
  smo_estimator_B.a0[1] = 0.0;
  smo_estimator_B.a0[2] = 0.0;
  smo_estimator_B.a0[3] = -robot->Gravity[0];
  smo_estimator_B.a0[4] = -robot->Gravity[1];
  smo_estimator_B.a0[5] = -robot->Gravity[2];
  smo_estimator_emxInit_real_T(&vJ, 2);
  smo_estimator_B.nb = robot->NumBodies;
  smo_estimator_B.i_p4 = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  smo_estimator_B.unnamed_idx_1 = static_cast<int32_T>(smo_estimator_B.nb);
  vJ->size[1] = smo_estimator_B.unnamed_idx_1;
  smo_es_emxEnsureCapacity_real_T(vJ, smo_estimator_B.i_p4);
  smo_estimator_B.loop_ub_tmp = 6 * smo_estimator_B.unnamed_idx_1 - 1;
  for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 <=
       smo_estimator_B.loop_ub_tmp; smo_estimator_B.i_p4++) {
    vJ->data[smo_estimator_B.i_p4] = 0.0;
  }

  smo_estimator_emxInit_real_T(&vB, 2);
  smo_estimator_B.i_p4 = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = smo_estimator_B.unnamed_idx_1;
  smo_es_emxEnsureCapacity_real_T(vB, smo_estimator_B.i_p4);
  for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 <=
       smo_estimator_B.loop_ub_tmp; smo_estimator_B.i_p4++) {
    vB->data[smo_estimator_B.i_p4] = 0.0;
  }

  smo_estimator_emxInit_real_T(&aB, 2);
  smo_estimator_B.i_p4 = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = smo_estimator_B.unnamed_idx_1;
  smo_es_emxEnsureCapacity_real_T(aB, smo_estimator_B.i_p4);
  for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 <=
       smo_estimator_B.loop_ub_tmp; smo_estimator_B.i_p4++) {
    aB->data[smo_estimator_B.i_p4] = 0.0;
  }

  for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 7; smo_estimator_B.i_p4
       ++) {
    tau[smo_estimator_B.i_p4] = 0.0;
  }

  smo_estima_emxInit_f_cell_wrap1(&X, 2);
  smo_estima_emxInit_f_cell_wrap1(&Xtree, 2);
  smo_estimator_B.loop_ub_tmp = smo_estimator_B.unnamed_idx_1 - 1;
  smo_estimator_B.i_p4 = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = smo_estimator_B.unnamed_idx_1;
  emxEnsureCapacity_f_cell_wrap1(Xtree, smo_estimator_B.i_p4);
  smo_estimator_B.i_p4 = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = smo_estimator_B.unnamed_idx_1;
  emxEnsureCapacity_f_cell_wrap1(X, smo_estimator_B.i_p4);
  for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k <=
       smo_estimator_B.loop_ub_tmp; smo_estimator_B.b_k++) {
    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 36;
         smo_estimator_B.i_p4++) {
      Xtree->data[smo_estimator_B.b_k].f1[smo_estimator_B.i_p4] = 0.0;
    }

    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
         smo_estimator_B.i_p4++) {
      Xtree->data[smo_estimator_B.b_k].f1[smo_estimator_B.i_p4 + 6 *
        smo_estimator_B.i_p4] = 1.0;
    }

    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 36;
         smo_estimator_B.i_p4++) {
      X->data[smo_estimator_B.b_k].f1[smo_estimator_B.i_p4] = 0.0;
    }

    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
         smo_estimator_B.i_p4++) {
      X->data[smo_estimator_B.b_k].f1[smo_estimator_B.i_p4 + 6 *
        smo_estimator_B.i_p4] = 1.0;
    }
  }

  smo_estimator_emxInit_real_T(&f, 2);
  smo_estimator_B.i_p4 = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = smo_estimator_B.unnamed_idx_1;
  smo_es_emxEnsureCapacity_real_T(f, smo_estimator_B.i_p4);
  smo_estimator_emxInit_real_T(&S, 2);
  if (0 <= smo_estimator_B.loop_ub_tmp) {
    smo_estimator_B.dv2[0] = 0.0;
    smo_estimator_B.dv2[4] = 0.0;
    smo_estimator_B.dv2[8] = 0.0;
  }

  for (smo_estimator_B.unnamed_idx_1 = 0; smo_estimator_B.unnamed_idx_1 <=
       smo_estimator_B.loop_ub_tmp; smo_estimator_B.unnamed_idx_1++) {
    obj = robot->Bodies[smo_estimator_B.unnamed_idx_1];
    smo_estimator_B.i_p4 = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    smo_es_emxEnsureCapacity_real_T(S, smo_estimator_B.i_p4);
    smo_estimator_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 <= smo_estimator_B.b_k;
         smo_estimator_B.i_p4++) {
      S->data[smo_estimator_B.i_p4] = obj->JointInternal.MotionSubspace->
        data[smo_estimator_B.i_p4];
    }

    smo_estimator_B.a_idx_0 = robot->
      PositionDoFMap[smo_estimator_B.unnamed_idx_1];
    smo_estimator_B.a_idx_1 = robot->
      PositionDoFMap[smo_estimator_B.unnamed_idx_1 + 10];
    smo_estimator_B.b_idx_0 = robot->
      VelocityDoFMap[smo_estimator_B.unnamed_idx_1];
    smo_estimator_B.b_idx_1 = robot->
      VelocityDoFMap[smo_estimator_B.unnamed_idx_1 + 10];
    if (smo_estimator_B.a_idx_1 < smo_estimator_B.a_idx_0) {
      obj = robot->Bodies[smo_estimator_B.unnamed_idx_1];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, smo_estimator_B.T);
      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
           smo_estimator_B.i_p4++) {
        vJ->data[smo_estimator_B.i_p4 + 6 * smo_estimator_B.unnamed_idx_1] = 0.0;
      }
    } else {
      if (smo_estimator_B.a_idx_0 > smo_estimator_B.a_idx_1) {
        smo_estimator_B.b_k = 0;
        smo_estimator_B.i_p4 = -1;
      } else {
        smo_estimator_B.b_k = static_cast<int32_T>(smo_estimator_B.a_idx_0) - 1;
        smo_estimator_B.i_p4 = static_cast<int32_T>(smo_estimator_B.a_idx_1) - 1;
      }

      if (smo_estimator_B.b_idx_0 > smo_estimator_B.b_idx_1) {
        smo_estimator_B.p = -1;
      } else {
        smo_estimator_B.p = static_cast<int32_T>(smo_estimator_B.b_idx_0) - 2;
      }

      obj = robot->Bodies[smo_estimator_B.unnamed_idx_1];
      smo_estimator_B.q_size_tmp = smo_estimator_B.i_p4 - smo_estimator_B.b_k;
      smo_estimator_B.q_size = smo_estimator_B.q_size_tmp + 1;
      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 <=
           smo_estimator_B.q_size_tmp; smo_estimator_B.i_p4++) {
        smo_estimator_B.q_data[smo_estimator_B.i_p4] = q[smo_estimator_B.b_k +
          smo_estimator_B.i_p4];
      }

      rigidBodyJoint_transformBodyT_m(&obj->JointInternal,
        smo_estimator_B.q_data, &smo_estimator_B.q_size, smo_estimator_B.T);
      smo_estimator_B.inner = S->size[1] - 1;
      for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6; smo_estimator_B.b_k
           ++) {
        vJ->data[smo_estimator_B.b_k + 6 * smo_estimator_B.unnamed_idx_1] = 0.0;
      }

      for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k <= smo_estimator_B.inner;
           smo_estimator_B.b_k++) {
        smo_estimator_B.aoffset = smo_estimator_B.b_k * 6 - 1;
        for (smo_estimator_B.q_size_tmp = 0; smo_estimator_B.q_size_tmp < 6;
             smo_estimator_B.q_size_tmp++) {
          smo_estimator_B.i_p4 = 6 * smo_estimator_B.unnamed_idx_1 +
            smo_estimator_B.q_size_tmp;
          vJ->data[smo_estimator_B.i_p4] += S->data[(smo_estimator_B.aoffset +
            smo_estimator_B.q_size_tmp) + 1] * qdot[(smo_estimator_B.p +
            smo_estimator_B.b_k) + 1];
        }
      }
    }

    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 3;
         smo_estimator_B.i_p4++) {
      smo_estimator_B.R_j[3 * smo_estimator_B.i_p4] =
        smo_estimator_B.T[smo_estimator_B.i_p4];
      smo_estimator_B.R_j[3 * smo_estimator_B.i_p4 + 1] =
        smo_estimator_B.T[smo_estimator_B.i_p4 + 4];
      smo_estimator_B.R_j[3 * smo_estimator_B.i_p4 + 2] =
        smo_estimator_B.T[smo_estimator_B.i_p4 + 8];
    }

    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 9;
         smo_estimator_B.i_p4++) {
      smo_estimator_B.R_d[smo_estimator_B.i_p4] =
        -smo_estimator_B.R_j[smo_estimator_B.i_p4];
    }

    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 3;
         smo_estimator_B.i_p4++) {
      smo_estimator_B.b_k = smo_estimator_B.i_p4 << 2;
      smo_estimator_B.Tinv[smo_estimator_B.b_k] = smo_estimator_B.R_j[3 *
        smo_estimator_B.i_p4];
      smo_estimator_B.Tinv[smo_estimator_B.b_k + 1] = smo_estimator_B.R_j[3 *
        smo_estimator_B.i_p4 + 1];
      smo_estimator_B.Tinv[smo_estimator_B.b_k + 2] = smo_estimator_B.R_j[3 *
        smo_estimator_B.i_p4 + 2];
      smo_estimator_B.Tinv[smo_estimator_B.i_p4 + 12] =
        smo_estimator_B.R_d[smo_estimator_B.i_p4 + 6] * smo_estimator_B.T[14] +
        (smo_estimator_B.R_d[smo_estimator_B.i_p4 + 3] * smo_estimator_B.T[13] +
         smo_estimator_B.R_d[smo_estimator_B.i_p4] * smo_estimator_B.T[12]);
    }

    smo_estimator_B.Tinv[3] = 0.0;
    smo_estimator_B.Tinv[7] = 0.0;
    smo_estimator_B.Tinv[11] = 0.0;
    smo_estimator_B.Tinv[15] = 1.0;
    smo_estimator_B.dv2[3] = -smo_estimator_B.Tinv[14];
    smo_estimator_B.dv2[6] = smo_estimator_B.Tinv[13];
    smo_estimator_B.dv2[1] = smo_estimator_B.Tinv[14];
    smo_estimator_B.dv2[7] = -smo_estimator_B.Tinv[12];
    smo_estimator_B.dv2[2] = -smo_estimator_B.Tinv[13];
    smo_estimator_B.dv2[5] = smo_estimator_B.Tinv[12];
    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 3;
         smo_estimator_B.i_p4++) {
      for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 3; smo_estimator_B.b_k
           ++) {
        smo_estimator_B.q_size_tmp = smo_estimator_B.i_p4 + 3 *
          smo_estimator_B.b_k;
        smo_estimator_B.R_j[smo_estimator_B.q_size_tmp] = 0.0;
        smo_estimator_B.p = smo_estimator_B.b_k << 2;
        smo_estimator_B.R_j[smo_estimator_B.q_size_tmp] +=
          smo_estimator_B.Tinv[smo_estimator_B.p] *
          smo_estimator_B.dv2[smo_estimator_B.i_p4];
        smo_estimator_B.R_j[smo_estimator_B.q_size_tmp] +=
          smo_estimator_B.Tinv[smo_estimator_B.p + 1] *
          smo_estimator_B.dv2[smo_estimator_B.i_p4 + 3];
        smo_estimator_B.R_j[smo_estimator_B.q_size_tmp] +=
          smo_estimator_B.Tinv[smo_estimator_B.p + 2] *
          smo_estimator_B.dv2[smo_estimator_B.i_p4 + 6];
        X->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.b_k + 6 *
          smo_estimator_B.i_p4] = smo_estimator_B.Tinv[(smo_estimator_B.i_p4 <<
          2) + smo_estimator_B.b_k];
        X->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.b_k + 6 *
          (smo_estimator_B.i_p4 + 3)] = 0.0;
      }
    }

    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 3;
         smo_estimator_B.i_p4++) {
      X->data[smo_estimator_B.unnamed_idx_1].f1[6 * smo_estimator_B.i_p4 + 3] =
        smo_estimator_B.R_j[3 * smo_estimator_B.i_p4];
      smo_estimator_B.b_k = smo_estimator_B.i_p4 << 2;
      smo_estimator_B.q_size_tmp = 6 * (smo_estimator_B.i_p4 + 3);
      X->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.q_size_tmp + 3] =
        smo_estimator_B.Tinv[smo_estimator_B.b_k];
      X->data[smo_estimator_B.unnamed_idx_1].f1[6 * smo_estimator_B.i_p4 + 4] =
        smo_estimator_B.R_j[3 * smo_estimator_B.i_p4 + 1];
      X->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.q_size_tmp + 4] =
        smo_estimator_B.Tinv[smo_estimator_B.b_k + 1];
      X->data[smo_estimator_B.unnamed_idx_1].f1[6 * smo_estimator_B.i_p4 + 5] =
        smo_estimator_B.R_j[3 * smo_estimator_B.i_p4 + 2];
      X->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.q_size_tmp + 5] =
        smo_estimator_B.Tinv[smo_estimator_B.b_k + 2];
    }

    smo_estimator_B.a_idx_0 = robot->Bodies[smo_estimator_B.unnamed_idx_1]
      ->ParentIndex;
    if (smo_estimator_B.a_idx_0 > 0.0) {
      smo_estimator_B.m = static_cast<int32_T>(smo_estimator_B.a_idx_0);
      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
           smo_estimator_B.i_p4++) {
        smo_estimator_B.a_idx_1 = 0.0;
        for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6;
             smo_estimator_B.b_k++) {
          smo_estimator_B.a_idx_1 += vB->data[(smo_estimator_B.m - 1) * 6 +
            smo_estimator_B.b_k] * X->data[smo_estimator_B.unnamed_idx_1].f1[6 *
            smo_estimator_B.b_k + smo_estimator_B.i_p4];
        }

        smo_estimator_B.y[smo_estimator_B.i_p4] = vJ->data[6 *
          smo_estimator_B.unnamed_idx_1 + smo_estimator_B.i_p4] +
          smo_estimator_B.a_idx_1;
      }

      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
           smo_estimator_B.i_p4++) {
        vB->data[smo_estimator_B.i_p4 + 6 * smo_estimator_B.unnamed_idx_1] =
          smo_estimator_B.y[smo_estimator_B.i_p4];
      }

      smo_estimator_B.inner = S->size[1] - 1;
      for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6; smo_estimator_B.b_k
           ++) {
        smo_estimator_B.y[smo_estimator_B.b_k] = 0.0;
      }

      for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k <= smo_estimator_B.inner;
           smo_estimator_B.b_k++) {
        smo_estimator_B.aoffset = smo_estimator_B.b_k * 6 - 1;
        for (smo_estimator_B.q_size_tmp = 0; smo_estimator_B.q_size_tmp < 6;
             smo_estimator_B.q_size_tmp++) {
          smo_estimator_B.a_idx_1 = S->data[(smo_estimator_B.aoffset +
            smo_estimator_B.q_size_tmp) + 1] * 0.0 +
            smo_estimator_B.y[smo_estimator_B.q_size_tmp];
          smo_estimator_B.y[smo_estimator_B.q_size_tmp] =
            smo_estimator_B.a_idx_1;
        }
      }

      smo_estimator_B.R_j[0] = 0.0;
      smo_estimator_B.b_k = 6 * smo_estimator_B.unnamed_idx_1 + 2;
      smo_estimator_B.R_j[3] = -vB->data[smo_estimator_B.b_k];
      smo_estimator_B.i_p4 = 6 * smo_estimator_B.unnamed_idx_1 + 1;
      smo_estimator_B.R_j[6] = vB->data[smo_estimator_B.i_p4];
      smo_estimator_B.R_j[1] = vB->data[smo_estimator_B.b_k];
      smo_estimator_B.R_j[4] = 0.0;
      smo_estimator_B.R_j[7] = -vB->data[6 * smo_estimator_B.unnamed_idx_1];
      smo_estimator_B.R_j[2] = -vB->data[smo_estimator_B.i_p4];
      smo_estimator_B.R_j[5] = vB->data[6 * smo_estimator_B.unnamed_idx_1];
      smo_estimator_B.R_j[8] = 0.0;
      smo_estimator_B.R[3] = 0.0;
      smo_estimator_B.b_k = 6 * smo_estimator_B.unnamed_idx_1 + 5;
      smo_estimator_B.R[9] = -vB->data[smo_estimator_B.b_k];
      smo_estimator_B.i_p4 = 6 * smo_estimator_B.unnamed_idx_1 + 4;
      smo_estimator_B.R[15] = vB->data[smo_estimator_B.i_p4];
      smo_estimator_B.R[4] = vB->data[smo_estimator_B.b_k];
      smo_estimator_B.R[10] = 0.0;
      smo_estimator_B.b_k = 6 * smo_estimator_B.unnamed_idx_1 + 3;
      smo_estimator_B.R[16] = -vB->data[smo_estimator_B.b_k];
      smo_estimator_B.R[5] = -vB->data[smo_estimator_B.i_p4];
      smo_estimator_B.R[11] = vB->data[smo_estimator_B.b_k];
      smo_estimator_B.R[17] = 0.0;
      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 3;
           smo_estimator_B.i_p4++) {
        smo_estimator_B.a_idx_1 = smo_estimator_B.R_j[3 * smo_estimator_B.i_p4];
        smo_estimator_B.R[6 * smo_estimator_B.i_p4] = smo_estimator_B.a_idx_1;
        smo_estimator_B.b_k = 6 * (smo_estimator_B.i_p4 + 3);
        smo_estimator_B.R[smo_estimator_B.b_k] = 0.0;
        smo_estimator_B.R[smo_estimator_B.b_k + 3] = smo_estimator_B.a_idx_1;
        smo_estimator_B.a_idx_1 = smo_estimator_B.R_j[3 * smo_estimator_B.i_p4 +
          1];
        smo_estimator_B.R[6 * smo_estimator_B.i_p4 + 1] =
          smo_estimator_B.a_idx_1;
        smo_estimator_B.R[smo_estimator_B.b_k + 1] = 0.0;
        smo_estimator_B.R[smo_estimator_B.b_k + 4] = smo_estimator_B.a_idx_1;
        smo_estimator_B.a_idx_1 = smo_estimator_B.R_j[3 * smo_estimator_B.i_p4 +
          2];
        smo_estimator_B.R[6 * smo_estimator_B.i_p4 + 2] =
          smo_estimator_B.a_idx_1;
        smo_estimator_B.R[smo_estimator_B.b_k + 2] = 0.0;
        smo_estimator_B.R[smo_estimator_B.b_k + 5] = smo_estimator_B.a_idx_1;
      }

      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
           smo_estimator_B.i_p4++) {
        smo_estimator_B.a_idx_1 = 0.0;
        for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6;
             smo_estimator_B.b_k++) {
          smo_estimator_B.a_idx_1 += aB->data[(smo_estimator_B.m - 1) * 6 +
            smo_estimator_B.b_k] * X->data[smo_estimator_B.unnamed_idx_1].f1[6 *
            smo_estimator_B.b_k + smo_estimator_B.i_p4];
        }

        smo_estimator_B.X_e[smo_estimator_B.i_p4] = smo_estimator_B.a_idx_1 +
          smo_estimator_B.y[smo_estimator_B.i_p4];
      }

      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
           smo_estimator_B.i_p4++) {
        smo_estimator_B.y[smo_estimator_B.i_p4] = 0.0;
        for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6;
             smo_estimator_B.b_k++) {
          smo_estimator_B.a_idx_1 = smo_estimator_B.R[6 * smo_estimator_B.b_k +
            smo_estimator_B.i_p4] * vJ->data[6 * smo_estimator_B.unnamed_idx_1 +
            smo_estimator_B.b_k] + smo_estimator_B.y[smo_estimator_B.i_p4];
          smo_estimator_B.y[smo_estimator_B.i_p4] = smo_estimator_B.a_idx_1;
        }

        aB->data[smo_estimator_B.i_p4 + 6 * smo_estimator_B.unnamed_idx_1] =
          smo_estimator_B.X_e[smo_estimator_B.i_p4] +
          smo_estimator_B.y[smo_estimator_B.i_p4];
      }

      smo_estimator_B.R_j[0] = 0.0;
      smo_estimator_B.R_j[3] = -smo_estimator_B.T[14];
      smo_estimator_B.R_j[6] = smo_estimator_B.T[13];
      smo_estimator_B.R_j[1] = smo_estimator_B.T[14];
      smo_estimator_B.R_j[4] = 0.0;
      smo_estimator_B.R_j[7] = -smo_estimator_B.T[12];
      smo_estimator_B.R_j[2] = -smo_estimator_B.T[13];
      smo_estimator_B.R_j[5] = smo_estimator_B.T[12];
      smo_estimator_B.R_j[8] = 0.0;
      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 3;
           smo_estimator_B.i_p4++) {
        for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 3;
             smo_estimator_B.b_k++) {
          smo_estimator_B.q_size_tmp = smo_estimator_B.i_p4 + 3 *
            smo_estimator_B.b_k;
          smo_estimator_B.R_d[smo_estimator_B.q_size_tmp] = 0.0;
          smo_estimator_B.p = smo_estimator_B.b_k << 2;
          smo_estimator_B.R_d[smo_estimator_B.q_size_tmp] +=
            smo_estimator_B.T[smo_estimator_B.p] *
            smo_estimator_B.R_j[smo_estimator_B.i_p4];
          smo_estimator_B.R_d[smo_estimator_B.q_size_tmp] +=
            smo_estimator_B.T[smo_estimator_B.p + 1] *
            smo_estimator_B.R_j[smo_estimator_B.i_p4 + 3];
          smo_estimator_B.R_d[smo_estimator_B.q_size_tmp] +=
            smo_estimator_B.T[smo_estimator_B.p + 2] *
            smo_estimator_B.R_j[smo_estimator_B.i_p4 + 6];
          smo_estimator_B.R[smo_estimator_B.b_k + 6 * smo_estimator_B.i_p4] =
            smo_estimator_B.T[(smo_estimator_B.i_p4 << 2) + smo_estimator_B.b_k];
          smo_estimator_B.R[smo_estimator_B.b_k + 6 * (smo_estimator_B.i_p4 + 3)]
            = 0.0;
        }
      }

      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 3;
           smo_estimator_B.i_p4++) {
        smo_estimator_B.R[6 * smo_estimator_B.i_p4 + 3] = smo_estimator_B.R_d[3 *
          smo_estimator_B.i_p4];
        smo_estimator_B.b_k = smo_estimator_B.i_p4 << 2;
        smo_estimator_B.q_size_tmp = 6 * (smo_estimator_B.i_p4 + 3);
        smo_estimator_B.R[smo_estimator_B.q_size_tmp + 3] =
          smo_estimator_B.T[smo_estimator_B.b_k];
        smo_estimator_B.R[6 * smo_estimator_B.i_p4 + 4] = smo_estimator_B.R_d[3 *
          smo_estimator_B.i_p4 + 1];
        smo_estimator_B.R[smo_estimator_B.q_size_tmp + 4] =
          smo_estimator_B.T[smo_estimator_B.b_k + 1];
        smo_estimator_B.R[6 * smo_estimator_B.i_p4 + 5] = smo_estimator_B.R_d[3 *
          smo_estimator_B.i_p4 + 2];
        smo_estimator_B.R[smo_estimator_B.q_size_tmp + 5] =
          smo_estimator_B.T[smo_estimator_B.b_k + 2];
      }

      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
           smo_estimator_B.i_p4++) {
        for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6;
             smo_estimator_B.b_k++) {
          smo_estimator_B.p = smo_estimator_B.i_p4 + 6 * smo_estimator_B.b_k;
          smo_estimator_B.b_I[smo_estimator_B.p] = 0.0;
          for (smo_estimator_B.q_size_tmp = 0; smo_estimator_B.q_size_tmp < 6;
               smo_estimator_B.q_size_tmp++) {
            smo_estimator_B.b_I[smo_estimator_B.p] += Xtree->data
              [static_cast<int32_T>(smo_estimator_B.a_idx_0) - 1].f1[6 *
              smo_estimator_B.q_size_tmp + smo_estimator_B.i_p4] *
              smo_estimator_B.R[6 * smo_estimator_B.b_k +
              smo_estimator_B.q_size_tmp];
          }
        }
      }

      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 36;
           smo_estimator_B.i_p4++) {
        Xtree->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.i_p4] =
          smo_estimator_B.b_I[smo_estimator_B.i_p4];
      }
    } else {
      smo_estimator_B.inner = S->size[1] - 1;
      for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6; smo_estimator_B.b_k
           ++) {
        smo_estimator_B.i_p4 = 6 * smo_estimator_B.unnamed_idx_1 +
          smo_estimator_B.b_k;
        vB->data[smo_estimator_B.i_p4] = vJ->data[smo_estimator_B.i_p4];
        smo_estimator_B.y[smo_estimator_B.b_k] = 0.0;
      }

      for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k <= smo_estimator_B.inner;
           smo_estimator_B.b_k++) {
        smo_estimator_B.aoffset = smo_estimator_B.b_k * 6 - 1;
        for (smo_estimator_B.q_size_tmp = 0; smo_estimator_B.q_size_tmp < 6;
             smo_estimator_B.q_size_tmp++) {
          smo_estimator_B.a_idx_1 = S->data[(smo_estimator_B.aoffset +
            smo_estimator_B.q_size_tmp) + 1] * 0.0 +
            smo_estimator_B.y[smo_estimator_B.q_size_tmp];
          smo_estimator_B.y[smo_estimator_B.q_size_tmp] =
            smo_estimator_B.a_idx_1;
        }
      }

      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
           smo_estimator_B.i_p4++) {
        smo_estimator_B.a_idx_1 = 0.0;
        for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6;
             smo_estimator_B.b_k++) {
          smo_estimator_B.a_idx_1 += X->data[smo_estimator_B.unnamed_idx_1].f1[6
            * smo_estimator_B.b_k + smo_estimator_B.i_p4] *
            smo_estimator_B.a0[smo_estimator_B.b_k];
        }

        aB->data[smo_estimator_B.i_p4 + 6 * smo_estimator_B.unnamed_idx_1] =
          smo_estimator_B.a_idx_1 + smo_estimator_B.y[smo_estimator_B.i_p4];
      }

      smo_estimator_B.R_j[0] = 0.0;
      smo_estimator_B.R_j[3] = -smo_estimator_B.T[14];
      smo_estimator_B.R_j[6] = smo_estimator_B.T[13];
      smo_estimator_B.R_j[1] = smo_estimator_B.T[14];
      smo_estimator_B.R_j[4] = 0.0;
      smo_estimator_B.R_j[7] = -smo_estimator_B.T[12];
      smo_estimator_B.R_j[2] = -smo_estimator_B.T[13];
      smo_estimator_B.R_j[5] = smo_estimator_B.T[12];
      smo_estimator_B.R_j[8] = 0.0;
      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 3;
           smo_estimator_B.i_p4++) {
        for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 3;
             smo_estimator_B.b_k++) {
          smo_estimator_B.q_size_tmp = smo_estimator_B.i_p4 + 3 *
            smo_estimator_B.b_k;
          smo_estimator_B.R_d[smo_estimator_B.q_size_tmp] = 0.0;
          smo_estimator_B.p = smo_estimator_B.b_k << 2;
          smo_estimator_B.R_d[smo_estimator_B.q_size_tmp] +=
            smo_estimator_B.T[smo_estimator_B.p] *
            smo_estimator_B.R_j[smo_estimator_B.i_p4];
          smo_estimator_B.R_d[smo_estimator_B.q_size_tmp] +=
            smo_estimator_B.T[smo_estimator_B.p + 1] *
            smo_estimator_B.R_j[smo_estimator_B.i_p4 + 3];
          smo_estimator_B.R_d[smo_estimator_B.q_size_tmp] +=
            smo_estimator_B.T[smo_estimator_B.p + 2] *
            smo_estimator_B.R_j[smo_estimator_B.i_p4 + 6];
          Xtree->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.b_k + 6 *
            smo_estimator_B.i_p4] = smo_estimator_B.T[(smo_estimator_B.i_p4 << 2)
            + smo_estimator_B.b_k];
          Xtree->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.b_k + 6 *
            (smo_estimator_B.i_p4 + 3)] = 0.0;
        }
      }

      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 3;
           smo_estimator_B.i_p4++) {
        Xtree->data[smo_estimator_B.unnamed_idx_1].f1[6 * smo_estimator_B.i_p4 +
          3] = smo_estimator_B.R_d[3 * smo_estimator_B.i_p4];
        smo_estimator_B.b_k = smo_estimator_B.i_p4 << 2;
        smo_estimator_B.q_size_tmp = 6 * (smo_estimator_B.i_p4 + 3);
        Xtree->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.q_size_tmp
          + 3] = smo_estimator_B.T[smo_estimator_B.b_k];
        Xtree->data[smo_estimator_B.unnamed_idx_1].f1[6 * smo_estimator_B.i_p4 +
          4] = smo_estimator_B.R_d[3 * smo_estimator_B.i_p4 + 1];
        Xtree->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.q_size_tmp
          + 4] = smo_estimator_B.T[smo_estimator_B.b_k + 1];
        Xtree->data[smo_estimator_B.unnamed_idx_1].f1[6 * smo_estimator_B.i_p4 +
          5] = smo_estimator_B.R_d[3 * smo_estimator_B.i_p4 + 2];
        Xtree->data[smo_estimator_B.unnamed_idx_1].f1[smo_estimator_B.q_size_tmp
          + 5] = smo_estimator_B.T[smo_estimator_B.b_k + 2];
      }
    }

    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 36;
         smo_estimator_B.i_p4++) {
      smo_estimator_B.b_I[smo_estimator_B.i_p4] = robot->
        Bodies[smo_estimator_B.unnamed_idx_1]->
        SpatialInertia[smo_estimator_B.i_p4];
    }

    smo_estimator_B.R_j[0] = 0.0;
    smo_estimator_B.b_k = 6 * smo_estimator_B.unnamed_idx_1 + 2;
    smo_estimator_B.R_j[3] = -vB->data[smo_estimator_B.b_k];
    smo_estimator_B.i_p4 = 6 * smo_estimator_B.unnamed_idx_1 + 1;
    smo_estimator_B.R_j[6] = vB->data[smo_estimator_B.i_p4];
    smo_estimator_B.R_j[1] = vB->data[smo_estimator_B.b_k];
    smo_estimator_B.R_j[4] = 0.0;
    smo_estimator_B.R_j[7] = -vB->data[6 * smo_estimator_B.unnamed_idx_1];
    smo_estimator_B.R_j[2] = -vB->data[smo_estimator_B.i_p4];
    smo_estimator_B.R_j[5] = vB->data[6 * smo_estimator_B.unnamed_idx_1];
    smo_estimator_B.R_j[8] = 0.0;
    smo_estimator_B.R[18] = 0.0;
    smo_estimator_B.b_k = 6 * smo_estimator_B.unnamed_idx_1 + 5;
    smo_estimator_B.R[24] = -vB->data[smo_estimator_B.b_k];
    smo_estimator_B.i_p4 = 6 * smo_estimator_B.unnamed_idx_1 + 4;
    smo_estimator_B.R[30] = vB->data[smo_estimator_B.i_p4];
    smo_estimator_B.R[19] = vB->data[smo_estimator_B.b_k];
    smo_estimator_B.R[25] = 0.0;
    smo_estimator_B.b_k = 6 * smo_estimator_B.unnamed_idx_1 + 3;
    smo_estimator_B.R[31] = -vB->data[smo_estimator_B.b_k];
    smo_estimator_B.R[20] = -vB->data[smo_estimator_B.i_p4];
    smo_estimator_B.R[26] = vB->data[smo_estimator_B.b_k];
    smo_estimator_B.R[32] = 0.0;
    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 3;
         smo_estimator_B.i_p4++) {
      smo_estimator_B.a_idx_1 = smo_estimator_B.R_j[3 * smo_estimator_B.i_p4];
      smo_estimator_B.R[6 * smo_estimator_B.i_p4] = smo_estimator_B.a_idx_1;
      smo_estimator_B.R[6 * smo_estimator_B.i_p4 + 3] = 0.0;
      smo_estimator_B.b_k = 6 * (smo_estimator_B.i_p4 + 3);
      smo_estimator_B.R[smo_estimator_B.b_k + 3] = smo_estimator_B.a_idx_1;
      smo_estimator_B.a_idx_1 = smo_estimator_B.R_j[3 * smo_estimator_B.i_p4 + 1];
      smo_estimator_B.R[6 * smo_estimator_B.i_p4 + 1] = smo_estimator_B.a_idx_1;
      smo_estimator_B.R[6 * smo_estimator_B.i_p4 + 4] = 0.0;
      smo_estimator_B.R[smo_estimator_B.b_k + 4] = smo_estimator_B.a_idx_1;
      smo_estimator_B.a_idx_1 = smo_estimator_B.R_j[3 * smo_estimator_B.i_p4 + 2];
      smo_estimator_B.R[6 * smo_estimator_B.i_p4 + 2] = smo_estimator_B.a_idx_1;
      smo_estimator_B.R[6 * smo_estimator_B.i_p4 + 5] = 0.0;
      smo_estimator_B.R[smo_estimator_B.b_k + 5] = smo_estimator_B.a_idx_1;
    }

    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
         smo_estimator_B.i_p4++) {
      smo_estimator_B.X_e[smo_estimator_B.i_p4] = 0.0;
      smo_estimator_B.b_I_b[smo_estimator_B.i_p4] = 0.0;
      for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6; smo_estimator_B.b_k
           ++) {
        smo_estimator_B.a_idx_0 = smo_estimator_B.b_I[6 * smo_estimator_B.b_k +
          smo_estimator_B.i_p4];
        smo_estimator_B.q_size_tmp = 6 * smo_estimator_B.unnamed_idx_1 +
          smo_estimator_B.b_k;
        smo_estimator_B.a_idx_1 = vB->data[smo_estimator_B.q_size_tmp] *
          smo_estimator_B.a_idx_0 + smo_estimator_B.X_e[smo_estimator_B.i_p4];
        smo_estimator_B.a_idx_0 = aB->data[smo_estimator_B.q_size_tmp] *
          smo_estimator_B.a_idx_0 + smo_estimator_B.b_I_b[smo_estimator_B.i_p4];
        smo_estimator_B.X_e[smo_estimator_B.i_p4] = smo_estimator_B.a_idx_1;
        smo_estimator_B.b_I_b[smo_estimator_B.i_p4] = smo_estimator_B.a_idx_0;
      }
    }

    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
         smo_estimator_B.i_p4++) {
      smo_estimator_B.y[smo_estimator_B.i_p4] = 0.0;
      smo_estimator_B.a_idx_1 = 0.0;
      for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6; smo_estimator_B.b_k
           ++) {
        smo_estimator_B.a_idx_1 += Xtree->data[smo_estimator_B.unnamed_idx_1]
          .f1[6 * smo_estimator_B.i_p4 + smo_estimator_B.b_k] * fext[6 *
          smo_estimator_B.unnamed_idx_1 + smo_estimator_B.b_k];
        smo_estimator_B.y[smo_estimator_B.i_p4] += smo_estimator_B.R[6 *
          smo_estimator_B.b_k + smo_estimator_B.i_p4] *
          smo_estimator_B.X_e[smo_estimator_B.b_k];
      }

      f->data[smo_estimator_B.i_p4 + 6 * smo_estimator_B.unnamed_idx_1] =
        (smo_estimator_B.b_I_b[smo_estimator_B.i_p4] +
         smo_estimator_B.y[smo_estimator_B.i_p4]) - smo_estimator_B.a_idx_1;
    }
  }

  smo_estimator_emxFree_real_T(&aB);
  smo_estimator_emxFree_real_T(&vB);
  smo_estimator_emxFree_real_T(&vJ);
  smo_estima_emxFree_f_cell_wrap1(&Xtree);
  smo_estimator_B.q_size_tmp = static_cast<int32_T>(((-1.0 - smo_estimator_B.nb)
    + 1.0) / -1.0) - 1;
  smo_estimator_emxInit_real_T(&taui, 1);
  smo_estimator_emxInit_char_T(&a, 2);
  if (0 <= smo_estimator_B.q_size_tmp) {
    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 5;
         smo_estimator_B.i_p4++) {
      smo_estimator_B.b_l[smo_estimator_B.i_p4] = tmp[smo_estimator_B.i_p4];
    }
  }

  for (smo_estimator_B.loop_ub_tmp = 0; smo_estimator_B.loop_ub_tmp <=
       smo_estimator_B.q_size_tmp; smo_estimator_B.loop_ub_tmp++) {
    smo_estimator_B.a_idx_0 = smo_estimator_B.nb + -static_cast<real_T>
      (smo_estimator_B.loop_ub_tmp);
    smo_estimator_B.p = static_cast<int32_T>(smo_estimator_B.a_idx_0);
    smo_estimator_B.inner = smo_estimator_B.p - 1;
    obj = robot->Bodies[smo_estimator_B.inner];
    smo_estimator_B.i_p4 = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    smo_es_emxEnsureCapacity_char_T(a, smo_estimator_B.i_p4);
    smo_estimator_B.b_k = obj->JointInternal.Type->size[0] *
      obj->JointInternal.Type->size[1] - 1;
    for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 <= smo_estimator_B.b_k;
         smo_estimator_B.i_p4++) {
      a->data[smo_estimator_B.i_p4] = obj->JointInternal.Type->
        data[smo_estimator_B.i_p4];
    }

    smo_estimator_B.b_bool = false;
    if (a->size[1] == 5) {
      smo_estimator_B.i_p4 = 1;
      do {
        exitg1 = 0;
        if (smo_estimator_B.i_p4 - 1 < 5) {
          smo_estimator_B.unnamed_idx_1 = smo_estimator_B.i_p4 - 1;
          if (a->data[smo_estimator_B.unnamed_idx_1] !=
              smo_estimator_B.b_l[smo_estimator_B.unnamed_idx_1]) {
            exitg1 = 1;
          } else {
            smo_estimator_B.i_p4++;
          }
        } else {
          smo_estimator_B.b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!smo_estimator_B.b_bool) {
      obj = robot->Bodies[smo_estimator_B.inner];
      smo_estimator_B.i_p4 = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      smo_es_emxEnsureCapacity_real_T(S, smo_estimator_B.i_p4);
      smo_estimator_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 <= smo_estimator_B.b_k;
           smo_estimator_B.i_p4++) {
        S->data[smo_estimator_B.i_p4] = obj->JointInternal.MotionSubspace->
          data[smo_estimator_B.i_p4];
      }

      smo_estimator_B.m = S->size[1] - 1;
      smo_estimator_B.i_p4 = taui->size[0];
      taui->size[0] = S->size[1];
      smo_es_emxEnsureCapacity_real_T(taui, smo_estimator_B.i_p4);
      for (smo_estimator_B.unnamed_idx_1 = 0; smo_estimator_B.unnamed_idx_1 <=
           smo_estimator_B.m; smo_estimator_B.unnamed_idx_1++) {
        smo_estimator_B.aoffset = smo_estimator_B.unnamed_idx_1 * 6 - 1;
        smo_estimator_B.a_idx_1 = 0.0;
        for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6;
             smo_estimator_B.b_k++) {
          smo_estimator_B.a_idx_1 += f->data[(static_cast<int32_T>
            (smo_estimator_B.a_idx_0) - 1) * 6 + smo_estimator_B.b_k] * S->data
            [(smo_estimator_B.aoffset + smo_estimator_B.b_k) + 1];
        }

        taui->data[smo_estimator_B.unnamed_idx_1] = smo_estimator_B.a_idx_1;
      }

      smo_estimator_B.b_idx_0 = robot->VelocityDoFMap[smo_estimator_B.p - 1];
      smo_estimator_B.b_idx_1 = robot->VelocityDoFMap[smo_estimator_B.p + 9];
      if (smo_estimator_B.b_idx_0 > smo_estimator_B.b_idx_1) {
        smo_estimator_B.b_k = 0;
        smo_estimator_B.i_p4 = 0;
      } else {
        smo_estimator_B.b_k = static_cast<int32_T>(smo_estimator_B.b_idx_0) - 1;
        smo_estimator_B.i_p4 = static_cast<int32_T>(smo_estimator_B.b_idx_1);
      }

      smo_estimator_B.unnamed_idx_1 = smo_estimator_B.i_p4 - smo_estimator_B.b_k;
      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 <
           smo_estimator_B.unnamed_idx_1; smo_estimator_B.i_p4++) {
        tau[smo_estimator_B.b_k + smo_estimator_B.i_p4] = taui->
          data[smo_estimator_B.i_p4];
      }
    }

    smo_estimator_B.a_idx_0 = robot->Bodies[smo_estimator_B.inner]->ParentIndex;
    if (smo_estimator_B.a_idx_0 > 0.0) {
      smo_estimator_B.m = static_cast<int32_T>(smo_estimator_B.a_idx_0);
      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
           smo_estimator_B.i_p4++) {
        smo_estimator_B.a_idx_1 = 0.0;
        for (smo_estimator_B.b_k = 0; smo_estimator_B.b_k < 6;
             smo_estimator_B.b_k++) {
          smo_estimator_B.a_idx_1 += f->data[(smo_estimator_B.p - 1) * 6 +
            smo_estimator_B.b_k] * X->data[smo_estimator_B.inner].f1[6 *
            smo_estimator_B.i_p4 + smo_estimator_B.b_k];
        }

        smo_estimator_B.a0[smo_estimator_B.i_p4] = f->data[(smo_estimator_B.m -
          1) * 6 + smo_estimator_B.i_p4] + smo_estimator_B.a_idx_1;
      }

      for (smo_estimator_B.i_p4 = 0; smo_estimator_B.i_p4 < 6;
           smo_estimator_B.i_p4++) {
        f->data[smo_estimator_B.i_p4 + 6 * (smo_estimator_B.m - 1)] =
          smo_estimator_B.a0[smo_estimator_B.i_p4];
      }
    }
  }

  smo_estimator_emxFree_char_T(&a);
  smo_estimator_emxFree_real_T(&taui);
  smo_estimator_emxFree_real_T(&S);
  smo_estimator_emxFree_real_T(&f);
  smo_estima_emxFree_f_cell_wrap1(&X);
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

static void smo_estimator_atan2(const real_T y_data[], const int32_T y_size[3],
  const real_T x_data[], const int32_T x_size[3], real_T r_data[], int32_T
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

static void matlabCodegenHandle_matla_mrfu4(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_smo_estima_m_T
  *pStruct)
{
  smo_estimator_emxFree_char_T(&pStruct->Type);
  smo_estimator_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_p_robotics_manip_(p_robotics_manip_internal_R_m_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_q_robotics_manip_(q_robotics_manip_internal_R_m_T
  *pStruct)
{
  emxFreeStruct_p_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_b_m_T
  *pStruct)
{
  emxFreeStruct_q_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_o_robotics_manip_(o_robotics_manip_internal_R_m_T
  *pStruct)
{
  emxFreeStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_smo_estimato_T
  *pStruct)
{
  smo_estimator_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_p_robotics_mani_m(p_robotics_manip_internal_Rig_T
  *pStruct)
{
  smo_estimator_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_q_robotics_mani_m(q_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_p_robotics_mani_m(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_m(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_q_robotics_mani_m(&pStruct->TreeInternal);
}

static void emxFreeStruct_o_robotics_mani_m(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  smo_estimator_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxFreeStruct_q_robotics_man_mr(q_robotics_manip_internal__mr_T
  *pStruct)
{
  emxFreeStruct_p_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slman_mr(robotics_slmanip_internal__mr_T
  *pStruct)
{
  emxFreeStruct_q_robotics_man_mr(&pStruct->TreeInternal);
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabC_mrf(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_smo_estima_m_T
  *pStruct)
{
  smo_estimator_emxInit_char_T(&pStruct->Type, 2);
  smo_estimator_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_p_robotics_manip_(p_robotics_manip_internal_R_m_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxInitStruct_q_robotics_manip_(q_robotics_manip_internal_R_m_T
  *pStruct)
{
  emxInitStruct_p_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_b_m_T
  *pStruct)
{
  emxInitStruct_q_robotics_manip_(&pStruct->TreeInternal);
}

static void emxInitStruct_o_robotics_manip_(o_robotics_manip_internal_R_m_T
  *pStruct)
{
  emxInitStruct_c_rigidBodyJoint(&pStruct->JointInternal);
}

static o_robotics_manip_internal_R_m_T *smo_RigidBody_RigidBody_mrfu4kk
  (o_robotics_manip_internal_R_m_T *obj)
{
  o_robotics_manip_internal_R_m_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_R_m_T *sm_RigidBody_RigidBody_mrfu4kk1
  (o_robotics_manip_internal_R_m_T *obj)
{
  o_robotics_manip_internal_R_m_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = -1.0;
  return b_obj;
}

static o_robotics_manip_internal_R_m_T *s_RigidBody_RigidBody_mrfu4kk1b
  (o_robotics_manip_internal_R_m_T *obj)
{
  o_robotics_manip_internal_R_m_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_R_m_T *RigidBody_RigidBody_mrfu4kk1bh
  (o_robotics_manip_internal_R_m_T *obj)
{
  o_robotics_manip_internal_R_m_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_R_m_T *RigidBody_RigidBody_mrfu4kk1bht
  (o_robotics_manip_internal_R_m_T *obj)
{
  o_robotics_manip_internal_R_m_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_R_m_T *RigidBody_RigidBod_mrfu4kk1bhtk
  (o_robotics_manip_internal_R_m_T *obj)
{
  o_robotics_manip_internal_R_m_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = -1.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_R_m_T *RigidBody_RigidBo_mrfu4kk1bhtk3
  (o_robotics_manip_internal_R_m_T *obj)
{
  o_robotics_manip_internal_R_m_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = tmp_5[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 1.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_R_m_T *RigidBody_RigidB_mrfu4kk1bhtk3a
  (o_robotics_manip_internal_R_m_T *obj)
{
  o_robotics_manip_internal_R_m_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_R_m_T *RigidBody_Rigid_mrfu4kk1bhtk3a2
  (o_robotics_manip_internal_R_m_T *obj)
{
  o_robotics_manip_internal_R_m_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static o_robotics_manip_internal_R_m_T *s_RigidBody_Rigid_h
  (o_robotics_manip_internal_R_m_T *obj)
{
  o_robotics_manip_internal_R_m_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_1[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  return b_obj;
}

static p_robotics_manip_internal_R_m_T *s_RigidBody_Rigid_o
  (p_robotics_manip_internal_R_m_T *obj)
{
  p_robotics_manip_internal_R_m_T *b_obj;
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      smo_estimator_B.msubspace_data[b_kstr] = tmp[b_kstr];
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
      smo_estimator_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      smo_estimator_B.msubspace_data[b_kstr] = 0;
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
  smo_es_emxEnsureCapacity_real_T(obj->JointInternal.MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal.MotionSubspace->data[b_kstr] =
      smo_estimator_B.msubspace_data[b_kstr];
  }

  return b_obj;
}

static q_robotics_manip_internal_R_m_T *s_RigidBodyTree_RigidBodyTree_m
  (q_robotics_manip_internal_R_m_T *obj, o_robotics_manip_internal_R_m_T *iobj_0,
   o_robotics_manip_internal_R_m_T *iobj_1, o_robotics_manip_internal_R_m_T
   *iobj_2, o_robotics_manip_internal_R_m_T *iobj_3,
   o_robotics_manip_internal_R_m_T *iobj_4, o_robotics_manip_internal_R_m_T
   *iobj_5, o_robotics_manip_internal_R_m_T *iobj_6,
   o_robotics_manip_internal_R_m_T *iobj_7, o_robotics_manip_internal_R_m_T
   *iobj_8, o_robotics_manip_internal_R_m_T *iobj_9)
{
  q_robotics_manip_internal_R_m_T *b_obj;
  int32_T i;
  static const int8_T tmp[20] = { 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 1, 2, 3, 4, 5, 6,
    7, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = smo_RigidBody_RigidBody_mrfu4kk(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = sm_RigidBody_RigidBody_mrfu4kk1(iobj_9);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = s_RigidBody_RigidBody_mrfu4kk1b(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = RigidBody_RigidBody_mrfu4kk1bh(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = RigidBody_RigidBody_mrfu4kk1bht(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = RigidBody_RigidBod_mrfu4kk1bhtk(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = RigidBody_RigidBo_mrfu4kk1bhtk3(iobj_5);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = RigidBody_RigidB_mrfu4kk1bhtk3a(iobj_6);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = RigidBody_Rigid_mrfu4kk1bhtk3a2(iobj_7);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = s_RigidBody_Rigid_h(iobj_8);
  obj->Bodies[9]->Index = 10.0;
  obj->NumBodies = 10.0;
  obj->VelocityNumber = 7.0;
  for (i = 0; i < 20; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  for (i = 0; i < 20; i++) {
    obj->VelocityDoFMap[i] = tmp[i];
  }

  s_RigidBody_Rigid_o(&obj->Base);
  return b_obj;
}

static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_smo_estimato_T
  *pStruct)
{
  smo_estimator_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInitStruct_p_robotics_mani_m(p_robotics_manip_internal_Rig_T
  *pStruct)
{
  smo_estimator_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static void emxInitStruct_q_robotics_mani_m(q_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_p_robotics_mani_m(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_m(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_q_robotics_mani_m(&pStruct->TreeInternal);
}

static void emxInitStruct_o_robotics_mani_m(o_robotics_manip_internal_Rig_T
  *pStruct)
{
  smo_estimator_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
}

static o_robotics_manip_internal_Rig_T *smo_estimat_RigidBody_RigidBody
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 0.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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

static o_robotics_manip_internal_Rig_T *smo_estim_RigidBody_RigidBody_m
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 1.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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

static o_robotics_manip_internal_Rig_T *smo_esti_RigidBody_RigidBody_mr
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 2.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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

static o_robotics_manip_internal_Rig_T *smo_est_RigidBody_RigidBody_mrf
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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

static o_robotics_manip_internal_Rig_T *smo_es_RigidBody_RigidBody_mrfu
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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

static o_robotics_manip_internal_Rig_T *smo_e_RigidBody_RigidBody_mrfu4
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 5.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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

static o_robotics_manip_internal_Rig_T *smo__RigidBody_RigidBody_mrfu4k
  (o_robotics_manip_internal_Rig_T *obj)
{
  o_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  smo_es_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 6.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  smo_es_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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

static q_robotics_manip_internal_Rig_T *smo_RigidBodyTree_RigidBodyTree
  (q_robotics_manip_internal_Rig_T *obj, o_robotics_manip_internal_Rig_T *iobj_0,
   o_robotics_manip_internal_Rig_T *iobj_1, o_robotics_manip_internal_Rig_T
   *iobj_2, o_robotics_manip_internal_Rig_T *iobj_3,
   o_robotics_manip_internal_Rig_T *iobj_4, o_robotics_manip_internal_Rig_T
   *iobj_5, o_robotics_manip_internal_Rig_T *iobj_6,
   o_robotics_manip_internal_Rig_T *iobj_7, o_robotics_manip_internal_Rig_T
   *iobj_8, o_robotics_manip_internal_Rig_T *iobj_9)
{
  q_robotics_manip_internal_Rig_T *b_obj;
  p_robotics_manip_internal_Rig_T *obj_0;
  emxArray_char_T_smo_estimator_T *switch_expression;
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
  obj->Bodies[0] = smo_estimat_RigidBody_RigidBody(iobj_0);
  obj->Bodies[1] = smo_estim_RigidBody_RigidBody_m(iobj_9);
  obj->Bodies[2] = smo_esti_RigidBody_RigidBody_mr(iobj_1);
  obj->Bodies[3] = smo_est_RigidBody_RigidBody_mrf(iobj_2);
  obj->Bodies[4] = smo_es_RigidBody_RigidBody_mrfu(iobj_3);
  obj->Bodies[5] = smo_e_RigidBody_RigidBody_mrfu4(iobj_4);
  obj->Bodies[6] = smo__RigidBody_RigidBody_mrfu4k(iobj_5);
  b_kstr = iobj_6->NameInternal->size[0] * iobj_6->NameInternal->size[1];
  iobj_6->NameInternal->size[0] = 1;
  iobj_6->NameInternal->size[1] = 20;
  smo_es_emxEnsureCapacity_char_T(iobj_6->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 20; b_kstr++) {
    iobj_6->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  iobj_6->ParentIndex = 7.0;
  b_kstr = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1];
  iobj_6->JointInternal.Type->size[0] = 1;
  iobj_6->JointInternal.Type->size[1] = 5;
  smo_es_emxEnsureCapacity_char_T(iobj_6->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_6->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  smo_estimator_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_6->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_6->JointInternal.Type->size[0] * iobj_6->
    JointInternal.Type->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_6->JointInternal.Type->data[b_kstr];
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
    iobj_6->JointInternal.PositionNumber = 1.0;
    iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_6->JointInternal.PositionNumber = 1.0;
    iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_6->JointInternal.PositionNumber = 0.0;
    iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
    iobj_6->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_6->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_6->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  iobj_6->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_6->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_6->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[7] = iobj_6;
  b_kstr = iobj_7->NameInternal->size[0] * iobj_7->NameInternal->size[1];
  iobj_7->NameInternal->size[0] = 1;
  iobj_7->NameInternal->size[1] = 18;
  smo_es_emxEnsureCapacity_char_T(iobj_7->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 18; b_kstr++) {
    iobj_7->NameInternal->data[b_kstr] = tmp_5[b_kstr];
  }

  iobj_7->ParentIndex = 8.0;
  b_kstr = iobj_7->JointInternal.Type->size[0] * iobj_7->
    JointInternal.Type->size[1];
  iobj_7->JointInternal.Type->size[0] = 1;
  iobj_7->JointInternal.Type->size[1] = 5;
  smo_es_emxEnsureCapacity_char_T(iobj_7->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_7->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_7->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
    iobj_7->JointInternal.JointToParentTransform[b_kstr] = tmp_6[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_7->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  iobj_7->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_7->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_7->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[8] = iobj_7;
  b_kstr = iobj_8->NameInternal->size[0] * iobj_8->NameInternal->size[1];
  iobj_8->NameInternal->size[0] = 1;
  iobj_8->NameInternal->size[1] = 19;
  smo_es_emxEnsureCapacity_char_T(iobj_8->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 19; b_kstr++) {
    iobj_8->NameInternal->data[b_kstr] = tmp_7[b_kstr];
  }

  iobj_8->ParentIndex = 8.0;
  b_kstr = iobj_8->JointInternal.Type->size[0] * iobj_8->
    JointInternal.Type->size[1];
  iobj_8->JointInternal.Type->size[0] = 1;
  iobj_8->JointInternal.Type->size[1] = 5;
  smo_es_emxEnsureCapacity_char_T(iobj_8->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_8->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_8->JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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
    iobj_8->JointInternal.JointToParentTransform[b_kstr] = tmp_8[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    iobj_8->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  iobj_8->JointInternal.JointAxisInternal[0] = 0.0;
  iobj_8->JointInternal.JointAxisInternal[1] = 0.0;
  iobj_8->JointInternal.JointAxisInternal[2] = 0.0;
  obj->Bodies[9] = iobj_8;
  obj->NumBodies = 10.0;
  obj->PositionNumber = 7.0;
  obj_0 = &obj->Base;
  b_kstr = obj->Base.NameInternal->size[0] * obj->Base.NameInternal->size[1];
  obj->Base.NameInternal->size[0] = 1;
  obj->Base.NameInternal->size[1] = 19;
  smo_es_emxEnsureCapacity_char_T(obj->Base.NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 19; b_kstr++) {
    obj->Base.NameInternal->data[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->Base.JointInternal.Type->size[0] * obj->
    Base.JointInternal.Type->size[1];
  obj->Base.JointInternal.Type->size[0] = 1;
  obj->Base.JointInternal.Type->size[1] = 5;
  smo_es_emxEnsureCapacity_char_T(obj->Base.JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj_0->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Base.JointInternal.Type->size[1];
  smo_es_emxEnsureCapacity_char_T(switch_expression, b_kstr);
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

  smo_estimator_emxFree_char_T(&switch_expression);
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

static void emxInitStruct_q_robotics_man_mr(q_robotics_manip_internal__mr_T
  *pStruct)
{
  emxInitStruct_p_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slman_mr(robotics_slmanip_internal__mr_T
  *pStruct)
{
  emxInitStruct_q_robotics_man_mr(&pStruct->TreeInternal);
}

static q_robotics_manip_internal__mr_T *RigidBodyTree_RigidBodyTree_mr
  (q_robotics_manip_internal__mr_T *obj, o_robotics_manip_internal_R_m_T *iobj_0,
   o_robotics_manip_internal_R_m_T *iobj_1, o_robotics_manip_internal_R_m_T
   *iobj_2, o_robotics_manip_internal_R_m_T *iobj_3,
   o_robotics_manip_internal_R_m_T *iobj_4, o_robotics_manip_internal_R_m_T
   *iobj_5, o_robotics_manip_internal_R_m_T *iobj_6,
   o_robotics_manip_internal_R_m_T *iobj_7, o_robotics_manip_internal_R_m_T
   *iobj_8, o_robotics_manip_internal_R_m_T *iobj_9)
{
  q_robotics_manip_internal__mr_T *b_obj;
  int32_T i;
  static const int8_T tmp[20] = { 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 1, 2, 3, 4, 5, 6,
    7, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = smo_RigidBody_RigidBody_mrfu4kk(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = sm_RigidBody_RigidBody_mrfu4kk1(iobj_9);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = s_RigidBody_RigidBody_mrfu4kk1b(iobj_1);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = RigidBody_RigidBody_mrfu4kk1bh(iobj_2);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = RigidBody_RigidBody_mrfu4kk1bht(iobj_3);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = RigidBody_RigidBod_mrfu4kk1bhtk(iobj_4);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = RigidBody_RigidBo_mrfu4kk1bhtk3(iobj_5);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = RigidBody_RigidB_mrfu4kk1bhtk3a(iobj_6);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = RigidBody_Rigid_mrfu4kk1bhtk3a2(iobj_7);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = s_RigidBody_Rigid_h(iobj_8);
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

  s_RigidBody_Rigid_o(&obj->Base);
  return b_obj;
}

// Model step function
void smo_estimator_step(void)
{
  robotics_slmanip_internal_blo_T *obj;
  q_robotics_manip_internal_Rig_T *obj_0;
  emxArray_f_cell_wrap_smo_esti_T *Ttree;
  emxArray_char_T_smo_estimator_T *bname;
  o_robotics_manip_internal_Rig_T *obj_1;
  robotics_slmanip_internal__mr_T *obj_2;
  emxArray_real_T_smo_estimator_T *L;
  emxArray_real_T_smo_estimator_T *lambda;
  emxArray_real_T_smo_estimator_T *H;
  emxArray_real_T_smo_estimator_T *tmp;
  static const char_T tmp_0[19] = { 's', 'h', 'o', 'u', 'l', 'd', 'e', 'r', 's',
    '_', 'l', 'e', 'f', 't', '_', 'l', 'i', 'n', 'k' };

  static const char_T tmp_1[14] = { 'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
    'l', 'i', 'n', 'k', '6' };

  int32_T exitg1;
  boolean_T exitg2;
  if (rtmIsMajorTimeStep(smo_estimator_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&smo_estimator_M->solverInfo,
                          ((smo_estimator_M->Timing.clockTick0+1)*
      smo_estimator_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(smo_estimator_M)) {
    smo_estimator_M->Timing.t[0] = rtsiGetT(&smo_estimator_M->solverInfo);
  }

  smo_estimator_emxInit_real_T(&H, 2);
  if (rtmIsMajorTimeStep(smo_estimator_M) &&
      smo_estimator_M->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Atomic SubSystem: '<S5>/Subscribe'
    // MATLABSystem: '<S14>/SourceBlock' incorporates:
    //   Inport: '<S16>/In1'

    smo_estimator_SystemCore_step(&smo_estimator_B.b_varargout_1,
      smo_estimator_B.bias, &smo_estimator_B.b_varargout_2_Data_SL_Info_Curr,
      &smo_estimator_B.b_varargout_2_Data_SL_Info_Rece,
      &smo_estimator_B.b_varargout_2_Layout_DataOffset,
      smo_estimator_B.b_varargout_2_Layout_Dim,
      &smo_estimator_B.b_varargout_2_Layout_Dim_SL_Inf,
      &smo_estimator_B.b_varargout_2_Layout_Dim_SL_I_j);

    // Outputs for Enabled SubSystem: '<S14>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S16>/Enable'

    if (smo_estimator_B.b_varargout_1) {
      for (smo_estimator_B.i = 0; smo_estimator_B.i < 7; smo_estimator_B.i++) {
        smo_estimator_B.In1_k.Data[smo_estimator_B.i] =
          smo_estimator_B.bias[smo_estimator_B.i];
      }

      smo_estimator_B.In1_k.Data_SL_Info.CurrentLength =
        smo_estimator_B.b_varargout_2_Data_SL_Info_Curr;
      smo_estimator_B.In1_k.Data_SL_Info.ReceivedLength =
        smo_estimator_B.b_varargout_2_Data_SL_Info_Rece;
      smo_estimator_B.In1_k.Layout.DataOffset =
        smo_estimator_B.b_varargout_2_Layout_DataOffset;
      memcpy(&smo_estimator_B.In1_k.Layout.Dim[0],
             &smo_estimator_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_smo_estimator_std_msgs_MultiArrayDimension) << 4U);
      smo_estimator_B.In1_k.Layout.Dim_SL_Info.CurrentLength =
        smo_estimator_B.b_varargout_2_Layout_Dim_SL_Inf;
      smo_estimator_B.In1_k.Layout.Dim_SL_Info.ReceivedLength =
        smo_estimator_B.b_varargout_2_Layout_Dim_SL_I_j;
    }

    // End of MATLABSystem: '<S14>/SourceBlock'
    // End of Outputs for SubSystem: '<S14>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<S5>/Subscribe'

    // MATLABSystem: '<S8>/MATLAB System'
    RigidBodyTreeDynamics_massMatri(&smo_estimator_DW.obj_h.TreeInternal,
      smo_estimator_B.In1_k.Data, H);
    for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 49;
         smo_estimator_B.b_kstr++) {
      smo_estimator_B.MATLABSystem[smo_estimator_B.b_kstr] = H->
        data[smo_estimator_B.b_kstr];
    }

    // End of MATLABSystem: '<S8>/MATLAB System'
    smo_estimat_emxInit_f_cell_wrap(&Ttree, 2);
    smo_estimator_emxInit_char_T(&bname, 2);

    // MATLABSystem: '<S7>/MATLAB System'
    obj = &smo_estimator_DW.obj_o;
    obj_0 = &smo_estimator_DW.obj_o.TreeInternal;
    RigidBodyTree_forwardKinematics(&obj->TreeInternal,
      smo_estimator_B.In1_k.Data, Ttree);
    smo_estimator_B.bid1 = -1.0;
    smo_estimator_B.b_kstr = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj_0->Base.NameInternal->size[1];
    smo_es_emxEnsureCapacity_char_T(bname, smo_estimator_B.b_kstr);
    smo_estimator_B.n_c = obj_0->Base.NameInternal->size[0] *
      obj_0->Base.NameInternal->size[1] - 1;
    for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <=
         smo_estimator_B.n_c; smo_estimator_B.b_kstr++) {
      bname->data[smo_estimator_B.b_kstr] = obj_0->Base.NameInternal->
        data[smo_estimator_B.b_kstr];
    }

    for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 19;
         smo_estimator_B.b_kstr++) {
      smo_estimator_B.b_n[smo_estimator_B.b_kstr] = tmp_0[smo_estimator_B.b_kstr];
    }

    smo_estimator_B.b_varargout_1 = false;
    if (bname->size[1] == 19) {
      smo_estimator_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (smo_estimator_B.b_kstr - 1 < 19) {
          smo_estimator_B.n_c = smo_estimator_B.b_kstr - 1;
          if (bname->data[smo_estimator_B.n_c] !=
              smo_estimator_B.b_n[smo_estimator_B.n_c]) {
            exitg1 = 1;
          } else {
            smo_estimator_B.b_kstr++;
          }
        } else {
          smo_estimator_B.b_varargout_1 = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (smo_estimator_B.b_varargout_1) {
      smo_estimator_B.bid1 = 0.0;
    } else {
      smo_estimator_B.vNum = obj->TreeInternal.NumBodies;
      smo_estimator_B.i = 0;
      exitg2 = false;
      while ((!exitg2) && (smo_estimator_B.i <= static_cast<int32_T>
                           (smo_estimator_B.vNum) - 1)) {
        obj_1 = obj_0->Bodies[smo_estimator_B.i];
        smo_estimator_B.b_kstr = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = obj_1->NameInternal->size[1];
        smo_es_emxEnsureCapacity_char_T(bname, smo_estimator_B.b_kstr);
        smo_estimator_B.n_c = obj_1->NameInternal->size[0] * obj_1->
          NameInternal->size[1] - 1;
        for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <=
             smo_estimator_B.n_c; smo_estimator_B.b_kstr++) {
          bname->data[smo_estimator_B.b_kstr] = obj_1->NameInternal->
            data[smo_estimator_B.b_kstr];
        }

        for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 19;
             smo_estimator_B.b_kstr++) {
          smo_estimator_B.b_n[smo_estimator_B.b_kstr] =
            tmp_0[smo_estimator_B.b_kstr];
        }

        smo_estimator_B.b_varargout_1 = false;
        if (bname->size[1] == 19) {
          smo_estimator_B.b_kstr = 1;
          do {
            exitg1 = 0;
            if (smo_estimator_B.b_kstr - 1 < 19) {
              smo_estimator_B.n_c = smo_estimator_B.b_kstr - 1;
              if (bname->data[smo_estimator_B.n_c] !=
                  smo_estimator_B.b_n[smo_estimator_B.n_c]) {
                exitg1 = 1;
              } else {
                smo_estimator_B.b_kstr++;
              }
            } else {
              smo_estimator_B.b_varargout_1 = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (smo_estimator_B.b_varargout_1) {
          smo_estimator_B.bid1 = static_cast<real_T>(smo_estimator_B.i) + 1.0;
          exitg2 = true;
        } else {
          smo_estimator_B.i++;
        }
      }
    }

    if (smo_estimator_B.bid1 == 0.0) {
      memset(&smo_estimator_B.T1[0], 0, sizeof(real_T) << 4U);
      smo_estimator_B.T1[0] = 1.0;
      smo_estimator_B.T1[5] = 1.0;
      smo_estimator_B.T1[10] = 1.0;
      smo_estimator_B.T1[15] = 1.0;
    } else {
      for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 16;
           smo_estimator_B.b_kstr++) {
        smo_estimator_B.T1[smo_estimator_B.b_kstr] = Ttree->data
          [static_cast<int32_T>(smo_estimator_B.bid1) - 1]
          .f1[smo_estimator_B.b_kstr];
      }
    }

    smo_estimator_B.bid1 = -1.0;
    smo_estimator_B.b_kstr = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = obj_0->Base.NameInternal->size[1];
    smo_es_emxEnsureCapacity_char_T(bname, smo_estimator_B.b_kstr);
    smo_estimator_B.n_c = obj_0->Base.NameInternal->size[0] *
      obj_0->Base.NameInternal->size[1] - 1;
    for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <=
         smo_estimator_B.n_c; smo_estimator_B.b_kstr++) {
      bname->data[smo_estimator_B.b_kstr] = obj_0->Base.NameInternal->
        data[smo_estimator_B.b_kstr];
    }

    for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 14;
         smo_estimator_B.b_kstr++) {
      smo_estimator_B.b_i[smo_estimator_B.b_kstr] = tmp_1[smo_estimator_B.b_kstr];
    }

    smo_estimator_B.b_varargout_1 = false;
    if (bname->size[1] == 14) {
      smo_estimator_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (smo_estimator_B.b_kstr - 1 < 14) {
          smo_estimator_B.n_c = smo_estimator_B.b_kstr - 1;
          if (bname->data[smo_estimator_B.n_c] !=
              smo_estimator_B.b_i[smo_estimator_B.n_c]) {
            exitg1 = 1;
          } else {
            smo_estimator_B.b_kstr++;
          }
        } else {
          smo_estimator_B.b_varargout_1 = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (smo_estimator_B.b_varargout_1) {
      smo_estimator_B.bid1 = 0.0;
    } else {
      smo_estimator_B.vNum = obj->TreeInternal.NumBodies;
      smo_estimator_B.i = 0;
      exitg2 = false;
      while ((!exitg2) && (smo_estimator_B.i <= static_cast<int32_T>
                           (smo_estimator_B.vNum) - 1)) {
        obj_1 = obj_0->Bodies[smo_estimator_B.i];
        smo_estimator_B.b_kstr = bname->size[0] * bname->size[1];
        bname->size[0] = 1;
        bname->size[1] = obj_1->NameInternal->size[1];
        smo_es_emxEnsureCapacity_char_T(bname, smo_estimator_B.b_kstr);
        smo_estimator_B.n_c = obj_1->NameInternal->size[0] * obj_1->
          NameInternal->size[1] - 1;
        for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <=
             smo_estimator_B.n_c; smo_estimator_B.b_kstr++) {
          bname->data[smo_estimator_B.b_kstr] = obj_1->NameInternal->
            data[smo_estimator_B.b_kstr];
        }

        for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 14;
             smo_estimator_B.b_kstr++) {
          smo_estimator_B.b_i[smo_estimator_B.b_kstr] =
            tmp_1[smo_estimator_B.b_kstr];
        }

        smo_estimator_B.b_varargout_1 = false;
        if (bname->size[1] == 14) {
          smo_estimator_B.b_kstr = 1;
          do {
            exitg1 = 0;
            if (smo_estimator_B.b_kstr - 1 < 14) {
              smo_estimator_B.n_c = smo_estimator_B.b_kstr - 1;
              if (bname->data[smo_estimator_B.n_c] !=
                  smo_estimator_B.b_i[smo_estimator_B.n_c]) {
                exitg1 = 1;
              } else {
                smo_estimator_B.b_kstr++;
              }
            } else {
              smo_estimator_B.b_varargout_1 = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (smo_estimator_B.b_varargout_1) {
          smo_estimator_B.bid1 = static_cast<real_T>(smo_estimator_B.i) + 1.0;
          exitg2 = true;
        } else {
          smo_estimator_B.i++;
        }
      }
    }

    smo_estimator_emxFree_char_T(&bname);

    // MATLABSystem: '<S7>/MATLAB System'
    if (smo_estimator_B.bid1 == 0.0) {
      memset(&smo_estimator_B.T2[0], 0, sizeof(real_T) << 4U);
      smo_estimator_B.T2[0] = 1.0;
      smo_estimator_B.T2[5] = 1.0;
      smo_estimator_B.T2[10] = 1.0;
      smo_estimator_B.T2[15] = 1.0;
    } else {
      for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 16;
           smo_estimator_B.b_kstr++) {
        smo_estimator_B.T2[smo_estimator_B.b_kstr] = Ttree->data
          [static_cast<int32_T>(smo_estimator_B.bid1) - 1]
          .f1[smo_estimator_B.b_kstr];
      }
    }

    smo_estimat_emxFree_f_cell_wrap(&Ttree);

    // MATLABSystem: '<S7>/MATLAB System'
    for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 3;
         smo_estimator_B.b_kstr++) {
      smo_estimator_B.R_p[3 * smo_estimator_B.b_kstr] =
        smo_estimator_B.T2[smo_estimator_B.b_kstr];
      smo_estimator_B.R_p[3 * smo_estimator_B.b_kstr + 1] =
        smo_estimator_B.T2[smo_estimator_B.b_kstr + 4];
      smo_estimator_B.R_p[3 * smo_estimator_B.b_kstr + 2] =
        smo_estimator_B.T2[smo_estimator_B.b_kstr + 8];
    }

    for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 9;
         smo_estimator_B.b_kstr++) {
      smo_estimator_B.R_l[smo_estimator_B.b_kstr] =
        -smo_estimator_B.R_p[smo_estimator_B.b_kstr];
    }

    for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 3;
         smo_estimator_B.b_kstr++) {
      smo_estimator_B.i = smo_estimator_B.b_kstr << 2;
      smo_estimator_B.R_k[smo_estimator_B.i] = smo_estimator_B.R_p[3 *
        smo_estimator_B.b_kstr];
      smo_estimator_B.R_k[smo_estimator_B.i + 1] = smo_estimator_B.R_p[3 *
        smo_estimator_B.b_kstr + 1];
      smo_estimator_B.R_k[smo_estimator_B.i + 2] = smo_estimator_B.R_p[3 *
        smo_estimator_B.b_kstr + 2];
      smo_estimator_B.R_k[smo_estimator_B.b_kstr + 12] =
        smo_estimator_B.R_l[smo_estimator_B.b_kstr + 6] * smo_estimator_B.T2[14]
        + (smo_estimator_B.R_l[smo_estimator_B.b_kstr + 3] * smo_estimator_B.T2
           [13] + smo_estimator_B.R_l[smo_estimator_B.b_kstr] *
           smo_estimator_B.T2[12]);
    }

    smo_estimator_B.R_k[3] = 0.0;
    smo_estimator_B.R_k[7] = 0.0;
    smo_estimator_B.R_k[11] = 0.0;
    smo_estimator_B.R_k[15] = 1.0;
    for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 4;
         smo_estimator_B.b_kstr++) {
      for (smo_estimator_B.n_c = 0; smo_estimator_B.n_c < 4; smo_estimator_B.n_c
           ++) {
        smo_estimator_B.i = smo_estimator_B.b_kstr << 2;
        smo_estimator_B.rtb_MATLABSystem_d_tmp = smo_estimator_B.n_c +
          smo_estimator_B.i;
        smo_estimator_B.T2[smo_estimator_B.rtb_MATLABSystem_d_tmp] = 0.0;
        smo_estimator_B.T2[smo_estimator_B.rtb_MATLABSystem_d_tmp] +=
          smo_estimator_B.T1[smo_estimator_B.i] *
          smo_estimator_B.R_k[smo_estimator_B.n_c];
        smo_estimator_B.T2[smo_estimator_B.rtb_MATLABSystem_d_tmp] +=
          smo_estimator_B.T1[smo_estimator_B.i + 1] *
          smo_estimator_B.R_k[smo_estimator_B.n_c + 4];
        smo_estimator_B.T2[smo_estimator_B.rtb_MATLABSystem_d_tmp] +=
          smo_estimator_B.T1[smo_estimator_B.i + 2] *
          smo_estimator_B.R_k[smo_estimator_B.n_c + 8];
        smo_estimator_B.T2[smo_estimator_B.rtb_MATLABSystem_d_tmp] +=
          smo_estimator_B.T1[smo_estimator_B.i + 3] *
          smo_estimator_B.R_k[smo_estimator_B.n_c + 12];
      }
    }

    // Outputs for Atomic SubSystem: '<S5>/Subscribe1'
    // MATLABSystem: '<S15>/SourceBlock' incorporates:
    //   Inport: '<S17>/In1'

    smo_estimator_SystemCore_step_m(&smo_estimator_B.b_varargout_1,
      smo_estimator_B.bias, &smo_estimator_B.b_varargout_2_Data_SL_Info_Curr,
      &smo_estimator_B.b_varargout_2_Data_SL_Info_Rece,
      &smo_estimator_B.b_varargout_2_Layout_DataOffset,
      smo_estimator_B.b_varargout_2_Layout_Dim,
      &smo_estimator_B.b_varargout_2_Layout_Dim_SL_Inf,
      &smo_estimator_B.b_varargout_2_Layout_Dim_SL_I_j);

    // Outputs for Enabled SubSystem: '<S15>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S17>/Enable'

    if (smo_estimator_B.b_varargout_1) {
      for (smo_estimator_B.i = 0; smo_estimator_B.i < 7; smo_estimator_B.i++) {
        smo_estimator_B.In1.Data[smo_estimator_B.i] =
          smo_estimator_B.bias[smo_estimator_B.i];
      }

      smo_estimator_B.In1.Data_SL_Info.CurrentLength =
        smo_estimator_B.b_varargout_2_Data_SL_Info_Curr;
      smo_estimator_B.In1.Data_SL_Info.ReceivedLength =
        smo_estimator_B.b_varargout_2_Data_SL_Info_Rece;
      smo_estimator_B.In1.Layout.DataOffset =
        smo_estimator_B.b_varargout_2_Layout_DataOffset;
      memcpy(&smo_estimator_B.In1.Layout.Dim[0],
             &smo_estimator_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_smo_estimator_std_msgs_MultiArrayDimension) << 4U);
      smo_estimator_B.In1.Layout.Dim_SL_Info.CurrentLength =
        smo_estimator_B.b_varargout_2_Layout_Dim_SL_Inf;
      smo_estimator_B.In1.Layout.Dim_SL_Info.ReceivedLength =
        smo_estimator_B.b_varargout_2_Layout_Dim_SL_I_j;
    }

    // End of MATLABSystem: '<S15>/SourceBlock'
    // End of Outputs for SubSystem: '<S15>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<S5>/Subscribe1'
  }

  smo_estimator_emxInit_real_T(&L, 2);
  smo_estimator_emxInit_real_T(&lambda, 2);
  smo_estimator_emxInit_real_T(&tmp, 1);

  // MATLABSystem: '<S18>/MATLAB System' incorporates:
  //   Constant: '<S6>/Constant2'
  //   Integrator: '<S6>/Integrator'

  obj_2 = &smo_estimator_DW.obj;
  RigidBodyTreeDynamics_massMat_m(&smo_estimator_DW.obj.TreeInternal,
    smo_estimator_B.In1_k.Data, L, lambda);
  smo_estimator_B.vNum = obj_2->TreeInternal.VelocityNumber;
  smo_estimator_B.rtb_MATLABSystem_d_tmp = static_cast<int32_T>
    (smo_estimator_B.vNum);
  smo_estimator_B.b_kstr = tmp->size[0];
  tmp->size[0] = smo_estimator_B.rtb_MATLABSystem_d_tmp;
  smo_es_emxEnsureCapacity_real_T(tmp, smo_estimator_B.b_kstr);
  for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <
       smo_estimator_B.rtb_MATLABSystem_d_tmp; smo_estimator_B.b_kstr++) {
    tmp->data[smo_estimator_B.b_kstr] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj_2->TreeInternal,
    smo_estimator_B.In1_k.Data, &smo_estimator_X.Integrator_CSTATE[7],
    smo_estimator_P.Constant2_Value, smo_estimator_B.bias);
  smo_estimator_emxFree_real_T(&tmp);

  // MATLABSystem: '<S18>/MATLAB System'
  for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 7;
       smo_estimator_B.b_kstr++) {
    smo_estimator_B.bias[smo_estimator_B.b_kstr] =
      smo_estimator_B.In1.Data[smo_estimator_B.b_kstr] -
      smo_estimator_B.bias[smo_estimator_B.b_kstr];
  }

  if ((L->size[0] == 0) || (L->size[1] == 0)) {
    smo_estimator_B.u1 = 0;
  } else {
    smo_estimator_B.i = L->size[0];
    smo_estimator_B.u1 = L->size[1];
    if (smo_estimator_B.i > smo_estimator_B.u1) {
      smo_estimator_B.u1 = smo_estimator_B.i;
    }
  }

  smo_estimator_B.b_kstr = H->size[0] * H->size[1];
  H->size[0] = L->size[0];
  H->size[1] = L->size[1];
  smo_es_emxEnsureCapacity_real_T(H, smo_estimator_B.b_kstr);
  smo_estimator_B.n_c = L->size[0] * L->size[1] - 1;
  for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <= smo_estimator_B.n_c;
       smo_estimator_B.b_kstr++) {
    H->data[smo_estimator_B.b_kstr] = L->data[smo_estimator_B.b_kstr];
  }

  smo_estimator_B.iend = static_cast<int32_T>(((-1.0 - static_cast<real_T>
    (smo_estimator_B.u1)) + 1.0) / -1.0) - 1;
  for (smo_estimator_B.i = 0; smo_estimator_B.i <= smo_estimator_B.iend;
       smo_estimator_B.i++) {
    smo_estimator_B.j = static_cast<real_T>(smo_estimator_B.u1) +
      -static_cast<real_T>(smo_estimator_B.i);
    smo_estimator_B.b_kstr = static_cast<int32_T>(smo_estimator_B.j);
    smo_estimator_B.n_c = smo_estimator_B.b_kstr - 1;
    H->data[(static_cast<int32_T>(smo_estimator_B.j) + H->size[0] * (
              static_cast<int32_T>(smo_estimator_B.j) - 1)) - 1] = sqrt(H->data
      [(smo_estimator_B.n_c * H->size[0] + smo_estimator_B.b_kstr) - 1]);
    smo_estimator_B.bid1 = lambda->data[smo_estimator_B.n_c];
    while (smo_estimator_B.bid1 > 0.0) {
      smo_estimator_B.i_k = static_cast<int32_T>(smo_estimator_B.bid1) - 1;
      H->data[(static_cast<int32_T>(smo_estimator_B.j) + H->size[0] * (
                static_cast<int32_T>(smo_estimator_B.bid1) - 1)) - 1] = H->data
        [(smo_estimator_B.i_k * H->size[0] + smo_estimator_B.b_kstr) - 1] /
        H->data[((static_cast<int32_T>(smo_estimator_B.j) - 1) * H->size[0] +
                 static_cast<int32_T>(smo_estimator_B.j)) - 1];
      smo_estimator_B.bid1 = lambda->data[smo_estimator_B.i_k];
    }

    smo_estimator_B.bid1 = lambda->data[smo_estimator_B.n_c];
    while (smo_estimator_B.bid1 > 0.0) {
      smo_estimator_B.j = smo_estimator_B.bid1;
      while (smo_estimator_B.j > 0.0) {
        smo_estimator_B.n_c = static_cast<int32_T>(smo_estimator_B.j) - 1;
        H->data[(static_cast<int32_T>(smo_estimator_B.bid1) + H->size[0] * (
                  static_cast<int32_T>(smo_estimator_B.j) - 1)) - 1] = H->data
          [(smo_estimator_B.n_c * H->size[0] + static_cast<int32_T>
            (smo_estimator_B.bid1)) - 1] - H->data[((static_cast<int32_T>
          (smo_estimator_B.bid1) - 1) * H->size[0] + smo_estimator_B.b_kstr) - 1]
          * H->data[((static_cast<int32_T>(smo_estimator_B.j) - 1) * H->size[0]
                     + smo_estimator_B.b_kstr) - 1];
        smo_estimator_B.j = lambda->data[smo_estimator_B.n_c];
      }

      smo_estimator_B.bid1 = lambda->data[static_cast<int32_T>
        (smo_estimator_B.bid1) - 1];
    }
  }

  smo_estimator_B.b_kstr = L->size[0] * L->size[1];
  L->size[0] = H->size[0];
  L->size[1] = H->size[1];
  smo_es_emxEnsureCapacity_real_T(L, smo_estimator_B.b_kstr);
  smo_estimator_B.n_c = H->size[0] * H->size[1] - 1;
  for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <= smo_estimator_B.n_c;
       smo_estimator_B.b_kstr++) {
    L->data[smo_estimator_B.b_kstr] = H->data[smo_estimator_B.b_kstr];
  }

  smo_estimator_B.n_c = H->size[1];
  if ((H->size[0] == 0) || (H->size[1] == 0) || (1 >= H->size[1])) {
  } else {
    smo_estimator_B.iend = 0;
    for (smo_estimator_B.b_kstr = 2; smo_estimator_B.b_kstr <=
         smo_estimator_B.n_c; smo_estimator_B.b_kstr++) {
      for (smo_estimator_B.i = 0; smo_estimator_B.i <= smo_estimator_B.iend;
           smo_estimator_B.i++) {
        L->data[smo_estimator_B.i + L->size[0] * (smo_estimator_B.b_kstr - 1)] =
          0.0;
      }

      if (smo_estimator_B.iend + 1 < H->size[0]) {
        smo_estimator_B.iend++;
      }
    }
  }

  smo_estimator_emxFree_real_T(&H);

  // MATLABSystem: '<S18>/MATLAB System'
  smo_estimator_B.iend = static_cast<int32_T>(((-1.0 - smo_estimator_B.vNum) +
    1.0) / -1.0) - 1;
  for (smo_estimator_B.i = 0; smo_estimator_B.i <= smo_estimator_B.iend;
       smo_estimator_B.i++) {
    smo_estimator_B.n_c = static_cast<int32_T>(smo_estimator_B.vNum + -
      static_cast<real_T>(smo_estimator_B.i));
    smo_estimator_B.b_kstr = smo_estimator_B.n_c - 1;
    smo_estimator_B.bias[smo_estimator_B.b_kstr] /= L->data
      [(smo_estimator_B.b_kstr * L->size[0] + smo_estimator_B.n_c) - 1];
    smo_estimator_B.j = lambda->data[smo_estimator_B.b_kstr];
    while (smo_estimator_B.j > 0.0) {
      smo_estimator_B.u1 = static_cast<int32_T>(smo_estimator_B.j) - 1;
      smo_estimator_B.bias[smo_estimator_B.u1] -= L->data[(smo_estimator_B.u1 *
        L->size[0] + smo_estimator_B.n_c) - 1] *
        smo_estimator_B.bias[smo_estimator_B.b_kstr];
      smo_estimator_B.j = lambda->data[smo_estimator_B.u1];
    }
  }

  smo_estimator_B.n_c = smo_estimator_B.rtb_MATLABSystem_d_tmp - 1;
  for (smo_estimator_B.i = 0; smo_estimator_B.i <= smo_estimator_B.n_c;
       smo_estimator_B.i++) {
    smo_estimator_B.j = lambda->data[smo_estimator_B.i];
    while (smo_estimator_B.j > 0.0) {
      smo_estimator_B.b_kstr = static_cast<int32_T>(smo_estimator_B.j) - 1;
      smo_estimator_B.bias[smo_estimator_B.i] -= L->data[smo_estimator_B.b_kstr *
        L->size[0] + smo_estimator_B.i] *
        smo_estimator_B.bias[smo_estimator_B.b_kstr];
      smo_estimator_B.j = lambda->data[smo_estimator_B.b_kstr];
    }

    smo_estimator_B.bias[smo_estimator_B.i] /= L->data[L->size[0] *
      smo_estimator_B.i + smo_estimator_B.i];
  }

  smo_estimator_emxFree_real_T(&lambda);
  smo_estimator_emxFree_real_T(&L);
  if (rtmIsMajorTimeStep(smo_estimator_M) &&
      smo_estimator_M->Timing.TaskCounters.TID[3] == 0) {
    // MATLABSystem: '<S3>/Get Parameter7'
    ParamGet_smo_estimator_383.get_parameter(&smo_estimator_B.GetParameter7_o1);

    // MATLABSystem: '<S3>/Get Parameter8'
    ParamGet_smo_estimator_384.get_parameter(&smo_estimator_B.GetParameter8_o1);

    // MATLABSystem: '<S3>/Get Parameter9'
    ParamGet_smo_estimator_385.get_parameter(&smo_estimator_B.GetParameter9_o1);
  }

  // MATLAB Function: '<S6>/Observer' incorporates:
  //   Integrator: '<S6>/Integrator'
  //   MATLABSystem: '<S18>/MATLAB System'

  memset(&smo_estimator_B.xp_est[0], 0, 14U * sizeof(real_T));
  for (smo_estimator_B.i = 0; smo_estimator_B.i < 7; smo_estimator_B.i++) {
    smo_estimator_B.vNum = 2.0 / (exp
      ((smo_estimator_X.Integrator_CSTATE[smo_estimator_B.i] -
        smo_estimator_B.In1_k.Data[smo_estimator_B.i]) /
       smo_estimator_B.GetParameter9_o1) + 1.0) - 1.0;
    smo_estimator_B.z[smo_estimator_B.i] = smo_estimator_B.vNum *
      smo_estimator_B.GetParameter8_o1;
    smo_estimator_B.xp_est[smo_estimator_B.i] = smo_estimator_B.vNum * (sqrt
      (fabs(smo_estimator_B.In1_k.Data[smo_estimator_B.i] -
            smo_estimator_X.Integrator_CSTATE[smo_estimator_B.i])) *
      smo_estimator_B.GetParameter7_o1) +
      smo_estimator_X.Integrator_CSTATE[smo_estimator_B.i + 7];
    smo_estimator_B.xp_est[smo_estimator_B.i + 7] =
      smo_estimator_B.bias[smo_estimator_B.i] +
      smo_estimator_B.z[smo_estimator_B.i];
  }

  // End of MATLAB Function: '<S6>/Observer'
  if (rtmIsMajorTimeStep(smo_estimator_M) &&
      smo_estimator_M->Timing.TaskCounters.TID[1] == 0) {
    // MATLABSystem: '<S2>/Coordinate Transformation Conversion1'
    smo_estimator_B.vNum = sqrt(smo_estimator_B.T2[0] * smo_estimator_B.T2[0] +
      smo_estimator_B.T2[1] * smo_estimator_B.T2[1]);
    smo_estimator_B.CoordinateTransformationConvers[0] = rt_atan2d_snf
      (smo_estimator_B.T2[6], smo_estimator_B.T2[10]);
    smo_estimator_B.CoordinateTransformationConvers[1] = rt_atan2d_snf
      (-smo_estimator_B.T2[2], smo_estimator_B.vNum);
    smo_estimator_B.CoordinateTransformationConvers[2] = rt_atan2d_snf
      (smo_estimator_B.T2[1], smo_estimator_B.T2[0]);
    if (smo_estimator_B.vNum < 2.2204460492503131E-15) {
      smo_estimator_B.n_c = 0;
      for (smo_estimator_B.i = 0; smo_estimator_B.i < 1; smo_estimator_B.i++) {
        smo_estimator_B.n_c++;
      }

      smo_estimator_B.rtb_MATLABSystem_d_size[0] = 1;
      smo_estimator_B.rtb_MATLABSystem_d_size[1] = 1;
      smo_estimator_B.rtb_MATLABSystem_d_size[2] = smo_estimator_B.n_c;
      smo_estimator_B.rtb_MATLABSystem_d_size_o[0] = 1;
      smo_estimator_B.rtb_MATLABSystem_d_size_o[1] = 1;
      smo_estimator_B.rtb_MATLABSystem_d_size_o[2] = smo_estimator_B.n_c;
      smo_estimator_B.rtb_MATLABSystem_d_size_n[0] = 1;
      smo_estimator_B.rtb_MATLABSystem_d_size_n[1] = 1;
      smo_estimator_B.rtb_MATLABSystem_d_size_n[2] = smo_estimator_B.n_c;
      for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <
           smo_estimator_B.n_c; smo_estimator_B.b_kstr++) {
        smo_estimator_B.rtb_MATLABSystem_d_data = -smo_estimator_B.T2[9];
        smo_estimator_B.rtb_MATLABSystem_d_data_p = smo_estimator_B.T2[5];
        smo_estimator_B.rtb_MATLABSystem_d_data_p5 = -smo_estimator_B.T2[2];
      }

      smo_estimator_atan2(&smo_estimator_B.rtb_MATLABSystem_d_data,
                          smo_estimator_B.rtb_MATLABSystem_d_size,
                          &smo_estimator_B.rtb_MATLABSystem_d_data_p,
                          smo_estimator_B.rtb_MATLABSystem_d_size_o,
                          &smo_estimator_B.tmp_data, smo_estimator_B.tmp_size);
      smo_estimator_B.sy_size[0] = 1;
      smo_estimator_B.sy_size[1] = 1;
      smo_estimator_B.sy_size[2] = smo_estimator_B.n_c;
      for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <
           smo_estimator_B.n_c; smo_estimator_B.b_kstr++) {
        smo_estimator_B.rtb_MATLABSystem_d_data = smo_estimator_B.vNum;
      }

      smo_estimator_atan2(&smo_estimator_B.rtb_MATLABSystem_d_data_p5,
                          smo_estimator_B.rtb_MATLABSystem_d_size_n,
                          &smo_estimator_B.rtb_MATLABSystem_d_data,
                          smo_estimator_B.sy_size,
                          &smo_estimator_B.rtb_MATLABSystem_d_data_p,
                          smo_estimator_B.rtb_MATLABSystem_d_size);
      smo_estimator_B.n_c = smo_estimator_B.tmp_size[2];
      for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <
           smo_estimator_B.n_c; smo_estimator_B.b_kstr++) {
        smo_estimator_B.CoordinateTransformationConvers[0] =
          smo_estimator_B.tmp_data;
      }

      smo_estimator_B.n_c = smo_estimator_B.rtb_MATLABSystem_d_size[2];
      for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr <
           smo_estimator_B.n_c; smo_estimator_B.b_kstr++) {
        smo_estimator_B.CoordinateTransformationConvers[1] =
          smo_estimator_B.rtb_MATLABSystem_d_data_p;
      }

      smo_estimator_B.CoordinateTransformationConvers[2] = 0.0;
    }

    smo_estimator_B.vNum = smo_estimator_B.CoordinateTransformationConvers[0];
    smo_estimator_B.CoordinateTransformationConvers[0] =
      smo_estimator_B.CoordinateTransformationConvers[2];
    smo_estimator_B.CoordinateTransformationConvers[2] = smo_estimator_B.vNum;

    // End of MATLABSystem: '<S2>/Coordinate Transformation Conversion1'
  }

  // MATLAB Function: '<S2>/mass estimator' incorporates:
  //   Integrator: '<S6>/Integrator'

  smo_estimator_B.bid1 = 0.0;
  smo_estimator_B.vNum = 3.3121686421112381E-170;
  for (smo_estimator_B.i = 0; smo_estimator_B.i < 7; smo_estimator_B.i++) {
    smo_estimator_B.j = fabs(smo_estimator_X.Integrator_CSTATE[smo_estimator_B.i
      + 7]);
    if (smo_estimator_B.j > smo_estimator_B.vNum) {
      smo_estimator_B.t = smo_estimator_B.vNum / smo_estimator_B.j;
      smo_estimator_B.bid1 = smo_estimator_B.bid1 * smo_estimator_B.t *
        smo_estimator_B.t + 1.0;
      smo_estimator_B.vNum = smo_estimator_B.j;
    } else {
      smo_estimator_B.t = smo_estimator_B.j / smo_estimator_B.vNum;
      smo_estimator_B.bid1 += smo_estimator_B.t * smo_estimator_B.t;
    }
  }

  smo_estimator_B.bid1 = smo_estimator_B.vNum * sqrt(smo_estimator_B.bid1);
  smo_estimator_B.vNum = sin(smo_estimator_B.CoordinateTransformationConvers[1]);
  if (fabs(smo_estimator_B.vNum) > 0.2) {
    if (smo_estimator_B.bid1 < 0.15) {
      // TransferFcn: '<S1>/Low Pass (z1)' incorporates:
      //   TransferFcn: '<S1>/Low Pass (z1)1'
      //   TransferFcn: '<S1>/Low Pass (z1)2'
      //   TransferFcn: '<S1>/Low Pass (z1)3'
      //   TransferFcn: '<S1>/Low Pass (z1)4'
      //   TransferFcn: '<S1>/Low Pass (z1)5'
      //   TransferFcn: '<S1>/Low Pass (z1)6'

      smo_estimator_B.bid1 = smo_estimator_P.lowpass_B[0] /
        smo_estimator_P.lowpass_A[0];
      smo_estimator_B.j = smo_estimator_P.lowpass_B[1] /
        smo_estimator_P.lowpass_A[0] - smo_estimator_B.bid1 *
        (smo_estimator_P.lowpass_A[1] / smo_estimator_P.lowpass_A[0]);
      smo_estimator_B.t = smo_estimator_P.lowpass_B[0] /
        smo_estimator_P.lowpass_A[0];
      smo_estimator_B.rtb_LowPassz1_tmp = smo_estimator_P.lowpass_B[2] /
        smo_estimator_P.lowpass_A[0] - smo_estimator_B.bid1 *
        (smo_estimator_P.lowpass_A[2] / smo_estimator_P.lowpass_A[0]);
      smo_estimator_B.rtb_LowPassz1_tmp_a = smo_estimator_P.lowpass_B[3] /
        smo_estimator_P.lowpass_A[0] - smo_estimator_B.bid1 *
        (smo_estimator_P.lowpass_A[3] / smo_estimator_P.lowpass_A[0]);
      smo_estimator_B.bid1 = smo_estimator_P.lowpass_B[4] /
        smo_estimator_P.lowpass_A[0] - smo_estimator_B.bid1 *
        (smo_estimator_P.lowpass_A[4] / smo_estimator_P.lowpass_A[0]);

      // SignalConversion generated from: '<S9>/ SFunction ' incorporates:
      //   TransferFcn: '<S1>/Low Pass (z1)'
      //   TransferFcn: '<S1>/Low Pass (z1)1'
      //   TransferFcn: '<S1>/Low Pass (z1)2'
      //   TransferFcn: '<S1>/Low Pass (z1)3'
      //   TransferFcn: '<S1>/Low Pass (z1)4'
      //   TransferFcn: '<S1>/Low Pass (z1)5'
      //   TransferFcn: '<S1>/Low Pass (z1)6'

      smo_estimator_B.bias[0] = (((smo_estimator_B.j *
        smo_estimator_X.LowPassz1_CSTATE[0] + smo_estimator_B.t *
        smo_estimator_B.z[0]) + smo_estimator_B.rtb_LowPassz1_tmp *
        smo_estimator_X.LowPassz1_CSTATE[1]) +
        smo_estimator_B.rtb_LowPassz1_tmp_a * smo_estimator_X.LowPassz1_CSTATE[2])
        + smo_estimator_B.bid1 * smo_estimator_X.LowPassz1_CSTATE[3];
      smo_estimator_B.bias[1] = (((smo_estimator_B.j *
        smo_estimator_X.LowPassz11_CSTATE[0] + smo_estimator_B.t *
        smo_estimator_B.z[1]) + smo_estimator_B.rtb_LowPassz1_tmp *
        smo_estimator_X.LowPassz11_CSTATE[1]) +
        smo_estimator_B.rtb_LowPassz1_tmp_a * smo_estimator_X.LowPassz11_CSTATE
        [2]) + smo_estimator_B.bid1 * smo_estimator_X.LowPassz11_CSTATE[3];
      smo_estimator_B.bias[2] = (((smo_estimator_B.j *
        smo_estimator_X.LowPassz12_CSTATE[0] + smo_estimator_B.t *
        smo_estimator_B.z[2]) + smo_estimator_B.rtb_LowPassz1_tmp *
        smo_estimator_X.LowPassz12_CSTATE[1]) +
        smo_estimator_B.rtb_LowPassz1_tmp_a * smo_estimator_X.LowPassz12_CSTATE
        [2]) + smo_estimator_B.bid1 * smo_estimator_X.LowPassz12_CSTATE[3];
      smo_estimator_B.bias[3] = (((smo_estimator_B.j *
        smo_estimator_X.LowPassz13_CSTATE[0] + smo_estimator_B.t *
        smo_estimator_B.z[3]) + smo_estimator_B.rtb_LowPassz1_tmp *
        smo_estimator_X.LowPassz13_CSTATE[1]) +
        smo_estimator_B.rtb_LowPassz1_tmp_a * smo_estimator_X.LowPassz13_CSTATE
        [2]) + smo_estimator_B.bid1 * smo_estimator_X.LowPassz13_CSTATE[3];
      smo_estimator_B.bias[4] = (((smo_estimator_B.j *
        smo_estimator_X.LowPassz14_CSTATE[0] + smo_estimator_B.t *
        smo_estimator_B.z[4]) + smo_estimator_B.rtb_LowPassz1_tmp *
        smo_estimator_X.LowPassz14_CSTATE[1]) +
        smo_estimator_B.rtb_LowPassz1_tmp_a * smo_estimator_X.LowPassz14_CSTATE
        [2]) + smo_estimator_B.bid1 * smo_estimator_X.LowPassz14_CSTATE[3];
      smo_estimator_B.bias[5] = (((smo_estimator_B.j *
        smo_estimator_X.LowPassz15_CSTATE[0] + smo_estimator_B.t *
        smo_estimator_B.z[5]) + smo_estimator_B.rtb_LowPassz1_tmp *
        smo_estimator_X.LowPassz15_CSTATE[1]) +
        smo_estimator_B.rtb_LowPassz1_tmp_a * smo_estimator_X.LowPassz15_CSTATE
        [2]) + smo_estimator_B.bid1 * smo_estimator_X.LowPassz15_CSTATE[3];
      smo_estimator_B.bias[6] = (((smo_estimator_B.j *
        smo_estimator_X.LowPassz16_CSTATE[0] + smo_estimator_B.t *
        smo_estimator_B.z[6]) + smo_estimator_B.rtb_LowPassz1_tmp *
        smo_estimator_X.LowPassz16_CSTATE[1]) +
        smo_estimator_B.rtb_LowPassz1_tmp_a * smo_estimator_X.LowPassz16_CSTATE
        [2]) + smo_estimator_B.bid1 * smo_estimator_X.LowPassz16_CSTATE[3];
      for (smo_estimator_B.b_kstr = 0; smo_estimator_B.b_kstr < 7;
           smo_estimator_B.b_kstr++) {
        smo_estimator_B.dv5[smo_estimator_B.b_kstr] = 0.0;
        for (smo_estimator_B.n_c = 0; smo_estimator_B.n_c < 7;
             smo_estimator_B.n_c++) {
          smo_estimator_B.dv5[smo_estimator_B.b_kstr] +=
            smo_estimator_B.MATLABSystem[7 * smo_estimator_B.n_c +
            smo_estimator_B.b_kstr] * smo_estimator_B.bias[smo_estimator_B.n_c];
        }
      }

      smo_estimator_B.vNum = -smo_estimator_B.dv5[5] / (1.9129500000000002 *
        smo_estimator_B.vNum);
    } else {
      smo_estimator_B.vNum = 0.0;
    }
  } else {
    smo_estimator_B.vNum = 0.0;
  }

  // End of MATLAB Function: '<S2>/mass estimator'

  // RateTransition: '<S4>/Rate Transition'
  if ((rtmIsMajorTimeStep(smo_estimator_M) &&
       smo_estimator_M->Timing.TaskCounters.TID[1] == 0) && (rtmIsMajorTimeStep
       (smo_estimator_M) &&
       smo_estimator_M->Timing.TaskCounters.TID[2] == 0)) {
    smo_estimator_DW.RateTransition_Buffer = smo_estimator_B.vNum;
  }

  if (rtmIsMajorTimeStep(smo_estimator_M) &&
      smo_estimator_M->Timing.TaskCounters.TID[2] == 0) {
    // BusAssignment: '<S4>/Bus Assignment2'
    smo_estimator_B.BusAssignment2.Data = smo_estimator_DW.RateTransition_Buffer;

    // Outputs for Atomic SubSystem: '<S4>/Publish2'
    // MATLABSystem: '<S12>/SinkBlock'
    Pub_smo_estimator_311.publish(&smo_estimator_B.BusAssignment2);

    // End of Outputs for SubSystem: '<S4>/Publish2'
  }

  // End of RateTransition: '<S4>/Rate Transition'
  if (rtmIsMajorTimeStep(smo_estimator_M) &&
      smo_estimator_M->Timing.TaskCounters.TID[1] == 0) {
    // BusAssignment: '<S4>/Bus Assignment3' incorporates:
    //   Constant: '<S11>/Constant'
    //   Constant: '<S4>/Constant1'
    //   Integrator: '<S6>/Integrator'

    smo_estimator_B.BusAssignment3 = smo_estimator_P.Constant_Value_i;
    for (smo_estimator_B.i = 0; smo_estimator_B.i < 7; smo_estimator_B.i++) {
      smo_estimator_B.BusAssignment3.Data[smo_estimator_B.i] =
        smo_estimator_X.Integrator_CSTATE[smo_estimator_B.i + 7];
    }

    smo_estimator_B.BusAssignment3.Data_SL_Info.CurrentLength =
      smo_estimator_P.Constant1_Value;
    smo_estimator_B.BusAssignment3.Data_SL_Info.ReceivedLength =
      smo_estimator_P.Constant1_Value;

    // End of BusAssignment: '<S4>/Bus Assignment3'

    // Outputs for Atomic SubSystem: '<S4>/Publish3'
    // MATLABSystem: '<S13>/SinkBlock'
    Pub_smo_estimator_331.publish(&smo_estimator_B.BusAssignment3);

    // End of Outputs for SubSystem: '<S4>/Publish3'
  }

  if (rtmIsMajorTimeStep(smo_estimator_M)) {
    rt_ertODEUpdateContinuousStates(&smo_estimator_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++smo_estimator_M->Timing.clockTick0;
    smo_estimator_M->Timing.t[0] = rtsiGetSolverStopTime
      (&smo_estimator_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.004s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.004, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      smo_estimator_M->Timing.clockTick1++;
    }

    rate_scheduler();
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void smo_estimator_derivatives(void)
{
  real_T LowPassz1_CSTATE_tmp;
  real_T LowPassz1_CSTATE_tmp_0;
  real_T LowPassz1_CSTATE_tmp_1;
  real_T LowPassz1_CSTATE_tmp_2;
  XDot_smo_estimator_T *_rtXdot;
  _rtXdot = ((XDot_smo_estimator_T *) smo_estimator_M->derivs);

  // Derivatives for Integrator: '<S6>/Integrator'
  memcpy(&_rtXdot->Integrator_CSTATE[0], &smo_estimator_B.xp_est[0], 14U *
         sizeof(real_T));

  // Derivatives for TransferFcn: '<S1>/Low Pass (z1)' incorporates:
  //   TransferFcn: '<S1>/Low Pass (z1)1'
  //   TransferFcn: '<S1>/Low Pass (z1)2'
  //   TransferFcn: '<S1>/Low Pass (z1)3'
  //   TransferFcn: '<S1>/Low Pass (z1)4'
  //   TransferFcn: '<S1>/Low Pass (z1)5'
  //   TransferFcn: '<S1>/Low Pass (z1)6'

  _rtXdot->LowPassz1_CSTATE[0] = 0.0;
  LowPassz1_CSTATE_tmp = -smo_estimator_P.lowpass_A[1] /
    smo_estimator_P.lowpass_A[0];
  _rtXdot->LowPassz1_CSTATE[0] += LowPassz1_CSTATE_tmp *
    smo_estimator_X.LowPassz1_CSTATE[0];
  _rtXdot->LowPassz1_CSTATE[1] = 0.0;
  LowPassz1_CSTATE_tmp_0 = -smo_estimator_P.lowpass_A[2] /
    smo_estimator_P.lowpass_A[0];
  _rtXdot->LowPassz1_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    smo_estimator_X.LowPassz1_CSTATE[1];
  _rtXdot->LowPassz1_CSTATE[2] = 0.0;
  LowPassz1_CSTATE_tmp_1 = -smo_estimator_P.lowpass_A[3] /
    smo_estimator_P.lowpass_A[0];
  _rtXdot->LowPassz1_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    smo_estimator_X.LowPassz1_CSTATE[2];
  _rtXdot->LowPassz1_CSTATE[3] = 0.0;
  LowPassz1_CSTATE_tmp_2 = -smo_estimator_P.lowpass_A[4] /
    smo_estimator_P.lowpass_A[0];
  _rtXdot->LowPassz1_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    smo_estimator_X.LowPassz1_CSTATE[3];
  _rtXdot->LowPassz1_CSTATE[1] += smo_estimator_X.LowPassz1_CSTATE[0];
  _rtXdot->LowPassz1_CSTATE[2] += smo_estimator_X.LowPassz1_CSTATE[1];
  _rtXdot->LowPassz1_CSTATE[3] += smo_estimator_X.LowPassz1_CSTATE[2];
  _rtXdot->LowPassz1_CSTATE[0] += smo_estimator_B.z[0];

  // Derivatives for TransferFcn: '<S1>/Low Pass (z1)1'
  _rtXdot->LowPassz11_CSTATE[0] = 0.0;
  _rtXdot->LowPassz11_CSTATE[0] += LowPassz1_CSTATE_tmp *
    smo_estimator_X.LowPassz11_CSTATE[0];
  _rtXdot->LowPassz11_CSTATE[1] = 0.0;
  _rtXdot->LowPassz11_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    smo_estimator_X.LowPassz11_CSTATE[1];
  _rtXdot->LowPassz11_CSTATE[2] = 0.0;
  _rtXdot->LowPassz11_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    smo_estimator_X.LowPassz11_CSTATE[2];
  _rtXdot->LowPassz11_CSTATE[3] = 0.0;
  _rtXdot->LowPassz11_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    smo_estimator_X.LowPassz11_CSTATE[3];
  _rtXdot->LowPassz11_CSTATE[1] += smo_estimator_X.LowPassz11_CSTATE[0];
  _rtXdot->LowPassz11_CSTATE[2] += smo_estimator_X.LowPassz11_CSTATE[1];
  _rtXdot->LowPassz11_CSTATE[3] += smo_estimator_X.LowPassz11_CSTATE[2];
  _rtXdot->LowPassz11_CSTATE[0] += smo_estimator_B.z[1];

  // Derivatives for TransferFcn: '<S1>/Low Pass (z1)2'
  _rtXdot->LowPassz12_CSTATE[0] = 0.0;
  _rtXdot->LowPassz12_CSTATE[0] += LowPassz1_CSTATE_tmp *
    smo_estimator_X.LowPassz12_CSTATE[0];
  _rtXdot->LowPassz12_CSTATE[1] = 0.0;
  _rtXdot->LowPassz12_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    smo_estimator_X.LowPassz12_CSTATE[1];
  _rtXdot->LowPassz12_CSTATE[2] = 0.0;
  _rtXdot->LowPassz12_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    smo_estimator_X.LowPassz12_CSTATE[2];
  _rtXdot->LowPassz12_CSTATE[3] = 0.0;
  _rtXdot->LowPassz12_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    smo_estimator_X.LowPassz12_CSTATE[3];
  _rtXdot->LowPassz12_CSTATE[1] += smo_estimator_X.LowPassz12_CSTATE[0];
  _rtXdot->LowPassz12_CSTATE[2] += smo_estimator_X.LowPassz12_CSTATE[1];
  _rtXdot->LowPassz12_CSTATE[3] += smo_estimator_X.LowPassz12_CSTATE[2];
  _rtXdot->LowPassz12_CSTATE[0] += smo_estimator_B.z[2];

  // Derivatives for TransferFcn: '<S1>/Low Pass (z1)3'
  _rtXdot->LowPassz13_CSTATE[0] = 0.0;
  _rtXdot->LowPassz13_CSTATE[0] += LowPassz1_CSTATE_tmp *
    smo_estimator_X.LowPassz13_CSTATE[0];
  _rtXdot->LowPassz13_CSTATE[1] = 0.0;
  _rtXdot->LowPassz13_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    smo_estimator_X.LowPassz13_CSTATE[1];
  _rtXdot->LowPassz13_CSTATE[2] = 0.0;
  _rtXdot->LowPassz13_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    smo_estimator_X.LowPassz13_CSTATE[2];
  _rtXdot->LowPassz13_CSTATE[3] = 0.0;
  _rtXdot->LowPassz13_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    smo_estimator_X.LowPassz13_CSTATE[3];
  _rtXdot->LowPassz13_CSTATE[1] += smo_estimator_X.LowPassz13_CSTATE[0];
  _rtXdot->LowPassz13_CSTATE[2] += smo_estimator_X.LowPassz13_CSTATE[1];
  _rtXdot->LowPassz13_CSTATE[3] += smo_estimator_X.LowPassz13_CSTATE[2];
  _rtXdot->LowPassz13_CSTATE[0] += smo_estimator_B.z[3];

  // Derivatives for TransferFcn: '<S1>/Low Pass (z1)4'
  _rtXdot->LowPassz14_CSTATE[0] = 0.0;
  _rtXdot->LowPassz14_CSTATE[0] += LowPassz1_CSTATE_tmp *
    smo_estimator_X.LowPassz14_CSTATE[0];
  _rtXdot->LowPassz14_CSTATE[1] = 0.0;
  _rtXdot->LowPassz14_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    smo_estimator_X.LowPassz14_CSTATE[1];
  _rtXdot->LowPassz14_CSTATE[2] = 0.0;
  _rtXdot->LowPassz14_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    smo_estimator_X.LowPassz14_CSTATE[2];
  _rtXdot->LowPassz14_CSTATE[3] = 0.0;
  _rtXdot->LowPassz14_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    smo_estimator_X.LowPassz14_CSTATE[3];
  _rtXdot->LowPassz14_CSTATE[1] += smo_estimator_X.LowPassz14_CSTATE[0];
  _rtXdot->LowPassz14_CSTATE[2] += smo_estimator_X.LowPassz14_CSTATE[1];
  _rtXdot->LowPassz14_CSTATE[3] += smo_estimator_X.LowPassz14_CSTATE[2];
  _rtXdot->LowPassz14_CSTATE[0] += smo_estimator_B.z[4];

  // Derivatives for TransferFcn: '<S1>/Low Pass (z1)5'
  _rtXdot->LowPassz15_CSTATE[0] = 0.0;
  _rtXdot->LowPassz15_CSTATE[0] += LowPassz1_CSTATE_tmp *
    smo_estimator_X.LowPassz15_CSTATE[0];
  _rtXdot->LowPassz15_CSTATE[1] = 0.0;
  _rtXdot->LowPassz15_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    smo_estimator_X.LowPassz15_CSTATE[1];
  _rtXdot->LowPassz15_CSTATE[2] = 0.0;
  _rtXdot->LowPassz15_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    smo_estimator_X.LowPassz15_CSTATE[2];
  _rtXdot->LowPassz15_CSTATE[3] = 0.0;
  _rtXdot->LowPassz15_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    smo_estimator_X.LowPassz15_CSTATE[3];
  _rtXdot->LowPassz15_CSTATE[1] += smo_estimator_X.LowPassz15_CSTATE[0];
  _rtXdot->LowPassz15_CSTATE[2] += smo_estimator_X.LowPassz15_CSTATE[1];
  _rtXdot->LowPassz15_CSTATE[3] += smo_estimator_X.LowPassz15_CSTATE[2];
  _rtXdot->LowPassz15_CSTATE[0] += smo_estimator_B.z[5];

  // Derivatives for TransferFcn: '<S1>/Low Pass (z1)6'
  _rtXdot->LowPassz16_CSTATE[0] = 0.0;
  _rtXdot->LowPassz16_CSTATE[0] += LowPassz1_CSTATE_tmp *
    smo_estimator_X.LowPassz16_CSTATE[0];
  _rtXdot->LowPassz16_CSTATE[1] = 0.0;
  _rtXdot->LowPassz16_CSTATE[0] += LowPassz1_CSTATE_tmp_0 *
    smo_estimator_X.LowPassz16_CSTATE[1];
  _rtXdot->LowPassz16_CSTATE[2] = 0.0;
  _rtXdot->LowPassz16_CSTATE[0] += LowPassz1_CSTATE_tmp_1 *
    smo_estimator_X.LowPassz16_CSTATE[2];
  _rtXdot->LowPassz16_CSTATE[3] = 0.0;
  _rtXdot->LowPassz16_CSTATE[0] += LowPassz1_CSTATE_tmp_2 *
    smo_estimator_X.LowPassz16_CSTATE[3];
  _rtXdot->LowPassz16_CSTATE[1] += smo_estimator_X.LowPassz16_CSTATE[0];
  _rtXdot->LowPassz16_CSTATE[2] += smo_estimator_X.LowPassz16_CSTATE[1];
  _rtXdot->LowPassz16_CSTATE[3] += smo_estimator_X.LowPassz16_CSTATE[2];
  _rtXdot->LowPassz16_CSTATE[0] += smo_estimator_B.z[6];
}

// Model initialize function
void smo_estimator_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&smo_estimator_M->solverInfo,
                          &smo_estimator_M->Timing.simTimeStep);
    rtsiSetTPtr(&smo_estimator_M->solverInfo, &rtmGetTPtr(smo_estimator_M));
    rtsiSetStepSizePtr(&smo_estimator_M->solverInfo,
                       &smo_estimator_M->Timing.stepSize0);
    rtsiSetdXPtr(&smo_estimator_M->solverInfo, &smo_estimator_M->derivs);
    rtsiSetContStatesPtr(&smo_estimator_M->solverInfo, (real_T **)
                         &smo_estimator_M->contStates);
    rtsiSetNumContStatesPtr(&smo_estimator_M->solverInfo,
      &smo_estimator_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&smo_estimator_M->solverInfo,
      &smo_estimator_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&smo_estimator_M->solverInfo,
      &smo_estimator_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&smo_estimator_M->solverInfo,
      &smo_estimator_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&smo_estimator_M->solverInfo, (&rtmGetErrorStatus
      (smo_estimator_M)));
    rtsiSetRTModelPtr(&smo_estimator_M->solverInfo, smo_estimator_M);
  }

  rtsiSetSimTimeStep(&smo_estimator_M->solverInfo, MAJOR_TIME_STEP);
  smo_estimator_M->intgData.y = smo_estimator_M->odeY;
  smo_estimator_M->intgData.f[0] = smo_estimator_M->odeF[0];
  smo_estimator_M->intgData.f[1] = smo_estimator_M->odeF[1];
  smo_estimator_M->intgData.f[2] = smo_estimator_M->odeF[2];
  smo_estimator_M->intgData.f[3] = smo_estimator_M->odeF[3];
  smo_estimator_M->contStates = ((X_smo_estimator_T *) &smo_estimator_X);
  rtsiSetSolverData(&smo_estimator_M->solverInfo, static_cast<void *>
                    (&smo_estimator_M->intgData));
  rtsiSetSolverName(&smo_estimator_M->solverInfo,"ode4");
  rtmSetTPtr(smo_estimator_M, &smo_estimator_M->Timing.tArray[0]);
  smo_estimator_M->Timing.stepSize0 = 0.004;

  {
    int32_T i;
    static const char_T tmp[25] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r', 'e',
      '/', 'l', 'a', '_', 'c', 'u', 'r', 'r', 'e', 'n', 't', '_', 'p', 'o', 's',
      'e' };

    static const char_T tmp_0[19] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 't', 'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_1[15] = { '/', 'e', 's', 't', 'i', 'm', 'a', 't',
      'e', 'd', '_', 'm', 'a', 's', 's' };

    static const char_T tmp_2[21] = { '/', 's', 'm', 'o', '/', 'e', 's', 't',
      'i', 'm', 'a', 't', 'e', 'd', '_', 's', 'p', 'e', 'e', 'd', 's' };

    static const char_T tmp_3[11] = { '/', 's', 'm', 'o', '/', 'l', 'a', 'm',
      'b', 'd', 'a' };

    static const char_T tmp_4[10] = { '/', 's', 'm', 'o', '/', 'a', 'l', 'p',
      'h', 'a' };

    static const char_T tmp_5[10] = { '/', 's', 'm', 'o', '/', 'g', 'a', 'm',
      'm', 'a' };

    // InitializeConditions for Integrator: '<S6>/Integrator'
    memcpy(&smo_estimator_X.Integrator_CSTATE[0],
           &smo_estimator_P.Integrator_IC[0], 14U * sizeof(real_T));

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)'
    smo_estimator_X.LowPassz1_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)1'
    smo_estimator_X.LowPassz11_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)2'
    smo_estimator_X.LowPassz12_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)3'
    smo_estimator_X.LowPassz13_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)4'
    smo_estimator_X.LowPassz14_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)5'
    smo_estimator_X.LowPassz15_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)6'
    smo_estimator_X.LowPassz16_CSTATE[0] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)'
    smo_estimator_X.LowPassz1_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)1'
    smo_estimator_X.LowPassz11_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)2'
    smo_estimator_X.LowPassz12_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)3'
    smo_estimator_X.LowPassz13_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)4'
    smo_estimator_X.LowPassz14_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)5'
    smo_estimator_X.LowPassz15_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)6'
    smo_estimator_X.LowPassz16_CSTATE[1] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)'
    smo_estimator_X.LowPassz1_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)1'
    smo_estimator_X.LowPassz11_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)2'
    smo_estimator_X.LowPassz12_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)3'
    smo_estimator_X.LowPassz13_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)4'
    smo_estimator_X.LowPassz14_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)5'
    smo_estimator_X.LowPassz15_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)6'
    smo_estimator_X.LowPassz16_CSTATE[2] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)'
    smo_estimator_X.LowPassz1_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)1'
    smo_estimator_X.LowPassz11_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)2'
    smo_estimator_X.LowPassz12_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)3'
    smo_estimator_X.LowPassz13_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)4'
    smo_estimator_X.LowPassz14_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)5'
    smo_estimator_X.LowPassz15_CSTATE[3] = 0.0;

    // InitializeConditions for TransferFcn: '<S1>/Low Pass (z1)6'
    smo_estimator_X.LowPassz16_CSTATE[3] = 0.0;

    // SystemInitialize for Atomic SubSystem: '<S5>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S14>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S16>/Out1'
    smo_estimator_B.In1_k = smo_estimator_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S14>/Enabled Subsystem'

    // Start for MATLABSystem: '<S14>/SourceBlock'
    smo_estimator_DW.obj_m.matlabCodegenIsDeleted = false;
    smo_estimator_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 25; i++) {
      smo_estimator_B.cv[i] = tmp[i];
    }

    smo_estimator_B.cv[25] = '\x00';
    Sub_smo_estimator_299.createSubscriber(smo_estimator_B.cv, 1);
    smo_estimator_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S14>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S5>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<S5>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S15>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S17>/Out1'
    smo_estimator_B.In1 = smo_estimator_P.Out1_Y0_k;

    // End of SystemInitialize for SubSystem: '<S15>/Enabled Subsystem'

    // Start for MATLABSystem: '<S15>/SourceBlock'
    smo_estimator_DW.obj_ay.matlabCodegenIsDeleted = false;
    smo_estimator_DW.obj_ay.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      smo_estimator_B.cv2[i] = tmp_0[i];
    }

    smo_estimator_B.cv2[19] = '\x00';
    Sub_smo_estimator_638.createSubscriber(smo_estimator_B.cv2, 1);
    smo_estimator_DW.obj_ay.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S15>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S5>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<S4>/Publish2'
    // Start for MATLABSystem: '<S12>/SinkBlock'
    smo_estimator_DW.obj_ap.matlabCodegenIsDeleted = false;
    smo_estimator_DW.obj_ap.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      smo_estimator_B.cv3[i] = tmp_1[i];
    }

    smo_estimator_B.cv3[15] = '\x00';
    Pub_smo_estimator_311.createPublisher(smo_estimator_B.cv3, 1);
    smo_estimator_DW.obj_ap.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S12>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<S4>/Publish3'
    // Start for MATLABSystem: '<S13>/SinkBlock'
    smo_estimator_DW.obj_b.matlabCodegenIsDeleted = false;
    smo_estimator_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 21; i++) {
      smo_estimator_B.cv1[i] = tmp_2[i];
    }

    smo_estimator_B.cv1[21] = '\x00';
    Pub_smo_estimator_331.createPublisher(smo_estimator_B.cv1, 1);
    smo_estimator_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S13>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Publish3'
    emxInitStruct_robotics_slmanip_(&smo_estimator_DW.obj_h);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_1_m);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_20_i);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_19_o);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_18_h);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_17_i);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_16_e);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_15_k);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_14_a);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_13_n);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_12_p);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_11_o);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_10_p);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_9_g);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_8_b);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_7_f);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_6_i);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_5_o);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_4_m);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_3_b);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_2_g);

    // Start for MATLABSystem: '<S8>/MATLAB System'
    smo_estimator_DW.obj_h.isInitialized = 0;
    smo_estimator_DW.obj_h.isInitialized = 1;
    s_RigidBodyTree_RigidBodyTree_m(&smo_estimator_DW.obj_h.TreeInternal,
      &smo_estimator_DW.gobj_2_g, &smo_estimator_DW.gobj_4_m,
      &smo_estimator_DW.gobj_5_o, &smo_estimator_DW.gobj_6_i,
      &smo_estimator_DW.gobj_7_f, &smo_estimator_DW.gobj_8_b,
      &smo_estimator_DW.gobj_9_g, &smo_estimator_DW.gobj_10_p,
      &smo_estimator_DW.gobj_11_o, &smo_estimator_DW.gobj_3_b);
    emxInitStruct_robotics_slmani_m(&smo_estimator_DW.obj_o);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_1_g);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_20_n);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_19_i);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_18_f);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_17_l);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_16_b);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_15_c);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_14_m);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_13_d);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_12_d);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_11_h);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_10_h);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_9_m);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_8_h);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_7_n);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_6_h);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_5_n);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_4_p);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_3_b3);
    emxInitStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_2_i);

    // Start for MATLABSystem: '<S7>/MATLAB System'
    smo_estimator_DW.obj_o.isInitialized = 0;
    smo_estimator_DW.obj_o.isInitialized = 1;
    smo_RigidBodyTree_RigidBodyTree(&smo_estimator_DW.obj_o.TreeInternal,
      &smo_estimator_DW.gobj_2_i, &smo_estimator_DW.gobj_4_p,
      &smo_estimator_DW.gobj_5_n, &smo_estimator_DW.gobj_6_h,
      &smo_estimator_DW.gobj_7_n, &smo_estimator_DW.gobj_8_h,
      &smo_estimator_DW.gobj_9_m, &smo_estimator_DW.gobj_10_h,
      &smo_estimator_DW.gobj_11_h, &smo_estimator_DW.gobj_3_b3);
    emxInitStruct_robotics_slman_mr(&smo_estimator_DW.obj);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_1);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_20);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_19);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_18);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_17);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_16);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_15);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_14);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_13);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_12);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_11);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_10);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_9);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_8);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_7);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_6);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_5);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_4);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_3);
    emxInitStruct_o_robotics_manip_(&smo_estimator_DW.gobj_2);

    // Start for MATLABSystem: '<S18>/MATLAB System'
    smo_estimator_DW.obj.isInitialized = 0;
    smo_estimator_DW.obj.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_mr(&smo_estimator_DW.obj.TreeInternal,
      &smo_estimator_DW.gobj_2, &smo_estimator_DW.gobj_4,
      &smo_estimator_DW.gobj_5, &smo_estimator_DW.gobj_6,
      &smo_estimator_DW.gobj_7, &smo_estimator_DW.gobj_8,
      &smo_estimator_DW.gobj_9, &smo_estimator_DW.gobj_10,
      &smo_estimator_DW.gobj_11, &smo_estimator_DW.gobj_3);

    // Start for MATLABSystem: '<S3>/Get Parameter7'
    smo_estimator_DW.obj_a.matlabCodegenIsDeleted = false;
    smo_estimator_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      smo_estimator_B.cv4[i] = tmp_3[i];
    }

    smo_estimator_B.cv4[11] = '\x00';
    ParamGet_smo_estimator_383.initialize(smo_estimator_B.cv4);
    ParamGet_smo_estimator_383.initialize_error_codes(0, 1, 2, 3);
    ParamGet_smo_estimator_383.set_initial_value(0.0);
    smo_estimator_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/Get Parameter7'

    // Start for MATLABSystem: '<S3>/Get Parameter8'
    smo_estimator_DW.obj_oo.matlabCodegenIsDeleted = false;
    smo_estimator_DW.obj_oo.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      smo_estimator_B.cv5[i] = tmp_4[i];
    }

    smo_estimator_B.cv5[10] = '\x00';
    ParamGet_smo_estimator_384.initialize(smo_estimator_B.cv5);
    ParamGet_smo_estimator_384.initialize_error_codes(0, 1, 2, 3);
    ParamGet_smo_estimator_384.set_initial_value(0.0);
    smo_estimator_DW.obj_oo.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/Get Parameter8'

    // Start for MATLABSystem: '<S3>/Get Parameter9'
    smo_estimator_DW.obj_ok.matlabCodegenIsDeleted = false;
    smo_estimator_DW.obj_ok.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      smo_estimator_B.cv5[i] = tmp_5[i];
    }

    smo_estimator_B.cv5[10] = '\x00';
    ParamGet_smo_estimator_385.initialize(smo_estimator_B.cv5);
    ParamGet_smo_estimator_385.initialize_error_codes(0, 1, 2, 3);
    ParamGet_smo_estimator_385.set_initial_value(0.0);
    smo_estimator_DW.obj_ok.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/Get Parameter9'
  }
}

// Model terminate function
void smo_estimator_terminate(void)
{
  // Terminate for Atomic SubSystem: '<S5>/Subscribe'
  // Terminate for MATLABSystem: '<S14>/SourceBlock'
  matlabCodegenHandle_matla_mrfu4(&smo_estimator_DW.obj_m);

  // End of Terminate for SubSystem: '<S5>/Subscribe'
  emxFreeStruct_robotics_slmanip_(&smo_estimator_DW.obj_h);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_1_m);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_20_i);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_19_o);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_18_h);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_17_i);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_16_e);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_15_k);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_14_a);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_13_n);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_12_p);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_11_o);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_10_p);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_9_g);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_8_b);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_7_f);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_6_i);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_5_o);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_4_m);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_3_b);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_2_g);
  emxFreeStruct_robotics_slmani_m(&smo_estimator_DW.obj_o);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_1_g);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_20_n);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_19_i);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_18_f);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_17_l);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_16_b);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_15_c);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_14_m);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_13_d);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_12_d);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_11_h);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_10_h);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_9_m);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_8_h);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_7_n);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_6_h);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_5_n);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_4_p);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_3_b3);
  emxFreeStruct_o_robotics_mani_m(&smo_estimator_DW.gobj_2_i);

  // Terminate for Atomic SubSystem: '<S5>/Subscribe1'
  // Terminate for MATLABSystem: '<S15>/SourceBlock'
  matlabCodegenHandle_matla_mrfu4(&smo_estimator_DW.obj_ay);

  // End of Terminate for SubSystem: '<S5>/Subscribe1'
  emxFreeStruct_robotics_slman_mr(&smo_estimator_DW.obj);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_1);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_20);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_19);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_18);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_17);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_16);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_15);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_14);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_13);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_12);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_11);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_10);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_9);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_8);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_7);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_6);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_5);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_4);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_3);
  emxFreeStruct_o_robotics_manip_(&smo_estimator_DW.gobj_2);

  // Terminate for MATLABSystem: '<S3>/Get Parameter7'
  matlabCodegenHandle_matlabCodeg(&smo_estimator_DW.obj_a);

  // Terminate for MATLABSystem: '<S3>/Get Parameter8'
  matlabCodegenHandle_matlabCodeg(&smo_estimator_DW.obj_oo);

  // Terminate for MATLABSystem: '<S3>/Get Parameter9'
  matlabCodegenHandle_matlabCodeg(&smo_estimator_DW.obj_ok);

  // Terminate for Atomic SubSystem: '<S4>/Publish2'
  // Terminate for MATLABSystem: '<S12>/SinkBlock'
  matlabCodegenHandle_matlabC_mrf(&smo_estimator_DW.obj_ap);

  // End of Terminate for SubSystem: '<S4>/Publish2'

  // Terminate for Atomic SubSystem: '<S4>/Publish3'
  // Terminate for MATLABSystem: '<S13>/SinkBlock'
  matlabCodegenHandle_matlabC_mrf(&smo_estimator_DW.obj_b);

  // End of Terminate for SubSystem: '<S4>/Publish3'
}

//
// File trailer for generated code.
//
// [EOF]
//
