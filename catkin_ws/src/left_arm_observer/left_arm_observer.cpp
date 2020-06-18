//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_observer.cpp
//
// Code generated for Simulink model 'left_arm_observer'.
//
// Model version                  : 1.173
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Wed Jun 17 14:56:22 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "left_arm_observer.h"
#include "left_arm_observer_private.h"

// Block signals (default storage)
B_left_arm_observer_T left_arm_observer_B;

// Continuous states
X_left_arm_observer_T left_arm_observer_X;

// Block states (default storage)
DW_left_arm_observer_T left_arm_observer_DW;

// Real-time model
RT_MODEL_left_arm_observer_T left_arm_observer_M_ = RT_MODEL_left_arm_observer_T
  ();
RT_MODEL_left_arm_observer_T *const left_arm_observer_M = &left_arm_observer_M_;

// Forward declaration for local functions
static void left_arm_observe_emxInit_real_T(emxArray_real_T_left_arm_obse_T
  **pEmxArray, int32_T numDimensions);
static void left_arm_observ_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_observer_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0);
static void left_a_emxEnsureCapacity_real_T(emxArray_real_T_left_arm_obse_T
  *emxArray, int32_T oldNumel);
static void left_arm_ob_emxInit_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions);
static void l_emxEnsureCapacity_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel);
static void le_rigidBodyJoint_get_JointAxis(const
  rigidBodyJoint_left_arm_obser_T *obj, real_T ax[3]);
static void left_arm_observer_normalizeRows(const real_T matrix[3], real_T
  normRowMatrix[3]);
static void left_arm_observer_cat(real_T varargin_1, real_T varargin_2, real_T
  varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6, real_T
  varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void rigidBodyJoint_transformBodyT_i(const
  rigidBodyJoint_left_arm_obser_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16]);
static void rigidBodyJoint_transformBodyToP(const
  rigidBodyJoint_left_arm_obser_T *obj, real_T T[16]);
static void left_arm_observer_tforminv(const real_T T[16], real_T Tinv[16]);
static void left_arm_ob_tformToSpatialXform(const real_T T[16], real_T X[36]);
static void left_arm_observe_emxFree_real_T(emxArray_real_T_left_arm_obse_T
  **pEmxArray);
static void left_arm_ob_emxFree_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMat_i(k_robotics_manip_internal__ih_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_obse_T *H);
static c_robotics_core_internal_code_T *NameValueParser_NameValueParser
  (c_robotics_core_internal_code_T *obj);
static real_T left_arm_observer_rt_powd_snf(real_T u0, real_T u1);
static void left_arm__generateQuinticCoeffs(const real_T posPts[2], const real_T
  velPts[2], const real_T accPts[2], real_T finalTime, real_T coeffVec[6]);
static void le_addFlatSegmentsToPPFormParts(const real_T oldbreaks[2], const
  real_T oldCoeffs[42], real_T newBreaks[4], real_T newCoefs[126]);
static void left_arm_observer_ppval(const real_T pp_breaks[4], const real_T
  pp_coefs[126], const real_T x[2], real_T v[14]);
static void left_arm_obs_changeEndSegBreaks(const real_T oldBreaks[4], const
  real_T evalTime[2], real_T newBreaks[4]);
static void left_arm_o_polyCoeffsDerivative(const real_T coeffs[126], real_T
  dCoeffs[126]);
static void left_arm_observ_quinticpolytraj(const real_T wayPoints[14], const
  real_T timePoints[2], const real_T t[2], const real_T varargin_2[14], const
  real_T varargin_4[14], real_T q[14], real_T qd[14], real_T qdd[14], real_T
  pp_breaks[4], real_T pp_coefs[126]);
static void PolyTrajSys_computePPDerivative(const real_T pp_breaks[4], const
  real_T pp_coefs[126], real_T t, real_T ppd_breaks[4], real_T ppd_coefs[126],
  real_T ppdd_breaks[4], real_T ppdd_coefs[126]);
static void left_arm_observer_ppval_i(const real_T pp_breaks[4], const real_T
  pp_coefs[126], real_T x, real_T v[7]);
static void left_arm_o_PolyTrajSys_stepImpl(const
  robotics_slcore_internal_bloc_T *obj, real_T time, real_T q[7], real_T qd[7],
  real_T qdd[7]);
static void left_arm_observer_eye(real_T b_I[36]);
static void left_arm_observe_emxInit_char_T(emxArray_char_T_left_arm_obse_T
  **pEmxArray, int32_T numDimensions);
static void left_a_emxEnsureCapacity_char_T(emxArray_char_T_left_arm_obse_T
  *emxArray, int32_T oldNumel);
static void left_arm_observe_emxFree_char_T(emxArray_char_T_left_arm_obse_T
  **pEmxArray);
static boolean_T left_arm_observer_strcmp(const emxArray_char_T_left_arm_obse_T *
  a);
static void lef_GravityTorqueBlock_stepImpl(robotics_slmanip_internal_b_i_T *obj,
  const real_T q[7], real_T jointTorq[7]);
static void left_arm_ob_emxInit_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions);
static void l_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel);
static void left_arm_ob_emxFree_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray);
static void RigidBodyTreeDynamics_massMatri(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_obse_T *H,
  emxArray_real_T_left_arm_obse_T *lambda);
static void RigidBodyTreeDynamics_inverseDy(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], const real_T qdot[7], const
  emxArray_real_T_left_arm_obse_T *qddot, const real_T fext[60], real_T tau[7]);
static void matlabCodegenHandle_matlabCo_ih(ros_slros_internal_block_Subs_T *obj);
static void le_emxFreeStruct_rigidBodyJoint(rigidBodyJoint_left_arm_obser_T
  *pStruct);
static void emxFreeStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal__ih_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal__ih_T
  *pStruct);
static void emxFreeStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_k_robotics_mani_i(k_robotics_manip_internal_R_i_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_i(robotics_slmanip_internal_b_i_T
  *pStruct);
static void emxFreeStruct_k_robotics_man_ih(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slman_ih(robotics_slmanip_internal_blo_T
  *pStruct);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void le_emxInitStruct_rigidBodyJoint(rigidBodyJoint_left_arm_obser_T
  *pStruct);
static void emxInitStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal__ih_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal__ih_T
  *pStruct);
static void emxInitStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct);
static j_robotics_manip_internal_Rig_T *left_arm_ob_RigidBody_RigidBody
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left_arm__RigidBody_RigidBody_i
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left_arm_RigidBody_RigidBody_ih
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left_ar_RigidBody_RigidBody_ihp
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left_a_RigidBody_RigidBody_ihpp
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left__RigidBody_RigidBody_ihppq
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *left_RigidBody_RigidBody_ihppqb
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *lef_RigidBody_RigidBody_ihppqbe
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *le_RigidBody_RigidBody_ihppqbei
  (j_robotics_manip_internal_Rig_T *obj);
static j_robotics_manip_internal_Rig_T *l_RigidBody_RigidBody_ihppqbeig
  (j_robotics_manip_internal_Rig_T *obj);
static i_robotics_manip_internal_Rig_T *RigidBody_RigidBody_ihppqbeig2
  (i_robotics_manip_internal_Rig_T *obj);
static k_robotics_manip_internal__ih_T *RigidBodyTree_RigidBodyTree_ih
  (k_robotics_manip_internal__ih_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9);
static void emxInitStruct_k_robotics_mani_i(k_robotics_manip_internal_R_i_T
  *pStruct);
static void emxInitStruct_robotics_slmani_i(robotics_slmanip_internal_b_i_T
  *pStruct);
static k_robotics_manip_internal_R_i_T *l_RigidBodyTree_RigidBodyTree_i
  (k_robotics_manip_internal_R_i_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9);
static void emxInitStruct_k_robotics_man_ih(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slman_ih(robotics_slmanip_internal_blo_T
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

  (left_arm_observer_M->Timing.TaskCounters.TID[2])++;
  if ((left_arm_observer_M->Timing.TaskCounters.TID[2]) > 49) {// Sample time: [0.05s, 0.0s] 
    left_arm_observer_M->Timing.TaskCounters.TID[2] = 0;
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
  left_arm_observer_derivatives();

  // f1 = f(t + (h/2), y + (h/2)*f0)
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  left_arm_observer_step();
  left_arm_observer_derivatives();

  // f2 = f(t + (h/2), y + (h/2)*f1)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  left_arm_observer_step();
  left_arm_observer_derivatives();

  // f3 = f(t + h, y + h*f2)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  left_arm_observer_step();
  left_arm_observer_derivatives();

  // tnew = t + h
  // ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3)
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void left_arm_observe_emxInit_real_T(emxArray_real_T_left_arm_obse_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_left_arm_obse_T *emxArray;
  *pEmxArray = (emxArray_real_T_left_arm_obse_T *)malloc(sizeof
    (emxArray_real_T_left_arm_obse_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (left_arm_observer_B.i_c = 0; left_arm_observer_B.i_c < numDimensions;
       left_arm_observer_B.i_c++) {
    emxArray->size[left_arm_observer_B.i_c] = 0;
  }
}

static void left_arm_observ_SystemCore_step(boolean_T *varargout_1, real_T
  varargout_2_Data[7], uint32_T *varargout_2_Data_SL_Info_Curren, uint32_T
  *varargout_2_Data_SL_Info_Receiv, uint32_T *varargout_2_Layout_DataOffset,
  SL_Bus_left_arm_observer_std_msgs_MultiArrayDimension varargout_2_Layout_Dim
  [16], uint32_T *varargout_2_Layout_Dim_SL_Info_, uint32_T
  *varargout_2_Layout_Dim_SL_Inf_0)
{
  int32_T i;
  *varargout_1 = Sub_left_arm_observer_299.getLatestMessage
    (&left_arm_observer_B.b_varargout_2);
  for (i = 0; i < 7; i++) {
    varargout_2_Data[i] = left_arm_observer_B.b_varargout_2.Data[i];
  }

  *varargout_2_Data_SL_Info_Curren =
    left_arm_observer_B.b_varargout_2.Data_SL_Info.CurrentLength;
  *varargout_2_Data_SL_Info_Receiv =
    left_arm_observer_B.b_varargout_2.Data_SL_Info.ReceivedLength;
  *varargout_2_Layout_DataOffset =
    left_arm_observer_B.b_varargout_2.Layout.DataOffset;
  memcpy(&varargout_2_Layout_Dim[0],
         &left_arm_observer_B.b_varargout_2.Layout.Dim[0], sizeof
         (SL_Bus_left_arm_observer_std_msgs_MultiArrayDimension) << 4U);
  *varargout_2_Layout_Dim_SL_Info_ =
    left_arm_observer_B.b_varargout_2.Layout.Dim_SL_Info.CurrentLength;
  *varargout_2_Layout_Dim_SL_Inf_0 =
    left_arm_observer_B.b_varargout_2.Layout.Dim_SL_Info.ReceivedLength;
}

static void left_a_emxEnsureCapacity_real_T(emxArray_real_T_left_arm_obse_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_observer_B.newNumel = 1;
  for (left_arm_observer_B.i_f = 0; left_arm_observer_B.i_f <
       emxArray->numDimensions; left_arm_observer_B.i_f++) {
    left_arm_observer_B.newNumel *= emxArray->size[left_arm_observer_B.i_f];
  }

  if (left_arm_observer_B.newNumel > emxArray->allocatedSize) {
    left_arm_observer_B.i_f = emxArray->allocatedSize;
    if (left_arm_observer_B.i_f < 16) {
      left_arm_observer_B.i_f = 16;
    }

    while (left_arm_observer_B.i_f < left_arm_observer_B.newNumel) {
      if (left_arm_observer_B.i_f > 1073741823) {
        left_arm_observer_B.i_f = MAX_int32_T;
      } else {
        left_arm_observer_B.i_f <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_observer_B.i_f), sizeof
                     (real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = left_arm_observer_B.i_f;
    emxArray->canFreeData = true;
  }
}

static void left_arm_ob_emxInit_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_e_cell_wrap_left_arm_T *emxArray;
  *pEmxArray = (emxArray_e_cell_wrap_left_arm_T *)malloc(sizeof
    (emxArray_e_cell_wrap_left_arm_T));
  emxArray = *pEmxArray;
  emxArray->data = (e_cell_wrap_left_arm_observ_i_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (left_arm_observer_B.i_kn = 0; left_arm_observer_B.i_kn < numDimensions;
       left_arm_observer_B.i_kn++) {
    emxArray->size[left_arm_observer_B.i_kn] = 0;
  }
}

static void l_emxEnsureCapacity_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_observer_B.newNumel_n3 = 1;
  for (left_arm_observer_B.i_oy = 0; left_arm_observer_B.i_oy <
       emxArray->numDimensions; left_arm_observer_B.i_oy++) {
    left_arm_observer_B.newNumel_n3 *= emxArray->size[left_arm_observer_B.i_oy];
  }

  if (left_arm_observer_B.newNumel_n3 > emxArray->allocatedSize) {
    left_arm_observer_B.i_oy = emxArray->allocatedSize;
    if (left_arm_observer_B.i_oy < 16) {
      left_arm_observer_B.i_oy = 16;
    }

    while (left_arm_observer_B.i_oy < left_arm_observer_B.newNumel_n3) {
      if (left_arm_observer_B.i_oy > 1073741823) {
        left_arm_observer_B.i_oy = MAX_int32_T;
      } else {
        left_arm_observer_B.i_oy <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_observer_B.i_oy), sizeof
                     (e_cell_wrap_left_arm_observ_i_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(e_cell_wrap_left_arm_observ_i_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (e_cell_wrap_left_arm_observ_i_T *)newData;
    emxArray->allocatedSize = left_arm_observer_B.i_oy;
    emxArray->canFreeData = true;
  }
}

static void le_rigidBodyJoint_get_JointAxis(const
  rigidBodyJoint_left_arm_obser_T *obj, real_T ax[3])
{
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  boolean_T guard1 = false;
  int32_T exitg1;
  for (left_arm_observer_B.b_kstr_m = 0; left_arm_observer_B.b_kstr_m < 8;
       left_arm_observer_B.b_kstr_m++) {
    left_arm_observer_B.b_o[left_arm_observer_B.b_kstr_m] =
      tmp[left_arm_observer_B.b_kstr_m];
  }

  left_arm_observer_B.b_bool_cx = false;
  if (obj->Type->size[1] == 8) {
    left_arm_observer_B.b_kstr_m = 1;
    do {
      exitg1 = 0;
      if (left_arm_observer_B.b_kstr_m - 1 < 8) {
        left_arm_observer_B.kstr_p = left_arm_observer_B.b_kstr_m - 1;
        if (obj->Type->data[left_arm_observer_B.kstr_p] !=
            left_arm_observer_B.b_o[left_arm_observer_B.kstr_p]) {
          exitg1 = 1;
        } else {
          left_arm_observer_B.b_kstr_m++;
        }
      } else {
        left_arm_observer_B.b_bool_cx = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (left_arm_observer_B.b_bool_cx) {
    guard1 = true;
  } else {
    for (left_arm_observer_B.b_kstr_m = 0; left_arm_observer_B.b_kstr_m < 9;
         left_arm_observer_B.b_kstr_m++) {
      left_arm_observer_B.b_i[left_arm_observer_B.b_kstr_m] =
        tmp_0[left_arm_observer_B.b_kstr_m];
    }

    left_arm_observer_B.b_bool_cx = false;
    if (obj->Type->size[1] == 9) {
      left_arm_observer_B.b_kstr_m = 1;
      do {
        exitg1 = 0;
        if (left_arm_observer_B.b_kstr_m - 1 < 9) {
          left_arm_observer_B.kstr_p = left_arm_observer_B.b_kstr_m - 1;
          if (obj->Type->data[left_arm_observer_B.kstr_p] !=
              left_arm_observer_B.b_i[left_arm_observer_B.kstr_p]) {
            exitg1 = 1;
          } else {
            left_arm_observer_B.b_kstr_m++;
          }
        } else {
          left_arm_observer_B.b_bool_cx = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_observer_B.b_bool_cx) {
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

static void left_arm_observer_normalizeRows(const real_T matrix[3], real_T
  normRowMatrix[3])
{
  left_arm_observer_B.b_h = 1.0 / sqrt((matrix[0] * matrix[0] + matrix[1] *
    matrix[1]) + matrix[2] * matrix[2]);
  normRowMatrix[0] = matrix[0] * left_arm_observer_B.b_h;
  normRowMatrix[1] = matrix[1] * left_arm_observer_B.b_h;
  normRowMatrix[2] = matrix[2] * left_arm_observer_B.b_h;
}

static void left_arm_observer_cat(real_T varargin_1, real_T varargin_2, real_T
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

static void rigidBodyJoint_transformBodyT_i(const
  rigidBodyJoint_left_arm_obser_T *obj, const real_T q_data[], const int32_T
  *q_size, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (left_arm_observer_B.b_kstr = 0; left_arm_observer_B.b_kstr < 5;
       left_arm_observer_B.b_kstr++) {
    left_arm_observer_B.b_g[left_arm_observer_B.b_kstr] =
      tmp[left_arm_observer_B.b_kstr];
  }

  left_arm_observer_B.b_bool_c = false;
  if (obj->Type->size[1] == 5) {
    left_arm_observer_B.b_kstr = 1;
    do {
      exitg1 = 0;
      if (left_arm_observer_B.b_kstr - 1 < 5) {
        left_arm_observer_B.kstr = left_arm_observer_B.b_kstr - 1;
        if (obj->Type->data[left_arm_observer_B.kstr] !=
            left_arm_observer_B.b_g[left_arm_observer_B.kstr]) {
          exitg1 = 1;
        } else {
          left_arm_observer_B.b_kstr++;
        }
      } else {
        left_arm_observer_B.b_bool_c = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (left_arm_observer_B.b_bool_c) {
    left_arm_observer_B.b_kstr = 0;
  } else {
    for (left_arm_observer_B.b_kstr = 0; left_arm_observer_B.b_kstr < 8;
         left_arm_observer_B.b_kstr++) {
      left_arm_observer_B.b_l[left_arm_observer_B.b_kstr] =
        tmp_0[left_arm_observer_B.b_kstr];
    }

    left_arm_observer_B.b_bool_c = false;
    if (obj->Type->size[1] == 8) {
      left_arm_observer_B.b_kstr = 1;
      do {
        exitg1 = 0;
        if (left_arm_observer_B.b_kstr - 1 < 8) {
          left_arm_observer_B.kstr = left_arm_observer_B.b_kstr - 1;
          if (obj->Type->data[left_arm_observer_B.kstr] !=
              left_arm_observer_B.b_l[left_arm_observer_B.kstr]) {
            exitg1 = 1;
          } else {
            left_arm_observer_B.b_kstr++;
          }
        } else {
          left_arm_observer_B.b_bool_c = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_observer_B.b_bool_c) {
      left_arm_observer_B.b_kstr = 1;
    } else {
      left_arm_observer_B.b_kstr = -1;
    }
  }

  switch (left_arm_observer_B.b_kstr) {
   case 0:
    memset(&left_arm_observer_B.TJ[0], 0, sizeof(real_T) << 4U);
    left_arm_observer_B.TJ[0] = 1.0;
    left_arm_observer_B.TJ[5] = 1.0;
    left_arm_observer_B.TJ[10] = 1.0;
    left_arm_observer_B.TJ[15] = 1.0;
    break;

   case 1:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_observer_B.v);
    left_arm_observer_B.result_data[0] = left_arm_observer_B.v[0];
    left_arm_observer_B.result_data[1] = left_arm_observer_B.v[1];
    left_arm_observer_B.result_data[2] = left_arm_observer_B.v[2];
    if (0 <= (*q_size != 0) - 1) {
      left_arm_observer_B.result_data[3] = q_data[0];
    }

    left_arm_observer_normalizeRows(&left_arm_observer_B.result_data[0],
      left_arm_observer_B.v);
    left_arm_observer_B.cth = cos(left_arm_observer_B.result_data[3]);
    left_arm_observer_B.sth = sin(left_arm_observer_B.result_data[3]);
    left_arm_observer_B.tempR_tmp = left_arm_observer_B.v[1] *
      left_arm_observer_B.v[0] * (1.0 - left_arm_observer_B.cth);
    left_arm_observer_B.tempR_tmp_i = left_arm_observer_B.v[2] *
      left_arm_observer_B.sth;
    left_arm_observer_B.tempR_tmp_f = left_arm_observer_B.v[2] *
      left_arm_observer_B.v[0] * (1.0 - left_arm_observer_B.cth);
    left_arm_observer_B.tempR_tmp_g = left_arm_observer_B.v[1] *
      left_arm_observer_B.sth;
    left_arm_observer_B.tempR_tmp_c = left_arm_observer_B.v[2] *
      left_arm_observer_B.v[1] * (1.0 - left_arm_observer_B.cth);
    left_arm_observer_B.sth *= left_arm_observer_B.v[0];
    left_arm_observer_cat(left_arm_observer_B.v[0] * left_arm_observer_B.v[0] *
                          (1.0 - left_arm_observer_B.cth) +
                          left_arm_observer_B.cth, left_arm_observer_B.tempR_tmp
                          - left_arm_observer_B.tempR_tmp_i,
                          left_arm_observer_B.tempR_tmp_f +
                          left_arm_observer_B.tempR_tmp_g,
                          left_arm_observer_B.tempR_tmp +
                          left_arm_observer_B.tempR_tmp_i,
                          left_arm_observer_B.v[1] * left_arm_observer_B.v[1] *
                          (1.0 - left_arm_observer_B.cth) +
                          left_arm_observer_B.cth,
                          left_arm_observer_B.tempR_tmp_c -
                          left_arm_observer_B.sth,
                          left_arm_observer_B.tempR_tmp_f -
                          left_arm_observer_B.tempR_tmp_g,
                          left_arm_observer_B.tempR_tmp_c +
                          left_arm_observer_B.sth, left_arm_observer_B.v[2] *
                          left_arm_observer_B.v[2] * (1.0 -
      left_arm_observer_B.cth) + left_arm_observer_B.cth,
                          left_arm_observer_B.tempR_d);
    for (left_arm_observer_B.b_kstr = 0; left_arm_observer_B.b_kstr < 3;
         left_arm_observer_B.b_kstr++) {
      left_arm_observer_B.kstr = left_arm_observer_B.b_kstr + 1;
      left_arm_observer_B.R_l[left_arm_observer_B.kstr - 1] =
        left_arm_observer_B.tempR_d[(left_arm_observer_B.kstr - 1) * 3];
      left_arm_observer_B.kstr = left_arm_observer_B.b_kstr + 1;
      left_arm_observer_B.R_l[left_arm_observer_B.kstr + 2] =
        left_arm_observer_B.tempR_d[(left_arm_observer_B.kstr - 1) * 3 + 1];
      left_arm_observer_B.kstr = left_arm_observer_B.b_kstr + 1;
      left_arm_observer_B.R_l[left_arm_observer_B.kstr + 5] =
        left_arm_observer_B.tempR_d[(left_arm_observer_B.kstr - 1) * 3 + 2];
    }

    memset(&left_arm_observer_B.TJ[0], 0, sizeof(real_T) << 4U);
    for (left_arm_observer_B.b_kstr = 0; left_arm_observer_B.b_kstr < 3;
         left_arm_observer_B.b_kstr++) {
      left_arm_observer_B.kstr = left_arm_observer_B.b_kstr << 2;
      left_arm_observer_B.TJ[left_arm_observer_B.kstr] =
        left_arm_observer_B.R_l[3 * left_arm_observer_B.b_kstr];
      left_arm_observer_B.TJ[left_arm_observer_B.kstr + 1] =
        left_arm_observer_B.R_l[3 * left_arm_observer_B.b_kstr + 1];
      left_arm_observer_B.TJ[left_arm_observer_B.kstr + 2] =
        left_arm_observer_B.R_l[3 * left_arm_observer_B.b_kstr + 2];
    }

    left_arm_observer_B.TJ[15] = 1.0;
    break;

   default:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_observer_B.v);
    memset(&left_arm_observer_B.tempR_d[0], 0, 9U * sizeof(real_T));
    left_arm_observer_B.tempR_d[0] = 1.0;
    left_arm_observer_B.tempR_d[4] = 1.0;
    left_arm_observer_B.tempR_d[8] = 1.0;
    for (left_arm_observer_B.b_kstr = 0; left_arm_observer_B.b_kstr < 3;
         left_arm_observer_B.b_kstr++) {
      left_arm_observer_B.kstr = left_arm_observer_B.b_kstr << 2;
      left_arm_observer_B.TJ[left_arm_observer_B.kstr] =
        left_arm_observer_B.tempR_d[3 * left_arm_observer_B.b_kstr];
      left_arm_observer_B.TJ[left_arm_observer_B.kstr + 1] =
        left_arm_observer_B.tempR_d[3 * left_arm_observer_B.b_kstr + 1];
      left_arm_observer_B.TJ[left_arm_observer_B.kstr + 2] =
        left_arm_observer_B.tempR_d[3 * left_arm_observer_B.b_kstr + 2];
      left_arm_observer_B.TJ[left_arm_observer_B.b_kstr + 12] =
        left_arm_observer_B.v[left_arm_observer_B.b_kstr] * q_data[0];
    }

    left_arm_observer_B.TJ[3] = 0.0;
    left_arm_observer_B.TJ[7] = 0.0;
    left_arm_observer_B.TJ[11] = 0.0;
    left_arm_observer_B.TJ[15] = 1.0;
    break;
  }

  for (left_arm_observer_B.b_kstr = 0; left_arm_observer_B.b_kstr < 4;
       left_arm_observer_B.b_kstr++) {
    for (left_arm_observer_B.kstr = 0; left_arm_observer_B.kstr < 4;
         left_arm_observer_B.kstr++) {
      left_arm_observer_B.obj_tmp_tmp = left_arm_observer_B.kstr << 2;
      left_arm_observer_B.obj_tmp = left_arm_observer_B.b_kstr +
        left_arm_observer_B.obj_tmp_tmp;
      left_arm_observer_B.obj[left_arm_observer_B.obj_tmp] = 0.0;
      left_arm_observer_B.obj[left_arm_observer_B.obj_tmp] +=
        left_arm_observer_B.TJ[left_arm_observer_B.obj_tmp_tmp] *
        obj->JointToParentTransform[left_arm_observer_B.b_kstr];
      left_arm_observer_B.obj[left_arm_observer_B.obj_tmp] +=
        left_arm_observer_B.TJ[left_arm_observer_B.obj_tmp_tmp + 1] *
        obj->JointToParentTransform[left_arm_observer_B.b_kstr + 4];
      left_arm_observer_B.obj[left_arm_observer_B.obj_tmp] +=
        left_arm_observer_B.TJ[left_arm_observer_B.obj_tmp_tmp + 2] *
        obj->JointToParentTransform[left_arm_observer_B.b_kstr + 8];
      left_arm_observer_B.obj[left_arm_observer_B.obj_tmp] +=
        left_arm_observer_B.TJ[left_arm_observer_B.obj_tmp_tmp + 3] *
        obj->JointToParentTransform[left_arm_observer_B.b_kstr + 12];
    }

    for (left_arm_observer_B.kstr = 0; left_arm_observer_B.kstr < 4;
         left_arm_observer_B.kstr++) {
      left_arm_observer_B.obj_tmp_tmp = left_arm_observer_B.kstr << 2;
      left_arm_observer_B.obj_tmp = left_arm_observer_B.b_kstr +
        left_arm_observer_B.obj_tmp_tmp;
      T[left_arm_observer_B.obj_tmp] = 0.0;
      T[left_arm_observer_B.obj_tmp] += obj->
        ChildToJointTransform[left_arm_observer_B.obj_tmp_tmp] *
        left_arm_observer_B.obj[left_arm_observer_B.b_kstr];
      T[left_arm_observer_B.obj_tmp] += obj->
        ChildToJointTransform[left_arm_observer_B.obj_tmp_tmp + 1] *
        left_arm_observer_B.obj[left_arm_observer_B.b_kstr + 4];
      T[left_arm_observer_B.obj_tmp] += obj->
        ChildToJointTransform[left_arm_observer_B.obj_tmp_tmp + 2] *
        left_arm_observer_B.obj[left_arm_observer_B.b_kstr + 8];
      T[left_arm_observer_B.obj_tmp] += obj->
        ChildToJointTransform[left_arm_observer_B.obj_tmp_tmp + 3] *
        left_arm_observer_B.obj[left_arm_observer_B.b_kstr + 12];
    }
  }
}

static void rigidBodyJoint_transformBodyToP(const
  rigidBodyJoint_left_arm_obser_T *obj, real_T T[16])
{
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  for (left_arm_observer_B.b_kstr_o = 0; left_arm_observer_B.b_kstr_o < 5;
       left_arm_observer_B.b_kstr_o++) {
    left_arm_observer_B.b_fi[left_arm_observer_B.b_kstr_o] =
      tmp[left_arm_observer_B.b_kstr_o];
  }

  left_arm_observer_B.b_bool_d = false;
  if (obj->Type->size[1] == 5) {
    left_arm_observer_B.b_kstr_o = 1;
    do {
      exitg1 = 0;
      if (left_arm_observer_B.b_kstr_o - 1 < 5) {
        left_arm_observer_B.kstr_i = left_arm_observer_B.b_kstr_o - 1;
        if (obj->Type->data[left_arm_observer_B.kstr_i] !=
            left_arm_observer_B.b_fi[left_arm_observer_B.kstr_i]) {
          exitg1 = 1;
        } else {
          left_arm_observer_B.b_kstr_o++;
        }
      } else {
        left_arm_observer_B.b_bool_d = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (left_arm_observer_B.b_bool_d) {
    left_arm_observer_B.b_kstr_o = 0;
  } else {
    for (left_arm_observer_B.b_kstr_o = 0; left_arm_observer_B.b_kstr_o < 8;
         left_arm_observer_B.b_kstr_o++) {
      left_arm_observer_B.b_ip[left_arm_observer_B.b_kstr_o] =
        tmp_0[left_arm_observer_B.b_kstr_o];
    }

    left_arm_observer_B.b_bool_d = false;
    if (obj->Type->size[1] == 8) {
      left_arm_observer_B.b_kstr_o = 1;
      do {
        exitg1 = 0;
        if (left_arm_observer_B.b_kstr_o - 1 < 8) {
          left_arm_observer_B.kstr_i = left_arm_observer_B.b_kstr_o - 1;
          if (obj->Type->data[left_arm_observer_B.kstr_i] !=
              left_arm_observer_B.b_ip[left_arm_observer_B.kstr_i]) {
            exitg1 = 1;
          } else {
            left_arm_observer_B.b_kstr_o++;
          }
        } else {
          left_arm_observer_B.b_bool_d = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (left_arm_observer_B.b_bool_d) {
      left_arm_observer_B.b_kstr_o = 1;
    } else {
      left_arm_observer_B.b_kstr_o = -1;
    }
  }

  switch (left_arm_observer_B.b_kstr_o) {
   case 0:
    memset(&left_arm_observer_B.TJ_p[0], 0, sizeof(real_T) << 4U);
    left_arm_observer_B.TJ_p[0] = 1.0;
    left_arm_observer_B.TJ_p[5] = 1.0;
    left_arm_observer_B.TJ_p[10] = 1.0;
    left_arm_observer_B.TJ_p[15] = 1.0;
    break;

   case 1:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_observer_B.v_c);
    left_arm_observer_B.v_ct[0] = left_arm_observer_B.v_c[0];
    left_arm_observer_B.v_ct[1] = left_arm_observer_B.v_c[1];
    left_arm_observer_B.v_ct[2] = left_arm_observer_B.v_c[2];
    left_arm_observer_normalizeRows(left_arm_observer_B.v_ct,
      left_arm_observer_B.v_c);
    left_arm_observer_B.tempR_tmp_o = left_arm_observer_B.v_c[1] *
      left_arm_observer_B.v_c[0] * 0.0;
    left_arm_observer_B.tempR_tmp_h = left_arm_observer_B.v_c[2] *
      left_arm_observer_B.v_c[0] * 0.0;
    left_arm_observer_B.tempR_tmp_l = left_arm_observer_B.v_c[2] *
      left_arm_observer_B.v_c[1] * 0.0;
    left_arm_observer_cat(left_arm_observer_B.v_c[0] * left_arm_observer_B.v_c[0]
                          * 0.0 + 1.0, left_arm_observer_B.tempR_tmp_o -
                          left_arm_observer_B.v_c[2] * 0.0,
                          left_arm_observer_B.tempR_tmp_h +
                          left_arm_observer_B.v_c[1] * 0.0,
                          left_arm_observer_B.tempR_tmp_o +
                          left_arm_observer_B.v_c[2] * 0.0,
                          left_arm_observer_B.v_c[1] * left_arm_observer_B.v_c[1]
                          * 0.0 + 1.0, left_arm_observer_B.tempR_tmp_l -
                          left_arm_observer_B.v_c[0] * 0.0,
                          left_arm_observer_B.tempR_tmp_h -
                          left_arm_observer_B.v_c[1] * 0.0,
                          left_arm_observer_B.tempR_tmp_l +
                          left_arm_observer_B.v_c[0] * 0.0,
                          left_arm_observer_B.v_c[2] * left_arm_observer_B.v_c[2]
                          * 0.0 + 1.0, left_arm_observer_B.tempR_b);
    for (left_arm_observer_B.b_kstr_o = 0; left_arm_observer_B.b_kstr_o < 3;
         left_arm_observer_B.b_kstr_o++) {
      left_arm_observer_B.kstr_i = left_arm_observer_B.b_kstr_o + 1;
      left_arm_observer_B.R_o[left_arm_observer_B.kstr_i - 1] =
        left_arm_observer_B.tempR_b[(left_arm_observer_B.kstr_i - 1) * 3];
      left_arm_observer_B.kstr_i = left_arm_observer_B.b_kstr_o + 1;
      left_arm_observer_B.R_o[left_arm_observer_B.kstr_i + 2] =
        left_arm_observer_B.tempR_b[(left_arm_observer_B.kstr_i - 1) * 3 + 1];
      left_arm_observer_B.kstr_i = left_arm_observer_B.b_kstr_o + 1;
      left_arm_observer_B.R_o[left_arm_observer_B.kstr_i + 5] =
        left_arm_observer_B.tempR_b[(left_arm_observer_B.kstr_i - 1) * 3 + 2];
    }

    memset(&left_arm_observer_B.TJ_p[0], 0, sizeof(real_T) << 4U);
    for (left_arm_observer_B.b_kstr_o = 0; left_arm_observer_B.b_kstr_o < 3;
         left_arm_observer_B.b_kstr_o++) {
      left_arm_observer_B.kstr_i = left_arm_observer_B.b_kstr_o << 2;
      left_arm_observer_B.TJ_p[left_arm_observer_B.kstr_i] =
        left_arm_observer_B.R_o[3 * left_arm_observer_B.b_kstr_o];
      left_arm_observer_B.TJ_p[left_arm_observer_B.kstr_i + 1] =
        left_arm_observer_B.R_o[3 * left_arm_observer_B.b_kstr_o + 1];
      left_arm_observer_B.TJ_p[left_arm_observer_B.kstr_i + 2] =
        left_arm_observer_B.R_o[3 * left_arm_observer_B.b_kstr_o + 2];
    }

    left_arm_observer_B.TJ_p[15] = 1.0;
    break;

   default:
    le_rigidBodyJoint_get_JointAxis(obj, left_arm_observer_B.v_c);
    memset(&left_arm_observer_B.tempR_b[0], 0, 9U * sizeof(real_T));
    left_arm_observer_B.tempR_b[0] = 1.0;
    left_arm_observer_B.tempR_b[4] = 1.0;
    left_arm_observer_B.tempR_b[8] = 1.0;
    for (left_arm_observer_B.b_kstr_o = 0; left_arm_observer_B.b_kstr_o < 3;
         left_arm_observer_B.b_kstr_o++) {
      left_arm_observer_B.kstr_i = left_arm_observer_B.b_kstr_o << 2;
      left_arm_observer_B.TJ_p[left_arm_observer_B.kstr_i] =
        left_arm_observer_B.tempR_b[3 * left_arm_observer_B.b_kstr_o];
      left_arm_observer_B.TJ_p[left_arm_observer_B.kstr_i + 1] =
        left_arm_observer_B.tempR_b[3 * left_arm_observer_B.b_kstr_o + 1];
      left_arm_observer_B.TJ_p[left_arm_observer_B.kstr_i + 2] =
        left_arm_observer_B.tempR_b[3 * left_arm_observer_B.b_kstr_o + 2];
      left_arm_observer_B.TJ_p[left_arm_observer_B.b_kstr_o + 12] =
        left_arm_observer_B.v_c[left_arm_observer_B.b_kstr_o] * 0.0;
    }

    left_arm_observer_B.TJ_p[3] = 0.0;
    left_arm_observer_B.TJ_p[7] = 0.0;
    left_arm_observer_B.TJ_p[11] = 0.0;
    left_arm_observer_B.TJ_p[15] = 1.0;
    break;
  }

  for (left_arm_observer_B.b_kstr_o = 0; left_arm_observer_B.b_kstr_o < 4;
       left_arm_observer_B.b_kstr_o++) {
    for (left_arm_observer_B.kstr_i = 0; left_arm_observer_B.kstr_i < 4;
         left_arm_observer_B.kstr_i++) {
      left_arm_observer_B.obj_tmp_tmp_c = left_arm_observer_B.kstr_i << 2;
      left_arm_observer_B.obj_tmp_m = left_arm_observer_B.b_kstr_o +
        left_arm_observer_B.obj_tmp_tmp_c;
      left_arm_observer_B.obj_c[left_arm_observer_B.obj_tmp_m] = 0.0;
      left_arm_observer_B.obj_c[left_arm_observer_B.obj_tmp_m] +=
        left_arm_observer_B.TJ_p[left_arm_observer_B.obj_tmp_tmp_c] *
        obj->JointToParentTransform[left_arm_observer_B.b_kstr_o];
      left_arm_observer_B.obj_c[left_arm_observer_B.obj_tmp_m] +=
        left_arm_observer_B.TJ_p[left_arm_observer_B.obj_tmp_tmp_c + 1] *
        obj->JointToParentTransform[left_arm_observer_B.b_kstr_o + 4];
      left_arm_observer_B.obj_c[left_arm_observer_B.obj_tmp_m] +=
        left_arm_observer_B.TJ_p[left_arm_observer_B.obj_tmp_tmp_c + 2] *
        obj->JointToParentTransform[left_arm_observer_B.b_kstr_o + 8];
      left_arm_observer_B.obj_c[left_arm_observer_B.obj_tmp_m] +=
        left_arm_observer_B.TJ_p[left_arm_observer_B.obj_tmp_tmp_c + 3] *
        obj->JointToParentTransform[left_arm_observer_B.b_kstr_o + 12];
    }

    for (left_arm_observer_B.kstr_i = 0; left_arm_observer_B.kstr_i < 4;
         left_arm_observer_B.kstr_i++) {
      left_arm_observer_B.obj_tmp_tmp_c = left_arm_observer_B.kstr_i << 2;
      left_arm_observer_B.obj_tmp_m = left_arm_observer_B.b_kstr_o +
        left_arm_observer_B.obj_tmp_tmp_c;
      T[left_arm_observer_B.obj_tmp_m] = 0.0;
      T[left_arm_observer_B.obj_tmp_m] += obj->
        ChildToJointTransform[left_arm_observer_B.obj_tmp_tmp_c] *
        left_arm_observer_B.obj_c[left_arm_observer_B.b_kstr_o];
      T[left_arm_observer_B.obj_tmp_m] += obj->
        ChildToJointTransform[left_arm_observer_B.obj_tmp_tmp_c + 1] *
        left_arm_observer_B.obj_c[left_arm_observer_B.b_kstr_o + 4];
      T[left_arm_observer_B.obj_tmp_m] += obj->
        ChildToJointTransform[left_arm_observer_B.obj_tmp_tmp_c + 2] *
        left_arm_observer_B.obj_c[left_arm_observer_B.b_kstr_o + 8];
      T[left_arm_observer_B.obj_tmp_m] += obj->
        ChildToJointTransform[left_arm_observer_B.obj_tmp_tmp_c + 3] *
        left_arm_observer_B.obj_c[left_arm_observer_B.b_kstr_o + 12];
    }
  }
}

static void left_arm_observer_tforminv(const real_T T[16], real_T Tinv[16])
{
  for (left_arm_observer_B.i5 = 0; left_arm_observer_B.i5 < 3;
       left_arm_observer_B.i5++) {
    left_arm_observer_B.R_n[3 * left_arm_observer_B.i5] =
      T[left_arm_observer_B.i5];
    left_arm_observer_B.R_n[3 * left_arm_observer_B.i5 + 1] =
      T[left_arm_observer_B.i5 + 4];
    left_arm_observer_B.R_n[3 * left_arm_observer_B.i5 + 2] =
      T[left_arm_observer_B.i5 + 8];
  }

  for (left_arm_observer_B.i5 = 0; left_arm_observer_B.i5 < 9;
       left_arm_observer_B.i5++) {
    left_arm_observer_B.R_b[left_arm_observer_B.i5] =
      -left_arm_observer_B.R_n[left_arm_observer_B.i5];
  }

  for (left_arm_observer_B.i5 = 0; left_arm_observer_B.i5 < 3;
       left_arm_observer_B.i5++) {
    left_arm_observer_B.Tinv_tmp = left_arm_observer_B.i5 << 2;
    Tinv[left_arm_observer_B.Tinv_tmp] = left_arm_observer_B.R_n[3 *
      left_arm_observer_B.i5];
    Tinv[left_arm_observer_B.Tinv_tmp + 1] = left_arm_observer_B.R_n[3 *
      left_arm_observer_B.i5 + 1];
    Tinv[left_arm_observer_B.Tinv_tmp + 2] = left_arm_observer_B.R_n[3 *
      left_arm_observer_B.i5 + 2];
    Tinv[left_arm_observer_B.i5 + 12] =
      left_arm_observer_B.R_b[left_arm_observer_B.i5 + 6] * T[14] +
      (left_arm_observer_B.R_b[left_arm_observer_B.i5 + 3] * T[13] +
       left_arm_observer_B.R_b[left_arm_observer_B.i5] * T[12]);
  }

  Tinv[3] = 0.0;
  Tinv[7] = 0.0;
  Tinv[11] = 0.0;
  Tinv[15] = 1.0;
}

static void left_arm_ob_tformToSpatialXform(const real_T T[16], real_T X[36])
{
  left_arm_observer_B.dv6[0] = 0.0;
  left_arm_observer_B.dv6[3] = -T[14];
  left_arm_observer_B.dv6[6] = T[13];
  left_arm_observer_B.dv6[1] = T[14];
  left_arm_observer_B.dv6[4] = 0.0;
  left_arm_observer_B.dv6[7] = -T[12];
  left_arm_observer_B.dv6[2] = -T[13];
  left_arm_observer_B.dv6[5] = T[12];
  left_arm_observer_B.dv6[8] = 0.0;
  for (left_arm_observer_B.i3 = 0; left_arm_observer_B.i3 < 3;
       left_arm_observer_B.i3++) {
    for (left_arm_observer_B.X_tmp = 0; left_arm_observer_B.X_tmp < 3;
         left_arm_observer_B.X_tmp++) {
      left_arm_observer_B.X_tmp_i = left_arm_observer_B.i3 + 3 *
        left_arm_observer_B.X_tmp;
      left_arm_observer_B.dv7[left_arm_observer_B.X_tmp_i] = 0.0;
      left_arm_observer_B.i4 = left_arm_observer_B.X_tmp << 2;
      left_arm_observer_B.dv7[left_arm_observer_B.X_tmp_i] +=
        T[left_arm_observer_B.i4] *
        left_arm_observer_B.dv6[left_arm_observer_B.i3];
      left_arm_observer_B.dv7[left_arm_observer_B.X_tmp_i] +=
        T[left_arm_observer_B.i4 + 1] *
        left_arm_observer_B.dv6[left_arm_observer_B.i3 + 3];
      left_arm_observer_B.dv7[left_arm_observer_B.X_tmp_i] +=
        T[left_arm_observer_B.i4 + 2] *
        left_arm_observer_B.dv6[left_arm_observer_B.i3 + 6];
      X[left_arm_observer_B.X_tmp + 6 * left_arm_observer_B.i3] = T
        [(left_arm_observer_B.i3 << 2) + left_arm_observer_B.X_tmp];
      X[left_arm_observer_B.X_tmp + 6 * (left_arm_observer_B.i3 + 3)] = 0.0;
    }
  }

  for (left_arm_observer_B.i3 = 0; left_arm_observer_B.i3 < 3;
       left_arm_observer_B.i3++) {
    X[6 * left_arm_observer_B.i3 + 3] = left_arm_observer_B.dv7[3 *
      left_arm_observer_B.i3];
    left_arm_observer_B.X_tmp = left_arm_observer_B.i3 << 2;
    left_arm_observer_B.X_tmp_i = 6 * (left_arm_observer_B.i3 + 3);
    X[left_arm_observer_B.X_tmp_i + 3] = T[left_arm_observer_B.X_tmp];
    X[6 * left_arm_observer_B.i3 + 4] = left_arm_observer_B.dv7[3 *
      left_arm_observer_B.i3 + 1];
    X[left_arm_observer_B.X_tmp_i + 4] = T[left_arm_observer_B.X_tmp + 1];
    X[6 * left_arm_observer_B.i3 + 5] = left_arm_observer_B.dv7[3 *
      left_arm_observer_B.i3 + 2];
    X[left_arm_observer_B.X_tmp_i + 5] = T[left_arm_observer_B.X_tmp + 2];
  }
}

static void left_arm_observe_emxFree_real_T(emxArray_real_T_left_arm_obse_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_left_arm_obse_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_left_arm_obse_T *)NULL;
  }
}

static void left_arm_ob_emxFree_e_cell_wrap(emxArray_e_cell_wrap_left_arm_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_e_cell_wrap_left_arm_T *)NULL) {
    if (((*pEmxArray)->data != (e_cell_wrap_left_arm_observ_i_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_e_cell_wrap_left_arm_T *)NULL;
  }
}

static void RigidBodyTreeDynamics_massMat_i(k_robotics_manip_internal__ih_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_obse_T *H)
{
  emxArray_e_cell_wrap_left_arm_T *Ic;
  emxArray_e_cell_wrap_left_arm_T *X;
  emxArray_real_T_left_arm_obse_T *Si;
  emxArray_real_T_left_arm_obse_T *Fi;
  emxArray_real_T_left_arm_obse_T *Sj;
  emxArray_real_T_left_arm_obse_T *Hji;
  j_robotics_manip_internal_Rig_T *obj;
  emxArray_real_T_left_arm_obse_T *a;
  emxArray_real_T_left_arm_obse_T *B;
  left_arm_observer_B.nb_k = robot->NumBodies;
  left_arm_observer_B.vNum_p = robot->VelocityNumber;
  left_arm_observer_B.f = H->size[0] * H->size[1];
  left_arm_observer_B.b_i_m = static_cast<int32_T>(left_arm_observer_B.vNum_p);
  H->size[0] = left_arm_observer_B.b_i_m;
  H->size[1] = left_arm_observer_B.b_i_m;
  left_a_emxEnsureCapacity_real_T(H, left_arm_observer_B.f);
  left_arm_observer_B.n_j = left_arm_observer_B.b_i_m *
    left_arm_observer_B.b_i_m - 1;
  for (left_arm_observer_B.f = 0; left_arm_observer_B.f <=
       left_arm_observer_B.n_j; left_arm_observer_B.f++) {
    H->data[left_arm_observer_B.f] = 0.0;
  }

  left_arm_ob_emxInit_e_cell_wrap(&Ic, 2);
  left_arm_ob_emxInit_e_cell_wrap(&X, 2);
  left_arm_observer_B.c_tmp = static_cast<int32_T>(left_arm_observer_B.nb_k);
  left_arm_observer_B.c = left_arm_observer_B.c_tmp - 1;
  left_arm_observer_B.f = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = left_arm_observer_B.c_tmp;
  l_emxEnsureCapacity_e_cell_wrap(Ic, left_arm_observer_B.f);
  left_arm_observer_B.f = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_observer_B.c_tmp;
  l_emxEnsureCapacity_e_cell_wrap(X, left_arm_observer_B.f);
  for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m <=
       left_arm_observer_B.c; left_arm_observer_B.b_i_m++) {
    for (left_arm_observer_B.f = 0; left_arm_observer_B.f < 36;
         left_arm_observer_B.f++) {
      Ic->data[left_arm_observer_B.b_i_m].f1[left_arm_observer_B.f] =
        robot->Bodies[left_arm_observer_B.b_i_m]->
        SpatialInertia[left_arm_observer_B.f];
    }

    left_arm_observer_B.vNum_p = robot->PositionDoFMap[left_arm_observer_B.b_i_m];
    left_arm_observer_B.p_idx_1_a = robot->
      PositionDoFMap[left_arm_observer_B.b_i_m + 10];
    if (left_arm_observer_B.p_idx_1_a < left_arm_observer_B.vNum_p) {
      obj = robot->Bodies[left_arm_observer_B.b_i_m];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        left_arm_observer_B.T_g);
    } else {
      if (left_arm_observer_B.vNum_p > left_arm_observer_B.p_idx_1_a) {
        left_arm_observer_B.c_tmp = 0;
        left_arm_observer_B.f = -1;
      } else {
        left_arm_observer_B.c_tmp = static_cast<int32_T>
          (left_arm_observer_B.vNum_p) - 1;
        left_arm_observer_B.f = static_cast<int32_T>
          (left_arm_observer_B.p_idx_1_a) - 1;
      }

      obj = robot->Bodies[left_arm_observer_B.b_i_m];
      left_arm_observer_B.q_size_tmp_n = left_arm_observer_B.f -
        left_arm_observer_B.c_tmp;
      left_arm_observer_B.q_size_n = left_arm_observer_B.q_size_tmp_n + 1;
      for (left_arm_observer_B.f = 0; left_arm_observer_B.f <=
           left_arm_observer_B.q_size_tmp_n; left_arm_observer_B.f++) {
        left_arm_observer_B.q_data_o[left_arm_observer_B.f] =
          q[left_arm_observer_B.c_tmp + left_arm_observer_B.f];
      }

      rigidBodyJoint_transformBodyT_i(&obj->JointInternal,
        left_arm_observer_B.q_data_o, &left_arm_observer_B.q_size_n,
        left_arm_observer_B.T_g);
    }

    left_arm_observer_tforminv(left_arm_observer_B.T_g, left_arm_observer_B.dv1);
    left_arm_ob_tformToSpatialXform(left_arm_observer_B.dv1, X->
      data[left_arm_observer_B.b_i_m].f1);
  }

  left_arm_observer_B.c = static_cast<int32_T>(((-1.0 - left_arm_observer_B.nb_k)
    + 1.0) / -1.0) - 1;
  left_arm_observe_emxInit_real_T(&Si, 2);
  left_arm_observe_emxInit_real_T(&Fi, 2);
  left_arm_observe_emxInit_real_T(&Sj, 2);
  left_arm_observe_emxInit_real_T(&Hji, 2);
  left_arm_observe_emxInit_real_T(&a, 2);
  left_arm_observe_emxInit_real_T(&B, 2);
  for (left_arm_observer_B.c_tmp = 0; left_arm_observer_B.c_tmp <=
       left_arm_observer_B.c; left_arm_observer_B.c_tmp++) {
    left_arm_observer_B.pid_tmp_d = static_cast<int32_T>
      (left_arm_observer_B.nb_k + -static_cast<real_T>(left_arm_observer_B.c_tmp));
    left_arm_observer_B.q_size_tmp_n = left_arm_observer_B.pid_tmp_d - 1;
    left_arm_observer_B.pid_p = robot->Bodies[left_arm_observer_B.q_size_tmp_n
      ]->ParentIndex;
    left_arm_observer_B.vNum_p = robot->
      VelocityDoFMap[left_arm_observer_B.pid_tmp_d - 1];
    left_arm_observer_B.p_idx_1_a = robot->
      VelocityDoFMap[left_arm_observer_B.pid_tmp_d + 9];
    if (left_arm_observer_B.pid_p > 0.0) {
      for (left_arm_observer_B.f = 0; left_arm_observer_B.f < 6;
           left_arm_observer_B.f++) {
        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
             left_arm_observer_B.b_i_m++) {
          left_arm_observer_B.X_tmp_c = left_arm_observer_B.f + 6 *
            left_arm_observer_B.b_i_m;
          left_arm_observer_B.X_c[left_arm_observer_B.X_tmp_c] = 0.0;
          for (left_arm_observer_B.n_j = 0; left_arm_observer_B.n_j < 6;
               left_arm_observer_B.n_j++) {
            left_arm_observer_B.X_c[left_arm_observer_B.X_tmp_c] += X->
              data[left_arm_observer_B.q_size_tmp_n].f1[6 *
              left_arm_observer_B.f + left_arm_observer_B.n_j] * Ic->
              data[left_arm_observer_B.q_size_tmp_n].f1[6 *
              left_arm_observer_B.b_i_m + left_arm_observer_B.n_j];
          }
        }
      }

      for (left_arm_observer_B.f = 0; left_arm_observer_B.f < 6;
           left_arm_observer_B.f++) {
        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
             left_arm_observer_B.b_i_m++) {
          left_arm_observer_B.b_idx_0_j = 0.0;
          for (left_arm_observer_B.n_j = 0; left_arm_observer_B.n_j < 6;
               left_arm_observer_B.n_j++) {
            left_arm_observer_B.b_idx_0_j += left_arm_observer_B.X_c[6 *
              left_arm_observer_B.n_j + left_arm_observer_B.f] * X->
              data[left_arm_observer_B.q_size_tmp_n].f1[6 *
              left_arm_observer_B.b_i_m + left_arm_observer_B.n_j];
          }

          left_arm_observer_B.n_j = 6 * left_arm_observer_B.b_i_m +
            left_arm_observer_B.f;
          Ic->data[static_cast<int32_T>(left_arm_observer_B.pid_p) - 1]
            .f1[left_arm_observer_B.n_j] += left_arm_observer_B.b_idx_0_j;
        }
      }
    }

    left_arm_observer_B.b_idx_0_j = robot->
      VelocityDoFMap[left_arm_observer_B.pid_tmp_d - 1];
    left_arm_observer_B.b_idx_1_e = robot->
      VelocityDoFMap[left_arm_observer_B.pid_tmp_d + 9];
    if (left_arm_observer_B.b_idx_0_j <= left_arm_observer_B.b_idx_1_e) {
      obj = robot->Bodies[left_arm_observer_B.q_size_tmp_n];
      left_arm_observer_B.f = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(Si, left_arm_observer_B.f);
      left_arm_observer_B.n_j = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_observer_B.f = 0; left_arm_observer_B.f <=
           left_arm_observer_B.n_j; left_arm_observer_B.f++) {
        Si->data[left_arm_observer_B.f] = obj->
          JointInternal.MotionSubspace->data[left_arm_observer_B.f];
      }

      left_arm_observer_B.n_j = Si->size[1] - 1;
      left_arm_observer_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_observer_B.f);
      for (left_arm_observer_B.b_j_m = 0; left_arm_observer_B.b_j_m <=
           left_arm_observer_B.n_j; left_arm_observer_B.b_j_m++) {
        left_arm_observer_B.pid_tmp_d = left_arm_observer_B.b_j_m * 6 - 1;
        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
             left_arm_observer_B.b_i_m++) {
          Fi->data[(left_arm_observer_B.pid_tmp_d + left_arm_observer_B.b_i_m) +
            1] = 0.0;
        }

        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
             left_arm_observer_B.b_i_m++) {
          left_arm_observer_B.aoffset_e = left_arm_observer_B.b_i_m * 6 - 1;
          left_arm_observer_B.temp_p = Si->data[(left_arm_observer_B.pid_tmp_d +
            left_arm_observer_B.b_i_m) + 1];
          for (left_arm_observer_B.c_i_j = 0; left_arm_observer_B.c_i_j < 6;
               left_arm_observer_B.c_i_j++) {
            left_arm_observer_B.i_m = left_arm_observer_B.c_i_j + 1;
            left_arm_observer_B.f = left_arm_observer_B.pid_tmp_d +
              left_arm_observer_B.i_m;
            Fi->data[left_arm_observer_B.f] += Ic->
              data[left_arm_observer_B.q_size_tmp_n]
              .f1[left_arm_observer_B.aoffset_e + left_arm_observer_B.i_m] *
              left_arm_observer_B.temp_p;
          }
        }
      }

      if (left_arm_observer_B.vNum_p > left_arm_observer_B.p_idx_1_a) {
        left_arm_observer_B.pid_tmp_d = 0;
        left_arm_observer_B.X_tmp_c = 0;
      } else {
        left_arm_observer_B.pid_tmp_d = static_cast<int32_T>
          (left_arm_observer_B.vNum_p) - 1;
        left_arm_observer_B.X_tmp_c = left_arm_observer_B.pid_tmp_d;
      }

      left_arm_observer_B.f = a->size[0] * a->size[1];
      a->size[0] = Si->size[1];
      a->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a, left_arm_observer_B.f);
      for (left_arm_observer_B.f = 0; left_arm_observer_B.f < 6;
           left_arm_observer_B.f++) {
        left_arm_observer_B.n_j = Si->size[1];
        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m <
             left_arm_observer_B.n_j; left_arm_observer_B.b_i_m++) {
          a->data[left_arm_observer_B.b_i_m + a->size[0] * left_arm_observer_B.f]
            = Si->data[6 * left_arm_observer_B.b_i_m + left_arm_observer_B.f];
        }
      }

      left_arm_observer_B.m_f = a->size[0];
      left_arm_observer_B.n_j = Fi->size[1] - 1;
      left_arm_observer_B.f = Hji->size[0] * Hji->size[1];
      Hji->size[0] = a->size[0];
      Hji->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(Hji, left_arm_observer_B.f);
      for (left_arm_observer_B.b_j_m = 0; left_arm_observer_B.b_j_m <=
           left_arm_observer_B.n_j; left_arm_observer_B.b_j_m++) {
        left_arm_observer_B.coffset_a = left_arm_observer_B.b_j_m *
          left_arm_observer_B.m_f - 1;
        left_arm_observer_B.boffset_g = left_arm_observer_B.b_j_m * 6 - 1;
        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m <
             left_arm_observer_B.m_f; left_arm_observer_B.b_i_m++) {
          Hji->data[(left_arm_observer_B.coffset_a + left_arm_observer_B.b_i_m)
            + 1] = 0.0;
        }

        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
             left_arm_observer_B.b_i_m++) {
          left_arm_observer_B.aoffset_e = left_arm_observer_B.b_i_m *
            left_arm_observer_B.m_f - 1;
          left_arm_observer_B.temp_p = Fi->data[(left_arm_observer_B.boffset_g +
            left_arm_observer_B.b_i_m) + 1];
          for (left_arm_observer_B.c_i_j = 0; left_arm_observer_B.c_i_j <
               left_arm_observer_B.m_f; left_arm_observer_B.c_i_j++) {
            left_arm_observer_B.i_m = left_arm_observer_B.c_i_j + 1;
            left_arm_observer_B.f = left_arm_observer_B.coffset_a +
              left_arm_observer_B.i_m;
            Hji->data[left_arm_observer_B.f] += a->
              data[left_arm_observer_B.aoffset_e + left_arm_observer_B.i_m] *
              left_arm_observer_B.temp_p;
          }
        }
      }

      left_arm_observer_B.n_j = Hji->size[1];
      for (left_arm_observer_B.f = 0; left_arm_observer_B.f <
           left_arm_observer_B.n_j; left_arm_observer_B.f++) {
        left_arm_observer_B.b_j_m = Hji->size[0];
        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m <
             left_arm_observer_B.b_j_m; left_arm_observer_B.b_i_m++) {
          H->data[(left_arm_observer_B.pid_tmp_d + left_arm_observer_B.b_i_m) +
            H->size[0] * (left_arm_observer_B.X_tmp_c + left_arm_observer_B.f)] =
            Hji->data[Hji->size[0] * left_arm_observer_B.f +
            left_arm_observer_B.b_i_m];
        }
      }

      for (left_arm_observer_B.f = 0; left_arm_observer_B.f < 6;
           left_arm_observer_B.f++) {
        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
             left_arm_observer_B.b_i_m++) {
          left_arm_observer_B.X_c[left_arm_observer_B.b_i_m + 6 *
            left_arm_observer_B.f] = X->data[left_arm_observer_B.q_size_tmp_n].
            f1[6 * left_arm_observer_B.b_i_m + left_arm_observer_B.f];
        }
      }

      left_arm_observer_B.f = B->size[0] * B->size[1];
      B->size[0] = 6;
      B->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(B, left_arm_observer_B.f);
      left_arm_observer_B.n_j = Fi->size[0] * Fi->size[1] - 1;
      for (left_arm_observer_B.f = 0; left_arm_observer_B.f <=
           left_arm_observer_B.n_j; left_arm_observer_B.f++) {
        B->data[left_arm_observer_B.f] = Fi->data[left_arm_observer_B.f];
      }

      left_arm_observer_B.n_j = Fi->size[1];
      left_arm_observer_B.f = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = left_arm_observer_B.n_j;
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_observer_B.f);
      for (left_arm_observer_B.b_j_m = 0; left_arm_observer_B.b_j_m <
           left_arm_observer_B.n_j; left_arm_observer_B.b_j_m++) {
        left_arm_observer_B.pid_tmp_d = left_arm_observer_B.b_j_m * 6 - 1;
        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
             left_arm_observer_B.b_i_m++) {
          Fi->data[(left_arm_observer_B.pid_tmp_d + left_arm_observer_B.b_i_m) +
            1] = 0.0;
        }

        for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
             left_arm_observer_B.b_i_m++) {
          left_arm_observer_B.aoffset_e = left_arm_observer_B.b_i_m * 6 - 1;
          left_arm_observer_B.temp_p = B->data[(left_arm_observer_B.pid_tmp_d +
            left_arm_observer_B.b_i_m) + 1];
          for (left_arm_observer_B.c_i_j = 0; left_arm_observer_B.c_i_j < 6;
               left_arm_observer_B.c_i_j++) {
            left_arm_observer_B.i_m = left_arm_observer_B.c_i_j + 1;
            left_arm_observer_B.f = left_arm_observer_B.pid_tmp_d +
              left_arm_observer_B.i_m;
            Fi->data[left_arm_observer_B.f] +=
              left_arm_observer_B.X_c[left_arm_observer_B.aoffset_e +
              left_arm_observer_B.i_m] * left_arm_observer_B.temp_p;
          }
        }
      }

      while (left_arm_observer_B.pid_p > 0.0) {
        left_arm_observer_B.pid_tmp_d = static_cast<int32_T>
          (left_arm_observer_B.pid_p);
        left_arm_observer_B.q_size_tmp_n = left_arm_observer_B.pid_tmp_d - 1;
        obj = robot->Bodies[left_arm_observer_B.q_size_tmp_n];
        left_arm_observer_B.f = Sj->size[0] * Sj->size[1];
        Sj->size[0] = 6;
        Sj->size[1] = obj->JointInternal.MotionSubspace->size[1];
        left_a_emxEnsureCapacity_real_T(Sj, left_arm_observer_B.f);
        left_arm_observer_B.n_j = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (left_arm_observer_B.f = 0; left_arm_observer_B.f <=
             left_arm_observer_B.n_j; left_arm_observer_B.f++) {
          Sj->data[left_arm_observer_B.f] = obj->
            JointInternal.MotionSubspace->data[left_arm_observer_B.f];
        }

        left_arm_observer_B.b_idx_0_j = robot->
          VelocityDoFMap[left_arm_observer_B.pid_tmp_d - 1];
        left_arm_observer_B.b_idx_1_e = robot->
          VelocityDoFMap[left_arm_observer_B.pid_tmp_d + 9];
        if (left_arm_observer_B.b_idx_0_j <= left_arm_observer_B.b_idx_1_e) {
          left_arm_observer_B.f = a->size[0] * a->size[1];
          a->size[0] = Sj->size[1];
          a->size[1] = 6;
          left_a_emxEnsureCapacity_real_T(a, left_arm_observer_B.f);
          for (left_arm_observer_B.f = 0; left_arm_observer_B.f < 6;
               left_arm_observer_B.f++) {
            left_arm_observer_B.n_j = Sj->size[1];
            for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m <
                 left_arm_observer_B.n_j; left_arm_observer_B.b_i_m++) {
              a->data[left_arm_observer_B.b_i_m + a->size[0] *
                left_arm_observer_B.f] = Sj->data[6 * left_arm_observer_B.b_i_m
                + left_arm_observer_B.f];
            }
          }

          left_arm_observer_B.m_f = a->size[0];
          left_arm_observer_B.n_j = Fi->size[1] - 1;
          left_arm_observer_B.f = Hji->size[0] * Hji->size[1];
          Hji->size[0] = a->size[0];
          Hji->size[1] = Fi->size[1];
          left_a_emxEnsureCapacity_real_T(Hji, left_arm_observer_B.f);
          for (left_arm_observer_B.b_j_m = 0; left_arm_observer_B.b_j_m <=
               left_arm_observer_B.n_j; left_arm_observer_B.b_j_m++) {
            left_arm_observer_B.coffset_a = left_arm_observer_B.b_j_m *
              left_arm_observer_B.m_f - 1;
            left_arm_observer_B.boffset_g = left_arm_observer_B.b_j_m * 6 - 1;
            for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m <
                 left_arm_observer_B.m_f; left_arm_observer_B.b_i_m++) {
              Hji->data[(left_arm_observer_B.coffset_a +
                         left_arm_observer_B.b_i_m) + 1] = 0.0;
            }

            for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
                 left_arm_observer_B.b_i_m++) {
              left_arm_observer_B.aoffset_e = left_arm_observer_B.b_i_m *
                left_arm_observer_B.m_f - 1;
              left_arm_observer_B.temp_p = Fi->data
                [(left_arm_observer_B.boffset_g + left_arm_observer_B.b_i_m) + 1];
              for (left_arm_observer_B.c_i_j = 0; left_arm_observer_B.c_i_j <
                   left_arm_observer_B.m_f; left_arm_observer_B.c_i_j++) {
                left_arm_observer_B.i_m = left_arm_observer_B.c_i_j + 1;
                left_arm_observer_B.f = left_arm_observer_B.coffset_a +
                  left_arm_observer_B.i_m;
                Hji->data[left_arm_observer_B.f] += a->
                  data[left_arm_observer_B.aoffset_e + left_arm_observer_B.i_m] *
                  left_arm_observer_B.temp_p;
              }
            }
          }

          if (left_arm_observer_B.b_idx_0_j > left_arm_observer_B.b_idx_1_e) {
            left_arm_observer_B.pid_tmp_d = 0;
          } else {
            left_arm_observer_B.pid_tmp_d = static_cast<int32_T>
              (left_arm_observer_B.b_idx_0_j) - 1;
          }

          if (left_arm_observer_B.vNum_p > left_arm_observer_B.p_idx_1_a) {
            left_arm_observer_B.X_tmp_c = 0;
          } else {
            left_arm_observer_B.X_tmp_c = static_cast<int32_T>
              (left_arm_observer_B.vNum_p) - 1;
          }

          left_arm_observer_B.n_j = Hji->size[1];
          for (left_arm_observer_B.f = 0; left_arm_observer_B.f <
               left_arm_observer_B.n_j; left_arm_observer_B.f++) {
            left_arm_observer_B.b_j_m = Hji->size[0];
            for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m <
                 left_arm_observer_B.b_j_m; left_arm_observer_B.b_i_m++) {
              H->data[(left_arm_observer_B.pid_tmp_d + left_arm_observer_B.b_i_m)
                + H->size[0] * (left_arm_observer_B.X_tmp_c +
                                left_arm_observer_B.f)] = Hji->data[Hji->size[0]
                * left_arm_observer_B.f + left_arm_observer_B.b_i_m];
            }
          }

          if (left_arm_observer_B.vNum_p > left_arm_observer_B.p_idx_1_a) {
            left_arm_observer_B.pid_tmp_d = 0;
          } else {
            left_arm_observer_B.pid_tmp_d = static_cast<int32_T>
              (left_arm_observer_B.vNum_p) - 1;
          }

          if (left_arm_observer_B.b_idx_0_j > left_arm_observer_B.b_idx_1_e) {
            left_arm_observer_B.X_tmp_c = 0;
          } else {
            left_arm_observer_B.X_tmp_c = static_cast<int32_T>
              (left_arm_observer_B.b_idx_0_j) - 1;
          }

          left_arm_observer_B.n_j = Hji->size[0];
          for (left_arm_observer_B.f = 0; left_arm_observer_B.f <
               left_arm_observer_B.n_j; left_arm_observer_B.f++) {
            left_arm_observer_B.b_j_m = Hji->size[1];
            for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m <
                 left_arm_observer_B.b_j_m; left_arm_observer_B.b_i_m++) {
              H->data[(left_arm_observer_B.pid_tmp_d + left_arm_observer_B.b_i_m)
                + H->size[0] * (left_arm_observer_B.X_tmp_c +
                                left_arm_observer_B.f)] = Hji->data[Hji->size[0]
                * left_arm_observer_B.b_i_m + left_arm_observer_B.f];
            }
          }
        }

        for (left_arm_observer_B.f = 0; left_arm_observer_B.f < 6;
             left_arm_observer_B.f++) {
          for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
               left_arm_observer_B.b_i_m++) {
            left_arm_observer_B.X_c[left_arm_observer_B.b_i_m + 6 *
              left_arm_observer_B.f] = X->data[left_arm_observer_B.q_size_tmp_n]
              .f1[6 * left_arm_observer_B.b_i_m + left_arm_observer_B.f];
          }
        }

        left_arm_observer_B.f = B->size[0] * B->size[1];
        B->size[0] = 6;
        B->size[1] = Fi->size[1];
        left_a_emxEnsureCapacity_real_T(B, left_arm_observer_B.f);
        left_arm_observer_B.n_j = Fi->size[0] * Fi->size[1] - 1;
        for (left_arm_observer_B.f = 0; left_arm_observer_B.f <=
             left_arm_observer_B.n_j; left_arm_observer_B.f++) {
          B->data[left_arm_observer_B.f] = Fi->data[left_arm_observer_B.f];
        }

        left_arm_observer_B.n_j = Fi->size[1];
        left_arm_observer_B.f = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = left_arm_observer_B.n_j;
        left_a_emxEnsureCapacity_real_T(Fi, left_arm_observer_B.f);
        for (left_arm_observer_B.b_j_m = 0; left_arm_observer_B.b_j_m <
             left_arm_observer_B.n_j; left_arm_observer_B.b_j_m++) {
          left_arm_observer_B.pid_tmp_d = left_arm_observer_B.b_j_m * 6 - 1;
          for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
               left_arm_observer_B.b_i_m++) {
            Fi->data[(left_arm_observer_B.pid_tmp_d + left_arm_observer_B.b_i_m)
              + 1] = 0.0;
          }

          for (left_arm_observer_B.b_i_m = 0; left_arm_observer_B.b_i_m < 6;
               left_arm_observer_B.b_i_m++) {
            left_arm_observer_B.aoffset_e = left_arm_observer_B.b_i_m * 6 - 1;
            left_arm_observer_B.temp_p = B->data[(left_arm_observer_B.pid_tmp_d
              + left_arm_observer_B.b_i_m) + 1];
            for (left_arm_observer_B.c_i_j = 0; left_arm_observer_B.c_i_j < 6;
                 left_arm_observer_B.c_i_j++) {
              left_arm_observer_B.i_m = left_arm_observer_B.c_i_j + 1;
              left_arm_observer_B.f = left_arm_observer_B.pid_tmp_d +
                left_arm_observer_B.i_m;
              Fi->data[left_arm_observer_B.f] +=
                left_arm_observer_B.X_c[left_arm_observer_B.aoffset_e +
                left_arm_observer_B.i_m] * left_arm_observer_B.temp_p;
            }
          }
        }

        left_arm_observer_B.pid_p = robot->
          Bodies[left_arm_observer_B.q_size_tmp_n]->ParentIndex;
      }
    }
  }

  left_arm_observe_emxFree_real_T(&B);
  left_arm_observe_emxFree_real_T(&a);
  left_arm_observe_emxFree_real_T(&Hji);
  left_arm_observe_emxFree_real_T(&Sj);
  left_arm_observe_emxFree_real_T(&Fi);
  left_arm_observe_emxFree_real_T(&Si);
  left_arm_ob_emxFree_e_cell_wrap(&X);
  left_arm_ob_emxFree_e_cell_wrap(&Ic);
}

static c_robotics_core_internal_code_T *NameValueParser_NameValueParser
  (c_robotics_core_internal_code_T *obj)
{
  return obj;
}

static real_T left_arm_observer_rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    left_arm_observer_B.d = fabs(u0);
    left_arm_observer_B.d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (left_arm_observer_B.d == 1.0) {
        y = 1.0;
      } else if (left_arm_observer_B.d > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (left_arm_observer_B.d1 == 0.0) {
      y = 1.0;
    } else if (left_arm_observer_B.d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

static void left_arm__generateQuinticCoeffs(const real_T posPts[2], const real_T
  velPts[2], const real_T accPts[2], real_T finalTime, real_T coeffVec[6])
{
  coeffVec[0] = posPts[0];
  coeffVec[1] = velPts[0];
  coeffVec[2] = accPts[0] / 2.0;
  coeffVec[3] = 0.0;
  coeffVec[4] = 0.0;
  coeffVec[5] = 0.0;
  left_arm_observer_B.xtmp = left_arm_observer_rt_powd_snf(finalTime, 3.0);
  left_arm_observer_B.b_coeffVec_tmp = left_arm_observer_rt_powd_snf(finalTime,
    4.0);
  left_arm_observer_B.dv8[0] = 1.0;
  left_arm_observer_B.dv8[3] = finalTime;
  left_arm_observer_B.d2 = finalTime * finalTime;
  left_arm_observer_B.dv8[6] = left_arm_observer_B.d2;
  left_arm_observer_B.dv8[1] = 0.0;
  left_arm_observer_B.dv8[4] = 1.0;
  left_arm_observer_B.dv8[7] = 2.0 * finalTime;
  left_arm_observer_B.dv8[2] = 0.0;
  left_arm_observer_B.dv8[5] = 0.0;
  left_arm_observer_B.dv8[8] = 2.0;
  left_arm_observer_B.posPts[0] = posPts[1];
  left_arm_observer_B.posPts[1] = velPts[1];
  left_arm_observer_B.posPts[2] = accPts[1];
  left_arm_observer_B.dv9[0] = 10.0 / left_arm_observer_B.xtmp;
  left_arm_observer_B.dv9[3] = -4.0 / left_arm_observer_B.d2;
  left_arm_observer_B.dv9[6] = 1.0 / (2.0 * finalTime);
  left_arm_observer_B.dv9[1] = -15.0 / left_arm_observer_B.b_coeffVec_tmp;
  left_arm_observer_B.dv9[4] = 7.0 / left_arm_observer_B.xtmp;
  left_arm_observer_B.dv9[7] = -1.0 / left_arm_observer_B.d2;
  left_arm_observer_B.dv9[2] = 6.0 / left_arm_observer_rt_powd_snf(finalTime,
    5.0);
  left_arm_observer_B.dv9[5] = -3.0 / left_arm_observer_B.b_coeffVec_tmp;
  left_arm_observer_B.dv9[8] = 1.0 / (2.0 * left_arm_observer_B.xtmp);
  for (left_arm_observer_B.i6 = 0; left_arm_observer_B.i6 < 3;
       left_arm_observer_B.i6++) {
    left_arm_observer_B.posPts_a[left_arm_observer_B.i6] =
      left_arm_observer_B.posPts[left_arm_observer_B.i6] -
      (left_arm_observer_B.dv8[left_arm_observer_B.i6 + 6] * coeffVec[2] +
       (left_arm_observer_B.dv8[left_arm_observer_B.i6 + 3] * coeffVec[1] +
        left_arm_observer_B.dv8[left_arm_observer_B.i6] * coeffVec[0]));
  }

  for (left_arm_observer_B.i6 = 0; left_arm_observer_B.i6 < 3;
       left_arm_observer_B.i6++) {
    coeffVec[left_arm_observer_B.i6 + 3] =
      left_arm_observer_B.dv9[left_arm_observer_B.i6 + 6] *
      left_arm_observer_B.posPts_a[2] +
      (left_arm_observer_B.dv9[left_arm_observer_B.i6 + 3] *
       left_arm_observer_B.posPts_a[1] +
       left_arm_observer_B.dv9[left_arm_observer_B.i6] *
       left_arm_observer_B.posPts_a[0]);
  }

  for (left_arm_observer_B.i6 = 0; left_arm_observer_B.i6 < 6;
       left_arm_observer_B.i6++) {
  }

  left_arm_observer_B.xtmp = coeffVec[0];
  coeffVec[0] = coeffVec[5];
  coeffVec[5] = left_arm_observer_B.xtmp;
  left_arm_observer_B.xtmp = coeffVec[1];
  coeffVec[1] = coeffVec[4];
  coeffVec[4] = left_arm_observer_B.xtmp;
  left_arm_observer_B.xtmp = coeffVec[2];
  coeffVec[2] = coeffVec[3];
  coeffVec[3] = left_arm_observer_B.xtmp;
}

static void le_addFlatSegmentsToPPFormParts(const real_T oldbreaks[2], const
  real_T oldCoeffs[42], real_T newBreaks[4], real_T newCoefs[126])
{
  static const int8_T tmp[6] = { 0, 0, 0, 0, 0, 1 };

  memset(&left_arm_observer_B.newSegmentCoeffs[0], 0, 42U * sizeof(real_T));
  for (left_arm_observer_B.i2 = 0; left_arm_observer_B.i2 < 7;
       left_arm_observer_B.i2++) {
    left_arm_observer_B.newSegmentCoeffs[left_arm_observer_B.i2 + 35] = 0.0;
    for (left_arm_observer_B.b_i_d = 0; left_arm_observer_B.b_i_d < 6;
         left_arm_observer_B.b_i_d++) {
      left_arm_observer_B.newSegmentCoeffs[left_arm_observer_B.i2 + 35] +=
        oldCoeffs[7 * left_arm_observer_B.b_i_d + left_arm_observer_B.i2] *
        static_cast<real_T>(tmp[left_arm_observer_B.b_i_d]);
    }
  }

  memset(&left_arm_observer_B.coefsWithFlatStart[0], 0, 84U * sizeof(real_T));
  left_arm_observer_B.holdPoint = oldbreaks[1] - oldbreaks[0];
  for (left_arm_observer_B.b_i_d = 0; left_arm_observer_B.b_i_d < 6;
       left_arm_observer_B.b_i_d++) {
    for (left_arm_observer_B.i2 = 0; left_arm_observer_B.i2 < 7;
         left_arm_observer_B.i2++) {
      left_arm_observer_B.coefsWithFlatStart_tmp = 7 * left_arm_observer_B.b_i_d
        + left_arm_observer_B.i2;
      left_arm_observer_B.coefsWithFlatStart[((left_arm_observer_B.i2 + 1) + 14 *
        left_arm_observer_B.b_i_d) - 1] =
        left_arm_observer_B.newSegmentCoeffs[left_arm_observer_B.coefsWithFlatStart_tmp];
      left_arm_observer_B.coefsWithFlatStart[((left_arm_observer_B.i2 + 8) + 14 *
        left_arm_observer_B.b_i_d) - 1] =
        oldCoeffs[left_arm_observer_B.coefsWithFlatStart_tmp];
    }

    left_arm_observer_B.evalPointVector[left_arm_observer_B.b_i_d] =
      left_arm_observer_rt_powd_snf(left_arm_observer_B.holdPoint, 6.0 - (
      static_cast<real_T>(left_arm_observer_B.b_i_d) + 1.0));
  }

  memset(&left_arm_observer_B.newSegmentCoeffs[0], 0, 42U * sizeof(real_T));
  for (left_arm_observer_B.i2 = 0; left_arm_observer_B.i2 < 7;
       left_arm_observer_B.i2++) {
    left_arm_observer_B.newSegmentCoeffs[left_arm_observer_B.i2 + 35] = 0.0;
    for (left_arm_observer_B.b_i_d = 0; left_arm_observer_B.b_i_d < 6;
         left_arm_observer_B.b_i_d++) {
      left_arm_observer_B.newSegmentCoeffs[left_arm_observer_B.i2 + 35] +=
        left_arm_observer_B.coefsWithFlatStart[(14 * left_arm_observer_B.b_i_d +
        left_arm_observer_B.i2) + 7] *
        left_arm_observer_B.evalPointVector[left_arm_observer_B.b_i_d];
    }
  }

  memset(&newCoefs[0], 0, 126U * sizeof(real_T));
  for (left_arm_observer_B.i2 = 0; left_arm_observer_B.i2 < 6;
       left_arm_observer_B.i2++) {
    for (left_arm_observer_B.b_i_d = 0; left_arm_observer_B.b_i_d < 14;
         left_arm_observer_B.b_i_d++) {
      newCoefs[((left_arm_observer_B.b_i_d + 1) + 21 * left_arm_observer_B.i2) -
        1] = left_arm_observer_B.coefsWithFlatStart[14 * left_arm_observer_B.i2
        + left_arm_observer_B.b_i_d];
    }

    for (left_arm_observer_B.b_i_d = 0; left_arm_observer_B.b_i_d < 7;
         left_arm_observer_B.b_i_d++) {
      newCoefs[((left_arm_observer_B.b_i_d + 15) + 21 * left_arm_observer_B.i2)
        - 1] = left_arm_observer_B.newSegmentCoeffs[7 * left_arm_observer_B.i2 +
        left_arm_observer_B.b_i_d];
    }
  }

  newBreaks[0] = oldbreaks[0] - 1.0;
  newBreaks[1] = oldbreaks[0];
  newBreaks[2] = oldbreaks[1];
  newBreaks[3] = oldbreaks[1] + 1.0;
}

static void left_arm_observer_ppval(const real_T pp_breaks[4], const real_T
  pp_coefs[126], const real_T x[2], real_T v[14])
{
  int32_T iv0;
  int32_T j;
  real_T xloc;
  int32_T b_ix;
  int32_T low_i;
  int32_T low_ip1;
  int32_T high_i;
  int32_T mid_i;
  int32_T v_tmp;
  for (b_ix = 0; b_ix < 2; b_ix++) {
    iv0 = b_ix * 7 - 1;
    if (rtIsNaN(x[b_ix])) {
      for (low_ip1 = 0; low_ip1 < 7; low_ip1++) {
        v[(iv0 + low_ip1) + 1] = x[b_ix];
      }
    } else {
      low_i = 0;
      low_ip1 = 1;
      high_i = 4;
      while (high_i > low_ip1 + 1) {
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (x[b_ix] >= pp_breaks[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i;
        } else {
          high_i = mid_i;
        }
      }

      low_ip1 = low_i * 7;
      xloc = x[b_ix] - pp_breaks[low_i];
      for (low_i = 0; low_i < 7; low_i++) {
        v[(iv0 + low_i) + 1] = pp_coefs[low_ip1 + low_i];
      }

      for (low_i = 0; low_i < 5; low_i++) {
        high_i = ((low_i + 1) * 21 + low_ip1) - 1;
        for (mid_i = 0; mid_i < 7; mid_i++) {
          j = mid_i + 1;
          v_tmp = iv0 + j;
          v[v_tmp] = v[v_tmp] * xloc + pp_coefs[high_i + j];
        }
      }
    }
  }
}

static void left_arm_obs_changeEndSegBreaks(const real_T oldBreaks[4], const
  real_T evalTime[2], real_T newBreaks[4])
{
  real_T dt;
  int32_T tGreaterThanTfIdx_data;
  boolean_T x[2];
  int32_T ii_data;
  int32_T idx;
  int32_T b_ii;
  int32_T ii_size_idx_1;
  real_T u0;
  boolean_T exitg1;
  newBreaks[0] = oldBreaks[0];
  newBreaks[1] = oldBreaks[1];
  newBreaks[3] = oldBreaks[3];
  dt = 0.01;
  x[0] = (evalTime[0] > oldBreaks[2]);
  x[1] = (evalTime[1] > oldBreaks[2]);
  idx = 0;
  ii_size_idx_1 = 1;
  b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (b_ii - 1 < 2)) {
    if (x[b_ii - 1]) {
      idx = 1;
      ii_data = b_ii;
      exitg1 = true;
    } else {
      b_ii++;
    }
  }

  if (idx == 0) {
    ii_size_idx_1 = 0;
  }

  if (0 <= ii_size_idx_1 - 1) {
    tGreaterThanTfIdx_data = ii_data;
  }

  if (ii_size_idx_1 != 0) {
    u0 = (evalTime[tGreaterThanTfIdx_data - 1] - oldBreaks[2]) / 2.0;
    if (u0 < 0.01) {
      dt = u0;
    }
  }

  newBreaks[2] = oldBreaks[2] + dt;
}

static void left_arm_o_polyCoeffsDerivative(const real_T coeffs[126], real_T
  dCoeffs[126])
{
  int32_T b_i;
  int32_T tmp;
  int32_T b_i_0;
  int32_T i;
  memset(&dCoeffs[0], 0, 126U * sizeof(real_T));
  for (b_i = 0; b_i < 5; b_i++) {
    tmp = 5 - b_i;
    b_i_0 = b_i + 1;
    for (i = 0; i < 21; i++) {
      dCoeffs[i + 21 * (b_i + 1)] = coeffs[(b_i_0 - 1) * 21 + i] *
        static_cast<real_T>(tmp);
    }
  }
}

static void left_arm_observ_quinticpolytraj(const real_T wayPoints[14], const
  real_T timePoints[2], const real_T t[2], const real_T varargin_2[14], const
  real_T varargin_4[14], real_T q[14], real_T qd[14], real_T qdd[14], real_T
  pp_breaks[4], real_T pp_coefs[126])
{
  real_T *velBC;
  real_T *accBC;
  NameValueParser_NameValueParser(&left_arm_observer_B.parser);
  memcpy(&left_arm_observer_B.parsedResults[0].f1[0], &varargin_2[0], 14U *
         sizeof(real_T));
  memcpy(&left_arm_observer_B.parsedResults[1].f1[0], &varargin_4[0], 14U *
         sizeof(real_T));
  memcpy(&left_arm_observer_B.parser.ParsedResults[0],
         &left_arm_observer_B.parsedResults[0], sizeof
         (e_cell_wrap_left_arm_observer_T) << 1U);
  velBC = &left_arm_observer_B.parser.ParsedResults[0].f1[0];
  accBC = &left_arm_observer_B.parser.ParsedResults[1].f1[0];
  memset(&left_arm_observer_B.coefMat[0], 0, 42U * sizeof(real_T));
  left_arm_observer_B.finalTime = timePoints[1] - timePoints[0];
  for (left_arm_observer_B.b_j = 0; left_arm_observer_B.b_j < 7;
       left_arm_observer_B.b_j++) {
    left_arm_observer_B.wayPoints[0] = wayPoints[left_arm_observer_B.b_j];
    left_arm_observer_B.velBC[0] = velBC[left_arm_observer_B.b_j];
    left_arm_observer_B.accBC[0] = accBC[left_arm_observer_B.b_j];
    left_arm_observer_B.wayPoints[1] = wayPoints[left_arm_observer_B.b_j + 7];
    left_arm_observer_B.velBC[1] = velBC[left_arm_observer_B.b_j + 7];
    left_arm_observer_B.accBC[1] = accBC[left_arm_observer_B.b_j + 7];
    left_arm__generateQuinticCoeffs(left_arm_observer_B.wayPoints,
      left_arm_observer_B.velBC, left_arm_observer_B.accBC,
      left_arm_observer_B.finalTime, left_arm_observer_B.dv14);
    for (left_arm_observer_B.i1 = 0; left_arm_observer_B.i1 < 6;
         left_arm_observer_B.i1++) {
      left_arm_observer_B.coefMat[left_arm_observer_B.b_j + 7 *
        left_arm_observer_B.i1] =
        left_arm_observer_B.dv14[left_arm_observer_B.i1];
    }
  }

  le_addFlatSegmentsToPPFormParts(timePoints, left_arm_observer_B.coefMat,
    left_arm_observer_B.modBreaks, left_arm_observer_B.modCoeffs);
  pp_breaks[0] = left_arm_observer_B.modBreaks[0];
  pp_breaks[1] = left_arm_observer_B.modBreaks[1];
  pp_breaks[2] = left_arm_observer_B.modBreaks[2];
  pp_breaks[3] = left_arm_observer_B.modBreaks[3];
  memcpy(&pp_coefs[0], &left_arm_observer_B.modCoeffs[0], 126U * sizeof(real_T));
  left_arm_observer_ppval(left_arm_observer_B.modBreaks,
    left_arm_observer_B.modCoeffs, t, q);
  left_arm_obs_changeEndSegBreaks(left_arm_observer_B.modBreaks, t,
    left_arm_observer_B.derivativeBreaks);
  left_arm_o_polyCoeffsDerivative(left_arm_observer_B.modCoeffs,
    left_arm_observer_B.dCoeffs);
  left_arm_observer_ppval(left_arm_observer_B.derivativeBreaks,
    left_arm_observer_B.dCoeffs, t, qd);
  left_arm_o_polyCoeffsDerivative(left_arm_observer_B.dCoeffs,
    left_arm_observer_B.modCoeffs);
  left_arm_observer_ppval(left_arm_observer_B.derivativeBreaks,
    left_arm_observer_B.modCoeffs, t, qdd);
}

static void PolyTrajSys_computePPDerivative(const real_T pp_breaks[4], const
  real_T pp_coefs[126], real_T t, real_T ppd_breaks[4], real_T ppd_coefs[126],
  real_T ppdd_breaks[4], real_T ppdd_coefs[126])
{
  real_T dt;
  int32_T b_i;
  int32_T i;
  int32_T ii_size_idx_0;
  int32_T ii_size_idx_1;
  real_T u0;
  ppdd_breaks[0] = pp_breaks[0];
  ppdd_breaks[1] = pp_breaks[1];
  ppdd_breaks[3] = pp_breaks[3];
  dt = 0.01;
  if (t > pp_breaks[2]) {
    ii_size_idx_0 = 1;
    ii_size_idx_1 = 1;
  } else {
    ii_size_idx_0 = 0;
    ii_size_idx_1 = 0;
  }

  if ((ii_size_idx_0 != 0) && (ii_size_idx_1 != 0)) {
    u0 = (t - pp_breaks[2]) / 2.0;
    if (u0 < 0.01) {
      dt = u0;
    }
  }

  ppdd_breaks[2] = pp_breaks[2] + dt;
  memset(&ppd_coefs[0], 0, 126U * sizeof(real_T));
  for (ii_size_idx_0 = 0; ii_size_idx_0 < 5; ii_size_idx_0++) {
    ii_size_idx_1 = 5 - ii_size_idx_0;
    b_i = ii_size_idx_0 + 1;
    for (i = 0; i < 21; i++) {
      ppd_coefs[i + 21 * (ii_size_idx_0 + 1)] = pp_coefs[(b_i - 1) * 21 + i] *
        static_cast<real_T>(ii_size_idx_1);
    }
  }

  memset(&ppdd_coefs[0], 0, 126U * sizeof(real_T));
  for (ii_size_idx_0 = 0; ii_size_idx_0 < 5; ii_size_idx_0++) {
    ii_size_idx_1 = 5 - ii_size_idx_0;
    b_i = ii_size_idx_0 + 1;
    for (i = 0; i < 21; i++) {
      ppdd_coefs[i + 21 * (ii_size_idx_0 + 1)] = ppd_coefs[(b_i - 1) * 21 + i] *
        static_cast<real_T>(ii_size_idx_1);
    }
  }

  ppd_breaks[0] = pp_breaks[0];
  ppd_breaks[1] = pp_breaks[1];
  ppd_breaks[2] = ppdd_breaks[2];
  ppd_breaks[3] = pp_breaks[3];
}

static void left_arm_observer_ppval_i(const real_T pp_breaks[4], const real_T
  pp_coefs[126], real_T x, real_T v[7])
{
  real_T xloc;
  int32_T low_i;
  int32_T low_ip1;
  int32_T high_i;
  int32_T mid_i;
  if (rtIsNaN(x)) {
    for (low_ip1 = 0; low_ip1 < 7; low_ip1++) {
      v[low_ip1] = x;
    }
  } else {
    low_i = 0;
    low_ip1 = 1;
    high_i = 4;
    while (high_i > low_ip1 + 1) {
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (x >= pp_breaks[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i;
      } else {
        high_i = mid_i;
      }
    }

    low_ip1 = low_i * 7;
    xloc = x - pp_breaks[low_i];
    for (low_i = 0; low_i < 7; low_i++) {
      v[low_i] = pp_coefs[low_ip1 + low_i];
    }

    for (low_i = 0; low_i < 5; low_i++) {
      high_i = ((low_i + 1) * 21 + low_ip1) - 1;
      for (mid_i = 0; mid_i < 7; mid_i++) {
        v[mid_i] = pp_coefs[(high_i + mid_i) + 1] + xloc * v[mid_i];
      }
    }
  }
}

static void left_arm_o_PolyTrajSys_stepImpl(const
  robotics_slcore_internal_bloc_T *obj, real_T time, real_T q[7], real_T qd[7],
  real_T qdd[7])
{
  left_arm_observ_quinticpolytraj(obj->Waypoints, obj->TimePoints,
    obj->TimePoints, obj->VelocityBoundaryCondition,
    obj->AccelerationBoundaryCondition, left_arm_observer_B.unusedU4,
    left_arm_observer_B.unusedU5, left_arm_observer_B.unusedU6,
    left_arm_observer_B.pp_breaks, left_arm_observer_B.pp_coefs);
  PolyTrajSys_computePPDerivative(left_arm_observer_B.pp_breaks,
    left_arm_observer_B.pp_coefs, time, left_arm_observer_B.ppd_breaks,
    left_arm_observer_B.ppd_coefs, left_arm_observer_B.ppdd_breaks,
    left_arm_observer_B.ppdd_coefs);
  left_arm_observer_ppval_i(left_arm_observer_B.pp_breaks,
    left_arm_observer_B.pp_coefs, time, q);
  left_arm_observer_ppval_i(left_arm_observer_B.ppd_breaks,
    left_arm_observer_B.ppd_coefs, time, qd);
  left_arm_observer_ppval_i(left_arm_observer_B.ppdd_breaks,
    left_arm_observer_B.ppdd_coefs, time, qdd);
}

static void left_arm_observer_eye(real_T b_I[36])
{
  int32_T b_k;
  memset(&b_I[0], 0, 36U * sizeof(real_T));
  for (b_k = 0; b_k < 6; b_k++) {
    b_I[b_k + 6 * b_k] = 1.0;
  }
}

static void left_arm_observe_emxInit_char_T(emxArray_char_T_left_arm_obse_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_left_arm_obse_T *emxArray;
  *pEmxArray = (emxArray_char_T_left_arm_obse_T *)malloc(sizeof
    (emxArray_char_T_left_arm_obse_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (left_arm_observer_B.i_m1 = 0; left_arm_observer_B.i_m1 < numDimensions;
       left_arm_observer_B.i_m1++) {
    emxArray->size[left_arm_observer_B.i_m1] = 0;
  }
}

static void left_a_emxEnsureCapacity_char_T(emxArray_char_T_left_arm_obse_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_observer_B.newNumel_p = 1;
  for (left_arm_observer_B.i_p = 0; left_arm_observer_B.i_p <
       emxArray->numDimensions; left_arm_observer_B.i_p++) {
    left_arm_observer_B.newNumel_p *= emxArray->size[left_arm_observer_B.i_p];
  }

  if (left_arm_observer_B.newNumel_p > emxArray->allocatedSize) {
    left_arm_observer_B.i_p = emxArray->allocatedSize;
    if (left_arm_observer_B.i_p < 16) {
      left_arm_observer_B.i_p = 16;
    }

    while (left_arm_observer_B.i_p < left_arm_observer_B.newNumel_p) {
      if (left_arm_observer_B.i_p > 1073741823) {
        left_arm_observer_B.i_p = MAX_int32_T;
      } else {
        left_arm_observer_B.i_p <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_observer_B.i_p), sizeof
                     (char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = left_arm_observer_B.i_p;
    emxArray->canFreeData = true;
  }
}

static void left_arm_observe_emxFree_char_T(emxArray_char_T_left_arm_obse_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_left_arm_obse_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_left_arm_obse_T *)NULL;
  }
}

static boolean_T left_arm_observer_strcmp(const emxArray_char_T_left_arm_obse_T *
  a)
{
  boolean_T b_bool;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  for (left_arm_observer_B.b_kstr_c = 0; left_arm_observer_B.b_kstr_c < 5;
       left_arm_observer_B.b_kstr_c++) {
    left_arm_observer_B.b_ei[left_arm_observer_B.b_kstr_c] =
      tmp[left_arm_observer_B.b_kstr_c];
  }

  b_bool = false;
  if (a->size[1] == 5) {
    left_arm_observer_B.b_kstr_c = 1;
    do {
      exitg1 = 0;
      if (left_arm_observer_B.b_kstr_c - 1 < 5) {
        left_arm_observer_B.kstr_g = left_arm_observer_B.b_kstr_c - 1;
        if (a->data[left_arm_observer_B.kstr_g] !=
            left_arm_observer_B.b_ei[left_arm_observer_B.kstr_g]) {
          exitg1 = 1;
        } else {
          left_arm_observer_B.b_kstr_c++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

static void lef_GravityTorqueBlock_stepImpl(robotics_slmanip_internal_b_i_T *obj,
  const real_T q[7], real_T jointTorq[7])
{
  k_robotics_manip_internal_R_i_T *robot;
  emxArray_e_cell_wrap_left_arm_T *X;
  emxArray_e_cell_wrap_left_arm_T *Xtree;
  emxArray_real_T_left_arm_obse_T *vJ;
  emxArray_real_T_left_arm_obse_T *vB;
  emxArray_real_T_left_arm_obse_T *aB;
  emxArray_real_T_left_arm_obse_T *f;
  emxArray_real_T_left_arm_obse_T *S;
  emxArray_real_T_left_arm_obse_T *taui;
  j_robotics_manip_internal_Rig_T *obj_0;
  emxArray_real_T_left_arm_obse_T *a;
  emxArray_char_T_left_arm_obse_T *switch_expression;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int32_T exitg1;
  robot = &obj->TreeInternal;
  left_arm_observer_B.a0_n[0] = 0.0;
  left_arm_observer_B.a0_n[1] = 0.0;
  left_arm_observer_B.a0_n[2] = 0.0;
  left_arm_observer_B.a0_n[3] = -obj->TreeInternal.Gravity[0];
  left_arm_observer_B.a0_n[4] = -obj->TreeInternal.Gravity[1];
  left_arm_observer_B.a0_n[5] = -obj->TreeInternal.Gravity[2];
  left_arm_observe_emxInit_real_T(&vJ, 2);
  left_arm_observer_B.nb_o = obj->TreeInternal.NumBodies;
  left_arm_observer_B.u = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  left_arm_observer_B.unnamed_idx_1_k = static_cast<int32_T>
    (left_arm_observer_B.nb_o);
  vJ->size[1] = left_arm_observer_B.unnamed_idx_1_k;
  left_a_emxEnsureCapacity_real_T(vJ, left_arm_observer_B.u);
  left_arm_observer_B.aoffset_f = 6 * left_arm_observer_B.unnamed_idx_1_k - 1;
  for (left_arm_observer_B.u = 0; left_arm_observer_B.u <=
       left_arm_observer_B.aoffset_f; left_arm_observer_B.u++) {
    vJ->data[left_arm_observer_B.u] = 0.0;
  }

  left_arm_observe_emxInit_real_T(&vB, 2);
  left_arm_observer_B.u = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = left_arm_observer_B.unnamed_idx_1_k;
  left_a_emxEnsureCapacity_real_T(vB, left_arm_observer_B.u);
  for (left_arm_observer_B.u = 0; left_arm_observer_B.u <=
       left_arm_observer_B.aoffset_f; left_arm_observer_B.u++) {
    vB->data[left_arm_observer_B.u] = 0.0;
  }

  left_arm_observe_emxInit_real_T(&aB, 2);
  left_arm_observer_B.u = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = left_arm_observer_B.unnamed_idx_1_k;
  left_a_emxEnsureCapacity_real_T(aB, left_arm_observer_B.u);
  for (left_arm_observer_B.u = 0; left_arm_observer_B.u <=
       left_arm_observer_B.aoffset_f; left_arm_observer_B.u++) {
    aB->data[left_arm_observer_B.u] = 0.0;
  }

  for (left_arm_observer_B.i_o = 0; left_arm_observer_B.i_o < 7;
       left_arm_observer_B.i_o++) {
    jointTorq[left_arm_observer_B.i_o] = 0.0;
  }

  left_arm_ob_emxInit_e_cell_wrap(&X, 2);
  left_arm_ob_emxInit_e_cell_wrap(&Xtree, 2);
  left_arm_observer_B.i_o = left_arm_observer_B.unnamed_idx_1_k - 1;
  left_arm_observer_B.u = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = left_arm_observer_B.unnamed_idx_1_k;
  l_emxEnsureCapacity_e_cell_wrap(Xtree, left_arm_observer_B.u);
  left_arm_observer_B.u = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_observer_B.unnamed_idx_1_k;
  l_emxEnsureCapacity_e_cell_wrap(X, left_arm_observer_B.u);
  if (0 <= left_arm_observer_B.i_o) {
    left_arm_observer_eye(left_arm_observer_B.b_I_m);
  }

  for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o <=
       left_arm_observer_B.i_o; left_arm_observer_B.b_k_o++) {
    for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 36;
         left_arm_observer_B.u++) {
      Xtree->data[left_arm_observer_B.b_k_o].f1[left_arm_observer_B.u] =
        left_arm_observer_B.b_I_m[left_arm_observer_B.u];
      X->data[left_arm_observer_B.b_k_o].f1[left_arm_observer_B.u] =
        left_arm_observer_B.b_I_m[left_arm_observer_B.u];
    }
  }

  left_arm_observe_emxInit_real_T(&f, 2);
  left_arm_observer_B.u = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = left_arm_observer_B.unnamed_idx_1_k;
  left_a_emxEnsureCapacity_real_T(f, left_arm_observer_B.u);
  left_arm_observe_emxInit_real_T(&S, 2);
  left_arm_observe_emxInit_char_T(&switch_expression, 2);
  for (left_arm_observer_B.unnamed_idx_1_k = 0;
       left_arm_observer_B.unnamed_idx_1_k <= left_arm_observer_B.i_o;
       left_arm_observer_B.unnamed_idx_1_k++) {
    obj_0 = robot->Bodies[left_arm_observer_B.unnamed_idx_1_k];
    left_arm_observer_B.u = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj_0->JointInternal.MotionSubspace->size[1];
    left_a_emxEnsureCapacity_real_T(S, left_arm_observer_B.u);
    left_arm_observer_B.aoffset_f = obj_0->JointInternal.MotionSubspace->size[0]
      * obj_0->JointInternal.MotionSubspace->size[1] - 1;
    for (left_arm_observer_B.u = 0; left_arm_observer_B.u <=
         left_arm_observer_B.aoffset_f; left_arm_observer_B.u++) {
      S->data[left_arm_observer_B.u] = obj_0->JointInternal.MotionSubspace->
        data[left_arm_observer_B.u];
    }

    left_arm_observer_B.a_idx_0_m = robot->
      PositionDoFMap[left_arm_observer_B.unnamed_idx_1_k];
    left_arm_observer_B.a_idx_1_m = robot->
      PositionDoFMap[left_arm_observer_B.unnamed_idx_1_k + 10];
    left_arm_observer_B.b_idx_0_c = robot->
      VelocityDoFMap[left_arm_observer_B.unnamed_idx_1_k];
    left_arm_observer_B.b_idx_1_f = robot->
      VelocityDoFMap[left_arm_observer_B.unnamed_idx_1_k + 10];
    if (left_arm_observer_B.a_idx_1_m < left_arm_observer_B.a_idx_0_m) {
      obj_0 = robot->Bodies[left_arm_observer_B.unnamed_idx_1_k];
      rigidBodyJoint_transformBodyToP(&obj_0->JointInternal,
        left_arm_observer_B.T_k);
      left_arm_observer_B.t_l = 1;
      left_arm_observer_B.qddoti_data[0] = 0.0;
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        vJ->data[left_arm_observer_B.u + 6 * left_arm_observer_B.unnamed_idx_1_k]
          = 0.0;
      }
    } else {
      if (left_arm_observer_B.a_idx_0_m > left_arm_observer_B.a_idx_1_m) {
        left_arm_observer_B.b_k_o = 0;
        left_arm_observer_B.j_n = 0;
      } else {
        left_arm_observer_B.b_k_o = static_cast<int32_T>
          (left_arm_observer_B.a_idx_0_m) - 1;
        left_arm_observer_B.j_n = static_cast<int32_T>
          (left_arm_observer_B.a_idx_1_m);
      }

      if (left_arm_observer_B.b_idx_0_c > left_arm_observer_B.b_idx_1_f) {
        left_arm_observer_B.m_p = 0;
        left_arm_observer_B.inner_p = 0;
        left_arm_observer_B.u = 0;
        left_arm_observer_B.t_l = -1;
      } else {
        left_arm_observer_B.m_p = static_cast<int32_T>
          (left_arm_observer_B.b_idx_0_c) - 1;
        left_arm_observer_B.inner_p = static_cast<int32_T>
          (left_arm_observer_B.b_idx_1_f);
        left_arm_observer_B.u = left_arm_observer_B.m_p;
        left_arm_observer_B.t_l = left_arm_observer_B.inner_p - 1;
      }

      left_arm_observer_B.u = left_arm_observer_B.t_l - left_arm_observer_B.u;
      left_arm_observer_B.t_l = left_arm_observer_B.u + 1;
      if (0 <= left_arm_observer_B.u) {
        memset(&left_arm_observer_B.qddoti_data[0], 0, (left_arm_observer_B.u +
                1) * sizeof(real_T));
      }

      obj_0 = robot->Bodies[left_arm_observer_B.unnamed_idx_1_k];
      left_arm_observer_B.u = switch_expression->size[0] *
        switch_expression->size[1];
      switch_expression->size[0] = 1;
      switch_expression->size[1] = obj_0->JointInternal.Type->size[1];
      left_a_emxEnsureCapacity_char_T(switch_expression, left_arm_observer_B.u);
      left_arm_observer_B.aoffset_f = obj_0->JointInternal.Type->size[0] *
        obj_0->JointInternal.Type->size[1] - 1;
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u <=
           left_arm_observer_B.aoffset_f; left_arm_observer_B.u++) {
        switch_expression->data[left_arm_observer_B.u] =
          obj_0->JointInternal.Type->data[left_arm_observer_B.u];
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 5;
           left_arm_observer_B.u++) {
        left_arm_observer_B.b_e[left_arm_observer_B.u] =
          tmp[left_arm_observer_B.u];
      }

      left_arm_observer_B.b_bool_i = false;
      if (switch_expression->size[1] == 5) {
        left_arm_observer_B.u = 1;
        do {
          exitg1 = 0;
          if (left_arm_observer_B.u - 1 < 5) {
            left_arm_observer_B.aoffset_f = left_arm_observer_B.u - 1;
            if (switch_expression->data[left_arm_observer_B.aoffset_f] !=
                left_arm_observer_B.b_e[left_arm_observer_B.aoffset_f]) {
              exitg1 = 1;
            } else {
              left_arm_observer_B.u++;
            }
          } else {
            left_arm_observer_B.b_bool_i = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (left_arm_observer_B.b_bool_i) {
        left_arm_observer_B.u = 0;
      } else {
        for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 8;
             left_arm_observer_B.u++) {
          left_arm_observer_B.b_o2[left_arm_observer_B.u] =
            tmp_0[left_arm_observer_B.u];
        }

        left_arm_observer_B.b_bool_i = false;
        if (switch_expression->size[1] == 8) {
          left_arm_observer_B.u = 1;
          do {
            exitg1 = 0;
            if (left_arm_observer_B.u - 1 < 8) {
              left_arm_observer_B.aoffset_f = left_arm_observer_B.u - 1;
              if (switch_expression->data[left_arm_observer_B.aoffset_f] !=
                  left_arm_observer_B.b_o2[left_arm_observer_B.aoffset_f]) {
                exitg1 = 1;
              } else {
                left_arm_observer_B.u++;
              }
            } else {
              left_arm_observer_B.b_bool_i = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (left_arm_observer_B.b_bool_i) {
          left_arm_observer_B.u = 1;
        } else {
          left_arm_observer_B.u = -1;
        }
      }

      switch (left_arm_observer_B.u) {
       case 0:
        memset(&left_arm_observer_B.TJ_c[0], 0, sizeof(real_T) << 4U);
        left_arm_observer_B.TJ_c[0] = 1.0;
        left_arm_observer_B.TJ_c[5] = 1.0;
        left_arm_observer_B.TJ_c[10] = 1.0;
        left_arm_observer_B.TJ_c[15] = 1.0;
        break;

       case 1:
        le_rigidBodyJoint_get_JointAxis(&obj_0->JointInternal,
          left_arm_observer_B.v_h);
        left_arm_observer_B.aoffset_f = left_arm_observer_B.j_n -
          left_arm_observer_B.b_k_o;
        for (left_arm_observer_B.u = 0; left_arm_observer_B.u <
             left_arm_observer_B.aoffset_f; left_arm_observer_B.u++) {
          left_arm_observer_B.l_data[left_arm_observer_B.u] =
            left_arm_observer_B.b_k_o + left_arm_observer_B.u;
        }

        left_arm_observer_B.result_data_m[0] = left_arm_observer_B.v_h[0];
        left_arm_observer_B.result_data_m[1] = left_arm_observer_B.v_h[1];
        left_arm_observer_B.result_data_m[2] = left_arm_observer_B.v_h[2];
        if (0 <= (left_arm_observer_B.aoffset_f != 0) - 1) {
          left_arm_observer_B.result_data_m[3] = q[left_arm_observer_B.l_data[0]];
        }

        left_arm_observer_normalizeRows(&left_arm_observer_B.result_data_m[0],
          left_arm_observer_B.v_h);
        left_arm_observer_B.a_idx_0_m = cos(left_arm_observer_B.result_data_m[3]);
        left_arm_observer_B.sth_l = sin(left_arm_observer_B.result_data_m[3]);
        left_arm_observer_B.a_idx_1_m = left_arm_observer_B.v_h[1] *
          left_arm_observer_B.v_h[0] * (1.0 - left_arm_observer_B.a_idx_0_m);
        left_arm_observer_B.b_idx_0_c = left_arm_observer_B.v_h[2] *
          left_arm_observer_B.sth_l;
        left_arm_observer_B.b_idx_1_f = left_arm_observer_B.v_h[2] *
          left_arm_observer_B.v_h[0] * (1.0 - left_arm_observer_B.a_idx_0_m);
        left_arm_observer_B.tempR_tmp_p = left_arm_observer_B.v_h[1] *
          left_arm_observer_B.sth_l;
        left_arm_observer_B.tempR_tmp_e = left_arm_observer_B.v_h[2] *
          left_arm_observer_B.v_h[1] * (1.0 - left_arm_observer_B.a_idx_0_m);
        left_arm_observer_B.sth_l *= left_arm_observer_B.v_h[0];
        left_arm_observer_cat(left_arm_observer_B.v_h[0] *
                              left_arm_observer_B.v_h[0] * (1.0 -
          left_arm_observer_B.a_idx_0_m) + left_arm_observer_B.a_idx_0_m,
                              left_arm_observer_B.a_idx_1_m -
                              left_arm_observer_B.b_idx_0_c,
                              left_arm_observer_B.b_idx_1_f +
                              left_arm_observer_B.tempR_tmp_p,
                              left_arm_observer_B.a_idx_1_m +
                              left_arm_observer_B.b_idx_0_c,
                              left_arm_observer_B.v_h[1] *
                              left_arm_observer_B.v_h[1] * (1.0 -
          left_arm_observer_B.a_idx_0_m) + left_arm_observer_B.a_idx_0_m,
                              left_arm_observer_B.tempR_tmp_e -
                              left_arm_observer_B.sth_l,
                              left_arm_observer_B.b_idx_1_f -
                              left_arm_observer_B.tempR_tmp_p,
                              left_arm_observer_B.tempR_tmp_e +
                              left_arm_observer_B.sth_l,
                              left_arm_observer_B.v_h[2] *
                              left_arm_observer_B.v_h[2] * (1.0 -
          left_arm_observer_B.a_idx_0_m) + left_arm_observer_B.a_idx_0_m,
                              left_arm_observer_B.tempR_l);
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 3;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.u = left_arm_observer_B.b_k_o + 1;
          left_arm_observer_B.R_dy[left_arm_observer_B.u - 1] =
            left_arm_observer_B.tempR_l[(left_arm_observer_B.u - 1) * 3];
          left_arm_observer_B.u = left_arm_observer_B.b_k_o + 1;
          left_arm_observer_B.R_dy[left_arm_observer_B.u + 2] =
            left_arm_observer_B.tempR_l[(left_arm_observer_B.u - 1) * 3 + 1];
          left_arm_observer_B.u = left_arm_observer_B.b_k_o + 1;
          left_arm_observer_B.R_dy[left_arm_observer_B.u + 5] =
            left_arm_observer_B.tempR_l[(left_arm_observer_B.u - 1) * 3 + 2];
        }

        memset(&left_arm_observer_B.TJ_c[0], 0, sizeof(real_T) << 4U);
        for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 3;
             left_arm_observer_B.u++) {
          left_arm_observer_B.aoffset_f = left_arm_observer_B.u << 2;
          left_arm_observer_B.TJ_c[left_arm_observer_B.aoffset_f] =
            left_arm_observer_B.R_dy[3 * left_arm_observer_B.u];
          left_arm_observer_B.TJ_c[left_arm_observer_B.aoffset_f + 1] =
            left_arm_observer_B.R_dy[3 * left_arm_observer_B.u + 1];
          left_arm_observer_B.TJ_c[left_arm_observer_B.aoffset_f + 2] =
            left_arm_observer_B.R_dy[3 * left_arm_observer_B.u + 2];
        }

        left_arm_observer_B.TJ_c[15] = 1.0;
        break;

       default:
        le_rigidBodyJoint_get_JointAxis(&obj_0->JointInternal,
          left_arm_observer_B.v_h);
        memset(&left_arm_observer_B.tempR_l[0], 0, 9U * sizeof(real_T));
        left_arm_observer_B.tempR_l[0] = 1.0;
        left_arm_observer_B.tempR_l[4] = 1.0;
        left_arm_observer_B.tempR_l[8] = 1.0;
        for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 3;
             left_arm_observer_B.u++) {
          left_arm_observer_B.aoffset_f = left_arm_observer_B.u << 2;
          left_arm_observer_B.TJ_c[left_arm_observer_B.aoffset_f] =
            left_arm_observer_B.tempR_l[3 * left_arm_observer_B.u];
          left_arm_observer_B.TJ_c[left_arm_observer_B.aoffset_f + 1] =
            left_arm_observer_B.tempR_l[3 * left_arm_observer_B.u + 1];
          left_arm_observer_B.TJ_c[left_arm_observer_B.aoffset_f + 2] =
            left_arm_observer_B.tempR_l[3 * left_arm_observer_B.u + 2];
          left_arm_observer_B.TJ_c[left_arm_observer_B.u + 12] =
            left_arm_observer_B.v_h[left_arm_observer_B.u] *
            q[left_arm_observer_B.b_k_o];
        }

        left_arm_observer_B.TJ_c[3] = 0.0;
        left_arm_observer_B.TJ_c[7] = 0.0;
        left_arm_observer_B.TJ_c[11] = 0.0;
        left_arm_observer_B.TJ_c[15] = 1.0;
        break;
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 16;
           left_arm_observer_B.u++) {
        left_arm_observer_B.a[left_arm_observer_B.u] =
          obj_0->JointInternal.JointToParentTransform[left_arm_observer_B.u];
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 16;
           left_arm_observer_B.u++) {
        left_arm_observer_B.b[left_arm_observer_B.u] =
          obj_0->JointInternal.ChildToJointTransform[left_arm_observer_B.u];
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 4;
           left_arm_observer_B.u++) {
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 4;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.aoffset_f = left_arm_observer_B.b_k_o << 2;
          left_arm_observer_B.j_n = left_arm_observer_B.u +
            left_arm_observer_B.aoffset_f;
          left_arm_observer_B.a_b[left_arm_observer_B.j_n] = 0.0;
          left_arm_observer_B.a_b[left_arm_observer_B.j_n] +=
            left_arm_observer_B.TJ_c[left_arm_observer_B.aoffset_f] *
            left_arm_observer_B.a[left_arm_observer_B.u];
          left_arm_observer_B.a_b[left_arm_observer_B.j_n] +=
            left_arm_observer_B.TJ_c[left_arm_observer_B.aoffset_f + 1] *
            left_arm_observer_B.a[left_arm_observer_B.u + 4];
          left_arm_observer_B.a_b[left_arm_observer_B.j_n] +=
            left_arm_observer_B.TJ_c[left_arm_observer_B.aoffset_f + 2] *
            left_arm_observer_B.a[left_arm_observer_B.u + 8];
          left_arm_observer_B.a_b[left_arm_observer_B.j_n] +=
            left_arm_observer_B.TJ_c[left_arm_observer_B.aoffset_f + 3] *
            left_arm_observer_B.a[left_arm_observer_B.u + 12];
        }

        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 4;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.j_n = left_arm_observer_B.b_k_o << 2;
          left_arm_observer_B.aoffset_f = left_arm_observer_B.u +
            left_arm_observer_B.j_n;
          left_arm_observer_B.T_k[left_arm_observer_B.aoffset_f] = 0.0;
          left_arm_observer_B.T_k[left_arm_observer_B.aoffset_f] +=
            left_arm_observer_B.b[left_arm_observer_B.j_n] *
            left_arm_observer_B.a_b[left_arm_observer_B.u];
          left_arm_observer_B.T_k[left_arm_observer_B.aoffset_f] +=
            left_arm_observer_B.b[left_arm_observer_B.j_n + 1] *
            left_arm_observer_B.a_b[left_arm_observer_B.u + 4];
          left_arm_observer_B.T_k[left_arm_observer_B.aoffset_f] +=
            left_arm_observer_B.b[left_arm_observer_B.j_n + 2] *
            left_arm_observer_B.a_b[left_arm_observer_B.u + 8];
          left_arm_observer_B.T_k[left_arm_observer_B.aoffset_f] +=
            left_arm_observer_B.b[left_arm_observer_B.j_n + 3] *
            left_arm_observer_B.a_b[left_arm_observer_B.u + 12];
        }
      }

      if ((S->size[1] == 1) || (left_arm_observer_B.inner_p -
           left_arm_observer_B.m_p == 1)) {
        for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
             left_arm_observer_B.u++) {
          left_arm_observer_B.b_k_o = left_arm_observer_B.u + 6 *
            left_arm_observer_B.unnamed_idx_1_k;
          vJ->data[left_arm_observer_B.b_k_o] = 0.0;
          left_arm_observer_B.aoffset_f = S->size[1];
          for (left_arm_observer_B.m_p = 0; left_arm_observer_B.m_p <
               left_arm_observer_B.aoffset_f; left_arm_observer_B.m_p++) {
            vJ->data[left_arm_observer_B.b_k_o] += S->data[6 *
              left_arm_observer_B.m_p + left_arm_observer_B.u] * 0.0;
          }
        }
      } else {
        left_arm_observer_B.inner_p = S->size[1] - 1;
        for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
             left_arm_observer_B.u++) {
          vJ->data[left_arm_observer_B.u + 6 *
            left_arm_observer_B.unnamed_idx_1_k] = 0.0;
        }

        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o <=
             left_arm_observer_B.inner_p; left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.aoffset_f = left_arm_observer_B.b_k_o * 6 - 1;
          for (left_arm_observer_B.c_i_i = 0; left_arm_observer_B.c_i_i < 6;
               left_arm_observer_B.c_i_i++) {
            left_arm_observer_B.u = 6 * left_arm_observer_B.unnamed_idx_1_k +
              left_arm_observer_B.c_i_i;
            vJ->data[left_arm_observer_B.u] += S->data
              [(left_arm_observer_B.aoffset_f + left_arm_observer_B.c_i_i) + 1] *
              0.0;
          }
        }
      }
    }

    left_arm_observer_tforminv(left_arm_observer_B.T_k, left_arm_observer_B.TJ_c);
    left_arm_ob_tformToSpatialXform(left_arm_observer_B.TJ_c, X->
      data[left_arm_observer_B.unnamed_idx_1_k].f1);
    left_arm_observer_B.a_idx_0_m = robot->
      Bodies[left_arm_observer_B.unnamed_idx_1_k]->ParentIndex;
    if (left_arm_observer_B.a_idx_0_m > 0.0) {
      left_arm_observer_B.m_p = static_cast<int32_T>
        (left_arm_observer_B.a_idx_0_m);
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        left_arm_observer_B.a_idx_1_m = 0.0;
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 6;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.a_idx_1_m += vB->data[(left_arm_observer_B.m_p - 1)
            * 6 + left_arm_observer_B.b_k_o] * X->
            data[left_arm_observer_B.unnamed_idx_1_k].f1[6 *
            left_arm_observer_B.b_k_o + left_arm_observer_B.u];
        }

        left_arm_observer_B.y_m[left_arm_observer_B.u] = vJ->data[6 *
          left_arm_observer_B.unnamed_idx_1_k + left_arm_observer_B.u] +
          left_arm_observer_B.a_idx_1_m;
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        vB->data[left_arm_observer_B.u + 6 * left_arm_observer_B.unnamed_idx_1_k]
          = left_arm_observer_B.y_m[left_arm_observer_B.u];
      }

      if ((S->size[1] == 1) || (left_arm_observer_B.t_l == 1)) {
        left_arm_observer_B.aoffset_f = S->size[1];
        for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
             left_arm_observer_B.u++) {
          left_arm_observer_B.y_m[left_arm_observer_B.u] = 0.0;
          for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o <
               left_arm_observer_B.aoffset_f; left_arm_observer_B.b_k_o++) {
            left_arm_observer_B.a_idx_1_m = S->data[6 *
              left_arm_observer_B.b_k_o + left_arm_observer_B.u] *
              left_arm_observer_B.qddoti_data[left_arm_observer_B.b_k_o] +
              left_arm_observer_B.y_m[left_arm_observer_B.u];
            left_arm_observer_B.y_m[left_arm_observer_B.u] =
              left_arm_observer_B.a_idx_1_m;
          }
        }
      } else {
        left_arm_observer_B.inner_p = S->size[1] - 1;
        for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
             left_arm_observer_B.u++) {
          left_arm_observer_B.y_m[left_arm_observer_B.u] = 0.0;
        }

        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o <=
             left_arm_observer_B.inner_p; left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.aoffset_f = left_arm_observer_B.b_k_o * 6 - 1;
          for (left_arm_observer_B.c_i_i = 0; left_arm_observer_B.c_i_i < 6;
               left_arm_observer_B.c_i_i++) {
            left_arm_observer_B.a_idx_1_m = S->data
              [(left_arm_observer_B.aoffset_f + left_arm_observer_B.c_i_i) + 1] *
              0.0 + left_arm_observer_B.y_m[left_arm_observer_B.c_i_i];
            left_arm_observer_B.y_m[left_arm_observer_B.c_i_i] =
              left_arm_observer_B.a_idx_1_m;
          }
        }
      }

      left_arm_observer_B.tempR_l[0] = 0.0;
      left_arm_observer_B.t_l = 6 * left_arm_observer_B.unnamed_idx_1_k + 2;
      left_arm_observer_B.tempR_l[3] = -vB->data[left_arm_observer_B.t_l];
      left_arm_observer_B.u = 6 * left_arm_observer_B.unnamed_idx_1_k + 1;
      left_arm_observer_B.tempR_l[6] = vB->data[left_arm_observer_B.u];
      left_arm_observer_B.tempR_l[1] = vB->data[left_arm_observer_B.t_l];
      left_arm_observer_B.tempR_l[4] = 0.0;
      left_arm_observer_B.tempR_l[7] = -vB->data[6 *
        left_arm_observer_B.unnamed_idx_1_k];
      left_arm_observer_B.tempR_l[2] = -vB->data[left_arm_observer_B.u];
      left_arm_observer_B.tempR_l[5] = vB->data[6 *
        left_arm_observer_B.unnamed_idx_1_k];
      left_arm_observer_B.tempR_l[8] = 0.0;
      left_arm_observer_B.tempR[3] = 0.0;
      left_arm_observer_B.t_l = 6 * left_arm_observer_B.unnamed_idx_1_k + 5;
      left_arm_observer_B.tempR[9] = -vB->data[left_arm_observer_B.t_l];
      left_arm_observer_B.u = 6 * left_arm_observer_B.unnamed_idx_1_k + 4;
      left_arm_observer_B.tempR[15] = vB->data[left_arm_observer_B.u];
      left_arm_observer_B.tempR[4] = vB->data[left_arm_observer_B.t_l];
      left_arm_observer_B.tempR[10] = 0.0;
      left_arm_observer_B.t_l = 6 * left_arm_observer_B.unnamed_idx_1_k + 3;
      left_arm_observer_B.tempR[16] = -vB->data[left_arm_observer_B.t_l];
      left_arm_observer_B.tempR[5] = -vB->data[left_arm_observer_B.u];
      left_arm_observer_B.tempR[11] = vB->data[left_arm_observer_B.t_l];
      left_arm_observer_B.tempR[17] = 0.0;
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 3;
           left_arm_observer_B.u++) {
        left_arm_observer_B.a_idx_1_m = left_arm_observer_B.tempR_l[3 *
          left_arm_observer_B.u];
        left_arm_observer_B.tempR[6 * left_arm_observer_B.u] =
          left_arm_observer_B.a_idx_1_m;
        left_arm_observer_B.t_l = 6 * (left_arm_observer_B.u + 3);
        left_arm_observer_B.tempR[left_arm_observer_B.t_l] = 0.0;
        left_arm_observer_B.tempR[left_arm_observer_B.t_l + 3] =
          left_arm_observer_B.a_idx_1_m;
        left_arm_observer_B.a_idx_1_m = left_arm_observer_B.tempR_l[3 *
          left_arm_observer_B.u + 1];
        left_arm_observer_B.tempR[6 * left_arm_observer_B.u + 1] =
          left_arm_observer_B.a_idx_1_m;
        left_arm_observer_B.tempR[left_arm_observer_B.t_l + 1] = 0.0;
        left_arm_observer_B.tempR[left_arm_observer_B.t_l + 4] =
          left_arm_observer_B.a_idx_1_m;
        left_arm_observer_B.a_idx_1_m = left_arm_observer_B.tempR_l[3 *
          left_arm_observer_B.u + 2];
        left_arm_observer_B.tempR[6 * left_arm_observer_B.u + 2] =
          left_arm_observer_B.a_idx_1_m;
        left_arm_observer_B.tempR[left_arm_observer_B.t_l + 2] = 0.0;
        left_arm_observer_B.tempR[left_arm_observer_B.t_l + 5] =
          left_arm_observer_B.a_idx_1_m;
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        left_arm_observer_B.a_idx_1_m = 0.0;
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 6;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.a_idx_1_m += aB->data[(left_arm_observer_B.m_p - 1)
            * 6 + left_arm_observer_B.b_k_o] * X->
            data[left_arm_observer_B.unnamed_idx_1_k].f1[6 *
            left_arm_observer_B.b_k_o + left_arm_observer_B.u];
        }

        left_arm_observer_B.X_cz[left_arm_observer_B.u] =
          left_arm_observer_B.a_idx_1_m +
          left_arm_observer_B.y_m[left_arm_observer_B.u];
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        left_arm_observer_B.y_m[left_arm_observer_B.u] = 0.0;
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 6;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.a_idx_1_m = left_arm_observer_B.tempR[6 *
            left_arm_observer_B.b_k_o + left_arm_observer_B.u] * vJ->data[6 *
            left_arm_observer_B.unnamed_idx_1_k + left_arm_observer_B.b_k_o] +
            left_arm_observer_B.y_m[left_arm_observer_B.u];
          left_arm_observer_B.y_m[left_arm_observer_B.u] =
            left_arm_observer_B.a_idx_1_m;
        }

        aB->data[left_arm_observer_B.u + 6 * left_arm_observer_B.unnamed_idx_1_k]
          = left_arm_observer_B.X_cz[left_arm_observer_B.u] +
          left_arm_observer_B.y_m[left_arm_observer_B.u];
      }

      left_arm_observer_B.R_dy[0] = 0.0;
      left_arm_observer_B.R_dy[3] = -left_arm_observer_B.T_k[14];
      left_arm_observer_B.R_dy[6] = left_arm_observer_B.T_k[13];
      left_arm_observer_B.R_dy[1] = left_arm_observer_B.T_k[14];
      left_arm_observer_B.R_dy[4] = 0.0;
      left_arm_observer_B.R_dy[7] = -left_arm_observer_B.T_k[12];
      left_arm_observer_B.R_dy[2] = -left_arm_observer_B.T_k[13];
      left_arm_observer_B.R_dy[5] = left_arm_observer_B.T_k[12];
      left_arm_observer_B.R_dy[8] = 0.0;
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 3;
           left_arm_observer_B.u++) {
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 3;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.m_p = left_arm_observer_B.u + 3 *
            left_arm_observer_B.b_k_o;
          left_arm_observer_B.dv5[left_arm_observer_B.m_p] = 0.0;
          left_arm_observer_B.t_l = left_arm_observer_B.b_k_o << 2;
          left_arm_observer_B.dv5[left_arm_observer_B.m_p] +=
            left_arm_observer_B.T_k[left_arm_observer_B.t_l] *
            left_arm_observer_B.R_dy[left_arm_observer_B.u];
          left_arm_observer_B.dv5[left_arm_observer_B.m_p] +=
            left_arm_observer_B.T_k[left_arm_observer_B.t_l + 1] *
            left_arm_observer_B.R_dy[left_arm_observer_B.u + 3];
          left_arm_observer_B.dv5[left_arm_observer_B.m_p] +=
            left_arm_observer_B.T_k[left_arm_observer_B.t_l + 2] *
            left_arm_observer_B.R_dy[left_arm_observer_B.u + 6];
          left_arm_observer_B.b_I_m[left_arm_observer_B.b_k_o + 6 *
            left_arm_observer_B.u] = left_arm_observer_B.T_k
            [(left_arm_observer_B.u << 2) + left_arm_observer_B.b_k_o];
          left_arm_observer_B.b_I_m[left_arm_observer_B.b_k_o + 6 *
            (left_arm_observer_B.u + 3)] = 0.0;
        }
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 3;
           left_arm_observer_B.u++) {
        left_arm_observer_B.b_I_m[6 * left_arm_observer_B.u + 3] =
          left_arm_observer_B.dv5[3 * left_arm_observer_B.u];
        left_arm_observer_B.aoffset_f = left_arm_observer_B.u << 2;
        left_arm_observer_B.t_l = 6 * (left_arm_observer_B.u + 3);
        left_arm_observer_B.b_I_m[left_arm_observer_B.t_l + 3] =
          left_arm_observer_B.T_k[left_arm_observer_B.aoffset_f];
        left_arm_observer_B.b_I_m[6 * left_arm_observer_B.u + 4] =
          left_arm_observer_B.dv5[3 * left_arm_observer_B.u + 1];
        left_arm_observer_B.b_I_m[left_arm_observer_B.t_l + 4] =
          left_arm_observer_B.T_k[left_arm_observer_B.aoffset_f + 1];
        left_arm_observer_B.b_I_m[6 * left_arm_observer_B.u + 5] =
          left_arm_observer_B.dv5[3 * left_arm_observer_B.u + 2];
        left_arm_observer_B.b_I_m[left_arm_observer_B.t_l + 5] =
          left_arm_observer_B.T_k[left_arm_observer_B.aoffset_f + 2];
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 6;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.t_l = left_arm_observer_B.u + 6 *
            left_arm_observer_B.b_k_o;
          left_arm_observer_B.tempR[left_arm_observer_B.t_l] = 0.0;
          for (left_arm_observer_B.m_p = 0; left_arm_observer_B.m_p < 6;
               left_arm_observer_B.m_p++) {
            left_arm_observer_B.tempR[left_arm_observer_B.t_l] += Xtree->data[
              static_cast<int32_T>(left_arm_observer_B.a_idx_0_m) - 1].f1[6 *
              left_arm_observer_B.m_p + left_arm_observer_B.u] *
              left_arm_observer_B.b_I_m[6 * left_arm_observer_B.b_k_o +
              left_arm_observer_B.m_p];
          }
        }
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 36;
           left_arm_observer_B.u++) {
        Xtree->data[left_arm_observer_B.unnamed_idx_1_k]
          .f1[left_arm_observer_B.u] =
          left_arm_observer_B.tempR[left_arm_observer_B.u];
      }
    } else {
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        left_arm_observer_B.b_k_o = 6 * left_arm_observer_B.unnamed_idx_1_k +
          left_arm_observer_B.u;
        vB->data[left_arm_observer_B.b_k_o] = vJ->data[left_arm_observer_B.b_k_o];
      }

      if ((S->size[1] == 1) || (left_arm_observer_B.t_l == 1)) {
        left_arm_observer_B.aoffset_f = S->size[1];
        for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
             left_arm_observer_B.u++) {
          left_arm_observer_B.y_m[left_arm_observer_B.u] = 0.0;
          for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o <
               left_arm_observer_B.aoffset_f; left_arm_observer_B.b_k_o++) {
            left_arm_observer_B.a_idx_1_m = S->data[6 *
              left_arm_observer_B.b_k_o + left_arm_observer_B.u] *
              left_arm_observer_B.qddoti_data[left_arm_observer_B.b_k_o] +
              left_arm_observer_B.y_m[left_arm_observer_B.u];
            left_arm_observer_B.y_m[left_arm_observer_B.u] =
              left_arm_observer_B.a_idx_1_m;
          }
        }
      } else {
        left_arm_observer_B.inner_p = S->size[1] - 1;
        for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
             left_arm_observer_B.u++) {
          left_arm_observer_B.y_m[left_arm_observer_B.u] = 0.0;
        }

        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o <=
             left_arm_observer_B.inner_p; left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.aoffset_f = left_arm_observer_B.b_k_o * 6 - 1;
          for (left_arm_observer_B.c_i_i = 0; left_arm_observer_B.c_i_i < 6;
               left_arm_observer_B.c_i_i++) {
            left_arm_observer_B.a_idx_1_m = S->data
              [(left_arm_observer_B.aoffset_f + left_arm_observer_B.c_i_i) + 1] *
              0.0 + left_arm_observer_B.y_m[left_arm_observer_B.c_i_i];
            left_arm_observer_B.y_m[left_arm_observer_B.c_i_i] =
              left_arm_observer_B.a_idx_1_m;
          }
        }
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        left_arm_observer_B.a_idx_1_m = 0.0;
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 6;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.a_idx_1_m += X->
            data[left_arm_observer_B.unnamed_idx_1_k].f1[6 *
            left_arm_observer_B.b_k_o + left_arm_observer_B.u] *
            left_arm_observer_B.a0_n[left_arm_observer_B.b_k_o];
        }

        aB->data[left_arm_observer_B.u + 6 * left_arm_observer_B.unnamed_idx_1_k]
          = left_arm_observer_B.a_idx_1_m +
          left_arm_observer_B.y_m[left_arm_observer_B.u];
      }

      left_arm_observer_B.R_dy[0] = 0.0;
      left_arm_observer_B.R_dy[3] = -left_arm_observer_B.T_k[14];
      left_arm_observer_B.R_dy[6] = left_arm_observer_B.T_k[13];
      left_arm_observer_B.R_dy[1] = left_arm_observer_B.T_k[14];
      left_arm_observer_B.R_dy[4] = 0.0;
      left_arm_observer_B.R_dy[7] = -left_arm_observer_B.T_k[12];
      left_arm_observer_B.R_dy[2] = -left_arm_observer_B.T_k[13];
      left_arm_observer_B.R_dy[5] = left_arm_observer_B.T_k[12];
      left_arm_observer_B.R_dy[8] = 0.0;
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 3;
           left_arm_observer_B.u++) {
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 3;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.m_p = left_arm_observer_B.u + 3 *
            left_arm_observer_B.b_k_o;
          left_arm_observer_B.dv5[left_arm_observer_B.m_p] = 0.0;
          left_arm_observer_B.t_l = left_arm_observer_B.b_k_o << 2;
          left_arm_observer_B.dv5[left_arm_observer_B.m_p] +=
            left_arm_observer_B.T_k[left_arm_observer_B.t_l] *
            left_arm_observer_B.R_dy[left_arm_observer_B.u];
          left_arm_observer_B.dv5[left_arm_observer_B.m_p] +=
            left_arm_observer_B.T_k[left_arm_observer_B.t_l + 1] *
            left_arm_observer_B.R_dy[left_arm_observer_B.u + 3];
          left_arm_observer_B.dv5[left_arm_observer_B.m_p] +=
            left_arm_observer_B.T_k[left_arm_observer_B.t_l + 2] *
            left_arm_observer_B.R_dy[left_arm_observer_B.u + 6];
          Xtree->data[left_arm_observer_B.unnamed_idx_1_k]
            .f1[left_arm_observer_B.b_k_o + 6 * left_arm_observer_B.u] =
            left_arm_observer_B.T_k[(left_arm_observer_B.u << 2) +
            left_arm_observer_B.b_k_o];
          Xtree->data[left_arm_observer_B.unnamed_idx_1_k]
            .f1[left_arm_observer_B.b_k_o + 6 * (left_arm_observer_B.u + 3)] =
            0.0;
        }
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 3;
           left_arm_observer_B.u++) {
        Xtree->data[left_arm_observer_B.unnamed_idx_1_k].f1[6 *
          left_arm_observer_B.u + 3] = left_arm_observer_B.dv5[3 *
          left_arm_observer_B.u];
        left_arm_observer_B.b_k_o = left_arm_observer_B.u << 2;
        left_arm_observer_B.m_p = 6 * (left_arm_observer_B.u + 3);
        Xtree->data[left_arm_observer_B.unnamed_idx_1_k]
          .f1[left_arm_observer_B.m_p + 3] =
          left_arm_observer_B.T_k[left_arm_observer_B.b_k_o];
        Xtree->data[left_arm_observer_B.unnamed_idx_1_k].f1[6 *
          left_arm_observer_B.u + 4] = left_arm_observer_B.dv5[3 *
          left_arm_observer_B.u + 1];
        Xtree->data[left_arm_observer_B.unnamed_idx_1_k]
          .f1[left_arm_observer_B.m_p + 4] =
          left_arm_observer_B.T_k[left_arm_observer_B.b_k_o + 1];
        Xtree->data[left_arm_observer_B.unnamed_idx_1_k].f1[6 *
          left_arm_observer_B.u + 5] = left_arm_observer_B.dv5[3 *
          left_arm_observer_B.u + 2];
        Xtree->data[left_arm_observer_B.unnamed_idx_1_k]
          .f1[left_arm_observer_B.m_p + 5] =
          left_arm_observer_B.T_k[left_arm_observer_B.b_k_o + 2];
      }
    }

    for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 36;
         left_arm_observer_B.u++) {
      left_arm_observer_B.b_I_m[left_arm_observer_B.u] = robot->
        Bodies[left_arm_observer_B.unnamed_idx_1_k]->
        SpatialInertia[left_arm_observer_B.u];
    }

    left_arm_observer_B.tempR_l[0] = 0.0;
    left_arm_observer_B.t_l = 6 * left_arm_observer_B.unnamed_idx_1_k + 2;
    left_arm_observer_B.tempR_l[3] = -vB->data[left_arm_observer_B.t_l];
    left_arm_observer_B.u = 6 * left_arm_observer_B.unnamed_idx_1_k + 1;
    left_arm_observer_B.tempR_l[6] = vB->data[left_arm_observer_B.u];
    left_arm_observer_B.tempR_l[1] = vB->data[left_arm_observer_B.t_l];
    left_arm_observer_B.tempR_l[4] = 0.0;
    left_arm_observer_B.tempR_l[7] = -vB->data[6 *
      left_arm_observer_B.unnamed_idx_1_k];
    left_arm_observer_B.tempR_l[2] = -vB->data[left_arm_observer_B.u];
    left_arm_observer_B.tempR_l[5] = vB->data[6 *
      left_arm_observer_B.unnamed_idx_1_k];
    left_arm_observer_B.tempR_l[8] = 0.0;
    left_arm_observer_B.tempR[18] = 0.0;
    left_arm_observer_B.t_l = 6 * left_arm_observer_B.unnamed_idx_1_k + 5;
    left_arm_observer_B.tempR[24] = -vB->data[left_arm_observer_B.t_l];
    left_arm_observer_B.u = 6 * left_arm_observer_B.unnamed_idx_1_k + 4;
    left_arm_observer_B.tempR[30] = vB->data[left_arm_observer_B.u];
    left_arm_observer_B.tempR[19] = vB->data[left_arm_observer_B.t_l];
    left_arm_observer_B.tempR[25] = 0.0;
    left_arm_observer_B.t_l = 6 * left_arm_observer_B.unnamed_idx_1_k + 3;
    left_arm_observer_B.tempR[31] = -vB->data[left_arm_observer_B.t_l];
    left_arm_observer_B.tempR[20] = -vB->data[left_arm_observer_B.u];
    left_arm_observer_B.tempR[26] = vB->data[left_arm_observer_B.t_l];
    left_arm_observer_B.tempR[32] = 0.0;
    for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 3;
         left_arm_observer_B.u++) {
      left_arm_observer_B.a_idx_1_m = left_arm_observer_B.tempR_l[3 *
        left_arm_observer_B.u];
      left_arm_observer_B.tempR[6 * left_arm_observer_B.u] =
        left_arm_observer_B.a_idx_1_m;
      left_arm_observer_B.tempR[6 * left_arm_observer_B.u + 3] = 0.0;
      left_arm_observer_B.t_l = 6 * (left_arm_observer_B.u + 3);
      left_arm_observer_B.tempR[left_arm_observer_B.t_l + 3] =
        left_arm_observer_B.a_idx_1_m;
      left_arm_observer_B.a_idx_1_m = left_arm_observer_B.tempR_l[3 *
        left_arm_observer_B.u + 1];
      left_arm_observer_B.tempR[6 * left_arm_observer_B.u + 1] =
        left_arm_observer_B.a_idx_1_m;
      left_arm_observer_B.tempR[6 * left_arm_observer_B.u + 4] = 0.0;
      left_arm_observer_B.tempR[left_arm_observer_B.t_l + 4] =
        left_arm_observer_B.a_idx_1_m;
      left_arm_observer_B.a_idx_1_m = left_arm_observer_B.tempR_l[3 *
        left_arm_observer_B.u + 2];
      left_arm_observer_B.tempR[6 * left_arm_observer_B.u + 2] =
        left_arm_observer_B.a_idx_1_m;
      left_arm_observer_B.tempR[6 * left_arm_observer_B.u + 5] = 0.0;
      left_arm_observer_B.tempR[left_arm_observer_B.t_l + 5] =
        left_arm_observer_B.a_idx_1_m;
    }

    for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
         left_arm_observer_B.u++) {
      left_arm_observer_B.X_cz[left_arm_observer_B.u] = 0.0;
      left_arm_observer_B.b_I_md[left_arm_observer_B.u] = 0.0;
      for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 6;
           left_arm_observer_B.b_k_o++) {
        left_arm_observer_B.a_idx_0_m = left_arm_observer_B.b_I_m[6 *
          left_arm_observer_B.b_k_o + left_arm_observer_B.u];
        left_arm_observer_B.t_l = 6 * left_arm_observer_B.unnamed_idx_1_k +
          left_arm_observer_B.b_k_o;
        left_arm_observer_B.a_idx_1_m = vB->data[left_arm_observer_B.t_l] *
          left_arm_observer_B.a_idx_0_m +
          left_arm_observer_B.X_cz[left_arm_observer_B.u];
        left_arm_observer_B.a_idx_0_m = aB->data[left_arm_observer_B.t_l] *
          left_arm_observer_B.a_idx_0_m +
          left_arm_observer_B.b_I_md[left_arm_observer_B.u];
        left_arm_observer_B.X_cz[left_arm_observer_B.u] =
          left_arm_observer_B.a_idx_1_m;
        left_arm_observer_B.b_I_md[left_arm_observer_B.u] =
          left_arm_observer_B.a_idx_0_m;
      }
    }

    for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
         left_arm_observer_B.u++) {
      left_arm_observer_B.y_m[left_arm_observer_B.u] = 0.0;
      left_arm_observer_B.a_idx_1_m = 0.0;
      for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 6;
           left_arm_observer_B.b_k_o++) {
        left_arm_observer_B.a_idx_1_m += Xtree->
          data[left_arm_observer_B.unnamed_idx_1_k].f1[6 * left_arm_observer_B.u
          + left_arm_observer_B.b_k_o] * 0.0;
        left_arm_observer_B.y_m[left_arm_observer_B.u] +=
          left_arm_observer_B.tempR[6 * left_arm_observer_B.b_k_o +
          left_arm_observer_B.u] *
          left_arm_observer_B.X_cz[left_arm_observer_B.b_k_o];
      }

      f->data[left_arm_observer_B.u + 6 * left_arm_observer_B.unnamed_idx_1_k] =
        (left_arm_observer_B.b_I_md[left_arm_observer_B.u] +
         left_arm_observer_B.y_m[left_arm_observer_B.u]) -
        left_arm_observer_B.a_idx_1_m;
    }
  }

  left_arm_observe_emxFree_char_T(&switch_expression);
  left_arm_observe_emxFree_real_T(&aB);
  left_arm_observe_emxFree_real_T(&vB);
  left_arm_observe_emxFree_real_T(&vJ);
  left_arm_ob_emxFree_e_cell_wrap(&Xtree);
  left_arm_observer_B.i_o = static_cast<int32_T>(((-1.0 -
    left_arm_observer_B.nb_o) + 1.0) / -1.0) - 1;
  left_arm_observe_emxInit_real_T(&taui, 1);
  left_arm_observe_emxInit_real_T(&a, 2);
  for (left_arm_observer_B.t_l = 0; left_arm_observer_B.t_l <=
       left_arm_observer_B.i_o; left_arm_observer_B.t_l++) {
    left_arm_observer_B.a_idx_0_m = left_arm_observer_B.nb_o +
      -static_cast<real_T>(left_arm_observer_B.t_l);
    left_arm_observer_B.inner_p = static_cast<int32_T>
      (left_arm_observer_B.a_idx_0_m);
    left_arm_observer_B.j_n = left_arm_observer_B.inner_p - 1;
    obj_0 = robot->Bodies[left_arm_observer_B.j_n];
    if (!left_arm_observer_strcmp(obj_0->JointInternal.Type)) {
      obj_0 = robot->Bodies[left_arm_observer_B.j_n];
      left_arm_observer_B.u = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj_0->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(S, left_arm_observer_B.u);
      left_arm_observer_B.aoffset_f = obj_0->JointInternal.MotionSubspace->size
        [0] * obj_0->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u <=
           left_arm_observer_B.aoffset_f; left_arm_observer_B.u++) {
        S->data[left_arm_observer_B.u] = obj_0->
          JointInternal.MotionSubspace->data[left_arm_observer_B.u];
      }

      left_arm_observer_B.u = a->size[0] * a->size[1];
      a->size[0] = S->size[1];
      a->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a, left_arm_observer_B.u);
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        left_arm_observer_B.aoffset_f = S->size[1];
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o <
             left_arm_observer_B.aoffset_f; left_arm_observer_B.b_k_o++) {
          a->data[left_arm_observer_B.b_k_o + a->size[0] * left_arm_observer_B.u]
            = S->data[6 * left_arm_observer_B.b_k_o + left_arm_observer_B.u];
        }
      }

      left_arm_observer_B.m_p = a->size[0] - 1;
      left_arm_observer_B.u = taui->size[0];
      taui->size[0] = a->size[0];
      left_a_emxEnsureCapacity_real_T(taui, left_arm_observer_B.u);
      for (left_arm_observer_B.unnamed_idx_1_k = 0;
           left_arm_observer_B.unnamed_idx_1_k <= left_arm_observer_B.m_p;
           left_arm_observer_B.unnamed_idx_1_k++) {
        taui->data[left_arm_observer_B.unnamed_idx_1_k] = 0.0;
      }

      for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 6;
           left_arm_observer_B.b_k_o++) {
        left_arm_observer_B.aoffset_f = (left_arm_observer_B.m_p + 1) *
          left_arm_observer_B.b_k_o - 1;
        for (left_arm_observer_B.c_i_i = 0; left_arm_observer_B.c_i_i <=
             left_arm_observer_B.m_p; left_arm_observer_B.c_i_i++) {
          taui->data[left_arm_observer_B.c_i_i] += f->data[(static_cast<int32_T>
            (left_arm_observer_B.a_idx_0_m) - 1) * 6 + left_arm_observer_B.b_k_o]
            * a->data[(left_arm_observer_B.aoffset_f + left_arm_observer_B.c_i_i)
            + 1];
        }
      }

      left_arm_observer_B.b_idx_0_c = robot->
        VelocityDoFMap[left_arm_observer_B.inner_p - 1];
      left_arm_observer_B.b_idx_1_f = robot->
        VelocityDoFMap[left_arm_observer_B.inner_p + 9];
      if (left_arm_observer_B.b_idx_0_c > left_arm_observer_B.b_idx_1_f) {
        left_arm_observer_B.m_p = 0;
        left_arm_observer_B.unnamed_idx_1_k = 0;
      } else {
        left_arm_observer_B.m_p = static_cast<int32_T>
          (left_arm_observer_B.b_idx_0_c) - 1;
        left_arm_observer_B.unnamed_idx_1_k = static_cast<int32_T>
          (left_arm_observer_B.b_idx_1_f);
      }

      left_arm_observer_B.unnamed_idx_1_k -= left_arm_observer_B.m_p;
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u <
           left_arm_observer_B.unnamed_idx_1_k; left_arm_observer_B.u++) {
        jointTorq[left_arm_observer_B.m_p + left_arm_observer_B.u] = taui->
          data[left_arm_observer_B.u];
      }
    }

    left_arm_observer_B.a_idx_0_m = robot->Bodies[left_arm_observer_B.j_n]
      ->ParentIndex;
    if (left_arm_observer_B.a_idx_0_m > 0.0) {
      left_arm_observer_B.m_p = static_cast<int32_T>
        (left_arm_observer_B.a_idx_0_m);
      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        left_arm_observer_B.a_idx_1_m = 0.0;
        for (left_arm_observer_B.b_k_o = 0; left_arm_observer_B.b_k_o < 6;
             left_arm_observer_B.b_k_o++) {
          left_arm_observer_B.a_idx_1_m += f->data[(left_arm_observer_B.inner_p
            - 1) * 6 + left_arm_observer_B.b_k_o] * X->
            data[left_arm_observer_B.j_n].f1[6 * left_arm_observer_B.u +
            left_arm_observer_B.b_k_o];
        }

        left_arm_observer_B.a0_n[left_arm_observer_B.u] = f->data
          [(left_arm_observer_B.m_p - 1) * 6 + left_arm_observer_B.u] +
          left_arm_observer_B.a_idx_1_m;
      }

      for (left_arm_observer_B.u = 0; left_arm_observer_B.u < 6;
           left_arm_observer_B.u++) {
        f->data[left_arm_observer_B.u + 6 * (left_arm_observer_B.m_p - 1)] =
          left_arm_observer_B.a0_n[left_arm_observer_B.u];
      }
    }
  }

  left_arm_observe_emxFree_real_T(&a);
  left_arm_observe_emxFree_real_T(&taui);
  left_arm_observe_emxFree_real_T(&S);
  left_arm_observe_emxFree_real_T(&f);
  left_arm_ob_emxFree_e_cell_wrap(&X);
}

static void left_arm_ob_emxInit_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_left_arm_T *emxArray;
  *pEmxArray = (emxArray_f_cell_wrap_left_arm_T *)malloc(sizeof
    (emxArray_f_cell_wrap_left_arm_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_left_arm_observer_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (left_arm_observer_B.i_j = 0; left_arm_observer_B.i_j < numDimensions;
       left_arm_observer_B.i_j++) {
    emxArray->size[left_arm_observer_B.i_j] = 0;
  }
}

static void l_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  *emxArray, int32_T oldNumel)
{
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  left_arm_observer_B.newNumel_n = 1;
  for (left_arm_observer_B.i_k = 0; left_arm_observer_B.i_k <
       emxArray->numDimensions; left_arm_observer_B.i_k++) {
    left_arm_observer_B.newNumel_n *= emxArray->size[left_arm_observer_B.i_k];
  }

  if (left_arm_observer_B.newNumel_n > emxArray->allocatedSize) {
    left_arm_observer_B.i_k = emxArray->allocatedSize;
    if (left_arm_observer_B.i_k < 16) {
      left_arm_observer_B.i_k = 16;
    }

    while (left_arm_observer_B.i_k < left_arm_observer_B.newNumel_n) {
      if (left_arm_observer_B.i_k > 1073741823) {
        left_arm_observer_B.i_k = MAX_int32_T;
      } else {
        left_arm_observer_B.i_k <<= 1;
      }
    }

    newData = calloc(static_cast<uint32_T>(left_arm_observer_B.i_k), sizeof
                     (f_cell_wrap_left_arm_observer_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_left_arm_observer_T)
             * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_left_arm_observer_T *)newData;
    emxArray->allocatedSize = left_arm_observer_B.i_k;
    emxArray->canFreeData = true;
  }
}

static void left_arm_ob_emxFree_f_cell_wrap(emxArray_f_cell_wrap_left_arm_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_left_arm_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_left_arm_observer_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_left_arm_T *)NULL;
  }
}

static void RigidBodyTreeDynamics_massMatri(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], emxArray_real_T_left_arm_obse_T *H,
  emxArray_real_T_left_arm_obse_T *lambda)
{
  emxArray_f_cell_wrap_left_arm_T *Ic;
  emxArray_f_cell_wrap_left_arm_T *X;
  emxArray_real_T_left_arm_obse_T *lambda_;
  emxArray_real_T_left_arm_obse_T *Si;
  emxArray_real_T_left_arm_obse_T *Fi;
  emxArray_real_T_left_arm_obse_T *Sj;
  emxArray_real_T_left_arm_obse_T *Hji;
  emxArray_real_T_left_arm_obse_T *s;
  j_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_left_arm_obse_T *a;
  emxArray_real_T_left_arm_obse_T *a_0;
  emxArray_real_T_left_arm_obse_T *B;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T exitg1;
  int32_T exitg2;
  left_arm_observer_B.nb_m = robot->NumBodies;
  left_arm_observer_B.vNum_m = robot->VelocityNumber;
  left_arm_observer_B.nm1d2 = H->size[0] * H->size[1];
  left_arm_observer_B.b_i_f = static_cast<int32_T>(left_arm_observer_B.vNum_m);
  H->size[0] = left_arm_observer_B.b_i_f;
  H->size[1] = left_arm_observer_B.b_i_f;
  left_a_emxEnsureCapacity_real_T(H, left_arm_observer_B.nm1d2);
  left_arm_observer_B.n_h = left_arm_observer_B.b_i_f *
    left_arm_observer_B.b_i_f - 1;
  for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <=
       left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
    H->data[left_arm_observer_B.nm1d2] = 0.0;
  }

  left_arm_observe_emxInit_real_T(&lambda_, 2);
  left_arm_observer_B.nm1d2 = lambda_->size[0] * lambda_->size[1];
  lambda_->size[0] = 1;
  left_arm_observer_B.unnamed_idx_1_n = static_cast<int32_T>
    (left_arm_observer_B.nb_m);
  lambda_->size[1] = left_arm_observer_B.unnamed_idx_1_n;
  left_a_emxEnsureCapacity_real_T(lambda_, left_arm_observer_B.nm1d2);
  left_arm_observer_B.idx = left_arm_observer_B.unnamed_idx_1_n - 1;
  for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <=
       left_arm_observer_B.idx; left_arm_observer_B.nm1d2++) {
    lambda_->data[left_arm_observer_B.nm1d2] = 0.0;
  }

  left_arm_observer_B.nm1d2 = lambda->size[0] * lambda->size[1];
  lambda->size[0] = 1;
  lambda->size[1] = left_arm_observer_B.b_i_f;
  left_a_emxEnsureCapacity_real_T(lambda, left_arm_observer_B.nm1d2);
  left_arm_observer_B.n_h = left_arm_observer_B.b_i_f - 1;
  for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <=
       left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
    lambda->data[left_arm_observer_B.nm1d2] = 0.0;
  }

  left_arm_ob_emxInit_f_cell_wrap(&Ic, 2);
  left_arm_ob_emxInit_f_cell_wrap(&X, 2);
  left_arm_observer_B.nm1d2 = Ic->size[0] * Ic->size[1];
  Ic->size[0] = 1;
  Ic->size[1] = left_arm_observer_B.unnamed_idx_1_n;
  l_emxEnsureCapacity_f_cell_wrap(Ic, left_arm_observer_B.nm1d2);
  left_arm_observer_B.nm1d2 = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_observer_B.unnamed_idx_1_n;
  l_emxEnsureCapacity_f_cell_wrap(X, left_arm_observer_B.nm1d2);
  for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f <=
       left_arm_observer_B.idx; left_arm_observer_B.b_i_f++) {
    for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 < 36;
         left_arm_observer_B.nm1d2++) {
      Ic->data[left_arm_observer_B.b_i_f].f1[left_arm_observer_B.nm1d2] =
        robot->Bodies[left_arm_observer_B.b_i_f]->
        SpatialInertia[left_arm_observer_B.nm1d2];
    }

    left_arm_observer_B.vNum_m = robot->PositionDoFMap[left_arm_observer_B.b_i_f];
    left_arm_observer_B.p_idx_1 = robot->
      PositionDoFMap[left_arm_observer_B.b_i_f + 10];
    if (left_arm_observer_B.p_idx_1 < left_arm_observer_B.vNum_m) {
      obj = robot->Bodies[left_arm_observer_B.b_i_f];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal,
        left_arm_observer_B.T_f);
    } else {
      if (left_arm_observer_B.vNum_m > left_arm_observer_B.p_idx_1) {
        left_arm_observer_B.unnamed_idx_1_n = 0;
        left_arm_observer_B.nm1d2 = -1;
      } else {
        left_arm_observer_B.unnamed_idx_1_n = static_cast<int32_T>
          (left_arm_observer_B.vNum_m) - 1;
        left_arm_observer_B.nm1d2 = static_cast<int32_T>
          (left_arm_observer_B.p_idx_1) - 1;
      }

      obj = robot->Bodies[left_arm_observer_B.b_i_f];
      left_arm_observer_B.q_size_tmp = left_arm_observer_B.nm1d2 -
        left_arm_observer_B.unnamed_idx_1_n;
      left_arm_observer_B.q_size_c = left_arm_observer_B.q_size_tmp + 1;
      for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <=
           left_arm_observer_B.q_size_tmp; left_arm_observer_B.nm1d2++) {
        left_arm_observer_B.q_data_j[left_arm_observer_B.nm1d2] =
          q[left_arm_observer_B.unnamed_idx_1_n + left_arm_observer_B.nm1d2];
      }

      rigidBodyJoint_transformBodyT_i(&obj->JointInternal,
        left_arm_observer_B.q_data_j, &left_arm_observer_B.q_size_c,
        left_arm_observer_B.T_f);
    }

    left_arm_observer_tforminv(left_arm_observer_B.T_f, left_arm_observer_B.dv);
    left_arm_ob_tformToSpatialXform(left_arm_observer_B.dv, X->
      data[left_arm_observer_B.b_i_f].f1);
  }

  left_arm_observer_B.idx = static_cast<int32_T>(((-1.0 -
    left_arm_observer_B.nb_m) + 1.0) / -1.0) - 1;
  left_arm_observe_emxInit_real_T(&Si, 2);
  left_arm_observe_emxInit_real_T(&Fi, 2);
  left_arm_observe_emxInit_real_T(&Sj, 2);
  left_arm_observe_emxInit_real_T(&Hji, 2);
  left_arm_observe_emxInit_char_T(&a, 2);
  left_arm_observe_emxInit_real_T(&a_0, 2);
  left_arm_observe_emxInit_real_T(&B, 2);
  for (left_arm_observer_B.unnamed_idx_1_n = 0;
       left_arm_observer_B.unnamed_idx_1_n <= left_arm_observer_B.idx;
       left_arm_observer_B.unnamed_idx_1_n++) {
    left_arm_observer_B.pid_tmp = static_cast<int32_T>(left_arm_observer_B.nb_m
      + -static_cast<real_T>(left_arm_observer_B.unnamed_idx_1_n));
    left_arm_observer_B.q_size_tmp = left_arm_observer_B.pid_tmp - 1;
    left_arm_observer_B.pid = robot->Bodies[left_arm_observer_B.q_size_tmp]
      ->ParentIndex;
    left_arm_observer_B.vNum_m = robot->
      VelocityDoFMap[left_arm_observer_B.pid_tmp - 1];
    left_arm_observer_B.p_idx_1 = robot->
      VelocityDoFMap[left_arm_observer_B.pid_tmp + 9];
    if (left_arm_observer_B.pid > 0.0) {
      for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 < 6;
           left_arm_observer_B.nm1d2++) {
        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
             left_arm_observer_B.b_i_f++) {
          left_arm_observer_B.n_h = left_arm_observer_B.nm1d2 + 6 *
            left_arm_observer_B.b_i_f;
          left_arm_observer_B.X[left_arm_observer_B.n_h] = 0.0;
          for (left_arm_observer_B.cb = 0; left_arm_observer_B.cb < 6;
               left_arm_observer_B.cb++) {
            left_arm_observer_B.X[left_arm_observer_B.n_h] += X->
              data[left_arm_observer_B.q_size_tmp].f1[6 *
              left_arm_observer_B.nm1d2 + left_arm_observer_B.cb] * Ic->
              data[left_arm_observer_B.q_size_tmp].f1[6 *
              left_arm_observer_B.b_i_f + left_arm_observer_B.cb];
          }
        }
      }

      for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 < 6;
           left_arm_observer_B.nm1d2++) {
        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
             left_arm_observer_B.b_i_f++) {
          left_arm_observer_B.b_idx_0_h = 0.0;
          for (left_arm_observer_B.cb = 0; left_arm_observer_B.cb < 6;
               left_arm_observer_B.cb++) {
            left_arm_observer_B.b_idx_0_h += left_arm_observer_B.X[6 *
              left_arm_observer_B.cb + left_arm_observer_B.nm1d2] * X->
              data[left_arm_observer_B.q_size_tmp].f1[6 *
              left_arm_observer_B.b_i_f + left_arm_observer_B.cb];
          }

          left_arm_observer_B.cb = 6 * left_arm_observer_B.b_i_f +
            left_arm_observer_B.nm1d2;
          Ic->data[static_cast<int32_T>(left_arm_observer_B.pid) - 1]
            .f1[left_arm_observer_B.cb] += left_arm_observer_B.b_idx_0_h;
        }
      }

      lambda_->data[left_arm_observer_B.q_size_tmp] = left_arm_observer_B.pid;
      if (lambda_->data[left_arm_observer_B.q_size_tmp] > 0.0) {
        for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 < 5;
             left_arm_observer_B.nm1d2++) {
          left_arm_observer_B.b_h2[left_arm_observer_B.nm1d2] =
            tmp[left_arm_observer_B.nm1d2];
        }
      }

      exitg1 = false;
      while ((!exitg1) && (lambda_->data[left_arm_observer_B.q_size_tmp] > 0.0))
      {
        obj = robot->Bodies[static_cast<int32_T>(lambda_->
          data[left_arm_observer_B.q_size_tmp]) - 1];
        left_arm_observer_B.nm1d2 = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = obj->JointInternal.Type->size[1];
        left_a_emxEnsureCapacity_char_T(a, left_arm_observer_B.nm1d2);
        left_arm_observer_B.n_h = obj->JointInternal.Type->size[0] *
          obj->JointInternal.Type->size[1] - 1;
        for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <=
             left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
          a->data[left_arm_observer_B.nm1d2] = obj->JointInternal.Type->
            data[left_arm_observer_B.nm1d2];
        }

        left_arm_observer_B.b_bool_g = false;
        if (a->size[1] == 5) {
          left_arm_observer_B.nm1d2 = 1;
          do {
            exitg2 = 0;
            if (left_arm_observer_B.nm1d2 - 1 < 5) {
              left_arm_observer_B.n_h = left_arm_observer_B.nm1d2 - 1;
              if (a->data[left_arm_observer_B.n_h] !=
                  left_arm_observer_B.b_h2[left_arm_observer_B.n_h]) {
                exitg2 = 1;
              } else {
                left_arm_observer_B.nm1d2++;
              }
            } else {
              left_arm_observer_B.b_bool_g = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (left_arm_observer_B.b_bool_g) {
          lambda_->data[left_arm_observer_B.q_size_tmp] = robot->Bodies[
            static_cast<int32_T>(lambda_->data[left_arm_observer_B.q_size_tmp])
            - 1]->ParentIndex;
        } else {
          exitg1 = true;
        }
      }
    }

    left_arm_observer_B.b_idx_0_h = robot->
      VelocityDoFMap[left_arm_observer_B.pid_tmp - 1];
    left_arm_observer_B.b_idx_1_c = robot->
      VelocityDoFMap[left_arm_observer_B.pid_tmp + 9];
    if (left_arm_observer_B.b_idx_0_h <= left_arm_observer_B.b_idx_1_c) {
      obj = robot->Bodies[left_arm_observer_B.q_size_tmp];
      left_arm_observer_B.nm1d2 = Si->size[0] * Si->size[1];
      Si->size[0] = 6;
      Si->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(Si, left_arm_observer_B.nm1d2);
      left_arm_observer_B.n_h = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <=
           left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
        Si->data[left_arm_observer_B.nm1d2] = obj->
          JointInternal.MotionSubspace->data[left_arm_observer_B.nm1d2];
      }

      left_arm_observer_B.n_h = Si->size[1] - 1;
      left_arm_observer_B.nm1d2 = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = Si->size[1];
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_observer_B.nm1d2);
      for (left_arm_observer_B.b_j_k = 0; left_arm_observer_B.b_j_k <=
           left_arm_observer_B.n_h; left_arm_observer_B.b_j_k++) {
        left_arm_observer_B.pid_tmp = left_arm_observer_B.b_j_k * 6 - 1;
        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
             left_arm_observer_B.b_i_f++) {
          Fi->data[(left_arm_observer_B.pid_tmp + left_arm_observer_B.b_i_f) + 1]
            = 0.0;
        }

        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
             left_arm_observer_B.b_i_f++) {
          left_arm_observer_B.aoffset_m = left_arm_observer_B.b_i_f * 6 - 1;
          left_arm_observer_B.temp = Si->data[(left_arm_observer_B.pid_tmp +
            left_arm_observer_B.b_i_f) + 1];
          for (left_arm_observer_B.c_i_p = 0; left_arm_observer_B.c_i_p < 6;
               left_arm_observer_B.c_i_p++) {
            left_arm_observer_B.i_ad = left_arm_observer_B.c_i_p + 1;
            left_arm_observer_B.nm1d2 = left_arm_observer_B.pid_tmp +
              left_arm_observer_B.i_ad;
            Fi->data[left_arm_observer_B.nm1d2] += Ic->
              data[left_arm_observer_B.q_size_tmp]
              .f1[left_arm_observer_B.aoffset_m + left_arm_observer_B.i_ad] *
              left_arm_observer_B.temp;
          }
        }
      }

      if (left_arm_observer_B.vNum_m > left_arm_observer_B.p_idx_1) {
        left_arm_observer_B.pid_tmp = 0;
        left_arm_observer_B.cb = 0;
      } else {
        left_arm_observer_B.pid_tmp = static_cast<int32_T>
          (left_arm_observer_B.vNum_m) - 1;
        left_arm_observer_B.cb = left_arm_observer_B.pid_tmp;
      }

      left_arm_observer_B.nm1d2 = a_0->size[0] * a_0->size[1];
      a_0->size[0] = Si->size[1];
      a_0->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a_0, left_arm_observer_B.nm1d2);
      for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 < 6;
           left_arm_observer_B.nm1d2++) {
        left_arm_observer_B.n_h = Si->size[1];
        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f <
             left_arm_observer_B.n_h; left_arm_observer_B.b_i_f++) {
          a_0->data[left_arm_observer_B.b_i_f + a_0->size[0] *
            left_arm_observer_B.nm1d2] = Si->data[6 * left_arm_observer_B.b_i_f
            + left_arm_observer_B.nm1d2];
        }
      }

      left_arm_observer_B.m_b = a_0->size[0];
      left_arm_observer_B.n_h = Fi->size[1] - 1;
      left_arm_observer_B.nm1d2 = Hji->size[0] * Hji->size[1];
      Hji->size[0] = a_0->size[0];
      Hji->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(Hji, left_arm_observer_B.nm1d2);
      for (left_arm_observer_B.b_j_k = 0; left_arm_observer_B.b_j_k <=
           left_arm_observer_B.n_h; left_arm_observer_B.b_j_k++) {
        left_arm_observer_B.coffset = left_arm_observer_B.b_j_k *
          left_arm_observer_B.m_b - 1;
        left_arm_observer_B.boffset = left_arm_observer_B.b_j_k * 6 - 1;
        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f <
             left_arm_observer_B.m_b; left_arm_observer_B.b_i_f++) {
          Hji->data[(left_arm_observer_B.coffset + left_arm_observer_B.b_i_f) +
            1] = 0.0;
        }

        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
             left_arm_observer_B.b_i_f++) {
          left_arm_observer_B.aoffset_m = left_arm_observer_B.b_i_f *
            left_arm_observer_B.m_b - 1;
          left_arm_observer_B.temp = Fi->data[(left_arm_observer_B.boffset +
            left_arm_observer_B.b_i_f) + 1];
          for (left_arm_observer_B.c_i_p = 0; left_arm_observer_B.c_i_p <
               left_arm_observer_B.m_b; left_arm_observer_B.c_i_p++) {
            left_arm_observer_B.i_ad = left_arm_observer_B.c_i_p + 1;
            left_arm_observer_B.nm1d2 = left_arm_observer_B.coffset +
              left_arm_observer_B.i_ad;
            Hji->data[left_arm_observer_B.nm1d2] += a_0->
              data[left_arm_observer_B.aoffset_m + left_arm_observer_B.i_ad] *
              left_arm_observer_B.temp;
          }
        }
      }

      left_arm_observer_B.n_h = Hji->size[1];
      for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <
           left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
        left_arm_observer_B.b_j_k = Hji->size[0];
        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f <
             left_arm_observer_B.b_j_k; left_arm_observer_B.b_i_f++) {
          H->data[(left_arm_observer_B.pid_tmp + left_arm_observer_B.b_i_f) +
            H->size[0] * (left_arm_observer_B.cb + left_arm_observer_B.nm1d2)] =
            Hji->data[Hji->size[0] * left_arm_observer_B.nm1d2 +
            left_arm_observer_B.b_i_f];
        }
      }

      for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 < 6;
           left_arm_observer_B.nm1d2++) {
        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
             left_arm_observer_B.b_i_f++) {
          left_arm_observer_B.X[left_arm_observer_B.b_i_f + 6 *
            left_arm_observer_B.nm1d2] = X->data[left_arm_observer_B.q_size_tmp]
            .f1[6 * left_arm_observer_B.b_i_f + left_arm_observer_B.nm1d2];
        }
      }

      left_arm_observer_B.nm1d2 = B->size[0] * B->size[1];
      B->size[0] = 6;
      B->size[1] = Fi->size[1];
      left_a_emxEnsureCapacity_real_T(B, left_arm_observer_B.nm1d2);
      left_arm_observer_B.n_h = Fi->size[0] * Fi->size[1] - 1;
      for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <=
           left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
        B->data[left_arm_observer_B.nm1d2] = Fi->data[left_arm_observer_B.nm1d2];
      }

      left_arm_observer_B.n_h = Fi->size[1];
      left_arm_observer_B.nm1d2 = Fi->size[0] * Fi->size[1];
      Fi->size[0] = 6;
      Fi->size[1] = left_arm_observer_B.n_h;
      left_a_emxEnsureCapacity_real_T(Fi, left_arm_observer_B.nm1d2);
      for (left_arm_observer_B.b_j_k = 0; left_arm_observer_B.b_j_k <
           left_arm_observer_B.n_h; left_arm_observer_B.b_j_k++) {
        left_arm_observer_B.pid_tmp = left_arm_observer_B.b_j_k * 6 - 1;
        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
             left_arm_observer_B.b_i_f++) {
          Fi->data[(left_arm_observer_B.pid_tmp + left_arm_observer_B.b_i_f) + 1]
            = 0.0;
        }

        for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
             left_arm_observer_B.b_i_f++) {
          left_arm_observer_B.aoffset_m = left_arm_observer_B.b_i_f * 6 - 1;
          left_arm_observer_B.temp = B->data[(left_arm_observer_B.pid_tmp +
            left_arm_observer_B.b_i_f) + 1];
          for (left_arm_observer_B.c_i_p = 0; left_arm_observer_B.c_i_p < 6;
               left_arm_observer_B.c_i_p++) {
            left_arm_observer_B.i_ad = left_arm_observer_B.c_i_p + 1;
            left_arm_observer_B.nm1d2 = left_arm_observer_B.pid_tmp +
              left_arm_observer_B.i_ad;
            Fi->data[left_arm_observer_B.nm1d2] +=
              left_arm_observer_B.X[left_arm_observer_B.aoffset_m +
              left_arm_observer_B.i_ad] * left_arm_observer_B.temp;
          }
        }
      }

      while (left_arm_observer_B.pid > 0.0) {
        left_arm_observer_B.b_i_f = static_cast<int32_T>(left_arm_observer_B.pid);
        left_arm_observer_B.q_size_tmp = left_arm_observer_B.b_i_f - 1;
        obj = robot->Bodies[left_arm_observer_B.q_size_tmp];
        left_arm_observer_B.nm1d2 = Sj->size[0] * Sj->size[1];
        Sj->size[0] = 6;
        Sj->size[1] = obj->JointInternal.MotionSubspace->size[1];
        left_a_emxEnsureCapacity_real_T(Sj, left_arm_observer_B.nm1d2);
        left_arm_observer_B.n_h = obj->JointInternal.MotionSubspace->size[0] *
          obj->JointInternal.MotionSubspace->size[1] - 1;
        for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <=
             left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
          Sj->data[left_arm_observer_B.nm1d2] =
            obj->JointInternal.MotionSubspace->data[left_arm_observer_B.nm1d2];
        }

        left_arm_observer_B.b_idx_0_h = robot->
          VelocityDoFMap[left_arm_observer_B.b_i_f - 1];
        left_arm_observer_B.b_idx_1_c = robot->
          VelocityDoFMap[left_arm_observer_B.b_i_f + 9];
        if (left_arm_observer_B.b_idx_0_h <= left_arm_observer_B.b_idx_1_c) {
          left_arm_observer_B.nm1d2 = a_0->size[0] * a_0->size[1];
          a_0->size[0] = Sj->size[1];
          a_0->size[1] = 6;
          left_a_emxEnsureCapacity_real_T(a_0, left_arm_observer_B.nm1d2);
          for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 < 6;
               left_arm_observer_B.nm1d2++) {
            left_arm_observer_B.n_h = Sj->size[1];
            for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f <
                 left_arm_observer_B.n_h; left_arm_observer_B.b_i_f++) {
              a_0->data[left_arm_observer_B.b_i_f + a_0->size[0] *
                left_arm_observer_B.nm1d2] = Sj->data[6 *
                left_arm_observer_B.b_i_f + left_arm_observer_B.nm1d2];
            }
          }

          left_arm_observer_B.m_b = a_0->size[0];
          left_arm_observer_B.n_h = Fi->size[1] - 1;
          left_arm_observer_B.nm1d2 = Hji->size[0] * Hji->size[1];
          Hji->size[0] = a_0->size[0];
          Hji->size[1] = Fi->size[1];
          left_a_emxEnsureCapacity_real_T(Hji, left_arm_observer_B.nm1d2);
          for (left_arm_observer_B.b_j_k = 0; left_arm_observer_B.b_j_k <=
               left_arm_observer_B.n_h; left_arm_observer_B.b_j_k++) {
            left_arm_observer_B.coffset = left_arm_observer_B.b_j_k *
              left_arm_observer_B.m_b - 1;
            left_arm_observer_B.boffset = left_arm_observer_B.b_j_k * 6 - 1;
            for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f <
                 left_arm_observer_B.m_b; left_arm_observer_B.b_i_f++) {
              Hji->data[(left_arm_observer_B.coffset + left_arm_observer_B.b_i_f)
                + 1] = 0.0;
            }

            for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
                 left_arm_observer_B.b_i_f++) {
              left_arm_observer_B.aoffset_m = left_arm_observer_B.b_i_f *
                left_arm_observer_B.m_b - 1;
              left_arm_observer_B.temp = Fi->data[(left_arm_observer_B.boffset +
                left_arm_observer_B.b_i_f) + 1];
              for (left_arm_observer_B.c_i_p = 0; left_arm_observer_B.c_i_p <
                   left_arm_observer_B.m_b; left_arm_observer_B.c_i_p++) {
                left_arm_observer_B.i_ad = left_arm_observer_B.c_i_p + 1;
                left_arm_observer_B.nm1d2 = left_arm_observer_B.coffset +
                  left_arm_observer_B.i_ad;
                Hji->data[left_arm_observer_B.nm1d2] += a_0->
                  data[left_arm_observer_B.aoffset_m + left_arm_observer_B.i_ad]
                  * left_arm_observer_B.temp;
              }
            }
          }

          if (left_arm_observer_B.b_idx_0_h > left_arm_observer_B.b_idx_1_c) {
            left_arm_observer_B.pid_tmp = 0;
          } else {
            left_arm_observer_B.pid_tmp = static_cast<int32_T>
              (left_arm_observer_B.b_idx_0_h) - 1;
          }

          if (left_arm_observer_B.vNum_m > left_arm_observer_B.p_idx_1) {
            left_arm_observer_B.cb = 0;
          } else {
            left_arm_observer_B.cb = static_cast<int32_T>
              (left_arm_observer_B.vNum_m) - 1;
          }

          left_arm_observer_B.n_h = Hji->size[1];
          for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <
               left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
            left_arm_observer_B.b_j_k = Hji->size[0];
            for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f <
                 left_arm_observer_B.b_j_k; left_arm_observer_B.b_i_f++) {
              H->data[(left_arm_observer_B.pid_tmp + left_arm_observer_B.b_i_f)
                + H->size[0] * (left_arm_observer_B.cb +
                                left_arm_observer_B.nm1d2)] = Hji->data
                [Hji->size[0] * left_arm_observer_B.nm1d2 +
                left_arm_observer_B.b_i_f];
            }
          }

          if (left_arm_observer_B.vNum_m > left_arm_observer_B.p_idx_1) {
            left_arm_observer_B.pid_tmp = 0;
          } else {
            left_arm_observer_B.pid_tmp = static_cast<int32_T>
              (left_arm_observer_B.vNum_m) - 1;
          }

          if (left_arm_observer_B.b_idx_0_h > left_arm_observer_B.b_idx_1_c) {
            left_arm_observer_B.cb = 0;
          } else {
            left_arm_observer_B.cb = static_cast<int32_T>
              (left_arm_observer_B.b_idx_0_h) - 1;
          }

          left_arm_observer_B.n_h = Hji->size[0];
          for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <
               left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
            left_arm_observer_B.b_j_k = Hji->size[1];
            for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f <
                 left_arm_observer_B.b_j_k; left_arm_observer_B.b_i_f++) {
              H->data[(left_arm_observer_B.pid_tmp + left_arm_observer_B.b_i_f)
                + H->size[0] * (left_arm_observer_B.cb +
                                left_arm_observer_B.nm1d2)] = Hji->data
                [Hji->size[0] * left_arm_observer_B.b_i_f +
                left_arm_observer_B.nm1d2];
            }
          }
        }

        for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 < 6;
             left_arm_observer_B.nm1d2++) {
          for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
               left_arm_observer_B.b_i_f++) {
            left_arm_observer_B.X[left_arm_observer_B.b_i_f + 6 *
              left_arm_observer_B.nm1d2] = X->
              data[left_arm_observer_B.q_size_tmp].f1[6 *
              left_arm_observer_B.b_i_f + left_arm_observer_B.nm1d2];
          }
        }

        left_arm_observer_B.nm1d2 = B->size[0] * B->size[1];
        B->size[0] = 6;
        B->size[1] = Fi->size[1];
        left_a_emxEnsureCapacity_real_T(B, left_arm_observer_B.nm1d2);
        left_arm_observer_B.n_h = Fi->size[0] * Fi->size[1] - 1;
        for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <=
             left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
          B->data[left_arm_observer_B.nm1d2] = Fi->
            data[left_arm_observer_B.nm1d2];
        }

        left_arm_observer_B.n_h = Fi->size[1];
        left_arm_observer_B.nm1d2 = Fi->size[0] * Fi->size[1];
        Fi->size[0] = 6;
        Fi->size[1] = left_arm_observer_B.n_h;
        left_a_emxEnsureCapacity_real_T(Fi, left_arm_observer_B.nm1d2);
        for (left_arm_observer_B.b_j_k = 0; left_arm_observer_B.b_j_k <
             left_arm_observer_B.n_h; left_arm_observer_B.b_j_k++) {
          left_arm_observer_B.pid_tmp = left_arm_observer_B.b_j_k * 6 - 1;
          for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
               left_arm_observer_B.b_i_f++) {
            Fi->data[(left_arm_observer_B.pid_tmp + left_arm_observer_B.b_i_f) +
              1] = 0.0;
          }

          for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f < 6;
               left_arm_observer_B.b_i_f++) {
            left_arm_observer_B.aoffset_m = left_arm_observer_B.b_i_f * 6 - 1;
            left_arm_observer_B.temp = B->data[(left_arm_observer_B.pid_tmp +
              left_arm_observer_B.b_i_f) + 1];
            for (left_arm_observer_B.c_i_p = 0; left_arm_observer_B.c_i_p < 6;
                 left_arm_observer_B.c_i_p++) {
              left_arm_observer_B.i_ad = left_arm_observer_B.c_i_p + 1;
              left_arm_observer_B.nm1d2 = left_arm_observer_B.pid_tmp +
                left_arm_observer_B.i_ad;
              Fi->data[left_arm_observer_B.nm1d2] +=
                left_arm_observer_B.X[left_arm_observer_B.aoffset_m +
                left_arm_observer_B.i_ad] * left_arm_observer_B.temp;
            }
          }
        }

        left_arm_observer_B.pid = robot->Bodies[left_arm_observer_B.q_size_tmp
          ]->ParentIndex;
      }
    }
  }

  left_arm_observe_emxFree_real_T(&B);
  left_arm_observe_emxFree_real_T(&a_0);
  left_arm_observe_emxFree_char_T(&a);
  left_arm_observe_emxFree_real_T(&Hji);
  left_arm_observe_emxFree_real_T(&Sj);
  left_arm_observe_emxFree_real_T(&Fi);
  left_arm_observe_emxFree_real_T(&Si);
  left_arm_ob_emxFree_f_cell_wrap(&X);
  left_arm_ob_emxFree_f_cell_wrap(&Ic);
  for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 < 10;
       left_arm_observer_B.nm1d2++) {
    left_arm_observer_B.mask[left_arm_observer_B.nm1d2] = (robot->
      VelocityDoFMap[left_arm_observer_B.nm1d2] <= robot->
      VelocityDoFMap[left_arm_observer_B.nm1d2 + 10]);
  }

  left_arm_observer_B.idx = 0;
  left_arm_observer_B.nm1d2 = 1;
  exitg1 = false;
  while ((!exitg1) && (left_arm_observer_B.nm1d2 - 1 < 10)) {
    if (left_arm_observer_B.mask[left_arm_observer_B.nm1d2 - 1]) {
      left_arm_observer_B.idx++;
      left_arm_observer_B.ii_data[left_arm_observer_B.idx - 1] =
        left_arm_observer_B.nm1d2;
      if (left_arm_observer_B.idx >= 10) {
        exitg1 = true;
      } else {
        left_arm_observer_B.nm1d2++;
      }
    } else {
      left_arm_observer_B.nm1d2++;
    }
  }

  if (1 > left_arm_observer_B.idx) {
    left_arm_observer_B.idx = 0;
  }

  for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <
       left_arm_observer_B.idx; left_arm_observer_B.nm1d2++) {
    left_arm_observer_B.nonFixedIndices_data[left_arm_observer_B.nm1d2] =
      left_arm_observer_B.ii_data[left_arm_observer_B.nm1d2];
  }

  left_arm_observer_B.idx--;
  left_arm_observe_emxInit_real_T(&s, 2);
  for (left_arm_observer_B.unnamed_idx_1_n = 0;
       left_arm_observer_B.unnamed_idx_1_n <= left_arm_observer_B.idx;
       left_arm_observer_B.unnamed_idx_1_n++) {
    left_arm_observer_B.vNum_m = robot->
      VelocityDoFMap[left_arm_observer_B.nonFixedIndices_data[left_arm_observer_B.unnamed_idx_1_n]
      - 1];
    left_arm_observer_B.p_idx_1 = robot->
      VelocityDoFMap[left_arm_observer_B.nonFixedIndices_data[left_arm_observer_B.unnamed_idx_1_n]
      + 9];
    if (rtIsNaN(left_arm_observer_B.vNum_m) || rtIsNaN
        (left_arm_observer_B.p_idx_1)) {
      left_arm_observer_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_observer_B.nm1d2);
      s->data[0] = (rtNaN);
    } else if (left_arm_observer_B.p_idx_1 < left_arm_observer_B.vNum_m) {
      s->size[0] = 1;
      s->size[1] = 0;
    } else if ((rtIsInf(left_arm_observer_B.vNum_m) || rtIsInf
                (left_arm_observer_B.p_idx_1)) && (left_arm_observer_B.vNum_m ==
                left_arm_observer_B.p_idx_1)) {
      left_arm_observer_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_observer_B.nm1d2);
      s->data[0] = (rtNaN);
    } else if (floor(left_arm_observer_B.vNum_m) == left_arm_observer_B.vNum_m)
    {
      left_arm_observer_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      left_arm_observer_B.n_h = static_cast<int32_T>(floor
        (left_arm_observer_B.p_idx_1 - left_arm_observer_B.vNum_m));
      s->size[1] = left_arm_observer_B.n_h + 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_observer_B.nm1d2);
      for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <=
           left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
        s->data[left_arm_observer_B.nm1d2] = left_arm_observer_B.vNum_m +
          static_cast<real_T>(left_arm_observer_B.nm1d2);
      }
    } else {
      left_arm_observer_B.nb_m = floor((left_arm_observer_B.p_idx_1 -
        left_arm_observer_B.vNum_m) + 0.5);
      left_arm_observer_B.pid = left_arm_observer_B.vNum_m +
        left_arm_observer_B.nb_m;
      left_arm_observer_B.b_idx_0_h = left_arm_observer_B.pid -
        left_arm_observer_B.p_idx_1;
      left_arm_observer_B.b_idx_1_c = fabs(left_arm_observer_B.vNum_m);
      left_arm_observer_B.temp = fabs(left_arm_observer_B.p_idx_1);
      if ((left_arm_observer_B.b_idx_1_c > left_arm_observer_B.temp) || rtIsNaN
          (left_arm_observer_B.temp)) {
        left_arm_observer_B.temp = left_arm_observer_B.b_idx_1_c;
      }

      if (fabs(left_arm_observer_B.b_idx_0_h) < 4.4408920985006262E-16 *
          left_arm_observer_B.temp) {
        left_arm_observer_B.nb_m++;
        left_arm_observer_B.pid = left_arm_observer_B.p_idx_1;
      } else if (left_arm_observer_B.b_idx_0_h > 0.0) {
        left_arm_observer_B.pid = (left_arm_observer_B.nb_m - 1.0) +
          left_arm_observer_B.vNum_m;
      } else {
        left_arm_observer_B.nb_m++;
      }

      if (left_arm_observer_B.nb_m >= 0.0) {
        left_arm_observer_B.nm1d2 = static_cast<int32_T>
          (left_arm_observer_B.nb_m);
      } else {
        left_arm_observer_B.nm1d2 = 0;
      }

      left_arm_observer_B.n_h = left_arm_observer_B.nm1d2 - 1;
      left_arm_observer_B.nm1d2 = s->size[0] * s->size[1];
      s->size[0] = 1;
      s->size[1] = left_arm_observer_B.n_h + 1;
      left_a_emxEnsureCapacity_real_T(s, left_arm_observer_B.nm1d2);
      if (left_arm_observer_B.n_h + 1 > 0) {
        s->data[0] = left_arm_observer_B.vNum_m;
        if (left_arm_observer_B.n_h + 1 > 1) {
          s->data[left_arm_observer_B.n_h] = left_arm_observer_B.pid;
          left_arm_observer_B.nm1d2 = left_arm_observer_B.n_h / 2;
          left_arm_observer_B.q_size_tmp = left_arm_observer_B.nm1d2 - 2;
          for (left_arm_observer_B.b_i_f = 0; left_arm_observer_B.b_i_f <=
               left_arm_observer_B.q_size_tmp; left_arm_observer_B.b_i_f++) {
            left_arm_observer_B.pid_tmp = left_arm_observer_B.b_i_f + 1;
            s->data[left_arm_observer_B.pid_tmp] = left_arm_observer_B.vNum_m +
              static_cast<real_T>(left_arm_observer_B.pid_tmp);
            s->data[left_arm_observer_B.n_h - left_arm_observer_B.pid_tmp] =
              left_arm_observer_B.pid - static_cast<real_T>
              (left_arm_observer_B.pid_tmp);
          }

          if (left_arm_observer_B.nm1d2 << 1 == left_arm_observer_B.n_h) {
            s->data[left_arm_observer_B.nm1d2] = (left_arm_observer_B.vNum_m +
              left_arm_observer_B.pid) / 2.0;
          } else {
            s->data[left_arm_observer_B.nm1d2] = left_arm_observer_B.vNum_m +
              static_cast<real_T>(left_arm_observer_B.nm1d2);
            s->data[left_arm_observer_B.nm1d2 + 1] = left_arm_observer_B.pid -
              static_cast<real_T>(left_arm_observer_B.nm1d2);
          }
        }
      }
    }

    if (left_arm_observer_B.vNum_m > left_arm_observer_B.p_idx_1) {
      left_arm_observer_B.q_size_tmp = 0;
    } else {
      left_arm_observer_B.q_size_tmp = static_cast<int32_T>
        (left_arm_observer_B.vNum_m) - 1;
    }

    left_arm_observer_B.n_h = s->size[1];
    for (left_arm_observer_B.nm1d2 = 0; left_arm_observer_B.nm1d2 <
         left_arm_observer_B.n_h; left_arm_observer_B.nm1d2++) {
      lambda->data[left_arm_observer_B.q_size_tmp + left_arm_observer_B.nm1d2] =
        s->data[left_arm_observer_B.nm1d2] - 1.0;
    }

    if (lambda_->
        data[left_arm_observer_B.nonFixedIndices_data[left_arm_observer_B.unnamed_idx_1_n]
        - 1] == 0.0) {
      lambda->data[static_cast<int32_T>(left_arm_observer_B.vNum_m) - 1] = 0.0;
    } else {
      left_arm_observer_B.nm1d2 = static_cast<int32_T>(lambda_->
        data[left_arm_observer_B.nonFixedIndices_data[left_arm_observer_B.unnamed_idx_1_n]
        - 1]);
      left_arm_observer_B.b_idx_1_c = robot->
        VelocityDoFMap[left_arm_observer_B.nm1d2 + 9];
      lambda->data[static_cast<int32_T>(left_arm_observer_B.vNum_m) - 1] =
        left_arm_observer_B.b_idx_1_c;
    }
  }

  left_arm_observe_emxFree_real_T(&s);
  left_arm_observe_emxFree_real_T(&lambda_);
}

static void RigidBodyTreeDynamics_inverseDy(k_robotics_manip_internal_Rig_T
  *robot, const real_T q[7], const real_T qdot[7], const
  emxArray_real_T_left_arm_obse_T *qddot, const real_T fext[60], real_T tau[7])
{
  emxArray_f_cell_wrap_left_arm_T *X;
  emxArray_f_cell_wrap_left_arm_T *Xtree;
  emxArray_real_T_left_arm_obse_T *vJ;
  emxArray_real_T_left_arm_obse_T *vB;
  emxArray_real_T_left_arm_obse_T *aB;
  emxArray_real_T_left_arm_obse_T *f;
  emxArray_real_T_left_arm_obse_T *S;
  emxArray_real_T_left_arm_obse_T *qddoti;
  j_robotics_manip_internal_Rig_T *obj;
  emxArray_char_T_left_arm_obse_T *a;
  emxArray_real_T_left_arm_obse_T *a_0;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  left_arm_observer_B.a0[0] = 0.0;
  left_arm_observer_B.a0[1] = 0.0;
  left_arm_observer_B.a0[2] = 0.0;
  left_arm_observer_B.a0[3] = -robot->Gravity[0];
  left_arm_observer_B.a0[4] = -robot->Gravity[1];
  left_arm_observer_B.a0[5] = -robot->Gravity[2];
  left_arm_observe_emxInit_real_T(&vJ, 2);
  left_arm_observer_B.nb = robot->NumBodies;
  left_arm_observer_B.i_af = vJ->size[0] * vJ->size[1];
  vJ->size[0] = 6;
  left_arm_observer_B.unnamed_idx_1 = static_cast<int32_T>
    (left_arm_observer_B.nb);
  vJ->size[1] = left_arm_observer_B.unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(vJ, left_arm_observer_B.i_af);
  left_arm_observer_B.loop_ub_tmp = 6 * left_arm_observer_B.unnamed_idx_1 - 1;
  for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af <=
       left_arm_observer_B.loop_ub_tmp; left_arm_observer_B.i_af++) {
    vJ->data[left_arm_observer_B.i_af] = 0.0;
  }

  left_arm_observe_emxInit_real_T(&vB, 2);
  left_arm_observer_B.i_af = vB->size[0] * vB->size[1];
  vB->size[0] = 6;
  vB->size[1] = left_arm_observer_B.unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(vB, left_arm_observer_B.i_af);
  for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af <=
       left_arm_observer_B.loop_ub_tmp; left_arm_observer_B.i_af++) {
    vB->data[left_arm_observer_B.i_af] = 0.0;
  }

  left_arm_observe_emxInit_real_T(&aB, 2);
  left_arm_observer_B.i_af = aB->size[0] * aB->size[1];
  aB->size[0] = 6;
  aB->size[1] = left_arm_observer_B.unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(aB, left_arm_observer_B.i_af);
  for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af <=
       left_arm_observer_B.loop_ub_tmp; left_arm_observer_B.i_af++) {
    aB->data[left_arm_observer_B.i_af] = 0.0;
  }

  for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 7;
       left_arm_observer_B.i_af++) {
    tau[left_arm_observer_B.i_af] = 0.0;
  }

  left_arm_ob_emxInit_f_cell_wrap(&X, 2);
  left_arm_ob_emxInit_f_cell_wrap(&Xtree, 2);
  left_arm_observer_B.loop_ub_tmp = left_arm_observer_B.unnamed_idx_1 - 1;
  left_arm_observer_B.i_af = Xtree->size[0] * Xtree->size[1];
  Xtree->size[0] = 1;
  Xtree->size[1] = left_arm_observer_B.unnamed_idx_1;
  l_emxEnsureCapacity_f_cell_wrap(Xtree, left_arm_observer_B.i_af);
  left_arm_observer_B.i_af = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = left_arm_observer_B.unnamed_idx_1;
  l_emxEnsureCapacity_f_cell_wrap(X, left_arm_observer_B.i_af);
  for (left_arm_observer_B.b_k = 0; left_arm_observer_B.b_k <=
       left_arm_observer_B.loop_ub_tmp; left_arm_observer_B.b_k++) {
    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 36;
         left_arm_observer_B.i_af++) {
      Xtree->data[left_arm_observer_B.b_k].f1[left_arm_observer_B.i_af] = 0.0;
    }

    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
         left_arm_observer_B.i_af++) {
      Xtree->data[left_arm_observer_B.b_k].f1[left_arm_observer_B.i_af + 6 *
        left_arm_observer_B.i_af] = 1.0;
    }

    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 36;
         left_arm_observer_B.i_af++) {
      X->data[left_arm_observer_B.b_k].f1[left_arm_observer_B.i_af] = 0.0;
    }

    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
         left_arm_observer_B.i_af++) {
      X->data[left_arm_observer_B.b_k].f1[left_arm_observer_B.i_af + 6 *
        left_arm_observer_B.i_af] = 1.0;
    }
  }

  left_arm_observe_emxInit_real_T(&f, 2);
  left_arm_observer_B.i_af = f->size[0] * f->size[1];
  f->size[0] = 6;
  f->size[1] = left_arm_observer_B.unnamed_idx_1;
  left_a_emxEnsureCapacity_real_T(f, left_arm_observer_B.i_af);
  left_arm_observe_emxInit_real_T(&S, 2);
  left_arm_observe_emxInit_real_T(&qddoti, 1);
  if (0 <= left_arm_observer_B.loop_ub_tmp) {
    left_arm_observer_B.dv2[0] = 0.0;
    left_arm_observer_B.dv2[4] = 0.0;
    left_arm_observer_B.dv2[8] = 0.0;
  }

  for (left_arm_observer_B.unnamed_idx_1 = 0; left_arm_observer_B.unnamed_idx_1 <=
       left_arm_observer_B.loop_ub_tmp; left_arm_observer_B.unnamed_idx_1++) {
    obj = robot->Bodies[left_arm_observer_B.unnamed_idx_1];
    left_arm_observer_B.i_af = S->size[0] * S->size[1];
    S->size[0] = 6;
    S->size[1] = obj->JointInternal.MotionSubspace->size[1];
    left_a_emxEnsureCapacity_real_T(S, left_arm_observer_B.i_af);
    left_arm_observer_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
      obj->JointInternal.MotionSubspace->size[1] - 1;
    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af <=
         left_arm_observer_B.b_k; left_arm_observer_B.i_af++) {
      S->data[left_arm_observer_B.i_af] = obj->
        JointInternal.MotionSubspace->data[left_arm_observer_B.i_af];
    }

    left_arm_observer_B.a_idx_0 = robot->
      PositionDoFMap[left_arm_observer_B.unnamed_idx_1];
    left_arm_observer_B.a_idx_1 = robot->
      PositionDoFMap[left_arm_observer_B.unnamed_idx_1 + 10];
    left_arm_observer_B.b_idx_0 = robot->
      VelocityDoFMap[left_arm_observer_B.unnamed_idx_1];
    left_arm_observer_B.b_idx_1 = robot->
      VelocityDoFMap[left_arm_observer_B.unnamed_idx_1 + 10];
    if (left_arm_observer_B.a_idx_1 < left_arm_observer_B.a_idx_0) {
      obj = robot->Bodies[left_arm_observer_B.unnamed_idx_1];
      rigidBodyJoint_transformBodyToP(&obj->JointInternal, left_arm_observer_B.T);
      left_arm_observer_B.i_af = qddoti->size[0];
      qddoti->size[0] = 1;
      left_a_emxEnsureCapacity_real_T(qddoti, left_arm_observer_B.i_af);
      qddoti->data[0] = 0.0;
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        vJ->data[left_arm_observer_B.i_af + 6 *
          left_arm_observer_B.unnamed_idx_1] = 0.0;
      }
    } else {
      if (left_arm_observer_B.a_idx_0 > left_arm_observer_B.a_idx_1) {
        left_arm_observer_B.inner = 0;
        left_arm_observer_B.m = -1;
      } else {
        left_arm_observer_B.inner = static_cast<int32_T>
          (left_arm_observer_B.a_idx_0) - 1;
        left_arm_observer_B.m = static_cast<int32_T>(left_arm_observer_B.a_idx_1)
          - 1;
      }

      if (left_arm_observer_B.b_idx_0 > left_arm_observer_B.b_idx_1) {
        left_arm_observer_B.p_tmp = 0;
        left_arm_observer_B.o_tmp = 0;
        left_arm_observer_B.aoffset = 0;
        left_arm_observer_B.b_k = -1;
      } else {
        left_arm_observer_B.p_tmp = static_cast<int32_T>
          (left_arm_observer_B.b_idx_0) - 1;
        left_arm_observer_B.o_tmp = static_cast<int32_T>
          (left_arm_observer_B.b_idx_1);
        left_arm_observer_B.aoffset = left_arm_observer_B.p_tmp;
        left_arm_observer_B.b_k = left_arm_observer_B.o_tmp - 1;
      }

      left_arm_observer_B.i_af = qddoti->size[0];
      left_arm_observer_B.b_k -= left_arm_observer_B.aoffset;
      qddoti->size[0] = left_arm_observer_B.b_k + 1;
      left_a_emxEnsureCapacity_real_T(qddoti, left_arm_observer_B.i_af);
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af <=
           left_arm_observer_B.b_k; left_arm_observer_B.i_af++) {
        qddoti->data[left_arm_observer_B.i_af] = qddot->
          data[left_arm_observer_B.aoffset + left_arm_observer_B.i_af];
      }

      obj = robot->Bodies[left_arm_observer_B.unnamed_idx_1];
      left_arm_observer_B.m -= left_arm_observer_B.inner;
      left_arm_observer_B.q_size = left_arm_observer_B.m + 1;
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af <=
           left_arm_observer_B.m; left_arm_observer_B.i_af++) {
        left_arm_observer_B.q_data[left_arm_observer_B.i_af] =
          q[left_arm_observer_B.inner + left_arm_observer_B.i_af];
      }

      rigidBodyJoint_transformBodyT_i(&obj->JointInternal,
        left_arm_observer_B.q_data, &left_arm_observer_B.q_size,
        left_arm_observer_B.T);
      if ((S->size[1] == 1) || (left_arm_observer_B.o_tmp -
           left_arm_observer_B.p_tmp == 1)) {
        for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
             left_arm_observer_B.i_af++) {
          left_arm_observer_B.aoffset = left_arm_observer_B.i_af + 6 *
            left_arm_observer_B.unnamed_idx_1;
          vJ->data[left_arm_observer_B.aoffset] = 0.0;
          left_arm_observer_B.b_k = S->size[1];
          for (left_arm_observer_B.inner = 0; left_arm_observer_B.inner <
               left_arm_observer_B.b_k; left_arm_observer_B.inner++) {
            vJ->data[left_arm_observer_B.aoffset] += S->data[6 *
              left_arm_observer_B.inner + left_arm_observer_B.i_af] *
              qdot[left_arm_observer_B.p_tmp + left_arm_observer_B.inner];
          }
        }
      } else {
        left_arm_observer_B.inner = S->size[1] - 1;
        for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
             left_arm_observer_B.i_af++) {
          vJ->data[left_arm_observer_B.i_af + 6 *
            left_arm_observer_B.unnamed_idx_1] = 0.0;
        }

        for (left_arm_observer_B.b_k = 0; left_arm_observer_B.b_k <=
             left_arm_observer_B.inner; left_arm_observer_B.b_k++) {
          left_arm_observer_B.aoffset = left_arm_observer_B.b_k * 6 - 1;
          for (left_arm_observer_B.c_i = 0; left_arm_observer_B.c_i < 6;
               left_arm_observer_B.c_i++) {
            left_arm_observer_B.i_af = 6 * left_arm_observer_B.unnamed_idx_1 +
              left_arm_observer_B.c_i;
            vJ->data[left_arm_observer_B.i_af] += S->data
              [(left_arm_observer_B.aoffset + left_arm_observer_B.c_i) + 1] *
              qdot[left_arm_observer_B.p_tmp + left_arm_observer_B.b_k];
          }
        }
      }
    }

    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 3;
         left_arm_observer_B.i_af++) {
      left_arm_observer_B.R_d[3 * left_arm_observer_B.i_af] =
        left_arm_observer_B.T[left_arm_observer_B.i_af];
      left_arm_observer_B.R_d[3 * left_arm_observer_B.i_af + 1] =
        left_arm_observer_B.T[left_arm_observer_B.i_af + 4];
      left_arm_observer_B.R_d[3 * left_arm_observer_B.i_af + 2] =
        left_arm_observer_B.T[left_arm_observer_B.i_af + 8];
    }

    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 9;
         left_arm_observer_B.i_af++) {
      left_arm_observer_B.R_g[left_arm_observer_B.i_af] =
        -left_arm_observer_B.R_d[left_arm_observer_B.i_af];
    }

    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 3;
         left_arm_observer_B.i_af++) {
      left_arm_observer_B.p_tmp = left_arm_observer_B.i_af << 2;
      left_arm_observer_B.Tinv[left_arm_observer_B.p_tmp] =
        left_arm_observer_B.R_d[3 * left_arm_observer_B.i_af];
      left_arm_observer_B.Tinv[left_arm_observer_B.p_tmp + 1] =
        left_arm_observer_B.R_d[3 * left_arm_observer_B.i_af + 1];
      left_arm_observer_B.Tinv[left_arm_observer_B.p_tmp + 2] =
        left_arm_observer_B.R_d[3 * left_arm_observer_B.i_af + 2];
      left_arm_observer_B.Tinv[left_arm_observer_B.i_af + 12] =
        left_arm_observer_B.R_g[left_arm_observer_B.i_af + 6] *
        left_arm_observer_B.T[14] +
        (left_arm_observer_B.R_g[left_arm_observer_B.i_af + 3] *
         left_arm_observer_B.T[13] +
         left_arm_observer_B.R_g[left_arm_observer_B.i_af] *
         left_arm_observer_B.T[12]);
    }

    left_arm_observer_B.Tinv[3] = 0.0;
    left_arm_observer_B.Tinv[7] = 0.0;
    left_arm_observer_B.Tinv[11] = 0.0;
    left_arm_observer_B.Tinv[15] = 1.0;
    left_arm_observer_B.dv2[3] = -left_arm_observer_B.Tinv[14];
    left_arm_observer_B.dv2[6] = left_arm_observer_B.Tinv[13];
    left_arm_observer_B.dv2[1] = left_arm_observer_B.Tinv[14];
    left_arm_observer_B.dv2[7] = -left_arm_observer_B.Tinv[12];
    left_arm_observer_B.dv2[2] = -left_arm_observer_B.Tinv[13];
    left_arm_observer_B.dv2[5] = left_arm_observer_B.Tinv[12];
    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 3;
         left_arm_observer_B.i_af++) {
      for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 3;
           left_arm_observer_B.aoffset++) {
        left_arm_observer_B.inner = left_arm_observer_B.i_af + 3 *
          left_arm_observer_B.aoffset;
        left_arm_observer_B.dv3[left_arm_observer_B.inner] = 0.0;
        left_arm_observer_B.p_tmp = left_arm_observer_B.aoffset << 2;
        left_arm_observer_B.dv3[left_arm_observer_B.inner] +=
          left_arm_observer_B.Tinv[left_arm_observer_B.p_tmp] *
          left_arm_observer_B.dv2[left_arm_observer_B.i_af];
        left_arm_observer_B.dv3[left_arm_observer_B.inner] +=
          left_arm_observer_B.Tinv[left_arm_observer_B.p_tmp + 1] *
          left_arm_observer_B.dv2[left_arm_observer_B.i_af + 3];
        left_arm_observer_B.dv3[left_arm_observer_B.inner] +=
          left_arm_observer_B.Tinv[left_arm_observer_B.p_tmp + 2] *
          left_arm_observer_B.dv2[left_arm_observer_B.i_af + 6];
        X->data[left_arm_observer_B.unnamed_idx_1]
          .f1[left_arm_observer_B.aoffset + 6 * left_arm_observer_B.i_af] =
          left_arm_observer_B.Tinv[(left_arm_observer_B.i_af << 2) +
          left_arm_observer_B.aoffset];
        X->data[left_arm_observer_B.unnamed_idx_1]
          .f1[left_arm_observer_B.aoffset + 6 * (left_arm_observer_B.i_af + 3)] =
          0.0;
      }
    }

    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 3;
         left_arm_observer_B.i_af++) {
      X->data[left_arm_observer_B.unnamed_idx_1].f1[6 * left_arm_observer_B.i_af
        + 3] = left_arm_observer_B.dv3[3 * left_arm_observer_B.i_af];
      left_arm_observer_B.aoffset = left_arm_observer_B.i_af << 2;
      left_arm_observer_B.inner = 6 * (left_arm_observer_B.i_af + 3);
      X->data[left_arm_observer_B.unnamed_idx_1].f1[left_arm_observer_B.inner +
        3] = left_arm_observer_B.Tinv[left_arm_observer_B.aoffset];
      X->data[left_arm_observer_B.unnamed_idx_1].f1[6 * left_arm_observer_B.i_af
        + 4] = left_arm_observer_B.dv3[3 * left_arm_observer_B.i_af + 1];
      X->data[left_arm_observer_B.unnamed_idx_1].f1[left_arm_observer_B.inner +
        4] = left_arm_observer_B.Tinv[left_arm_observer_B.aoffset + 1];
      X->data[left_arm_observer_B.unnamed_idx_1].f1[6 * left_arm_observer_B.i_af
        + 5] = left_arm_observer_B.dv3[3 * left_arm_observer_B.i_af + 2];
      X->data[left_arm_observer_B.unnamed_idx_1].f1[left_arm_observer_B.inner +
        5] = left_arm_observer_B.Tinv[left_arm_observer_B.aoffset + 2];
    }

    left_arm_observer_B.a_idx_0 = robot->
      Bodies[left_arm_observer_B.unnamed_idx_1]->ParentIndex;
    if (left_arm_observer_B.a_idx_0 > 0.0) {
      left_arm_observer_B.m = static_cast<int32_T>(left_arm_observer_B.a_idx_0);
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        left_arm_observer_B.a_idx_1 = 0.0;
        for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 6;
             left_arm_observer_B.aoffset++) {
          left_arm_observer_B.a_idx_1 += vB->data[(left_arm_observer_B.m - 1) *
            6 + left_arm_observer_B.aoffset] * X->
            data[left_arm_observer_B.unnamed_idx_1].f1[6 *
            left_arm_observer_B.aoffset + left_arm_observer_B.i_af];
        }

        left_arm_observer_B.vJ[left_arm_observer_B.i_af] = vJ->data[6 *
          left_arm_observer_B.unnamed_idx_1 + left_arm_observer_B.i_af] +
          left_arm_observer_B.a_idx_1;
      }

      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        vB->data[left_arm_observer_B.i_af + 6 *
          left_arm_observer_B.unnamed_idx_1] =
          left_arm_observer_B.vJ[left_arm_observer_B.i_af];
      }

      if ((S->size[1] == 1) || (qddoti->size[0] == 1)) {
        left_arm_observer_B.b_k = S->size[1];
        for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
             left_arm_observer_B.i_af++) {
          left_arm_observer_B.y[left_arm_observer_B.i_af] = 0.0;
          for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset <
               left_arm_observer_B.b_k; left_arm_observer_B.aoffset++) {
            left_arm_observer_B.a_idx_1 = S->data[6 *
              left_arm_observer_B.aoffset + left_arm_observer_B.i_af] *
              qddoti->data[left_arm_observer_B.aoffset] +
              left_arm_observer_B.y[left_arm_observer_B.i_af];
            left_arm_observer_B.y[left_arm_observer_B.i_af] =
              left_arm_observer_B.a_idx_1;
          }
        }
      } else {
        left_arm_observer_B.inner = S->size[1] - 1;
        for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
             left_arm_observer_B.i_af++) {
          left_arm_observer_B.y[left_arm_observer_B.i_af] = 0.0;
        }

        for (left_arm_observer_B.b_k = 0; left_arm_observer_B.b_k <=
             left_arm_observer_B.inner; left_arm_observer_B.b_k++) {
          left_arm_observer_B.aoffset = left_arm_observer_B.b_k * 6 - 1;
          for (left_arm_observer_B.c_i = 0; left_arm_observer_B.c_i < 6;
               left_arm_observer_B.c_i++) {
            left_arm_observer_B.a_idx_1 = S->data[(left_arm_observer_B.aoffset +
              left_arm_observer_B.c_i) + 1] * qddoti->
              data[left_arm_observer_B.b_k] +
              left_arm_observer_B.y[left_arm_observer_B.c_i];
            left_arm_observer_B.y[left_arm_observer_B.c_i] =
              left_arm_observer_B.a_idx_1;
          }
        }
      }

      left_arm_observer_B.R_d[0] = 0.0;
      left_arm_observer_B.p_tmp = 6 * left_arm_observer_B.unnamed_idx_1 + 2;
      left_arm_observer_B.R_d[3] = -vB->data[left_arm_observer_B.p_tmp];
      left_arm_observer_B.i_af = 6 * left_arm_observer_B.unnamed_idx_1 + 1;
      left_arm_observer_B.R_d[6] = vB->data[left_arm_observer_B.i_af];
      left_arm_observer_B.R_d[1] = vB->data[left_arm_observer_B.p_tmp];
      left_arm_observer_B.R_d[4] = 0.0;
      left_arm_observer_B.R_d[7] = -vB->data[6 *
        left_arm_observer_B.unnamed_idx_1];
      left_arm_observer_B.R_d[2] = -vB->data[left_arm_observer_B.i_af];
      left_arm_observer_B.R_d[5] = vB->data[6 *
        left_arm_observer_B.unnamed_idx_1];
      left_arm_observer_B.R_d[8] = 0.0;
      left_arm_observer_B.b_I[3] = 0.0;
      left_arm_observer_B.p_tmp = 6 * left_arm_observer_B.unnamed_idx_1 + 5;
      left_arm_observer_B.b_I[9] = -vB->data[left_arm_observer_B.p_tmp];
      left_arm_observer_B.i_af = 6 * left_arm_observer_B.unnamed_idx_1 + 4;
      left_arm_observer_B.b_I[15] = vB->data[left_arm_observer_B.i_af];
      left_arm_observer_B.b_I[4] = vB->data[left_arm_observer_B.p_tmp];
      left_arm_observer_B.b_I[10] = 0.0;
      left_arm_observer_B.p_tmp = 6 * left_arm_observer_B.unnamed_idx_1 + 3;
      left_arm_observer_B.b_I[16] = -vB->data[left_arm_observer_B.p_tmp];
      left_arm_observer_B.b_I[5] = -vB->data[left_arm_observer_B.i_af];
      left_arm_observer_B.b_I[11] = vB->data[left_arm_observer_B.p_tmp];
      left_arm_observer_B.b_I[17] = 0.0;
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 3;
           left_arm_observer_B.i_af++) {
        left_arm_observer_B.a_idx_1 = left_arm_observer_B.R_d[3 *
          left_arm_observer_B.i_af];
        left_arm_observer_B.b_I[6 * left_arm_observer_B.i_af] =
          left_arm_observer_B.a_idx_1;
        left_arm_observer_B.p_tmp = 6 * (left_arm_observer_B.i_af + 3);
        left_arm_observer_B.b_I[left_arm_observer_B.p_tmp] = 0.0;
        left_arm_observer_B.b_I[left_arm_observer_B.p_tmp + 3] =
          left_arm_observer_B.a_idx_1;
        left_arm_observer_B.a_idx_1 = left_arm_observer_B.R_d[3 *
          left_arm_observer_B.i_af + 1];
        left_arm_observer_B.b_I[6 * left_arm_observer_B.i_af + 1] =
          left_arm_observer_B.a_idx_1;
        left_arm_observer_B.b_I[left_arm_observer_B.p_tmp + 1] = 0.0;
        left_arm_observer_B.b_I[left_arm_observer_B.p_tmp + 4] =
          left_arm_observer_B.a_idx_1;
        left_arm_observer_B.a_idx_1 = left_arm_observer_B.R_d[3 *
          left_arm_observer_B.i_af + 2];
        left_arm_observer_B.b_I[6 * left_arm_observer_B.i_af + 2] =
          left_arm_observer_B.a_idx_1;
        left_arm_observer_B.b_I[left_arm_observer_B.p_tmp + 2] = 0.0;
        left_arm_observer_B.b_I[left_arm_observer_B.p_tmp + 5] =
          left_arm_observer_B.a_idx_1;
      }

      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        left_arm_observer_B.a_idx_1 = 0.0;
        for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 6;
             left_arm_observer_B.aoffset++) {
          left_arm_observer_B.a_idx_1 += aB->data[(left_arm_observer_B.m - 1) *
            6 + left_arm_observer_B.aoffset] * X->
            data[left_arm_observer_B.unnamed_idx_1].f1[6 *
            left_arm_observer_B.aoffset + left_arm_observer_B.i_af];
        }

        left_arm_observer_B.vJ[left_arm_observer_B.i_af] =
          left_arm_observer_B.a_idx_1 +
          left_arm_observer_B.y[left_arm_observer_B.i_af];
      }

      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        left_arm_observer_B.y[left_arm_observer_B.i_af] = 0.0;
        for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 6;
             left_arm_observer_B.aoffset++) {
          left_arm_observer_B.a_idx_1 = left_arm_observer_B.b_I[6 *
            left_arm_observer_B.aoffset + left_arm_observer_B.i_af] * vJ->data[6
            * left_arm_observer_B.unnamed_idx_1 + left_arm_observer_B.aoffset] +
            left_arm_observer_B.y[left_arm_observer_B.i_af];
          left_arm_observer_B.y[left_arm_observer_B.i_af] =
            left_arm_observer_B.a_idx_1;
        }

        aB->data[left_arm_observer_B.i_af + 6 *
          left_arm_observer_B.unnamed_idx_1] =
          left_arm_observer_B.vJ[left_arm_observer_B.i_af] +
          left_arm_observer_B.y[left_arm_observer_B.i_af];
      }

      left_arm_observer_B.R_g[0] = 0.0;
      left_arm_observer_B.R_g[3] = -left_arm_observer_B.T[14];
      left_arm_observer_B.R_g[6] = left_arm_observer_B.T[13];
      left_arm_observer_B.R_g[1] = left_arm_observer_B.T[14];
      left_arm_observer_B.R_g[4] = 0.0;
      left_arm_observer_B.R_g[7] = -left_arm_observer_B.T[12];
      left_arm_observer_B.R_g[2] = -left_arm_observer_B.T[13];
      left_arm_observer_B.R_g[5] = left_arm_observer_B.T[12];
      left_arm_observer_B.R_g[8] = 0.0;
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 3;
           left_arm_observer_B.i_af++) {
        for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 3;
             left_arm_observer_B.aoffset++) {
          left_arm_observer_B.inner = left_arm_observer_B.i_af + 3 *
            left_arm_observer_B.aoffset;
          left_arm_observer_B.dv4[left_arm_observer_B.inner] = 0.0;
          left_arm_observer_B.p_tmp = left_arm_observer_B.aoffset << 2;
          left_arm_observer_B.dv4[left_arm_observer_B.inner] +=
            left_arm_observer_B.T[left_arm_observer_B.p_tmp] *
            left_arm_observer_B.R_g[left_arm_observer_B.i_af];
          left_arm_observer_B.dv4[left_arm_observer_B.inner] +=
            left_arm_observer_B.T[left_arm_observer_B.p_tmp + 1] *
            left_arm_observer_B.R_g[left_arm_observer_B.i_af + 3];
          left_arm_observer_B.dv4[left_arm_observer_B.inner] +=
            left_arm_observer_B.T[left_arm_observer_B.p_tmp + 2] *
            left_arm_observer_B.R_g[left_arm_observer_B.i_af + 6];
          left_arm_observer_B.b_I[left_arm_observer_B.aoffset + 6 *
            left_arm_observer_B.i_af] = left_arm_observer_B.T
            [(left_arm_observer_B.i_af << 2) + left_arm_observer_B.aoffset];
          left_arm_observer_B.b_I[left_arm_observer_B.aoffset + 6 *
            (left_arm_observer_B.i_af + 3)] = 0.0;
        }
      }

      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 3;
           left_arm_observer_B.i_af++) {
        left_arm_observer_B.b_I[6 * left_arm_observer_B.i_af + 3] =
          left_arm_observer_B.dv4[3 * left_arm_observer_B.i_af];
        left_arm_observer_B.p_tmp = left_arm_observer_B.i_af << 2;
        left_arm_observer_B.inner = 6 * (left_arm_observer_B.i_af + 3);
        left_arm_observer_B.b_I[left_arm_observer_B.inner + 3] =
          left_arm_observer_B.T[left_arm_observer_B.p_tmp];
        left_arm_observer_B.b_I[6 * left_arm_observer_B.i_af + 4] =
          left_arm_observer_B.dv4[3 * left_arm_observer_B.i_af + 1];
        left_arm_observer_B.b_I[left_arm_observer_B.inner + 4] =
          left_arm_observer_B.T[left_arm_observer_B.p_tmp + 1];
        left_arm_observer_B.b_I[6 * left_arm_observer_B.i_af + 5] =
          left_arm_observer_B.dv4[3 * left_arm_observer_B.i_af + 2];
        left_arm_observer_B.b_I[left_arm_observer_B.inner + 5] =
          left_arm_observer_B.T[left_arm_observer_B.p_tmp + 2];
      }

      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 6;
             left_arm_observer_B.aoffset++) {
          left_arm_observer_B.p_tmp = left_arm_observer_B.i_af + 6 *
            left_arm_observer_B.aoffset;
          left_arm_observer_B.Xtree[left_arm_observer_B.p_tmp] = 0.0;
          for (left_arm_observer_B.inner = 0; left_arm_observer_B.inner < 6;
               left_arm_observer_B.inner++) {
            left_arm_observer_B.Xtree[left_arm_observer_B.p_tmp] += Xtree->data[
              static_cast<int32_T>(left_arm_observer_B.a_idx_0) - 1].f1[6 *
              left_arm_observer_B.inner + left_arm_observer_B.i_af] *
              left_arm_observer_B.b_I[6 * left_arm_observer_B.aoffset +
              left_arm_observer_B.inner];
          }
        }
      }

      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 36;
           left_arm_observer_B.i_af++) {
        Xtree->data[left_arm_observer_B.unnamed_idx_1]
          .f1[left_arm_observer_B.i_af] =
          left_arm_observer_B.Xtree[left_arm_observer_B.i_af];
      }
    } else {
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        left_arm_observer_B.aoffset = 6 * left_arm_observer_B.unnamed_idx_1 +
          left_arm_observer_B.i_af;
        vB->data[left_arm_observer_B.aoffset] = vJ->
          data[left_arm_observer_B.aoffset];
      }

      if ((S->size[1] == 1) || (qddoti->size[0] == 1)) {
        left_arm_observer_B.b_k = S->size[1];
        for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
             left_arm_observer_B.i_af++) {
          left_arm_observer_B.y[left_arm_observer_B.i_af] = 0.0;
          for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset <
               left_arm_observer_B.b_k; left_arm_observer_B.aoffset++) {
            left_arm_observer_B.a_idx_1 = S->data[6 *
              left_arm_observer_B.aoffset + left_arm_observer_B.i_af] *
              qddoti->data[left_arm_observer_B.aoffset] +
              left_arm_observer_B.y[left_arm_observer_B.i_af];
            left_arm_observer_B.y[left_arm_observer_B.i_af] =
              left_arm_observer_B.a_idx_1;
          }
        }
      } else {
        left_arm_observer_B.inner = S->size[1] - 1;
        for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
             left_arm_observer_B.i_af++) {
          left_arm_observer_B.y[left_arm_observer_B.i_af] = 0.0;
        }

        for (left_arm_observer_B.b_k = 0; left_arm_observer_B.b_k <=
             left_arm_observer_B.inner; left_arm_observer_B.b_k++) {
          left_arm_observer_B.aoffset = left_arm_observer_B.b_k * 6 - 1;
          for (left_arm_observer_B.c_i = 0; left_arm_observer_B.c_i < 6;
               left_arm_observer_B.c_i++) {
            left_arm_observer_B.a_idx_1 = S->data[(left_arm_observer_B.aoffset +
              left_arm_observer_B.c_i) + 1] * qddoti->
              data[left_arm_observer_B.b_k] +
              left_arm_observer_B.y[left_arm_observer_B.c_i];
            left_arm_observer_B.y[left_arm_observer_B.c_i] =
              left_arm_observer_B.a_idx_1;
          }
        }
      }

      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        left_arm_observer_B.a_idx_1 = 0.0;
        for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 6;
             left_arm_observer_B.aoffset++) {
          left_arm_observer_B.a_idx_1 += X->
            data[left_arm_observer_B.unnamed_idx_1].f1[6 *
            left_arm_observer_B.aoffset + left_arm_observer_B.i_af] *
            left_arm_observer_B.a0[left_arm_observer_B.aoffset];
        }

        aB->data[left_arm_observer_B.i_af + 6 *
          left_arm_observer_B.unnamed_idx_1] = left_arm_observer_B.a_idx_1 +
          left_arm_observer_B.y[left_arm_observer_B.i_af];
      }

      left_arm_observer_B.R_g[0] = 0.0;
      left_arm_observer_B.R_g[3] = -left_arm_observer_B.T[14];
      left_arm_observer_B.R_g[6] = left_arm_observer_B.T[13];
      left_arm_observer_B.R_g[1] = left_arm_observer_B.T[14];
      left_arm_observer_B.R_g[4] = 0.0;
      left_arm_observer_B.R_g[7] = -left_arm_observer_B.T[12];
      left_arm_observer_B.R_g[2] = -left_arm_observer_B.T[13];
      left_arm_observer_B.R_g[5] = left_arm_observer_B.T[12];
      left_arm_observer_B.R_g[8] = 0.0;
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 3;
           left_arm_observer_B.i_af++) {
        for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 3;
             left_arm_observer_B.aoffset++) {
          left_arm_observer_B.inner = left_arm_observer_B.i_af + 3 *
            left_arm_observer_B.aoffset;
          left_arm_observer_B.dv4[left_arm_observer_B.inner] = 0.0;
          left_arm_observer_B.p_tmp = left_arm_observer_B.aoffset << 2;
          left_arm_observer_B.dv4[left_arm_observer_B.inner] +=
            left_arm_observer_B.T[left_arm_observer_B.p_tmp] *
            left_arm_observer_B.R_g[left_arm_observer_B.i_af];
          left_arm_observer_B.dv4[left_arm_observer_B.inner] +=
            left_arm_observer_B.T[left_arm_observer_B.p_tmp + 1] *
            left_arm_observer_B.R_g[left_arm_observer_B.i_af + 3];
          left_arm_observer_B.dv4[left_arm_observer_B.inner] +=
            left_arm_observer_B.T[left_arm_observer_B.p_tmp + 2] *
            left_arm_observer_B.R_g[left_arm_observer_B.i_af + 6];
          Xtree->data[left_arm_observer_B.unnamed_idx_1]
            .f1[left_arm_observer_B.aoffset + 6 * left_arm_observer_B.i_af] =
            left_arm_observer_B.T[(left_arm_observer_B.i_af << 2) +
            left_arm_observer_B.aoffset];
          Xtree->data[left_arm_observer_B.unnamed_idx_1]
            .f1[left_arm_observer_B.aoffset + 6 * (left_arm_observer_B.i_af + 3)]
            = 0.0;
        }
      }

      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 3;
           left_arm_observer_B.i_af++) {
        Xtree->data[left_arm_observer_B.unnamed_idx_1].f1[6 *
          left_arm_observer_B.i_af + 3] = left_arm_observer_B.dv4[3 *
          left_arm_observer_B.i_af];
        left_arm_observer_B.aoffset = left_arm_observer_B.i_af << 2;
        left_arm_observer_B.inner = 6 * (left_arm_observer_B.i_af + 3);
        Xtree->data[left_arm_observer_B.unnamed_idx_1]
          .f1[left_arm_observer_B.inner + 3] =
          left_arm_observer_B.T[left_arm_observer_B.aoffset];
        Xtree->data[left_arm_observer_B.unnamed_idx_1].f1[6 *
          left_arm_observer_B.i_af + 4] = left_arm_observer_B.dv4[3 *
          left_arm_observer_B.i_af + 1];
        Xtree->data[left_arm_observer_B.unnamed_idx_1]
          .f1[left_arm_observer_B.inner + 4] =
          left_arm_observer_B.T[left_arm_observer_B.aoffset + 1];
        Xtree->data[left_arm_observer_B.unnamed_idx_1].f1[6 *
          left_arm_observer_B.i_af + 5] = left_arm_observer_B.dv4[3 *
          left_arm_observer_B.i_af + 2];
        Xtree->data[left_arm_observer_B.unnamed_idx_1]
          .f1[left_arm_observer_B.inner + 5] =
          left_arm_observer_B.T[left_arm_observer_B.aoffset + 2];
      }
    }

    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 36;
         left_arm_observer_B.i_af++) {
      left_arm_observer_B.b_I[left_arm_observer_B.i_af] = robot->
        Bodies[left_arm_observer_B.unnamed_idx_1]->
        SpatialInertia[left_arm_observer_B.i_af];
    }

    left_arm_observer_B.R_d[0] = 0.0;
    left_arm_observer_B.p_tmp = 6 * left_arm_observer_B.unnamed_idx_1 + 2;
    left_arm_observer_B.R_d[3] = -vB->data[left_arm_observer_B.p_tmp];
    left_arm_observer_B.i_af = 6 * left_arm_observer_B.unnamed_idx_1 + 1;
    left_arm_observer_B.R_d[6] = vB->data[left_arm_observer_B.i_af];
    left_arm_observer_B.R_d[1] = vB->data[left_arm_observer_B.p_tmp];
    left_arm_observer_B.R_d[4] = 0.0;
    left_arm_observer_B.R_d[7] = -vB->data[6 * left_arm_observer_B.unnamed_idx_1];
    left_arm_observer_B.R_d[2] = -vB->data[left_arm_observer_B.i_af];
    left_arm_observer_B.R_d[5] = vB->data[6 * left_arm_observer_B.unnamed_idx_1];
    left_arm_observer_B.R_d[8] = 0.0;
    left_arm_observer_B.R[18] = 0.0;
    left_arm_observer_B.p_tmp = 6 * left_arm_observer_B.unnamed_idx_1 + 5;
    left_arm_observer_B.R[24] = -vB->data[left_arm_observer_B.p_tmp];
    left_arm_observer_B.i_af = 6 * left_arm_observer_B.unnamed_idx_1 + 4;
    left_arm_observer_B.R[30] = vB->data[left_arm_observer_B.i_af];
    left_arm_observer_B.R[19] = vB->data[left_arm_observer_B.p_tmp];
    left_arm_observer_B.R[25] = 0.0;
    left_arm_observer_B.p_tmp = 6 * left_arm_observer_B.unnamed_idx_1 + 3;
    left_arm_observer_B.R[31] = -vB->data[left_arm_observer_B.p_tmp];
    left_arm_observer_B.R[20] = -vB->data[left_arm_observer_B.i_af];
    left_arm_observer_B.R[26] = vB->data[left_arm_observer_B.p_tmp];
    left_arm_observer_B.R[32] = 0.0;
    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 3;
         left_arm_observer_B.i_af++) {
      left_arm_observer_B.a_idx_1 = left_arm_observer_B.R_d[3 *
        left_arm_observer_B.i_af];
      left_arm_observer_B.R[6 * left_arm_observer_B.i_af] =
        left_arm_observer_B.a_idx_1;
      left_arm_observer_B.R[6 * left_arm_observer_B.i_af + 3] = 0.0;
      left_arm_observer_B.p_tmp = 6 * (left_arm_observer_B.i_af + 3);
      left_arm_observer_B.R[left_arm_observer_B.p_tmp + 3] =
        left_arm_observer_B.a_idx_1;
      left_arm_observer_B.a_idx_1 = left_arm_observer_B.R_d[3 *
        left_arm_observer_B.i_af + 1];
      left_arm_observer_B.R[6 * left_arm_observer_B.i_af + 1] =
        left_arm_observer_B.a_idx_1;
      left_arm_observer_B.R[6 * left_arm_observer_B.i_af + 4] = 0.0;
      left_arm_observer_B.R[left_arm_observer_B.p_tmp + 4] =
        left_arm_observer_B.a_idx_1;
      left_arm_observer_B.a_idx_1 = left_arm_observer_B.R_d[3 *
        left_arm_observer_B.i_af + 2];
      left_arm_observer_B.R[6 * left_arm_observer_B.i_af + 2] =
        left_arm_observer_B.a_idx_1;
      left_arm_observer_B.R[6 * left_arm_observer_B.i_af + 5] = 0.0;
      left_arm_observer_B.R[left_arm_observer_B.p_tmp + 5] =
        left_arm_observer_B.a_idx_1;
    }

    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
         left_arm_observer_B.i_af++) {
      left_arm_observer_B.b_I_n[left_arm_observer_B.i_af] = 0.0;
      left_arm_observer_B.b_I_i[left_arm_observer_B.i_af] = 0.0;
      for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 6;
           left_arm_observer_B.aoffset++) {
        left_arm_observer_B.a_idx_0 = left_arm_observer_B.b_I[6 *
          left_arm_observer_B.aoffset + left_arm_observer_B.i_af];
        left_arm_observer_B.p_tmp = 6 * left_arm_observer_B.unnamed_idx_1 +
          left_arm_observer_B.aoffset;
        left_arm_observer_B.a_idx_1 = vB->data[left_arm_observer_B.p_tmp] *
          left_arm_observer_B.a_idx_0 +
          left_arm_observer_B.b_I_n[left_arm_observer_B.i_af];
        left_arm_observer_B.a_idx_0 = aB->data[left_arm_observer_B.p_tmp] *
          left_arm_observer_B.a_idx_0 +
          left_arm_observer_B.b_I_i[left_arm_observer_B.i_af];
        left_arm_observer_B.b_I_n[left_arm_observer_B.i_af] =
          left_arm_observer_B.a_idx_1;
        left_arm_observer_B.b_I_i[left_arm_observer_B.i_af] =
          left_arm_observer_B.a_idx_0;
      }
    }

    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
         left_arm_observer_B.i_af++) {
      left_arm_observer_B.R_oy[left_arm_observer_B.i_af] = 0.0;
      left_arm_observer_B.a_idx_1 = 0.0;
      for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 6;
           left_arm_observer_B.aoffset++) {
        left_arm_observer_B.a_idx_1 += Xtree->
          data[left_arm_observer_B.unnamed_idx_1].f1[6 *
          left_arm_observer_B.i_af + left_arm_observer_B.aoffset] * fext[6 *
          left_arm_observer_B.unnamed_idx_1 + left_arm_observer_B.aoffset];
        left_arm_observer_B.R_oy[left_arm_observer_B.i_af] +=
          left_arm_observer_B.R[6 * left_arm_observer_B.aoffset +
          left_arm_observer_B.i_af] *
          left_arm_observer_B.b_I_n[left_arm_observer_B.aoffset];
      }

      f->data[left_arm_observer_B.i_af + 6 * left_arm_observer_B.unnamed_idx_1] =
        (left_arm_observer_B.b_I_i[left_arm_observer_B.i_af] +
         left_arm_observer_B.R_oy[left_arm_observer_B.i_af]) -
        left_arm_observer_B.a_idx_1;
    }
  }

  left_arm_observe_emxFree_real_T(&aB);
  left_arm_observe_emxFree_real_T(&vB);
  left_arm_observe_emxFree_real_T(&vJ);
  left_arm_ob_emxFree_f_cell_wrap(&Xtree);
  left_arm_observer_B.loop_ub_tmp = static_cast<int32_T>(((-1.0 -
    left_arm_observer_B.nb) + 1.0) / -1.0) - 1;
  left_arm_observe_emxInit_char_T(&a, 2);
  left_arm_observe_emxInit_real_T(&a_0, 2);
  if (0 <= left_arm_observer_B.loop_ub_tmp) {
    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 5;
         left_arm_observer_B.i_af++) {
      left_arm_observer_B.b_a[left_arm_observer_B.i_af] =
        tmp[left_arm_observer_B.i_af];
    }
  }

  for (left_arm_observer_B.p_tmp = 0; left_arm_observer_B.p_tmp <=
       left_arm_observer_B.loop_ub_tmp; left_arm_observer_B.p_tmp++) {
    left_arm_observer_B.a_idx_0 = left_arm_observer_B.nb + -static_cast<real_T>
      (left_arm_observer_B.p_tmp);
    left_arm_observer_B.inner = static_cast<int32_T>(left_arm_observer_B.a_idx_0);
    left_arm_observer_B.o_tmp = left_arm_observer_B.inner - 1;
    obj = robot->Bodies[left_arm_observer_B.o_tmp];
    left_arm_observer_B.i_af = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = obj->JointInternal.Type->size[1];
    left_a_emxEnsureCapacity_char_T(a, left_arm_observer_B.i_af);
    left_arm_observer_B.b_k = obj->JointInternal.Type->size[0] *
      obj->JointInternal.Type->size[1] - 1;
    for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af <=
         left_arm_observer_B.b_k; left_arm_observer_B.i_af++) {
      a->data[left_arm_observer_B.i_af] = obj->JointInternal.Type->
        data[left_arm_observer_B.i_af];
    }

    left_arm_observer_B.b_bool = false;
    if (a->size[1] == 5) {
      left_arm_observer_B.i_af = 1;
      do {
        exitg1 = 0;
        if (left_arm_observer_B.i_af - 1 < 5) {
          left_arm_observer_B.unnamed_idx_1 = left_arm_observer_B.i_af - 1;
          if (a->data[left_arm_observer_B.unnamed_idx_1] !=
              left_arm_observer_B.b_a[left_arm_observer_B.unnamed_idx_1]) {
            exitg1 = 1;
          } else {
            left_arm_observer_B.i_af++;
          }
        } else {
          left_arm_observer_B.b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!left_arm_observer_B.b_bool) {
      obj = robot->Bodies[left_arm_observer_B.o_tmp];
      left_arm_observer_B.i_af = S->size[0] * S->size[1];
      S->size[0] = 6;
      S->size[1] = obj->JointInternal.MotionSubspace->size[1];
      left_a_emxEnsureCapacity_real_T(S, left_arm_observer_B.i_af);
      left_arm_observer_B.b_k = obj->JointInternal.MotionSubspace->size[0] *
        obj->JointInternal.MotionSubspace->size[1] - 1;
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af <=
           left_arm_observer_B.b_k; left_arm_observer_B.i_af++) {
        S->data[left_arm_observer_B.i_af] = obj->
          JointInternal.MotionSubspace->data[left_arm_observer_B.i_af];
      }

      left_arm_observer_B.i_af = a_0->size[0] * a_0->size[1];
      a_0->size[0] = S->size[1];
      a_0->size[1] = 6;
      left_a_emxEnsureCapacity_real_T(a_0, left_arm_observer_B.i_af);
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        left_arm_observer_B.b_k = S->size[1];
        for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset <
             left_arm_observer_B.b_k; left_arm_observer_B.aoffset++) {
          a_0->data[left_arm_observer_B.aoffset + a_0->size[0] *
            left_arm_observer_B.i_af] = S->data[6 * left_arm_observer_B.aoffset
            + left_arm_observer_B.i_af];
        }
      }

      left_arm_observer_B.m = a_0->size[0] - 1;
      left_arm_observer_B.i_af = qddoti->size[0];
      qddoti->size[0] = a_0->size[0];
      left_a_emxEnsureCapacity_real_T(qddoti, left_arm_observer_B.i_af);
      for (left_arm_observer_B.unnamed_idx_1 = 0;
           left_arm_observer_B.unnamed_idx_1 <= left_arm_observer_B.m;
           left_arm_observer_B.unnamed_idx_1++) {
        qddoti->data[left_arm_observer_B.unnamed_idx_1] = 0.0;
      }

      for (left_arm_observer_B.b_k = 0; left_arm_observer_B.b_k < 6;
           left_arm_observer_B.b_k++) {
        left_arm_observer_B.aoffset = (left_arm_observer_B.m + 1) *
          left_arm_observer_B.b_k - 1;
        for (left_arm_observer_B.c_i = 0; left_arm_observer_B.c_i <=
             left_arm_observer_B.m; left_arm_observer_B.c_i++) {
          qddoti->data[left_arm_observer_B.c_i] += f->data[(static_cast<int32_T>
            (left_arm_observer_B.a_idx_0) - 1) * 6 + left_arm_observer_B.b_k] *
            a_0->data[(left_arm_observer_B.aoffset + left_arm_observer_B.c_i) +
            1];
        }
      }

      left_arm_observer_B.b_idx_0 = robot->
        VelocityDoFMap[left_arm_observer_B.inner - 1];
      left_arm_observer_B.b_idx_1 = robot->
        VelocityDoFMap[left_arm_observer_B.inner + 9];
      if (left_arm_observer_B.b_idx_0 > left_arm_observer_B.b_idx_1) {
        left_arm_observer_B.m = 0;
        left_arm_observer_B.i_af = 0;
      } else {
        left_arm_observer_B.m = static_cast<int32_T>(left_arm_observer_B.b_idx_0)
          - 1;
        left_arm_observer_B.i_af = static_cast<int32_T>
          (left_arm_observer_B.b_idx_1);
      }

      left_arm_observer_B.unnamed_idx_1 = left_arm_observer_B.i_af -
        left_arm_observer_B.m;
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af <
           left_arm_observer_B.unnamed_idx_1; left_arm_observer_B.i_af++) {
        tau[left_arm_observer_B.m + left_arm_observer_B.i_af] = qddoti->
          data[left_arm_observer_B.i_af];
      }
    }

    left_arm_observer_B.a_idx_0 = robot->Bodies[left_arm_observer_B.o_tmp]
      ->ParentIndex;
    if (left_arm_observer_B.a_idx_0 > 0.0) {
      left_arm_observer_B.m = static_cast<int32_T>(left_arm_observer_B.a_idx_0);
      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        left_arm_observer_B.a_idx_1 = 0.0;
        for (left_arm_observer_B.aoffset = 0; left_arm_observer_B.aoffset < 6;
             left_arm_observer_B.aoffset++) {
          left_arm_observer_B.a_idx_1 += f->data[(left_arm_observer_B.inner - 1)
            * 6 + left_arm_observer_B.aoffset] * X->
            data[left_arm_observer_B.o_tmp].f1[6 * left_arm_observer_B.i_af +
            left_arm_observer_B.aoffset];
        }

        left_arm_observer_B.a0[left_arm_observer_B.i_af] = f->data
          [(left_arm_observer_B.m - 1) * 6 + left_arm_observer_B.i_af] +
          left_arm_observer_B.a_idx_1;
      }

      for (left_arm_observer_B.i_af = 0; left_arm_observer_B.i_af < 6;
           left_arm_observer_B.i_af++) {
        f->data[left_arm_observer_B.i_af + 6 * (left_arm_observer_B.m - 1)] =
          left_arm_observer_B.a0[left_arm_observer_B.i_af];
      }
    }
  }

  left_arm_observe_emxFree_real_T(&a_0);
  left_arm_observe_emxFree_char_T(&a);
  left_arm_observe_emxFree_real_T(&qddoti);
  left_arm_observe_emxFree_real_T(&S);
  left_arm_observe_emxFree_real_T(&f);
  left_arm_ob_emxFree_f_cell_wrap(&X);
}

static void matlabCodegenHandle_matlabCo_ih(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void le_emxFreeStruct_rigidBodyJoint(rigidBodyJoint_left_arm_obser_T
  *pStruct)
{
  left_arm_observe_emxFree_char_T(&pStruct->Type);
  left_arm_observe_emxFree_real_T(&pStruct->MotionSubspace);
}

static void emxFreeStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct)
{
  le_emxFreeStruct_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal__ih_T
  *pStruct)
{
  emxFreeStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal__ih_T
  *pStruct)
{
  emxFreeStruct_k_robotics_manip_(&pStruct->TreeInternal);
}

static void emxFreeStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct)
{
  le_emxFreeStruct_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxFreeStruct_k_robotics_mani_i(k_robotics_manip_internal_R_i_T
  *pStruct)
{
  emxFreeStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slmani_i(robotics_slmanip_internal_b_i_T
  *pStruct)
{
  emxFreeStruct_k_robotics_mani_i(&pStruct->TreeInternal);
}

static void emxFreeStruct_k_robotics_man_ih(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxFreeStruct_robotics_slman_ih(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_k_robotics_man_ih(&pStruct->TreeInternal);
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void le_emxInitStruct_rigidBodyJoint(rigidBodyJoint_left_arm_obser_T
  *pStruct)
{
  left_arm_observe_emxInit_char_T(&pStruct->Type, 2);
  left_arm_observe_emxInit_real_T(&pStruct->MotionSubspace, 2);
}

static void emxInitStruct_i_robotics_manip_(i_robotics_manip_internal_Rig_T
  *pStruct)
{
  le_emxInitStruct_rigidBodyJoint(&pStruct->JointInternal);
}

static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal__ih_T
  *pStruct)
{
  emxInitStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal__ih_T
  *pStruct)
{
  emxInitStruct_k_robotics_manip_(&pStruct->TreeInternal);
}

static void emxInitStruct_j_robotics_manip_(j_robotics_manip_internal_Rig_T
  *pStruct)
{
  le_emxInitStruct_rigidBodyJoint(&pStruct->JointInternal);
}

static j_robotics_manip_internal_Rig_T *left_arm_ob_RigidBody_RigidBody
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
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

static j_robotics_manip_internal_Rig_T *left_arm__RigidBody_RigidBody_i
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
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

static j_robotics_manip_internal_Rig_T *left_arm_RigidBody_RigidBody_ih
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
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

static j_robotics_manip_internal_Rig_T *left_ar_RigidBody_RigidBody_ihp
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
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

static j_robotics_manip_internal_Rig_T *left_a_RigidBody_RigidBody_ihpp
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
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

static j_robotics_manip_internal_Rig_T *left__RigidBody_RigidBody_ihppq
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
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

static j_robotics_manip_internal_Rig_T *left_RigidBody_RigidBody_ihppqb
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
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

static j_robotics_manip_internal_Rig_T *lef_RigidBody_RigidBody_ihppqbe
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
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

static j_robotics_manip_internal_Rig_T *le_RigidBody_RigidBody_ihppqbei
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
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

static j_robotics_manip_internal_Rig_T *l_RigidBody_RigidBody_ihppqbeig
  (j_robotics_manip_internal_Rig_T *obj)
{
  j_robotics_manip_internal_Rig_T *b_obj;
  int8_T msubspace_data[36];
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
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

static i_robotics_manip_internal_Rig_T *RigidBody_RigidBody_ihppqbeig2
  (i_robotics_manip_internal_Rig_T *obj)
{
  i_robotics_manip_internal_Rig_T *b_obj;
  emxArray_char_T_left_arm_obse_T *switch_expression;
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

  left_arm_observe_emxInit_char_T(&switch_expression, 2);
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

  left_arm_observe_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      left_arm_observer_B.msubspace_data[b_kstr] = tmp[b_kstr];
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
      left_arm_observer_B.msubspace_data[b_kstr] = tmp[b_kstr];
    }

    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      left_arm_observer_B.msubspace_data[b_kstr] = 0;
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
      left_arm_observer_B.msubspace_data[b_kstr];
  }

  return b_obj;
}

static k_robotics_manip_internal__ih_T *RigidBodyTree_RigidBodyTree_ih
  (k_robotics_manip_internal__ih_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9)
{
  k_robotics_manip_internal__ih_T *b_obj;
  int32_T i;
  static const int8_T tmp[20] = { 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 1, 2, 3, 4, 5, 6,
    7, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = left_arm_ob_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = left_arm__RigidBody_RigidBody_i(iobj_1);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = left_arm_RigidBody_RigidBody_ih(iobj_2);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = left_ar_RigidBody_RigidBody_ihp(iobj_3);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = left_a_RigidBody_RigidBody_ihpp(iobj_4);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = left__RigidBody_RigidBody_ihppq(iobj_5);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = left_RigidBody_RigidBody_ihppqb(iobj_6);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = lef_RigidBody_RigidBody_ihppqbe(iobj_7);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = le_RigidBody_RigidBody_ihppqbei(iobj_8);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = l_RigidBody_RigidBody_ihppqbeig(iobj_9);
  obj->Bodies[9]->Index = 10.0;
  obj->NumBodies = 10.0;
  obj->VelocityNumber = 7.0;
  for (i = 0; i < 20; i++) {
    obj->PositionDoFMap[i] = tmp[i];
  }

  for (i = 0; i < 20; i++) {
    obj->VelocityDoFMap[i] = tmp[i];
  }

  RigidBody_RigidBody_ihppqbeig2(&obj->Base);
  return b_obj;
}

static void emxInitStruct_k_robotics_mani_i(k_robotics_manip_internal_R_i_T
  *pStruct)
{
  emxInitStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slmani_i(robotics_slmanip_internal_b_i_T
  *pStruct)
{
  emxInitStruct_k_robotics_mani_i(&pStruct->TreeInternal);
}

static k_robotics_manip_internal_R_i_T *l_RigidBodyTree_RigidBodyTree_i
  (k_robotics_manip_internal_R_i_T *obj, j_robotics_manip_internal_Rig_T *iobj_0,
   j_robotics_manip_internal_Rig_T *iobj_1, j_robotics_manip_internal_Rig_T
   *iobj_2, j_robotics_manip_internal_Rig_T *iobj_3,
   j_robotics_manip_internal_Rig_T *iobj_4, j_robotics_manip_internal_Rig_T
   *iobj_5, j_robotics_manip_internal_Rig_T *iobj_6,
   j_robotics_manip_internal_Rig_T *iobj_7, j_robotics_manip_internal_Rig_T
   *iobj_8, j_robotics_manip_internal_Rig_T *iobj_9)
{
  k_robotics_manip_internal_R_i_T *b_obj;
  int32_T i;
  static const int8_T tmp[20] = { 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 1, 2, 3, 4, 5, 6,
    7, -1, -1, -1 };

  b_obj = obj;
  obj->Bodies[0] = left_arm_ob_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = left_arm__RigidBody_RigidBody_i(iobj_1);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = left_arm_RigidBody_RigidBody_ih(iobj_2);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = left_ar_RigidBody_RigidBody_ihp(iobj_3);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = left_a_RigidBody_RigidBody_ihpp(iobj_4);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = left__RigidBody_RigidBody_ihppq(iobj_5);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = left_RigidBody_RigidBody_ihppqb(iobj_6);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = lef_RigidBody_RigidBody_ihppqbe(iobj_7);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = le_RigidBody_RigidBody_ihppqbei(iobj_8);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = l_RigidBody_RigidBody_ihppqbeig(iobj_9);
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

  RigidBody_RigidBody_ihppqbeig2(&obj->Base);
  return b_obj;
}

static void emxInitStruct_k_robotics_man_ih(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_i_robotics_manip_(&pStruct->Base);
}

static void emxInitStruct_robotics_slman_ih(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_k_robotics_man_ih(&pStruct->TreeInternal);
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
  obj->Bodies[0] = left_arm_ob_RigidBody_RigidBody(iobj_0);
  obj->Bodies[0]->Index = 1.0;
  obj->Bodies[1] = left_arm__RigidBody_RigidBody_i(iobj_1);
  obj->Bodies[1]->Index = 2.0;
  obj->Bodies[2] = left_arm_RigidBody_RigidBody_ih(iobj_2);
  obj->Bodies[2]->Index = 3.0;
  obj->Bodies[3] = left_ar_RigidBody_RigidBody_ihp(iobj_3);
  obj->Bodies[3]->Index = 4.0;
  obj->Bodies[4] = left_a_RigidBody_RigidBody_ihpp(iobj_4);
  obj->Bodies[4]->Index = 5.0;
  obj->Bodies[5] = left__RigidBody_RigidBody_ihppq(iobj_5);
  obj->Bodies[5]->Index = 6.0;
  obj->Bodies[6] = left_RigidBody_RigidBody_ihppqb(iobj_6);
  obj->Bodies[6]->Index = 7.0;
  obj->Bodies[7] = lef_RigidBody_RigidBody_ihppqbe(iobj_7);
  obj->Bodies[7]->Index = 8.0;
  obj->Bodies[8] = le_RigidBody_RigidBody_ihppqbei(iobj_8);
  obj->Bodies[8]->Index = 9.0;
  obj->Bodies[9] = l_RigidBody_RigidBody_ihppqbeig(iobj_9);
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

  RigidBody_RigidBody_ihppqbeig2(&obj->Base);
  return b_obj;
}

// Model step function
void left_arm_observer_step(void)
{
  emxArray_real_T_left_arm_obse_T *b;
  robotics_slmanip_internal_blo_T *obj;
  emxArray_real_T_left_arm_obse_T *L;
  emxArray_real_T_left_arm_obse_T *lambda;
  emxArray_real_T_left_arm_obse_T *H;
  emxArray_real_T_left_arm_obse_T *tmp;
  static const real_T kp[7] = { 20.5, 1.0, 1.0, 14.5, 1.0, 10.5, 1.0 };

  static const int8_T kd[7] = { 7, 1, 1, 5, 1, 5, 1 };

  boolean_T exitg1;
  if (rtmIsMajorTimeStep(left_arm_observer_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&left_arm_observer_M->solverInfo,
                          ((left_arm_observer_M->Timing.clockTick0+1)*
      left_arm_observer_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(left_arm_observer_M)) {
    left_arm_observer_M->Timing.t[0] = rtsiGetT(&left_arm_observer_M->solverInfo);
  }

  left_arm_observe_emxInit_real_T(&b, 2);
  if (rtmIsMajorTimeStep(left_arm_observer_M) &&
      left_arm_observer_M->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe'
    // MATLABSystem: '<S10>/SourceBlock' incorporates:
    //   Inport: '<S12>/In1'

    left_arm_observ_SystemCore_step(&left_arm_observer_B.b_varargout_1,
      left_arm_observer_B.bias,
      &left_arm_observer_B.b_varargout_2_Data_SL_Info_Curr,
      &left_arm_observer_B.b_varargout_2_Data_SL_Info_Rece,
      &left_arm_observer_B.b_varargout_2_Layout_DataOffset,
      left_arm_observer_B.b_varargout_2_Layout_Dim,
      &left_arm_observer_B.b_varargout_2_Layout_Dim_SL_Inf,
      &left_arm_observer_B.b_varargout_2_Layout_Dim_SL_I_g);

    // Outputs for Enabled SubSystem: '<S10>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S12>/Enable'

    if (left_arm_observer_B.b_varargout_1) {
      for (left_arm_observer_B.i = 0; left_arm_observer_B.i < 7;
           left_arm_observer_B.i++) {
        left_arm_observer_B.In1.Data[left_arm_observer_B.i] =
          left_arm_observer_B.bias[left_arm_observer_B.i];
      }

      left_arm_observer_B.In1.Data_SL_Info.CurrentLength =
        left_arm_observer_B.b_varargout_2_Data_SL_Info_Curr;
      left_arm_observer_B.In1.Data_SL_Info.ReceivedLength =
        left_arm_observer_B.b_varargout_2_Data_SL_Info_Rece;
      left_arm_observer_B.In1.Layout.DataOffset =
        left_arm_observer_B.b_varargout_2_Layout_DataOffset;
      memcpy(&left_arm_observer_B.In1.Layout.Dim[0],
             &left_arm_observer_B.b_varargout_2_Layout_Dim[0], sizeof
             (SL_Bus_left_arm_observer_std_msgs_MultiArrayDimension) << 4U);
      left_arm_observer_B.In1.Layout.Dim_SL_Info.CurrentLength =
        left_arm_observer_B.b_varargout_2_Layout_Dim_SL_Inf;
      left_arm_observer_B.In1.Layout.Dim_SL_Info.ReceivedLength =
        left_arm_observer_B.b_varargout_2_Layout_Dim_SL_I_g;
    }

    // End of MATLABSystem: '<S10>/SourceBlock'
    // End of Outputs for SubSystem: '<S10>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe'

    // MATLABSystem: '<S5>/MATLAB System'
    RigidBodyTreeDynamics_massMat_i(&left_arm_observer_DW.obj_jz.TreeInternal,
      left_arm_observer_B.In1.Data, b);
    for (left_arm_observer_B.j_c = 0; left_arm_observer_B.j_c < 49;
         left_arm_observer_B.j_c++) {
      left_arm_observer_B.MATLABSystem[left_arm_observer_B.j_c] = b->
        data[left_arm_observer_B.j_c];
    }

    // End of MATLABSystem: '<S5>/MATLAB System'
  }

  // MATLABSystem: '<Root>/Polynomial Trajectory' incorporates:
  //   Clock: '<Root>/Clock'

  left_arm_observer_B.b_varargout_1 = false;
  left_arm_observer_B.p = true;
  left_arm_observer_B.i = 0;
  exitg1 = false;
  while ((!exitg1) && (left_arm_observer_B.i < 14)) {
    if (!(left_arm_observer_DW.obj_n.Waypoints[left_arm_observer_B.i] ==
          left_arm_observer_P.PolynomialTrajectory_Waypoints[left_arm_observer_B.i]))
    {
      left_arm_observer_B.p = false;
      exitg1 = true;
    } else {
      left_arm_observer_B.i++;
    }
  }

  if (left_arm_observer_B.p) {
    left_arm_observer_B.b_varargout_1 = true;
  }

  if (!left_arm_observer_B.b_varargout_1) {
    if (left_arm_observer_DW.obj_n.isInitialized == 1) {
      left_arm_observer_DW.obj_n.TunablePropsChanged = true;
    }

    memcpy(&left_arm_observer_DW.obj_n.Waypoints[0],
           &left_arm_observer_P.PolynomialTrajectory_Waypoints[0], 14U * sizeof
           (real_T));
  }

  left_arm_observer_B.b_varargout_1 = false;
  left_arm_observer_B.p = true;
  left_arm_observer_B.i = 0;
  exitg1 = false;
  while ((!exitg1) && (left_arm_observer_B.i < 2)) {
    if (!(left_arm_observer_DW.obj_n.TimePoints[left_arm_observer_B.i] ==
          left_arm_observer_P.PolynomialTrajectory_TimePoints[left_arm_observer_B.i]))
    {
      left_arm_observer_B.p = false;
      exitg1 = true;
    } else {
      left_arm_observer_B.i++;
    }
  }

  if (left_arm_observer_B.p) {
    left_arm_observer_B.b_varargout_1 = true;
  }

  if (!left_arm_observer_B.b_varargout_1) {
    if (left_arm_observer_DW.obj_n.isInitialized == 1) {
      left_arm_observer_DW.obj_n.TunablePropsChanged = true;
    }

    left_arm_observer_DW.obj_n.TimePoints[0] =
      left_arm_observer_P.PolynomialTrajectory_TimePoints[0];
    left_arm_observer_DW.obj_n.TimePoints[1] =
      left_arm_observer_P.PolynomialTrajectory_TimePoints[1];
  }

  left_arm_observer_B.b_varargout_1 = false;
  left_arm_observer_B.p = true;
  left_arm_observer_B.i = 0;
  exitg1 = false;
  while ((!exitg1) && (left_arm_observer_B.i < 14)) {
    if (!(left_arm_observer_DW.obj_n.VelocityBoundaryCondition[left_arm_observer_B.i]
          ==
          left_arm_observer_P.PolynomialTrajectory_VelocityBo[left_arm_observer_B.i]))
    {
      left_arm_observer_B.p = false;
      exitg1 = true;
    } else {
      left_arm_observer_B.i++;
    }
  }

  if (left_arm_observer_B.p) {
    left_arm_observer_B.b_varargout_1 = true;
  }

  if (!left_arm_observer_B.b_varargout_1) {
    if (left_arm_observer_DW.obj_n.isInitialized == 1) {
      left_arm_observer_DW.obj_n.TunablePropsChanged = true;
    }

    memcpy(&left_arm_observer_DW.obj_n.VelocityBoundaryCondition[0],
           &left_arm_observer_P.PolynomialTrajectory_VelocityBo[0], 14U * sizeof
           (real_T));
  }

  left_arm_observer_B.b_varargout_1 = false;
  left_arm_observer_B.p = true;
  left_arm_observer_B.i = 0;
  exitg1 = false;
  while ((!exitg1) && (left_arm_observer_B.i < 14)) {
    if (!(left_arm_observer_DW.obj_n.AccelerationBoundaryCondition[left_arm_observer_B.i]
          ==
          left_arm_observer_P.PolynomialTrajectory_Accelerati[left_arm_observer_B.i]))
    {
      left_arm_observer_B.p = false;
      exitg1 = true;
    } else {
      left_arm_observer_B.i++;
    }
  }

  if (left_arm_observer_B.p) {
    left_arm_observer_B.b_varargout_1 = true;
  }

  if (!left_arm_observer_B.b_varargout_1) {
    if (left_arm_observer_DW.obj_n.isInitialized == 1) {
      left_arm_observer_DW.obj_n.TunablePropsChanged = true;
    }

    memcpy(&left_arm_observer_DW.obj_n.AccelerationBoundaryCondition[0],
           &left_arm_observer_P.PolynomialTrajectory_Accelerati[0], 14U * sizeof
           (real_T));
  }

  if (left_arm_observer_DW.obj_n.TunablePropsChanged) {
    left_arm_observer_DW.obj_n.TunablePropsChanged = false;
  }

  left_arm_o_PolyTrajSys_stepImpl(&left_arm_observer_DW.obj_n,
    left_arm_observer_M->Timing.t[0], left_arm_observer_B.bias,
    left_arm_observer_B.z1, left_arm_observer_B.b_varargout_3);

  // MATLABSystem: '<S4>/MATLAB System' incorporates:
  //   MATLABSystem: '<Root>/Polynomial Trajectory'

  lef_GravityTorqueBlock_stepImpl(&left_arm_observer_DW.obj_j,
    left_arm_observer_B.bias, left_arm_observer_B.b_varargout_3);

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   Integrator: '<Root>/Integrator'
  //   MATLABSystem: '<Root>/Polynomial Trajectory'
  //   MATLABSystem: '<S4>/MATLAB System'

  for (left_arm_observer_B.i = 0; left_arm_observer_B.i < 7;
       left_arm_observer_B.i++) {
    left_arm_observer_B.torque[left_arm_observer_B.i] =
      ((left_arm_observer_B.bias[left_arm_observer_B.i] -
        left_arm_observer_B.In1.Data[left_arm_observer_B.i]) *
       kp[left_arm_observer_B.i] +
       left_arm_observer_B.b_varargout_3[left_arm_observer_B.i]) +
      (left_arm_observer_B.z1[left_arm_observer_B.i] -
       left_arm_observer_X.Integrator_CSTATE[left_arm_observer_B.i + 7]) *
      static_cast<real_T>(kd[left_arm_observer_B.i]);
  }

  // End of MATLAB Function: '<Root>/MATLAB Function1'
  left_arm_observe_emxInit_real_T(&lambda, 2);
  left_arm_observe_emxInit_real_T(&tmp, 1);

  // MATLABSystem: '<S3>/MATLAB System' incorporates:
  //   Constant: '<Root>/Constant2'
  //   Integrator: '<Root>/Integrator'

  obj = &left_arm_observer_DW.obj;
  RigidBodyTreeDynamics_massMatri(&left_arm_observer_DW.obj.TreeInternal,
    left_arm_observer_B.In1.Data, b, lambda);
  left_arm_observer_B.vNum = obj->TreeInternal.VelocityNumber;
  left_arm_observer_B.j_c = tmp->size[0];
  left_arm_observer_B.i = static_cast<int32_T>(left_arm_observer_B.vNum);
  tmp->size[0] = left_arm_observer_B.i;
  left_a_emxEnsureCapacity_real_T(tmp, left_arm_observer_B.j_c);
  for (left_arm_observer_B.j_c = 0; left_arm_observer_B.j_c <
       left_arm_observer_B.i; left_arm_observer_B.j_c++) {
    tmp->data[left_arm_observer_B.j_c] = 0.0;
  }

  RigidBodyTreeDynamics_inverseDy(&obj->TreeInternal,
    left_arm_observer_B.In1.Data, &left_arm_observer_X.Integrator_CSTATE[7], tmp,
    left_arm_observer_P.Constant2_Value, left_arm_observer_B.bias);
  left_arm_observe_emxFree_real_T(&tmp);

  // MATLABSystem: '<S3>/MATLAB System'
  for (left_arm_observer_B.j_c = 0; left_arm_observer_B.j_c < 7;
       left_arm_observer_B.j_c++) {
    left_arm_observer_B.bias[left_arm_observer_B.j_c] =
      left_arm_observer_B.torque[left_arm_observer_B.j_c] -
      left_arm_observer_B.bias[left_arm_observer_B.j_c];
  }

  if ((b->size[0] == 0) || (b->size[1] == 0)) {
    left_arm_observer_B.iend = 0;
  } else {
    left_arm_observer_B.u0 = b->size[0];
    left_arm_observer_B.iend = b->size[1];
    if (left_arm_observer_B.u0 > left_arm_observer_B.iend) {
      left_arm_observer_B.iend = left_arm_observer_B.u0;
    }
  }

  left_arm_observe_emxInit_real_T(&H, 2);

  // MATLABSystem: '<S3>/MATLAB System'
  left_arm_observer_B.j_c = H->size[0] * H->size[1];
  H->size[0] = b->size[0];
  H->size[1] = b->size[1];
  left_a_emxEnsureCapacity_real_T(H, left_arm_observer_B.j_c);
  left_arm_observer_B.u0 = b->size[0] * b->size[1] - 1;
  for (left_arm_observer_B.j_c = 0; left_arm_observer_B.j_c <=
       left_arm_observer_B.u0; left_arm_observer_B.j_c++) {
    H->data[left_arm_observer_B.j_c] = b->data[left_arm_observer_B.j_c];
  }

  left_arm_observe_emxFree_real_T(&b);

  // MATLABSystem: '<S3>/MATLAB System'
  left_arm_observer_B.n = static_cast<int32_T>(((-1.0 - static_cast<real_T>
    (left_arm_observer_B.iend)) + 1.0) / -1.0) - 1;
  for (left_arm_observer_B.u0 = 0; left_arm_observer_B.u0 <=
       left_arm_observer_B.n; left_arm_observer_B.u0++) {
    left_arm_observer_B.j = static_cast<real_T>(left_arm_observer_B.iend) + -
      static_cast<real_T>(left_arm_observer_B.u0);
    left_arm_observer_B.j_c = static_cast<int32_T>(left_arm_observer_B.j);
    left_arm_observer_B.bias_tmp = left_arm_observer_B.j_c - 1;
    H->data[(static_cast<int32_T>(left_arm_observer_B.j) + H->size[0] * (
              static_cast<int32_T>(left_arm_observer_B.j) - 1)) - 1] = sqrt
      (H->data[(left_arm_observer_B.bias_tmp * H->size[0] +
                left_arm_observer_B.j_c) - 1]);
    left_arm_observer_B.k = lambda->data[left_arm_observer_B.bias_tmp];
    while (left_arm_observer_B.k > 0.0) {
      left_arm_observer_B.i_a = static_cast<int32_T>(left_arm_observer_B.k) - 1;
      H->data[(static_cast<int32_T>(left_arm_observer_B.j) + H->size[0] * (
                static_cast<int32_T>(left_arm_observer_B.k) - 1)) - 1] = H->
        data[(left_arm_observer_B.i_a * H->size[0] + left_arm_observer_B.j_c) -
        1] / H->data[((static_cast<int32_T>(left_arm_observer_B.j) - 1) *
                      H->size[0] + static_cast<int32_T>(left_arm_observer_B.j))
        - 1];
      left_arm_observer_B.k = lambda->data[left_arm_observer_B.i_a];
    }

    left_arm_observer_B.k = lambda->data[left_arm_observer_B.bias_tmp];
    while (left_arm_observer_B.k > 0.0) {
      left_arm_observer_B.j = left_arm_observer_B.k;
      while (left_arm_observer_B.j > 0.0) {
        left_arm_observer_B.bias_tmp = static_cast<int32_T>
          (left_arm_observer_B.j) - 1;
        H->data[(static_cast<int32_T>(left_arm_observer_B.k) + H->size[0] * (
                  static_cast<int32_T>(left_arm_observer_B.j) - 1)) - 1] =
          H->data[(left_arm_observer_B.bias_tmp * H->size[0] +
                   static_cast<int32_T>(left_arm_observer_B.k)) - 1] - H->data
          [((static_cast<int32_T>(left_arm_observer_B.k) - 1) * H->size[0] +
            left_arm_observer_B.j_c) - 1] * H->data[((static_cast<int32_T>
          (left_arm_observer_B.j) - 1) * H->size[0] + left_arm_observer_B.j_c) -
          1];
        left_arm_observer_B.j = lambda->data[left_arm_observer_B.bias_tmp];
      }

      left_arm_observer_B.k = lambda->data[static_cast<int32_T>
        (left_arm_observer_B.k) - 1];
    }
  }

  left_arm_observe_emxInit_real_T(&L, 2);

  // MATLABSystem: '<S3>/MATLAB System'
  left_arm_observer_B.j_c = L->size[0] * L->size[1];
  L->size[0] = H->size[0];
  L->size[1] = H->size[1];
  left_a_emxEnsureCapacity_real_T(L, left_arm_observer_B.j_c);
  left_arm_observer_B.u0 = H->size[0] * H->size[1] - 1;
  for (left_arm_observer_B.j_c = 0; left_arm_observer_B.j_c <=
       left_arm_observer_B.u0; left_arm_observer_B.j_c++) {
    L->data[left_arm_observer_B.j_c] = H->data[left_arm_observer_B.j_c];
  }

  left_arm_observer_B.n = H->size[1];
  if ((H->size[0] == 0) || (H->size[1] == 0) || (1 >= H->size[1])) {
  } else {
    left_arm_observer_B.iend = 0;
    for (left_arm_observer_B.j_c = 2; left_arm_observer_B.j_c <=
         left_arm_observer_B.n; left_arm_observer_B.j_c++) {
      for (left_arm_observer_B.u0 = 0; left_arm_observer_B.u0 <=
           left_arm_observer_B.iend; left_arm_observer_B.u0++) {
        L->data[left_arm_observer_B.u0 + L->size[0] * (left_arm_observer_B.j_c -
          1)] = 0.0;
      }

      if (left_arm_observer_B.iend + 1 < H->size[0]) {
        left_arm_observer_B.iend++;
      }
    }
  }

  left_arm_observe_emxFree_real_T(&H);

  // MATLABSystem: '<S3>/MATLAB System'
  left_arm_observer_B.n = static_cast<int32_T>(((-1.0 - left_arm_observer_B.vNum)
    + 1.0) / -1.0) - 1;
  for (left_arm_observer_B.u0 = 0; left_arm_observer_B.u0 <=
       left_arm_observer_B.n; left_arm_observer_B.u0++) {
    left_arm_observer_B.j_c = static_cast<int32_T>(left_arm_observer_B.vNum + -
      static_cast<real_T>(left_arm_observer_B.u0));
    left_arm_observer_B.iend = left_arm_observer_B.j_c - 1;
    left_arm_observer_B.bias[left_arm_observer_B.iend] /= L->data
      [(left_arm_observer_B.iend * L->size[0] + left_arm_observer_B.j_c) - 1];
    left_arm_observer_B.j = lambda->data[left_arm_observer_B.iend];
    while (left_arm_observer_B.j > 0.0) {
      left_arm_observer_B.bias_tmp = static_cast<int32_T>(left_arm_observer_B.j)
        - 1;
      left_arm_observer_B.bias[left_arm_observer_B.bias_tmp] -= L->data
        [(left_arm_observer_B.bias_tmp * L->size[0] + left_arm_observer_B.j_c) -
        1] * left_arm_observer_B.bias[left_arm_observer_B.iend];
      left_arm_observer_B.j = lambda->data[left_arm_observer_B.bias_tmp];
    }
  }

  left_arm_observer_B.i--;
  for (left_arm_observer_B.u0 = 0; left_arm_observer_B.u0 <=
       left_arm_observer_B.i; left_arm_observer_B.u0++) {
    left_arm_observer_B.j = lambda->data[left_arm_observer_B.u0];
    while (left_arm_observer_B.j > 0.0) {
      left_arm_observer_B.iend = static_cast<int32_T>(left_arm_observer_B.j) - 1;
      left_arm_observer_B.bias[left_arm_observer_B.u0] -= L->
        data[left_arm_observer_B.iend * L->size[0] + left_arm_observer_B.u0] *
        left_arm_observer_B.bias[left_arm_observer_B.iend];
      left_arm_observer_B.j = lambda->data[left_arm_observer_B.iend];
    }

    left_arm_observer_B.bias[left_arm_observer_B.u0] /= L->data[L->size[0] *
      left_arm_observer_B.u0 + left_arm_observer_B.u0];
  }

  left_arm_observe_emxFree_real_T(&lambda);
  left_arm_observe_emxFree_real_T(&L);

  // MATLAB Function: '<Root>/Observer' incorporates:
  //   Integrator: '<Root>/Integrator'
  //   MATLABSystem: '<S3>/MATLAB System'

  memset(&left_arm_observer_B.xp_est[0], 0, 14U * sizeof(real_T));
  for (left_arm_observer_B.i = 0; left_arm_observer_B.i < 7;
       left_arm_observer_B.i++) {
    left_arm_observer_B.vNum =
      left_arm_observer_B.In1.Data[left_arm_observer_B.i] -
      left_arm_observer_X.Integrator_CSTATE[left_arm_observer_B.i];
    if (left_arm_observer_B.vNum < 0.0) {
      left_arm_observer_B.scale = -1.0;
    } else if (left_arm_observer_B.vNum > 0.0) {
      left_arm_observer_B.scale = 1.0;
    } else if (left_arm_observer_B.vNum == 0.0) {
      left_arm_observer_B.scale = 0.0;
    } else {
      left_arm_observer_B.scale = (rtNaN);
    }

    left_arm_observer_B.z[left_arm_observer_B.i] = 6.0 *
      left_arm_observer_B.scale;
    left_arm_observer_B.xp_est[left_arm_observer_B.i] = sqrt(fabs
      (left_arm_observer_B.vNum)) * 6.0 * left_arm_observer_B.scale +
      left_arm_observer_X.Integrator_CSTATE[left_arm_observer_B.i + 7];
    left_arm_observer_B.xp_est[left_arm_observer_B.i + 7] =
      left_arm_observer_B.bias[left_arm_observer_B.i] +
      left_arm_observer_B.z[left_arm_observer_B.i];
  }

  // End of MATLAB Function: '<Root>/Observer'
  if (rtmIsMajorTimeStep(left_arm_observer_M) &&
      left_arm_observer_M->Timing.TaskCounters.TID[1] == 0) {
    // BusAssignment: '<Root>/Bus Assignment1' incorporates:
    //   Constant: '<Root>/Constant'
    //   Constant: '<S1>/Constant'

    left_arm_observer_B.BusAssignment1 = left_arm_observer_P.Constant_Value_e;
    for (left_arm_observer_B.i = 0; left_arm_observer_B.i < 7;
         left_arm_observer_B.i++) {
      left_arm_observer_B.BusAssignment1.Data[left_arm_observer_B.i] =
        left_arm_observer_B.torque[left_arm_observer_B.i];
    }

    left_arm_observer_B.BusAssignment1.Data_SL_Info.CurrentLength =
      left_arm_observer_P.Constant_Value_d;
    left_arm_observer_B.BusAssignment1.Data_SL_Info.ReceivedLength =
      left_arm_observer_P.Constant_Value_d;

    // End of BusAssignment: '<Root>/Bus Assignment1'

    // Outputs for Atomic SubSystem: '<Root>/Publish1'
    // MATLABSystem: '<S8>/SinkBlock'
    Pub_left_arm_observer_304.publish(&left_arm_observer_B.BusAssignment1);

    // End of Outputs for SubSystem: '<Root>/Publish1'
  }

  // TransferFcn: '<Root>/Low Pass (z1)'
  left_arm_observer_B.vNum = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)'
  left_arm_observer_B.k = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)1'
  left_arm_observer_B.j = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)2'
  left_arm_observer_B.LowPassz22 = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)3'
  left_arm_observer_B.LowPassz23 = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)4'
  left_arm_observer_B.LowPassz24 = 0.0;

  // TransferFcn: '<Root>/Low Pass (z2)5'
  left_arm_observer_B.LowPassz25 = 0.0;
  for (left_arm_observer_B.i = 0; left_arm_observer_B.i < 5;
       left_arm_observer_B.i++) {
    // TransferFcn: '<Root>/Low Pass (z1)'
    left_arm_observer_B.vNum +=
      left_arm_observer_P.LowPassz1_C[left_arm_observer_B.i] *
      left_arm_observer_X.LowPassz1_CSTATE[left_arm_observer_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)'
    left_arm_observer_B.k +=
      left_arm_observer_P.LowPassz2_C[left_arm_observer_B.i] *
      left_arm_observer_X.LowPassz2_CSTATE[left_arm_observer_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)1'
    left_arm_observer_B.j +=
      left_arm_observer_P.LowPassz21_C[left_arm_observer_B.i] *
      left_arm_observer_X.LowPassz21_CSTATE[left_arm_observer_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)2'
    left_arm_observer_B.LowPassz22 +=
      left_arm_observer_P.LowPassz22_C[left_arm_observer_B.i] *
      left_arm_observer_X.LowPassz22_CSTATE[left_arm_observer_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)3'
    left_arm_observer_B.LowPassz23 +=
      left_arm_observer_P.LowPassz23_C[left_arm_observer_B.i] *
      left_arm_observer_X.LowPassz23_CSTATE[left_arm_observer_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)4'
    left_arm_observer_B.LowPassz24 +=
      left_arm_observer_P.LowPassz24_C[left_arm_observer_B.i] *
      left_arm_observer_X.LowPassz24_CSTATE[left_arm_observer_B.i];

    // TransferFcn: '<Root>/Low Pass (z2)5'
    left_arm_observer_B.LowPassz25 +=
      left_arm_observer_P.LowPassz25_C[left_arm_observer_B.i] *
      left_arm_observer_X.LowPassz25_CSTATE[left_arm_observer_B.i];
  }

  // MATLAB Function: '<Root>/mass estimator' incorporates:
  //   Integrator: '<Root>/Integrator'

  left_arm_observer_B.vel = 0.0;
  left_arm_observer_B.scale = 3.3121686421112381E-170;
  for (left_arm_observer_B.i = 0; left_arm_observer_B.i < 7;
       left_arm_observer_B.i++) {
    left_arm_observer_B.absxk = fabs
      (left_arm_observer_X.Integrator_CSTATE[left_arm_observer_B.i + 7]);
    if (left_arm_observer_B.absxk > left_arm_observer_B.scale) {
      left_arm_observer_B.t = left_arm_observer_B.scale /
        left_arm_observer_B.absxk;
      left_arm_observer_B.vel = left_arm_observer_B.vel * left_arm_observer_B.t *
        left_arm_observer_B.t + 1.0;
      left_arm_observer_B.scale = left_arm_observer_B.absxk;
    } else {
      left_arm_observer_B.t = left_arm_observer_B.absxk /
        left_arm_observer_B.scale;
      left_arm_observer_B.vel += left_arm_observer_B.t * left_arm_observer_B.t;
    }
  }

  left_arm_observer_B.vel = left_arm_observer_B.scale * sqrt
    (left_arm_observer_B.vel);
  left_arm_observer_B.scale = sin((left_arm_observer_B.In1.Data[0] +
    left_arm_observer_B.In1.Data[3]) + left_arm_observer_B.In1.Data[5]);
  if (fabs(left_arm_observer_B.scale) > 0.1) {
    if (left_arm_observer_B.vel < 0.1) {
      // SignalConversion generated from: '<S11>/ SFunction '
      left_arm_observer_B.bias[0] = left_arm_observer_B.vNum;
      left_arm_observer_B.bias[1] = left_arm_observer_B.k;
      left_arm_observer_B.bias[2] = left_arm_observer_B.j;
      left_arm_observer_B.bias[3] = left_arm_observer_B.LowPassz22;
      left_arm_observer_B.bias[4] = left_arm_observer_B.LowPassz23;
      left_arm_observer_B.bias[5] = left_arm_observer_B.LowPassz24;
      left_arm_observer_B.bias[6] = left_arm_observer_B.LowPassz25;
      for (left_arm_observer_B.j_c = 0; left_arm_observer_B.j_c < 7;
           left_arm_observer_B.j_c++) {
        left_arm_observer_B.z1[left_arm_observer_B.j_c] = 0.0;
        for (left_arm_observer_B.bias_tmp = 0; left_arm_observer_B.bias_tmp < 7;
             left_arm_observer_B.bias_tmp++) {
          left_arm_observer_B.z1[left_arm_observer_B.j_c] +=
            left_arm_observer_B.MATLABSystem[7 * left_arm_observer_B.bias_tmp +
            left_arm_observer_B.j_c] *
            left_arm_observer_B.bias[left_arm_observer_B.bias_tmp];
        }
      }

      left_arm_observer_B.vNum = -left_arm_observer_B.z1[5] /
        (1.9129500000000002 * left_arm_observer_B.scale);
    } else {
      left_arm_observer_B.vNum = 0.0;
    }
  } else {
    left_arm_observer_B.vNum = 0.0;
  }

  // End of MATLAB Function: '<Root>/mass estimator'

  // RateTransition: '<Root>/Rate Transition'
  if ((rtmIsMajorTimeStep(left_arm_observer_M) &&
       left_arm_observer_M->Timing.TaskCounters.TID[1] == 0) &&
      (rtmIsMajorTimeStep(left_arm_observer_M) &&
       left_arm_observer_M->Timing.TaskCounters.TID[2] == 0)) {
    left_arm_observer_DW.RateTransition_Buffer = left_arm_observer_B.vNum;
  }

  if (rtmIsMajorTimeStep(left_arm_observer_M) &&
      left_arm_observer_M->Timing.TaskCounters.TID[2] == 0) {
    // BusAssignment: '<Root>/Bus Assignment2'
    left_arm_observer_B.BusAssignment2.Data =
      left_arm_observer_DW.RateTransition_Buffer;

    // Outputs for Atomic SubSystem: '<Root>/Publish2'
    // MATLABSystem: '<S9>/SinkBlock'
    Pub_left_arm_observer_311.publish(&left_arm_observer_B.BusAssignment2);

    // End of Outputs for SubSystem: '<Root>/Publish2'
  }

  // End of RateTransition: '<Root>/Rate Transition'
  if (rtmIsMajorTimeStep(left_arm_observer_M)) {
    rt_ertODEUpdateContinuousStates(&left_arm_observer_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++left_arm_observer_M->Timing.clockTick0;
    left_arm_observer_M->Timing.t[0] = rtsiGetSolverStopTime
      (&left_arm_observer_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.001s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.001, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      left_arm_observer_M->Timing.clockTick1++;
    }

    rate_scheduler();
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void left_arm_observer_derivatives(void)
{
  int_T is;
  XDot_left_arm_observer_T *_rtXdot;
  _rtXdot = ((XDot_left_arm_observer_T *) left_arm_observer_M->derivs);

  // Derivatives for Integrator: '<Root>/Integrator'
  memcpy(&_rtXdot->Integrator_CSTATE[0], &left_arm_observer_B.xp_est[0], 14U *
         sizeof(real_T));
  for (is = 0; is < 5; is++) {
    // Derivatives for TransferFcn: '<Root>/Low Pass (z1)'
    _rtXdot->LowPassz1_CSTATE[is] = 0.0;
    _rtXdot->LowPassz1_CSTATE[0] += left_arm_observer_P.LowPassz1_A[is] *
      left_arm_observer_X.LowPassz1_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)'
    _rtXdot->LowPassz2_CSTATE[is] = 0.0;
    _rtXdot->LowPassz2_CSTATE[0] += left_arm_observer_P.LowPassz2_A[is] *
      left_arm_observer_X.LowPassz2_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)1'
    _rtXdot->LowPassz21_CSTATE[is] = 0.0;
    _rtXdot->LowPassz21_CSTATE[0] += left_arm_observer_P.LowPassz21_A[is] *
      left_arm_observer_X.LowPassz21_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)2'
    _rtXdot->LowPassz22_CSTATE[is] = 0.0;
    _rtXdot->LowPassz22_CSTATE[0] += left_arm_observer_P.LowPassz22_A[is] *
      left_arm_observer_X.LowPassz22_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)3'
    _rtXdot->LowPassz23_CSTATE[is] = 0.0;
    _rtXdot->LowPassz23_CSTATE[0] += left_arm_observer_P.LowPassz23_A[is] *
      left_arm_observer_X.LowPassz23_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)4'
    _rtXdot->LowPassz24_CSTATE[is] = 0.0;
    _rtXdot->LowPassz24_CSTATE[0] += left_arm_observer_P.LowPassz24_A[is] *
      left_arm_observer_X.LowPassz24_CSTATE[is];

    // Derivatives for TransferFcn: '<Root>/Low Pass (z2)5'
    _rtXdot->LowPassz25_CSTATE[is] = 0.0;
    _rtXdot->LowPassz25_CSTATE[0] += left_arm_observer_P.LowPassz25_A[is] *
      left_arm_observer_X.LowPassz25_CSTATE[is];
  }

  // Derivatives for TransferFcn: '<Root>/Low Pass (z1)'
  _rtXdot->LowPassz1_CSTATE[1] += left_arm_observer_X.LowPassz1_CSTATE[0];
  _rtXdot->LowPassz1_CSTATE[2] += left_arm_observer_X.LowPassz1_CSTATE[1];
  _rtXdot->LowPassz1_CSTATE[3] += left_arm_observer_X.LowPassz1_CSTATE[2];
  _rtXdot->LowPassz1_CSTATE[4] += left_arm_observer_X.LowPassz1_CSTATE[3];
  _rtXdot->LowPassz1_CSTATE[0] += left_arm_observer_B.z[0];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)'
  _rtXdot->LowPassz2_CSTATE[1] += left_arm_observer_X.LowPassz2_CSTATE[0];
  _rtXdot->LowPassz2_CSTATE[2] += left_arm_observer_X.LowPassz2_CSTATE[1];
  _rtXdot->LowPassz2_CSTATE[3] += left_arm_observer_X.LowPassz2_CSTATE[2];
  _rtXdot->LowPassz2_CSTATE[4] += left_arm_observer_X.LowPassz2_CSTATE[3];
  _rtXdot->LowPassz2_CSTATE[0] += left_arm_observer_B.z[1];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)1'
  _rtXdot->LowPassz21_CSTATE[1] += left_arm_observer_X.LowPassz21_CSTATE[0];
  _rtXdot->LowPassz21_CSTATE[2] += left_arm_observer_X.LowPassz21_CSTATE[1];
  _rtXdot->LowPassz21_CSTATE[3] += left_arm_observer_X.LowPassz21_CSTATE[2];
  _rtXdot->LowPassz21_CSTATE[4] += left_arm_observer_X.LowPassz21_CSTATE[3];
  _rtXdot->LowPassz21_CSTATE[0] += left_arm_observer_B.z[2];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)2'
  _rtXdot->LowPassz22_CSTATE[1] += left_arm_observer_X.LowPassz22_CSTATE[0];
  _rtXdot->LowPassz22_CSTATE[2] += left_arm_observer_X.LowPassz22_CSTATE[1];
  _rtXdot->LowPassz22_CSTATE[3] += left_arm_observer_X.LowPassz22_CSTATE[2];
  _rtXdot->LowPassz22_CSTATE[4] += left_arm_observer_X.LowPassz22_CSTATE[3];
  _rtXdot->LowPassz22_CSTATE[0] += left_arm_observer_B.z[3];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)3'
  _rtXdot->LowPassz23_CSTATE[1] += left_arm_observer_X.LowPassz23_CSTATE[0];
  _rtXdot->LowPassz23_CSTATE[2] += left_arm_observer_X.LowPassz23_CSTATE[1];
  _rtXdot->LowPassz23_CSTATE[3] += left_arm_observer_X.LowPassz23_CSTATE[2];
  _rtXdot->LowPassz23_CSTATE[4] += left_arm_observer_X.LowPassz23_CSTATE[3];
  _rtXdot->LowPassz23_CSTATE[0] += left_arm_observer_B.z[4];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)4'
  _rtXdot->LowPassz24_CSTATE[1] += left_arm_observer_X.LowPassz24_CSTATE[0];
  _rtXdot->LowPassz24_CSTATE[2] += left_arm_observer_X.LowPassz24_CSTATE[1];
  _rtXdot->LowPassz24_CSTATE[3] += left_arm_observer_X.LowPassz24_CSTATE[2];
  _rtXdot->LowPassz24_CSTATE[4] += left_arm_observer_X.LowPassz24_CSTATE[3];
  _rtXdot->LowPassz24_CSTATE[0] += left_arm_observer_B.z[5];

  // Derivatives for TransferFcn: '<Root>/Low Pass (z2)5'
  _rtXdot->LowPassz25_CSTATE[1] += left_arm_observer_X.LowPassz25_CSTATE[0];
  _rtXdot->LowPassz25_CSTATE[2] += left_arm_observer_X.LowPassz25_CSTATE[1];
  _rtXdot->LowPassz25_CSTATE[3] += left_arm_observer_X.LowPassz25_CSTATE[2];
  _rtXdot->LowPassz25_CSTATE[4] += left_arm_observer_X.LowPassz25_CSTATE[3];
  _rtXdot->LowPassz25_CSTATE[0] += left_arm_observer_B.z[6];
}

// Model initialize function
void left_arm_observer_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&left_arm_observer_M->solverInfo,
                          &left_arm_observer_M->Timing.simTimeStep);
    rtsiSetTPtr(&left_arm_observer_M->solverInfo, &rtmGetTPtr
                (left_arm_observer_M));
    rtsiSetStepSizePtr(&left_arm_observer_M->solverInfo,
                       &left_arm_observer_M->Timing.stepSize0);
    rtsiSetdXPtr(&left_arm_observer_M->solverInfo, &left_arm_observer_M->derivs);
    rtsiSetContStatesPtr(&left_arm_observer_M->solverInfo, (real_T **)
                         &left_arm_observer_M->contStates);
    rtsiSetNumContStatesPtr(&left_arm_observer_M->solverInfo,
      &left_arm_observer_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&left_arm_observer_M->solverInfo,
      &left_arm_observer_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&left_arm_observer_M->solverInfo,
      &left_arm_observer_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&left_arm_observer_M->solverInfo,
      &left_arm_observer_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&left_arm_observer_M->solverInfo, (&rtmGetErrorStatus
      (left_arm_observer_M)));
    rtsiSetRTModelPtr(&left_arm_observer_M->solverInfo, left_arm_observer_M);
  }

  rtsiSetSimTimeStep(&left_arm_observer_M->solverInfo, MAJOR_TIME_STEP);
  left_arm_observer_M->intgData.y = left_arm_observer_M->odeY;
  left_arm_observer_M->intgData.f[0] = left_arm_observer_M->odeF[0];
  left_arm_observer_M->intgData.f[1] = left_arm_observer_M->odeF[1];
  left_arm_observer_M->intgData.f[2] = left_arm_observer_M->odeF[2];
  left_arm_observer_M->intgData.f[3] = left_arm_observer_M->odeF[3];
  left_arm_observer_M->contStates = ((X_left_arm_observer_T *)
    &left_arm_observer_X);
  rtsiSetSolverData(&left_arm_observer_M->solverInfo, static_cast<void *>
                    (&left_arm_observer_M->intgData));
  rtsiSetSolverName(&left_arm_observer_M->solverInfo,"ode4");
  rtmSetTPtr(left_arm_observer_M, &left_arm_observer_M->Timing.tArray[0]);
  left_arm_observer_M->Timing.stepSize0 = 0.001;

  {
    int_T is;
    static const char_T tmp[25] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r', 'e',
      '/', 'l', 'a', '_', 'c', 'u', 'r', 'r', 'e', 'n', 't', '_', 'p', 'o', 's',
      'e' };

    static const char_T tmp_0[19] = { '/', 'h', 'a', 'r', 'd', 'w', 'a', 'r',
      'e', '/', 'l', 'a', '_', 't', 'o', 'r', 'q', 'u', 'e' };

    static const char_T tmp_1[15] = { '/', 'e', 's', 't', 'i', 'm', 'a', 't',
      'e', 'd', '_', 'm', 'a', 's', 's' };

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S10>/SourceBlock'
    left_arm_observer_DW.obj_m.matlabCodegenIsDeleted = false;
    left_arm_observer_DW.obj_m.isInitialized = 1;
    for (is = 0; is < 25; is++) {
      left_arm_observer_B.cv[is] = tmp[is];
    }

    left_arm_observer_B.cv[25] = '\x00';
    Sub_left_arm_observer_299.createSubscriber(left_arm_observer_B.cv, 1);
    left_arm_observer_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S10>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'
    emxInitStruct_robotics_slmanip_(&left_arm_observer_DW.obj_jz);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_0);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_19);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_18);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_17);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_16);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_15);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_14);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_13);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_12);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_11);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_10);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_9);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_8);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_7);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_6);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_5);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_4);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_3);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_2);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_1);

    // Start for MATLABSystem: '<S5>/MATLAB System'
    left_arm_observer_DW.obj_jz.isInitialized = 0;
    left_arm_observer_DW.obj_jz.isInitialized = 1;
    RigidBodyTree_RigidBodyTree_ih(&left_arm_observer_DW.obj_jz.TreeInternal,
      &left_arm_observer_DW.gobj_0, &left_arm_observer_DW.gobj_19,
      &left_arm_observer_DW.gobj_18, &left_arm_observer_DW.gobj_17,
      &left_arm_observer_DW.gobj_16, &left_arm_observer_DW.gobj_15,
      &left_arm_observer_DW.gobj_14, &left_arm_observer_DW.gobj_13,
      &left_arm_observer_DW.gobj_12, &left_arm_observer_DW.gobj_11);

    // Start for MATLABSystem: '<Root>/Polynomial Trajectory'
    left_arm_observer_DW.obj_n.isInitialized = 0;
    if (left_arm_observer_DW.obj_n.isInitialized == 1) {
      left_arm_observer_DW.obj_n.TunablePropsChanged = true;
    }

    memcpy(&left_arm_observer_DW.obj_n.Waypoints[0],
           &left_arm_observer_P.PolynomialTrajectory_Waypoints[0], 14U * sizeof
           (real_T));
    if (left_arm_observer_DW.obj_n.isInitialized == 1) {
      left_arm_observer_DW.obj_n.TunablePropsChanged = true;
    }

    left_arm_observer_DW.obj_n.TimePoints[0] =
      left_arm_observer_P.PolynomialTrajectory_TimePoints[0];
    left_arm_observer_DW.obj_n.TimePoints[1] =
      left_arm_observer_P.PolynomialTrajectory_TimePoints[1];
    if (left_arm_observer_DW.obj_n.isInitialized == 1) {
      left_arm_observer_DW.obj_n.TunablePropsChanged = true;
    }

    memcpy(&left_arm_observer_DW.obj_n.VelocityBoundaryCondition[0],
           &left_arm_observer_P.PolynomialTrajectory_VelocityBo[0], 14U * sizeof
           (real_T));
    if (left_arm_observer_DW.obj_n.isInitialized == 1) {
      left_arm_observer_DW.obj_n.TunablePropsChanged = true;
    }

    memcpy(&left_arm_observer_DW.obj_n.AccelerationBoundaryCondition[0],
           &left_arm_observer_P.PolynomialTrajectory_Accelerati[0], 14U * sizeof
           (real_T));
    left_arm_observer_DW.obj_n.isInitialized = 1;
    left_arm_observer_DW.obj_n.TunablePropsChanged = false;

    // End of Start for MATLABSystem: '<Root>/Polynomial Trajectory'
    emxInitStruct_robotics_slmani_i(&left_arm_observer_DW.obj_j);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_0_f);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_19_h);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_18_b);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_17_d);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_16_a);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_15_k);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_14_a);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_13_l);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_12_g);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_11_b);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_10_n);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_9_k);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_8_g);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_7_a);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_6_k);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_5_g);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_4_p);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_3_j);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_2_p);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_1_o);

    // Start for MATLABSystem: '<S4>/MATLAB System'
    left_arm_observer_DW.obj_j.isInitialized = 0;
    left_arm_observer_DW.obj_j.isInitialized = 1;
    l_RigidBodyTree_RigidBodyTree_i(&left_arm_observer_DW.obj_j.TreeInternal,
      &left_arm_observer_DW.gobj_0_f, &left_arm_observer_DW.gobj_19_h,
      &left_arm_observer_DW.gobj_18_b, &left_arm_observer_DW.gobj_17_d,
      &left_arm_observer_DW.gobj_16_a, &left_arm_observer_DW.gobj_15_k,
      &left_arm_observer_DW.gobj_14_a, &left_arm_observer_DW.gobj_13_l,
      &left_arm_observer_DW.gobj_12_g, &left_arm_observer_DW.gobj_11_b);
    emxInitStruct_robotics_slman_ih(&left_arm_observer_DW.obj);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_0_e);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_19_i);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_18_m);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_17_e);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_16_e);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_15_d);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_14_f);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_13_n);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_12_m);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_11_p);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_10_e);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_9_l);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_8_j);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_7_i);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_6_ke);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_5_e);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_4_f);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_3_f);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_2_b);
    emxInitStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_1_f);

    // Start for MATLABSystem: '<S3>/MATLAB System'
    left_arm_observer_DW.obj.isInitialized = 0;
    left_arm_observer_DW.obj.isInitialized = 1;
    lef_RigidBodyTree_RigidBodyTree(&left_arm_observer_DW.obj.TreeInternal,
      &left_arm_observer_DW.gobj_0_e, &left_arm_observer_DW.gobj_19_i,
      &left_arm_observer_DW.gobj_18_m, &left_arm_observer_DW.gobj_17_e,
      &left_arm_observer_DW.gobj_16_e, &left_arm_observer_DW.gobj_15_d,
      &left_arm_observer_DW.gobj_14_f, &left_arm_observer_DW.gobj_13_n,
      &left_arm_observer_DW.gobj_12_m, &left_arm_observer_DW.gobj_11_p);

    // Start for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S8>/SinkBlock'
    left_arm_observer_DW.obj_d.matlabCodegenIsDeleted = false;
    left_arm_observer_DW.obj_d.isInitialized = 1;
    for (is = 0; is < 19; is++) {
      left_arm_observer_B.cv1[is] = tmp_0[is];
    }

    left_arm_observer_B.cv1[19] = '\x00';
    Pub_left_arm_observer_304.createPublisher(left_arm_observer_B.cv1, 1);
    left_arm_observer_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S8>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish1'

    // Start for Atomic SubSystem: '<Root>/Publish2'
    // Start for MATLABSystem: '<S9>/SinkBlock'
    left_arm_observer_DW.obj_a.matlabCodegenIsDeleted = false;
    left_arm_observer_DW.obj_a.isInitialized = 1;
    for (is = 0; is < 15; is++) {
      left_arm_observer_B.cv2[is] = tmp_1[is];
    }

    left_arm_observer_B.cv2[15] = '\x00';
    Pub_left_arm_observer_311.createPublisher(left_arm_observer_B.cv2, 1);
    left_arm_observer_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S9>/SinkBlock'
    // End of Start for SubSystem: '<Root>/Publish2'

    // InitializeConditions for Integrator: '<Root>/Integrator'
    memcpy(&left_arm_observer_X.Integrator_CSTATE[0],
           &left_arm_observer_P.Integrator_IC[0], 14U * sizeof(real_T));
    for (is = 0; is < 5; is++) {
      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z1)'
      left_arm_observer_X.LowPassz1_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)'
      left_arm_observer_X.LowPassz2_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)1'
      left_arm_observer_X.LowPassz21_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)2'
      left_arm_observer_X.LowPassz22_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)3'
      left_arm_observer_X.LowPassz23_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)4'
      left_arm_observer_X.LowPassz24_CSTATE[is] = 0.0;

      // InitializeConditions for TransferFcn: '<Root>/Low Pass (z2)5'
      left_arm_observer_X.LowPassz25_CSTATE[is] = 0.0;
    }

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S10>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S12>/Out1'
    left_arm_observer_B.In1 = left_arm_observer_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S10>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'
  }
}

// Model terminate function
void left_arm_observer_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S10>/SourceBlock'
  matlabCodegenHandle_matlabCo_ih(&left_arm_observer_DW.obj_m);

  // End of Terminate for SubSystem: '<Root>/Subscribe'
  emxFreeStruct_robotics_slmanip_(&left_arm_observer_DW.obj_jz);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_0);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_19);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_18);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_17);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_16);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_15);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_14);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_13);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_12);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_11);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_10);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_9);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_8);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_7);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_6);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_5);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_4);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_3);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_2);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_1);
  emxFreeStruct_robotics_slmani_i(&left_arm_observer_DW.obj_j);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_0_f);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_19_h);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_18_b);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_17_d);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_16_a);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_15_k);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_14_a);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_13_l);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_12_g);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_11_b);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_10_n);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_9_k);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_8_g);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_7_a);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_6_k);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_5_g);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_4_p);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_3_j);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_2_p);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_1_o);
  emxFreeStruct_robotics_slman_ih(&left_arm_observer_DW.obj);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_0_e);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_19_i);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_18_m);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_17_e);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_16_e);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_15_d);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_14_f);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_13_n);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_12_m);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_11_p);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_10_e);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_9_l);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_8_j);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_7_i);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_6_ke);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_5_e);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_4_f);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_3_f);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_2_b);
  emxFreeStruct_j_robotics_manip_(&left_arm_observer_DW.gobj_1_f);

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S8>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_observer_DW.obj_d);

  // End of Terminate for SubSystem: '<Root>/Publish1'

  // Terminate for Atomic SubSystem: '<Root>/Publish2'
  // Terminate for MATLABSystem: '<S9>/SinkBlock'
  matlabCodegenHandle_matlabCodeg(&left_arm_observer_DW.obj_a);

  // End of Terminate for SubSystem: '<Root>/Publish2'
}

//
// File trailer for generated code.
//
// [EOF]
//
