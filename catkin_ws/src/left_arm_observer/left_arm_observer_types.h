//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: left_arm_observer_types.h
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
#ifndef RTW_HEADER_left_arm_observer_types_h_
#define RTW_HEADER_left_arm_observer_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_left_arm_observer_std_msgs_MultiArrayDimension_
#define DEFINED_TYPEDEF_FOR_SL_Bus_left_arm_observer_std_msgs_MultiArrayDimension_

// MsgType=std_msgs/MultiArrayDimension
typedef struct {
  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Label_SL_Info:TruncateAction=warn 
  uint8_T Label[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Label
  SL_Bus_ROSVariableLengthArrayInfo Label_SL_Info;
  uint32_T Size;
  uint32_T Stride;
} SL_Bus_left_arm_observer_std_msgs_MultiArrayDimension;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_left_arm_observer_std_msgs_MultiArrayLayout_
#define DEFINED_TYPEDEF_FOR_SL_Bus_left_arm_observer_std_msgs_MultiArrayLayout_

// MsgType=std_msgs/MultiArrayLayout
typedef struct {
  uint32_T DataOffset;

  // MsgType=std_msgs/MultiArrayDimension:IsVarLen=1:VarLenCategory=data:VarLenElem=Dim_SL_Info:TruncateAction=warn 
  SL_Bus_left_arm_observer_std_msgs_MultiArrayDimension Dim[16];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Dim
  SL_Bus_ROSVariableLengthArrayInfo Dim_SL_Info;
} SL_Bus_left_arm_observer_std_msgs_MultiArrayLayout;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_left_arm_observer_std_msgs_Float64MultiArray_
#define DEFINED_TYPEDEF_FOR_SL_Bus_left_arm_observer_std_msgs_Float64MultiArray_

// MsgType=std_msgs/Float64MultiArray
typedef struct {
  // IsVarLen=1:VarLenCategory=data:VarLenElem=Data_SL_Info:TruncateAction=warn
  real_T Data[7];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Data
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;

  // MsgType=std_msgs/MultiArrayLayout
  SL_Bus_left_arm_observer_std_msgs_MultiArrayLayout Layout;
} SL_Bus_left_arm_observer_std_msgs_Float64MultiArray;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_left_arm_observer_std_msgs_Float64_
#define DEFINED_TYPEDEF_FOR_SL_Bus_left_arm_observer_std_msgs_Float64_

// MsgType=std_msgs/Float64
typedef struct {
  real_T Data;
} SL_Bus_left_arm_observer_std_msgs_Float64;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_8MDKU1QH03aEqJoxBiQBqF_
#define DEFINED_TYPEDEF_FOR_struct_8MDKU1QH03aEqJoxBiQBqF_

typedef struct {
  real_T NameLength;
  uint8_T Name[20];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[10];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
} struct_8MDKU1QH03aEqJoxBiQBqF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_5Np81zJKxJDg8ksUwS8TqH_
#define DEFINED_TYPEDEF_FOR_struct_5Np81zJKxJDg8ksUwS8TqH_

typedef struct {
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[20];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
} struct_5Np81zJKxJDg8ksUwS8TqH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_EuvQoF9u9X0OfH8NoM43sB_
#define DEFINED_TYPEDEF_FOR_struct_EuvQoF9u9X0OfH8NoM43sB_

typedef struct {
  real_T NumBodies;
  real_T Gravity[3];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[20];
  real_T VelocityDoFMap[20];
  real_T MaxNameLength;
  real_T MaxJointPositionNumber;
  uint8_T DataFormat;
  real_T JointPositionLimits[14];
  struct_8MDKU1QH03aEqJoxBiQBqF Bodies[11];
  struct_5Np81zJKxJDg8ksUwS8TqH Joints[11];
} struct_EuvQoF9u9X0OfH8NoM43sB;

#endif

#ifndef struct_tag_se9SihvfVOWUxRfnSBSRy7D
#define struct_tag_se9SihvfVOWUxRfnSBSRy7D

struct tag_se9SihvfVOWUxRfnSBSRy7D
{
  uint32_T VelocityBoundaryCondition;
  uint32_T AccelerationBoundaryCondition;
};

#endif                                 //struct_tag_se9SihvfVOWUxRfnSBSRy7D

#ifndef typedef_se9SihvfVOWUxRfnSBSRy7D_left__T
#define typedef_se9SihvfVOWUxRfnSBSRy7D_left__T

typedef struct tag_se9SihvfVOWUxRfnSBSRy7D se9SihvfVOWUxRfnSBSRy7D_left__T;

#endif                                 //typedef_se9SihvfVOWUxRfnSBSRy7D_left__T

#ifndef struct_tag_s88wEDgTi6Bbogjq7PbKTuE
#define struct_tag_s88wEDgTi6Bbogjq7PbKTuE

struct tag_s88wEDgTi6Bbogjq7PbKTuE
{
  real_T VelocityBoundaryCondition;
  real_T AccelerationBoundaryCondition;
};

#endif                                 //struct_tag_s88wEDgTi6Bbogjq7PbKTuE

#ifndef typedef_s88wEDgTi6Bbogjq7PbKTuE_left__T
#define typedef_s88wEDgTi6Bbogjq7PbKTuE_left__T

typedef struct tag_s88wEDgTi6Bbogjq7PbKTuE s88wEDgTi6Bbogjq7PbKTuE_left__T;

#endif                                 //typedef_s88wEDgTi6Bbogjq7PbKTuE_left__T

#ifndef typedef_ros_slros_internal_block_Publ_T
#define typedef_ros_slros_internal_block_Publ_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} ros_slros_internal_block_Publ_T;

#endif                                 //typedef_ros_slros_internal_block_Publ_T

#ifndef typedef_ros_slros_internal_block_Subs_T
#define typedef_ros_slros_internal_block_Subs_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} ros_slros_internal_block_Subs_T;

#endif                                 //typedef_ros_slros_internal_block_Subs_T

#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T

struct emxArray_char_T
{
  char_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_char_T

#ifndef typedef_emxArray_char_T_left_arm_obse_T
#define typedef_emxArray_char_T_left_arm_obse_T

typedef struct emxArray_char_T emxArray_char_T_left_arm_obse_T;

#endif                                 //typedef_emxArray_char_T_left_arm_obse_T

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T

#ifndef typedef_emxArray_real_T_left_arm_obse_T
#define typedef_emxArray_real_T_left_arm_obse_T

typedef struct emxArray_real_T emxArray_real_T_left_arm_obse_T;

#endif                                 //typedef_emxArray_real_T_left_arm_obse_T

#ifndef typedef_struct_T_left_arm_observer_T
#define typedef_struct_T_left_arm_observer_T

typedef struct {
  char_T Value[8];
} struct_T_left_arm_observer_T;

#endif                                 //typedef_struct_T_left_arm_observer_T

#ifndef struct_tag_so4pBuofLXIy3ixtLNBoZq
#define struct_tag_so4pBuofLXIy3ixtLNBoZq

struct tag_so4pBuofLXIy3ixtLNBoZq
{
  char_T PartialMatching[6];
};

#endif                                 //struct_tag_so4pBuofLXIy3ixtLNBoZq

#ifndef typedef_so4pBuofLXIy3ixtLNBoZq_left_a_T
#define typedef_so4pBuofLXIy3ixtLNBoZq_left_a_T

typedef struct tag_so4pBuofLXIy3ixtLNBoZq so4pBuofLXIy3ixtLNBoZq_left_a_T;

#endif                                 //typedef_so4pBuofLXIy3ixtLNBoZq_left_a_T

#ifndef typedef_c_cell_wrap_left_arm_observer_T
#define typedef_c_cell_wrap_left_arm_observer_T

typedef struct {
  uint32_T f1[8];
} c_cell_wrap_left_arm_observer_T;

#endif                                 //typedef_c_cell_wrap_left_arm_observer_T

#ifndef typedef_e_cell_wrap_left_arm_observer_T
#define typedef_e_cell_wrap_left_arm_observer_T

typedef struct {
  real_T f1[14];
} e_cell_wrap_left_arm_observer_T;

#endif                                 //typedef_e_cell_wrap_left_arm_observer_T

#ifndef typedef_c_robotics_core_internal_code_T
#define typedef_c_robotics_core_internal_code_T

typedef struct {
  e_cell_wrap_left_arm_observer_T ParsedResults[2];
} c_robotics_core_internal_code_T;

#endif                                 //typedef_c_robotics_core_internal_code_T

#ifndef struct_tag_ssoOm1dpzkll0ActLkI75EE
#define struct_tag_ssoOm1dpzkll0ActLkI75EE

struct tag_ssoOm1dpzkll0ActLkI75EE
{
  real_T NameLength;
  uint8_T Name[20];
  real_T ParentIndex;
  real_T NumChildren;
  real_T ChildrenIndices[10];
  real_T Mass;
  real_T CenterOfMass[3];
  real_T Inertia[9];
  real_T SpatialInertia[36];
};

#endif                                 //struct_tag_ssoOm1dpzkll0ActLkI75EE

#ifndef typedef_ssoOm1dpzkll0ActLkI75EE_left__T
#define typedef_ssoOm1dpzkll0ActLkI75EE_left__T

typedef struct tag_ssoOm1dpzkll0ActLkI75EE ssoOm1dpzkll0ActLkI75EE_left__T;

#endif                                 //typedef_ssoOm1dpzkll0ActLkI75EE_left__T

#ifndef struct_tag_sb1GSy5uAZojAgyRiXnSERH
#define struct_tag_sb1GSy5uAZojAgyRiXnSERH

struct tag_sb1GSy5uAZojAgyRiXnSERH
{
  uint8_T Type;
  real_T NameLength;
  uint8_T Name[20];
  real_T VelocityNumber;
  real_T PositionNumber;
  real_T MotionSubspace[36];
  real_T JointAxis[3];
  real_T PositionLimits[14];
  real_T HomePosition[7];
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
};

#endif                                 //struct_tag_sb1GSy5uAZojAgyRiXnSERH

#ifndef typedef_sb1GSy5uAZojAgyRiXnSERH_left__T
#define typedef_sb1GSy5uAZojAgyRiXnSERH_left__T

typedef struct tag_sb1GSy5uAZojAgyRiXnSERH sb1GSy5uAZojAgyRiXnSERH_left__T;

#endif                                 //typedef_sb1GSy5uAZojAgyRiXnSERH_left__T

#ifndef struct_tag_s2BTGTsPQneIOsjQTihd6PE
#define struct_tag_s2BTGTsPQneIOsjQTihd6PE

struct tag_s2BTGTsPQneIOsjQTihd6PE
{
  real_T NumBodies;
  real_T Gravity[3];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[20];
  real_T VelocityDoFMap[20];
  real_T MaxNameLength;
  real_T MaxJointPositionNumber;
  uint8_T DataFormat;
  real_T JointPositionLimits[14];
  ssoOm1dpzkll0ActLkI75EE_left__T Bodies[11];
  sb1GSy5uAZojAgyRiXnSERH_left__T Joints[11];
};

#endif                                 //struct_tag_s2BTGTsPQneIOsjQTihd6PE

#ifndef typedef_s2BTGTsPQneIOsjQTihd6PE_left__T
#define typedef_s2BTGTsPQneIOsjQTihd6PE_left__T

typedef struct tag_s2BTGTsPQneIOsjQTihd6PE s2BTGTsPQneIOsjQTihd6PE_left__T;

#endif                                 //typedef_s2BTGTsPQneIOsjQTihd6PE_left__T

#ifndef typedef_f_cell_wrap_left_arm_observer_T
#define typedef_f_cell_wrap_left_arm_observer_T

typedef struct {
  real_T f1[36];
} f_cell_wrap_left_arm_observer_T;

#endif                                 //typedef_f_cell_wrap_left_arm_observer_T

#ifndef typedef_rigidBodyJoint_left_arm_obser_T
#define typedef_rigidBodyJoint_left_arm_obser_T

typedef struct {
  emxArray_char_T_left_arm_obse_T *Type;
  emxArray_real_T_left_arm_obse_T *MotionSubspace;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  real_T JointAxisInternal[3];
} rigidBodyJoint_left_arm_obser_T;

#endif                                 //typedef_rigidBodyJoint_left_arm_obser_T

#ifndef typedef_j_robotics_manip_internal_Rig_T
#define typedef_j_robotics_manip_internal_Rig_T

typedef struct {
  real_T Index;
  rigidBodyJoint_left_arm_obser_T JointInternal;
  real_T ParentIndex;
  real_T SpatialInertia[36];
} j_robotics_manip_internal_Rig_T;

#endif                                 //typedef_j_robotics_manip_internal_Rig_T

#ifndef typedef_i_robotics_manip_internal_Rig_T
#define typedef_i_robotics_manip_internal_Rig_T

typedef struct {
  rigidBodyJoint_left_arm_obser_T JointInternal;
} i_robotics_manip_internal_Rig_T;

#endif                                 //typedef_i_robotics_manip_internal_Rig_T

#ifndef typedef_k_robotics_manip_internal_Rig_T
#define typedef_k_robotics_manip_internal_Rig_T

typedef struct {
  real_T NumBodies;
  i_robotics_manip_internal_Rig_T Base;
  real_T Gravity[3];
  j_robotics_manip_internal_Rig_T *Bodies[10];
  real_T VelocityNumber;
  real_T PositionDoFMap[20];
  real_T VelocityDoFMap[20];
} k_robotics_manip_internal_Rig_T;

#endif                                 //typedef_k_robotics_manip_internal_Rig_T

#ifndef typedef_robotics_slmanip_internal_blo_T
#define typedef_robotics_slmanip_internal_blo_T

typedef struct {
  int32_T isInitialized;
  k_robotics_manip_internal_Rig_T TreeInternal;
} robotics_slmanip_internal_blo_T;

#endif                                 //typedef_robotics_slmanip_internal_blo_T

#ifndef typedef_e_cell_wrap_left_arm_observ_i_T
#define typedef_e_cell_wrap_left_arm_observ_i_T

typedef struct {
  real_T f1[36];
} e_cell_wrap_left_arm_observ_i_T;

#endif                                 //typedef_e_cell_wrap_left_arm_observ_i_T

#ifndef typedef_k_robotics_manip_internal_R_i_T
#define typedef_k_robotics_manip_internal_R_i_T

typedef struct {
  real_T NumBodies;
  i_robotics_manip_internal_Rig_T Base;
  real_T Gravity[3];
  j_robotics_manip_internal_Rig_T *Bodies[10];
  real_T PositionDoFMap[20];
  real_T VelocityDoFMap[20];
} k_robotics_manip_internal_R_i_T;

#endif                                 //typedef_k_robotics_manip_internal_R_i_T

#ifndef typedef_robotics_slmanip_internal_b_i_T
#define typedef_robotics_slmanip_internal_b_i_T

typedef struct {
  int32_T isInitialized;
  k_robotics_manip_internal_R_i_T TreeInternal;
} robotics_slmanip_internal_b_i_T;

#endif                                 //typedef_robotics_slmanip_internal_b_i_T

#ifndef typedef_k_robotics_manip_internal__ih_T
#define typedef_k_robotics_manip_internal__ih_T

typedef struct {
  real_T NumBodies;
  i_robotics_manip_internal_Rig_T Base;
  j_robotics_manip_internal_Rig_T *Bodies[10];
  real_T VelocityNumber;
  real_T PositionDoFMap[20];
  real_T VelocityDoFMap[20];
} k_robotics_manip_internal__ih_T;

#endif                                 //typedef_k_robotics_manip_internal__ih_T

#ifndef typedef_robotics_slmanip_internal__ih_T
#define typedef_robotics_slmanip_internal__ih_T

typedef struct {
  int32_T isInitialized;
  k_robotics_manip_internal__ih_T TreeInternal;
} robotics_slmanip_internal__ih_T;

#endif                                 //typedef_robotics_slmanip_internal__ih_T

#ifndef typedef_emxArray_f_cell_wrap_left_arm_T
#define typedef_emxArray_f_cell_wrap_left_arm_T

typedef struct {
  f_cell_wrap_left_arm_observer_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_f_cell_wrap_left_arm_T;

#endif                                 //typedef_emxArray_f_cell_wrap_left_arm_T

#ifndef typedef_emxArray_e_cell_wrap_left_arm_T
#define typedef_emxArray_e_cell_wrap_left_arm_T

typedef struct {
  e_cell_wrap_left_arm_observ_i_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
} emxArray_e_cell_wrap_left_arm_T;

#endif                                 //typedef_emxArray_e_cell_wrap_left_arm_T

#ifndef typedef_robotics_slcore_internal_bloc_T
#define typedef_robotics_slcore_internal_bloc_T

typedef struct {
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  c_cell_wrap_left_arm_observer_T inputVarSize;
  real_T Waypoints[14];
  real_T TimePoints[2];
  real_T VelocityBoundaryCondition[14];
  real_T AccelerationBoundaryCondition[14];
} robotics_slcore_internal_bloc_T;

#endif                                 //typedef_robotics_slcore_internal_bloc_T

// Parameters (default storage)
typedef struct P_left_arm_observer_T_ P_left_arm_observer_T;

// Forward declaration for rtModel
typedef struct tag_RTM_left_arm_observer_T RT_MODEL_left_arm_observer_T;

#endif                                 // RTW_HEADER_left_arm_observer_types_h_

//
// File trailer for generated code.
//
// [EOF]
//