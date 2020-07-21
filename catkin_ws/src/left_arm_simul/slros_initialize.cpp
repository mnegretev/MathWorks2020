#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "left_arm_simul";

// For Block left_arm_simul/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_left_arm_simul_std_msgs_Float64MultiArray> Sub_left_arm_simul_160;

// For Block left_arm_simul/Subscribe1
SimulinkSubscriber<std_msgs::Float64, SL_Bus_left_arm_simul_std_msgs_Float64> Sub_left_arm_simul_272;

// For Block left_arm_simul/ROS Publishers/Publish1
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_simul_std_msgs_Float64MultiArray> Pub_left_arm_simul_163;

// For Block left_arm_simul/ROS Publishers/Publish2
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_left_arm_simul_sensor_msgs_JointState> Pub_left_arm_simul_191;

// For Block left_arm_simul/Get Parameter
SimulinkParameterGetter<int32_T, int> ParamGet_left_arm_simul_433;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

