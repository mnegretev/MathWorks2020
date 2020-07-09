#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "left_arm_ctrl_obs";

// For Block left_arm_ctrl_obs/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Sub_left_arm_ctrl_obs_299;

// For Block left_arm_ctrl_obs/Subscribe1
SimulinkSubscriber<std_msgs::Float32MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray> Sub_left_arm_ctrl_obs_318;

// For Block left_arm_ctrl_obs/Publish1
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Pub_left_arm_ctrl_obs_304;

// For Block left_arm_ctrl_obs/Publish2
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_311;

// For Block left_arm_ctrl_obs/Publish3
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Pub_left_arm_ctrl_obs_331;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

