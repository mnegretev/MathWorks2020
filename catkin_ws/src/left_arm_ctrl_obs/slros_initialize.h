#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block left_arm_ctrl_obs/Subscribe
extern SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Sub_left_arm_ctrl_obs_299;

// For Block left_arm_ctrl_obs/Subscribe1
extern SimulinkSubscriber<std_msgs::Float32MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray> Sub_left_arm_ctrl_obs_318;

// For Block left_arm_ctrl_obs/Publish1
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Pub_left_arm_ctrl_obs_304;

// For Block left_arm_ctrl_obs/Publish2
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_311;

// For Block left_arm_ctrl_obs/Publish3
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Pub_left_arm_ctrl_obs_331;

void slros_node_init(int argc, char** argv);

#endif
