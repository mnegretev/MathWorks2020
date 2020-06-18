#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block la_2dof_ctrl_obs/Subscribe
extern SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray> Sub_la_2dof_ctrl_obs_193;

// For Block la_2dof_ctrl_obs/Subscribe1
extern SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray> Sub_la_2dof_ctrl_obs_195;

// For Block la_2dof_ctrl_obs/Publish1
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray> Pub_la_2dof_ctrl_obs_201;

// For Block la_2dof_ctrl_obs/Publish2
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64> Pub_la_2dof_ctrl_obs_206;

void slros_node_init(int argc, char** argv);

#endif
