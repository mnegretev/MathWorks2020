#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "la_2dof_ctrl_obs";

// For Block la_2dof_ctrl_obs/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray> Sub_la_2dof_ctrl_obs_193;

// For Block la_2dof_ctrl_obs/Subscribe1
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray> Sub_la_2dof_ctrl_obs_195;

// For Block la_2dof_ctrl_obs/Publish1
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64MultiArray> Pub_la_2dof_ctrl_obs_201;

// For Block la_2dof_ctrl_obs/Publish2
SimulinkPublisher<std_msgs::Float64, SL_Bus_la_2dof_ctrl_obs_std_msgs_Float64> Pub_la_2dof_ctrl_obs_206;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

