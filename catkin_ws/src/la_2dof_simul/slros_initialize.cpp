#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "la_2dof_simul";

// For Block la_2dof_simul/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_la_2dof_simul_std_msgs_Float64MultiArray> Sub_la_2dof_simul_193;

// For Block la_2dof_simul/Publish1
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_la_2dof_simul_std_msgs_Float64MultiArray> Pub_la_2dof_simul_198;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

