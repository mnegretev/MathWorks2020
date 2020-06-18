#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "left_arm_observer";

// For Block left_arm_observer/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_left_arm_observer_std_msgs_Float64MultiArray> Sub_left_arm_observer_299;

// For Block left_arm_observer/Publish1
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_observer_std_msgs_Float64MultiArray> Pub_left_arm_observer_304;

// For Block left_arm_observer/Publish2
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_observer_std_msgs_Float64> Pub_left_arm_observer_311;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

