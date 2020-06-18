#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "left_arm_simul";

// For Block left_arm_simul/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_left_arm_simul_std_msgs_Float64MultiArray> Sub_left_arm_simul_160;

// For Block left_arm_simul/Publish1
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_simul_std_msgs_Float64MultiArray> Pub_left_arm_simul_163;

// For Block left_arm_simul/Publish2
SimulinkPublisher<sensor_msgs::JointState, SL_Bus_left_arm_simul_sensor_msgs_JointState> Pub_left_arm_simul_191;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}
