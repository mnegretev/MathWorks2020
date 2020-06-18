#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block left_arm_observer/Subscribe
extern SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_left_arm_observer_std_msgs_Float64MultiArray> Sub_left_arm_observer_299;

// For Block left_arm_observer/Publish1
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_observer_std_msgs_Float64MultiArray> Pub_left_arm_observer_304;

// For Block left_arm_observer/Publish2
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_observer_std_msgs_Float64> Pub_left_arm_observer_311;

void slros_node_init(int argc, char** argv);

#endif
