#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "slros_time.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block left_arm_simul/Subscribe
extern SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_left_arm_simul_std_msgs_Float64MultiArray> Sub_left_arm_simul_160;

// For Block left_arm_simul/Subscribe1
extern SimulinkSubscriber<std_msgs::Float64, SL_Bus_left_arm_simul_std_msgs_Float64> Sub_left_arm_simul_272;

// For Block left_arm_simul/Publish1
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_simul_std_msgs_Float64MultiArray> Pub_left_arm_simul_163;

// For Block left_arm_simul/Publish2
extern SimulinkPublisher<sensor_msgs::JointState, SL_Bus_left_arm_simul_sensor_msgs_JointState> Pub_left_arm_simul_191;

void slros_node_init(int argc, char** argv);

#endif
