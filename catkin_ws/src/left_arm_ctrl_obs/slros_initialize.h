#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block left_arm_ctrl_obs/ROS Subscribers/Subscribe
extern SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Sub_left_arm_ctrl_obs_299;

// For Block left_arm_ctrl_obs/ROS Subscribers/Subscribe1
extern SimulinkSubscriber<std_msgs::Float32MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray> Sub_left_arm_ctrl_obs_318;

// For Block left_arm_ctrl_obs/ROS Publishers/Publish1
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Pub_left_arm_ctrl_obs_304;

// For Block left_arm_ctrl_obs/ROS Publishers/Publish2
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_311;

// For Block left_arm_ctrl_obs/ROS Publishers/Publish3
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Pub_left_arm_ctrl_obs_331;

// For Block left_arm_ctrl_obs/ROS Publishers1/Publish1
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_518;

// For Block left_arm_ctrl_obs/ROS Publishers1/Publish2
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_511;

// For Block left_arm_ctrl_obs/ROS Publishers1/Publish3
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_522;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_368;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_370;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter10
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_403;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter11
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_404;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter12
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_405;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter13
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_406;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_372;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_374;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_376;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_378;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_380;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter7
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_400;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter8
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_401;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter9
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_402;

// For Block left_arm_ctrl_obs/ROS Params SMO/Get Parameter7
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_383;

// For Block left_arm_ctrl_obs/ROS Params SMO/Get Parameter8
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_384;

// For Block left_arm_ctrl_obs/ROS Params SMO/Get Parameter9
extern SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_385;

void slros_node_init(int argc, char** argv);

#endif
