#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "left_arm_ctrl_obs";

// For Block left_arm_ctrl_obs/ROS Subscribers/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Sub_left_arm_ctrl_obs_299;

// For Block left_arm_ctrl_obs/ROS Subscribers/Subscribe1
SimulinkSubscriber<std_msgs::Float32MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float32MultiArray> Sub_left_arm_ctrl_obs_318;

// For Block left_arm_ctrl_obs/ROS Publishers/Publish1
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Pub_left_arm_ctrl_obs_304;

// For Block left_arm_ctrl_obs/ROS Publishers/Publish2
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_311;

// For Block left_arm_ctrl_obs/ROS Publishers/Publish3
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64MultiArray> Pub_left_arm_ctrl_obs_331;

// For Block left_arm_ctrl_obs/ROS Publishers1/Publish1
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_518;

// For Block left_arm_ctrl_obs/ROS Publishers1/Publish2
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_511;

// For Block left_arm_ctrl_obs/ROS Publishers1/Publish3
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_522;

// For Block left_arm_ctrl_obs/ROS Publishers2/Publish1
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_540;

// For Block left_arm_ctrl_obs/ROS Publishers2/Publish2
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_541;

// For Block left_arm_ctrl_obs/ROS Publishers2/Publish3
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_542;

// For Block left_arm_ctrl_obs/ROS Publishers3/Publish1
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_555;

// For Block left_arm_ctrl_obs/ROS Publishers3/Publish2
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_556;

// For Block left_arm_ctrl_obs/ROS Publishers3/Publish3
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_557;

// For Block left_arm_ctrl_obs/ROS Publishers3/Publish4
SimulinkPublisher<std_msgs::Float64, SL_Bus_left_arm_ctrl_obs_std_msgs_Float64> Pub_left_arm_ctrl_obs_578;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_368;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_370;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter10
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_403;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter11
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_404;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter12
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_405;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter13
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_406;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_372;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_374;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_376;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_378;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter6
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_380;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter7
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_400;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter8
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_401;

// For Block left_arm_ctrl_obs/ROS Params PD/Get Parameter9
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_402;

// For Block left_arm_ctrl_obs/ROS Params SMO/Get Parameter7
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_383;

// For Block left_arm_ctrl_obs/ROS Params SMO/Get Parameter8
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_384;

// For Block left_arm_ctrl_obs/ROS Params SMO/Get Parameter9
SimulinkParameterGetter<real64_T, double> ParamGet_left_arm_ctrl_obs_385;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

