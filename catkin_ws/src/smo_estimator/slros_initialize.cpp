#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "smo_estimator";

// For Block smo_estimator/ROS Subscribers/Subscribe
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_smo_estimator_std_msgs_Float64MultiArray> Sub_smo_estimator_299;

// For Block smo_estimator/ROS Subscribers/Subscribe1
SimulinkSubscriber<std_msgs::Float64MultiArray, SL_Bus_smo_estimator_std_msgs_Float64MultiArray> Sub_smo_estimator_638;

// For Block smo_estimator/ROS Publishers/Publish2
SimulinkPublisher<std_msgs::Float64, SL_Bus_smo_estimator_std_msgs_Float64> Pub_smo_estimator_311;

// For Block smo_estimator/ROS Publishers/Publish3
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_smo_estimator_std_msgs_Float64MultiArray> Pub_smo_estimator_331;

// For Block smo_estimator/ROS Params SMO/Get Parameter7
SimulinkParameterGetter<real64_T, double> ParamGet_smo_estimator_383;

// For Block smo_estimator/ROS Params SMO/Get Parameter8
SimulinkParameterGetter<real64_T, double> ParamGet_smo_estimator_384;

// For Block smo_estimator/ROS Params SMO/Get Parameter9
SimulinkParameterGetter<real64_T, double> ParamGet_smo_estimator_385;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

