#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block nav3/Joystick_Subsystem/Subscribe
extern SimulinkSubscriber<sensor_msgs::Joy, SL_Bus_nav3_sensor_msgs_Joy> Sub_nav3_555;

// For Block nav3/Perception_Subsystem/Battery_Subsystem/Live_Subscriber
extern SimulinkSubscriber<std_msgs::Float32, SL_Bus_nav3_std_msgs_Float32> Sub_nav3_165;

// For Block nav3/Perception_Subsystem/Battery_Subsystem/Live_Subscriber1
extern SimulinkSubscriber<std_msgs::Float32, SL_Bus_nav3_std_msgs_Float32> Sub_nav3_166;

// For Block nav3/Perception_Subsystem/IMU_Subsystem/Live_Subscriber
extern SimulinkSubscriber<sensor_msgs::Imu, SL_Bus_nav3_sensor_msgs_Imu> Sub_nav3_187;

// For Block nav3/Perception_Subsystem/SLAM_Subsystem/Live_Subscriber
extern SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_nav3_nav_msgs_Odometry> Sub_nav3_202;

// For Block nav3/Perception_Subsystem/Simulation_Subsystem/Simulation_Subscriber
extern SimulinkSubscriber<geometry_msgs::PoseWithCovarianceStamped, SL_Bus_nav3_geometry_msgs_PoseWithCovarianceStamped> Sub_nav3_223;

// For Block nav3/Service_Request_Subsystem/RESET_IMU/Subscribe
extern SimulinkSubscriber<std_msgs::Int16, SL_Bus_nav3_std_msgs_Int16> Sub_nav3_499;

// For Block nav3/Actuation_Subsystem/Real_Actuation/Actuator/Publish
extern SimulinkPublisher<geometry_msgs::Twist, SL_Bus_nav3_geometry_msgs_Twist> Pub_nav3_442;

// For Block nav3/Actuation_Subsystem/Simulation_Actuation/Publish
extern SimulinkPublisher<geometry_msgs::Twist, SL_Bus_nav3_geometry_msgs_Twist> Pub_nav3_422;

// For Block nav3/Logging_Subsystem/Publish
extern SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_nav3_std_msgs_Float64MultiArray> Pub_nav3_516;

// For Block nav3/Service_Request_Subsystem/RESET_IMU/Publish
extern SimulinkPublisher<std_msgs::Int16, SL_Bus_nav3_std_msgs_Int16> Pub_nav3_494;

// For Block nav3/Parameter_Server_Subsystem/Control_Parameters/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_242;

// For Block nav3/Parameter_Server_Subsystem/Control_Parameters/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_274;

// For Block nav3/Parameter_Server_Subsystem/Control_Parameters/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_275;

// For Block nav3/Parameter_Server_Subsystem/Control_Parameters/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_276;

// For Block nav3/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_237;

// For Block nav3/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_325;

// For Block nav3/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_326;

// For Block nav3/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_327;

// For Block nav3/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_365;

// For Block nav3/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_368;

// For Block nav3/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter6
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_541;

// For Block nav3/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_340;

// For Block nav3/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter1
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_341;

// For Block nav3/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter2
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_342;

// For Block nav3/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter3
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_357;

// For Block nav3/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter4
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_358;

// For Block nav3/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter5
extern SimulinkParameterGetter<real64_T, double> ParamGet_nav3_359;

// For Block nav3/Parameter_Server_Subsystem/State_Parameters/Get Parameter1
extern SimulinkParameterGetter<int32_T, int> ParamGet_nav3_292;

// For Block nav3/Parameter_Server_Subsystem/State_Parameters/Get Parameter2
extern SimulinkParameterGetter<int32_T, int> ParamGet_nav3_293;

// For Block nav3/Parameter_Server_Subsystem/State_Parameters/Get Parameter3
extern SimulinkParameterGetter<int32_T, int> ParamGet_nav3_294;

// For Block nav3/Parameter_Server_Subsystem/State_Parameters/Get Parameter4
extern SimulinkParameterGetter<int32_T, int> ParamGet_nav3_295;

// For Block nav3/Parameter_Server_Subsystem/State_Parameters/Get Parameter5
extern SimulinkParameterGetter<int32_T, int> ParamGet_nav3_303;

// For Block nav3/Parameter_Server_Subsystem/State_Parameters/Get Parameter6
extern SimulinkParameterGetter<int32_T, int> ParamGet_nav3_306;

// For Block nav3/Parameter_Server_Subsystem/System_Parameters/Get Parameter5
extern SimulinkParameterGetter<int32_T, int> ParamGet_nav3_381;

// For Block nav3/Service_Request_Subsystem/RESET_IMU/RESET_IMU/Set Parameter
extern SimulinkParameterSetter<real64_T, double> ParamSet_nav3_475;

void slros_node_init(int argc, char** argv);

#endif