#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "nav3v2";

// For Block nav3v2/Joystick_Subsystem/Subscribe
SimulinkSubscriber<sensor_msgs::Joy, SL_Bus_nav3v2_sensor_msgs_Joy> Sub_nav3v2_555;

// For Block nav3v2/Perception_Subsystem/Battery_Subsystem/Live_Subscriber
SimulinkSubscriber<std_msgs::Float32, SL_Bus_nav3v2_std_msgs_Float32> Sub_nav3v2_165;

// For Block nav3v2/Perception_Subsystem/Battery_Subsystem/Live_Subscriber1
SimulinkSubscriber<std_msgs::Float32, SL_Bus_nav3v2_std_msgs_Float32> Sub_nav3v2_166;

// For Block nav3v2/Perception_Subsystem/IMU_Subsystem/Live_Subscriber
SimulinkSubscriber<sensor_msgs::Imu, SL_Bus_nav3v2_sensor_msgs_Imu> Sub_nav3v2_187;

// For Block nav3v2/Perception_Subsystem/Move_Base_Subsystem/Live_Subscriber
SimulinkSubscriber<geometry_msgs::Twist, SL_Bus_nav3v2_geometry_msgs_Twist> Sub_nav3v2_590;

// For Block nav3v2/Perception_Subsystem/SLAM_Subsystem/Live_Subscriber
SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_nav3v2_nav_msgs_Odometry> Sub_nav3v2_202;

// For Block nav3v2/Perception_Subsystem/Simulation_Subsystem/Simulation_Subscriber
SimulinkSubscriber<geometry_msgs::PoseWithCovarianceStamped, SL_Bus_nav3v2_geometry_msgs_PoseWithCovarianceStamped> Sub_nav3v2_223;

// For Block nav3v2/Service_Request_Subsystem/RESET_IMU/Subscribe
SimulinkSubscriber<std_msgs::Int16, SL_Bus_nav3v2_std_msgs_Int16> Sub_nav3v2_499;

// For Block nav3v2/Actuation_Subsystem/LED_Actuation/Publish2
SimulinkPublisher<std_msgs::Int32, SL_Bus_nav3v2_std_msgs_Int32> Pub_nav3v2_615;

// For Block nav3v2/Actuation_Subsystem/Real_Actuation/Actuator/Publish
SimulinkPublisher<geometry_msgs::Twist, SL_Bus_nav3v2_geometry_msgs_Twist> Pub_nav3v2_442;

// For Block nav3v2/Actuation_Subsystem/Simulation_Actuation/Publish
SimulinkPublisher<geometry_msgs::Twist, SL_Bus_nav3v2_geometry_msgs_Twist> Pub_nav3v2_422;

// For Block nav3v2/Logging_Subsystem/Publish
SimulinkPublisher<std_msgs::Float64MultiArray, SL_Bus_nav3v2_std_msgs_Float64MultiArray> Pub_nav3v2_516;

// For Block nav3v2/Logging_Subsystem/Publish1
SimulinkPublisher<std_msgs::Float32, SL_Bus_nav3v2_std_msgs_Float32> Pub_nav3v2_566;

// For Block nav3v2/Logging_Subsystem/Publish2
SimulinkPublisher<std_msgs::Int32, SL_Bus_nav3v2_std_msgs_Int32> Pub_nav3v2_570;

// For Block nav3v2/Service_Request_Subsystem/RESET_IMU/Publish
SimulinkPublisher<std_msgs::Int16, SL_Bus_nav3v2_std_msgs_Int16> Pub_nav3v2_494;

// For Block nav3v2/Parameter_Server_Subsystem/Control_Parameters/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_242;

// For Block nav3v2/Parameter_Server_Subsystem/Control_Parameters/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_274;

// For Block nav3v2/Parameter_Server_Subsystem/Control_Parameters/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_275;

// For Block nav3v2/Parameter_Server_Subsystem/Control_Parameters/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_276;

// For Block nav3v2/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_237;

// For Block nav3v2/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_325;

// For Block nav3v2/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_326;

// For Block nav3v2/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_327;

// For Block nav3v2/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_365;

// For Block nav3v2/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_368;

// For Block nav3v2/Parameter_Server_Subsystem/Experiment_Parameters/Get Parameter6
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_541;

// For Block nav3v2/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_340;

// For Block nav3v2/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter1
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_341;

// For Block nav3v2/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter2
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_342;

// For Block nav3v2/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter3
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_357;

// For Block nav3v2/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter4
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_358;

// For Block nav3v2/Parameter_Server_Subsystem/Robot_Parameters/Get Parameter5
SimulinkParameterGetter<real64_T, double> ParamGet_nav3v2_359;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter1
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_292;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter10
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_627;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter11
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_628;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter12
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_629;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter13
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_636;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter2
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_293;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter3
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_294;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter4
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_295;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter5
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_303;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter6
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_306;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter7
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_624;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter8
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_625;

// For Block nav3v2/Parameter_Server_Subsystem/State_Parameters/Get Parameter9
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_626;

// For Block nav3v2/Parameter_Server_Subsystem/System_Parameters/Get Parameter5
SimulinkParameterGetter<int32_T, int> ParamGet_nav3v2_381;

// For Block nav3v2/Service_Request_Subsystem/RESET_IMU/RESET_IMU/Set Parameter
SimulinkParameterSetter<real64_T, double> ParamSet_nav3v2_475;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

