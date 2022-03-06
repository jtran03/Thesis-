// RobotChassis.h
// A Robot Chassis that handles all the movement that the bot needs to do.

#ifndef _robot_chassis_h
#define _robot_chassis_h

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class RobotChassis
{
public:
  RobotChassis();
  ~RobotChassis();

  // Command the robot to move with a certain linear and angular velocity.
  void setTwist(float linear, float angular);

private:
  // Velocity Publisher
  ros::Publisher cmd_vel_pub_;

  // Store a handle to the ros node, so ros knows to hold the node open for our subscriber
  ros::NodeHandle nh_;
};

#endif