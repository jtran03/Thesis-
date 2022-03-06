// RobotChassis.cpp
// The chassis for the robot.

#include "RobotChassis.h"

RobotChassis::RobotChassis()
{
    std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "cmd_vel");
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
}

RobotChassis::~RobotChassis()
{
}

// Command the robot to move with a certain linear and angular velocity.
void RobotChassis::setTwist(float linear, float angular)
{
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    cmd_vel_pub_.publish(cmd_vel);
}