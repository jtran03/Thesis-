// MazeSolverBot.cpp
// A robot that solves Mazes, based on the Turtlebot3 hardware.

#include "MazeSolverBot.h"
#include <iostream>
#include <string>
#include <algorithm>

MazeSolverBot::MazeSolverBot() : camera(),
                                 scanner(),
                                 chassis()
{
}

MazeSolverBot::~MazeSolverBot()
{
}

// Various constants that determine how the bot solves the maze
void MazeSolverBot::doSolveMaze()
{
  ros::Rate loop_rate(125);

  // Wait for other components to become ready
  while (!(camera.isReady() && scanner.isReady()))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Transient radius of the spiral for searching. Reset to zero in normal operation.
  double spiralRadius = 0;

  // Minimum ranges
  double forward_range_min;
  double left_range_min;

  // Stop when we can see red
  while (ros::ok() && !camera.canSeeRed())
  {
    // Get the left minimum range
    left_range_min = scanner.GetMinInRange(left_range_min_angle, left_range_max_angle);

    // Forward-range consists of two ranges, so manually merge them here
    forward_range_min = std::min(
        scanner.GetMinInRange(0, forward_angle_halfrange),
        scanner.GetMinInRange(360 - forward_angle_halfrange, 360));

    std::stringstream infomsg;
    infomsg << "f:" << forward_range_min << ",s:" << left_range_min << "\n";

    // Check if we're too close to the wall in front
    if (forward_range_min < check_forward_dist_close)
    {
      // if so, move backwards
      spiralRadius = 0;
      infomsg << "Forward way too close: was" << forward_range_min << "vs" << check_forward_dist_close;
      chassis.setTwist(-1.0 * linear_velocity, 0.0);
    }
    // Check if we're somewhat close to a wall in front
    else if (forward_range_min < check_forward_dist)
    {
      // if so, turn right to avoid the wall, but slowly
      spiralRadius = 0;
      infomsg << "Forward too close: was" << forward_range_min << "vs" << check_forward_dist;
      chassis.setTwist(0.4 * linear_velocity, -2.0 * angular_velocity);
    }
    // Check if we're too close to the left wall
    else if (left_range_min < check_side_dist_close)
    {
      // if so, turn right to avoid the wall
      spiralRadius = 0;
      infomsg << "Left too close: was" << left_range_min << "vs"
              << check_side_dist_close << ", R=" << spiralRadius;
      chassis.setTwist(linear_velocity, -2.0 * angular_velocity);
    }
    // Check if we're too far from the left wall
    else if (left_range_min > check_side_dist)
    {
      // We haven't seen any walls nearby, we might be lost
      // Move in a slowly increasing spiral
      spiralRadius += 0.001;
      infomsg << "Left too far: was" << left_range_min << "vs" << check_side_dist
              << ", R=" << spiralRadius;
      chassis.setTwist(linear_velocity, 2.0 * angular_velocity / (1.0 + spiralRadius));
    }
    else
    {
      // Move forward! We're close to a wall on the left hand side
      spiralRadius = 0;
      infomsg << "Moving forward";
      chassis.setTwist(linear_velocity, 0.0);
    }

    // Debugging
    std::string _infomsg = infomsg.str();
    ROS_INFO(_infomsg.data());

    // Letting other threads activate
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Stop when done
  chassis.setTwist(0.0, 0.0);
  ros::spinOnce();
  loop_rate.sleep();
}