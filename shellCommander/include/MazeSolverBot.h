// MazeSolverBot.h
// A Maze Solving Robot.

#ifndef _maze_solver_bot_h
#define _maze_solver_bot_h

#include "CameraSensor.h"
#include "LaserScanner.h"
#include "RobotChassis.h"

class MazeSolverBot
{
public:
  MazeSolverBot();
  ~MazeSolverBot();

  // Block thread and solve the maze.
  void doSolveMaze();

private:
  CameraSensor camera;
  LaserScanner scanner;
  RobotChassis chassis;

  // Various constants that determine how the bot solves the maze

  // Angle range that is considered forward.
  static const int forward_angle_halfrange = 20;

  // Distance in front that should be clear, otherwise the bot will turn right
  static constexpr double check_forward_dist = 0.25;

  // Distance before the bot reverses
  static constexpr double check_forward_dist_close = 0.15;

  // Angle range that is considered to the left (technically forward-left).
  static const int left_range_min_angle = 70;
  static const int left_range_max_angle = 80;

  // Upper / lower bound distance that the bot will try to maintain with the wall
  static constexpr double check_side_dist = 0.23;
  static constexpr double check_side_dist_close = 0.17;

  // Rate at which the bot should spiral when lost
  static constexpr double spiral_angle_decrease_rate = 0.001;

  // Rate at which the bot moves
  static constexpr float linear_velocity = 0.2;
  static constexpr float angular_velocity = 0.4;
};

#endif