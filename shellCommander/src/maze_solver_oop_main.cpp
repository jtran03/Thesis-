// maze_solver_oop_main.cpp
// Maze solver bot main file.

#include "MazeSolverBot.h"
#include <iostream>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  MazeSolverBot mazeSolver;
  mazeSolver.doSolveMaze();
  return 0;
}