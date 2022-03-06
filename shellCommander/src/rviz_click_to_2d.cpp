/*
 * Automatic Addison
 * Website: https://automaticaddison.com
 *   ROS node that converts the user's desired initial pose and goal location
 *   into a usable format.
 * Subscribe:
 *   initialpose : The initial position and orientation of the robot using
 *                 quaternions. (geometry_msgs/PoseWithCovarianceStamped)
 *   move_base_simple/goal : Goal position and
 *                           orientation (geometry_msgs::PoseStamped)
 * Publish: This node publishes to the following topics:
 *   goal_2d : Goal position and orientation (geometry_msgs::PoseStamped)
 *   initial_2d : The initial position and orientation of the robot using
 *                Euler angles. (geometry_msgs/PoseStamped)
 * From Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */

// Include statements
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <iostream>

// Initialize ROS publishers
ros::Publisher pub;

// Take move_base_simple/goal as input and publish goal_2d
void handle_goal(const geometry_msgs::PoseStamped &goal) {
  std::cout << "x = " << goal.pose.position.x << std::endl;
  std::cout << "y = " << goal.pose.position.y << std::endl;
  std::cout << "z = " << goal.pose.position.z << std::endl;
  std::cout << "thetaX = " << goal.pose.orientation.x << std::endl;
  std::cout << "thetaY = " << goal.pose.orientation.y << std::endl;
  std::cout << "thetaZ = " << goal.pose.orientation.z << std::endl;
  std::cout << "thetaW = " << goal.pose.orientation.w << std::endl;
  tf::Quaternion q(0, 0, goal.pose.orientation.z, goal.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
}

// Initialize ROS publishers
int main(int argc, char **argv) {

  ros::init(argc, argv, "rviz_click_to_2d");

  // Store a handle to the ros node, so ros knows to hold the node open for our subscriber
  ros::NodeHandle nh_;

  std::string rviz_topic_name = nh_.param<std::string>("rviz_click_to_2d", "move_base_simple/goal");
  pub = nh_.advertise<geometry_msgs::PoseStamped>(rviz_topic_name, 10);

  ros::Subscriber sub = nh_.subscribe("move_base_simple/goal", 0, handle_goal);
  ros::Rate loop_rate(10);


  // Set current position of robot as x = 0, y = 0
  int x = 0;
  int y = 0;

  // Create the first point of the square
  geometry_msgs::PoseStamped square_1;
  square_1.pose.position.x = 0.5;
  square_1.pose.position.y = 0.5;
  square_1.pose.position.z = 0;
  square_1.pose.orientation.x = 0;
  square_1.pose.orientation.y = 0;
  square_1.pose.orientation.z = 0;
  square_1.pose.orientation.w = 1;
  square_1.header.frame_id = "map";
  // square_1.header.stamp = ros::Time::now();


  int count = 1;
  while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        if (count == 1)
        {
            pub.publish(square_1);
            count++;
        }
  }
  return 0;
}

// inspired by markwsilliman/turtlebot repository

// #include <ros/ros.h>
//
// //The following line is where we import the ``MoveBaseAction`` library which is responsible for accepting goals from users and move the robot to the specified location in its world.
// #include <move_base_msgs/MoveBaseAction.h>
//
// #include <actionlib/client/simple_action_client.h>
//
// //this line is where we create the client that will communicate with actions that adhere to the base station interface
// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//
// //This is where you create the goal to send to move_base using move_base_msgs::MoveBaseGoal messages to tell the robot to move one meter forward in the coordinate frame.
// move_base_msgs::MoveBaseGoal goal;
//
// // Function
// void updateGoal(int x, int y, int w, int z);
//
// int main(int argc, char** argv){
//
//   ros::init(argc, argv, "map_navigation");
//
//   // Help!
//   MoveBaseClient ac("move_base", true);
//
//   //wait for the action server to come up and then start the process
//   while(!ac.waitForServer(ros::Duration(5.0))){
//     ROS_INFO("Waiting for the move_base action server to come up");
//   }
//
//   ros::Rate loop_rate(10);
//
//   int help;
// // While loop
// while (ros::ok()) {
//
//     ros::spinOnce();
//
//     loop_rate.sleep();
//
//     // Update a goal
//     updateGoal(-0.2, 0, 0, 1);
//
//     ROS_INFO("Sending goal");
//
//     //this command sends the goal to the move_base node to be processed
//     ac.sendGoal(goal);
//
//     //After finalizing everything you have to wait for the goal to finish processing
//     ac.waitForResult();
//
//     //here we check for the goal if it succeded or failed and send a message according to the goal status.
//     if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     {
//         ROS_INFO("Hooray, the base moved 1 meter forward");
//         help = 2;
//     }
//     else
//     {
//         ROS_INFO("The base failed to move forward 1 meter for some reason");
//     }
//
//     if (help == 2)
//     {
//       // Update a goal
//       updateGoal(0.2, 0, 1, 0);
//
//       ROS_INFO("Sending goal");
//
//       //this command sends the goal to the move_base node to be processed
//       ac.sendGoal(goal);
//
//       //After finalizing everything you have to wait for the goal to finish processing
//       ac.waitForResult();
//
//       //here we check for the goal if it succeded or failed and send a message according to the goal status.
//       if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//       {
//           ROS_INFO("Hooray, the base moved 1 meter forward");
//           help = 3;
//       }
//       else
//       {
//           ROS_INFO("The base failed to move forward 1 meter for some reason");
//       }
//
//     }
//
//   }
//
// return 0;
//
// }
//
// void updateGoal(int x, int y, int w, int z)
// {
//     //we'll send a goal to the robot to move 1 meter forward
//     goal.target_pose.header.frame_id = "base_link";
//     goal.target_pose.header.stamp = ros::Time::now();
//
//     goal.target_pose.pose.position.x = x;
//     goal.target_pose.pose.position.y = y;
//     goal.target_pose.pose.orientation.w = w;
//     goal.target_pose.pose.orientation.z = z;
// }
