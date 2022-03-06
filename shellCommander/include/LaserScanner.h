// LaserScanner.h
// A Laser scanner with associated utility functions.

#ifndef _laser_scanner_h
#define _laser_scanner_h

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserScanner
{
public:
  LaserScanner();
  ~LaserScanner();

  // Return true if functions are ready to be called with valid results
  bool isReady();

  // Get the minimum value in a certain angular range.
  double GetMinInRange(int left, int right);

private:
  // Callback for the laser subscriber.
  void laserCb(const sensor_msgs::LaserScan::ConstPtr &msg);

  // The last scan that was recieved (null at start).
  sensor_msgs::LaserScan::ConstPtr lastScan;

  // Subscriber to the laser scanner
  ros::Subscriber laser_sub_;

  // Store a handle to the ros node, so ros knows to hold the node open for our subscriber
  ros::NodeHandle nh_;
};

#endif