// LaserScanner.cpp
// A Laser scanner with associated utility functions.

#include "LaserScanner.h"

LaserScanner::LaserScanner() : lastScan(NULL)
{
  laser_sub_ = nh_.subscribe("scan", 10, &LaserScanner::laserCb, this);
}

LaserScanner::~LaserScanner()
{
}

// Return true if functions are ready to be called with valid results
void LaserScanner::laserCb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  lastScan = msg;
}

// Return true if functions are ready to be called with valid results
bool LaserScanner::isReady()
{
  // Check if any data has been received
  if (lastScan == NULL)
  {
    return false;
  }
  return true;
}

// Get the minimum value in a certain angular range.
double LaserScanner::GetMinInRange(int left, int right)
{
  // Wait until data has been received
  while (isReady() == false)
  {
  };

  // Get the minimum range of the laser scan
  double range_min = lastScan->range_max;

  for (int num = left; num < right; num++)
  {
    if (!std::isinf(lastScan->ranges.at(num)) && lastScan->ranges.at(num) < range_min)
    {
      range_min = lastScan->ranges.at(num);
    }
  }
  return range_min;
}