// Copyright (c) 2020 Steve Macenski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
​
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string.h>
#include "../include/ip_connection.h"
#include "../include/brick_imu_v2.h"
​
#define HOST "localhost"
#define PORT 4223
​
ros::Publisher imuPub_;
std::string frameId_;
boost::array<const double, 9> zeros_ = { 0, 0, 0,
                                         0, 0, 0,
                                         0, 0, 0 };
​
void cb_quaternion(int16_t w, int16_t x, int16_t y, int16_t z, void * user_data)
{
  (void)user_data;
​
  sensor_msgs::Imu imuMsg;
  imuMsg.header.frame_id = frameId_;
  imuMsg.header.stamp = ros::Time::now();
​
  imuMsg.orientation.w = w/16383.0;
  imuMsg.orientation.x = x/16383.0;
  imuMsg.orientation.y = y/16383.0;
  imuMsg.orientation.z = z/16383.0;
​
  imuMsg.orientation_covariance = zeros_;
  imuMsg.orientation_covariance[0] = 1e-8;
  imuMsg.orientation_covariance[4] = 1e-8;
  imuMsg.orientation_covariance[8] = 1e-8;
​
  imuMsg.angular_velocity.x = 1;
  imuMsg.angular_velocity.y = 2;
  imuMsg.angular_velocity.z = 3;
​
  imuMsg.linear_acceleration.x = 4;
  imuMsg.linear_acceleration.y = 5;
  imuMsg.linear_acceleration.z = 6;
​
  //publish the message
  imuPub_.publish(imuMsg);
}
​
void cb_all_data(int16_t acceleration[3], int16_t magnetic_field[3],
                 int16_t angular_velocity[3], int16_t euler_angle[3],
                 int16_t quaternion[4], int16_t linear_acceleration[3],
                 int16_t gravity_vector[3], int8_t temperature,
                 uint8_t calibration_status, void *user_data) {
​
  (void)user_data;
​
  sensor_msgs::Imu imuMsg;
  imuMsg.header.frame_id = frameId_;
  imuMsg.header.stamp = ros::Time::now();
​
  imuMsg.orientation.w = quaternion[0]/16383.0;
  imuMsg.orientation.x = quaternion[1]/16383.0;
  imuMsg.orientation.y = quaternion[2]/16383.0;
  imuMsg.orientation.z = quaternion[3]/16383.0;
​
  imuMsg.orientation_covariance = zeros_;
  imuMsg.orientation_covariance[0] = 1e-8;
  imuMsg.orientation_covariance[4] = 1e-8;
  imuMsg.orientation_covariance[8] = 1e-8;
​
  imuMsg.angular_velocity.x = angular_velocity[0]/16.0;
  imuMsg.angular_velocity.y = angular_velocity[1]/16.0;
  imuMsg.angular_velocity.z = angular_velocity[2]/16.0;
​
  imuMsg.linear_acceleration.x = linear_acceleration[0]/100.0;
  imuMsg.linear_acceleration.y = linear_acceleration[1]/100.0;
  imuMsg.linear_acceleration.z = linear_acceleration[2]/100.0;
​
  //publish the message
  imuPub_.publish(imuMsg);
}
​
​
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "brick_imu_v2");
​
  int period;
  std::string uid;
  ros::NodeHandle nh("~");
  nh.param("frame_id", frameId_, std::string("imu"));
  nh.param("period_ms", period, 10);
  nh.param("uid", uid, std::string("6Det55"));
​
  imuPub_ = nh.advertise<sensor_msgs::Imu>("imu", 10);
​
  IMUV2 imu;
  IPConnection ipcon;
  ipcon_create(&ipcon);
  imu_v2_create(&imu, uid.c_str(), &ipcon);
  if(ipcon_connect(&ipcon, HOST, PORT) < 0)
  {
    ROS_WARN("Bricks IMU V2: Could not connect!");
    return 1;
  }
​
  imu_v2_register_callback(&imu,
                       IMU_V2_CALLBACK_ALL_DATA,
                       (void *)cb_all_data,
                       NULL);
  imu_v2_set_all_data_period(&imu, 100);
​
  ros::spin();
  ipcon_destroy(&ipcon);
  return 0;
}
