// CameraSensor.h
// A camera sensor with a few wrapper functions.

#ifndef _camera_sensor_h
#define _camera_sensor_h

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class CameraSensor
{
public:
  CameraSensor();
  ~CameraSensor();

  // Return true if the camera can see the stop colour (orange/red)
  bool canSeeRed();

  // Return true if functions are ready to be called with valid results
  bool isReady();

  // Draw the camera view, for debugging purposes
  void draw();

private:
  // Callback for the subscriber
  void imageCb(const sensor_msgs::ImageConstPtr &msg);

  // Store the last image that was recieved
  cv_bridge::CvImagePtr cv_ptr;

  // Store the image converted to HSV form (so we don't need to reprocess it)
  cv::Mat hsvImg;

  // Store the contours from the image, which will be used to calculate whether we should stop
  std::vector<std::vector<cv::Point>> contours;

  // Store a handle to the ros node, so ros knows to hold the node open for our subscriber
  ros::NodeHandle nh_;

  // An image transport, that contains some structs which give us information about the image
  image_transport::ImageTransport it_;

  // Our image subscriber.
  image_transport::Subscriber image_sub_;

  // Whether or not the camera is ready. Returned by isReady.
  bool camera_state;
};

#endif
