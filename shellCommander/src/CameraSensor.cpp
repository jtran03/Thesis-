// CameraSensor.cpp
// A camera sensor that detects if the robot should stop.

#include "CameraSensor.h"

CameraSensor::CameraSensor() : it_(nh_)
{

    // The camera is not ready
    camera_state = false;
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &CameraSensor::imageCb, this);
};

CameraSensor::~CameraSensor()
{

    cv::destroyWindow("Original Video");
    cv::destroyWindow("Filtered Video");
};

// Return true if functions are ready to be called with valid results
bool CameraSensor::isReady()
{
    return camera_state;
}

// Callback for the subscriber
void CameraSensor::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    // Get the image into openCV
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        camera_state = false;
        return;
    }

    // Convert to HSV color space so we can filter out the colors
    cv::cvtColor(cv_ptr->image, hsvImg, cv::COLOR_BGR2HSV);

    // Filter out the stop color (orangeish)
    cv::inRange(hsvImg, cv::Scalar(0, 100, 100), cv::Scalar(50, 255, 255), hsvImg);

    // Do a binary threshold to help with contours
    cv::threshold(hsvImg, hsvImg, 150, 255, cv::THRESH_BINARY);

    // Find the contours so we can find the red areas
    cv::findContours(hsvImg, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    // Camera is ready
    camera_state = true;
}

// Return true if the camera can see the stop colour (orange/red)
bool CameraSensor::canSeeRed()
{
    // Check all the contours to see if there are any large ones that are big stop signs
    bool result = false;
    for (int c = 0; c < contours.size(); c++)
    {
        // bigger than 1/4 of screen
        if (cv::contourArea(contours[c]) > cv_ptr->image.cols * cv_ptr->image.rows / 2)
        {
            result = true;
        }
    }
    return result;
}

// Draw the camera view, for debugging purposes
void CameraSensor::draw()
{
    if (isReady() == true)
    {
        cv::imshow("Original Video", cv_ptr->image);
        cv::imshow("Filtered Video", hsvImg);
        cv::waitKey(3);
    }
}
