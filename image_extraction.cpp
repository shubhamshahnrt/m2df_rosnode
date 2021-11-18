#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

std::chrono::steady_clock::time_point lastSampleTime;

void callback(const sensor_msgs::ImageConstPtr& image, const nav_msgs::Odometry::ConstPtr& odometry)
{
  auto newSampleTime = std::chrono::steady_clock::now();
  std::chrono::duration<float> gap = newSampleTime - lastSampleTime;
  float seconds = gap.count();
  if(seconds > 0.000001f) std::cout << "Duration: " << seconds << ":: frequency: " << 1.0f/seconds << std::endl;
  lastSampleTime = std::chrono::steady_clock::now();

  std::stringstream timestampSStreamImage; timestampSStreamImage << image->header.stamp;
  std::string timestampStringImage = timestampSStreamImage.str();
  std::stringstream timestampSStreamOdom; timestampSStreamOdom << image->header.stamp;
  std::string timestampStringOdom = timestampSStreamOdom.str();

  cv_bridge::CvImagePtr imagePtr;
  try {
    imagePtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //===========================================================================

  cv::Mat inputImg = imagePtr->image;
  cv::imwrite("/home/nrt/data/rgb/" + timestampStringImage + ".jpg", inputImg);

  //===========================================================================

  std::ofstream trajectoryFile("/home/nrt/data/trajectory.txt", std::ios::app);

  trajectoryFile << 
  timestampStringOdom << " " <<
  odometry->pose.pose.position.x << " " <<
  odometry->pose.pose.position.y << " " <<
  odometry->pose.pose.position.z << " " <<
  odometry->pose.pose.orientation.x << " " <<
  odometry->pose.pose.orientation.y << " " <<
  odometry->pose.pose.orientation.z << " " <<
  odometry->pose.pose.orientation.w << std::endl;

  trajectoryFile.close();

  //===========================================================================
}

int main(int argc, char** argv)
{
    lastSampleTime = std::chrono::steady_clock::now();

    ros::init(argc, argv, "image_extraction");
    ros::NodeHandle nodeHandle;

    message_filters::Subscriber<sensor_msgs::Image> imageSubscriber(nodeHandle, "/webcam/image_raw", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odometrySubscriber(nodeHandle, "/mavros/global_position/local", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> ImageOdomPolicy;
    message_filters::Synchronizer<ImageOdomPolicy> synchronizer(ImageOdomPolicy(10), imageSubscriber, odometrySubscriber);

    synchronizer.registerCallback(boost::bind(&callback, _1, _2));
    
    ros::spin();
    return 0;
}
