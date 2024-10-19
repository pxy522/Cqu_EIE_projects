/**
 * @file visual_rgbd.hpp
 * @brief 使用OpenCV可视化RGBD图像
*/

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>  // OpenCV GUI

/**
 * /camara/color/image_raw 是RGB图像话题
 * /camara/depth/image_rect_raw 是深度图像话题
*/

namespace m_lio {
    void rgb_callback   ( const sensor_msgs::ImageConstPtr& msg );
    void depth_callback ( const sensor_msgs::ImageConstPtr& msg );
}