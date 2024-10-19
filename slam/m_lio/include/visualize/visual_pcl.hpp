/**
 * @file visual_pcl.hpp
 * @brief 使用PCL可视化点云
*/

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// others
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>    // pcl的ros转换
#include <pcl/visualization/cloud_viewer.h>     // pcl可视化

namespace m_lio {
    void pcl_callback( const sensor_msgs::PointCloud2ConstPtr& msg );
}