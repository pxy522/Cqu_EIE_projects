#include "m_lio/include/visualize/visual_pcl.hpp"
#include <pcl/point_types.h>
#include <memory>

namespace m_lio {

    void pcl_callback( const sensor_msgs::PointCloud2ConstPtr& msg ) 
    {
        std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>::ptr> cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg( *msg, *cloud );
        pcl::visualization::CloudViewer viewer( "Cloud Viewer" );
        viewer.showCloud( cloud );
        while ( !viewer.wasStopped() ) {
            ros::spinOnce();
        }
    }
}