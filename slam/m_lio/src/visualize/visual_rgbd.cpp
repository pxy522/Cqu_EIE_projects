#include "m_lio/include/visualize/visual_rgbd.hpp"

namespace m_lio {
    void rgb_callback   ( const sensor_msgs::ImageConstPtr& msg ) {
        cv_bridge::CvImageConstPtr cv_ptr = nullptr;
        try {
            cv_ptr = cv_bridge::toCvShare( msg, sensor_msgs::image_encodings::BGR8 );
            cv::imshow( "RGB", cv_ptr->image );
        } catch ( cv_bridge::Exception& e ) {
            ROS_ERROR( "RGB cv_bridge exception: %s", e.what() );
            return;
        }
        if ( cv::waitKey( 1 ) == 27 ) {
            ros::shutdown();
        }
    }

    void depth_callback ( const sensor_msgs::ImageConstPtr& msg ) {
        cv_bridge::CvImageConstPtr cv_ptr = nullptr;
        try {
            cv_ptr = cv_bridge::toCvShare( msg, sensor_msgs::image_encodings::TYPE_16UC1 );
            cv::imshow( "Depth", cv_ptr->image );
        } catch ( cv_bridge::Exception& e ) {
            ROS_ERROR( "Depth cv_bridge exception: %s", e.what() );
            return;
        }
        if ( cv::waitKey( 1 ) == 27 ) {
            ros::shutdown();
        }
    }
}