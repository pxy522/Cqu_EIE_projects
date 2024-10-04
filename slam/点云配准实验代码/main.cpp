#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>

#include "optimized_ICP_GN.h"

using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_ptr(new pcl::PointCloud<pcl::PointXYZ>);                   /*目标点云*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_ptr(new pcl::PointCloud<pcl::PointXYZ>);                   /*源点云*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_opti_transformed_ptr(new pcl::PointCloud<pcl::PointXYZ>);  /*优化ICP变换后的源点云*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_svd_transformed_ptr(new pcl::PointCloud<pcl::PointXYZ>);   /*SVD ICP变换后的源点云*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_ini_transformed_ptr(new pcl::PointCloud<pcl::PointXYZ>);   /*ICP变换后的源点云*/

    pcl::io::loadPCDFile("../data/room_scan2.pcd", *cloud_target_ptr);
    pcl::io::loadPCDFile("../data/room_scan1.pcd", *cloud_source_ptr);

    Eigen::Matrix4f T_predict, T_final;     /*初始变换矩阵和最终变换矩阵*/
    T_predict.setIdentity();                /*稍显冗余，但是为了保险起见，初始化为单位矩阵*/
    T_predict << 0.765, 0.643, -0.027, -1.472,
        -0.6, 0.765, -0.023, 1.3,
        0.006, 0.035, 0.999, -0.3,
        0, 0, 0, 1;

    pcl::transformPointCloud(*cloud_source_ptr, *cloud_source_ini_transformed_ptr, T_predict);
    pcl::transformPointCloud(*cloud_source_ptr, *cloud_source_svd_transformed_ptr, T_predict);

    std::cout << "Wait, matching..." << std::endl;

    // =======================   optimized icp   =======================
    // 补全相关代码
    OptimizedICPGN icp_optimized;
    
    // 设置目标点云
    icp_optimized.SetTargetCloud(cloud_target_ptr);
    
    // 最大迭代次数和对应点距离
    icp_optimized.SetMaxIterations(30);
    icp_optimized.SetMaxCorrespondDistance(0.3);
    icp_optimized.SetTransformationEpsilon(1e-5);

    // 优化ICP配准
    icp_optimized.Match(cloud_source_ptr, T_predict, cloud_source_opti_transformed_ptr, T_final);

    // 输出结果
    std::cout << "============== Optimized ICP =================" << std::endl;
    std::cout << "最终变换矩阵 T: \n" << T_final << std::endl;
    std::cout << "匹配得分(越小越好): " << icp_optimized.GetFitnessScore() << std::endl;
    
    if (icp_optimized.HasConverged()) {
        std::cout << "优化ICP已收敛。" << std::endl;
    } else {
        std::cout << "优化ICP未收敛。" << std::endl;
    }
    
    // =======================   optimized icp   =======================

    // =======================   gicp   =======================
        /* pcl 1.4.0 最新加入 */
        /* 待完善 */
    // =======================   gicp   =======================

    // =======================   svd icp   =======================
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_svd;
    icp_svd.setInputTarget(cloud_target_ptr);
    icp_svd.setInputSource(cloud_source_ptr);
    icp_svd.setMaxCorrespondenceDistance(0.3);
    icp_svd.setMaximumIterations(30);
    icp_svd.setEuclideanFitnessEpsilon(1e-4);
    icp_svd.setTransformationEpsilon(1e-4);
    icp_svd.align(*cloud_source_svd_transformed_ptr, T_predict);
    std::cout << "\n============== SVD ICP =================" << std::endl;
    std::cout << "T final: \n"
              << icp_svd.getFinalTransformation() << std::endl;
    std::cout << "fitness score: " << icp_svd.getFitnessScore() << std::endl;
    // =======================   svd icp   =======================

    // 可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->initCameraParameters();

    int v1(0);
    int v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Optimized ICP", 10, 10, "optimized icp", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_opti_color(
        cloud_source_opti_transformed_ptr, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_source_opti_transformed_ptr, source_opti_color, "source opti cloud", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color_0(cloud_target_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_target_ptr, target_color_0, "target cloud1", v1);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_ini_color(cloud_source_ini_transformed_ptr,
    //                                                                                  0, 255, 0);
    // viewer->addPointCloud<pcl::PointXYZ>(cloud_source_ini_transformed_ptr, source_ini_color, "source ini cloud", v1);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
    viewer->addText("SVD ICP", 10, 10, "svd icp", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color_1(cloud_target_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_target_ptr, target_color_1, "target cloud2", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_svd_color(cloud_source_svd_transformed_ptr,
                                                                                     0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_source_svd_transformed_ptr, source_svd_color, "source svd cloud", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source opti cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source svd cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target cloud2");
    viewer->addCoordinateSystem(1.0);

    viewer->setCameraPosition(0, 0, 20, 0, 10, 10, v1);
    viewer->setCameraPosition(0, 0, 20, 0, 10, 10, v2);

    viewer->spin();

    return 0;
}