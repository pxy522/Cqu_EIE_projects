#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/room_scan1.pcd", *target_cloud) == -1)
    {
        PCL_ERROR("Could not find pcd \n");
        return (-1);
    }

    std::cout << "load " << target_cloud->size() << " data points from target_cloud" << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/room_scan2.pcd", *source_cloud) == -1)
    {
        PCL_ERROR("Could not find pcd \n");
        return (-1);
    }
    std::cout << "load " << source_cloud->size() << " data points from source_cloud" << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;

    approximate_voxel_filter.setLeafSize(0.3, 0.3, 0.3);
    approximate_voxel_filter.setInputCloud(source_cloud);
    approximate_voxel_filter.filter(*filter_cloud);

    std::cout << "Filter cloud contain " << filter_cloud->size() << "data points from source_cloud" << endl;

    Eigen::AngleAxisf init_rotation(0.69, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translasition(1.0, 0, 0);
    Eigen::Matrix4f init_guss = (init_translasition * init_rotation).matrix();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // =======================   ndt   =======================
    // 补全相关代码
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    
    // 设置 NDT 参数
    ndt.setTransformationEpsilon(0.01);       // 收敛条件
    ndt.setStepSize(0.1);                     // 优化步长
    ndt.setResolution(1.0);                   // NDT 网格分辨率
    ndt.setMaximumIterations(30);             // 最大迭代次数

    // 设置输入源点云和目标点云
    ndt.setInputSource(source_cloud);
    ndt.setInputTarget(target_cloud);

    // 结果点云和变换矩阵
    Eigen::Matrix4f T_ndt = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt_transformed_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // 进行配准
    ndt.align(*cloud_ndt_transformed_ptr, init_guss);
    T_ndt = ndt.getFinalTransformation();

    // 输出 NDT 结果
    std::cout << "============== NDT =================" << std::endl;
    std::cout << "NDT 变换矩阵 T: \n" << T_ndt << std::endl;
    std::cout << "NDT 匹配得分: " << ndt.getFitnessScore() << std::endl;

    if (ndt.hasConverged()) {
        std::cout << "NDT 已收敛。" << std::endl;
    } else {
        std::cout << "NDT 未收敛。" << std::endl;
    }
    
    // =======================   ndt   =======================
    

    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target_cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output_cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud");
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    return 0;
}