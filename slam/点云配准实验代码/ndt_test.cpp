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
    /* 读取命令行输入并将参数写入ndt的参数中 */
    int opt = 0;
    float leaf_size = 0.2;
    float step_size = 0.1;
    float resolution = 1.0;
    int max_iterations = 100;
    float epsilon = 0.01;
    while ( (opt = getopt( argc, argv, "hl:s:r:i:e" )) != -1 )
    {
        switch (opt)
        {
            case 'h':
                std::cout << "Usage: " << argv[0] << " [-l 体素大小] [-s 步长] [-r ndt网格分辨率] [-i 最大迭代次数] [-e 迭代终止条件]" << std::endl;
                return 0;
            case 'l':
                leaf_size = atof(optarg);
                break;
            case 's':
                step_size = atof(optarg);
                break;
            case 'r':
                resolution = atof(optarg);
                break;
            case 'i':
                max_iterations = atoi(optarg);
                break;
            case 'e':
                epsilon = atof(optarg);
                break;
            default:
                std::cout << "Usage: " << argv[0] << " [-l leaf_size] [-s step_size] [-r resolution] [-i max_iterations]" << std::endl;
                return 0;
        }
    }
    

    /* 加载目标点云数据并输出是否读取到点云 */
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/room_scan1.pcd", *target_cloud) == -1)
    {
        PCL_ERROR("Could not find pcd \n");
        return (-1);
    }
    std::cout << "load " << target_cloud->size() << " data points from target_cloud" << endl;
    /* 加载源点云数据并输出是否读取到点云 */
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/room_scan2.pcd", *source_cloud) == -1)
    {
        PCL_ERROR("Could not find pcd \n");
        return (-1);
    }
    std::cout << "load " << source_cloud->size() << " data points from source_cloud" << endl;

    /*----------------------------------------    点云滤波     ---------------------------------------------------*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);   // 滤波后的点云
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;                      // 体素滤波器  

    approximate_voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);                  // 设置体素滤波器的体素大小
    approximate_voxel_filter.setInputCloud(source_cloud);                                   // 设置输入点云
    approximate_voxel_filter.filter(*filter_cloud);                                         // 执行滤波，存储结果到filter_cloud

    std::cout << "Filter cloud contains " << filter_cloud->size() << "data points from filtered source_cloud" << endl; // 目标点云滤波后的点数

    /*----------------------------------------    点云滤波     ---------------------------------------------------*/

    Eigen::AngleAxisf init_rotation(0.69, Eigen::Vector3f::UnitZ());                        // 初始化旋转
    Eigen::Translation3f init_translaition(1.0, 0, 0);                                     // 初始化平移
    Eigen::Matrix4f init_guss = (init_translaition * init_rotation).matrix();              // 初始化高斯分布

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);   // 输出点云

    // =======================   ndt   ======================= //
    
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;                    // NDT 配准对象
    
    // 设置 NDT 参数
    ndt.setTransformationEpsilon(epsilon);       // 收敛条件
    ndt.setStepSize(step_size);                     // 优化步长
    ndt.setResolution(resolution);                   // NDT 网格分辨率
    ndt.setMaximumIterations(max_iterations);             // 最大迭代次数

    // 设置输入源点云和目标点云
    ndt.setInputSource(filter_cloud);
    ndt.setInputTarget(target_cloud);

    // 结果点云和变换矩阵
    Eigen::Matrix4f T_ndt = Eigen::Matrix4f::Identity();

    // 进行配准
    ndt.align(*output_cloud, init_guss);
    T_ndt = ndt.getFinalTransformation();
    pcl::transformPointCloud(*source_cloud, *output_cloud, T_ndt);

    // 输出 NDT 结果
    std::cout << "============== NDT =================" << std::endl;
    std::cout << "NDT 变换矩阵 T: \n" << T_ndt << std::endl;
    std::cout << "NDT 匹配得分: " << ndt.getFitnessScore() << std::endl;

    if (ndt.hasConverged()) {
        std::cout << "NDT 已收敛。" << std::endl;
    } else {
        std::cout << "NDT 未收敛。" << std::endl;
    }
    
    // =======================   ndt   ======================= //
    

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