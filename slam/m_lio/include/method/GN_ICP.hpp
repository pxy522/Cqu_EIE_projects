#pragma once
/**
 * @file GN_ICP.hpp
 * @brief GN_ICP class
 *
 * @author pxy522
 * @version 1.0
 * @date 2024-10-05
 * 
 * 参考自：
 * @see https://github.com/zm0612/optimized_ICP/
 *
*/

/* ICP点云配准算法流程 */
/* Step1: 粗配准得初始Rt */
/* Step2: 迭代n次后得到R^n t^n */
/* Step3: 源点云经过R^n t^n变换后，与目标点云作最邻居匹配得到点对 */
/* Step4: 根据点对P j 通过优化 argmin(R^n+1 t^n+1){ P^j - R^t@q^j + t^n } 得到R^n+1 t^n+1 */
/* Step5: 判断是否收敛，若收敛则停止，否则继续回到Step2 */

/* 这里我们使用高斯牛顿法进行优化求解 R^n+1 t^n+1 */
/* Gauss-Newton's method solve ICP: J^T*J*delta_x = -J^T*e */
/* J =  [ (de / dt) , (de / dR) ] */
/* e =  P - Rq + t */
/* de / dR = R hat(q) */
/* de / dt = -I */
/* J = [ I  -R hat(q) ] */
/* Hessian = J^T * J */
/* B = -J^T * e */
/* delta_x = Hessian^(-1) * B */
/* t^n+1 = t^n + delta_x[3:6] */
/* R^n+1 = R^n * exp(delta_x[0:3]) */


#include <memory>                   // 智能指针

#include <eigen3/Eigen/Core>        // Eigen库
#include <pcl/common/transforms.h>  // pcl库的变换
#include <pcl/kdtree/kdtree_flann.h> // pcl库的kdtree


class GN_ICP
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // eigen自动内存对齐
    
    GN_ICP(); // 构造函数
    
    void SetMaxIterations           ( unsigned int iter );                                                        // 设置最大迭代次数
    void SetTransformationEpsilon   ( float transformation_epsilon );                                             // 设置变换矩阵收敛条件
    void SetMaxCorrespondDistance   ( float max_correspond_distance );                                            // 设置最大对应点距离
    void SetTargetCloud             ( const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud_ptr );              // 设置目标点云
    void SetSourceCloud             ( const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud_ptr );              // 设置源点云
    void align                      ( pcl::PointCloud<pcl::PointXYZ> &output, const Eigen::Matrix4f &predict );   // ICP配准

    void updateTransformation       ( const Eigen::Matrix<float, 6, 1> &delta_x, Eigen::Matrix4f &T );            // 更新变换矩阵

    bool ComputeJacobiHessian       ( const pcl::PointXYZ &source_cloud_point,                                    // 计算雅可比矩阵和海森矩阵
                                      const pcl::PointXYZ &transe_cloud_point,
                                      const pcl::PointXYZ &target_cloud_point,
                                      Eigen::Matrix<float, 6, 6> &Hessian, 
                                      Eigen::Matrix<float, 3, 6> & Jacobi, 
                                      Eigen::Matrix<float, 6, 1> & B );                                  
    
    Eigen::Matrix4f GetFinalTransformation(); // 获取最终变换矩阵


private:
    unsigned int    _max_iterations{};              // 最大迭代次数, 默认为0, 使用{}进行初始化, 为了防止未初始化, 使用unsigned int, 无符号整型，防止出现负数
    float           _transformation_epsilon{};      // 变换矩阵收敛条件, 默认为0, 使用{}进行初始化, 为了防止未初始化
    float           _max_correspond_distance{};     // 最大对应点距离, 默认为0, 使用{}进行初始化, 为了防止未初始化
    bool            _has_converge = false;          // 是否收敛, 默认为false
    Eigen::Matrix4f _final_transformation;          // 最终变换矩阵
    
    std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> _target_cloud_ptr = nullptr; // 目标点云
    std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> _source_cloud_ptr = nullptr; // 源点云
    std::unique_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> _kdtree_flann_ptr = nullptr; // 用于搜索的kdtree

};