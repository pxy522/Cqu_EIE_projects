#include "m_lio/include/method/GN_ICP.hpp"
#include "m_lio/include/method/utils.hpp"


GN_ICP::GN_ICP() : _kdtree_flann_ptr( std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>() ) { };


void GN_ICP::align( pcl::PointCloud<pcl::PointXYZ> &output, const Eigen::Matrix4f &predict ) 
{
    if ( _target_cloud_ptr == nullptr || _source_cloud_ptr == nullptr ) {
        std::cerr << __LINE__ << ": Target cloud or source cloud is nullptr!" << std::endl;
        return;
    }
    
    _final_transformation.setIdentity();                                                            // 初始化变换矩阵
    _final_transformation = predict;                                                                // 设置初始变换矩阵
    
    std::vector<int>    nn_indices( 1 );                                                            // 初始化最近邻索引
    std::vector<float>  nn_dists( 1 );                                                              // 初始化最近邻距离

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud( new pcl::PointCloud<pcl::PointXYZ> );

    int iter = 0;
    while ( iter <= _max_iterations && !_has_converge ) 
    {
        iter++;
        Eigen::Matrix<float, 3, 6>  J       = Eigen::Matrix<float, 3, 6>::Zero();                   // 初始化参数
        Eigen::Matrix<float, 6, 1>  B       = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 1>  delta_x = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 6>  Hessian = Eigen::Matrix<float, 6, 6>::Zero();

        pcl::transformPointCloud( *_source_cloud_ptr, *transformed_cloud, _final_transformation );  // 将源点云转换到目标点云坐标系下

        for ( size_t i = 0; i < transformed_cloud->size(); ++i )                                    // 遍历点云
        {   
            const pcl::PointXYZ &source_cloud_point = _source_cloud_ptr->points[i];
            const pcl::PointXYZ &transe_cloud_point = transformed_cloud->points[i];

            if ( pcl::isFinite( source_cloud_point ) == false ) {                                   // 判断点云是否有效
                continue;
            }

            _kdtree_flann_ptr->nearestKSearch( transe_cloud_point, 1, nn_indices, nn_dists );       // 搜索最近邻

            if ( nn_dists[0] > _max_correspond_distance ) {                                         // 判断最近邻距离是否大于最大对应点距离，即是否有效
                continue;
            }

            const pcl::PointXYZ &target_cloud_point = _target_cloud_ptr->points[nn_indices[0]];

            if ( !ComputeJacobiHessian( source_cloud_point, transe_cloud_point, target_cloud_point,  // 计算雅可比矩阵和海森矩阵
                                       Hessian, J, B )) continue; 

            delta_x = Hessian.ldlt().solve(B);  // 使用ldlt来求解线性方程组，这比直接求逆更加稳定和高效

            if ( delta_x.norm() < _transformation_epsilon ) {                                       // 判断是否收敛
                _has_converge = true;
                break;
            }
            
            updateTransformation( delta_x, _final_transformation );                                 // 更新变换矩阵
        }
    }
    pcl::transformPointCloud( *_source_cloud_ptr, output, _final_transformation );                  // 将output点云转换到目标点云坐标系下
}

/**
 *@brief: 计算雅可比矩阵和海森矩阵
 *@param: source_cloud_point: 源点云点
 *      transe_cloud_point: 转换后的源点云点 
 *      target_cloud_point: 目标点云点
 *      Hessian: 海森矩阵
 *      J: 雅可比矩阵
 *      B: B = -J^T * e
 *@return: Hessian行列式是否奇异，奇异返回false
*/
bool GN_ICP::ComputeJacobiHessian( const pcl::PointXYZ &source_cloud_point, 
                                   const pcl::PointXYZ &transe_cloud_point, 
                                   const pcl::PointXYZ &target_cloud_point,
                                   Eigen::Matrix<float, 6, 6> &Hessian, 
                                   Eigen::Matrix<float, 3, 6> & J, 
                                   Eigen::Matrix<float, 6, 1> & B ) {

    Eigen::Matrix3f     R = _final_transformation.block<3, 3>( 0, 0 );
    Eigen::Vector3f     t = _final_transformation.block<3, 1>( 0, 3 );
    Eigen::Vector3f     source_point_vector ( 
                        source_cloud_point.x, 
                        source_cloud_point.y, 
                        source_cloud_point.z ); 
    Eigen::Vector3f     e ( 
                        target_cloud_point.x - transe_cloud_point.x,
                        target_cloud_point.y - transe_cloud_point.y,
                        target_cloud_point.z - transe_cloud_point.z );

    J.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
    J.block<3,3>(0,3) = -R * Hat( source_point_vector );

    Hessian =   J.transpose()  * J;
    B       =   -J.transpose() * e;

    return Hessian.determinant() != 0;

}

void GN_ICP::updateTransformation( const Eigen::Matrix<float, 6, 1> &delta_x, Eigen::Matrix4f &T ) {
    Eigen::Matrix3f R = T.block<3, 3>( 0, 0 );
    Eigen::Vector3f t = T.block<3, 1>( 0, 3 );

    t += delta_x.head(3);
    R *= SO3Exp( delta_x.tail(3) ).matrix();

    T.block<3, 3>( 0, 0 ) = R;
    T.block<3, 1>( 0, 3 ) = t;
}

void GN_ICP::SetTargetCloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr & target_cloud_ptr ) 
{
    if ( target_cloud_ptr == nullptr ) {
        std::cerr << __LINE__ << ": Target cloud is nullptr!" << std::endl;
        return;
    }
    _target_cloud_ptr = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>( *target_cloud_ptr );
    _kdtree_flann_ptr->setInputCloud( target_cloud_ptr );
}

void GN_ICP::SetSourceCloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud_ptr ) 
{
    if ( source_cloud_ptr == nullptr ) {
        std::cerr << __LINE__ << ": Source cloud is nullptr!" << std::endl;
        return;
    }
    _source_cloud_ptr = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>( *source_cloud_ptr );
}

void GN_ICP::SetMaxIterations(unsigned int iter) {
    _max_iterations = iter;
}

void GN_ICP::SetTransformationEpsilon(float transformation_epsilon) {
    _transformation_epsilon = transformation_epsilon;
}

void GN_ICP::SetMaxCorrespondDistance(float max_correspond_distance) {
    _max_correspond_distance = max_correspond_distance;
}

Eigen::Matrix4f GN_ICP::GetFinalTransformation() {
    return _final_transformation;
}