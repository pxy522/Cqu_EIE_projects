/**
 * @file utils.hpp
 * @brief 工具函数
 * 
 * @author pxy522
 * 
 * 部分代码参考自：
 * zm0612
 * @see https://github.com/zm0612/optimized_ICP.git
*/

#pragma once

#include <Eigen/Dense>


template <typename Derived>
Eigen::Matrix< typename Derived::Scalar, 3, 3 > Hat( const Eigen::MatrixBase<Derived> &v )
{
    Eigen::Matrix< typename Derived::Scalar, 3, 3 > hat;
    hat << 0, -v(2), v(1),
           v(2), 0, -v(0),
           -v(1), v(0), 0;
    return hat;
}

/**
 * @brief SO3李代数指数映射
 * 
 * @tparam Derived 
 * @param v 
 * @return Eigen::Matrix<typename Derived::Scalar, 3, 3> 
 * @see https://github.com/zm0612/optimized_ICP/blob/main/common.h
 */
template<typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> SO3Exp(const Eigen::MatrixBase<Derived> &v) 
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
    typename Derived::Scalar theta = v.norm(); // 计算v的模
    Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normalized = v.normalized(); // 归一化
    // 罗德里格斯公式
    R = cos(theta) * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + 
        (typename Derived::Scalar(1.0) - cos(theta)) * v_normalized * v_normalized.transpose() + 
        sin(theta) * Hat(v_normalized);

    return R;
}