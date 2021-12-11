//
// Created by Zhang Zhimeng on 2021/11/29.
//

#ifndef TRAJECTORY_GENERATOR_TYPE_H
#define TRAJECTORY_GENERATOR_TYPE_H

#include <Eigen/Core>

#include <vector>

template<int dim>
using TypeVectorVecd = typename std::vector<Eigen::Matrix<double, dim, 1>,
        Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

typedef typename Eigen::MatrixXd MatXd;
typedef typename Eigen::VectorXd VecXd;

typedef typename Eigen::Vector2d Vec2d;
#endif //TRAJECTORY_GENERATOR_TYPE_H
