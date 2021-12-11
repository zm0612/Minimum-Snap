//
// Created by Zhang Zhimeng on 2021/11/25.
//

#ifndef PATH_SEARCHER_TYPE_H
#define PATH_SEARCHER_TYPE_H

#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<int dim>
using TypeVectorVecd = typename std::vector<Eigen::Matrix<double, dim, 1>,
        Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

typedef typename pcl::PointXYZ TypePoint;
typedef typename pcl::PointCloud<TypePoint> TypePointCloud;

typedef typename Eigen::Vector2d Vec2d;
typedef typename Eigen::Vector3d Vec3d;

#endif //PATH_SEARCHER_TYPE_H
