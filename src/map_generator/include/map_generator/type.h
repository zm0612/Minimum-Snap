//
// Created by Zhang Zhimeng on 2021/11/23.
//

#ifndef MAP_GENERATOR_TYPE_H
#define MAP_GENERATOR_TYPE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

typedef typename pcl::PointXYZ PointXYZ;
typedef typename pcl::PointCloud<PointXYZ> PointCloud;
typedef typename pcl::PointCloud<PointXYZ>::Ptr PointCloudPtr;

typedef typename std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> VectorVec3f;

#endif //MAP_GENERATOR_TYPE_H
