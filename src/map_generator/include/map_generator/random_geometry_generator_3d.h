//
// Created by Zhang Zhimeng on 2021/11/23.
//

#ifndef MAP_GENERATOR_RANDOM_GEOMETRY_GENERATOR_3D_H
#define MAP_GENERATOR_RANDOM_GEOMETRY_GENERATOR_3D_H

#include "map_generator/type.h"

#include <Eigen/Core>
#include <pcl/search/kdtree.h>

#include <cmath>
#include <random>

class RandomGeometryGenerator3D {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RandomGeometryGenerator3D(float x_lower, float x_upper,
                              float y_lower, float y_upper,
                              float z_lower, float z_upper,
                              float map_init_x, float map_init_y,
                              float ellipse_r_lower, float ellipse_r_upper, float resolution,
                              float pillar_h_lower, float pillar_h_upper,
                              float pillars_w_lower, float pillars_w_upper);

    PointCloudPtr Ellipse(unsigned int num_circle) const;

    PointCloudPtr Pillar(unsigned int num_pillar) const;

    PointCloudPtr GenerateGridMap(const PointCloudPtr &point_cloud_ptr);

private:
    PointXYZ Coordinate2GridIndex(const PointXYZ &pt) const;

    PointXYZ GridIndex2Coordinate(const PointXYZ &pt) const;

private:
    float x_lower_, x_upper_;
    float y_lower_, y_upper_;
    float z_lower_, z_upper_;
    float ellipse_r_lower_, ellipse_r_upper_;
    float pillar_h_lower_, pillar_h_upper_;
    float pillars_w_lower_, pillars_w_upper_;
    float map_init_x_, map_init_y_;
    float resolution_;

    int GRID_X_SIZE_;
    int GRID_Y_SIZE_;
    int GRID_Z_SIZE_;
};

#endif //MAP_GENERATOR_RANDOM_GEOMETRY_GENERATOR_3D_H