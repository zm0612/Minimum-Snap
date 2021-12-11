//
// Created by Zhang Zhimeng on 2021/11/24.
//

#ifndef MAP_GENERATOR_RANDOM_GEOMETRY_GENERATOR_2D_H
#define MAP_GENERATOR_RANDOM_GEOMETRY_GENERATOR_2D_H

#include "map_generator/type.h"

#include <nav_msgs/GridCells.h>

class RandomGeometryGenerator2D {
public:
    RandomGeometryGenerator2D(float x_lower, float x_upper, float y_lower, float y_upper,
                              float obs_w_lower, float obs_w_upper, float map_init_x,
                              float map_init_y, float resolution);

    PointCloudPtr Obstacle(unsigned int num_obstacle) const;

    nav_msgs::GridCells GenerateGridCells(const PointCloudPtr &point_cloud_ptr);

    PointXYZ Coordinate2GridIndex(const PointXYZ &pt);

    PointXYZ GridIndex2Coordinate(const PointXYZ &pt);

private:
    float x_lower_, x_upper_;
    float y_lower_, y_upper_;
    float obstacle_width_lower_, obstacle_width_upper_;
    float map_init_x_, map_init_y_;
    float resolution_;

    int GRID_X_SIZE_;
    int GRID_Y_SIZE_;
};

#endif //MAP_GENERATOR_RANDOM_GEOMETRY_GENERATOR_2D_H
