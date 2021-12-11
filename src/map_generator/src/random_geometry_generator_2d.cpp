//
// Created by Zhang Zhimeng on 2021/11/24.
//
#include "map_generator/random_geometry_generator_2d.h"

#include <random>

RandomGeometryGenerator2D::RandomGeometryGenerator2D(
        float x_lower, float x_upper, float y_lower, float y_upper,
        float obs_w_lower, float obs_w_upper, float map_init_x,
        float map_init_y, float resolution)
        : x_lower_(x_lower), x_upper_(x_upper), y_lower_(y_lower), y_upper_(y_upper),
          obstacle_width_lower_(obs_w_lower), obstacle_width_upper_(obs_w_upper),
          map_init_x_(map_init_x), map_init_y_(map_init_y), resolution_(resolution) {
    GRID_X_SIZE_ = int((x_upper_ - x_lower_) / resolution_);
    GRID_Y_SIZE_ = int((y_upper_ - y_lower_) / resolution_);
}


PointCloudPtr RandomGeometryGenerator2D::Obstacle(unsigned int num_obstacle) const {
    PointCloudPtr point_cloud_ptr(new PointCloud);

    std::random_device random_device;
    std::mt19937 gen(random_device());

    std::uniform_real_distribution<float> rand_x(x_lower_, x_upper_);
    std::uniform_real_distribution<float> rand_y(y_lower_, y_upper_);
    std::uniform_real_distribution<float> rand_w(obstacle_width_lower_,
                                                 obstacle_width_upper_);

    PointXYZ point_xyz;
    for (unsigned int i = 0; i < num_obstacle; ++i) {
        float x, y, w_x, w_y;
        x = rand_x(gen);
        y = rand_y(gen);
        w_x = rand_w(gen);
        w_y = rand_w(gen);

        if (std::sqrt(std::pow(x - map_init_x_, 2) + std::pow(y - map_init_y_, 2)) < 0.5f) {
            continue;
        }

        x = std::floor(x / resolution_) * resolution_ + resolution_ * 0.5f;
        y = std::floor(y / resolution_) * resolution_ + resolution_ * 0.5f;

        float w_x_number = std::ceil(w_x / resolution_);
        float w_y_number = std::ceil(w_y / resolution_);

        for (float d_x = -w_x_number * 0.5f; d_x < w_x_number * 0.5;) {
            for (float d_y = -w_y_number * 0.5f; d_y < w_y_number * 0.5f;) {
                point_xyz.x = x + d_x * resolution_ + 0.001f;
                point_xyz.y = y + d_y * resolution_ + 0.001f;
                point_cloud_ptr->points.emplace_back(point_xyz);

                d_y += 1.0f;
            }
            d_x += 1.0f;
        }
    }

    point_cloud_ptr->width = point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    point_cloud_ptr->is_dense = true;

    return point_cloud_ptr;
}

nav_msgs::GridCells RandomGeometryGenerator2D::GenerateGridCells(const PointCloudPtr &point_cloud_ptr) {
    nav_msgs::GridCells grid_cells;
    grid_cells.header.frame_id = "world";
    grid_cells.cell_height = resolution_;
    grid_cells.cell_width = resolution_;

    geometry_msgs::Point obstacle;
    for (const auto &point: *point_cloud_ptr) {
        PointXYZ point_xyz;
        point_xyz = GridIndex2Coordinate(Coordinate2GridIndex(point));

        obstacle.x = point_xyz.x;
        obstacle.y = point_xyz.y;
        obstacle.z = 0.0f;

        grid_cells.cells.emplace_back(obstacle);
    }

    return grid_cells;
}

PointXYZ RandomGeometryGenerator2D::Coordinate2GridIndex(const PointXYZ &pt) {
    PointXYZ grid_point;

    grid_point.x = float(std::min(std::max(int((pt.x - x_lower_) / resolution_), 0), GRID_X_SIZE_ - 1));
    grid_point.y = float(std::min(std::max(int((pt.y - y_lower_) / resolution_), 0), GRID_Y_SIZE_ - 1));

    return grid_point;
}

PointXYZ RandomGeometryGenerator2D::GridIndex2Coordinate(const PointXYZ &pt) {
    PointXYZ coordinate_point;

    coordinate_point.x = (pt.x + 0.5f) * resolution_ + x_lower_;
    coordinate_point.y = (pt.y + 0.5f) * resolution_ + y_lower_;

    return coordinate_point;
}