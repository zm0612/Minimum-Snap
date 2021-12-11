//
// Created by Zhang Zhimeng on 2021/11/23.
//

#include "map_generator/random_geometry_generator_3d.h"
#include <algorithm>

RandomGeometryGenerator3D::RandomGeometryGenerator3D(float x_lower, float x_upper, float y_lower, float y_upper,
                                                     float z_lower, float z_upper, float map_init_x, float map_init_y,
                                                     float ellipse_r_lower, float ellipse_r_upper, float resolution,
                                                     float pillar_h_lower, float pillar_h_upper,
                                                     float pillars_w_lower, float pillars_w_upper)
        : x_lower_(x_lower), x_upper_(x_upper),
          y_lower_(y_lower), y_upper_(y_upper),
          z_lower_(z_lower), z_upper_(z_upper),
          ellipse_r_lower_(ellipse_r_lower), ellipse_r_upper_(ellipse_r_upper), pillar_h_lower_(pillar_h_lower),
          pillar_h_upper_(pillar_h_upper), pillars_w_lower_(pillars_w_lower), pillars_w_upper_(pillars_w_upper),
          map_init_x_(map_init_x), map_init_y_(map_init_y), resolution_(resolution) {

    GRID_X_SIZE_ = int((x_upper_ - x_lower_) / resolution_);
    GRID_Y_SIZE_ = int((y_upper_ - y_lower_) / resolution_);
    GRID_Z_SIZE_ = int((z_upper_ - z_lower_) / resolution_);
}

PointCloudPtr RandomGeometryGenerator3D::Ellipse(const unsigned int num_circle) const {
    PointCloudPtr point_cloud_ptr(new PointCloud);

    std::random_device random_device;
    std::mt19937 gen(random_device());

    std::uniform_real_distribution<float> rand_ellipse_axis_length(ellipse_r_lower_, ellipse_r_upper_);

    std::uniform_real_distribution<float> rand_x(x_lower_, x_upper_);
    std::uniform_real_distribution<float> rand_y(y_lower_, y_upper_);
    std::uniform_real_distribution<float> rand_z(z_lower_, z_upper_);

    std::uniform_real_distribution<float> rand_roll(-M_PI, M_PI);
    std::uniform_real_distribution<float> rand_pitch(M_PI / 4.0f, M_PI / 2.0f);
    std::uniform_real_distribution<float> rand_yaw(M_PI / 4.0f, M_PI / 2.0f);

    for (unsigned int i = 0; i < num_circle; ++i) {
        float x, y, z;
        VectorVec3f ellipse_points;

        x = rand_x(gen);
        y = rand_y(gen);
        z = rand_z(gen) / 2.0f;

        // Make sure that no circular obstacles are generated within two meters from the origin.
        if (std::sqrt(std::pow(x - map_init_x_, 2) + std::pow(y - map_init_y_, 2)) < 2.0f) {
            continue;
        }

        float roll, pitch, yaw;
        roll = rand_roll(gen);
        pitch = rand_pitch(gen);
        yaw = rand_yaw(gen);

        Eigen::Matrix3f R = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                             Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                             Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();

        float ellipse_x, ellipse_y, ellipse_z;
        Eigen::Vector3f pt3;
        PointXYZ point_xyz;

        float a, b;
        a = rand_ellipse_axis_length(gen);
        b = rand_ellipse_axis_length(gen);

        for (float theta = -M_PI; theta < M_PI;) {
            ellipse_x = a * std::cos(theta);
            ellipse_y = b * std::sin(theta);
            ellipse_z = 0.0f;

            pt3 << ellipse_x, ellipse_y, ellipse_z;

            pt3 = R * pt3;
            pt3.x() += x;
            pt3.y() += y;
            pt3.z() += z;

            if (pt3.z() >= 0.0f) {
                point_xyz.x = pt3.x();
                point_xyz.y = pt3.y();
                point_xyz.z = pt3.z();
                point_cloud_ptr->points.emplace_back(point_xyz);
            }
            theta += 0.025f;
        }
    }

    point_cloud_ptr->width = point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    point_cloud_ptr->is_dense = true;

    return point_cloud_ptr;
}

PointCloudPtr RandomGeometryGenerator3D::Pillar(unsigned int num_pillar) const {
    PointCloudPtr point_cloud_ptr(new PointCloud);

    std::random_device random_device;
    std::mt19937 gen(random_device());

    std::uniform_real_distribution<float> rand_x(x_lower_, x_upper_);
    std::uniform_real_distribution<float> rand_y(y_lower_, y_upper_);
    std::uniform_real_distribution<float> rand_w(pillars_w_lower_, pillars_w_upper_);
    std::uniform_real_distribution<float> rand_h(pillar_h_lower_, pillar_h_upper_);

    PointXYZ point_xyz;
    for (unsigned int i = 0; i < num_pillar; ++i) {
        float x, y, w, h;
        x = rand_x(gen);
        y = rand_y(gen);
        w = rand_w(gen);

        if (std::sqrt(std::pow(x - map_init_x_, 2) + std::pow(y - map_init_y_, 2)) < 0.8f) {
            continue;
        }

        x = std::floor(x / resolution_) * resolution_ + resolution_ * 0.5f;
        y = std::floor(y / resolution_) * resolution_ + resolution_ * 0.5f;

        float width_number = std::ceil(w / resolution_);
        for (float d_x = -width_number * 0.5f; d_x < width_number * 0.5f;) {
            for (float d_y = -width_number * 0.5f; d_y < width_number * 0.5f;) {
                h = rand_h(gen);
                float height_number = std::ceil(h / resolution_);
                for (float d_z = 0.0f; d_z < height_number;) {
                    point_xyz.x = x + d_x * resolution_ + 0.001f;
                    point_xyz.y = y + d_y * resolution_ + 0.001f;
                    point_xyz.z = d_z * resolution_ + 0.001f;
                    point_cloud_ptr->points.emplace_back(point_xyz);

                    d_z += 1.0f;
                }
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

PointCloudPtr RandomGeometryGenerator3D::GenerateGridMap(const PointCloudPtr &point_cloud_ptr) {
    PointCloudPtr grid_point_cloud_ptr(new PointCloud);

    for (auto &point: point_cloud_ptr->points) {
        PointXYZ point_xyz;
        point_xyz = GridIndex2Coordinate(Coordinate2GridIndex(point));
        grid_point_cloud_ptr->points.emplace_back(point_xyz);
    }

    grid_point_cloud_ptr->is_dense = true;
    grid_point_cloud_ptr->width = grid_point_cloud_ptr->points.size();
    grid_point_cloud_ptr->height = 1;
    return grid_point_cloud_ptr;
}

PointXYZ RandomGeometryGenerator3D::Coordinate2GridIndex(const PointXYZ &pt) const {
    PointXYZ grid_point;

    grid_point.x = float(std::min(std::max(int((pt.x - x_lower_) / resolution_), 0), GRID_X_SIZE_ - 1));
    grid_point.y = float(std::min(std::max(int((pt.y - y_lower_) / resolution_), 0), GRID_Y_SIZE_ - 1));
    grid_point.z = float(std::min(std::max(int((pt.z - z_lower_) / resolution_), 0), GRID_Z_SIZE_ - 1));

    return grid_point;
}

PointXYZ RandomGeometryGenerator3D::GridIndex2Coordinate(const PointXYZ &pt) const {
    PointXYZ coordinate_point;

    coordinate_point.x = (pt.x + 0.5f) * resolution_ + x_lower_;
    coordinate_point.y = (pt.y + 0.5f) * resolution_ + y_lower_;
    coordinate_point.z = (pt.z + 0.5f) * resolution_ + z_lower_;

    return coordinate_point;
}