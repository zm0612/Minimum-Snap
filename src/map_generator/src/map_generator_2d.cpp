//
// Created by Zhang Zhimeng on 2021/11/24.
//

#include "map_generator/random_geometry_generator_2d.h"

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_generator_2d");
    ros::NodeHandle node_handle("~");

    ros::Publisher global_pointcloud_map_publisher =
            node_handle.advertise<sensor_msgs::PointCloud2>("global_pointcloud_map", 1);

    ros::Publisher global_grid_cells_publisher =
            node_handle.advertise<nav_msgs::GridCells>("global_grid_cells_map", 1);

    const unsigned int number_obstacle = node_handle.param("map/obstacle_number", 100);
    const float resolution = node_handle.param("map/resolution", 0.05f);
    const float sensor_rate = node_handle.param("sensor/rate", 0.5f);

    const float x_size = node_handle.param("map/x_size", 10.0f);
    const float y_size = node_handle.param("map/y_size", 10.0f);
    const float init_x = node_handle.param("init_x", 0.0f);
    const float init_y = node_handle.param("init_y", 0.0f);

    const float obstacle_w_lower = node_handle.param("Obstacle/lower_w", 0.3f);
    const float obstacle_w_upper = node_handle.param("Obstacle/upper_w", 1.0f);

    float x_lower = -x_size * 0.5f;
    float x_upper = x_size * 0.5f;
    float y_lower = -y_size * 0.5f;
    float y_upper = y_size * 0.5f;

    RandomGeometryGenerator2D random_geometry_generator_2d(
            x_lower, x_upper, y_lower, y_upper, obstacle_w_lower,
            obstacle_w_upper, init_x, init_y, resolution
    );

    PointCloudPtr point_cloud_ptr(new PointCloud);
    point_cloud_ptr = random_geometry_generator_2d.Obstacle(number_obstacle);

    nav_msgs::GridCells grid_cells = random_geometry_generator_2d.GenerateGridCells(point_cloud_ptr);

    sensor_msgs::PointCloud2 global_pointcloud_ros;
    pcl::toROSMsg(*point_cloud_ptr, global_pointcloud_ros);
    global_pointcloud_ros.header.frame_id = "world";
    global_pointcloud_ros.header.stamp = ros::Time::now();

    grid_cells.header.stamp = ros::Time::now();

    ros::Rate rate(sensor_rate);
    while (ros::ok()) {
        global_pointcloud_map_publisher.publish(global_pointcloud_ros);
        global_grid_cells_publisher.publish(grid_cells);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}