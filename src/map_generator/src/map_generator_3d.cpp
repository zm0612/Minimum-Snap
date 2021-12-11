//
// Created by Zhang Zhimeng on 2021/11/23.
//

#include "map_generator/type.h"
#include "map_generator/random_geometry_generator_3d.h"

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_generator_3d");
    ros::NodeHandle node_handle("~");

    ros::Publisher global_pointcloud_map_publisher =
            node_handle.advertise<sensor_msgs::PointCloud2>("global_pointcloud_map", 1);
    ros::Publisher global_grid_map_publisher =
            node_handle.advertise<sensor_msgs::PointCloud2>("global_grid_map", 1);
    ros::Publisher circles_pointcloud_map_publisher =
            node_handle.advertise<sensor_msgs::PointCloud2>("circles_pointcloud_map", 1);
    ros::Publisher pillars_pointcloud_map_publisher =
            node_handle.advertise<sensor_msgs::PointCloud2>("pillars_pointcloud_map", 1);

    float x_size = node_handle.param("map/x_size", 10.0f);
    float y_size = node_handle.param("map/y_size", 10.0f);
    float z_size = node_handle.param("map/z_size", 2.0f);

    float ellipse_r_lower = node_handle.param("EllipseObstacle/lower_r", 0.6f);
    float ellipse_r_upper = node_handle.param("EllipseObstacle/upper_r", 2.0f);

    float pillar_h_lower = node_handle.param("PillarObstacle/lower_h", 0.5f);
    float pillar_h_upper = node_handle.param("PillarObstacle/upper_h", 2.5f);
    float pillar_w_lower = node_handle.param("PillarObstacle/lower_w", 2.5f);
    float pillar_w_upper = node_handle.param("PillarObstacle/upper_w", 2.5f);

    unsigned int ellipse_number = node_handle.param("map/ellipse_number", 45);
    unsigned int pillar_number = node_handle.param("map/pillar_number", 350);
    float resolution = node_handle.param("map/resolution", 0.1f);
    float map_init_x = node_handle.param("map_init_x", 0.0f);
    float map_init_y = node_handle.param("map_init_y", 0.0f);

    double sensor_rate = node_handle.param("sensor/rate", 1.0);

    float x_lower = -x_size * 0.5f;
    float x_upper = x_size * 0.5f;
    float y_lower = -y_size * 0.5f;
    float y_upper = y_size * 0.5f;
    float z_upper = z_size;

    RandomGeometryGenerator3D random_geometry_generator(
            x_lower, x_upper, y_lower, y_upper,
            0.0f, z_upper, map_init_x, map_init_y,
            ellipse_r_lower, ellipse_r_upper, resolution, pillar_h_lower, pillar_h_upper,
            pillar_w_lower, pillar_w_upper
    );

    ros::Time start_ellipse = ros::Time::now();
    PointCloudPtr pointcloud_circle_ptr = random_geometry_generator.Ellipse(ellipse_number);
    ros::Time end_ellipse = ros::Time::now();
    ROS_INFO_STREAM("Generate " << ellipse_number << " ellipses use time: "
                                << (end_ellipse - start_ellipse).toSec() * 1000.0 << " ms");

    ros::Time start_pillar = ros::Time::now();
    PointCloudPtr pointcloud_pillar_ptr = random_geometry_generator.Pillar(pillar_number);
    ros::Time end_pillar = ros::Time::now();
    ROS_INFO_STREAM("Generate " << pillar_number << " pillars use time: "
                                << (end_pillar - start_pillar).toSec() * 1000.0 << " ms");

    PointCloudPtr global_pointcloud_ptr(new PointCloud);
    *global_pointcloud_ptr = *pointcloud_circle_ptr + *pointcloud_pillar_ptr;

    ros::Time time_stamp = ros::Time::now();

    sensor_msgs::PointCloud2 circles_pointcloud_ros;
    pcl::toROSMsg(*pointcloud_circle_ptr, circles_pointcloud_ros);
    circles_pointcloud_ros.header.frame_id = "world";
    circles_pointcloud_ros.header.stamp = time_stamp;

    sensor_msgs::PointCloud2 pillars_pointcloud_ros;
    pcl::toROSMsg(*pointcloud_pillar_ptr, pillars_pointcloud_ros);
    pillars_pointcloud_ros.header.frame_id = "world";
    pillars_pointcloud_ros.header.stamp = time_stamp;

    sensor_msgs::PointCloud2 global_pointcloud_ros;
    pcl::toROSMsg(*global_pointcloud_ptr, global_pointcloud_ros);
    global_pointcloud_ros.header.frame_id = "world";
    global_pointcloud_ros.header.stamp = time_stamp;

    PointCloudPtr global_pointcloud_grid = random_geometry_generator.GenerateGridMap(global_pointcloud_ptr);
    sensor_msgs::PointCloud2 global_grid_map_ros;
    pcl::toROSMsg(*global_pointcloud_grid, global_grid_map_ros);
    global_grid_map_ros.header.frame_id = "world";
    global_grid_map_ros.header.stamp = time_stamp;

    ros::Rate rate(sensor_rate);
    while (ros::ok()) {
        global_pointcloud_map_publisher.publish(global_pointcloud_ros);
        circles_pointcloud_map_publisher.publish(circles_pointcloud_ros);
        pillars_pointcloud_map_publisher.publish(pillars_pointcloud_ros);
        global_grid_map_publisher.publish(global_grid_map_ros);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}