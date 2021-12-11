//
// Created by Zhang Zhimeng on 2021/11/26.
//

#ifndef PATH_SEARCHER_MAP_TOOL_H
#define PATH_SEARCHER_MAP_TOOL_H

#include "path_searcher/utility/type.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <glog/logging.h>

template<typename SearcherPtr>
void InitGridMapObstacle2D(const sensor_msgs::PointCloud2Ptr &point_cloud_ros_ptr,
                           SearcherPtr &searcher, ros::Publisher &publisher) {

    TypePointCloud point_cloud_map;
    pcl::fromROSMsg(*point_cloud_ros_ptr, point_cloud_map);

    CHECK(!point_cloud_map.points.empty());

    sensor_msgs::PointCloud2 point_cloud_ros_vis;
    TypePointCloud point_cloud_map_vis;

    for (auto pt: point_cloud_map.points) {
        searcher->SetObstacle(pt.x, pt.y);
        Vec2d pt_rounding = searcher->CoordinateRounding(Vec2d(pt.x, pt.y));
        pt.x = static_cast<float>(pt_rounding.x());
        pt.y = static_cast<float>(pt_rounding.y());
        pt.z = 0.0f;

        point_cloud_map_vis.points.emplace_back(pt);
    }

    point_cloud_map_vis.width = point_cloud_map_vis.size();
    point_cloud_map_vis.height = 1u;
    point_cloud_map_vis.is_dense = true;

    pcl::toROSMsg(point_cloud_map_vis, point_cloud_ros_vis);
    point_cloud_ros_vis.header.frame_id = "world";

    publisher.template publish(point_cloud_ros_vis);
}

#endif //PATH_SEARCHER_MAP_TOOL_H
