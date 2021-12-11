//
// Created by Zhang Zhimeng on 2021/11/26.
//

#include "path_searcher/utility/map_tool.h"
#include "path_searcher/a_star/a_star_2d_flow.h"

#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>

AStar2DFlow::AStar2DFlow(ros::NodeHandle &nh) {
    astar_searcher_ptr_ = std::make_shared<AStar2D>();
    pointcloud_sub_ = std::make_shared<PointcloudSubscriber>(nh, "/map_generator_2d/global_pointcloud_map", 5);
    init_pose_sub_ = std::make_shared<InitPoseSubscriber2D>(nh, "/initialpose", 5);
    grid_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 10);
    visited_nodes_pub_ = nh.advertise<nav_msgs::GridCells>("visited_nodes", 10);

    double map_size_x = nh.param("map/x_size", 10.0);
    double map_size_y = nh.param("map/y_size", 10.0);
    double resolution = nh.param("map/resolution", 0.05);
    bool allow_diagonal = nh.param("planning/allow_diagonal", true);
    start_x_ = nh.param("planning/start_x", 0.0);
    start_y_ = nh.param("planning/start_y", 0.0);

    double x_lower = -map_size_x * 0.5;
    double x_upper = map_size_x * 0.5;
    double y_lower = -map_size_y * 0.5;
    double y_upper = map_size_y * 0.5;
    astar_searcher_ptr_->Init(x_lower, x_upper, y_lower, y_upper, resolution, allow_diagonal);

    has_map_ = false;
}

void AStar2DFlow::Run() {
    ReadData();

    if (!has_map_) {
        if (deque_map_.empty()) {
            return;
        }

        current_map_ptr_ = deque_map_.front();
        deque_map_.pop_front();
        InitGridMapObstacle2D<AStar2DPTR>(current_map_ptr_, astar_searcher_ptr_, grid_map_pub_);

        has_map_ = true;
        timestamp_ = current_map_ptr_->header.stamp;
    }
    deque_map_.clear();

    while (HasInitPoseData()) {
        InitPoseData();

        Vec2d start_pt(start_x_, start_y_);
        Vec2d goal_pt(
                current_init_pose_ptr_->pose.pose.position.x,
                current_init_pose_ptr_->pose.pose.position.y
        );

        if (astar_searcher_ptr_->Search(start_pt, goal_pt)) {
            PublishPath(astar_searcher_ptr_->GetPath());
            PublishVisitedNodes(astar_searcher_ptr_->GetVisitedNodeCoord());
        }
        astar_searcher_ptr_->Reset();
    }
}

bool AStar2DFlow::InitPoseData() {
    current_init_pose_ptr_ = deque_init_pose_.front();
    deque_init_pose_.pop_front();

    return true;
}

bool AStar2DFlow::HasInitPoseData() {
    return !deque_init_pose_.empty();
}

void AStar2DFlow::ReadData() {
    pointcloud_sub_->ParseData(deque_map_);
    init_pose_sub_->ParseData(deque_init_pose_);
}

void AStar2DFlow::PublishPath(const TypeVectorVecd<2> &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}

void AStar2DFlow::PublishVisitedNodes(const TypeVectorVecd<2> &visited_nodes_coord) {
    nav_msgs::GridCells grid_cells;
    grid_cells.header.frame_id = "world";
    grid_cells.cell_height = static_cast<float>(astar_searcher_ptr_->GetResolution());
    grid_cells.cell_width = static_cast<float>(astar_searcher_ptr_->GetResolution());

    geometry_msgs::Point pt;

    for (const auto &node_coord: visited_nodes_coord) {
        pt.x = node_coord.x();
        pt.y = node_coord.y();
        pt.z = 0.0f;

        grid_cells.cells.emplace_back(pt);
    }

    grid_cells.header.stamp = timestamp_;
    visited_nodes_pub_.publish(grid_cells);
}