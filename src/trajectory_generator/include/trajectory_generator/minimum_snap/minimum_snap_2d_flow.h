//
// Created by Zhang Zhimeng on 2021/11/29.
//

#ifndef TRAJECTORY_GENERATOR_MINIMUM_SNAP_2D_FLOW_H
#define TRAJECTORY_GENERATOR_MINIMUM_SNAP_2D_FLOW_H

#include "trajectory_generator/minimum_snap/minimum_snap.h"
#include "trajectory_generator/subscriber/path_subscriber.h"
#include "trajectory_generator/subscriber/pointcloud_subscriber.h"

#include <ros/ros.h>

class MinimumSnap2DFlow {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MinimumSnap2DFlow() = delete;

    explicit MinimumSnap2DFlow(ros::NodeHandle &node_handle);

    void Run();

private:
    inline bool HasPathData() const;

    inline bool HasMapData() const;

    void ReadData();

    void PublishTrajectory(const MatXd &poly_coeff_mat, const VecXd &segments_time);

    Vec2d GetPositionPolynomial(const MatXd &poly_coeff_mat, unsigned int k, double t);

    MatXd PickWaypoints(const nav_msgs::PathPtr &path_ptr) const;

private:
    std::shared_ptr<MinimumSnap> minimum_snap_ptr_;
    std::shared_ptr<PathSubscriber> path_sub_ptr_;
    std::shared_ptr<PointcloudSubscriber> pointcloud_sub_ptr_;

    ros::Publisher trajectory_pub_;

    std::deque<sensor_msgs::PointCloud2Ptr> deque_map_;
    std::deque<nav_msgs::PathPtr> deque_path_;

    nav_msgs::PathPtr current_path_ptr_;
    sensor_msgs::PointCloud2Ptr current_map_ptr_;

    ros::Time timestamp_;
};

#endif //TRAJECTORY_GENERATOR_MINIMUM_SNAP_2D_FLOW_H