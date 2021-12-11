//
// Created by Zhang Zhimeng on 2021/11/29.
//

#include "trajectory_generator/minimum_snap/minimum_snap_2d_flow.h"

#include <glog/logging.h>

MinimumSnap2DFlow::MinimumSnap2DFlow(ros::NodeHandle &node_handle) {
    path_sub_ptr_ = std::make_shared<PathSubscriber>(node_handle, "/run_astar_2d/searched_path", 5);
    pointcloud_sub_ptr_ = std::make_shared<PointcloudSubscriber>(
            node_handle, "/map_generator_2d/global_pointcloud_map", 5);
    trajectory_pub_ = node_handle.advertise<nav_msgs::Path>("generated_trajectory", 10);

    const double max_vel = node_handle.param("minimum_snap/max_vel", 1.0);
    const double max_accel = node_handle.param("minimum_snap/max_accel", 1.0);
    const unsigned int polynomial_order = node_handle.param("minimum_snap/order", 3);

    minimum_snap_ptr_ = std::make_shared<MinimumSnap>(polynomial_order, max_vel, max_accel);
}

void MinimumSnap2DFlow::Run() {
    ReadData();

    while (HasPathData()) {
        current_path_ptr_ = deque_path_.front();
        deque_path_.pop_front();

        ros::Time start_time = ros::Time::now();
        MatXd waypoint_posi = PickWaypoints(current_path_ptr_);
        VecXd waypoint_vel = VecXd::Zero(waypoint_posi.rows());
        VecXd waypoint_accel = VecXd::Zero(waypoint_posi.rows());

        VecXd time = minimum_snap_ptr_->AllocateTime(waypoint_posi);
        MatXd polynomial_coeff_x = minimum_snap_ptr_->SolveQPClosedForm(waypoint_posi.col(0),
                                                                        waypoint_vel, waypoint_accel, time);
        MatXd polynomial_coeff_y = minimum_snap_ptr_->SolveQPClosedForm(waypoint_posi.col(1),
                                                                        waypoint_vel, waypoint_accel, time);
        ros::Time end_time = ros::Time::now();

        ROS_INFO("\033[1;32m --> Time in Minimum Snap is %f ms\033[0m",
                 (end_time - start_time).toSec() * 1000.0);

        MatXd polynomial_coeff = MatXd::Zero(polynomial_coeff_x.rows(), polynomial_coeff_x.cols() * 2u);
        polynomial_coeff.leftCols(polynomial_coeff_x.cols()) = polynomial_coeff_x;
        polynomial_coeff.rightCols(polynomial_coeff_x.cols()) = polynomial_coeff_y;

        PublishTrajectory(polynomial_coeff, time);
    }
}

void MinimumSnap2DFlow::ReadData() {
    pointcloud_sub_ptr_->ParseData(deque_map_);
    path_sub_ptr_->ParseData(deque_path_);
}

bool MinimumSnap2DFlow::HasPathData() const {
    return !deque_path_.empty();
}

bool MinimumSnap2DFlow::HasMapData() const {
    return !deque_map_.empty();
}

MatXd MinimumSnap2DFlow::PickWaypoints(const nav_msgs::PathPtr &path_ptr) const {
    CHECK_GE(path_ptr->poses.size(), 2u);

    const unsigned int num_waypoint = path_ptr->poses.size();
    MatXd waypoints_mat = MatXd::Zero(num_waypoint, 2u);

    for (unsigned int i = 0; i < path_ptr->poses.size(); ++i) {
        waypoints_mat.block(i, 0, 1, 2) =
                Vec2d(path_ptr->poses[i].pose.position.x,
                      path_ptr->poses[i].pose.position.y).transpose();
    }

    return waypoints_mat;
}

Vec2d MinimumSnap2DFlow::GetPositionPolynomial(const MatXd &poly_coeff_mat, unsigned int k, double t) {
    Vec2d position;

    const unsigned int num_poly_coeff = minimum_snap_ptr_->GetPolyCoeffNum();

    for (unsigned int dim = 0; dim < 2u; ++dim) {
        VecXd coeff = (poly_coeff_mat.row(k)).segment(num_poly_coeff * dim, num_poly_coeff);
        VecXd time = VecXd::Zero(num_poly_coeff);

        for (unsigned int i = 0; i < num_poly_coeff; ++i) {
            if (i == 0) {
                time(i) = 1.0;
            } else {
                time(i) = pow(t, i);
            }
        }

        double temp_position = 0.0;
        for (unsigned int i = 0u; i < time.rows(); ++i) {
            temp_position = temp_position + coeff(i) * time(time.rows() - i - 1u);
        }

        position(dim) = temp_position;
    }

    return position;
}

void MinimumSnap2DFlow::PublishTrajectory(const MatXd &poly_coeff_mat,
                                          const VecXd &segments_time) {
    nav_msgs::Path trajectory;
    trajectory.header.frame_id = "world";
    trajectory.header.stamp = timestamp_;

    Vec2d temp_position;
    geometry_msgs::PoseStamped pose_stamped;
    for (unsigned int i = 0; i < segments_time.size(); ++i) {
        for (double t = 0.0; t < segments_time(i);) {
            temp_position = GetPositionPolynomial(poly_coeff_mat, i, t);

            pose_stamped.pose.position.x = temp_position.x();
            pose_stamped.pose.position.y = temp_position.y();
            pose_stamped.pose.position.z = 0.0;

            trajectory.poses.emplace_back(pose_stamped);

            t += 0.01;
        }
    }

    trajectory_pub_.publish(trajectory);
}