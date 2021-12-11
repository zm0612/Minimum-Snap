//
// Created by meng on 2020/9/9.
//

#include <fstream>
#include <cmath>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "trajectory_generator.h"

using namespace std;
using namespace Eigen;

double visualization_traj_width;
double Vel, Acc;
int dev_order;//cost函数对应的导数阶数, =3: 最小化jerk =4: 最小化snap
//当导数的阶数超过一定阶以后，数值发散的很快，此时数值可能不稳定了，
//主要原因是会超过int的数值精度范围，目前不能超过6
int min_order;

ros::Subscriber way_pts_sub;
ros::Publisher waypoint_traj_vis_pub, waypoint_path_vis_pub;

int poly_coeff_num;
MatrixXd poly_coeff;
VectorXd segment_traj_time;
Vector3d start_position = Vector3d::Zero();
Vector3d start_velocity = Vector3d::Zero();

void visWayPointTraj(MatrixXd polyCoeff, VectorXd time);

void visWayPointPath(MatrixXd path);

Vector3d getPosPoly(MatrixXd polyCoeff, int k, double t);

VectorXd timeAllocation(MatrixXd Path);

void TrajGeneration(const Eigen::MatrixXd &path);

void rcvWaypointsCallBack(const nav_msgs::Path &wp);

/*!
 * 订阅rviz发布的waypoints
 * @param wp
 */
void rcvWaypointsCallBack(const nav_msgs::Path &wp) {
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int) wp.poses.size(); k++) {
        Vector3d pt(wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if (wp.poses[k].pose.position.z < 0.0)
            break;
    }

    MatrixXd waypoints(wp_list.size() + 1, 3);
    waypoints.row(0) = start_position;

    for (int k = 0; k < (int) wp_list.size(); k++)
        waypoints.row(k + 1) = wp_list[k];

    TrajGeneration(waypoints);
}

void TrajGeneration(const Eigen::MatrixXd &path) {
    TrajectoryGeneratorTool trajectoryGeneratorWaypoint;

    MatrixXd vel = MatrixXd::Zero(2, 3);
    MatrixXd acc = MatrixXd::Zero(2, 3);

    vel.row(0) = start_velocity;
//    vel.row(1) = Vector3d(-1, -1, -1);//结束点的速度

    ros::Time start_time = ros::Time::now();
    segment_traj_time = timeAllocation(path);

    poly_coeff = trajectoryGeneratorWaypoint.SolveQPClosedForm(dev_order, path, vel, acc, segment_traj_time);
    ros::Duration use_time = (ros::Time::now() - start_time);
    ROS_INFO("\033[1;32m --> Generate trajectory by closed form solution use time: %f (ms)\033[0m",
             use_time.toSec() * 1000);

    visWayPointPath(path);

    visWayPointTraj(poly_coeff, segment_traj_time);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/vel", Vel, 1.0);//当前机器人能运行的最大速度
    nh.param("planning/acc", Acc, 1.0);//当前机器人能运行的最大加速度
    nh.param("planning/dev_order", dev_order, 3);
    nh.param("planning/min_order", min_order, 3);
    nh.param("vis/vis_traj_width", visualization_traj_width, 0.15);

    //_poly_numID is the maximum order of polynomial
    poly_coeff_num = 2 * dev_order;

    //state of start point
    start_position(0) = 0;
    start_position(1) = 0;
    start_position(2) = 0;

    start_velocity(0) = 0;
    start_velocity(1) = 0;
    start_velocity(2) = 0;

    way_pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallBack);

    waypoint_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    waypoint_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}

void visWayPointTraj(MatrixXd polyCoeff, VectorXd time) {
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp = ros::Time::now();
    _traj_vis.header.frame_id = "/map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = visualization_traj_width;
    _traj_vis.scale.y = visualization_traj_width;
    _traj_vis.scale.z = visualization_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;


    for (int i = 0; i < time.size(); i++) {
        for (double t = 0.0; t < time(i); t += 0.01, count += 1) {
            pos = getPosPoly(polyCoeff, i, t);
            cur(0) = pt.x = pos(0);
            cur(1) = pt.y = pos(1);
            cur(2) = pt.z = pos(2);
            _traj_vis.points.push_back(pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }

    waypoint_traj_vis_pub.publish(_traj_vis);
}

void visWayPointPath(MatrixXd path) {
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id = line_list.header.frame_id = "/map";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_list.ns = "wp_path";
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id = id;
    line_list.id = id;

    points.type = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;


    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;

    line_list.points.clear();

    for (int i = 0; i < path.rows(); i++) {
        geometry_msgs::Point p;
        p.x = path(i, 0);
        p.y = path(i, 1);
        p.z = path(i, 2);

        points.points.push_back(p);

        if (i < (path.rows() - 1)) {
            geometry_msgs::Point p_line;
            p_line = p;
            line_list.points.push_back(p_line);
            p_line.x = path(i + 1, 0);
            p_line.y = path(i + 1, 1);
            p_line.z = path(i + 1, 2);
            line_list.points.push_back(p_line);
        }
    }

    waypoint_path_vis_pub.publish(points);
    waypoint_path_vis_pub.publish(line_list);
}

/*!
 * 求解第k个轨迹段t时刻对应的位置
 * @param polyCoeff 多项式系数矩阵
 * @param k 轨迹段序号
 * @param t 时刻
 * @return [x,y,z]^T
 */
Vector3d getPosPoly(MatrixXd polyCoeff, int k, double t) {
    Vector3d ret;

    for (int dim = 0; dim < 3; dim++) {
        VectorXd coeff = (polyCoeff.row(k)).segment(dim * poly_coeff_num, poly_coeff_num);
        VectorXd time = VectorXd::Zero(poly_coeff_num);

        for (int j = 0; j < poly_coeff_num; j++)
            if (j == 0)
                time(j) = 1.0;
            else
                time(j) = pow(t, j);

        double temp_pose = 0.0;
        for (int i = 0; i < time.rows(); ++i) {
            temp_pose = temp_pose + coeff(i) * time(time.rows() - i - 1);
        }
        ret(dim) = temp_pose;
    }

    return ret;
}

/*!
 * 用于轨迹生成过程中，每段轨迹的分配时间的计算
 * @param Path 轨迹的航点
 * @return 每段轨迹应该对应的时间
 */
VectorXd timeAllocation(MatrixXd Path) {
    VectorXd times(Path.rows() - 1);

//#define USE_FIXED_TIME
#ifdef USE_FIXED_TIME
    times.setOnes();
#else
    const double MAX_VEL = 1.0;
    const double MAX_ACCEL = 1.0;
    const double t = MAX_VEL / MAX_ACCEL;
    const double dist_threshold_1 = MAX_ACCEL * t * t;

    double segment_t;
    for (unsigned int i = 1; i < Path.rows(); ++i) {
        double delta_dist = (Path.row(i) - Path.row(i - 1)).norm();
        if (delta_dist > dist_threshold_1) {
            segment_t = t * 2 + (delta_dist - dist_threshold_1) / MAX_VEL;
        } else {
            segment_t = std::sqrt(delta_dist / MAX_ACCEL);
        }

        times[i - 1] = segment_t;
    }
#endif

    return times;
}