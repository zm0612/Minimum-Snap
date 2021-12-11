//
// Created by Zhang Zhimeng on 2021/11/26.
//

#ifndef PATH_SEARCHER_A_STAR_2D_FLOW_H
#define PATH_SEARCHER_A_STAR_2D_FLOW_H

#include "path_searcher/a_star/a_star_2d.h"
#include "path_searcher/utility/type.h"
#include "path_searcher/utility/map_tool.h"
#include "path_searcher/subscriber/pointcloud_subscriber.h"
#include "path_searcher/subscriber/init_pose_subscriber.h"


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl_conversions/pcl_conversions.h>

#include <mutex>

class AStar2DFlow {
public:
    AStar2DFlow() = delete;

    explicit AStar2DFlow(ros::NodeHandle &nh);

    void Run();

private:
    bool InitPoseData();

    void ReadData();

    bool HasInitPoseData();

    void PublishPath(const TypeVectorVecd<2> &path);

    void PublishVisitedNodes(const TypeVectorVecd<2> &visited_nodes_coord);

private:
    std::shared_ptr<AStar2D> astar_searcher_ptr_;
    std::shared_ptr<PointcloudSubscriber> pointcloud_sub_;
    std::shared_ptr<InitPoseSubscriber2D> init_pose_sub_;

    ros::Publisher grid_map_pub_;
    ros::Publisher path_pub_;
    ros::Publisher visited_nodes_pub_;
    std::deque<sensor_msgs::PointCloud2Ptr> deque_map_;
    std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> deque_init_pose_;

    geometry_msgs::PoseWithCovarianceStampedPtr current_init_pose_ptr_;
    sensor_msgs::PointCloud2Ptr current_map_ptr_;

    ros::Time timestamp_;
    double start_x_, start_y_;
    bool has_map_;
};

#endif //PATH_SEARCHER_A_STAR_2D_FLOW_H