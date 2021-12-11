//
// Created by Zhang Zhimeng on 2021/11/30.
//

#ifndef TRAJECTORY_GENERATOR_POINTCLOUD_SUBSCRIBER_H
#define TRAJECTORY_GENERATOR_POINTCLOUD_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <deque>
#include <mutex>
#include <thread>
#include <string>

class PointcloudSubscriber {
public:
    PointcloudSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<sensor_msgs::PointCloud2Ptr> &deque_pointcloud_msg_ptr);

private:
    void MessageCallBack(const sensor_msgs::PointCloud2Ptr &pointcloud_msg_ptr);


private:
    ros::Subscriber subscriber_;
    std::deque<sensor_msgs::PointCloud2Ptr> deque_pointcloud_;

    std::mutex buff_mutex_;
};

#endif //TRAJECTORY_GENERATOR_POINTCLOUD_SUBSCRIBER_H
