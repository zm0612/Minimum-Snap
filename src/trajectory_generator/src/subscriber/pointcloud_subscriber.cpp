//
// Created by Zhang Zhimeng on 2021/11/30.
//
#include "trajectory_generator/subscriber/pointcloud_subscriber.h"

PointcloudSubscriber::PointcloudSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(topic_name, buff_size, &PointcloudSubscriber::MessageCallBack, this);
}

void PointcloudSubscriber::MessageCallBack(const sensor_msgs::PointCloud2Ptr &pointcloud_msg_ptr) {
    buff_mutex_.lock();
    deque_pointcloud_.emplace_back(pointcloud_msg_ptr);
    buff_mutex_.unlock();
}

void PointcloudSubscriber::ParseData(std::deque<sensor_msgs::PointCloud2Ptr> &deque_pointcloud_msg_ptr) {
    buff_mutex_.lock();
    if (!deque_pointcloud_.empty()) {
        deque_pointcloud_msg_ptr.insert(
                deque_pointcloud_msg_ptr.end(),
                deque_pointcloud_.begin(),
                deque_pointcloud_.end()
        );

        deque_pointcloud_.clear();
    }
    buff_mutex_.unlock();
}