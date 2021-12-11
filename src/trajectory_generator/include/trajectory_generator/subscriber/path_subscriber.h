//
// Created by Zhang Zhimeng on 2021/11/30.
//

#ifndef TRAJECTORY_GENERATOR_PATH_SUBSCRIBER_H
#define TRAJECTORY_GENERATOR_PATH_SUBSCRIBER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <deque>
#include <mutex>

class PathSubscriber {
public:
    PathSubscriber(ros::NodeHandle &nh, const std::string &topic, int buff_size);

    void ParseData(std::deque<nav_msgs::PathPtr> &deque_path_msg_ptr);

private:
    void MessageCallBack(const nav_msgs::PathPtr &path_msg_ptr);

private:
    ros::Subscriber subscriber_;
    std::deque<nav_msgs::PathPtr> deque_path_;
    std::mutex buff_mutex_;
};

#endif //TRAJECTORY_GENERATOR_PATH_SUBSCRIBER_H
