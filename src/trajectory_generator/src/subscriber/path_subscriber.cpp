//
// Created by Zhang Zhimeng on 2021/11/30.
//

#include "trajectory_generator/subscriber/path_subscriber.h"

PathSubscriber::PathSubscriber(ros::NodeHandle &nh, const std::string &topic, const int buff_size) {
    subscriber_ = nh.subscribe(topic, buff_size, &PathSubscriber::MessageCallBack, this);
}

void PathSubscriber::MessageCallBack(const nav_msgs::PathPtr &path_msg_ptr) {
    buff_mutex_.lock();
    deque_path_.emplace_back(path_msg_ptr);
    buff_mutex_.unlock();
}

void PathSubscriber::ParseData(std::deque<nav_msgs::PathPtr> &deque_path_msg_ptr) {
    buff_mutex_.lock();
    if (!deque_path_.empty()) {
        deque_path_msg_ptr.insert(deque_path_msg_ptr.end(),
                                  deque_path_.begin(),
                                  deque_path_.end());
        deque_path_.clear();
    }
    buff_mutex_.unlock();
}