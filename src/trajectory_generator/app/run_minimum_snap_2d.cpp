//
// Created by Zhang Zhimeng on 2021/11/29.
//

#include "trajectory_generator/minimum_snap/minimum_snap_2d_flow.h"
#include "3rd/backward.hpp"

#include <ros/ros.h>

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "run_minimum_snap_2d");
    ros::NodeHandle node_handle("~");

    MinimumSnap2DFlow minimum_snap_2d_flow(node_handle);

    ros::Rate rate(10);

    while (ros::ok()) {
        minimum_snap_2d_flow.Run();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};
