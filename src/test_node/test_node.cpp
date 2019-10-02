//
// Created by bo on 9/25/19.
//

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        ros::spinOnce();
        std::cout << "TEST" << std::endl;
        loop_rate.sleep();
    }

    return 0;
}