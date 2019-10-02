//
// Created by bo on 9/25/19.
//

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"


using namespace std;
using namespace ros;
using namespace cv;

void depthPointsCallback(const sensor_msgs::PointCloud2 &msg) {
    cout << msg << endl;
}

void depthRegisteredPointsCallback(const sensor_msgs::PointCloud2 &msg) {
    cout << msg << endl;
}

void rawImageCallback(const sensor_msgs::Image &image) {
    cout << image << endl;
}

void rawRectImageCallback(const sensor_msgs::Image &image) {
    cout << image << endl;
}

void cameraInfoCallback(const sensor_msgs::CameraInfo &cameraInfo) {
    cout << cameraInfo << endl;
}


int main(int argc, char **argv) {
    init(argc, argv, "camera_node");
    NodeHandle n("~");
    Rate loop_rate(1);

    Subscriber depthPointsSub = n.subscribe("/camera/depth/points", 10, depthPointsCallback);
    Subscriber depthPointsRegisteredSub = n.subscribe("/camera/depth_registered/points", 10, depthRegisteredPointsCallback);
    Subscriber imageSub = n.subscribe("/camera/depth/image_raw", 10, rawImageCallback);
    Subscriber imageRecSub = n.subscribe("/camera/depth/image_rect_raw", 10, rawRectImageCallback);
    Subscriber camInfoSub = n.subscribe("/camera/depth/camera_info", 10, cameraInfoCallback);

    while (ok()) {
        spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

