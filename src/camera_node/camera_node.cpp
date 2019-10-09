//
// Created by bo on 9/25/19.
//

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv-3.3.1-dev/opencv/cv.hpp>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"


using namespace std;
using namespace ros;
using namespace cv;
using namespace cv_bridge;


void imageCb(const sensor_msgs::Image &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        imshow("BLA", cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void depthPointsCallback(const sensor_msgs::PointCloud2 &msg) {
    cout << msg << endl;
}

void depthRegisteredPointsCallback(const sensor_msgs::PointCloud2 &msg) {
    for (unsigned char i : msg.data) {
        cout << i;
    }
}

void rawImageCallback(const sensor_msgs::Image &image) {
//    boost::shared_ptr<CvImage> cv_ptr = toCvCopy(image, sensor_msgs::image_encodings::BGR8);
//    Mat im = cv_ptr->image;
    imageCb(image);
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

//    Subscriber depthPointsSub = n.subscribe("/camera/depth/points", 10, depthPointsCallback);
//    Subscriber depthPointsRegisteredSub = n.subscribe("/camera/depth_registered/points", 10,
//                                                      depthRegisteredPointsCallback);
    Subscriber imageSub = n.subscribe("/camera/depth/image_raw", 10, rawImageCallback);
//    Subscriber imageRecSub = n.subscribe("/camera/depth/image_rect_raw", 10, rawRectImageCallback);

    while (ok()) {
        spinOnce();
        fflush(stdout);
        loop_rate.sleep();
    }

    return 0;
}

