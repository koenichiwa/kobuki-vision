//
// Created by bo on 11/13/19.
//


#include <pcl_conversions/pcl_conversions.h>
#include "message_filters/subscriber.h"
#include "cv_bridge/cv_bridge.h"
#include <message_filters/sync_policies/approximate_time.h>
#include "darknet_ros_msgs/BoundingBoxes.h"

#define ASTRA_FPS 30
#define MAX_QUEUE_SIZE 10

using namespace ros;
using namespace message_filters;
using namespace sensor_msgs;
using namespace cv_bridge;
using namespace cv;
using namespace darknet_ros_msgs;
using namespace std;


class YoloDetector {
private:
    message_filters::Subscriber<PointCloud2> pclSub;
    message_filters::Subscriber<Image> rgbImSub;
    message_filters::Subscriber<BoundingBoxes> yoloSub;
    Publisher rawImPub;
    NodeHandle n;

    void rgbImCallback(const ImageConstPtr &im) {
        rawImPub.publish(im);
    }

    void yoloCallback(const BoundingBoxesConstPtr &bb) {
        vector<BoundingBox> boxes = bb->bounding_boxes;
        for (unsigned long i = 0; i < boxes.size(); i++) {
            BoundingBox box = boxes[i];
            ROS_INFO_STREAM("Object " << i+1 << ": " << box.Class << ", %" << (box.probability*100));
        }
    }


public:
    YoloDetector(NodeHandle &n) : pclSub(n, "/camera/depth_registered/points", MAX_QUEUE_SIZE),
                                  rgbImSub(n, "/camera/rgb/image_rect_color", MAX_QUEUE_SIZE),
                                  yoloSub(n, "/darknet_ros/bounding_boxes", MAX_QUEUE_SIZE) {
        rgbImSub.registerCallback(&YoloDetector::rgbImCallback, this);
        yoloSub.registerCallback(&YoloDetector::yoloCallback, this);
        rawImPub = n.advertise<Image>("/camera_reading", MAX_QUEUE_SIZE);
        this->n = n;
    }

};

int main(int argc, char **argv) {
    init(argc, argv, "dto_node");
    NodeHandle n;
    Rate loopRate(ASTRA_FPS);
    YoloDetector yoloDetector(n);

    while (ok()) {
        spinOnce();
        loopRate.sleep();
    }

    return 0;
}
