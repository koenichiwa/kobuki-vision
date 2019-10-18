//
// Created by bo on 10/16/19.
//


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"

#define MAX_QUEUE_SIZE 10

using namespace std;
using namespace pcl;
using namespace ros;
using namespace sensor_msgs;
using namespace message_filters;


void callBack(const PointCloud2 &pointCloud, const Image &image) {
    PointCloud<PointXYZ> pc;
    fromROSMsg(pointCloud, pc);
    PointXYZ p = pc.at(image.width / 2, image.height / 2);
    cout << "X: " << p.x << ", Y: " << p.y << ", Z: " << p.z << endl;
}

int main(int argc, char **argv) {
    init(argc, argv, "dto_node");
    NodeHandle n("~");
    Rate loop_rate(1);

    message_filters::Subscriber<PointCloud2> depthPointsSub(n, "/camera/depth/points", MAX_QUEUE_SIZE);
    message_filters::Subscriber<Image> imageSub(n, "/camera/rgb/image_raw", MAX_QUEUE_SIZE);

    TimeSynchronizer<PointCloud2, Image> sync(depthPointsSub, imageSub, MAX_QUEUE_SIZE);
    sync.registerCallback(boost::bind(&callBack, _1, _2));

    while (ok()) {
        spinOnce();
        fflush(stdout);
        loop_rate.sleep();
    }

    return 0;
}

