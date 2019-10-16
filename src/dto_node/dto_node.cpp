//
// Created by bo on 10/16/19.
//


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include "sensor_msgs/PointCloud2.h"


using namespace std;
using namespace pcl;
using namespace ros;
using namespace sensor_msgs;

void depthPointsCallback(const PointCloud2 &msg) {
    PointCloud<PointXYZ> pc;
    fromROSMsg(msg, pc);
    PointXYZ p = pc.at(msg.width / 2, msg.height / 2);
    cout << "X: " << p.x << ", Y: " << p.y << ", Z: " << p.z << endl;
}

int main(int argc, char **argv){
    init(argc, argv, "dto_node");
    NodeHandle n("~");
    Rate loop_rate(1);
    Subscriber depthPointsSub = n.subscribe("/camera/depth/points", 10, depthPointsCallback);

    while (ok()) {
        spinOnce();
        fflush(stdout);
        loop_rate.sleep();
    }

    return 0;
}

