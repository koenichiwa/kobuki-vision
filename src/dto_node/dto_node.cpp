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

class DistanceToObject{

private:
    int Xpos = 0;
    int Ypos = 0;

public:
    void depthPointsCallback(const PointCloud2 &msg) {
        PointCloud<PointXYZ> pc;fromROSMsg(msg, pc);
        PointXYZ p = pc.at(this->Xpos , this->Ypos);
        cout << "X: " << p.x << ", Y: " << p.y << ", Z: " << p.z << endl;
    }

    void XYcallback(int i){

    }

};

int main(int argc, char **argv){
    init(argc, argv, "dto_node");
    NodeHandle n("~");
    Rate loop_rate(1);

    DistanceToObject DTO;
    Subscriber depthPointsSub = n.subscribe("/camera/depth/points", 10, &DistanceToObject::depthPointsCallback, &DTO);
    Subscriber XYsub = n.subscribe("/get/x/y", 10, &DistanceToObject::depthPointsCallback, &DTO);

    while (ok()) {
        spinOnce();
        fflush(stdout);
        loop_rate.sleep();
    }

    return 0;
}

