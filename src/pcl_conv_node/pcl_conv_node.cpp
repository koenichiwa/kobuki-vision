//
// Created by bo on 11/17/19.
//
#include <message_filters/subscriber.h>
#include <pcl_ros/point_cloud.h>


#define ASTRA_FPS 30
#define MAX_QUEUE_SIZE 1

using namespace ros;
using namespace message_filters;
using namespace sensor_msgs;
using namespace pcl;

/**
 * @author Bo Sterenborg
 * This class converts the default ros PointCloud to the Pcl PointCloud and publishes the result.
 */
class PclConverter {
private:
    message_filters::Subscriber<PointCloud2> rosPcSub;
    Publisher pclPub;

    /**
     * Method which takes a ros PointCloud and publishes a PCL PointCloud<PointXYZ>
     * @param pcl = ros' PointCloud.
     */
    void pclCallback(const PointCloud2ConstPtr &pcl) {
        pcl::PointCloud<PointXYZ> pc;
        fromROSMsg(*pcl, pc);
        pclPub.publish(pc.makeShared());
    }

public:
    explicit PclConverter(NodeHandle &n) : rosPcSub(n, "/camera/depth_registered/points", MAX_QUEUE_SIZE) {
        typedef PointCloud<PointXYZ> PCLCloud;
        this->pclPub = n.advertise<PCLCloud>("/vision/point_cloud", MAX_QUEUE_SIZE);
    }

    void startConverting() {
        rosPcSub.registerCallback(&PclConverter::pclCallback, this);
    }
};

int main(int argc, char **argv) {
    init(argc, argv, "pcl_conv_node");
    NodeHandle n;
    Rate loopRate(ASTRA_FPS);

    PclConverter conv(n);
    conv.startConverting();
    ROS_INFO("PCL Conversion node started...");

    while (ok()) {
        spinOnce();
        loopRate.sleep();
    }
}