//
// Created by bo on 10/16/19.
//


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include "opencv2/opencv.hpp"
#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/objdetect.hpp"
#include <message_filters/sync_policies/approximate_time.h>

#define MAX_QUEUE_SIZE 10

using namespace std;
using namespace pcl;
using namespace ros;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv_bridge;
using namespace cv;

void callBack(const PointCloud2ConstPtr &pointCloud, const ImageConstPtr &im) {
    PointCloud<PointXYZ> pc;
    fromROSMsg(*pointCloud, pc);

    //todo 29/10/2019 this is duplicate and needs to be handled by img_conv_node
    CvImagePtr cv_ptr;
    try {
        cv_ptr = toCvCopy(*im, image_encodings::BGR8);
        HOGDescriptor hogDet;
        hogDet.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        vector<Rect> found;
        vector<double> weights;

        hogDet.detectMultiScale(cv_ptr->image, found, weights);
        if (!found.empty()) {
            for (size_t i = 0; i < found.size(); i++) {
                Rect r = found.at(i);
                PointXYZ p = pc.at(r.x, r.y);
                cout << "Persoon " << i+1 << "; X: " << p.x << ", Y: " << p.y << ", Z: " << p.z << endl;
            }
        } else {
            cout << "Er zijn geen personen gevonden..." << endl;
        }
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    init(argc, argv, "dto_node");
    NodeHandle n;
    Rate loop_rate(1);

    message_filters::Subscriber<PointCloud2> depthPointsSub(n, "/camera/depth/points", MAX_QUEUE_SIZE);
    message_filters::Subscriber<Image> imageSub(n, "/camera/rgb/image_raw", MAX_QUEUE_SIZE);

    typedef sync_policies::ApproximateTime<PointCloud2, Image> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), depthPointsSub, imageSub);
    sync.registerCallback(boost::bind(&callBack, _1, _2));
//
    while (ok()) {
        spinOnce();
        fflush(stdout);
        loop_rate.sleep();
    }

    return 0;
}

