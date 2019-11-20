#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/String.h>

#define ASTRA_FPS 30
#define MAX_QUEUE_SIZE 1

using namespace ros;
using namespace message_filters;
using namespace sensor_msgs;
using namespace darknet_ros_msgs;
using namespace std;
using namespace ros;
using namespace pcl;
using namespace std_msgs;

/**
 * Class which acts as a middleware of this project. It takes objects used for detections, and publishes detections
 * when anything is found.
 */
class VisionMiddleware {
private:
    string detectedName = "";
    typedef sync_policies::ApproximateTime<PointCloud<PointXYZ>, BoundingBoxes> syncPolicy;
    message_filters::Subscriber<PointCloud<PointXYZ>> pclSub;
    message_filters::Subscriber<String> detectionObjectSub;
    message_filters::Subscriber<BoundingBoxes> yoloSub;
    Synchronizer<syncPolicy> synchronizer;
    Publisher detectionPub;

    /**
     * Checks if a given point is NaN.
     * @param p = PointXYZ from the PointCloud.
     * @return = boolean if any axis is NaN.
     */
    static bool pointIsNan(const PointXYZ p) {
        return isnan(p.x) || isnan(p.y) || isnan(p.z);
    }

    /**
     * Function which gets a PointCloud<PointXYZ> and a BoundingBoxes (Yolo detection) object, when there is a detection
     * and a PointCloud available. The synchronizer will manage synchronization, so the PointCloud matches the detection frame.
     *
     * @param pcl = the PointCloud<PointXYZ> from our own conversion.
     * @param bb = the BoundingBoxes object.
     */
    void recognitionCallback(const PointCloud<PointXYZ>::ConstPtr &pcl, const BoundingBoxesConstPtr &bb) {
        vector<BoundingBox> boxes = bb->bounding_boxes;
        for (const auto &box : boxes) {
            string foundName = box.Class;

            //check if what we found is what we need to detect
            if (foundName == detectedName) {
                long middleX = (box.xmax + box.xmin) / 2;
                long middleY = (box.ymax + box.ymin) / 2;
                PointXYZ pxyz = pcl->at((int) middleX, (int) middleY);

                //A detection with no valid PointXYZ is useless..
                if (!pointIsNan(pxyz)) {
                    stringstream ss;

                    //todo 20/11/2019, this should become a ros msg with given properties.
                    ss << "type: " << foundName << ", distance: 0, x: " << pxyz.x << ", y: " << pxyz.y << ", z: "
                       << pxyz.z;
                    String s;
                    s.data = ss.str();
                    detectionPub.publish(s);
                }
            }
        }
    }

    /**
     * Listen to the topic which makes it able to detect a specific object.
     * @param toDetect = String representation of detection object.
     */
    void detectionObjectCallback(const StringConstPtr &toDetect) {
        string d = toDetect.get()->data;
        ROS_INFO_STREAM("Speech team requested to detect a(n) " << d);
        this->detectedName = d;
    }


public:
    explicit VisionMiddleware(NodeHandle &n) : pclSub(n, "/vision/point_cloud", MAX_QUEUE_SIZE),
                                               yoloSub(n, "/darknet_ros/bounding_boxes", MAX_QUEUE_SIZE),
                                               detectionObjectSub(n, "/speech/detect", MAX_QUEUE_SIZE),
                                               synchronizer(syncPolicy(MAX_QUEUE_SIZE), pclSub, yoloSub) {
        synchronizer.registerCallback(boost::bind(&VisionMiddleware::recognitionCallback, this, _1, _2));
        detectionObjectSub.registerCallback(&VisionMiddleware::detectionObjectCallback, this);

        //todo 18/11/2019, this should become a publisher with a custom position message
        detectionPub = n.advertise<String>("/vision/object_position", MAX_QUEUE_SIZE);
    }

};

int main(int argc, char **argv) {
    init(argc, argv, "middleware_node");
    NodeHandle n;
    Rate loopRate(ASTRA_FPS);
    VisionMiddleware visionMiddleware(n);

    ROS_INFO("Middleware node started");
    while (ok()) {
        spinOnce();
        loopRate.sleep();
    }

    return 0;
}