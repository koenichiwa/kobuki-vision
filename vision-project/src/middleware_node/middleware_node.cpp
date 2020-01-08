#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/String.h>
#include <vision/ObjectPosition.h>
#include <vision/PclDistance.h>
#include <vision/PclContainsNaN.h>

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
using namespace vision;

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
    ServiceClient pclDistClient;
    ServiceClient pclNaNClient;

    /**
     * Function which gets a PointCloud<PointXYZ> and a BoundingBoxes (Yolo detection) object, when there is a detection
     * and a PointCloud available. The synchronizer will manage synchronization, so the PointCloud matches the detection frame.
     *
     * @param pcl = the PointCloud<PointXYZ> from our own conversion.
     * @param bb = the BoundingBoxes object.
     */
    void recognitionCallback(const PointCloud<PointXYZ>::ConstPtr &pcl, const BoundingBoxesConstPtr &bb) {
        Time now = Time::now();
        vector<BoundingBox> boxes = bb->bounding_boxes;
        for (const auto &box : boxes) {
            string foundName = box.Class;

            //check if what we found is what we need to detect and the probability is high enough
            if (foundName == detectedName) {
                long DeltaX = box.xmax - box.xmin;
                long DeltaY = box.ymax - box.ymin;
                long middleX = (box.xmax + box.xmin) / 2;
                long middleY = (box.ymax + box.ymin) / 2;
                PointXYZ pxyz = pcl->at((int) middleX, (int) middleY);

                //A detection with no valid PointXYZ is useless..
                PclContainsNaN pclContainsNaN;
                pclContainsNaN.request.x = pxyz.x;
                pclContainsNaN.request.y = pxyz.y;
                pclContainsNaN.request.z = pxyz.z;
                if (pclNaNClient.call(pclContainsNaN)) {
                    if (!pclContainsNaN.response.isNaN) {
                        stringstream ss;

                        ObjectPosition pos;
                        pos.x = pxyz.x;
                        pos.y = pxyz.y;
                        pos.z = pxyz.z;
                        pos.time = now;
                        pos.type = foundName;
                        pos.height = (DeltaY * 0.0019) * 2.54;
                        pos.width = (DeltaX * 0.0019) * 2.54;

                        PclDistance pclDistance;
                        pclDistance.request.x = pxyz.x;
                        pclDistance.request.z = pxyz.z;

                        if (pclDistClient.call(pclDistance)) {
                            pos.distance = pclDistance.response.distance;
                            detectionPub.publish(pos);
                        } else {
                            ROS_ERROR("Could not reach pclDistance service client.");
                        }
                    }
                } else {
                    ROS_ERROR("Could not reach pclNan service client.");
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
    explicit VisionMiddleware(NodeHandle
                              &n) : pclSub(n, "/vision/point_cloud", MAX_QUEUE_SIZE),
                                    yoloSub(n, "/darknet_ros/bounding_boxes", MAX_QUEUE_SIZE),
                                    detectionObjectSub(n, "/speech/detect", MAX_QUEUE_SIZE),
                                    synchronizer(syncPolicy(MAX_QUEUE_SIZE), pclSub, yoloSub) {
        synchronizer.registerCallback(boost::bind(&VisionMiddleware::recognitionCallback, this, _1, _2));
        detectionObjectSub.registerCallback(&VisionMiddleware::detectionObjectCallback, this);
        detectionPub = n.advertise<ObjectPosition>("/vision/object_detection", MAX_QUEUE_SIZE);
        pclDistClient = n.serviceClient<PclDistance>("calculatePCLDistance");
        pclNaNClient = n.serviceClient<PclContainsNaN>("checkIfPCLContainsNaN");
        pclDistClient.waitForExistence();
        pclNaNClient.waitForExistence();
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