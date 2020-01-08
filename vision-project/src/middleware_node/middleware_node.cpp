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
#define THRESHOLD 0.7

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
    class ObjectPositionEntry {
    public:
        float x, y, z, distance, height, width;
        Time time;
        string type;
    };

    typedef sync_policies::ApproximateTime<PointCloud<PointXYZ>, BoundingBoxes> syncPolicy;
    message_filters::Subscriber<PointCloud<PointXYZ>> pclSub;
    message_filters::Subscriber<BoundingBoxes> yoloSub;
    Synchronizer<syncPolicy> synchronizer;
    ServiceClient pclDistClient;
    ServiceClient pclNaNClient;
    map<string, ObjectPositionEntry> objectMap;
    ServiceServer objectPositionService;


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
            if (box.probability >= THRESHOLD) {
                float DeltaX = box.xmax - box.xmin;
                float DeltaY = box.ymax - box.ymin;
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
                        PclDistance pclDistance;
                        pclDistance.request.x = pxyz.x;
                        pclDistance.request.z = pxyz.z;

                        if (pclDistClient.call(pclDistance)) {
                            ObjectPositionEntry entry;
                            entry.x = pxyz.x;
                            entry.z = pxyz.z;
                            entry.y = pxyz.y;
                            entry.distance = pclDistance.response.distance;
                            entry.type = foundName;
                            entry.time = Time::now();
                            entry.width = DeltaX;
                            entry.height = DeltaY;
                            pair<string, ObjectPositionEntry> pair(foundName, entry);
                            objectMap.insert(pair);
                        } else {
                            ROS_ERROR("Could not reach pclDistance service client.");
                        }

                    }
                }else {
                    ROS_ERROR("Could not reach pclNan service client.");
                }
            }
        }
    }

    bool getObjectPosition(ObjectPosition::Request &req, ObjectPosition::Response &res) {
        if (objectMap.count(req.object)) {
            ObjectPositionEntry entry = objectMap.at(req.object);
            res.width = entry.width;
            res.time = entry.time;
            res.height = entry.height;
            res.distance = entry.distance;
            res.x = entry.x;
            res.y = entry.y;
            res.type = entry.type;
            res.z = entry.z;
            return true;
        }
        return false;
    }


public:
    explicit VisionMiddleware(NodeHandle
                              &n) : pclSub(n, "/vision/point_cloud", MAX_QUEUE_SIZE),
                                    yoloSub(n, "/darknet_ros/bounding_boxes", MAX_QUEUE_SIZE),
                                    synchronizer(syncPolicy(MAX_QUEUE_SIZE), pclSub, yoloSub) {

        synchronizer.registerCallback(boost::bind(&VisionMiddleware::recognitionCallback, this, _1, _2));
        objectPositionService = n.advertiseService("visionObjectPosition", &VisionMiddleware::getObjectPosition, this);
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