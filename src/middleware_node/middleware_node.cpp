#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/String.h>

#define ASTRA_FPS 30
#define MAX_QUEUE_SIZE 1
#define DEBUG true

using namespace ros;
using namespace message_filters;
using namespace sensor_msgs;
using namespace darknet_ros_msgs;
using namespace std;
using namespace ros;
using namespace pcl;
using namespace std_msgs;

class VisionMiddleware {
private:
    string detectable = "";
    typedef sync_policies::ApproximateTime<PointCloud<PointXYZ>, BoundingBoxes> syncPolicy;
    message_filters::Subscriber<PointCloud<PointXYZ>> pclSub;
    message_filters::Subscriber<String> speechSub;
    message_filters::Subscriber<BoundingBoxes> yoloSub;
    Synchronizer<syncPolicy> synchronizer;
    Publisher armPub;

    void recognitionCallback(const PointCloud<PointXYZ>::ConstPtr &pcl, const BoundingBoxesConstPtr &bb) {
        vector<BoundingBox> boxes = bb->bounding_boxes;
        for (unsigned long i = 0; i < boxes.size(); i++) {
            BoundingBox box = boxes[i];
            string found = box.Class;
            if (DEBUG) {
                long middleX = (box.xmax + box.xmin) / 2;
                long middleY = (box.ymax + box.ymin) / 2;
                ROS_INFO_STREAM("Object #" << i + 1 << ": " << box.Class << ", " << (box.probability * 100) << "%");
                PointXYZ pxyz = pcl->at(middleX, middleY);
                ROS_INFO_STREAM("Object #" << i + 1 << " PCL: X: " << pxyz.x << ", Y: " << pxyz.y << ", Z: " << pxyz.z);
            }
            if (found == detectable) {
                long middleX = (box.xmax + box.xmin) / 2;
                long middleY = (box.ymax + box.ymin) / 2;
                PointXYZ pxyz = pcl->at(middleX, middleY);

                stringstream ss;
                ss << "type: " << found << ", distance: not impl yet, x: " << pxyz.x << ", y: " << pxyz.y << ", z: "
                   << pxyz.z;
                String s;
                s.data = ss.str();
                armPub.publish(s);
            }
        }
    }

    void speechCallback(const StringConstPtr &toDetect) {
        string d = toDetect.get()->data;
        ROS_INFO_STREAM("Speech team requested to detect " << d);
        this->detectable = d;
    }


public:
    explicit VisionMiddleware(NodeHandle &n) : pclSub(n, "/vision/point_cloud", MAX_QUEUE_SIZE),
                                               yoloSub(n, "/darknet_ros/bounding_boxes", MAX_QUEUE_SIZE),
                                               speechSub(n, "/speech/detect", MAX_QUEUE_SIZE),
                                               synchronizer(syncPolicy(MAX_QUEUE_SIZE), pclSub, yoloSub) {
        synchronizer.registerCallback(boost::bind(&VisionMiddleware::recognitionCallback, this, _1, _2));
        speechSub.registerCallback(&VisionMiddleware::speechCallback, this);

        //todo 18/11/2019, this should become a publisher with a custom position message
        armPub = n.advertise<String>("/vision/object_position", MAX_QUEUE_SIZE);
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