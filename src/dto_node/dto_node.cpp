//
// Created by bo on 10/16/19.
//


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "message_filters/subscriber.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/objdetect.hpp"
#include <message_filters/sync_policies/approximate_time.h>
#include <sstream>
#include <string>

#define MAX_QUEUE_SIZE 10
#define ASTRA_FPS 30

using namespace std;
using namespace pcl;
using namespace ros;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv_bridge;
using namespace cv;


/**
 * @author Bo Sterenborg
 *
 * Class which detects objects when subscribing on the Orbbec Astra Camera.
 */
class ObjectDetector {

public:
    enum Detectable {
        RED_BALL, PEOPLE, YOLO_V3
    };

    ObjectDetector(NodeHandle &n, Detectable dtc) : pclSub(nh, "/camera/depth_registered/points", MAX_QUEUE_SIZE),
                                                    rgbImageSub(nh, "/camera/rgb/image_rect_color",
                                                                MAX_QUEUE_SIZE),
                                                    synchronizer(syncPolicy(MAX_QUEUE_SIZE), pclSub,
                                                                 rgbImageSub) {
        detectable = dtc;
        publisher = n.advertise<Image>("/camera_reading", MAX_QUEUE_SIZE);
    }

    /**
     * Starts the detecting (using given detectable).
     */
    void startDetecting() {
        synchronizer.registerCallback(boost::bind(&ObjectDetector::imgAndPCCallback, this, _1, _2));
    }

    /**
     * Changes the detectable at runtime.
     * @param dtc
     */
    void setNewDetectable(Detectable dtc) {
        detectable = dtc;
    }


private:
    const Scalar DETECTION_SCALAR = Scalar(255, 0, 255);
    const Size DEFAULT_ASTRA_IMAGE_SIZE = Size(640, 480);
    const int DETECTION_THICKNESS = 3;
    typedef sync_policies::ApproximateTime<PointCloud2, Image> syncPolicy;
    NodeHandle nh;
    message_filters::Subscriber<PointCloud2> pclSub;
    message_filters::Subscriber<Image> rgbImageSub;
    Synchronizer<syncPolicy> synchronizer;
    Publisher publisher;
    Detectable detectable;

    /**
     * Checks if a given PointXYZ is NaN. Is when any of the coordinates is NaN.
     * @param pxyz = PointXYZ (pcl PointCloud point).
     * @return  = boolean if the given point is NaN.
     */
    static bool pointIsNan(PointXYZ pxyz) {
        return isnan(pxyz.x) || isnan(pxyz.y) || isnan(pxyz.z);
    }

    /**
     * Converts a ros PointCloud to pcl PointCloud.
     * @param pointCloud = ros PointCloud.
     * @return = pcl PointCloud.
     */
    static PointCloud<PointXYZ> pcl2ToPCLPc(const PointCloud2ConstPtr &pointCloud) {
        PointCloud<PointXYZ> pc;
        fromROSMsg(*pointCloud, pc);
        return pc;
    }

    /**
     * Converts a ros image to a cv image.
     * @param im = ros image ptr.
     * @return = cv image.
     */
    static CvImagePtr rosImgToCVImg(const ImageConstPtr &im) {
        CvImagePtr cv_ptr;
        try {
            return toCvCopy(*im, image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    /**
     * Shows the given image in a window.
     * @param im = (cv) image.
     */
    static void showDetection(const Mat &im) {
        imshow("Detections", im);
        waitKey(1);
    }

    /**
     * Marks a found point with a circle.
     * @param im  = image.
     * @param center = center point.
     * @param radius  = radius of circle.
     */
    void markDetectionAsCircle(const Mat &im, const Point &center, int radius) {
        circle(im, center, radius, DETECTION_SCALAR, DETECTION_THICKNESS, 8, 0);
    }

    /**
     * Marks a found point with a rectangle.
     * @param im = image.
     * @param r = given rectangle to mark with.
     */
    void markDetectionAsRectangle(Mat &im, const Rect &r) {
        rectangle(im, r, DETECTION_SCALAR, DETECTION_THICKNESS);
    }

    /**
     * Detects a red ball.
     * @param pointCloud2Ptr = PointCloud2 from callback.
     * @param imgPtr = Image from callback
     */
    void detectRedBall(const PointCloud2ConstPtr &pointCloud2Ptr, const ImageConstPtr &imgPtr) {
        //these are the hue scalars for red
        const Scalar redLowerHueLowerBounds = Scalar(0, 100, 100);
        const Scalar redLowerHueUpperBounds = Scalar(10, 255, 255);
        const Scalar redHigherHueLowerBounds = Scalar(160, 100, 100);
        const Scalar redHigherHueUpperBounds = Scalar(179, 255, 255);

        const CvImagePtr cvPtr = rosImgToCVImg(imgPtr);
        const PointCloud<PointXYZ> pointCloud = pcl2ToPCLPc(pointCloud2Ptr);

        Mat im = cvPtr->image;
        resize(im, im, DEFAULT_ASTRA_IMAGE_SIZE);//force default Orbbec's size
        cvtColor(im, im, COLOR_BGR2HSV);

        //detect only the red ball(s) using hue
        Mat lowerHue, higherHue, combinedHue;
        inRange(im, redLowerHueLowerBounds, redLowerHueUpperBounds, lowerHue);
        inRange(im, redHigherHueLowerBounds, redHigherHueUpperBounds, higherHue);
        addWeighted(lowerHue, 1.0, higherHue, 1.0, 0.0, combinedHue);

        //add some noise reduction
        GaussianBlur(combinedHue, combinedHue, Size(9, 9), 2, 2);
        medianBlur(combinedHue, combinedHue, 3);

        //detect circles within the combined hue
        vector<Vec3f> circles;
        HoughCircles(combinedHue, circles, HOUGH_GRADIENT, 1.0, (double) combinedHue.rows, 150.0, 25.0, 10, 100);

        showDetection(combinedHue);
        //convert image back to bgr (in case we'd like to see it)
        cvtColor(im, im, COLOR_HSV2BGR);

        if (!circles.empty()) {
            for (unsigned long i = 0; i < circles.size(); i++) {
                Vec3f vec = circles[i];
                Point center(vec[0], vec[1]);
                PointXYZ pxyz = pointCloud.at(center.x, center.y);

                if (!pointIsNan(pxyz)) {
                    // circle outline
                    int radius = vec[2];

                    markDetectionAsCircle(im, center, radius);
                    stringstream msg;
                    ROS_INFO_STREAM(
                            "Rode bal " << i + 1 << ": X = " << pxyz.x << ", Y = " << pxyz.y << ", Z = " << pxyz.z
                                        << endl);
                }
            }
        } else {
            ROS_INFO("Geen rode bal gevonden...");
        }
//        showDetection(im);
    }

    /**
     * Detects people and their x, y and z coordinates.
     * @param pointCloud2Ptr = PointCloud2 from callback.
     * @param imgPtr = Image from callback
     */
    void detectPeople(const PointCloud2ConstPtr &pointCloud2Ptr, const ImageConstPtr &imgPtr) {
        const CvImagePtr cvPtr = rosImgToCVImg(imgPtr);
        const PointCloud<PointXYZ> pointCloud = pcl2ToPCLPc(pointCloud2Ptr);

        //timely detect people, future should support more
        HOGDescriptor hogDet;
        hogDet.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        vector<Rect> peopleRectangles;
        vector<double> weights;

        //Save detected people into rectangles
        hogDet.detectMultiScale(cvPtr->image, peopleRectangles, weights);
        if (!peopleRectangles.empty()) {
            for (size_t i = 0; i < peopleRectangles.size(); i++) {
                Rect r = peopleRectangles.at(i);
                Point center = (r.br() + r.tl()) * 0.5;

                PointXYZ p = pointCloud.at(center.x, center.y);
                if (!pointIsNan(p)) {
                    markDetectionAsRectangle(cvPtr->image, peopleRectangles[i]);
                    ROS_INFO_STREAM("Persoon " << i + 1 << "; X: " << p.x << ", Y: " << p.y << ", Z: " << p.z << endl);
                }
            }
        } else {
            ROS_INFO("Er zijn geen personen gevonden...");
        }
        showDetection(cvPtr->image);
    }

    void yolov3(const PointCloud2ConstPtr &pointCloud2Ptr, const ImageConstPtr &imgPtr) {
        ROS_INFO("HIER");
        publisher.publish(imgPtr);
    }

    /**
     * Callback for the PointCloud and RGB image.
     * @param pointCloud = pointCloud from ros.
     * @param im = RGB image from ros.
     */
    void imgAndPCCallback(const PointCloud2ConstPtr &pointCloud, const ImageConstPtr &im) {
        //check what we need to detect
        switch (detectable) {
            case RED_BALL: {
                detectRedBall(pointCloud, im);
                break;
            }
            case PEOPLE: {
                detectPeople(pointCloud, im);
                break;
            }
            case YOLO_V3: {
                yolov3(pointCloud, im);
                break;
            }
        }
    }

};

/**
 * Converts a ros image to a cv image.
 * @param im = ros image ptr.
 * @return = cv image.
 */
static CvImagePtr rosImgToCVImg(const ImageConstPtr &im) {
    CvImagePtr cv_ptr;
    try {
        return toCvCopy(*im, image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

/**
 * Shows the given image in a window.
 * @param im = (cv) image.
 */
static void showDetection(const Mat &im) {
    imshow("Detections", im);
    waitKey(1);
}

void callback(const ImageConstPtr &im) {
    showDetection(rosImgToCVImg(im)->image);
}

//int main(int argc, char **argv) {
//    init(argc, argv, "dto_node");
//    NodeHandle n;
//    Rate loopRate(ASTRA_FPS);
//
//    //todo 09/11/2019, the detectable becomes dynamic in the future and will be set by the 'speech' team
//    ObjectDetector objectDetector(n, ObjectDetector::Detectable::YOLO_V3);
//    objectDetector.startDetecting();
//
//    ros::Subscriber sub =  n.subscribe("/detection_image", MAX_QUEUE_SIZE, &callback);
//
//    while (ok()) {
//        spinOnce();
//        loopRate.sleep();
//    }
//
//    return 0;
//}

