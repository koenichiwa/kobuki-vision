//
// Created by bo on 10/16/19.
//


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include "opencv2/opencv.hpp"
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


void detectPeople(const CvImagePtr &cv_ptr, PointCloud<PointXYZ> pointCloud) {
    //timely detect people, future should support more
    HOGDescriptor hogDet;
    hogDet.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    vector<Rect> peopleRectangles;
    vector<double> weights;

    //Save detected people into rectangles
    hogDet.detectMultiScale(cv_ptr->image, peopleRectangles, weights);
    if (!peopleRectangles.empty()) {
        for (size_t i = 0; i < peopleRectangles.size(); i++) {
            Rect r = peopleRectangles.at(i);
            Point center = (r.br() + r.tl()) * 0.5;

            PointXYZ p = pointCloud.at(center.x, center.y);
            cout << "Persoon " << i + 1 << "; X: " << p.x << ", Y: " << p.y << ", Z: " << p.z << endl;
//                rectangle(cv_ptr->image, peopleRectangles[i], cv::Scalar(0,0,255), 3);
        }
    } else {
        cout << "Er zijn geen personen gevonden..." << endl;
    }
//        imshow("Personen", cv_ptr->image);
//        waitKey(0);
}


void detectRedBall(const CvImagePtr &cv_ptr, PointCloud<PointXYZ> pointCloud) {

    CascadeClassifier cascade;
    string cascade_name = "ball_cascade.xml";

    Mat frame = cv_ptr -> image;
    std::vector<Rect> balls;

    Mat frame_gray;
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    cascade.detectMultiScale(frame_gray,
            balls,
            1.1,
            5,
            8,
            Size(16,16));

    for (int i = 0; i < balls.size(); ++i) {
        Rect ball = balls.at(i);
        Point center = (ball.br() + ball.tl()) * 0.5;
        PointXYZ p = pointCloud.at(center.x, center.y);
        cout << "Ball " << i + 1 << "; X: " << p.x << ", Y: " << p.y << ", Z: " << p.z << endl;

    }



//    Mat srcGray;
//    cvtColor(cv_ptr->image, srcGray, CV_BGR2GRAY);
//
//    /// Reduce the noise so we avoid false circle detection
//    GaussianBlur(srcGray, srcGray, Size(9, 9), 2, 2);
//
//    vector<Vec3f> circles;
//
//    /// Apply the Hough Transform to find the circles
//    HoughCircles(srcGray, circles, CV_HOUGH_GRADIENT, 1, srcGray.rows / 8, 200, 100, 0, 0);
//
//    if (!circles.empty()) {
//        for (unsigned long i = 0; i < circles.size(); i++) {
//            int x = circles[i][0];
//            int y = circles[i][1];
//            PointXYZ p = pointCloud.at(x, y);
//            cout << "Bal " << i + 1 << "; X: " << p.x << ", Y: " << p.y << ", Z: " << p.z << endl;
//        }
//    } else {
//        cout << "Bal niet gevonden." << endl;
//    }
}

/**
 * Takes a PointCloud and image, then calculates the distance towards detected objects.
 * @param pointCloud = astra's PointCloud2
 * @param im = astra's image
 */
void callBack(const PointCloud2ConstPtr &pointCloud, const ImageConstPtr &im) {
    PointCloud<PointXYZ> pc;
    fromROSMsg(*pointCloud, pc);

    //todo 29/10/2019 this is duplicate and needs to be handled by img_conv_node
    CvImagePtr cv_ptr;
    try {
        cv_ptr = toCvCopy(*im, image_encodings::BGR8);

        detectPeople(cv_ptr, pc);
//        detectRedBall(cv_ptr, pc);

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

    //The 'approximate' syncpolicy is required to get an image and pointcloud
    //The timestamps are note fully synchronized...
    typedef sync_policies::ApproximateTime<PointCloud2, Image> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(MAX_QUEUE_SIZE), depthPointsSub, imageSub);
    sync.registerCallback(boost::bind(&callBack, _1, _2));

    while (ok()) {
        spinOnce();
        fflush(stdout);
        loop_rate.sleep();
    }

    return 0;
}

