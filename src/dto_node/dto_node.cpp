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
    const Scalar RED_LOWER_HUE_LOWER_BOUNDS = Scalar(0, 100, 100);
    const Scalar RED_LOWER_HUE_UPPER_BOUNDS = Scalar(10, 255, 255);
    const Scalar RED_HIGHER_HUE_LOWER_BOUNDS = Scalar(160, 100, 100);
    const Scalar RED_HIGHER_HUE_UPPER_BOUNDS = Scalar(179, 255, 255);
    
    Mat im = cv_ptr->image;
    resize(im, im, Size(640, 480));//force default Orbbec's size
    cvtColor(im, im, COLOR_BGR2HSV);

    //detect only the red ball(s) using hue
    Mat lowerHue, higherHue, combinedHue;
    inRange(im, RED_LOWER_HUE_LOWER_BOUNDS, RED_LOWER_HUE_UPPER_BOUNDS, lowerHue);
    inRange(im, RED_HIGHER_HUE_LOWER_BOUNDS, RED_HIGHER_HUE_UPPER_BOUNDS, higherHue);
    addWeighted(lowerHue, 1.0, higherHue, 1.0, 0.0, combinedHue);

    //add some noise reduction
    GaussianBlur(combinedHue, combinedHue, Size(9, 9), 2, 2);
    medianBlur(combinedHue, combinedHue, 3);

    //detect circles within the combined hue
    vector<Vec3f> circles;
    HoughCircles(combinedHue, circles, HOUGH_GRADIENT, 1.0, (double) combinedHue.rows / 16, 100.0, 30.0, 1, 125);

    //convert image back to bgr (in case we'd like to see it)
    cvtColor(im, im, COLOR_HSV2BGR);

    if (!circles.empty()) {
        for (unsigned long i = 0; i < circles.size(); i++) {
            Vec3f vec = circles[i];
            Point center(vec[0], vec[1]);
            // circle center
            circle(im, center, 1, Scalar(0, 100, 100), 3, 8, 0);
            // circle outline
            int radius = vec[2];
            circle(im, center, radius, Scalar(255, 0, 255), 3, 8, 0);

            PointXYZ pxyz = pointCloud.at(center.x, center.y);
            cout << "Rode bal " << i << ": X = " << pxyz.x << ", Y = " << pxyz.y << ", Z = " << pxyz.z << endl;
        }
    } else {
        cout << "Geen rode bal gevonden..." << endl;
    }

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

//        detectPeople(cv_ptr, pc);
        detectRedBall(cv_ptr, pc);

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

