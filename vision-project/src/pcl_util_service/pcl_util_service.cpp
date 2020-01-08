#include <vision/PclDistance.h>
#include <vision/PclContainsNaN.h>
#include <ros/ros.h>
#include <cmath>

using namespace vision;
using namespace std;
using namespace ros;

/**
 * @author Bo Sterenborg
 *
 * Calculates the distance to a given object, by x, y and z axis.
 * @param req = request object.
 * @param res = response object.
 * @return
 */
bool calculatePCLDistance(PclDistance::Request &req, PclDistance::Response &res) {
    res.distance = sqrt((req.x * req.x) + (req.z * req.z)) * 100;
    return true;
}

/**
 * @author Bo Sterenborg
 *
 * Checks if any of given x, y and z axis is NaN.
 * @param req  = request object.
 * @param res = response object.
 * @return = boolean if succeeded.
 */
bool checkIfPCLContainsNaN(PclContainsNaN::Request &req, PclContainsNaN::Response &res) {
    bool containsNan = isnan(req.x) || isnan(req.y) || isnan(req.z);
    res.isNaN = containsNan;
    return true;
}

int main(int argc, char **argv) {
    init(argc, argv, "pcl_util_service");
    NodeHandle n;
    ServiceServer distService = n.advertiseService("calculatePCLDistance", calculatePCLDistance);
    ServiceServer nanService = n.advertiseService("checkIfPCLContainsNaN", checkIfPCLContainsNaN);
    ROS_INFO("Pcl utility service started.");
    spin();
}