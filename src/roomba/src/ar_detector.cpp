#include "roomba/ar_detector.h"
#include "roomba/pub_sub_topics.h"
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>


ARDetector::ARDetector() {

};

void ARDetector::startup() {
    //initialise publishers and subscribers
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    cameraSub = it.subscribe(topics::cameraImage, 1, &ARDetector::imageCallback, this);
    //markerPub = //example is nh.advertise<geometry_msgs::Twist >("cmd_vel", 1, false);
    //TODO: which opencv params to use?
    detectorParams = cv::aruco::DetectorParameters::create();
    //TODO: which dictionary do the markers belong to?
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
}

void ARDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    //convert ros image to cv image
    cv_bridge::CvImagePtr cvImagePtr = rosToCvImage(msg);//this pointer is self managed
    //if image conversion failed
    if (cvImagePtr == NULL) return;//TODO: Any better way to handle failure?
    //else detect markers
    std::vector<int> markerIds;//index of markers in dictionary
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;//location of corners in (r,c) or (y,x)
    cv::aruco::detectMarkers(cvImagePtr->image, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
    //TODO: Publish markers from getARMarkerCoords
}

cv_bridge::CvImagePtr ARDetector::rosToCvImage(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr newCvImagePtr;
    try {
        //TODO: ensure image encoding is correct
        newCvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//if ros image not modified, toCvShare may be used
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return NULL;
    }
    return newCvImagePtr;
}

mapCoords ARDetector::getARMarkerCoords(markerCoords markerCorners) {

}