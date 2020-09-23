#include "roomba/ar_detector.h"

#include <ros/ros.h>

ARDetector::ARDetector() {

};

void ARDetector::startup() {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    cameraSub = it.subscribe("camera_image", 1, &ARDetector::imageCallback, this);
    //markerPub = //example is nh.advertise<geometry_msgs::Twist >("cmd_vel", 1, false);

}

void ARDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

}