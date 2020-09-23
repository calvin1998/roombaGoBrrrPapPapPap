#include "roomba/ar_detector.h"

#include <ros/ros.h>

ARDetector::ARDetector() {

};

void ARDetector::startup() {
    ros::NodeHandle nh;
    //scanSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &WallFollower::callbackScan, this);
    //commandSub = nh.subscribe<std_msgs::String>("cmd", 1, &WallFollower::callbackControl, this);
    //twistPub = nh.advertise< geometry_msgs::Twist >("cmd_vel", 1, false);

}