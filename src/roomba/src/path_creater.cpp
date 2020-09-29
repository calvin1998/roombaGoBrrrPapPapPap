#include "ros/ros.h"
//#include "nav_msgs/msg.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"


//global for now to make it easy
ros::Publisher path_pub;
nav_msgs::Path path;


void PointRecieved(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  ROS_INFO("#################### ODOM RECIEVED #############");
  path.header = msg->header;
  //pose = PoseStamped()
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  path.poses.push_back(pose);
  path_pub.publish(path);

}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "path_creater");

  ros::NodeHandle n;
  path_pub = n.advertise<nav_msgs::Path>("path", 1000);

  ros::Subscriber point_sub = n.subscribe("/odom", 1000, PointRecieved); //subscribing to the wrong size will crash

  ros::Rate loop_rate(10);

  ros::spin();


  return 0;
}



