#include <ros/ros.h>
#include <geometry_msgs/Point.h>


void callback(const geometry_msgs::Point::ConstPtr& msg)
{
  ROS_INFO_STREAM("Received robot location: " << msg->x << ", " << msg->y);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_location_subscriber");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("robot_location", 100, callback);
  ros::spin();
}


