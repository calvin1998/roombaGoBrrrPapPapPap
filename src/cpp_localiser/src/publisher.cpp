#include <ros/ros.h>
#include <geometry_msgs/Point.h>

/*
 * NOTE: This is a simple version. A better version would be to publish on a timer
 */


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_location_publisher");
  ros::NodeHandle nh;

  ros::Publisher location_pub = nh.advertise<geometry_msgs::Point>("robot_location", 10);
  ros::Rate loop_rate(1);

  // Initialise the location of the robot
  geometry_msgs::Point location;
  location.x = 10;
  location.y = 10;
  location.z = 10;

  while (ros::ok())
  {
     ROS_INFO_STREAM("Publishing robot location: " << location.x << ", " << location.y);
     location_pub.publish(location);

     // Simulate a move is some direction
     location.x += 1;
     location.y += 2;

     ros::spinOnce();
     loop_rate.sleep();
  }
}


