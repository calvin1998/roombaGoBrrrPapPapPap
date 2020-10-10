#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <vector>

visualization_msgs::MarkerArray marker_array;
std::vector<int> markerNumDetect; // number of detections per marker

ros::Publisher marker_pub;
ros::Publisher done_pub;
int num_markers;

void updateMarkerAtPosition(int pos, visualization_msgs::Marker m) {
  visualization_msgs::Marker marker = marker_array.markers[pos];
  int n = markerNumDetect[pos];
  marker.pose.position.x = (marker.pose.position.x * n + m.pose.position.x) / (n + 1);
  marker.pose.position.y = (marker.pose.position.y * n + m.pose.position.y) / (n + 1);
  marker.pose.position.z = (marker.pose.position.z * n + m.pose.position.z) / (n + 1);
  markerNumDetect[pos]++;
}

void MarkerRecieved(const visualization_msgs::Marker::ConstPtr& msg){
  visualization_msgs::Marker m;
  m.header = msg->header;
  m.ns = msg->ns;
  m.id = msg->id;
  m.type = msg->type;
  m.action = visualization_msgs::Marker::ADD;
  m.pose = msg->pose;
  m.scale = msg->scale;
  m.color = msg->color;
  m.lifetime = ros::Duration(0); //last forever
  m.frame_locked = msg->frame_locked;

  bool existing = false;
  visualization_msgs::Marker *existingMarker;
  for (int i = 0; i < marker_array.markers.size(); i++) {
    visualization_msgs::Marker marker = marker_array.markers[i];
    if ((marker.ns == m.ns && marker.id == m.id) || 
        (abs(marker.pose.position.x - m.pose.position.x) < 0.2 && abs(marker.pose.position.y - m.pose.position.y) < 0.2)) {
      //set marker pose to be running average of marker pose
      updateMarkerAtPosition(i, m);
      existing = true;
      break;
    }
  }

  if (!existing) {
     marker_array.markers.push_back(m);
     markerNumDetect.push_back(1);
     ROS_INFO("ADDING NEW MARKER");
  }
  marker_pub.publish(marker_array);

  if(marker_array.markers.size() >= num_markers){
    std_msgs::String msg; 
    msg.data = "done";
    done_pub.publish(msg);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;

  n.param("/ar_maker_publisher/num_markers", num_markers, 3);
  

  marker_pub = n.advertise<visualization_msgs::MarkerArray>("avlr_visualization_markerArray", 1);
  done_pub = n.advertise<std_msgs::String>("/ar_marker_detect_done",1);
  ros::Subscriber marker_sub = n.subscribe("/visualization_marker",1000,MarkerRecieved);

  ros::Rate loop_rate(10);
  // Set our initial shape type to be a cube
  ros::spin();
}