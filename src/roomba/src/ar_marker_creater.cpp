#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

visualization_msgs::MarkerArray marker_array;
std::vector<long> markerNumDetect;
ros::Publisher marker_pub;
ros::Publisher done_pub;
int num_markers;

static const std::string ns = "basic_shapes";

void updateMarkerAtPosition(int pos, geometry_msgs::Pose pose) {
  visualization_msgs::Marker marker = marker_array.markers[pos];
  /*
  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  marker.pose.position.z = pose.position.z;
  */
  
  long n = markerNumDetect[pos];
  marker.pose.position.x = (marker.pose.position.x * n + pose.position.x) / (n + 1);
  marker.pose.position.y = (marker.pose.position.y * n + pose.position.y) / (n + 1);
  marker.pose.position.z = (marker.pose.position.z * n + pose.position.z) / (n + 1);

  //marker.pose.orientation = pose.orientation;
  markerNumDetect[pos]++;
  
  marker_array.markers[pos] = marker;
}

void MarkerRecieved(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
  /*
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
  */

  for (int j = 0; j < msg->markers.size(); j++) {
    ar_track_alvar_msgs::AlvarMarker newAlvarMarker = msg->markers[j];
    bool existing = false;
    for(int i = 0; i < marker_array.markers.size(); i++){
      visualization_msgs::Marker marker = marker_array.markers[i];
      if((marker.id == newAlvarMarker.id) ||
        (abs(marker.pose.position.x - newAlvarMarker.pose.pose.position.x) < 0.2 
                && abs(marker.pose.position.y - newAlvarMarker.pose.pose.position.y) < 0.2)) {
        //Check if there is already a marker at that location
        //if a marker eixistis with a differnet ID then ignore it
        if (newAlvarMarker.confidence)
        updateMarkerAtPosition(i, newAlvarMarker.pose.pose);
        existing = true;
        break;
      }
    }

    if (!existing){
      visualization_msgs::Marker newVisMarker;

      newVisMarker.header = newAlvarMarker.header;
      newVisMarker.ns = ns;
      newVisMarker.id = newAlvarMarker.id;
      newVisMarker.type = visualization_msgs::Marker::CUBE;
      newVisMarker.action = visualization_msgs::Marker::ADD;
      newVisMarker.pose = newAlvarMarker.pose.pose;
      newVisMarker.scale.x = 0.2;
      newVisMarker.scale.y = 0.2;
      newVisMarker.scale.z = 0.04;
      newVisMarker.color.a = 1.0;
      newVisMarker.color.r = 1.0;
      newVisMarker.color.g = 0;
      newVisMarker.color.b = 0;
      newVisMarker.lifetime = ros::Duration(0); //last forever
      newVisMarker.frame_locked = false;

      marker_array.markers.push_back(newVisMarker);
      markerNumDetect.push_back(1);
      ROS_INFO("ADDING NEW MARKER");
    }
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
  ros::init(argc, argv, ns);
  ros::NodeHandle n;

  n.param("/ar_maker_publisher/num_markers", num_markers,3);
   

  marker_pub = n.advertise<visualization_msgs::MarkerArray>("avlr_visualization_markerArray", 1);
  done_pub = n.advertise<std_msgs::String>("/ar_marker_detect_done",1);
  ros::Subscriber marker_sub = n.subscribe("/ar_pose_marker",1000,MarkerRecieved);

  ros::Rate loop_rate(10);
  // Set our initial shape type to be a cube
  ros::spin();
}