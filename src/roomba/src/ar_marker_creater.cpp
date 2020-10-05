#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

visualization_msgs::MarkerArray marker_array;
ros::Publisher marker_pub;
ros::Publisher done_pub;
int num_markers;

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
  for(auto marker : marker_array.markers){
    if(marker.ns == msg->ns && marker.id == msg->id){
      marker = m;
      existing = true;
      break;
    } else if(abs(marker.pose.position.x - msg->pose.position.x) < 0.2 && abs(marker.pose.position.y - msg->pose.position.y) < 0.2){
      //Check if there is already a marker at that location
      //if a marker eixistis with a differnet ID then ignore it 
      existing = true;
      break;
    }
  }

  if (!existing){
     marker_array.markers.push_back(m);
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

  n.param("/ar_maker_publisher/num_markers", num_markers,3);
   

  marker_pub = n.advertise<visualization_msgs::MarkerArray>("avlr_visualization_markerArray", 1);
  done_pub = n.advertise<std_msgs::String>("/ar_marker_detect_done",1);
  ros::Subscriber marker_sub = n.subscribe("/visualization_marker",1000,MarkerRecieved);

  ros::Rate loop_rate(10);
  // Set our initial shape type to be a cube
  ros::spin();
}