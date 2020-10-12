#include <ros/ros.h>
#include "roomba/explorer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

Explorer::Explorer()
  : move_base_client_("move_base")
  , numRetries{0}
  , arFinished{false}
  , frontierFinished{false}
  , numMazeLoops{0}
  , leftSpawn{false}
{
  ROS_INFO("Dora the explorer started yall");

  // Wait for movebase
  while(!move_base_client_.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Get initial pose
  nav_msgs::OdometryConstPtr msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(5.0));
  finalGoal.target_pose.header.frame_id = "map";
  finalGoal.target_pose.header.stamp = ros::Time::now();
  finalGoal.target_pose.pose = msg->pose.pose;

  /*
  tf2::Quaternion q_orig, q_rot, q_new;

  // Get the original orientation of 'commanded_pose'
  tf2::convert(msg->pose.pose.orientation , q_orig);

  double r=0, p=0, y=3.14159;  // Rotate the previous pose by 180* about X
  q_rot.setRPY(r, p, y);

  q_new = q_rot*q_orig;  // Calculate the new orientation
  q_new.normalize();

  // Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_new, finalGoal.target_pose.pose.orientation);
  */
  ROS_INFO("Initial pose logged");

  // Start subscription to frontier
  ROS_INFO("Subscription started");
  frontierStatusSub = n.subscribe("/explore/status", 1000, &Explorer::handleFrontierStatus, this);
  arDetectedStatusSub = n.subscribe("/ar_marker_detect_done", 100, &Explorer::handleARFinished, this);
  wallFollowerPub = n.advertise<std_msgs::String>("/cmd", 10);
  moveBaseStatSub = n.subscribe("/move_base/status", 100, &Explorer::whereWeAt, this);
}

void Explorer::handleARFinished(const std_msgs::StringConstPtr& str) {
  if (str->data.compare("done") == 0) {
    arDetectedStatusSub.shutdown();
    arFinished = true;
    
    // With frontier
    // if (frontierFinished) {
    //   sendToWallFollow("stop");
    //   returnToStart(finalGoal);
    // }

    // Without frontier
    returnToStart(finalGoal);
  }
}

void Explorer::sendToWallFollow(const std::string str) {
  std_msgs::String msg;
  msg.data = str;
  wallFollowerPub.publish(msg);
}

void Explorer::handleFrontierStatus(const std_msgs::StringConstPtr& str) {
  // if (str->data.compare("stop") == 0) {
  //   ROS_INFO("frotiner stopped buddy");
  //   frontierStatusSub.shutdown();
  //   frontierFinished = true;
  //   if (arFinished) {
  //     returnToStart(finalGoal);
  //   } else {
  //     sendToWallFollow("start");
  //   }

  // } else {
  //   ROS_INFO("Frontier still running");
  // }
}

void Explorer::startup()
{
  
  ros::Rate loop_rate(10);
  sendToWallFollow("start");
  ros::spin();
}
  

void Explorer::returnToStart(move_base_msgs::MoveBaseGoal& goal) {
  // moveBaseStatSub = n.subscribe("/move_base/status", 100, &Explorer::whereWeAt, this);
  // ROS_INFO("Sending goal");
  // move_base_client_.sendGoal(goal);
  // move_base_client_.waitForResult();

  // if(move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
  //   ROS_INFO("Returned to start");
  // } else if (numRetries <= 3) {
  //   ROS_INFO("retrying...");
  //   returnToStart(goal);
  // } else {
  //   sendToWallFollow("start");
  // }
}

void Explorer::whereWeAt(const actionlib_msgs::GoalStatusArray& msg) {
  try {
    tf::StampedTransform transform;
    tfListener.lookupTransform("base_link", "odom", ros::Time(0), transform);
    auto x = transform.getOrigin().getX();
    auto y = transform.getOrigin().getY();
    auto z = transform.getOrigin().getZ();
    
    auto x_diff = fabs(x - finalGoal.target_pose.pose.position.x);
    auto y_diff = fabs(y - finalGoal.target_pose.pose.position.y);
    auto z_diff = fabs(z - finalGoal.target_pose.pose.position.z);
    
    ROS_INFO("x: %f y: %f z: %f, AR Finished: %d, LeftSpawn %d", x_diff, y_diff, z_diff, arFinished,leftSpawn);

    if(x_diff > 0.3 && y_diff > 0.3 && leftSpawn == false){
      ROS_INFO("##### LEFT SPAWN ########");
      leftSpawn = true;
    }

    if (x_diff < 0.15 && y_diff < 0.15 && leftSpawn) {
      
      numMazeLoops++;
      ROS_INFO("##### MAZE LOOP %d ########",numMazeLoops);
      leftSpawn = false;
      if(arFinished || numMazeLoops > 1){
        ROS_INFO("We are home");
        sendToWallFollow("stop");
        move_base_client_.cancelAllGoals();
        moveBaseStatSub.shutdown();
      } else {
        sendToWallFollow("right");
      }
    }
  } catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "explorer");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  Explorer ex;
  sleep(1);
  ex.startup();
  return 0;
}