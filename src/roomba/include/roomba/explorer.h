#ifndef ROOMBA_EXPLORER_HPP_
#define ROOMBA_EXPLORER_HPP_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "actionlib_msgs/GoalStatusArray.h"

class Explorer {
    private:
        void returnToStart(move_base_msgs::MoveBaseGoal& goal);
        void handleFrontierStatus(const std_msgs::StringConstPtr& str);
        void handleARFinished(const std_msgs::StringConstPtr& str);
        void sendToWallFollow(const std::string str);
        void whereWeAt(const actionlib_msgs::GoalStatusArray& msg);

        ros::Subscriber frontierStatusSub;
        ros::Subscriber arDetectedStatusSub;
        ros::Subscriber moveBaseStatSub;
        tf::TransformListener tfListener;
        ros::Publisher wallFollowerPub;
        move_base_msgs::MoveBaseGoal finalGoal;
        ros::NodeHandle n;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
            move_base_client_;

        int numRetries;
        bool arFinished;
        bool frontierFinished;

    public:
        Explorer();
        void startup();
};


#endif