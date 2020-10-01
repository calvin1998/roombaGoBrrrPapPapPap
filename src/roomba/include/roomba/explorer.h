#ifndef ROOMBA_EXPLORER_HPP_
#define ROOMBA_EXPLORER_HPP_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/String.h"

class Explorer {
    private:
        void returnToStart(move_base_msgs::MoveBaseGoal& goal);
        void handleFrontierStatus(const std_msgs::StringConstPtr& str);

        ros::Subscriber frontierStatusSub;
        move_base_msgs::MoveBaseGoal finalGoal;
        ros::NodeHandle n;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
            move_base_client_;
        int numRetries;

    public:
        Explorer();
        void startup();
};


#endif