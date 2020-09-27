#include "roomba/ar_detector.h"

#define LOG_START    "AR Detector ::"

int main_ar_detector(int argc, char** argv) {
    ros::init(argc, argv, "ar_detector");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
    ros::NodeHandle nh("~");

    ARDetector arDetector;
    arDetector.configure();
    arDetector.startup();

    while (ros::ok()) {
        ROS_INFO("%s Spinning", LOG_START);
        ros::spin();
    }

    // Disconnect
    arDetector.shutdown();

    return 0;
}

// Actual main method outside of namespace
int main(int argc, char** argv) {
    main_ar_detector(argc, argv);
}
