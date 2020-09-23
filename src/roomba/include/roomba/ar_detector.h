#ifndef AR_DETECTOR_HPP_
#define AR_DETECTOR_HPP_

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <sensor_msgs/Image.h>

#include <opencv2/aruco.hpp>


class ARDetector {
    private:
        ros::Subscriber cameraSub;//subscribes to camera images
        ros::Publisher markerPub;//publishes marker present?

    public:
        ARDetector();
        void startup();
};

#endif