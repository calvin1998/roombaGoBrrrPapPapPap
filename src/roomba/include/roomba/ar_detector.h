#ifndef AR_DETECTOR_HPP_
#define AR_DETECTOR_HPP_

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>

#include <opencv2/aruco.hpp>


class ARDetector {
    private:
        image_transport::Subscriber cameraSub;//subscribes to camera images
        ros::Publisher markerPub;//publishes marker present?

    public:
        ARDetector();
        void startup();
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif