#ifndef AR_DETECTOR_HPP_
#define AR_DETECTOR_HPP_

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/types.hpp>
#include <image_transport/image_transport.h>

typedef std::vector<std::vector<float>> mapCoords;
typedef std::vector<std::vector<cv::Point2f>> markerCoords;

class ARDetector {
    private:
        image_transport::Subscriber cameraSub;//subscribes to camera images
        ros::Publisher markerPub;//publishes marker present?
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
        cv::Ptr<cv::aruco::Dictionary> dictionary;

    public:
        ARDetector();
        //initialisation of ARDetector node
        void startup();
        //callback to perform when image topic is found
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        //converts ros image to cv image
        cv_bridge::CvImagePtr rosToCvImage(const sensor_msgs::ImageConstPtr& msg);
        //gets MAP coordinates of markers from detected AR points
        mapCoords getARMarkerCoords(markerCoords markerCorners);
};

#endif