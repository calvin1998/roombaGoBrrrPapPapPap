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
        //cv image to ros parameters
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        //results of marker detection
        std::vector<int> markerIds;//index of markers in dictionary
        markerCoords markerCorners, rejectedCandidates;//location of corners in (r,c) or (y,x)

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
        //get image for debug purposes
        void showImageWithMarkerOverlay(cv::Mat inputImage);//returns copy of image with overlaid markers
};

#endif