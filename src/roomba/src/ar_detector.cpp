#include "roomba/ar_detector.h"
#include "roomba/pub_sub_topics.h"
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <iostream>

#define LOG_START       "ARDetector ::"


ARDetector::ARDetector() {

};

void ARDetector::configure() {
    ros::NodeHandle paramNh("~");
    image_topic = paramNh.param<std::string>("imageTopic", "image");
    ROS_INFO("%s Using image topic: %s", LOG_START, image_topic.c_str());
}


void ARDetector::startup() {
    //initialise publishers and subscribers
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    cameraSub = it.subscribe(image_topic, 1, &ARDetector::imageCallback, this);
    //markerPub = //example is nh.advertise<geometry_msgs::Twist >("cmd_vel", 1, false);
    //TODO: which opencv params to use?
    detectorParams = cv::aruco::DetectorParameters::create();
    //TODO: which dictionary do the markers belong to?
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
}

void ARDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    //convert ros image to cv image
    cv_bridge::CvImagePtr cvImagePtr = rosToCvImage(msg);//this pointer is self managed
    //if image conversion failed
    if (cvImagePtr == NULL) {
        std::cout << "Failed to convert image from ROS to OpenCV" << std::endl;
        return;//TODO: Any better way to handle failure?
    }
    //std::cout << "M = " << std::endl << cvImagePtr->image << std::endl << std::endl;
    //else detect markers
    cv::aruco::detectMarkers(cvImagePtr->image, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
    showImageWithMarkerOverlay(cvImagePtr->image);//uncomment if you want to debug/visualise markers
    //TODO: Publish marker locations with getARMarkerCoords
}

cv_bridge::CvImagePtr ARDetector::rosToCvImage(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr newCvImagePtr;
    try {
        //TODO: ensure image encoding is correct
        newCvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);//if ros image not modified, toCvShare may be used
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return NULL;
    }
    return newCvImagePtr;
}

mapCoords ARDetector::getARMarkerCoords(markerCoords markerCorners) {

}

void ARDetector::showImageWithMarkerOverlay(cv::Mat inputImage) {
    if (inputImage.empty()) {
        std::cout << "Input image empty." << std::endl;
        return;
    }

    cv::namedWindow("Without Marker", cv::WINDOW_AUTOSIZE);
    cv::imshow("Without Marker", inputImage);

    cv::waitKey(100);

    cv::Mat rejectedImage = inputImage.clone();

    if (rejectedImage.empty()) {
        std::cout << "Failed to show image with rejected markers. Image is empty." << std::endl;
        return;
    }

    if (rejectedCandidates.size() > 0) {
        cv::namedWindow("Rejected", cv::WINDOW_AUTOSIZE);
        //draw markers onto image
        cv::aruco::drawDetectedMarkers(rejectedImage, rejectedCandidates);
        //show images
        cv::imshow("Rejected", rejectedImage);
        cv::waitKey(100);
    }

    cv::Mat outputImage = inputImage.clone();

    if (outputImage.empty()) {
        std::cout << "Failed to show image with marker overlay. Image is empty." << std::endl;
        return;
    }

    if (markerCorners.size() == 0 || markerIds.size() == 0) {
        std::cout << "Failed to show image with marker overlay. No image or markers found." << std::endl;
        std::cout << "Marker corners detected: " << markerCorners.size() << "\tMarker Ids: " << markerIds.size() 
        << "\tRejected Markers " << rejectedCandidates.size() << std::endl;
        return;
    }
    cv::namedWindow("With Marker", cv::WINDOW_AUTOSIZE);
    //draw markers onto image
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    //show images
    cv::imshow("With Marker", outputImage);
    cv::waitKey(100);
    //cv::waitKey;(1000);//wait for 1000ms OR keypress before continuing NOTE:Is a blocking activitiy
    return;
}

void ARDetector::shutdown() {

}


