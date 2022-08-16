#include <std_msgs/Empty.h>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sstream>

#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#pragma once

namespace botello_aruco
{

class BotelloArucoNode
{
private:
    // ================
    // Member variables.
    // ================

    // Node handle.
    ros::NodeHandle mnh;

    // Transform broadcaster.
    tf::TransformBroadcaster mbr;

    // Image transport.
    // image_transport::ImageTransport mit;

    // Verbose.
    bool mVerbose;

    // Camera info flag and details.
    bool mHaveCamInfo;
    cv::Mat mCameraMatrix;
    cv::Mat mDistortionCoeffs;
    std::string mCamFrameId;
    std::pair<int,int> mCamImageWidthHeight; 

    // Parameters for aruco detection.
    cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams = cv::aruco::DetectorParameters::create();


    // ================
    // Parameters.
    // ================
    // Tag sizes.
    double mTagLength;
    // Region of interest.
    int mImageRoiBrimX;
    int mImageRoiBrimY;

    // ================
    // Methods.
    // ================
    bool getParamsFromParamServer();

    /**
     * Given a message, transforms observed fiducial detections to the base_link frame.
     */
    void processImage(const sensor_msgs::ImageConstPtr & msg);

    /**
     * Set the camera info information from a message.
     */
    bool setCamInfoFromMsg(const sensor_msgs::CameraInfo::ConstPtr& msg);

    /**
     * Compute tag transform from a the closest of the tags in a given corners observation (multiple observations of 4 corners).
     * See https://github.com/UbiquityRobotics/fiducials/blob/noetic-devel/aruco_detect/src/aruco_detect.cpp 
     * Return true if successful.
     */
    bool estimatePoseClosestMarker(const std::vector<std::vector<cv::Point2f>> & corners,
                                   float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   cv::Vec3d& rvec, cv::Vec3d& tvec,
                                   double& reprojectionError,
                                   int& closestTagIx);

    /**
     * Set the mDetectorParams object.
     */
    void setArucoDetectorParams();

    // ================
    // Listeners.
    // ================
    // image_transport::Subscriber mImageRawSub;
    ros::Subscriber mImageRawSub;
    ros::Subscriber mSubCamInfo;
    // ================
    // Publishers.
    // ================
    // ros::Publisher mCmdVelPub;

    
    // ================
    // Callbacks.
    // ================
    // Process images.
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void camInfoCb(const sensor_msgs::CameraInfo::ConstPtr & msg);


    //***********************************
    // Utils functions
    //***********************************
    bool getTagsLocations(); // TODO
    
    /**
    * Compute area in image of a fiducial, using Heron's formula
    * to find the area of two triangles
    */
    double calcFiducialArea(const std::vector<cv::Point2f> &pts);
    
    /**
      * Return object points for the system centered in a single marker, given the marker length
      */
    void getSingleMarkerObjectPoints(float markerLength, std::vector<cv::Point3f>& objPoints);

    /**
     * Euclidean distance between two points
     */
    double dist(const cv::Point2f &p1, const cv::Point2f &p2);

    /**
     * Estimate reprojection error. 
     */
    double getReprojectionError(const std::vector<cv::Point3f> &objectPoints,
                            const std::vector<cv::Point2f> &imagePoints,
                            const cv::Mat &cameraMatrix, const cv::Mat  &distCoeffs,
                            const cv::Vec3d &rvec, const cv::Vec3d &tvec);

    /**
     * Compute standard deviation for a vector of doubles.
     */
    double standardDeviation(std::vector<double> & v);


public:
    BotelloArucoNode(const ros::NodeHandle & nh);
    ~BotelloArucoNode();

};

} // namespace botello_aruco
