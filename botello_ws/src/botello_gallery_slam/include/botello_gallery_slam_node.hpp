#include <sstream>
#include <numeric>
#include <fstream>

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "./types.hpp"
#pragma once
namespace botello_gallery_slam
{

class BotelloGallerySlamNode
{
private:
    // ================
    // Member variables.
    // ================

    // Node handle.
    ros::NodeHandle mnh;

    // Transform broadcaster.
    tf::TransformBroadcaster mbr;

    // Verbose.
    bool mVerbose;

    // Camera info flag and details.
    bool mHaveCamInfo;
    cv::Mat mCameraMatrix;
    cv::Mat mDistortionCoeffs;
    std::string mCamImageFrameId;
    std::pair<int,int> mCamImageWidthHeight; 

    // Storage of observations, odometry guesses (in odom frame), and odom steps (constraints).
    std::vector<LandmarkObservation> mLandmarkObservations; 
    std::vector<Pose3d> mBaselinkInOdomPoses;


    // Keep track of the frame ID.
    unsigned long mImageFrameId = 0;

    // Count number of frames skipped. Each time that enough frames were skipped, then a frame is registered alongside with the most recent odometry reading.
    int mNumFramesSkipped = 0;

    // Recent odom pose.
    tf::Transform mLastBaseLinkInOdom;




    // ================
    // Parameters.
    // ================
    double mTagLength;
    // Parameters for aruco detection.
    cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams = cv::aruco::DetectorParameters::create();

    // Confidence values.
    double mLandmarkObservationConfidence;

    // Only track one landmark. If this is set to != -1, then this ID of landmark will be the only one reported in the dataset.
    int mOnlyCaptureLandmarkId;

    // Skip this number of frames between dataset updates. It is necessary to have this number be at least 1, since images are at 30Hz and odom is at 20Hz for the tello.
    int mDatasetSkipFrames;

    // ================
    // Methods.
    // ================
    bool getParamsFromParamServer();
    void writeDatasetToFile(const std::string & fpath = "./pose_pix_orig.txt");

    /**
     * Given a message, transforms observed fiducial detections to the base_link frame.
     */
    void processImage(const sensor_msgs::ImageConstPtr & msg);

    /**
     * Given a message, add an odometry constraint to the dataset.
     */
    void processOdom(const nav_msgs::Odometry & msg);

    /**
     * Compute tag transform from a the closest of the tags in a given corners observation (multiple observations of 4 corners).
     * See https://github.com/UbiquityRobotics/fiducials/blob/noetic-devel/aruco_detect/src/aruco_detect.cpp 
     * Return true if successful.
     */
    bool estimatePoseToMarker(const std::vector<std::vector<cv::Point2f>> & corners,
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

    /**
     * Optimize observations and get positions of landmarks, alongside camera poses and camera intrinsics.
     */
    void solveSlamProblem();




    // ================
    // Listeners.
    // ================
    // image_transport::Subscriber mImageRawSub;
    ros::Subscriber mImageRawSub;
    ros::Subscriber mOdomSub;
    
    // ================
    // Publishers.
    // ================
    // ros::Publisher mCmdVelPub;

    
    // ================
    // Callbacks.
    // ================
    // Process images.
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void odomCb(const nav_msgs::Odometry& msg);


    //***********************************
    // Utils functions
    //***********************************
    
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
    BotelloGallerySlamNode(const ros::NodeHandle & nh);
    ~BotelloGallerySlamNode();

};

} // namespace botello_gallery_slam
