#include "botello_aruco_node.hpp"

/*
Some code adapted from aruco_detect ROS package.
*/

namespace botello_aruco
{




BotelloArucoNode::BotelloArucoNode(const ros::NodeHandle & nh) : mnh(nh)
{
    ROS_INFO("Instantiating BotelloArucoNode.");

    // Get params from server.
    getParamsFromParamServer();

    // Image transport.
    // image_transport::ImageTransport mit(mnh);

    // The correct way to do this is via image_transport. I (yoraish) could not, however, get it to work with a class member function as a callback so I chose to go with an ordinary subscriber. Let me know if you know how to use an image_transport subscriber with a member method as a callback.
    // image_transport::Subscriber mImageRawSub = mit.subscribe("/tello/image_raw", 1, BotelloArucoNode::imageCb);
    
    mImageRawSub = mnh.subscribe("/tello/camera/image_raw", 1, & BotelloArucoNode::imageCb, this);
    mSubCamInfo = mnh.subscribe("/tello/camera/camera_info", 1, & BotelloArucoNode::camInfoCb, this);

    // Aruco detection params.
    setArucoDetectorParams();

    // Member variables default values.
    mHaveCamInfo =  false;
    mCameraMatrix =  cv::Mat::zeros(3, 3, CV_64F);
    mDistortionCoeffs =  cv::Mat::zeros(1, 5, CV_64F);
    mCamImageWidthHeight =  std::make_pair(0, 0);

    // START params that will move to a yaml.
    mVerbose = true;
    // END params that will move to a yaml.

    ROS_INFO("Init BotelloArucoNode done.");

    // cv::namedWindow("view");
    // cv::startWindowThread();
}

BotelloArucoNode::~BotelloArucoNode()
{
    // cv::destroyWindow("view");
}

bool BotelloArucoNode::getParamsFromParamServer()
{
    ros::NodeHandle pnh("botello_aruco_node"); // To fetch params from the private namespace.


    if (!pnh.param<bool>("verbose", mVerbose, true))
    {
    ROS_ERROR_STREAM("Failed to get parameter verbose from server.");
    }
    if (!pnh.param<double>("tag_length", mTagLength, 0.066))
    {
    ROS_ERROR_STREAM("Failed to get parameter tag_length from server.");
    }
    if (!pnh.param<int>("image_roi_brim_x", mImageRoiBrimX, 400))
    {
    ROS_ERROR_STREAM("Failed to get parameter image_roi_brim_x from server.");
    }
    if (!pnh.param<int>("image_roi_brim_y", mImageRoiBrimY, 200))
    {
    ROS_ERROR_STREAM("Failed to get parameter image_roi_brim_y from server.");
    }


}


void BotelloArucoNode::setArucoDetectorParams()
{

    // Set the aruco detector parameters.
    ros::NodeHandle pnh("botello_aruco_node"); // To fetch params from the private namespace.
    if (!pnh.param<double>("adaptiveThreshConstant", mDetectorParams->adaptiveThreshConstant, 7))
    {
        ROS_ERROR_STREAM("Failed to get parameter adaptiveThreshConstant from server.");
    }
    if (!pnh.param<int>("adaptiveThreshWinSizeMin", mDetectorParams->adaptiveThreshWinSizeMin, 3))
    {
        ROS_ERROR_STREAM("Failed to get parameter adaptiveThreshWinSizeMin from server.");
    }
    if (!pnh.param<int>("adaptiveThreshWinSizeMax", mDetectorParams->adaptiveThreshWinSizeMax, 30))
    {
        ROS_ERROR_STREAM("Failed to get parameter adaptiveThreshWinSizeMax from server.");
    }
    if (!pnh.param<int>("adaptiveThreshWinSizeStep", mDetectorParams->adaptiveThreshWinSizeStep, 4))
    {
        ROS_ERROR_STREAM("Failed to get parameter adaptiveThreshWinSizeStep from server.");
    }
    if (!pnh.param<int>("cornerRefinementMaxIterations", mDetectorParams->cornerRefinementMaxIterations, 30))
    {
        ROS_ERROR_STREAM("Failed to get parameter cornerRefinementMaxIterations from server.");
    }
    if (!pnh.param<double>("cornerRefinementMinAccuracy", mDetectorParams->cornerRefinementMinAccuracy, 0.01))
    {
        ROS_ERROR_STREAM("Failed to get parameter cornerRefinementMinAccuracy from server.");
    }
    if (!pnh.param<int>("cornerRefinementWinSize", mDetectorParams->cornerRefinementWinSize, 5))
    {
        ROS_ERROR_STREAM("Failed to get parameter cornerRefinementWinSize from server.");
    }
    if (!pnh.param<double>("errorCorrectionRate", mDetectorParams->errorCorrectionRate, 0.6))
    {
        ROS_ERROR_STREAM("Failed to get parameter errorCorrectionRate from server.");
    }
    if (!pnh.param<double>("minCornerDistanceRate", mDetectorParams->minCornerDistanceRate, 0.05))
    {
        ROS_ERROR_STREAM("Failed to get parameter minCornerDistanceRate from server.");
    }
    if (!pnh.param<int>("markerBorderBits", mDetectorParams->markerBorderBits, 1))
    {
        ROS_ERROR_STREAM("Failed to get parameter markerBorderBits from server.");
    }
    if (!pnh.param<double>("maxErroneousBitsInBorderRate", mDetectorParams->maxErroneousBitsInBorderRate, 0.04))
    {
        ROS_ERROR_STREAM("Failed to get parameter maxErroneousBitsInBorderRate from server.");
    }
    if (!pnh.param<int>("minDistanceToBorder", mDetectorParams->minDistanceToBorder, 3))
    {
        ROS_ERROR_STREAM("Failed to get parameter minDistanceToBorder from server.");
    }
    if (!pnh.param<double>("minMarkerDistanceRate", mDetectorParams->minMarkerDistanceRate, 0.05))
    {
        ROS_ERROR_STREAM("Failed to get parameter minMarkerDistanceRate from server.");
    }
    if (!pnh.param<double>("minMarkerPerimeterRate", mDetectorParams->minMarkerPerimeterRate, 0.1))
    {
        ROS_ERROR_STREAM("Failed to get parameter minMarkerPerimeterRate from server.");
    }
    if (!pnh.param<double>("maxMarkerPerimeterRate", mDetectorParams->maxMarkerPerimeterRate, 4.0))
    {
        ROS_ERROR_STREAM("Failed to get parameter maxMarkerPerimeterRate from server.");
    }
    if (!pnh.param<double>("minOtsuStdDev", mDetectorParams->minOtsuStdDev, 5.0))
    {
        ROS_ERROR_STREAM("Failed to get parameter minOtsuStdDev from server.");
    }
    if (!pnh.param<double>("perspectiveRemoveIgnoredMarginPerCell", mDetectorParams->perspectiveRemoveIgnoredMarginPerCell, 0.13))
    {
        ROS_ERROR_STREAM("Failed to get parameter perspectiveRemoveIgnoredMarginPerCell from server.");
    }
    if (!pnh.param<int>("perspectiveRemovePixelPerCell", mDetectorParams->perspectiveRemovePixelPerCell, 8))
    {
        ROS_ERROR_STREAM("Failed to get parameter perspectiveRemovePixelPerCell from server.");
    }
    if (!pnh.param<double>("polygonalApproxAccuracyRate", mDetectorParams->polygonalApproxAccuracyRate, 0.01))
    {
        ROS_ERROR_STREAM("Failed to get parameter polygonalApproxAccuracyRate from server.");
    }

    mDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

}

void BotelloArucoNode::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    processImage(msg);
}





void BotelloArucoNode::processImage(const sensor_msgs::ImageConstPtr & msg)
{
    // ===========
    // CAMERA INFO FILTER.
    // ===========
    if (!mHaveCamInfo)
    {
        return;
    }

    tf::Quaternion qFrontCamInBaselink(-0.5, 0.5, -0.5, 0.5);
    tf::Matrix3x3 rotFrontCamInBaselink(qFrontCamInBaselink);
    tf::Vector3 tFrontCamInBaselink(0.03, 0, 0);
    tf::Transform  camInBaselink = tf::Transform(rotFrontCamInBaselink, tFrontCamInBaselink);

    // Placeholder transform between camera frame (parent) and tag.
    tf::Transform tagInCam;

    // Placeholder for tag frame (populated with ID of closest tag).
    std::string tagFrame;

    // Find fiducials in the image.
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(8); // TODO(yoraish): to yaml.
    std::vector <std::vector <cv::Point2f> > corners; // TODO(yoraish): class variable?
    std::vector <int> ids; // TODO(yoraish): class variable?


    try 
    {
        // Convert image to cv::Mat for processing.
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        /*
        cv::Mat croppedImg = cv_ptr->image;
        croppedImg = croppedImg(cv::Range(mImageRoiBrimY, mCamImageWidthHeight - mImageRoiBrimY), cv::Range(mImageRoiBrimX, mCamImageWidthHeight - mImageRoiBrimX)); 
       
        // Publish an image with the cropped region marked, if asked for verbose operation.
        if (mVerbose && false)
        {
            cv::Mat debugImg = cv_ptr->image.clone();
            // Draw rectangle.
            cv::Point topLeft(mImageRoiBrimX, mImageRoiBrimY);
            cv::Point bottomRight(mCamImageWidthHeight - mImageRoiBrimX,  mCamImageWidthHeight - mImageRoiBrimY);
            cv::rectangle(debugImg, topLeft, bottomRight, cv::Scalar(255, 0, 0), 5);

        
            // If running on a non-headless setup, optionally show images for debug.    
            cv::imshow("test", debugImg);
            cv::waitKey(0);
        }
        */ 
        
        cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, mDetectorParams);
        // cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, mDetectorParams);

        // Check that some tags have been found.
        int nTagsDetected = (int)corners.size();
        if (nTagsDetected == 0)
        {
            if(mVerbose)
            {
                ROS_INFO_STREAM("No tags in image.");
            }
            return;
        }
        
        // Compute the relative transform of the tag in the camera frame.
        cv::Vec3d  rvec, tvec;
        double reprojectionError;
        int closestTagIx;
        if (!estimatePoseClosestMarker(corners, 
                                    mTagLength,
                                    mCameraMatrix, mDistortionCoeffs,
                                    rvec, tvec,
                                    reprojectionError,
                                    closestTagIx))
        {
            if (mVerbose)
            {
                ROS_INFO_STREAM("Could not estimate pose for closest tag.");
            }
            return;
        }

        if(mVerbose)
        {
            for (int id : ids)
            {
                ROS_INFO_STREAM("   Detected id " << id);
            }
            ROS_INFO_STREAM("   Detected closest ID: " << ids[closestTagIx] << " T: [" << tvec[0] << ", " << tvec[1] << ", " << tvec[2] << "]" <<  " R: [" << rvec[0] << ", " << rvec[1] << ", " << rvec[2] << "]");
        }

        double angle = norm(rvec);
        cv::Vec3d axis = rvec / angle;

        double object_error =
                (reprojectionError / dist(corners[closestTagIx][0], corners[closestTagIx][2])) *
                (norm(tvec) / mTagLength);

        // Publish tf for the fiducial relative to the camera.
        tf::Quaternion qTagInCam;
        qTagInCam.setRotation(tf::Vector3(axis[0], axis[1], axis[2]), angle);
        tf::Vector3 tTagInCam(tvec[0], tvec[1], tvec[2]);
        tagInCam = tf::Transform(qTagInCam, tTagInCam);
        tagFrame = "tag" + std::to_string(ids[closestTagIx]);
    }
    catch(cv_bridge::Exception & e) {
        ROS_ERROR_STREAM("cv_bridge exception:" << e.what());
        return;
    }
    catch(cv::Exception & e) {
        ROS_ERROR_STREAM("CV exception:" << e.what());
        return;
    }

    // tf::Transform tagInBaselink = camInBaselink * tagInCam;
    tf::Transform tagInBaselink = tagInCam;

    // Publish the detected fiducial projected from current base_link.
    mbr.sendTransform(tf::StampedTransform(tagInBaselink, msg->header.stamp, tagFrame, "new_base_link"));
}


bool BotelloArucoNode::estimatePoseClosestMarker(const std::vector<std::vector<cv::Point2f>> & corners,
                                                  float markerLength,
                                                  const cv::Mat &cameraMatrix,
                                                  const cv::Mat &distCoeffs,
                                                  cv::Vec3d& rvec, cv::Vec3d& tvec,
                                                  double& reprojectionError,
                                                  int& closestTagIx) {

    CV_Assert(markerLength > 0);
    int nMarkers = (int)corners.size();
    std::vector<cv::Point3f> markerObjPoints;

    // Find closest marker, if observing more than one. Otherwise, set to the first ix, which is zero.
    closestTagIx = 0;

    if (nMarkers > 1)
    {

        closestTagIx = std::max_element(corners.begin(), corners.end(), [this](
             const std::vector<cv::Point2f>  & lhs, const std::vector<cv::Point2f>  & rhs){
                 return calcFiducialArea(lhs) < calcFiducialArea(rhs);}) - corners.begin();
    }
    else if (nMarkers == 1)
    {
        // Do nothing. Keep closestTagIx as 0. Case here for readability.
        closestTagIx = 0;
    }
    else 
    {
        return false;
    }

    // for each marker, calculate its pose
    double fiducialSize = markerLength;

    // TODO(yoraish): we can have fiducials with unique lengths.
    //    std::map<int, double>::iterator it = fiducialLens.find(ids[i]);
    //    if (it != fiducialLens.end()) {
    //       fiducialSize = it->second;
    //    }

    getSingleMarkerObjectPoints(fiducialSize, markerObjPoints);

    cv::solvePnP(markerObjPoints, corners[closestTagIx], cameraMatrix, distCoeffs,
                rvec, tvec);

    reprojectionError = getReprojectionError(markerObjPoints, corners[closestTagIx],
                            cameraMatrix, distCoeffs,
                            rvec, tvec);
    return true;
}


void BotelloArucoNode::getSingleMarkerObjectPoints(float markerLength, std::vector<cv::Point3f>& objPoints) 
{

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.push_back(cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(cv::Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(cv::Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
    objPoints.push_back(cv::Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}

double BotelloArucoNode::dist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

double BotelloArucoNode::calcFiducialArea(const std::vector<cv::Point2f> &pts)
{
    const cv::Point2f &p0 = pts.at(0);
    const cv::Point2f &p1 = pts.at(1);
    const cv::Point2f &p2 = pts.at(2);
    const cv::Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}


double BotelloArucoNode::standardDeviation(std::vector<double> & v)
{
    double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) { return x - mean; });

    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stddev = std::sqrt(sq_sum / v.size());
    return stddev;
}






void BotelloArucoNode::camInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    // Check if this camera 
    if (mHaveCamInfo) {
        return;
    }
    else{
        if (setCamInfoFromMsg(msg))
        {
            ROS_INFO_STREAM("Successfully set camera info.");
            mHaveCamInfo = true;
        }
    }
}


bool BotelloArucoNode::setCamInfoFromMsg(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    // Populate relevant cameraMatrix and distortionCoeffs objects.
    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                mCameraMatrix.at<double>(i, j) = msg->K[i*3+j];
            }
        }
        // If the images are requested to be cropped, then account for the cropped brim in the principal point value.
        mCameraMatrix.at<double>(0, 2) -= mImageRoiBrimX; 
        mCameraMatrix.at<double>(1, 2) -= mImageRoiBrimY; 

        for (int i=0; i<5; i++) {
            mDistortionCoeffs.at<double>(0,i) = msg->D[i];
        }

        mHaveCamInfo = true;
        mCamImageWidthHeight = std::make_pair(msg->width, msg->height);
        return true;
    }
    else {
        ROS_ERROR_STREAM("CameraInfo message has invalid intrinsics, K matrix all zeros");
        return false;
    }
}


double BotelloArucoNode::getReprojectionError(const std::vector<cv::Point3f> &objectPoints,
                            const std::vector<cv::Point2f> &imagePoints,
                            const cv::Mat &cameraMatrix, const cv::Mat  &distCoeffs,
                            const cv::Vec3d &rvec, const cv::Vec3d &tvec) {

    std::vector<cv::Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (unsigned int i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/(double)objectPoints.size();
    return rerror;
}

} // End namespace botello_aruco.




int main(int argc, char **argv)
{
    // ROS set-ups.
    // Node name.
    ros::init(argc, argv, "botello_aruco_node"); 
    ros::NodeHandle nh;
    botello_aruco::BotelloArucoNode botelloArucoNode(nh);
    


    ros::Rate rate(30);
    while( ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}