#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sstream>
#include "tello_driver/TelloStatus.h"

#pragma once

namespace botello_localization
{

class BotelloLocalizationNode
{
private:
    /* data */
    // ================
    // Member variables.
    // ================
 
    // Node handle.
    ros::NodeHandle mnh;

    // Transform broadcaster.
    tf::TransformBroadcaster mbr;

    // Tello status.
    bool mTelloStatusIsFlying;
    double mTelloStatusHeight;

    // Epsilon.
    double mEPSILON = 0.001;

    // Most recent transform for odom in parent-frame base_link.
    tf::StampedTransform mLastBaselinkInOdom;
    // Most recent map-odom transform.
    tf::StampedTransform mOdomInMap;

    // ================
    // Parameters.
    // ================
    // Maximum number of seconds without odometry updates. If exceeded raise an error that could be handled by landing/holding. TODO(yoraish): create error publishing and handling pipeline.
    double mMaxSecsNoOdom;
    // How often is it allowed to update the map-odom transform?
    double mMapOdomUpdateInterval;

    // ================
    // Methods.
    // ================
    bool getParamsFromParamServer();

    // ================
    // Listeners.
    // ================
    ros::Subscriber mOdomSub;
    ros::Subscriber mTelloStatusSub;
    ros::Subscriber mLandmarkSub;

    // ================
    // Publishers.
    // ================
    ros::Publisher mOdomPub;

    
    // ================
    // Callbacks.
    // ================
    void odomCb(const nav_msgs::Odometry & msg);
    void telloStatusCb(const tello_driver::TelloStatus & msg);

    // Listen to landmark observations in the base_link frame.
    void landmarkCb(const geometry_msgs::TransformStamped & msg);


public:
    BotelloLocalizationNode(const ros::NodeHandle & nh);
    ~BotelloLocalizationNode();
};

} // namespace botello_localization

