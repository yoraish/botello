
#include <std_msgs/Empty.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sstream>
#include "tello_driver/TelloStatus.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <boost/thread.hpp>
#include <vector>
#include  <cmath>
#pragma once

namespace botello_navigation
{

struct Waypoint
{
    std::string id;
    std::string frame;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class BotelloNavigationNode
{
private:
    // ================
    // Member variables.
    // ================

    // Node handle.
    ros::NodeHandle mnh;

    // Transform broadcaster.
    tf::TransformBroadcaster mbr;

    // Keep note of the requested waypoints.
    std::vector<Waypoint> mWaypoints;

    // Is the current goal reached (and a new one should be sent)?
    ros::Time mLastGoalReachTime;
    
    // Current waypoint index.
    int mCurrentWaypointIx;

    // Threads.
    std::vector<boost::thread*> mThreads;


    // ================
    // Parameters.
    // ================
    // Path to the waypoints yaml.
    std::string mWaypointsYamlPath;
    // The frame in which waypoints are specified.
    std::string mWaypointsFrame;

    // ================
    // Methods.
    // ================
    bool getParamsFromParamServer();
    bool readWaypointsFromFile();

    // Whether the current goal (from the tf tree) is reached.
    bool isGoalReached();
    
    // Check if there is a need to go the the next waypoint. If so, increment the mCurrentWaypointIx object.
    void updateCurrentWaypointIx();

    // Publish current waypoint to the tf tree.
    void publishGoalCurrentWaypoint();



    /**
     * Publish a goal pose to the TF tree. In the specified frame.
     * The given transform is of goal in "frame" frame.
     */
    void publishGoal(tf::Transform goalTransform, std::string frame);
    void publishGoal(double x, double y, double z, double roll, double pitch, double yaw, std::string frame);

    // ================
    // Listeners.
    // ================
    ros::Subscriber mManualOverrideSub;

    // ================
    // Publishers.
    // ================
    // ros::Publisher mCmdVelPub;

    
    // ================
    // Callbacks.
    // ================
    // void manualOverrideCb(const std_msgs::Empty & msg);


public:
    BotelloNavigationNode(const ros::NodeHandle & nh);
    ~BotelloNavigationNode();

    /**
     * Publish a sequence of waypoints. Publishing a waypoint only when the previous one was reached.
     * Waypoints are read from a yaml config file.
     */
    void spinUpdateWaypoints();
    void spinPublishWaypoints();
    void spinThreads();
};

} // namespace botello_navigation
