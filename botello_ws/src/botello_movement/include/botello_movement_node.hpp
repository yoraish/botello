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

#pragma once

namespace botello_movement
{

struct PidGains
{
    double kp;
    double ki;
    double kd;
};

class BotelloMovementNode
{
private:
    // ================
    // Member variables.
    // ================
    // Most recent manual override time.
    ros::Time mManualOverrideStamp;

    // Most recent successful lookup of `goal` frame.
    ros::Time mLastGoalLookupTime;

    // Node handle.
    ros::NodeHandle mnh;

    // Transform broadcaster.
    tf::TransformBroadcaster mbr;

    // PID gains (once object per axis).
    PidGains mXGains;
    PidGains mYGains;
    PidGains mZGains;
    PidGains mYawGains;


    // ================
    // Parameters.
    // ================
    // How long to disable autonomous command after a manual override msg.
    double mManualOverrideCooldownTimeout;

    // Number of seconds allowed to pass without successful goal-frame lookups before a stop signal gets sent.
    double mNoGoalsMaxTime;

    // ================
    // Methods.
    // ================
    bool getParamsFromParamServer();

    // Send /cmd_vel msgs to command velocities on the robot.
    void commandVelocity(const double & vx,
                         const double & vy,
                         const double & vz,
                         const double & vt);


    // Get a velocity command from an axis, gains, and error.
    double pid(const std::string & axis,const double & error, const PidGains & gains);

    // ================
    // Listeners.
    // ================
    ros::Subscriber mManualOverrideSub;

    // ================
    // Publishers.
    // ================
    ros::Publisher mCmdVelPub;

    
    // ================
    // Callbacks.
    // ================
    void manualOverrideCb(const std_msgs::Empty & msg);


public:
    BotelloMovementNode(const ros::NodeHandle & nh);
    ~BotelloMovementNode();

    // Get the current error between base_link and goal, and translate that to velocity commands via a PID controller.
    void controlVelocity();
};

} // namespace botello_movement
