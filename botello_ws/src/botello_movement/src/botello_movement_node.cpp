#include "botello_movement_node.hpp"

namespace botello_movement
{


BotelloMovementNode::BotelloMovementNode(const ros::NodeHandle & nh)
{
    ROS_INFO("Instantiating BotelloMovementNode.");
    mnh = nh;

    // Listen to manual override messages.
    mManualOverrideSub = mnh.subscribe("/botello/manual_override", 1, & BotelloMovementNode::manualOverrideCb, this);

    // Publish command velocities.
    mCmdVelPub = mnh.advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1);

    // Get params from server.
    getParamsFromParamServer();

    // Set the recent manual override to be at the inception.
    mManualOverrideStamp = ros::Time(0);

    // Set the recent successful goal lookup.
    mLastGoalLookupTime = ros::Time::now();

    ROS_INFO("Init BotelloMovementNode done.");
}

BotelloMovementNode::~BotelloMovementNode()
{
}

bool BotelloMovementNode::getParamsFromParamServer()
{
    ros::NodeHandle pnh("botello_movement_node"); // To fetch params from the private namespace.
    if (!pnh.param<double>("manual_override_cooldown_timeout", mManualOverrideCooldownTimeout, 4.0))
    {
        ROS_ERROR_STREAM("Using default value for param manual_override_cooldown_timeout");
    }
    if (!pnh.param<double>("no_goals_max_time", mNoGoalsMaxTime, 4.0))
    {
        ROS_ERROR_STREAM("Using default value for param no_goals_max_time");
    }
    // X axis PID gains.
    if (!pnh.param<double>("pid_gains/x/kp", mXGains.kp, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/x/kp");
    }
    if (!pnh.param<double>("pid_gains/x/ki", mXGains.ki, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/x/ki");
    }
    if (!pnh.param<double>("pid_gains/x/kd", mXGains.kd, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/x/kd");
    }

    // Y axis PID gains.
    if (!pnh.param<double>("pid_gains/y/kp", mYGains.kp, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/y/kp");
    }
    if (!pnh.param<double>("pid_gains/y/ki", mYGains.ki, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/y/ki");
    }
    if (!pnh.param<double>("pid_gains/y/kd", mYGains.kd, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/y/kd");
    }

    // Z axis PID gains.
    if (!pnh.param<double>("pid_gains/z/kp", mZGains.kp, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/z/kp");
    }
    if (!pnh.param<double>("pid_gains/z/ki", mZGains.ki, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/z/ki");
    }
    if (!pnh.param<double>("pid_gains/z/kd", mZGains.kd, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/z/kd");
    }

    // Yaw axis PID gains.
    if (!pnh.param<double>("pid_gains/yaw/kp", mYawGains.kp, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/yaw/kp");
    }
    if (!pnh.param<double>("pid_gains/yaw/ki", mYawGains.ki, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/yaw/ki");
    }
    if (!pnh.param<double>("pid_gains/yaw/kd", mYawGains.kd, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param pid_gains/yaw/kd");
    }
}


// Listen to the /botello/manual_override topic. If there are messages there, then block any autonomous commands for the next x seconds.
void BotelloMovementNode::manualOverrideCb(const std_msgs::Empty & msg)
{
    mManualOverrideStamp = ros::Time::now();
}

// Send /cmd_vel msgs to command velocities on the robot.
void BotelloMovementNode::commandVelocity(const double & vx,
                                          const double & vy,
                                          const double & vz,
                                          const double & vt)
{
    // Only allow autonomous commands if enough time has passed since a manual override.
    if ((ros::Time::now() - mManualOverrideStamp).toSec() <= mManualOverrideCooldownTimeout)
    {
        ROS_INFO_STREAM("Manual override cooldown. No autonomous cmd_vel allowed.");
        return;
    }
    geometry_msgs::Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;
    msg.linear.z = vz;
    msg.angular.z = vt;

    mCmdVelPub.publish(msg);
    }

void BotelloMovementNode::controlVelocity()
{

    // Find the error transform. It is goal in the base_link frame.
    static tf::TransformListener tfListener;
    tf::StampedTransform goalInBaselink;
    try
    {
        ros::Time now = ros::Time::now();
        tfListener.waitForTransform("base_link", "goal", now, ros::Duration(0.05));
        tfListener.lookupTransform ("base_link", "goal", now,  goalInBaselink);
    }
    catch (tf::TransformException ex)
    {
        // If could not find a transform to goal, then change nothing about the control velocities. Unless a long time has passed since the most recent goal transform successful lookup, in which case, command a zero velocity on all axes.
        if ((ros::Time::now() - mLastGoalLookupTime).toSec() > mNoGoalsMaxTime)
        {
            ROS_INFO_STREAM("Could not find goal transform for " << (ros::Time::now() - mLastGoalLookupTime).toSec() << " seconds. Exception:" << ex.what() << "\n");
            commandVelocity(0,0,0,0);
        }
        return;
    }

    // Note that we have found a transform to goal.
    mLastGoalLookupTime = ros::Time::now();

    // Compute velocities based on the error.
    double errX = goalInBaselink.getOrigin().getX();
    double errY = goalInBaselink.getOrigin().getY();
    double errZ = 0; // Do not control height for now.
    double errYaw = tf::getYaw(goalInBaselink.getRotation());
    double cmdVelX = pid("x", errX, mXGains);
    double cmdVelY = pid("y", errY, mYGains);
    double cmdVelZ = pid("z", errZ, mZGains);
    double cmdVelYaw = pid("yaw", errYaw, mYawGains);
    
    ROS_INFO_STREAM("Errors ot goal x y yaw: " << errX << " " << errY << " " << errYaw << "\n");

    // Tello commands are not right-handed.
    commandVelocity(-cmdVelY, cmdVelX , 0, -cmdVelYaw);
}

double BotelloMovementNode::pid(const std::string & axis,const double & error, const PidGains & gains)
{
    // TODO(yoraish): complete the PId.
    double cmd = gains.kp * error;
    cmd = cmd > 0.5 ? 0.5 : cmd;
    cmd = cmd < -0.5 ? -0.5 : cmd;
    return cmd;
}
} // End namespace botello_movement.


int main(int argc, char **argv)
{
    // ROS set-ups.
    // Node name.
    ros::init(argc, argv, "botello_movement_node"); 
    ros::NodeHandle nh;
    botello_movement::BotelloMovementNode botelloMovementNode(nh);
    
    ros::Rate rate(30);
    while( ros::ok())
    {
        botelloMovementNode.controlVelocity();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}