#include "botello_navigation_node.hpp"

namespace botello_navigation
{


BotelloNavigationNode::BotelloNavigationNode(const ros::NodeHandle & nh)
{
    ROS_INFO("Instantiating BotelloNavigationNode.");
    mnh = nh;

    // Get params from server.
    getParamsFromParamServer();

    // Get waypoints.
    readWaypointsFromFile();

    // Set some default values for member variables.
    mLastGoalReachTime = ros::Time::now();
    mCurrentWaypointIx = 0;

    ROS_INFO("Init BotelloNavigationNode done.");
}

BotelloNavigationNode::~BotelloNavigationNode()
{
}

bool BotelloNavigationNode::getParamsFromParamServer()
{
    ros::NodeHandle pnh("botello_navigation_node"); // To fetch params from the private namespace. I think that "~" is similar.
    if (!pnh.param<std::string>("waypoints_yaml_path", mWaypointsYamlPath, ""))
    {
        ROS_ERROR_STREAM("Using default value for param waypoints_yaml_path");
    }
}

void BotelloNavigationNode::publishGoal(tf::Transform goalTransform, std::string frame)
{
    mbr.sendTransform(tf::StampedTransform(goalTransform, ros::Time::now(), frame, "goal"));
}

void BotelloNavigationNode::publishGoal(double x, double y, double z, double roll, double pitch, double yaw, std::string frame)
{
    tf::Transform goalTransform;
    goalTransform.setOrigin( tf::Vector3(x, y, z) );
    tf::Quaternion qGoalTransform;
    qGoalTransform.setRPY(roll, pitch,yaw);
    goalTransform.setRotation(qGoalTransform);
    mbr.sendTransform(tf::StampedTransform(goalTransform, ros::Time::now(), frame, "goal"));
}

bool BotelloNavigationNode::readWaypointsFromFile()
{
    mWaypoints.clear();

    // reading the tiles yaml file
    std::string fr = mWaypointsYamlPath;
    if (!boost::filesystem::exists(fr))
    {
        return false;
    }

    std::ifstream fin_robot(fr.c_str());
	const std::string delimiter = "_";
    YAML::Node waypoints_yaml = YAML::Load(fin_robot);
    try
    {
        for (auto it = waypoints_yaml.begin(); it != waypoints_yaml.end(); ++it)
        {
			std::string key = it->first.as<std::string>();
			key = key.erase(0, key.find(delimiter) + delimiter.length());
			Waypoint wp;
			wp.x = it->second[0].as<float>();
			wp.y = it->second[1].as<float>();
			wp.z = it->second[2].as<float>();
			wp.roll = it->second[3].as<float>();
			wp.pitch = it->second[4].as<float>();
			wp.yaw = it->second[5].as<float>();
			wp.id = key;
			wp.frame = it->second[6].as<std::string>();;

            mWaypoints.push_back(wp);    

        }
    }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR_STREAM("The waypoints yaml contains invalid data.");
        return false;
    }

    // Sort the waypoints by their intended order (id).
    std::sort(mWaypoints.begin(), mWaypoints.end(),
            [&](Waypoint A, Waypoint B) -> bool {
                    return A.id < B.id;
                });

    // Print order.
    for (Waypoint wp : mWaypoints)
    {
        ROS_INFO_STREAM("Read waypoint id: " << wp.id << ". x y z r p y [" << wp.x << " " << wp.y << " " << wp.z << " " << wp.roll << " " << wp.pitch << " " << wp.yaw << "], frame: " << wp.frame);
    }
    return true;
}

bool BotelloNavigationNode::isGoalReached()
{

    // Find the error transform. It is goal in the base_link frame.
    static tf::TransformListener tfListener;
    tf::StampedTransform goalInBaselink;
    try
    {
        ros::Time now = ros::Time::now();
        tfListener.waitForTransform("base_link", "goal", now, ros::Duration(0.5));
        tfListener.lookupTransform ("base_link", "goal", now,  goalInBaselink);
    }
    catch (tf::TransformException ex)
    {
        // If could not find a transform to goal, then change nothing about the control velocities. Unless a long time has passed since the most recent goal transform successful lookup, in which case, command a zero velocity on all axes.
        // ROS_INFO_STREAM("[navigation] Could not find goal in baselink. Goal not reached. " << ex.what());
        return false;
    }

    double errX = goalInBaselink.getOrigin().getX();
    double errY = goalInBaselink.getOrigin().getY();
    double errYaw = tf::getYaw(goalInBaselink.getRotation());
    if ((fabs(errX) < 0.15) && 
        (fabs(errY) < 0.15) && 
        (fabs(errYaw) < 0.2) && 
        (ros::Time::now() - mLastGoalReachTime).toSec() > 1.0)// TODO(yoraish): move those to the config file.
    {
        mLastGoalReachTime = ros::Time::now();
        ROS_INFO_STREAM("Goal reached.");
        return true;
    }
    return false;
}

void BotelloNavigationNode::updateCurrentWaypointIx()
{
    // Check if the current waypoint has been reached.
    if (isGoalReached())
    {
        mCurrentWaypointIx += 1;
        mCurrentWaypointIx = mCurrentWaypointIx % mWaypoints.size();
    }
}

void BotelloNavigationNode::publishGoalCurrentWaypoint()
{
    Waypoint currWaypoint = mWaypoints.at(mCurrentWaypointIx);
    publishGoal(currWaypoint.x, currWaypoint.y, currWaypoint.z, currWaypoint.roll, currWaypoint.pitch, currWaypoint.yaw, currWaypoint.frame);
}


void BotelloNavigationNode::spinUpdateWaypoints()
{
    ros::Rate r(30);
    while (ros::ok)
    {
        updateCurrentWaypointIx();
        r.sleep();
        ros::spinOnce();
    }

}

void BotelloNavigationNode::spinPublishWaypoints()
{
    ros::Rate r(30);
    while (ros::ok)
    {
        publishGoalCurrentWaypoint();
        r.sleep();
        ros::spinOnce();
    }

}

void BotelloNavigationNode::spinThreads()
{
    boost::thread* publishWaypointsThread = new boost::thread(boost::bind(& BotelloNavigationNode::spinPublishWaypoints, this));
    boost::thread* updateWaypointsThread = new boost::thread(boost::bind(& BotelloNavigationNode::spinUpdateWaypoints, this));
    // mThreads.push_back(thr);
    while(ros::ok())
    {
    }
}



} // End namespace botello_navigation.


int main(int argc, char **argv)
{
    // ROS set-ups.
    // Node name.
    ros::init(argc, argv, "botello_navigation_node"); 
    ros::NodeHandle nh;
    botello_navigation::BotelloNavigationNode botelloNavigationNode(nh);

    // Splitting the operation here to threads since we'd like the publishing of goals/waypoints to be constant. This could be affected by waitForTransform that is called when updating the state of a waypoint between reached or not.
    botelloNavigationNode.spinThreads();


    return 0;
}