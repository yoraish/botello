#include "botello_localization_node.hpp"

namespace botello_localization
{


BotelloLocalizationNode::BotelloLocalizationNode(const ros::NodeHandle & nh)
{
    ROS_INFO("Instantiating BotelloLocalizationNode.");
    mnh = nh;

    // Get params from server. TODO(yoraish): add check for success of param retrieval.
    getParamsFromParamServer();

    // Listen to odometry messages.
    mOdomSub = mnh.subscribe("/tello/odom", 1, & BotelloLocalizationNode::odomCb, this);
    // Listen to tello status messages.
    mTelloStatusSub = mnh.subscribe("/tello/status", 1, & BotelloLocalizationNode::telloStatusCb, this);
    
    // Listen to tello status messages.
    mLandmarkSub = mnh.subscribe("/botello/landmark", 1, & BotelloLocalizationNode::landmarkCb, this);

    // Publish odom messages transformed to a right-handed coordinate frame.
    mOdomPub = mnh.advertise<nav_msgs::Odometry>("/botello/odom", 1);

    // Set last odom in map to the zero transform.
    mOdomInMap = tf::StampedTransform();

    ROS_INFO("Init BotelloLocalizationNode done.");
}

BotelloLocalizationNode::~BotelloLocalizationNode()
{
}


bool BotelloLocalizationNode::getParamsFromParamServer()
{
    ros::NodeHandle pnh("botello_localization_node"); // To fetch params from the private namespace.
    if (!pnh.param<double>("max_secs_no_odom", mMaxSecsNoOdom, 3.0))
    {
        ROS_ERROR_STREAM("Using default value for param max_secs_no_odom");
    }
    if (!pnh.param<double>("map_odom_update_interval", mMapOdomUpdateInterval, 1.0))
    {
        ROS_ERROR_STREAM("Using default value for param map_odom_update_interval");
    }
}



void BotelloLocalizationNode::odomCb(const nav_msgs::Odometry & msg)
{
    // Create a tf transform message and publish the tf to the tree (from odom to base_link).
    // Only do that if the tello state is flying (aka is_flying: true, or equivalently: fly_mode: 11)
    if (mTelloStatusIsFlying == true)
    {
        // The transform in the message (aka msg.pose.pose) is flipped. To flip it back:
        // Set x = x.
        // Set y = -y
        // Set yaw = -yaw
        tf::Transform telloBaselinkInTelloOdom;
        tf::poseMsgToTF(msg.pose.pose, telloBaselinkInTelloOdom);
        double origX = msg.pose.pose.position.x;
        double origY = msg.pose.pose.position.y;
        double origYaw = tf::getYaw(telloBaselinkInTelloOdom.getRotation());

        tf::StampedTransform baselinkInOdom;
        baselinkInOdom.setOrigin(tf::Vector3(origX, -origY, mTelloStatusHeight));
        baselinkInOdom.setRotation(tf::createQuaternionFromYaw(-origYaw));

        if (fabs(origX) > mEPSILON && fabs(origY) > mEPSILON)
        {
            mLastBaselinkInOdom = tf::StampedTransform(baselinkInOdom, msg.header.stamp, "odom", "base_link");
            mbr.sendTransform(mLastBaselinkInOdom);
        }

        // Also publish the odom in map transform here, if it is non-zero. TODO(yoraish): move this into its own publish thread.
        if (mOdomInMap.child_frame_id_ != "" && mOdomInMap.frame_id_ != "" )
        {
            mOdomInMap = tf::StampedTransform(mOdomInMap, ros::Time::now(), "map", "odom");
            mbr.sendTransform(mOdomInMap);
        }

    }
}


void BotelloLocalizationNode::telloStatusCb(const tello_driver::TelloStatus & msg)
{
    mTelloStatusIsFlying = msg.is_flying;
    mTelloStatusHeight = msg.height_m;
}

void BotelloLocalizationNode::landmarkCb(const geometry_msgs::TransformStamped & msg)
{
    static tf::TransformListener tfListener;

    // Upon a landmark observation, tweak the parent:map, child:odom, transform to get proper localization.

    // If our odom data is old, raise an error (TODO).
    if ((ros::Time::now() -  mLastBaselinkInOdom.stamp_).toSec() > mMaxSecsNoOdom)
    {
        ROS_ERROR_STREAM("Recent odom data is from " << (ros::Time::now() -  mLastBaselinkInOdom.stamp_).toSec() << " seconds ago.");
    }

    // Get base_link in map by getting landmak in map and projecting the message transform from there.
    std::string landmarkFrame = msg.child_frame_id;
    tf::StampedTransform landmarkInBaselink;
    tf::transformStampedMsgToTF(msg, landmarkInBaselink);
    tf::Transform baselinkInLandmark = landmarkInBaselink.inverse();

    // Get the landmark in the map. TODO(yoraish): if landmarks are static, then keep copies of their transforms (in map) as members.
    tf::StampedTransform landmarkInMap;

    try
    {
        ros::Time now = ros::Time::now();
        tfListener.waitForTransform("map", msg.child_frame_id, now, ros::Duration(0.05));
        tfListener.lookupTransform ("map", msg.child_frame_id, now,  landmarkInMap);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR_STREAM("Could not find landmark in map.");
        return;
    }

    tf::Transform baselinkInMap = landmarkInMap * baselinkInLandmark;
    // Get odom in base_link from a class variable and have a timeout on the time it was created (checked earlier). And compute odomInMap = baselinkInMap * (baselinkInOdom.inverse())
    tf::Transform odomInMap = baselinkInMap * mLastBaselinkInOdom.inverse();

    
    // If enough time has passed since the last odom-map tweak, publish new odom in map transform to the TF tree.
    // if ((ros::Time::now() - mOdomInMap.stamp_).toSec() >= mMapOdomUpdateInterval)
    // {
        mOdomInMap = tf::StampedTransform(odomInMap, msg.header.stamp, "map", "odom");
    // }
}

} // End namespace botello_localization.


int main(int argc, char **argv)
{
    // ROS set-ups.
    // Node name.
    ros::init(argc, argv, "botello_localization_node"); 
    ros::NodeHandle nh;
    botello_localization::BotelloLocalizationNode botelloLocalizationNode(nh);
    
    ros::Rate rate(40);
    while( ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}