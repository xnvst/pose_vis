#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>

#include <sstream>

#include <pose_vis/pose_vis.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );


    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("test_pose", 1000);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("test_path", 1000);

    ros::Rate loop_rate(1);

    nav_msgs::Path path_msg;
    path_msg.header.seq = 0;
    path_msg.header.frame_id = "world";

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        geometry_msgs::PoseStamped pose_msg;

        std_msgs::Header header;
        header.seq = count;
        header.stamp = ros::Time::now();
        header.frame_id = "world";

        pose_msg.header = header;
        pose_msg.pose.position.x = count;
        pose_msg.pose.position.y = count;
        pose_msg.pose.position.z = 0;
        pose_msg.pose.orientation.x = 0;
        pose_msg.pose.orientation.y = 0;
        pose_msg.pose.orientation.z = 0;
        pose_msg.pose.orientation.w = 0;
        path_msg.header.stamp = ros::Time::now();
        path_msg.poses.push_back(pose_msg);

        std::stringstream ss;
        ss << "hello world " << count << ", " << func(count);
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "test"));

        chatter_pub.publish(msg);
        pose_pub.publish(pose_msg);
        path_pub.publish(path_msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}