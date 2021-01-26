#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

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
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("test_marker", 1000);

    ros::Rate loop_rate(1);

    nav_msgs::Path path_msg;
    path_msg.header.seq = 0;
    path_msg.header.frame_id = "world";

    PoseVis posevis;
    posevis.CreatePoseFromData("./ppp_demo_bpq.dat");
    std::vector<Pose> pose_original_vec = posevis.get_fc_pos_vec();

    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();
    header.frame_id = "world";

    geometry_msgs::PoseStamped pose_msg;
    path_msg.header.stamp = ros::Time::now();
    for (int i=0; i<pose_original_vec.size(); ++i)
    {
        pose_msg.header = header;
        pose_msg.pose.position.x = pose_original_vec[i].latitude;
        pose_msg.pose.position.y = pose_original_vec[i].longitude;
        pose_msg.pose.position.z = pose_original_vec[i].altitue;
        pose_msg.pose.orientation.x = pose_original_vec[i].qx;
        pose_msg.pose.orientation.y = pose_original_vec[i].qy;
        pose_msg.pose.orientation.z = pose_original_vec[i].qz;
        pose_msg.pose.orientation.w = pose_original_vec[i].qw;
        path_msg.poses.push_back(pose_msg);
    }

    visualization_msgs::Marker points;
    points.header = header;
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = points.scale.y = points.scale.z = 0.5;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.color.r = 1.0f;
    points.color.a = 1.0;
    for (int i=0; i<100; i++)
    {
        geometry_msgs::Point p;
        p.x = (int32_t)i;
        p.y = (int32_t)i;
        p.z = (int32_t)i;
        points.points.push_back(p);
    }

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
#if 0
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
#endif
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "test"));

        chatter_pub.publish(msg);
        //pose_pub.publish(pose_msg);
        path_pub.publish(path_msg);

        marker_pub.publish(points);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}