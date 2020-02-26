/*
 * @Author: lifuguan
 * @Date: 2019-11-27 16:24:05
 * @LastEditTime: 2020-02-26 14:48:04
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/src/test.cpp
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <message_filters/subscriber.h>

#include "../include/utility.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    AutonomusTransportIndustrialSystem::PoseDrawer pd(nh);

    tf::TransformBroadcaster goal_frame_broadcaster;
    ros::Publisher pose_publisher_a = nh.advertise<geometry_msgs::PoseStamped>("pose_a", 1);

    tf::Transform a_to_b_tf(tf::Quaternion(-1.57, 0, 0.78), tf::Vector3(1, -2, 0));
    geometry_msgs::PoseStamped a_to_m_pose;

    ros::Rate rate(20);
    while (nh.ok())
    {
        // 广播tf : A->B
        goal_frame_broadcaster.sendTransform(tf::StampedTransform(a_to_b_tf, ros::Time::now(), "/A", "/B"));
        a_to_m_pose.header.stamp = ros::Time::now();
        a_to_m_pose.header.frame_id = "A";
        a_to_m_pose.pose.position.x = 1;
        a_to_m_pose.pose.position.y = 1;
        a_to_m_pose.pose.position.z = 1;
        a_to_m_pose.pose.orientation.x = 0;
        a_to_m_pose.pose.orientation.y = 0;
        a_to_m_pose.pose.orientation.z = 0;
        a_to_m_pose.pose.orientation.w = 1;
        pose_publisher_a.publish(a_to_m_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}