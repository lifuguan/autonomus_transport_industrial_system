/*
 * @Author: your name
 * @Date: 2020-04-10 08:57:47
 * @LastEditTime: 2020-04-17 00:47:06
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/src/test.cpp
 */
/*
 * @Author: lifuguan
 * @Date: 2019-11-27 16:24:05
 * @LastEditTime: 2020-04-12 22:50:38
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
#include "../include/PoseDrawer.h"
#include "../include/PointCloud.h"
#include "../include/NavigationGoal.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    AutonomusTransportIndustrialSystem::NavigationGoal ng(nh);
    AutonomusTransportIndustrialSystem::PoseDrawer pd(nh);

    geometry_msgs::PoseStamped goal_map, goal_base_link;
    goal_map.header.frame_id = "/map";
    goal_map.header.stamp = ros::Time::now();
    goal_map.pose.position.x = 6;
    goal_map.pose.position.y = -1;
    goal_map.pose.position.z = 0;
    goal_map.pose.orientation.x = 0;
    goal_map.pose.orientation.y = 0;
    goal_map.pose.orientation.z = 0;
    goal_map.pose.orientation.w = 1;


    ros::Publisher pose_publisher_a = nh.advertise<geometry_msgs::PoseStamped>("goal_map", 1);
    pd.PoseListener("goal_map", "base_link");

    ros::Rate rate(20);
    while (nh.ok())
    {
        pose_publisher_a.publish(goal_map);
        // 败笔
        ng.pubNavigationGoal(pd.pose_out);
        ros::spinOnce();
    }
    
    


    
    // AutonomusTransportIndustrialSystem::PoseDrawer pd(nh);
    // pd.PoseListener("pose_a", "B");

    // tf::TransformBroadcaster goal_frame_broadcaster;
    // ros::Publisher pose_publisher_a = nh.advertise<geometry_msgs::PoseStamped>("pose_a", 1);

    // tf::Transform a_to_b_tf(tf::Quaternion(-1.57, 0, 0.78), tf::Vector3(1, -2, 0));
    // geometry_msgs::PoseStamped a_to_m_pose;

    // ros::Rate rate(20);
    // while (nh.ok())
    // {
    //     // 广播tf : A->B
    //     goal_frame_broadcaster.sendTransform(tf::StampedTransform(a_to_b_tf, ros::Time::now(), "/A", "/B"));
    //     a_to_m_pose.header.stamp = ros::Time::now();
    //     a_to_m_pose.header.frame_id = "A";
    //     a_to_m_pose.pose.position.x = 1;
    //     a_to_m_pose.pose.position.y = 1;
    //     a_to_m_pose.pose.position.z = 1;
    //     a_to_m_pose.pose.orientation.x = 0;
    //     a_to_m_pose.pose.orientation.y = 0;
    //     a_to_m_pose.pose.orientation.z = 0;
    //     a_to_m_pose.pose.orientation.w = 1;
    //     pose_publisher_a.publish(a_to_m_pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    
    return 0;
}
