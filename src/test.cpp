/*
 * @Author: your name
 * @Date: 2020-04-10 08:57:47
 * @LastEditTime: 2020-04-17 21:15:44
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
#include <visualization_msgs/MarkerArray.h>


#include "../include/utility.h"
#include "../include/PoseDrawer.h"
#include "../include/PointCloud.h"
#include "../include/NavigationGoal.h"

/**
 * @description: 生成货物提取点
 * @param extraction_array 
 * @return: extraction_array
 */
void extractionGenerator(visualization_msgs::MarkerArray &extraction_array);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    ros::Publisher extraction_pub = nh.advertise<visualization_msgs::MarkerArray>("extraction_array", 10);
    // 货柜地点
    visualization_msgs::MarkerArray extraction_array;
    extraction_array.markers.resize(4);
    extractionGenerator(extraction_array);

    ros::Rate rate(20);
    while (nh.ok())
    {
        extraction_pub.publish(extraction_array);
        ros::spinOnce();
    }
    


    // AutonomusTransportIndustrialSystem::NavigationGoal ng(nh);
    // AutonomusTransportIndustrialSystem::PoseDrawer pd(nh);

    // geometry_msgs::PoseStamped goal_odom, goal_base_link;
    // goal_odom.header.frame_id = "/odom";
    // goal_odom.header.stamp = ros::Time(0);
    // goal_odom.pose.position.x = 6;
    // goal_odom.pose.position.y = -1;
    // goal_odom.pose.position.z = 0;
    // goal_odom.pose.orientation.x = 0;
    // goal_odom.pose.orientation.y = 0;
    // goal_odom.pose.orientation.z = 0;
    // goal_odom.pose.orientation.w = 1;


    // ros::Publisher pose_publisher_a = nh.advertise<geometry_msgs::PoseStamped>("goal_odom", 1);
    // pd.PoseListener("goal_odom", "base_link");

    // pose_publisher_a.publish(goal_odom);

    // // 败笔
    // ng.pubNavigationGoal(pd.pose_out);



    
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


void extractionGenerator(visualization_msgs::MarkerArray &extraction_array)
{
    // 生成相同部分
    for (int i = 0; i < 4; i++)
    {
        extraction_array.markers[i].header.frame_id = "map";
        extraction_array.markers[i].header.stamp = ros::Time::now();
        extraction_array.markers[i].ns = "red";
        extraction_array.markers[i].id = i;
        extraction_array.markers[i].action = visualization_msgs::Marker::ADD;
        extraction_array.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        extraction_array.markers[i].text = std::to_string(i);
        extraction_array.markers[i].scale.z = 0.4;
        extraction_array.markers[i].color.a = 1.0;
        extraction_array.markers[i].color.r = 1.0;
        extraction_array.markers[i].color.g = 0.0;
        extraction_array.markers[i].color.b = 0.0;
    }

    // 左边红色圆柱体
    extraction_array.markers[0].pose.position.x = 3.15;
    extraction_array.markers[0].pose.position.y = 3.35;
    extraction_array.markers[0].pose.position.z = 0.0;
    extraction_array.markers[0].pose.orientation.x = 0.0;
    extraction_array.markers[0].pose.orientation.y = 0.0;
    extraction_array.markers[0].pose.orientation.z = 0.0;
    extraction_array.markers[0].pose.orientation.w = 1.0;
    // 中间绿色圆柱体
    extraction_array.markers[1].pose.position.x = 6.21;
    extraction_array.markers[1].pose.position.y = 2.60;
    extraction_array.markers[1].pose.position.z = 0.0;
    extraction_array.markers[1].pose.orientation.x = 0.0;
    extraction_array.markers[1].pose.orientation.y = 0.0;
    extraction_array.markers[1].pose.orientation.z = 0.0;
    extraction_array.markers[1].pose.orientation.w = 1.0;
    // 右边蓝色圆柱体
    extraction_array.markers[2].pose.position.x = 7.65;
    extraction_array.markers[2].pose.position.y = -0.47;
    extraction_array.markers[2].pose.position.z = 0.1;
    extraction_array.markers[2].pose.orientation.x = 0.0;
    extraction_array.markers[2].pose.orientation.y = 0.0;
    extraction_array.markers[2].pose.orientation.z = 0.0;
    extraction_array.markers[2].pose.orientation.w = 1.0;
    
}