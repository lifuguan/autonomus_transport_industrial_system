/*
 * @Author: your name
 * @Date: 2020-04-15 10:37:12
 * @LastEditTime: 2020-05-14 01:10:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/include/NavigationGoal.h
 */

#ifndef NAVIGATIONGOAL_H
#define NAVIGATIONGOAL_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include "autonomus_transport_industrial_system/netComGoal.h"

namespace AutonomusTransportIndustrialSystem
{
    class NavigationGoal
    {
    private:
        ros::NodeHandle nh_;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
        move_base_msgs::MoveBaseGoal goal;

        ros::ServiceServer goalServer;
    public:
        NavigationGoal(ros::NodeHandle given_nh);
        ~NavigationGoal() = default;
        /**
         * @description: 回调函数，接收从netWorkCom处得到的goal信息
         * @param req 接收的信息
         * @param res 要返回的信息
         * @return: none
         */
        bool goalCallBack(autonomus_transport_industrial_system::netComGoal::Request &req, autonomus_transport_industrial_system::netComGoal::Response &res);

        /**
         * @description: 接受目标位置并广播（Overload 1）
         * @param pose.postion 目标位置
         * @return: none
         */
        void pubNavigationGoal(double p_x,double p_y,double p_z, double o_x, double o_y, double o_z, double o_w);

        /**
         * @description: 接受目标位置并广播（Overload 2）
         * @param pose 目标位置  
         * @return: none
         */
        void pubNavigationGoal(geometry_msgs::PoseStamped stamp);
    };
    
    NavigationGoal::NavigationGoal(ros::NodeHandle given_nh):nh_(given_nh)
    {
        goalServer = nh_.advertiseService("netComGoal", &NavigationGoal::goalCallBack, this);
        ros::spin();
    }

    bool NavigationGoal::goalCallBack(autonomus_transport_industrial_system::netComGoal::Request &req, autonomus_transport_industrial_system::netComGoal::Response &res)
    {
        pubNavigationGoal(req.g_x, req.g_y, 0, 0, 0, 0, 0); //广播goal坐标到move_base中
        return true;
    }


    void NavigationGoal::pubNavigationGoal(double p_x,double p_y,double p_z, 
    double o_x, double o_y, double o_z, double o_w)
    {
        move_base_client ac("move_base", true);
        // 等待move_base服务器响应
        while (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Wait for the move base server to come up");
        }
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = p_x;
        goal.target_pose.pose.position.y = p_y; 
        goal.target_pose.pose.position.z = p_z;
        goal.target_pose.pose.orientation.x = o_x;
        goal.target_pose.pose.orientation.y = o_y;
        goal.target_pose.pose.orientation.z = o_z;
        goal.target_pose.pose.orientation.w = o_w;

        ROS_INFO("Sending goal.");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");
    }

    void NavigationGoal::pubNavigationGoal(geometry_msgs::PoseStamped stamp)
    {
         move_base_client ac("move_base", true);
        // 等待move_base服务器响应
        while (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Wait for the move base server to come up");
        }
        goal.target_pose = stamp;
        
        ROS_INFO("Sending goal.");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
    
}



#endif