/*
 * @Author: your name
 * @Date: 2020-04-15 10:37:12
 * @LastEditTime: 2020-04-15 13:07:38
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/include/NavigationGoal.h
 */

#ifndef NAVIGATIONGOAL_H
#define NAVIGATIONGOAL_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace AutonomusTransportIndustrialSystem
{
    class NavigationGoal
    {
    private:
        ros::NodeHandle nh_;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
        move_base_msgs::MoveBaseGoal goal;
    public:
        NavigationGoal(ros::NodeHandle given_nh);
        ~NavigationGoal() = default;

        /**
         * @description: 接受目标位置并广播
         * @param pose 目标位置 
         * @return: none
         */
        void pubNavigationGoal(double p_x,double p_y,double p_z, double o_x, double o_y, double o_z, double o_w);
    };
    
    NavigationGoal::NavigationGoal(ros::NodeHandle given_nh):nh_(given_nh)
    {
        
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
    
}



#endif