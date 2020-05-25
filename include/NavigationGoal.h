/*
 * @Author: your name
 * @Date: 2020-04-15 10:37:12
 * @LastEditTime: 2020-05-25 22:29:44
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
    class NavigationGoal:public PoseDrawer
    {
    private:
        ros::NodeHandle nh_;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
        move_base_msgs::MoveBaseGoal goal;

        ros::ServiceServer goalServer;
    public:
        geometry_msgs::PoseStamped target_pose;

        NavigationGoal(ros::NodeHandle given_nh);
        ~NavigationGoal() = default;
        /**
         * @description: 回调函数，接收从netWorkCom处得到的goal信息
         * @param req 接收的信息
         * @param res 要返回的信息
         * @return: none
         */
        bool goalServiceCallBack(autonomus_transport_industrial_system::netComGoal::Request &req, autonomus_transport_industrial_system::netComGoal::Response &res);
        
        /**
         * @description: 接受回调函数得到的位置并广播（Overload 1）
         * @param none
         * @return: 
         */
        void pubNavigationGoal();
        /**
         * @description: 接受目标位置并广播（Overload 1）
         * @param pose postion 目标位置
         * @return: none
         */
        void pubNavigationGoal(double p_x,double p_y,double p_z, double o_x, double o_y, double o_z, double o_w);

        /**
         * @description: 接受目标位置并广播（Overload 2）
         * @param stamp 目标位置  
         * @return: none
         */
        void pubNavigationGoal(geometry_msgs::PoseStamped stamp);
    };
    
    NavigationGoal::NavigationGoal(ros::NodeHandle given_nh):nh_(given_nh),PoseDrawer(given_nh) 
    {
        goalServer = nh_.advertiseService("netComGoal", &NavigationGoal::goalServiceCallBack, this);
    }

    bool NavigationGoal::goalServiceCallBack(autonomus_transport_industrial_system::netComGoal::Request &req, autonomus_transport_industrial_system::netComGoal::Response &res)
    {
        ROS_INFO("req.x = %f, req.y = %f", req.g_x, req.g_y);
        target_pose.header.frame_id = "map";
        target_pose.header.stamp = ros::Time::now();
        target_pose.pose.position.x = req.g_x;
        target_pose.pose.position.y = req.g_y; 
        target_pose.pose.position.z = 0;
        target_pose.pose.orientation.x = 0;
        target_pose.pose.orientation.y = 0;
        target_pose.pose.orientation.z = 0;
        target_pose.pose.orientation.w = 1;

        goal.target_pose = TransformPose("base_link", target_pose);
        ROS_ERROR("taget pose:%d,%d",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

        tf::StampedTransform tolerence_tf =TfListener("map", "odom");
        ROS_INFO("Passing loop.");
        // 若tf对象不为空，则继续运算
        if (tolerence_tf.frame_id_ != "ERROR")
        {         
            ROS_INFO("Passing tolerence_tf.");
            // 若odom和map的欧拉距离小于阈值，则继续发布导航指令
            if (hypot(tolerence_tf.getOrigin().getX(), tolerence_tf.getOrigin().getY()) < 5)
            {
                ROS_INFO("Passing hypot.");
    
                pubNavigationGoal(); //广播goal坐标到move_base中
            }   
            else
            {
                ROS_ERROR("map and odom frame have a massive error!");
            }
        }


        res.status = true;
        return true;
    }

    void NavigationGoal::pubNavigationGoal()
    {
        move_base_client ac("move_base", true);
        // 等待move_base服务器响应
        while (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Wait for the move base server to come up");
        }
        
        ROS_INFO("Sending goal...");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");
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