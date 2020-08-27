/*
 * @Author: your name
 * @Date: 2020-04-15 10:37:12
 * @LastEditTime: 2020-08-10 17:16:00
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
#include <ros/service_server.h>

#include "autonomus_transport_industrial_system/tagDetected.h"
#include "autonomus_transport_industrial_system/netComGoal.h"
#include "PoseDrawer.h"

namespace AutonomusTransportIndustrialSystem
{
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;

    class NavigationGoal : public PoseDrawer
    {
    private:
        ros::NodeHandle nh_;
        move_base_msgs::MoveBaseGoal goal;

        ros::ServiceServer goalServer;
        ros::ServiceServer tagServer;
        move_base_client ac;

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
        bool goalServiceCallBack(autonomus_transport_industrial_system::netComGoal::Request &req,
                                 autonomus_transport_industrial_system::netComGoal::Response &res);
        /**
         *
         * @param req
         * @param res
         * @return
         */
        bool isTagDetectedCallBack(autonomus_transport_industrial_system::tagDetected::Request &req,
                                   autonomus_transport_industrial_system::tagDetected::Response &res);
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
        void pubNavigationGoal(double p_x, double p_y, double p_z, double o_x, double o_y, double o_z, double o_w);

        /**
         * @description: 接受目标位置并广播（Overload 2）
         * @param stamp 目标位置  
         * @return: none
         */
        void pubNavigationGoal(geometry_msgs::PoseStamped stamp);

        /**
         * @description: 
         * @param {type} 
         * @return {type} 
         */
        void doneMovingCallBack(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    };

    NavigationGoal::NavigationGoal(ros::NodeHandle given_nh) : nh_(given_nh), PoseDrawer(given_nh), ac("move_base", true)
    {
        goalServer = nh_.advertiseService("netComGoal", &NavigationGoal::goalServiceCallBack, this);
        tagServer = nh_.advertiseService("is_tag_detected", &NavigationGoal::isTagDetectedCallBack, this);
        // 等待move_base服务器响应
        while (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Wait for the move base server to come up");
        }
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
        ROS_WARN_STREAM("taget pose:" << goal.target_pose.pose.position.x << "," << goal.target_pose.pose.position.y);

        tf::StampedTransform tolerence_tf = TfListener("map", "odom");
        ROS_INFO("Passing loop.");
        // 若tf对象不为空，则继续运算
        if (tolerence_tf.frame_id_ != "ERROR")
        {
            ROS_INFO("Passing tolerence_tf.");
            // 若odom和map的欧拉距离小于阈值，则继续发布导航指令
            if (hypot(tolerence_tf.getOrigin().getX(), tolerence_tf.getOrigin().getY()) < 5)
            {
                pubNavigationGoal(); //广播goal坐标到move_base中
            }
            else
            {
                ROS_ERROR("/map and /odom frame have a massive error!");
            }
        }
        else
        {
            ROS_ERROR("/map and /odom is not linked.");
        }
        
        res.status = true;
        return true;
    }

    void NavigationGoal::pubNavigationGoal()
    {
        ac.sendGoal(goal, boost::bind(&NavigationGoal::doneMovingCallBack, this, _1, _2), move_base_client::SimpleActiveCallback(), move_base_client::SimpleFeedbackCallback());
        ROS_INFO("Sending goal....");
    }

    void NavigationGoal::pubNavigationGoal(double p_x, double p_y, double p_z,
                                           double o_x, double o_y, double o_z, double o_w)
    {
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = p_x;
        goal.target_pose.pose.position.y = p_y;
        goal.target_pose.pose.position.z = p_z;
        goal.target_pose.pose.orientation.x = o_x;
        goal.target_pose.pose.orientation.y = o_y;
        goal.target_pose.pose.orientation.z = o_z;
        goal.target_pose.pose.orientation.w = o_w;

        ac.sendGoal(goal, boost::bind(&NavigationGoal::doneMovingCallBack, this, _1, _2), move_base_client::SimpleActiveCallback(), move_base_client::SimpleFeedbackCallback());
        ROS_INFO("Sending goal...");
    }

    void NavigationGoal::pubNavigationGoal(geometry_msgs::PoseStamped stamp)
    {
        goal.target_pose = stamp;

        ac.sendGoal(goal, boost::bind(&NavigationGoal::doneMovingCallBack, this, _1, _2), move_base_client::SimpleActiveCallback(), move_base_client::SimpleFeedbackCallback());
        ROS_INFO("Sending goal...");
    }

    void NavigationGoal::doneMovingCallBack(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
    {
        if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_ERROR("Bad News! Failed to arrive the destination.");
        }
        else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Arrived.");
        }
    }

    bool NavigationGoal::isTagDetectedCallBack(autonomus_transport_industrial_system::tagDetected::Request &req,
                                               autonomus_transport_industrial_system::tagDetected::Response &res)
    {
        if (req.is_tag_detected == true)
        {
            ROS_INFO_STREAM("tag detected, cancelling move_base navigation...");
            ac.cancelAllGoals();
        }
        res.status = true;
        return true;
    }
} // namespace AutonomusTransportIndustrialSystem

#endif