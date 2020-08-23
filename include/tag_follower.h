/*
 * @Author: your name
 * @Date: 2020-08-17 19:17:42
 * @LastEditTime: 2020-08-17 19:37:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/include/tag_follower.h
 */

#ifndef TAG_FOLLOWER_
#define TAG_FOLLOWER_

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "autonomus_transport_industrial_system/netComTag.h"
namespace AutonomusTransportIndustrialSystem
{
    class TagFollower
    {
    private:
        ros::NodeHandle nh;
        tf::StampedTransform tag_transform;

        tf::TransformListener tag_listener;

        const int UN_INITIALIZE = -1;
        const int IS_DONE = -2;
        // -1: 未初始化；0~2：目标tag
        int tag_num = UN_INITIALIZE;

        /// tag 的ros service回调函数
        /// \param req
        /// \param res
        /// \return
        bool tagServiceCallBack(autonomus_transport_industrial_system::netComTag::Request & req, autonomus_transport_industrial_system::netComTag::Response& res);
    public:
        TagFollower(ros::NodeHandle given_nh) : nh(given_nh)
        {
            // 此处采取Service向netComModule订阅目标信息，修改tf监听值
            ros::ServiceServer tag_service = nh.advertiseService("netComTag", &TagFollower::tagServiceCallBack, this);
        }
        ~TagFollower() = default;

        void tagEventProcess(const ros::TimerEvent& event);
    };

    void TagFollower::tagEventProcess(const ros::TimerEvent &event)
    {
        // 读取tag的tf
        try
        {
            if (tag_num != UN_INITIALIZE && tag_num != IS_DONE)
            {
                tag_listener.lookupTransform("/base_link", "/tag_"+std::to_string(tag_num), ros::Time(0), tag_transform);
                ROS_INFO_STREAM("Actual position: x = " << tag_transform.getOrigin().z() << ", y = " << tag_transform.getOrigin().x());
            }
            else
            {
                ROS_INFO_STREAM("tag num is not initialize OR the mission is finished.");
            }
        } catch (tf::TransformException &ex)
        {
            ROS_WARN_STREAM("tf lookup is not exist : " << ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    bool TagFollower::tagServiceCallBack(autonomus_transport_industrial_system::netComTag::Request &req,
                                         autonomus_transport_industrial_system::netComTag::Response &res)
    {
        tag_num = req.tag_num;
        res.status = true;
        return true;
    }
}


#endif