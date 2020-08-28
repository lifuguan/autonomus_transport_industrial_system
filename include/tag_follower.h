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
#include <ros/service_client.h>

#include "autonomus_transport_industrial_system/tagDetected.h"
#include "autonomus_transport_industrial_system/netComTag.h"
#include "utility.h"

namespace AutonomusTransportIndustrialSystem
{
    class TagFollower
    {
    private:
        ros::NodeHandle nh;
        tf::StampedTransform tag_transform;
        tf::TransformListener tag_listener;

        // 如果发现tag，发送service给navigationGoal
        ros::ServiceClient abort_move_base;

        ros::ServiceServer tag_service;
        // 检测到为true， 没有检测到为false
        const bool DETECTED = true;
        const bool UNDETECTED = false;

        autonomus_transport_industrial_system::tagDetected is_tag_detected;
        autonomus_transport_industrial_system::tagDetected is_tag_detected_last;

        const int UN_INITIALIZE = -1;
        const int IS_DONE = -2;
        // -1: 未初始化；0~2：目标tag
        int tag_num = UN_INITIALIZE;

        bool check_transition_status;

        /**
         * tag 的ros service回调函数：由netWorkCom发送
         * @param req
         * @param res
         * @return
         */
        bool tagServiceCallBack(autonomus_transport_industrial_system::netComTag::Request & req, autonomus_transport_industrial_system::netComTag::Response& res);

        /**
         * 每次监听到新的导航点都重置一次tag的service
         */
        void initialize();

        /**
         * 检查tag检测的跳变状态
         * @param is_tag_detected 本次检测结果
         * @param is_tag_detected_last 上次检测结果
         * @return
         */
        bool checkTransition(autonomus_transport_industrial_system::tagDetected is_tag_detected,
                             autonomus_transport_industrial_system::tagDetected is_tag_detected_last);

    public:
        TagFollower(ros::NodeHandle given_nh) : nh(given_nh)
        {
            // 此处采取Service向netComModule订阅目标信息，修改tf监听值
            tag_service = nh.advertiseService("netComTag", &TagFollower::tagServiceCallBack, this);
            // 此处采取Service向navigationGoal广播tag是否被识别
            abort_move_base = nh.serviceClient<autonomus_transport_industrial_system::tagDetected>("is_tag_detected");
            initialize();
            check_transition_status = false;
        }
        ~TagFollower() = default;

        void tagMakePlan(const ros::TimerEvent& event);
    };

    void TagFollower::tagMakePlan(const ros::TimerEvent &event)
    {
        if (tag_num == UN_INITIALIZE)
        {
            ROS_INFO_STREAM("tag num is not initialize.");
            return;
        }
        else if (tag_num == IS_DONE)
        {
            ROS_INFO_STREAM("The mission is finished.");
            return;
        }
        else
        {
            // 读取tag的tf
            try
            {
                tag_listener.lookupTransform("/base_link", "/tag_"+std::to_string(tag_num), ros::Time(0), tag_transform);
                ROS_INFO_STREAM("Actual position: x = " << tag_transform.getOrigin().z() << ", y = " << tag_transform.getOrigin().x());

                is_tag_detected.request.is_tag_detected = DETECTED;
                // 如果发现tag且状态发生跳变，则发送service以中断move_base导航
                if (checkTransition(is_tag_detected, is_tag_detected_last) && GetEuclideanDistance(tag_transform) <= 2.0)
                {
                    abort_move_base.call(is_tag_detected);
                }
                is_tag_detected_last.request.is_tag_detected = is_tag_detected.request.is_tag_detected;
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN_STREAM("tf lookup is not exist : " << ex.what());
                is_tag_detected.request.is_tag_detected = UNDETECTED;
                return;
            }
        }

        if (GetEuclideanDistance(tag_transform) <= 0.2)
        {
            tag_num = IS_DONE;
        }

    }

    bool TagFollower::tagServiceCallBack(autonomus_transport_industrial_system::netComTag::Request &req,
                                         autonomus_transport_industrial_system::netComTag::Response &res)
    {
        tag_num = req.tag_num;
        ROS_INFO_STREAM("TagCB: tag " << tag_num << " detected.");
        initialize();
        res.status = true;
        return true;
    }

    void TagFollower::initialize()
    {

        is_tag_detected.request.is_tag_detected = UNDETECTED;
        is_tag_detected_last.request.is_tag_detected = UNDETECTED;
        // 初始化service
        abort_move_base.call(is_tag_detected);
    }

    bool TagFollower::checkTransition(autonomus_transport_industrial_system::tagDetected is_tag_detected,
                                      autonomus_transport_industrial_system::tagDetected is_tag_detected_last)
    {
        if (is_tag_detected.request.is_tag_detected == true && is_tag_detected_last.request.is_tag_detected == false)
        {
            check_transition_status = true;
            return check_transition_status;
        }
        else if (is_tag_detected.request.is_tag_detected == false && is_tag_detected_last.request.is_tag_detected == true)
        {
            check_transition_status = false;
            return check_transition_status;
        }
        else
        {
            return check_transition_status;
        }
    }
}

#endif