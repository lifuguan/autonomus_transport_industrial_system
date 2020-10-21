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
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include <ros/service_client.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>

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
        enum DetectStatus
        {
            DETECTED = true,
            UNDETECTED = false
        };

        enum DetectLogic
        {
            ALWAYS_NO_DETECTED,
            TRANSITION,
            ALWAYS_DETECTED
        };
        autonomus_transport_industrial_system::tagDetected is_tag_detected;
        autonomus_transport_industrial_system::tagDetected is_tag_detected_last;

        const int UN_INITIALIZE = -1;
        const int IS_DONE = -2;
        // -1: 未初始化；0~2：目标tag
        int tag_num = UN_INITIALIZE;

        DetectLogic check_transition_status;

        tf::TransformBroadcaster broadcaster;

        bool is_cancel_move_base = false;
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
        int checkTransition(autonomus_transport_industrial_system::tagDetected is_tag_detected,
                             autonomus_transport_industrial_system::tagDetected is_tag_detected_last);



        void genTraj(const cv::Point2d& p_0, double w_0, const cv::Point2d& p_1, double w_1);

    public:
        TagFollower(ros::NodeHandle given_nh) : nh(given_nh)
        {
            // 此处采取Service向netComModule订阅目标信息，修改tf监听值
            tag_service = nh.advertiseService("netComTag", &TagFollower::tagServiceCallBack, this);
            // 此处采取Service向navigationGoal广播tag是否被识别
            abort_move_base = nh.serviceClient<autonomus_transport_industrial_system::tagDetected>("is_tag_detected");
            initialize();
            check_transition_status = ALWAYS_NO_DETECTED;
        }
        ~TagFollower() = default;

        /**
         * 最主要的函数：协调tag_follow的整个流程
         * @param event 回调函数
         */
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
                tag_listener.lookupTransform("/camera_front", "/tag_"+std::to_string(tag_num), ros::Time(0), tag_transform);
                double roll, pitch, yaw;
                tf::Matrix3x3(tag_transform.getRotation()).getEulerYPR(yaw, pitch, roll);
                ROS_INFO_STREAM("Actual omega = " << roll << ", angle = " <<  180 * roll /  M_PI);

                is_tag_detected.request.is_tag_detected = DETECTED;
                // 如果发现tag且状态发生跳变，则发送service以中断move_base导航，同时生成轨迹
                if (checkTransition(is_tag_detected, is_tag_detected_last) != ALWAYS_NO_DETECTED && GetEuclideanDistance(tag_transform) <= 2.0)
                {
                    if (is_cancel_move_base == true)
                    {
                        abort_move_base.call(is_tag_detected);
                        is_cancel_move_base = false;
                    }

                    // 不断重新生成轨迹
                    genTraj(cv::Point2d(0,0), 0,
                            cv::Point2d(tag_transform.getRotation().x(), tag_transform.getRotation().y()), roll);
                }
                is_tag_detected_last.request.is_tag_detected = is_tag_detected.request.is_tag_detected;


                if (GetEuclideanDistance(tag_transform) <= 0.2)
                {
                    tag_num = IS_DONE;
                }
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN_STREAM("tf lookup is not exist : " << ex.what());
                is_tag_detected.request.is_tag_detected = UNDETECTED;
                return;
            }
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

        is_cancel_move_base = true;
        // 初始化service
        abort_move_base.call(is_tag_detected);
    }

    int TagFollower::checkTransition(autonomus_transport_industrial_system::tagDetected is_tag_detected,
                                      autonomus_transport_industrial_system::tagDetected is_tag_detected_last)
    {
        if (is_tag_detected.request.is_tag_detected == DETECTED && is_tag_detected_last.request.is_tag_detected == UNDETECTED)
        {
            check_transition_status = TRANSITION;
            return check_transition_status;
        }
        else if (is_tag_detected.request.is_tag_detected == DETECTED && is_tag_detected_last.request.is_tag_detected == DETECTED)
        {
            check_transition_status = ALWAYS_DETECTED;
            return check_transition_status;
        }
        else
        {
            check_transition_status = ALWAYS_NO_DETECTED;
            return check_transition_status;
        }
    }

    void TagFollower::genTraj(const cv::Point2d& p_0, double w_0, const cv::Point2d& p_1, double w_1)
    {
        /*****  求出交点（第三个控制点）  ****/
        // y = a*x + b, where y = 0
        double a = tan(w_1 / M_PI * 180);
        double b = p_1.y - a * p_1.x;
        cv::Point2d p_2((-b / a), 0);
        ROS_INFO_STREAM("Intersection point : " << p_2);
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(p_2.x, p_2.y, 0)), ros::Time::now(), "base_link", "p_2"));
    }


}

#endif