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
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace AutonomusTransportIndustrialSystem
{
    class TagFollower
    {
    private:
        ros::NodeHandle nh;
        tf::StampedTransform tag_transform;

        tf::TransformListener tag_listener;
    public:
        TagFollower(ros::NodeHandle given_nh) : nh(given_nh)
        {
            // 此处采取Service向Navigation订阅目标信息，修改tf监听值
        }
        ~TagFollower() = default;

        bool tagCallBack(const ros::TimerEvent&);
    };

    bool TagFollower::tagCallBack(const ros::TimerEvent &)
    {
        // 读取tag的tf
        try
        {
            tag_listener.lookupTransform("/base_link", "/tag_0", ros::Time(0), tag_transform);
            ROS_INFO_STREAM("Actual position: x = " << tag_transform.getOrigin().z() << ", y = " << tag_transform.getOrigin().x());

        } catch (tf::TransformException &ex)
        {
            ROS_WARN_STREAM("tf lookup is not exist : " << ex.what());
            ros::Duration(1.0).sleep();
        }
    }


}


#endif