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

namespace AutonomusTransportIndustrialSystem
{
    class tag_follower
    {
    private:
        ros::NodeHandle nh;
    public:
        tag_follower(ros::NodeHandle given_nh) : nh(given_nh);
        ~tag_follower() = default;
    };
    
    tag_follower::tag_follower(ros::NodeHandle given_nh)
    {

    }

    
}


#endif