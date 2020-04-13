/*
 * @Author: your name
 * @Date: 2020-04-12 22:22:42
 * @LastEditTime: 2020-04-13 16:53:02
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/include/PointCloud.h
 */

#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

namespace AutonomusTransportIndustrialSystem
{
    class PointCloudFusion
    {
        public:
        /**
         * @description: 初始化函数
         * @param n_ 
         */
        PointCloudFusion(ros::NodeHandle given_nh) : n_(given_nh)
        {
            // error: invalid use of non-static member function
            sub_pc_front = n_.subscribe<sensor_msgs::PointCloud2>("/kinect_alpha/kinect/depth/points", 1, &PointCloudFusion::pcCallBack, this);
            sub_pc_left = n_.subscribe<sensor_msgs::PointCloud2>("/kinect_left/kinect_left/depth/points", 1,  &PointCloudFusion::pcCallBack, this);
            sub_pc_right = n_.subscribe<sensor_msgs::PointCloud2>("/kinect_right/kinect_right/depth/points", 1,  &PointCloudFusion::pcCallBack, this);
        }
        
        private:
        ros::NodeHandle n_;
        // 三个摄像头的订阅
        ros::Subscriber sub_pc_front;
        ros::Subscriber sub_pc_left;
        ros::Subscriber sub_pc_right;
        sensor_msgs::PointCloud2 pc_data[4];
        sensor_msgs::PointCloud2 pc_data_sub;
        void pcCallBack(const sensor_msgs::PointCloud2ConstPtr& msg)
        {
            if (msg->header.frame_id == "camera_right")
            {
                pc_data[0] = *msg;
            }
            else if (msg->header.frame_id == "camera_left")
            {
                pc_data[1] = *msg;
            }
            else if (msg->header.frame_id == "camera_front")
            {
                pc_data[2] = *msg;
            }
            else
            {
                ROS_ERROR("UNKNOWN SUBSCRIBE!");
            }         
            pc_data_sub += pc_data[0];
        }
    };

}

#endif
