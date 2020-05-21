/*
 * @Author: your name
 * @Date: 2020-05-15 00:29:47
 * @LastEditTime: 2020-05-15 00:40:00
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/include/ExtractionDisplay.h
 */ 

#ifndef EXTRACTION_H
#define EXTRACTION_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>


namespace AutonomusTransportIndustrialSystem
{
    class ExtractionDisplay
    {
    private:
        ros::NodeHandle nh;
    public:
        // 货柜地点
        visualization_msgs::MarkerArray extraction_array;
        ros::Publisher extraction_pub;

        ExtractionDisplay(ros::NodeHandle given_nh);
        ~ExtractionDisplay() = default;

        /**
         * @description: 生成货物提取点
         * @param extraction_array 
         * @return: extraction_array
         */
        void extractionGenerator(visualization_msgs::MarkerArray &extraction_array);
    };

    ExtractionDisplay::ExtractionDisplay(ros::NodeHandle given_nh):nh(given_nh)
    {
        extraction_pub = nh.advertise<visualization_msgs::MarkerArray>("extraction_array", 10);
        
        extraction_array.markers.resize(4);
        extractionGenerator(extraction_array);
    }



    void ExtractionDisplay::extractionGenerator(visualization_msgs::MarkerArray &extraction_array)
    {
        // 生成相同部分
        for (int i = 0; i < 4; i++)
        {
            extraction_array.markers[i].header.frame_id = "map";
            extraction_array.markers[i].header.stamp = ros::Time::now();
            extraction_array.markers[i].ns = "red";
            extraction_array.markers[i].id = i;
            extraction_array.markers[i].action = visualization_msgs::Marker::ADD;
            extraction_array.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;

            extraction_array.markers[i].text = std::to_string(i);
            extraction_array.markers[i].scale.z = 0.4;
            extraction_array.markers[i].color.a = 1.0;
            extraction_array.markers[i].color.r = 1.0;
            extraction_array.markers[i].color.g = 0.0;
            extraction_array.markers[i].color.b = 0.0;
        }

        // 左边红色圆柱体
        extraction_array.markers[0].pose.position.x = 3.15;
        extraction_array.markers[0].pose.position.y = 3.35;
        extraction_array.markers[0].pose.position.z = 0.0;
        extraction_array.markers[0].pose.orientation.x = 0.0;
        extraction_array.markers[0].pose.orientation.y = 0.0;
        extraction_array.markers[0].pose.orientation.z = 0.0;
        extraction_array.markers[0].pose.orientation.w = 1.0;
        // 中间绿色圆柱体
        extraction_array.markers[1].pose.position.x = 6.21;
        extraction_array.markers[1].pose.position.y = 2.60;
        extraction_array.markers[1].pose.position.z = 0.0;
        extraction_array.markers[1].pose.orientation.x = 0.0;
        extraction_array.markers[1].pose.orientation.y = 0.0;
        extraction_array.markers[1].pose.orientation.z = 0.0;
        extraction_array.markers[1].pose.orientation.w = 1.0;
        // 右边蓝色圆柱体
        extraction_array.markers[2].pose.position.x = 7.65;
        extraction_array.markers[2].pose.position.y = -0.47;
        extraction_array.markers[2].pose.position.z = 0.1;
        extraction_array.markers[2].pose.orientation.x = 0.0;
        extraction_array.markers[2].pose.orientation.y = 0.0;
        extraction_array.markers[2].pose.orientation.z = 0.0;
        extraction_array.markers[2].pose.orientation.w = 1.0;
        
    }
} // namespace AutonomusTransportIndustrialSystem

#endif