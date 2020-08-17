/*
 * @Author: your name
 * @Date: 2020-04-10 08:57:47
 * @LastEditTime: 2020-08-17 19:57:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/src/test.cpp
 */


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>


#include "../include/utility.h"
#include "../include/PoseDrawer.h"
#include "../include/PointCloud.h"
#include "../include/NavigationGoal.h"
#include "../include/ExtractionDisplay.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    AutonomusTransportIndustrialSystem::ExtractionDisplay exd(nh);
    AutonomusTransportIndustrialSystem::PoseDrawer pd(nh);
    AutonomusTransportIndustrialSystem::NavigationGoal ng(nh);

    ros::Rate rate(0.5);
    while (ros::ok())
    {
        // 广播可视化提取点
        exd.extraction_pub.publish(exd.extraction_array);

        ros::spinOnce();
        rate.sleep();
    }
    
    return EXIT_SUCCESS;
}
