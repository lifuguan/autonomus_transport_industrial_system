//
// Created by robomaster on 2020/8/17.
//

#include "../include/tag_follower.h"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>





int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_follower");
    ros::NodeHandle nh;
    AutonomusTransportIndustrialSystem::TagFollower tg(nh);
    ros::Timer tag_timer = nh.createTimer(ros::Duration(0.1), &AutonomusTransportIndustrialSystem::TagFollower::tagMakePlan, &tg);

    ros::spin();
    return  EXIT_SUCCESS;
}