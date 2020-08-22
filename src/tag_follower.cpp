//
// Created by robomaster on 2020/8/17.
//

#include "tag_follower.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_follower");
    ros::NodeHandle nh;
    AutonomusTransportIndustrialSystem::TagFollower tg(nh);
    ros::Timer tag_timer = nh.createTimer(ros::Duration(0.1), &AutonomusTransportIndustrialSystem::TagFollower::tagEventProcess, &tg);

    ros::spin();
    return  EXIT_SUCCESS;
}