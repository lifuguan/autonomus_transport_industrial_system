//
// Created by lifuguan on 2020/8/23.
//

#ifndef ROBOT_PLANNER_H
#define ROBOT_PLANNER_H

#include <math.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>

namespace AutonomusTransportIndustrialSystem
{

    /**
     * 情人节走心形
     */
    class ValentineRobotPlanner
    {
    public:
        /**
         * 构造函数
         * @param given_nh
         */
        ValentineRobotPlanner(ros::NodeHandle given_nh);

        ~ValentineRobotPlanner() = default;

    private:
        ros::NodeHandle nh;

        nav_msgs::Path global_path_;

        ros::Publisher path_pub_;
        ros::Timer path_timer_;

        // ???
        std::string global_frame_;


        int dt_ = 3;
        double theta_ = 0;
        double delta_theta_;
        const int step_time_tot_ = 2000;
        int step_time_ = 0;
        double p_x_, p_y_;
        /**
         * 定时回调函数：循环广播路径
         * @param event
         */
        void showGlobalPathCallBack(const ros::TimerEvent& event);
    };

    ValentineRobotPlanner::ValentineRobotPlanner(ros::NodeHandle given_nh):nh(given_nh)
    {
        path_pub_ = nh.advertise<nav_msgs::Path>("valentine_path", 1, true);

        delta_theta_ = 2 * M_PI / step_time_;

        path_timer_ = nh.createTimer(ros::Duration(0.02), &ValentineRobotPlanner::showGlobalPathCallBack, this);
        // ???
        path_timer_.stop();
        // ???
        global_path_.header.frame_id = "map";
    }

    void ValentineRobotPlanner::showGlobalPathCallBack(const ros::TimerEvent &event)
    {
        if (step_time_ == step_time_tot_)
        {
            p_x_ = 0; p_y_ = 0; theta_ = 0;
            step_time_ = 0;
        }
        else
        {
            p_x_ = cos(theta_)*(1+cos(theta_));
            p_y_ = sin(theta_)*(1+cos(theta_));
            theta_ += delta_theta_;
            step_time_ += 1;
        }
    }


}


#endif //ROBOT_PLANNER_H
