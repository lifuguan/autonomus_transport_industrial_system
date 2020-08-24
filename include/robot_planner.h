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


    class RobotPlanner : public nav_core::BaseGlobalPlanner
    {
    public:

        RobotPlanner();
        /**
         * @brief  ValentineRobotPlanner的构造函数
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        RobotPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        ~RobotPlanner() = default;

    private:
        costmap_2d::Costmap2DROS* costmap_ros_;
        double step_size_, min_dist_from_robot_;
        costmap_2d::Costmap2D* costmap_;
        base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use

        bool initialized_;

        /**
         * 最重要的函数：接收一个来自world的goal_pose，并求出路径
         * @param start 出发点pose
         * @param goal 目标点pose
         * @param plan  通过planner计算得出的plan序列
         * @return
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);
        /**
         * 初始化函数
         * @param name
         * @param costmap_ros
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    };

    RobotPlanner::RobotPlanner()
    {
        costmap_ros_ = NULL;
        initialized_ = false;
    }

    RobotPlanner::RobotPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        costmap_ros_ = NULL;
        initialized_ = false;
        initialize(name, costmap_ros);
    }

    void RobotPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (initialized_ == true)
        {
            ROS_WARN_STREAM("This planner has already been initialized.");
        }
        else
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            ros::NodeHandle nh("~/"+name);

            world_model_ = new base_local_planner::CostmapModel(*costmap_);

            initialized_ = true;
        }
    }

    bool RobotPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                         const geometry_msgs::PoseStamped &goal,
                                         std::vector<geometry_msgs::PoseStamped> &plan)
    {
        // 检查是否初始化
        if (initialized_ == false)
        {
            ROS_ERROR_STREAM("The planner is not initialized, please call initialize() at the first time.");
            return false;
        }

        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

        plan.clear();
        costmap_ = costmap_ros_->getCostmap();

        // 起始点
        plan.push_back(start);

        // 中途的采样点
        for (int i=0; i<20; i++)
        {
            geometry_msgs::PoseStamped new_goal = goal;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

            new_goal.pose.position.x = -2.5+(0.05*i);
            new_goal.pose.position.y = -3.5+(0.05*i);

            new_goal.pose.orientation.x = goal_quat.x();
            new_goal.pose.orientation.y = goal_quat.y();
            new_goal.pose.orientation.z = goal_quat.z();
            new_goal.pose.orientation.w = goal_quat.w();

            plan.push_back(new_goal);
        }

        // 最终目的地
        plan.push_back(goal);
        return true;

    }
}


#endif //ROBOT_PLANNER_H
