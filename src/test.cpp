/*
 * @Author: lifuguan
 * @Date: 2019-11-27 16:24:05
 * @LastEditTime: 2020-02-21 17:21:11
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/src/test.cpp
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <message_filters/subscriber.h>

#include "../include/utility.h"


class PoseDrawer
{
public:
    PoseDrawer(ros::NodeHandle nh) : n_(nh), tf_(), target_frame_("B")
    {
        pose_sub_.subscribe(n_, "pose_a", 10);
        tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(pose_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback(boost::bind(&PoseDrawer::msgCallback, this, _1));
    };

private:
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<geometry_msgs::PoseStamped> *tf_filter_;
    ros::NodeHandle n_;
    std::string target_frame_;

    ros::Publisher pose_publisher_b = n_.advertise<geometry_msgs::PoseStamped>("pose_b", 1);

    void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped> &pose_ptr)
    {
        geometry_msgs::PoseStamped pose_out;
        try
        {
            tf_.transformPose(target_frame_, *pose_ptr, pose_out);
            ROS_INFO_STREAM(pose_out);
            pose_publisher_b.publish(pose_out);
        }
        catch (tf::TransformException &ex)
        {
            printf("Failure %s\n", ex.what()); //Print exception which was caught
        }
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    PoseDrawer pd(nh);

    tf::TransformBroadcaster goal_frame_broadcaster;
    ros::Publisher pose_publisher_a = nh.advertise<geometry_msgs::PoseStamped>("pose_a", 1);

    tf::Transform a_to_b_tf(tf::Quaternion(-1.57, 0, 0.78), tf::Vector3(1, -2, 0));
    geometry_msgs::PoseStamped a_to_m_pose;

    ros::Rate rate(20);
    while (nh.ok())
    {
        goal_frame_broadcaster.sendTransform(tf::StampedTransform(a_to_b_tf, ros::Time::now(), "/A", "/B"));
        a_to_m_pose.header.stamp = ros::Time::now();
        a_to_m_pose.header.frame_id = "A";
        a_to_m_pose.pose.position.x = 1;
        a_to_m_pose.pose.position.y = 1;
        a_to_m_pose.pose.position.z = 1;
        a_to_m_pose.pose.orientation.x = 0;
        a_to_m_pose.pose.orientation.y = 0;
        a_to_m_pose.pose.orientation.z = 0;
        a_to_m_pose.pose.orientation.w = 1;
        pose_publisher_a.publish(a_to_m_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}