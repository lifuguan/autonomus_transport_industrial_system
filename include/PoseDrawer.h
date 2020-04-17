/*
 * @Author: your name
 * @Date: 2020-02-27 15:10:30
 * @LastEditTime: 2020-04-17 21:18:49
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/include/PoseDrawer.h
 */

#ifndef POSEDRAWER_H
#define POSEDRAWER_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <message_filters/subscriber.h>

namespace AutonomusTransportIndustrialSystem
{
    
class PoseDrawer
{
public:
    /**
     * @description: 初始化函数
     * @param n_ 
     */
    PoseDrawer(ros::NodeHandle given_nh) : n_(given_nh)
    {
    }

    /**
     * @description: pose监听函数，调用poseCallBack回调函数
     * @param pose_sub_id 要监听的pose的id 
     * @param target_frame 要转换的目标tf的id 
     * @return: 在回调函数里
     */
    void PoseListener(std::string pose_sub_id, std::string target_frame);

    /**
     * @description: 公有的pose坐标转换
     * @param target_frame 要转换的目标tf的id
     * @param pose_in 要转换的当前pose
     * @return: pose_out_ 转换后的posestamped变量
     */
    geometry_msgs::PoseStamped TransformPose(const std::string target_frame, const geometry_msgs::PoseStamped pose_in);

    /**
     * @description: TODO (copy from robomaster 2020)
     * @param transform
     * @param input_pose
     * @param output_pose
     * @return: 
     */
    void TransformPose(const tf::StampedTransform transform, const geometry_msgs::PoseStamped &input_pose, geometry_msgs::PoseStamped &output_pose);

    /**
     * @description: 公有的pose广播函数
     * @param pose_brocaster_id 要广播的pose的名字
     * @param pose_brocaster 要广播的pose的内容
     */
    void PoseBroadcaster(std::string pose_brocaster_id, geometry_msgs::PoseStamped pose_brocaster);

    // 败笔
    geometry_msgs::PoseStamped pose_in;
    geometry_msgs::PoseStamped pose_out;
private:
    ros::NodeHandle n_;
    tf::TransformListener tf_;
    std::string target_frame_;

    
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    tf::MessageFilter<geometry_msgs::PoseStamped> *tf_filter_;
    ros::Publisher pose_publisher;
    
    /**
     * @description: 完全私有的pose广播函数
     * @param pose_brocaster_id 要广播的pose的名字
     * @param pose_out 广播内容
     */
    void PoseBroadcaster(std::string pose_brocaster_id);

    /**
     * @description: 完全私有的pose坐标转换
     * @return: pose_out 转换后的posestamped变量
     */
    void TransformPose();

    /**
     * @description: 监听pose的回调函数 (同时，其余操作可以在这里添加)
     * @return pose_in 返回全局私有posestamped变量
     */
    void poseCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped> &pose_ptr);
};


void AutonomusTransportIndustrialSystem::PoseDrawer::PoseListener(std::string pose_sub_id, std::string target_frame)
{
    target_frame_ = target_frame;

    pose_sub_.subscribe(n_, pose_sub_id, 10);
    // MessageFilter可以读取任何ros的信息并缓存起来知道可以被处理成目标帧
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(pose_sub_, tf_, target_frame, 10);
    tf_filter_->registerCallback(boost::bind(&PoseDrawer::poseCallback, this, _1));
}

void AutonomusTransportIndustrialSystem::PoseDrawer::PoseBroadcaster(std::string pose_brocaster_id)
{
    pose_publisher = n_.advertise<geometry_msgs::PoseStamped>(pose_brocaster_id, 1);
    pose_publisher.publish(pose_out);
}

void AutonomusTransportIndustrialSystem::PoseDrawer::PoseBroadcaster(std::string pose_brocaster_id, geometry_msgs::PoseStamped pose_brocaster)
{
    pose_publisher = n_.advertise<geometry_msgs::PoseStamped>(pose_brocaster_id, 1);
    pose_publisher.publish(pose_brocaster);
}

void AutonomusTransportIndustrialSystem::PoseDrawer::poseCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped> &pose_ptr)
{
    try
    {
        // 传递监听信息
        pose_in = *pose_ptr;
        // 其余操作
        // 败笔
        pose_out = TransformPose("base_link", pose_in);
    }
    catch (tf::TransformException &ex)
    {
        printf("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

void AutonomusTransportIndustrialSystem::PoseDrawer::TransformPose()
{
    try
    {
        tf_.transformPose(target_frame_, pose_in, pose_out);
        ROS_INFO_STREAM(pose_out);
    }
    catch (tf::TransformException &ex)
    {
        printf("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

geometry_msgs::PoseStamped AutonomusTransportIndustrialSystem::PoseDrawer::TransformPose(const std::string target_frame, const geometry_msgs::PoseStamped pose_in)
{
    if (target_frame == pose_in.header.frame_id)
    {
        return pose_in;
    }
    geometry_msgs::PoseStamped pose_out;
    tf::Stamped<tf::Pose> input_pose_tf, output_pose_tf;

    // pose转换成tf
    tf::poseStampedMsgToTF(pose_in, input_pose_tf);
    try
    {
        // 等待直到可以进行转换
        tf_.waitForTransform( target_frame, ros::Time::now(), 
                                pose_in.header.frame_id, pose_in.header.stamp, 
                                pose_in.header.frame_id, ros::Duration(0.5) );
        // 在tf环境下进行坐标转换
        tf_.transformPose(target_frame, input_pose_tf, output_pose_tf);
        ROS_INFO_STREAM(pose_out);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("ERROR: %s", ex.what());
    }

    // 将tf转换成pose
    tf::poseStampedTFToMsg(output_pose_tf, pose_out);
    return pose_out;
}

void AutonomusTransportIndustrialSystem::PoseDrawer::TransformPose(
    const tf::StampedTransform transform, const geometry_msgs::PoseStamped &input_pose, geometry_msgs::PoseStamped &output_pose)
{

    tf::Stamped<tf::Pose> input_pose_tf;
    tf::poseStampedMsgToTF(input_pose, input_pose_tf);
    input_pose_tf.setData(transform * input_pose_tf);
    input_pose_tf.stamp_ = transform.stamp_;
    input_pose_tf.frame_id_ = transform.frame_id_;
    tf::poseStampedTFToMsg(input_pose_tf, output_pose);
}

} // namespace AutonomusTransportIndustrialSystem

#endif //PROJECT_UTILITY_H