/*
 * @Author: lifuguan
 * @Date: 2020-02-21 17:11:17
 * @LastEditTime: 2020-05-15 00:23:31
 * @LastEditors: Please set LastEditors
 * @Description: 功能性头文件，包含各种基本函数
 * @FilePath: /autonomus_transport_industrial_system/src/utility.h
 */

#ifndef UTILITY_H
#define UTILITY_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

namespace AutonomusTransportIndustrialSystem
{
class Utility
{
public:
    /**
     * @description: 计算两点之间距离
     * @param pose_1 第一个点  
     * @param pose_2 第二个点
     * @return: double型变量，返回欧几里得距离
     */
    double GetEuclideanDistance(const geometry_msgs::PoseStamped &pose_1, const geometry_msgs::PoseStamped &pose_2);

    /**
     * @description: 计算两点之间距离
     * @param transform 两个tf之间的静态变化呢
     * @return: double型变量，返回欧几里得距离
     */
    double GetEuclideanDistance(tf::StampedTransform transform);

    /**
     * @description: 将四元数转换成欧拉角
     * @param orientation 四元数 
     * @note: roll代表绕x轴旋转，pitch代表绕y轴旋转，yaw代表绕z轴旋转
     * @return: 返回一个double型数组 RPY[rall, pitch, yaw, \t]
     */
    double *GetYawFromOrientation(const geometry_msgs::Quaternion &orientation);

private:
};

double AutonomusTransportIndustrialSystem::Utility::GetEuclideanDistance(
    const geometry_msgs::PoseStamped &pose_1, const geometry_msgs::PoseStamped &pose_2)
{
    return hypot(pose_1.pose.position.x - pose_2.pose.position.x, pose_1.pose.position.y - pose_2.pose.position.y);
}

double AutonomusTransportIndustrialSystem::Utility::GetEuclideanDistance(tf::StampedTransform transform)
{
    return hypot(transform.getOrigin().getX(), transform.getOrigin().getY());
}


double *AutonomusTransportIndustrialSystem::Utility::GetYawFromOrientation(
    const geometry_msgs::Quaternion &orientation)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(orientation, q);
    tf::Matrix3x3 m(q);
    static double RPY[4];
    m.getRPY(RPY[0], RPY[1], RPY[2]);
    return RPY;
}
} // namespace AutonomusTransportIndustrialSystem

#endif //PROJECT_UTILITY_H