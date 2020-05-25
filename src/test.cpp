/*
 * @Author: your name
 * @Date: 2020-04-10 08:57:47
 * @LastEditTime: 2020-05-25 22:31:25
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/src/test.cpp
 */
/*
 * @Author: lifuguan
 * @Date: 2019-11-27 16:24:05
 * @LastEditTime: 2020-04-12 22:50:38
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

        // tf::StampedTransform tolerence_tf =pd.TfListener("map", "odom");
        // ROS_INFO("Passing loop.");
        // // 若tf对象不为空，则继续运算
        // if (tolerence_tf.frame_id_ != "ERROR")
        // {         
        //     ROS_INFO("Passing tolerence_tf.");
        //     // 若odom和map的欧拉距离小于阈值，则继续发布导航指令
        //     if (hypot(tolerence_tf.getOrigin().getX(), tolerence_tf.getOrigin().getY()) < 5)
        //     {
        //         ROS_INFO("Passing hypot.");
    
        //         ng.pubNavigationGoal(); //广播goal坐标到move_base中
        //     }   
        //     else
        //     {
        //         ROS_ERROR("map and odom frame have a massive error!");
        //     }
        // }
        ros::spinOnce();
        rate.sleep();
    }

       
    
    return 0;
}

