/*
 * @Author: your name
 * @Date: 2020-04-10 08:57:47
 * @LastEditTime: 2020-05-21 00:25:45
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
    
    ros::Rate rate(0.5);
    while (ros::ok())
    {
        // 广播可视化提取点
        exd.extraction_pub.publish(exd.extraction_array);

        
        ros::spinOnce();
        rate.sleep();
    }

        tf::StampedTransform transform =pd.TfListener("map", "odom");
        // 若tf对象不为空，则继续运算
        if (transform.frame_id_ != "ERROR")
        {         
            // 若odom和map的欧拉距离小于阈值，则继续发布导航指令
            if (hypot(transform.getOrigin().getX(), transform.getOrigin().getY()) < 5)
            {
                // pubNavigationGoal(req.g_x, req.g_y, 0, 0, 0, 0, 0); //广播goal坐标到move_base中
                // res.status = true; // 返回真，证明收到
                return true;
            }   
            else
            {
                
                ROS_ERROR("map and odom frame have a massive error!");
                return false;
            }
        }
        else
        {
            return false;  
        }
    
    return 0;
}

