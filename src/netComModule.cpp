/*
 * @Author: your name
 * @Date: 2020-04-10 08:57:47
 * @LastEditTime: 2020-05-12 22:40:34
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

#include "../include/utility.h"

#include "../include/ServerCom.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    AutonomusTransportIndustrialSystem::ServerCom sevcom("106.13.162.250", 8003);

    ros::Rate rate(0.5);
    int i = 0;
    while (ros::ok())
    {
        std::string jsonStr = sevcom.jsonGenerator(200, 0.0+0.01*i, 0.1, 0);
        // ROS_INFO(jsonStr.c_str());
        sevcom.sevComUpload(jsonStr);
        i += 1;
        ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}

