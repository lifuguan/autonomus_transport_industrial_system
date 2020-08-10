/*
 * @Author: lifuguan
 * @Date: 2019-11-27 16:24:05
 * @LastEditTime: 2020-08-10 11:11:05
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/src/test.cpp
 */

#include "../include/utility.h"

#include "../include/NetworkCom.h"
#include "../include/PoseDrawer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NetworkCom");
    ros::NodeHandle nh;

    AutonomusTransportIndustrialSystem::NetworkCom sevcom("119.3.151.139", 8003, nh);
    AutonomusTransportIndustrialSystem::PoseDrawer pd(nh);
    tf::StampedTransform location;
    ros::Rate rate(0.5);
    int i = 0;

#pragma omp parallelaa
#pragma omp parallel sections
    {
#pragma omp section
        {
            while (ros::ok())
            {
                location = pd.TfListener("map", "base_link");
                std::cout << "location : " << location.getOrigin().getX() << "," << location.getOrigin().getY() << std::endl;
                std::string json_str = sevcom.jsonGenerator(sevcom.current_pos, location.getOrigin().getX(), location.getOrigin().getY(), 0);
                // ROS_INFO(json_str.c_str());
                // 发送定位信息到服务器
                sevcom.sevComUpload(json_str);
                i += 1;
                rate.sleep();
            }
        }
#pragma omp section
        {

            while (ros::ok())
            {
                std::string str_recv = sevcom.sevComRecv();
                // 解析并发送到move_base
                sevcom.recvJsonGoalToPub(str_recv);
            }
        }
    }

    return 0;
}
