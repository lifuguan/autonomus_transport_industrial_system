/*
 * @Author: lifuguan
 * @Date: 2019-11-27 16:24:05
 * @LastEditTime: 2020-05-21 21:57:40
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/src/test.cpp
 */

#include "../include/utility.h"

#include "../include/NetworkCom.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NetworkCom");
    ros::NodeHandle nh;

    AutonomusTransportIndustrialSystem::NetworkCom sevcom("106.13.162.250", 8003, nh);

    ros::Rate rate(0.5);
    int i = 0;

    #pragma omp parallelaa
    #pragma omp parallel sections
    {
        #pragma omp section
        {
            while (ros::ok())
            {
                std::string json_str = sevcom.jsonGenerator(sevcom.current_pos, 0.0+0.01*i, 0.1, 0);
                // ROS_INFO(json_str.c_str());
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
                // 解析并发送
                sevcom.recvJsonGoalToPub(str_recv);
                rate.sleep();
            }
        }
    }
    

    

    return 0;
}

