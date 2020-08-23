/*
 * @Author: your name
 * @Date: 2020-05-02 16:24:16
 * @LastEditTime: 2020-08-04 15:55:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/include/NetworkCom.h
 */



#ifndef SERVERSOM_
#define SERVERSOM_

#include <iostream>
#include <cstring>
#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <jsoncpp/json/json.h>
#include <json/writer.h>

#include <ros/ros.h>
#include "autonomus_transport_industrial_system/netComGoal.h"
#include "autonomus_transport_industrial_system/netComTag.h"
#include "utility.h"

#include <omp.h>
#include <unordered_map>
#define MAXLINE 4096

namespace AutonomusTransportIndustrialSystem
{
    class NetworkCom
    {
    private:
        int client_sockfd;
        int len;
        struct sockaddr_in remote_addr;
        fd_set rfds;
        char buf_send[BUFSIZ];  //数据传送的缓冲区
        char buf_recv[BUFSIZ];  //数据接收的缓冲区

        ros::NodeHandle nh;
        ros::ServiceClient goal_client; //ros service广播goal信息给navigationGoal节点
        ros::ServiceClient tag_client;  //ros service广播tag信息给tag_follower节点

        autonomus_transport_industrial_system::netComGoal comGoal;
        autonomus_transport_industrial_system::netComTag comTag;

        std::unordered_map<std::pair<double, double>, int, hash_pair> goal_tag_hash_map;
    public:
        enum code_type
        {
            current_pos = 200,  // 同步位置
            goal_pos = 100    // 送货位置
        };

        /**
         * @description: 初始化函数
         * @param: none
         * @return: none
         */
        NetworkCom(std::string ipaddr, int port, ros::NodeHandle given_nh);

        ~NetworkCom() = default;

        /**
         * @description: 将机器人位置通过TCP发送给服务器
         * @param send_string 要发送的json字符串
         * @return: none
         */
        void sevComUpload(std::string send_string);

        /**
         * @description: 从服务器处接收目的地
         * @param 
         * @return:要接收的json字符串
         */
        std::string sevComRecv();

        /**
         * @description: 将机器人当前位姿转换成json的字符串
         * @param code json广播代码
         * @param p_x position X ray
         * @param p_y position Y ray
         * @param p_w raotation
         * @return: 
         */
        std::string jsonGenerator(int code, double p_x, double p_y, double p_w);


        /**
         * @description: 解析从TCP处得到的json字符串解析得到坐标并通过service发布
         * @param str 从TCP处得到的json字符串并
         * @return: none
         */
        void recvJsonGoalToPub(std::string str);
    };

    NetworkCom::NetworkCom(std::string ipaddr, int port, ros::NodeHandle given_nh):nh(given_nh)
    {
        // 初始化：坐标——tag 哈希表
        std::pair<double, double> goal1(3.2,3); goal_tag_hash_map[goal1] = 0;
        std::pair<double, double> goal2(7,2); goal_tag_hash_map[goal2] = 1;
        std::pair<double, double> goal3(8.5,-1); goal_tag_hash_map[goal3] = 2;

        goal_client = nh.serviceClient<autonomus_transport_industrial_system::netComGoal>("netComGoal");
        tag_client = nh.serviceClient<autonomus_transport_industrial_system::netComTag>("netComTag");

        memset(&remote_addr,0,sizeof(remote_addr)); //数据初始化--清零
        remote_addr.sin_family=AF_INET; //设置为IP通信
        remote_addr.sin_addr.s_addr=inet_addr(ipaddr.c_str());//服务器IP地址
        remote_addr.sin_port=htons(port); //服务器端口号

        /*创建客户端套接字--IPv4协议，面向连接通信，TCP协议*/
        if((client_sockfd=socket(PF_INET,SOCK_STREAM,0))<0)
        {
            perror("socket error");
        }

        /*将套接字绑定到服务器的网络地址上*/
        if(connect(client_sockfd,(struct sockaddr *)&remote_addr,sizeof(struct sockaddr))<0)
        {
            perror("connect error");
        }
    }

    void NetworkCom::sevComUpload(std::string send_string)
    {
        for (int i = 0; i < send_string.length(); i++)
        {
            buf_send[i] = send_string[i];
        }
        len=send(client_sockfd, buf_send, strlen(buf_send), 0);
        if (len == 0)
        {
            ROS_ERROR("Failed to send.");
            return;
        }
        else
        {
            ROS_INFO("Succeed to send.");
            memset(buf_send, 0, sizeof(buf_send));
        }

    }
    std::string NetworkCom::sevComRecv()
    {
        // 接收服务器返回内容
        std::cout << "Waiting to receive..." << std::endl;
        len=recv(client_sockfd, buf_recv, BUFSIZ, 0);
        buf_recv[len]='/0';
        return buf_recv;
    }
    std::string NetworkCom::jsonGenerator(int code, double p_x, double p_y, double p_w)
    {
        // 一定要分开写
        Json::Value root;
        root["type"] = code;
        root["data"]["x"] = p_x;
        root["data"]["y"] = p_y;
        // std::cout << root << std::endl;
        Json::FastWriter writer;
        return writer.write(root);
    }

    void NetworkCom::recvJsonGoalToPub(std::string str)
    {
        Json::Reader reader;
        Json::Value root;
        reader.parse(str, root);
        if (root["type"] == NetworkCom::goal_pos)
        {
            std::pair<double, double> goal_recv(root["data"]["x"].asDouble(), root["data"]["y"].asDouble());

            // 在哈希表中查找是否存在该位置
            if (goal_tag_hash_map.find(goal_recv) != goal_tag_hash_map.end())
            {
                ROS_INFO_STREAM("(" << goal_recv.first << ", " << goal_recv.second  << ") match the hash map.");

                comTag.request.tag_num = goal_tag_hash_map[goal_recv];

                comGoal.request.g_x = root["data"]["x"].asDouble();
                comGoal.request.g_y = root["data"]["y"].asDouble();

                // 尝试建立通信并上传信息
                if (goal_client.call(comGoal))
                {
                    std::cout << "Movebase's Service:" << comGoal.response.status << std::endl; // 如果成功的话，接收返回信息
                }
                else
                {
                    ROS_ERROR("Goal service's transaction failed."); return;
                }

                // 此处采取Service向Navigation订阅目标信息，修改tf监听值
                if (tag_client.call(comTag))
                {
                    std::cout << "tag_follower's Service:" << comTag.response.status << std::endl;
                }
                else
                {
                    ROS_ERROR_STREAM("Tag service's transaction failed."); return;
                }
            }
            else
            {
                ROS_ERROR_STREAM("(" << goal_recv.first << ", " << goal_recv.second  << ") doesn't match the hash map."); return;
            }
        }
    }
}
#endif