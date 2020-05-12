/*
 * @Author: your name
 * @Date: 2020-05-02 16:24:16
 * @LastEditTime: 2020-05-13 00:47:28
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

#include "autonomus_transport_industrial_system/netComGoal.h"

#define MAXLINE 4096

namespace AutonomusTransportIndustrialSystem
{
    class NetworkCom
    {
    private:
        int client_sockfd;
        int len;
        struct sockaddr_in remote_addr;
        char bufSend[BUFSIZ];  //数据传送的缓冲区
        char bufRecv[BUFSIZ];  //数据接收的缓冲区

    public:
        enum code_type
        {
            current_pos = 200,  // 同步位置
            goal_pos = 100    // 送货位置
        };

        NetworkCom(std::string ipaddr, int port);

        ~NetworkCom() = default;

        void sevComUpload(std::string send_string);

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
         * @description: 解析从TCP处得到的json字符串并通过service发布
         * @param str 从TCP处得到的json字符串并
         * @return: none
         */
        void recvJsonGoalToPub(std::string str);
    };
    
    NetworkCom::NetworkCom(std::string ipaddr, int port)
    {
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
            bufSend[i] = send_string[i];
        }
        len=send(client_sockfd, bufSend, strlen(bufSend), 0);
        if (len == 0)
        {
            ROS_ERROR("Failed to send.");
            return;
        }        
    }
    std::string NetworkCom::sevComRecv()
    {
        // 接收服务器返回内容

		len=recv(client_sockfd, bufRecv, BUFSIZ, 0);
		bufRecv[len]='/0';
		printf("received:%s\n",bufRecv);
        return bufRecv;
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
            double p_x = root["goal"]["p_x"].asDouble();
            double p_y = root["goal"]["p_y"].asDouble();
            
        }
        
    }
}
#endif