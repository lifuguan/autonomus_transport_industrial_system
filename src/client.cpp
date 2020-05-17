#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <string>
#include <sstream>
#include <iostream> 
#include <omp.h>
#include <time.h>

using namespace std;
//#include "../include/NetworkCom.h"
#define MYPORT  8003
#define BUFFER_SIZE 1024
int main()
{
    int sock_cli;
    clock_t ti0;
    double ti,ti2;
    fd_set rfds;
    struct timeval tv;
    int retval, maxfd;

    ///定义sockfd
    sock_cli = socket(AF_INET,SOCK_STREAM, 0);
    ///定义sockaddr_in
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(MYPORT);  ///服务器端口
    servaddr.sin_addr.s_addr = inet_addr("106.13.162.250");  ///服务器ip

    //连接服务器，成功返回0，错误返回-1
    while (connect(sock_cli, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        perror("connect");
        exit(1);                /*用户输入信息了,开始处理信息并发送*/
                char sendbuf[BUFFER_SIZE];
                //fgmeets(sendbuf, sizeof(sendbuf), stdin);
                //std::string jsonStr = jsonGenerator(200, 0.0+0.01*i, 0.1, 0);
                //sendbuf = jsonStr.toCharArray();
                string str ("{\"type\":200,\"date\":{\"x\":0.5,\"y\":0.5}}");
                strcpy(sendbuf,str.c_str());
                send(sock_cli, sendbuf, strlen(sendbuf),0); //发送
                memset(sendbuf, 0, sizeof(sendbuf));
                
    }
    while(1)
    {
        /*把可读文件描述符的集合清空*/
        FD_ZERO(&rfds);
        /*把标准输入的文件描述符加入到集合中*/
        FD_SET(0, &rfds);
        maxfd = 0;
        /*把当前连接的文件描述符加入到集合中*/
        FD_SET(sock_cli, &rfds);
        /*找出文件描述符集合中最大的文件描述符*/
        if(maxfd < sock_cli)
            maxfd = sock_cli;
        #pragma omp parallelaa
        #pragma omp parallel sections
        {
            #pragma omp section
            {
                while(1)
                {
                    double ti = omp_get_wtime();
                    if(ti-ti2>=0.5)
                    {
                        ti2 = ti;
                        /*用户输入信息了,开始处理信息并发送*/
                        char sendbuf[BUFFER_SIZE];
                        //fgets(sendbuf, sizeof(sendbuf), stdin);
                        //std::string jsonStr = jsonGenerator(200, 0.0+0.01*i, 0.1, 0);
                        //sendbuf = jsonStr.toCharArray();
                        string str ("{\"type\":200,\"date\":{\"x\":0.5,\"y\":0.5}}\n");
                        strcpy(sendbuf,str.c_str());
                        send(sock_cli, sendbuf, strlen(sendbuf),0); //发送
                        memset(sendbuf, 0, sizeof(sendbuf));
                    }
                }
            }
            #pragma omp section
            {
                while(1)
                {
                    /*服务器发来了消息*/
                    if(FD_ISSET(sock_cli,&rfds))
                    {
                        char recvbuf[BUFFER_SIZE];
                        memset(recvbuf, 0, sizeof(recvbuf));
                        int len;
                        len = recv(sock_cli, recvbuf, sizeof(recvbuf),0);
                        printf("%s", recvbuf);                
                    }
                }
                
            }

        }
    }
    close(sock_cli);
    return 0;
}

