#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file server.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 套接字通信（服务器）
 * @version 0.1
 * @date 2024-09-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include "uart.hpp"

using namespace std;

class Server
{
private:
    int socketId, newSocket;
    struct sockaddr_in address;
    std::thread threadRes; // 接收子线程

    /**
     * @brief 32位数据内存对齐/联合体
     *
     */
    typedef union
    {
        char buffC[4];
        uint8_t buff[4];
        float float32;
        int int32;
    } Bit32Union;

    /**
     * @brief 16位数据内存对齐/联合体
     *
     */
    typedef union
    {
        char buffC[2];
        uint8_t buff[2];
        int int16;
        uint16_t uint16;
    } Bit16Union;

public:
    Server() {};
    ~Server()
    {
        closeServer();
    };

    Uart uart;
    int countDrop = 0;     // 应用程序掉线计数器
    bool startApp = false; // 应用程序启动标志

    /**
     * @brief 启动服务器监听
     *
     */
    bool start()
    {
        // 串口初始化
        if (uart.open("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0") != 0)
        {
            printf("[Error] Uart Open failed!\n");
            return false;
        }
        uart.startReceive(); // 启动数据接收子线程

        // 串口打开会触发DTR复位STM32，等待3秒让MCU启动完毕
        usleep(3000000);

        // 发送多次激活帧，确保STM32就绪
        uint8_t initFrame[6] = {0xAA, 0x00, 0x00, 0x00, 0x5A, 0xDD};
        for (int i = 0; i < 10; i++)
        {
            uart.transmitBytes(initFrame, 6);
            usleep(50000); // 50ms间隔
        }

        // 创建套接字
        if ((socketId = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
            perror("socket failed");
            return false;
        }

        // 绑定套接字到地址和端口
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(8899);

        if (bind(socketId, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            perror("bind failed");
            return false;
        }

        // 监听套接字
        if (listen(socketId, 3) < 0)
        {
            perror("listen");
            return false;
        }

        // 启动串口接收子线程
        threadRes = std::thread([this]()
                                {
        // 接受客户端连接
        int addrlen = sizeof(address);
        if ((newSocket = accept(socketId, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
        {
            perror("accept");
            return false;
        }
      while (1) 
      {
        receive(); // 串口接收校验
      } });

        return true;
    }

    /**
     * @brief 注销套接字通信
     *
     */
    void closeServer()
    {
        // 关闭套接字
        threadRes.join();
        close(newSocket);
        close(socketId);
        uart.close(); // 串口通信关闭
    }

    /**
     * @brief 接收客户端数据
     *
     */
    void receive()
    {
        char buffer[1024] = {0};
        // 接收客户端消息
        int len = recv(newSocket, buffer, 1024, 0);
        if (len <= 0)
        {
            // 客户端已退出
            perror("Client is outting!");
            int addrlen = sizeof(address);
            if ((newSocket = accept(socketId, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
            {
                perror("accept");
                exit(EXIT_FAILURE);
            }
        }

        startApp = true;
        countDrop = 0;
        // 发送至串口通信：只转发 0xAA 协议帧（批量发送，避免逐字节卡死STM32）
        if (len >= 6 && len <= 30 && (unsigned char)buffer[0] == 0xAA)
        {
            uart.transmitBytes((const uint8_t *)buffer, len);
        }
    }

    /**
     * @brief 发送数据
     *
     */
    void transmit(string data)
    {
        if (!startApp)
            return;

        const char *greeting = data.c_str();
        // 发送消息到客户端
        if (send(newSocket, greeting, strlen(greeting), 0) < 0)
        {
            perror("send failed");
        }
    }
};
