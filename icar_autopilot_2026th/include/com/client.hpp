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
 * @file client.hpp
 *
 * @author Leo
 * @brief 上下位机串口通信协议(基于套接字转换)
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream> // 输入输出类
#include <stdint.h> // 整型数据类
#include <string>
#include <thread>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstdio>
#include <unistd.h>
#include <sys/types.h>

using namespace std;

class Client
{
private:
// USB通信帧
#define USB_FRAME_HEAD 0x42 // USB通信帧头
#define USB_FRAME_LENMIN 4  // USB通信帧最短字节长度
#define USB_FRAME_LENMAX 12 // USB通信帧最长字节长度

// USB通信地址
#define USB_ADDR_CARCTRL 1 // 智能车速度+方向控制
#define USB_ADDR_BUZZER 4  // 蜂鸣器音效控制
#define USB_ADDR_LED 5     // LED灯效控制
#define USB_ADDR_KEY 0x10  // 按键信息
    /**
     * @brief 串口通信结构体
     *
     */
    typedef struct
    {
        bool start;                           // 开始接收标志
        uint8_t index;                        // 接收序列
        uint8_t buffRead[USB_FRAME_LENMAX];   // 临时缓冲数据
        uint8_t buffFinish[USB_FRAME_LENMAX]; // 校验成功数据
    } SerialStruct;

    std::thread threadRes;  // 串口接收子线程
    std::string portName;   // 端口名字
    SerialStruct serialStr; // 串口通信数据结构体
    int socketId = 0;
    int countInit = 0; // 初始化计数器
    /**
     * @brief 32位数据内存对齐/联合体
     *
     */
    typedef union
    {
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
        uint8_t buff[2];
        int int16;
        uint16_t uint16;
    } Bit16Union;

public:
    // 定义构造函数
    Client() {};
    // 定义析构函数
    ~Client() { closeClient(); };
    bool keypress = false; // 按键

    /**
     * @brief 蜂鸣器音效
     *
     */
    enum Buzzer
    {
        BUZZER_OK = 0,   // 确认
        BUZZER_WARNNING, // 报警
        BUZZER_FINISH,   // 完成
        BUZZER_DING,     // 提示
        BUZZER_START,    // 开机
    };

    /**
     * @brief 启动套接字通信
     *
     * @return true
     * @return false
     */
    bool start(void)
    {
        socketId = socket(PF_INET, SOCK_STREAM, 0);
        if (socketId < 0)
        {
            cout << "socket init error!" << endl;
            return false;
        }
        struct sockaddr_in address;
        memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port = htons(8899);
        // 将IPv4地址从文本转换为二进制形式
        if (inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) <= 0)
        {
            std::cerr << "Invalid address/ Address not supported" << std::endl;
            return false;
        }

        if (connect(socketId, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            cout << "socket connect error!" << endl;
            return false;
        }

        // 启动数据接收子线程
        threadRes = std::thread([this]()
                                {
        char buffer[1024] = {0};
        while (1) {
        int len = read(socketId, buffer, 1024);

        std::string str(buffer);
        if (str.find("Keypress") != std::string::npos)//按键按下
            keypress = true;
      } });
        return true;
    }

    /**
     * @brief 关闭客户端
     *
     */
    void closeClient()
    {
        carControl(0, 1500); // 舵机PWM中值 1500
        close(socketId);
        threadRes.join();
    }

    /**
     * @brief U8转char
     *
     * @param str
     * @param UnChar
     * @param ucLen
     */
    void convertUnCharToStr(char *data, unsigned char *buff)
    {
        int len = sizeof(buff) / sizeof(buff[0]);
        for (int i = 0; i < len; i++)
        {
            // 格式化输str,每unsigned char 转换字符占两位置%x写输%X写输
            sprintf(data + i * 2, "%02x", buff[i]);
        }
    }

    /**
     * @brief 套接字发送数据
     *
     */
    void transmit(uint8_t *buff, int len)
    {
        fprintf(stderr, "[TX] ");
        for (int i = 0; i < len; i++)
            fprintf(stderr, "%02x ", buff[i]);
        fprintf(stderr, "\n");
        
        char data[len];
        memcpy(data, buff, len);
        send(socketId, data, len, 0);
    }


    /**
     * @brief 速度+方向控制
     *
     * @param speed 速度：m/s
     * @param servo 方向：PWM（500~2500）
     */
void carControl(float speed, uint16_t servo)
{
    countInit++;
    if (countInit >= 50)
        countInit = 50;
    else
        { // speed = 0.0;
        }

    uint16_t mcSpeed = (uint16_t)(speed * 100.0f);
    if (mcSpeed > 100) mcSpeed = 100;

    uint16_t mcServo;
    if (servo <= 1100) mcServo = 60;
    else if (servo >= 1900) mcServo = 120;
    else mcServo = 90 + (int16_t)(servo - 1500) * 30 / 400;

    fprintf(stderr, "[NEW] mcSpeed=%d mcServo=%d\n", mcSpeed, mcServo);

    uint8_t buff[6];
    buff[0] = 0xAA;
    buff[1] = mcSpeed & 0xFF;
    buff[2] = (mcSpeed >> 8) & 0xFF;
    buff[3] = (mcServo >> 8) & 0xFF;
    buff[4] = mcServo & 0xFF;
    buff[5] = 0xDD;

    transmit(buff, 6);
}

    /**
     * @brief 蜂鸣器音效控制
     *
     * @param sound
     */
    void buzzerSound(Buzzer sound)
    {
        uint8_t buff[6];   // 多发送一个字节
        uint8_t check = 0; // 校验位

        buff[0] = USB_FRAME_HEAD;  // 帧头
        buff[1] = USB_ADDR_BUZZER; // 地址
        buff[2] = 5;               // 帧长
        switch (sound)
        {
        case Buzzer::BUZZER_OK: // 确认
            buff[3] = 1;
            break;
        case Buzzer::BUZZER_WARNNING: // 报警
            buff[3] = 2;
            break;
        case Buzzer::BUZZER_FINISH: // 完成
            buff[3] = 3;
            break;
        case Buzzer::BUZZER_DING: // 提示
            buff[3] = 4;
            break;
        case Buzzer::BUZZER_START: // 开机
            buff[3] = 5;
            break;
        }

        for (size_t i = 0; i < 4; i++)
            check += buff[i];
        buff[4] = check;

        transmit(buff, 6); // 发送数据
    }

    /**
     * @brief 发送心跳信号
     *
     */
    void sendHeart()
    {
        uint8_t buff[3];   // 多发送一个字节
        transmit(buff, 3); // 发送数据
    }
};
