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
 * @file collection.cpp
 * @author Leo
 * @brief 遥控手柄操控
 * @version 0.1
 * @date 2024-01-09
 * @copyright Copyright (c) 2024
 * @note 采图步骤：
 *                  [01] 启动OpenCV摄像头图像捕获
 *                  [02] 创建遥控手柄多线程任务
 *                  [03] 车速度与方向控制
 *                  [04] 图像显示与存储
 */
#include "utils/uart.hpp" // 串口通信
#include "utils/tools.hpp"
#include <opencv2/opencv.hpp>  // OpenCV终端部署
#include <opencv2/highgui.hpp> //
#include <thread>              // 线程类
#include <signal.h>            //
#include <cstdlib>             // c++常用函数类
#include <fcntl.h>             //
#include <iostream>            // 输入输出类
#include <string>              // 字符串类
#include <sys/stat.h>          // 获取文件属性
#include <sys/types.h>         // 基本系统数据类型
#include <stdio.h>
#include <sstream>
#include <unistd.h>
#include <signal.h>

using namespace std;
using namespace cv;

class Joystick
{
private:
    const int JS_EVENT_BUTTON = 0x01; // 遥控手柄宏：按钮类型 按下/释放
    const int JS_EVENT_AXIS = 0x02;   // 遥控手柄宏：摇杆类型
    const int JS_EVENT_INIT = 0x80;   // 遥控手柄宏：设备初始状态
    /**
     * @brief 遥控手柄事件数据结构
     *
     */
    struct EventJoy
    {
        unsigned int time;    // 手柄触发时间：ms
        short value;          // 操控值
        unsigned char type;   // 操控类型：按键/摇杆
        unsigned char number; // 编号
    };
    struct EventJoy joy;
    int idFileJoy = 0;
    std::unique_ptr<std::thread> threadJoy; // 遥控手柄子线程

public:
    bool sampleMore = false;   // 连续图像采样使能
    bool sampleOnce = false;   // 单次图像采样使能
    float speed = 0;           // 车速：m/s
    float servo = PWMSERVOMID; // 打舵：PWM
    bool ahead = true;         // 车辆速度方向:默认向前
    bool uartSend = false;     // 串口发送使能
    bool buzzer = false;       // 提示音效
    bool enable = true;        // 遥控使能

    /**
     * @brief 解析函数
     *
     */
    Joystick(void)
    {
        // 遥控手柄启动
        idFileJoy = open("/dev/input/js0", O_RDONLY);
        joy.number = 0;
    };

    /**
     * @brief 遥控手柄线程关闭
     *
     */
    void close(void)
    {
        threadJoy->join();
        // close(idFileJoy);
    }

    /**
     * @brief 遥控手柄子线程启动
     *
     */
    void start(void)
    {
        threadJoy = std::make_unique<std::thread>([this]()
                                                  {
        while (1) 
        {
            threadJoystick();
        } });
    }

    /**
     * @brief 游戏手柄控制-多线程任务
     *
     */
    void threadJoystick(void)
    {
        read(idFileJoy, &joy, sizeof(joy));
        int type = JS_EVENT_BUTTON | JS_EVENT_INIT;

        if (joy.type == JS_EVENT_AXIS) // 摇杆
        {
            // cout << "AXIS: " << to_string(joy.number) << " | " << to_string(joy.value) << endl;
            switch (joy.number)
            {
            case 0: // 方向控制
                servo = PWMSERVOMID + joy.value * (PWMSERVOMID - PWMSERVOMIN) / 32767;
                uartSend = true;
                break;
            case 5: // 两档速度选择:慢速档
                if (joy.value >= 1)
                {
                    if (ahead)
                        speed = 0.3;
                    else
                        speed = -0.3;
                    uartSend = true;
                }
                else
                {
                    speed = 0.0;
                    uartSend = true;
                }
                break;
            }
        }
        else if (joy.type == JS_EVENT_BUTTON) // 按键
        {
            // cout << "BUTTON: " << to_string(joy.number) << " | " << to_string(joy.value) << endl;
            switch (joy.number)
            {
            case 5: // 两档速度选择: 高速档
                if (joy.value >= 1)
                {
                    if (ahead)
                        speed = 0.5;
                    else
                        speed = -0.5;
                    uartSend = true;
                }
                else
                {
                    speed = 0.0;
                    uartSend = true;
                }
                break;
            case 2: // 开始单次采图
                if (joy.value == 1)
                {
                    buzzer = true;     // 蜂鸣器音效
                    sampleOnce = true; // 开启单张采图使能
                }
                break;
            case 3: // 开始连续采图
                if (joy.value == 1)
                {
                    buzzer = true;     // 蜂鸣器音效
                    sampleMore = true; // 开启连续采图使能
                }
                break;
            case 0:                 // 停止采图
                if (joy.value == 1) // 关闭采图使能
                {
                    sampleMore = false;
                    sampleOnce = false;
                    buzzer = true; // 蜂鸣器音效
                }
                break;
            case 7:
                if (joy.value == 1) // 向前
                {
                    ahead = true;
                    if (speed < 0)
                    {
                        speed = -speed;
                        uartSend = true;
                    }
                    buzzer = true; // 蜂鸣器音效
                }
                break;
            case 6:
                if (joy.value == 1) // 向后
                {
                    ahead = false;
                    if (speed < 0)
                    {
                        speed = -speed;
                        uartSend = true;
                    }
                    buzzer = true; // 蜂鸣器音效
                }
                break;
            case 10:
                if (joy.value == 1) // 退出遥控手柄操控模式
                {
                    enable = false;
                    buzzer = true; // 蜂鸣器音效
                }
                break;
            default: // 任意键停止运动
                speed = 0;
                uartSend = true;
                break;
            }
        }
    }
};
