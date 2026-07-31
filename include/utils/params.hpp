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
 * @file state.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 机器人状态信息
 * @version 0.1
 * @date 2025-04-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <string>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <fstream>
#include "utils/json.hpp"
#include "ctrl/track.hpp"

using namespace std;

/**
 * @brief FSM状态场景
 *
 */
enum class FsmMode
{
    NORMAL,    // 基础赛道
    FORK,      // 岔路
    PARK,      // 停车场
    BUSY,      // 施工障碍
    BUSY_WAIT, // 等待手动接管
    MANUAL,    // 手动接管模式
    SLOW,      // 慢行区
    CURVE,     // 连续弯道
    FINE,      // 禁行区
    STOP,      // 停车区
    CROSS,     // 斑马线
    YFORK,     // Y型岔路口
    STATION,   // 停靠站
};

/**
 * @brief 车辆控制指令
 *
 */
struct Control
{
    bool stop = false;            // 车辆停止运动
    bool back = false;            // 倒车
    bool slow = false;            // 车辆减速
    uint16_t servo = PWMSERVOMID; // 发送给舵机的PWM
    float speed = 0.0;            // 发送给电机的速度
    int center = COLSIMAGE / 2;   // 控制中心
    vector<PointX> centerEdge;    // 赛道中心点集
    int lineArea = 0;             // 面积规划行序号
    bool fitting = false;         // 控制中心拟合标志(停车场专用)
    bool parking = false;         // 停车场特殊模式
    int countAcc = 500;           // 缓加速计数器
    int outlineCooldown = 0;      // outlineCheck冷却计数器（停车场出库后暂时禁用）
    bool yforkReset = false;      // Y型岔路复位标志（park退出时设置）
    bool busyCrossSlow = false;   // busy圈斑马线消失后减速到0.3
};
/**
 * @brief 控制器核心参数
 *
 */
struct Config
{
    // 通用配置参数
    float velLow = 1.3;                                 // 智能车最低速:m/s
    float velHigh = 1.3;                                // 智能车最高速:m/s
    float velSlow = 0.5;                                // 慢性区速度:m/s
    float velPark = 0.6;                                // 充电站车速
    float velCurve = 1.0;                               // 连续弯道速度:m/s
    float velBusy = 0.8;                                // 施工区速度:m/s
    float velStop = 0.7;                                // 停车区速度: m/s
    float velCross = 0.7;                               // 斑马线速度: m/s
    float velYfork = 0.7;                               // Y型岔路口速度: m/s
    float runP1 = 2.2;                                  // 比例系数：直线控制量
    float runP2 = 0.007;                                // 动态P变化系数
    float turnP = 3.5;                                  // 比例系数：转弯控制量
    float turnD = 3.5;                                  // 微分系数：转弯控制量
    bool debug = false;                                 // 调试模式使能
    bool saveImg = false;                               // 存图使能
    bool saveIpm = false;                               // 存储IPM图像
    uint16_t rowCutUp = 10;                             // 图像顶部切行
    uint16_t rowCutBottom = 20;                         // 图像底部切行
    float overlap = 0.3;                                // 智能车与车道线重合度(%)
    float score = 0.2;                                  // AI检测置信度
    int binary = -1;                                    // 图像二值化阈值
    string model = "../res/models/yolov3_mobilenet_v1"; // 模型路径
    string video = "../res/samples/sample.mp4";         // 视频路径
    string alertTarget = "none";                        // 蜂鸣器报警目标: "none"/"cone"/"person"

    // 圈数配置
    int totalLaps = 3; // 总圈数

    // 每圈功能使能配置
    struct LapConfig
    {
        bool fork = false;
        bool fine = false;
        bool park = false;
        bool spot = false;
        bool curve = false;
        bool busy = false;
        bool slow = false;
        bool stop = false;
        bool cross = true;
        bool yfork = false;
        bool yforkLeft = true; // Y型岔路口走左分支(true)或右分支(false)
        bool station = true;   // 停靠站停车
        bool obstacle = true;   // 障碍物避障使能（锥桶/行人）
        int parkSpot = 0;
        // parkSpot指定的车位为目标车位，其余三个车位自动视为已占用

        // 施工区手动接管相关配置
        bool manualTakeover = false; // 是否启用手动接管
        bool busyStopEnable = false; // 是否在施工区停车
        int busyStopPoint = 0;       // 施工区停靠点选择 (0-不停车, 1-第一个停靠点, 2-第二个停靠点)
    };

    LapConfig lap1; // 第一圈配置
    LapConfig lap2; // 第二圈配置
    LapConfig lap3; // 第三圈配置

    // 当前圈配置指针
    LapConfig *currentLapConfig = nullptr;

    // 全局功能使能（默认值，可被单圈配置覆盖）
    bool fork = true;    // 岔路使能
    bool fine = true;    // 禁行区使能
    bool park = true;    // 停车场使能
    bool spot = true;    // 停车位使能
    bool curve = true;   // 连续弯道使能
    bool busy = true;    // 施工区使能
    bool slow = true;    // 慢行区使能
    bool stop = true;    // 停车区使能
    bool cross = true;   // 斑马线停车使能
    bool yfork = true;   // Y型岔路口使能
    bool station = true; // 停靠站停车使能
    bool obstacle = true; // 障碍物避障使能（锥桶/行人）

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Config, velLow, velHigh, velSlow, velPark, velCurve, velBusy, velStop, velCross, velYfork,
                                   runP1, runP2, turnP, turnD, debug, saveImg, saveIpm, rowCutUp, rowCutBottom,
                                   overlap, score, binary, model, video, alertTarget, totalLaps, fork, fine, park, spot, curve, busy, slow, stop, cross, yfork, station);
};

/**
 * @brief 车辆状态参数（FSM共享传递）
 *
 */
struct Params
{
public:
    /**
     * @brief Construct a new Params object
     *
     */
    Params()
    {
        // 加载本地json配置文件
        string path = "../res/config.json";
        std::ifstream fileStr(path);
        if (!fileStr.good())
        {
            std::cout << "Error: Params file path:[" << path << "] not find !!!" << std::endl;
            exit(-1);
        }
        nlohmann::json configs;
        fileStr >> configs;
        try
        {
            // 加载基础配置
            config.velLow = configs["通用配置参数"]["velLow"];
            config.velHigh = configs["通用配置参数"]["velHigh"];
            config.velSlow = configs["通用配置参数"]["velSlow"];
            config.velPark = configs["通用配置参数"]["velPark"];
            config.velCurve = configs["通用配置参数"]["velCurve"];
            config.velBusy = configs["通用配置参数"]["velBusy"];
            config.velStop = configs["通用配置参数"]["velStop"];
            config.velCross = configs["通用配置参数"]["velCross"];
            config.velYfork = configs["通用配置参数"].value("velYfork", 0.7);
            config.runP1 = configs["通用配置参数"]["runP1"];
            config.runP2 = configs["通用配置参数"]["runP2"];
            config.turnP = configs["通用配置参数"]["turnP"];
            config.turnD = configs["通用配置参数"]["turnD"];
            config.debug = configs["通用配置参数"]["debug"];
            config.saveImg = configs["通用配置参数"]["saveImg"];
            config.saveIpm = configs["通用配置参数"]["saveIpm"];
            config.rowCutUp = configs["通用配置参数"]["rowCutUp"];
            config.rowCutBottom = configs["通用配置参数"]["rowCutBottom"];
            config.overlap = configs["通用配置参数"]["overlap"];
            config.score = configs["通用配置参数"]["score"];
            config.binary = configs["通用配置参数"]["binary"];
            config.model = configs["通用配置参数"]["model"];
            config.video = configs["通用配置参数"]["video"];
            config.alertTarget = configs["通用配置参数"].value("alertTarget", "none");
            config.totalLaps = configs["圈数配置"]["totalLaps"];

            // 加载crossStop: 第几次见cross停车totalLaps
            crossStop = configs["圈数配置"].value("crossStop", 0);

            // 加载全局功能使能
            auto globalFeatures = configs["全局功能使能"];
            config.fork = globalFeatures["fork"];
            config.fine = globalFeatures["fine"];
            config.park = globalFeatures["park"];
            config.spot = globalFeatures.value("spot", true);
            config.curve = globalFeatures["curve"];
            config.busy = globalFeatures["busy"];
            config.slow = globalFeatures["slow"];
            config.stop = globalFeatures["stop"];
            config.cross = globalFeatures["cross"];
            config.yfork = globalFeatures["yfork"];
            config.station = globalFeatures["station"];
            config.obstacle = globalFeatures.value("obstacle", true);

            // 加载每圈配置
            auto lap1Config = configs["每圈功能使能配置"]["lap1"];
            config.lap1.fork = lap1Config["fork"];
            config.lap1.fine = lap1Config["fine"];
            config.lap1.park = lap1Config["park"];
            config.lap1.parkSpot = lap1Config["parkSpot"];
            config.lap1.spot = lap1Config.value("spot", true);
            config.lap1.curve = lap1Config["curve"];
            config.lap1.busy = lap1Config["busy"];
            config.lap1.slow = lap1Config["slow"];
            config.lap1.stop = lap1Config["stop"];
            config.lap1.cross = lap1Config["cross"];
            config.lap1.yfork = lap1Config["yfork"];
            config.lap1.yforkLeft = lap1Config.value("yforkLeft", true);
            config.lap1.station = lap1Config["station"];
            config.lap1.obstacle = lap1Config.value("obstacle", true);
            config.lap1.manualTakeover = lap1Config.value("manualTakeover", false);
            config.lap1.busyStopEnable = lap1Config.value("busyStopEnable", false);
            config.lap1.busyStopPoint = lap1Config.value("busyStopPoint", 0);

            auto lap2Config = configs["每圈功能使能配置"]["lap2"];
            config.lap2.fork = lap2Config["fork"];
            config.lap2.fine = lap2Config["fine"];
            config.lap2.park = lap2Config["park"];
            config.lap2.parkSpot = lap2Config["parkSpot"];
            config.lap2.spot = lap2Config.value("spot", true);
            config.lap2.curve = lap2Config["curve"];
            config.lap2.busy = lap2Config["busy"];
            config.lap2.slow = lap2Config["slow"];
            config.lap2.stop = lap2Config["stop"];
            config.lap2.cross = lap2Config["cross"];
            config.lap2.yfork = lap2Config["yfork"];
            config.lap2.yforkLeft = lap2Config.value("yforkLeft", true);
            config.lap2.station = lap2Config["station"];
            config.lap2.obstacle = lap2Config.value("obstacle", true);
            config.lap2.manualTakeover = lap2Config.value("manualTakeover", false);
            config.lap2.busyStopEnable = lap2Config.value("busyStopEnable", false);
            config.lap2.busyStopPoint = lap2Config.value("busyStopPoint", 0);

            auto lap3Config = configs["每圈功能使能配置"]["lap3"];
            config.lap3.fork = lap3Config["fork"];
            config.lap3.fine = lap3Config["fine"];
            config.lap3.park = lap3Config["park"];
            config.lap3.parkSpot = lap3Config["parkSpot"];
            config.lap3.spot = lap3Config.value("spot", true);
            config.lap3.curve = lap3Config["curve"];
            config.lap3.busy = lap3Config["busy"];
            config.lap3.slow = lap3Config["slow"];
            config.lap3.stop = lap3Config["stop"];
            config.lap3.cross = lap3Config["cross"];
            config.lap3.yfork = lap3Config["yfork"];
            config.lap3.yforkLeft = lap3Config.value("yforkLeft", true);
            config.lap3.station = lap3Config["station"];
            config.lap3.obstacle = lap3Config.value("obstacle", true);
            config.lap3.manualTakeover = lap3Config.value("manualTakeover", false);
            config.lap3.busyStopEnable = lap3Config.value("busyStopEnable", false);
            config.lap3.busyStopPoint = lap3Config.value("busyStopPoint", 0);
        }
        catch (const nlohmann::detail::exception &e)
        {
            std::cerr << "Json Params Parse failed :" << e.what() << std::endl;
            exit(-1);
        }

        mode = FsmMode::NORMAL;                    // 初始化控制模式
        modeLast = FsmMode::NORMAL;                // 初始化控制模式
        track = make_shared<Track>();              // 赛道线处理
        track->rowCutUp = config.rowCutUp;         // 图像顶部切行（前瞻距离）
        track->rowCutBottom = config.rowCutBottom; // 图像底部切行（盲区距离）

        // 初始化圈数和斑马线通过状态
        totalLaps = config.totalLaps;
        currentLap = 1;
        crossPassed = true; // 初始true，第一次检测到不算，需先消失再出现才计一次经过

        // 初始化第一圈配置
        updateLapConfig();
    };
    ~Params() {};

    Control ctrl;                       // 车辆控制指令(实时)
    Config config;                      // 系统配置
    FsmMode mode, modeLast;             // FSM状态场景
    shared_ptr<Track> track;            // 赛道识别类
    std::vector<PredictResult> results; // AI推理结果
    int totalLaps;                      // 总圈数
    int currentLap;                     // 当前圈数
    int crossStop;                      // 第几次检测到cross停车
    bool crossPassed;                   // 是否已经通过斑马线（每圈计数）
    bool manualTakeover = false;        // 手动接管模式（禁用出界exit）
    bool stationStopCompleted = false;  // station已完成一次停车
    bool stationStarted = false;        // station已触发检测（pressTimer启动）
    bool yforkGuiding = false;          // yfork正在强制引导转弯（期间station不检测）
    int yforkBranch = 0;                // yfork分支：0=无, 1=左, 2=右
    bool busyZone = false;              // 施工区标志（station据此调整检测参数）
    bool takeoverJustEnded = false;     // 手动接管刚结束（station据此复位计数）
    int alertCountdown = 0;             // 蜂鸣器报警倒计时（帧数）
    int alertDecelCount = 0;            // 报警目标减速倒计时（帧数）
    int busyAlertCountdown = 0;         // 施工区蜂鸣器倒计时（帧数）

public:
    /**
     * @brief 更新当前圈配置
     */
    void updateLapConfig()
    {
        switch (currentLap)
        {
        case 1:
            config.currentLapConfig = &config.lap1;
            break;
        case 2:
            config.currentLapConfig = &config.lap2;
            break;
        case 3:
            config.currentLapConfig = &config.lap3;
            break;
        default:
            config.currentLapConfig = nullptr;
            break;
        }
    }

    /**
     * @brief 切换到下一圈
     */
    void nextLap()
    {
        if (currentLap < totalLaps)
        {
            currentLap++;
            updateLapConfig();
        }
    }

private:
};