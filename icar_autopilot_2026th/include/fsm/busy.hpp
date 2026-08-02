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
 * @file busy.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 避障（施工区）控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 避障（施工区）控制
 *
 */
class FsmBusy : public FSMState
{
public:
    FsmBusy(std::shared_ptr<Params> par);
    ~FsmBusy();
    void run(Mat &img);
    void show(Mat &img);
    FsmMode getMode();
    void resetLap();
    bool slowing = false; // 减速使能

    // 手动接管相关方法
    void startManualTakeover();
    void endManualTakeover();
    bool isInManualTakeover() { return manualTakeover; }

private:
    bool enable = false;            // 场景检测使能标志
    bool manualTakeover = false;    // 手动接管标志
    bool waitingForTakeover = true; // 等待手动接管标志
    int countRec = 0;               // AI场景识别计数器
    int countSes = 0;               // 场次计数器
    int timeout = 0;                // 超时计数器
    int busyStopCount = 0;          // 停靠点计数器
    int busyStopMaskTime = 0;       // 停靠点时间屏蔽
    int recoveryFrames = 0;         // 手动接管后的恢复帧计数（防止重新触发接管）
    bool drivingThrough = false;    // 退出手动接管后继续在施工区模式行驶，等待左拐退出
    bool exiting = false;           // 退出转向中（看到左转标志后重绘车道线引导转向）
    int exitTimeout = 0;            // 退出转向超时计数器
    int countRes = 0;               // 场景识别计数器（退出转向标志丢失计数）
    static constexpr int LEFT_EXIT_CONFIRM_FRAMES = 2; // 左转标连续满足位置条件的帧数
    int leftExitConfirmFrames = 0;
    static constexpr int FIRST_STOP_EXIT_COOLDOWN_FRAMES = 10; // 第一个框停车后等待2秒再检测左转
    int stationExitCooldown = 0;

    // 停靠区停车状态管理
    enum ParkingState {
        PARKING_IDLE,
        PARKING_DELAY_WAIT,     // 等待延迟
        PARKING_ACTIVE,        // 正在停车
        PARKING_COMPLETE       // 停车完成
    } parkingState = PARKING_IDLE;

    // 停靠区停车相关变量
    bool immediateParking = false;      // 立即停车
    bool delayedParking = false;        // 延迟停车
    bool parkingInProgress = false;     // 停车进行中
    int parkingDelayStart = 0;         // 延迟开始时间（帧数）
    int parkingStartTime = 0;          // 停车开始时间（帧数）
    int parkingTargetSpot = 0;         // 停靠目标（1或2）
    int parkingStationCount = 0;       // 停靠区检测计数
    int parkingStationMaskTime = 0;    // 停靠区屏蔽时间

    // 停车控制变量（参考station.cpp）
    bool stationStarted = false;  // 停靠区检测开始标志
    int pressTimer = 0;          // 按下计时器
    int stopCounter = 0;         // 停车计数器

    void handleParkingSpot();
    bool isStationDetected();
    void endParkingSequence();

private:
    int frameCount = 0;  // 帧计数器
};
