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
 * @file busy.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 避障（施工区）控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmBusy::FsmBusy(std::shared_ptr<Params> par)
    : FSMState(FsmMode::BUSY, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmBusy::~FsmBusy()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmBusy::getMode()
{
    if (enable && params->config.busy && params->config.currentLapConfig->busy)
    {
        if (manualTakeover)
        {
            return FsmMode::MANUAL;
        }
        else if (waitingForTakeover)
        {
            return FsmMode::BUSY_WAIT;
        }
        else
        {
            return FsmMode::BUSY;
        }
    }
    return FsmMode::NORMAL;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmBusy::run(Mat &img)
{
    if (!params->config.busy) // 该模式未启用
        return;

    // 帧计数器递增
    frameCount++;

    // 手动接管恢复期递减
    if (recoveryFrames > 0)
        recoveryFrames--;

    enable = false; // 场景检测使能标志

    // 行驶通过施工区模式（退出手动接管后）：左转标志触发退出转向，和出停车场一致
    if (drivingThrough)
    {
        if (!exiting)
        {
            // 施工区停靠模式：等待station完成停车后再检测左转
            bool stationActive = params->stationStarted && !params->stationStopCompleted;
            bool waitStationStop = false;
            if (params->config.currentLapConfig &&
                params->config.currentLapConfig->busyStopEnable &&
                params->config.currentLapConfig->busyStopPoint > 0)
            {
                if (!params->stationStopCompleted)
                {
                    waitStationStop = true; // 正在停车，等待
                }
                else if (params->config.currentLapConfig->busyStopPoint == 1 &&
                         stationExitCooldown < FIRST_STOP_EXIT_COOLDOWN_FRAMES)
                {
                    // 第一个框：停车后等待一段时间再检测左转
                    stationExitCooldown++;
                    waitStationStop = true;
                }
            }

            if (!stationActive && !waitStationStop)
            {
                // 等待检测左转标志，触发退出转向
                for (int i = 0; i < params->results.size(); i++)
                {
                    if (params->results[i].type == LABEL_LEFT &&
                        params->results[i].width < 100 &&
                        params->results[i].height < 120 &&
                        (params->results[i].y + params->results[i].height / 2) > ROWSIMAGE * 0.40)
                    {
                        exiting = true;
                        exitTimeout = 0;
                        countRes = 0;
                        printf("[Busy] Left sign detected, starting exit turn\n");
                        break;
                    }
                }
            }
        }

        if (exiting)
        {
            // 每帧重绘左转车道线（步骤[04]的赛道识别会覆盖，所以必须在runFsm中重新设置）
            exitTimeout++;
            params->track->pointsEdgeLeft.clear();
            params->track->pointsEdgeRight.clear();
            PointX startL(ROWSIMAGE - 10, 1);
            PointX endL(ROWSIMAGE / 3, 1);
            PointX midL((startL.x + endL.x) * 0.3, (startL.y + endL.y) / 2);
            params->track->pointsEdgeLeft = Bezier(0.008, {startL, midL, endL});
            PointX startR(ROWSIMAGE - 10, int(COLSIMAGE * 0.8));
            PointX endR(ROWSIMAGE / 3, 5);
            PointX midR((startR.x + endR.x) * 0.3, (startR.y + endR.y) / 2);
            params->track->pointsEdgeRight = Bezier(0.008, {startR, midR, endR});

            // 左转标志消失即完成退出
            bool leftVisible = false;
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_LEFT &&
                    params->results[i].width < 100 &&
                    params->results[i].height < 120)
                {
                    leftVisible = true;
                    break;
                }
            }
            if (leftVisible)
                countRes = 0;
            else
                countRes++;

            if (countRes > 2 || exitTimeout > 15)
            { // 标志丢失或超时
                drivingThrough = false;
                exiting = false;
                enable = false;
                countRes = 0;
                params->busyZone = false;  // 施工区结束
                params->ctrl.countAcc = 0; // 出库缓加速，约1.7s后恢复正常速度
                params->track->pointsEdgeLeft.clear();
                params->track->pointsEdgeRight.clear();
                printf("[Busy] Exit turn complete, returning to normal mode\n");
                return;
            }
        }
        enable = true; // 保持施工区模式
    }

    if (params->track->pointsEdgeLeft.size() < ROWSIMAGE / 2 ||
        params->track->pointsEdgeRight.size() < ROWSIMAGE / 2)
        return;

    // 检测施工区标志，触发停车和手动接管
    for (int i = 0; i < params->results.size(); i++)
    {
        if (params->results[i].type == LABEL_BUSY)
        {
            if (params->results[i].height < 130 && params->results[i].width < 100)
            {
                countRec++;
                countSes = 0; // 复位会话计数器，防止连续检测时意外清零
                timeout = 0;

                // 检测到施工区，立即停车
                if (countRec > 2)
                {
                    if (!drivingThrough)
                    { // 行驶通过模式不重新停车
                        params->ctrl.stop = true;
                        params->ctrl.speed = 0;
                        params->ctrl.servo = PWMSERVOMID; // 方向回中
                    }
                    cout << "[Busy] Construction zone detected! Stopping vehicle" << endl;
                    cout << "[Busy] countRec: " << countRec << ", enable: " << enable << endl;
                    enable = true;                       // 设置场景检测使能标志
                    if (!params->busyZone)               // 只在首次进入施工区时触发蜂鸣
                        params->busyAlertCountdown = 40; // 触发1秒以上蜂鸣
                    params->busyZone = true;             // 标记驶入施工区
                }

                // 持续检测，触发手动接管
                if (countRec > 3)
                {
                    cout << "[Busy] === 手动接管检查 ===" << endl;
                    cout << "[Busy] countRec: " << countRec << endl;
                    cout << "[Busy] waitingForTakeover: " << waitingForTakeover << endl;
                    cout << "[Busy] manualTakeover: " << manualTakeover << endl;
                    cout << "[Busy] currentLap: " << params->currentLap << endl;
                    cout << "[Busy] currentLapConfig: " << params->config.currentLapConfig << endl;
                    cout << "[Busy] &config.lap1: " << &params->config.lap1 << endl;
                    cout << "[Busy] &config.lap2: " << &params->config.lap2 << endl;
                    cout << "[Busy] &config.lap3: " << &params->config.lap3 << endl;
                    if (params->config.currentLapConfig)
                    {
                        cout << "[Busy] manualTakeover config: " << params->config.currentLapConfig->manualTakeover << endl;
                        cout << "[Busy] busy config: " << params->config.currentLapConfig->busy << endl;
                    }
                    else
                    {
                        cout << "[Busy] ERROR: currentLapConfig is NULL!" << endl;
                    }

                    if (params->config.currentLapConfig && params->config.currentLapConfig->manualTakeover)
                    {
                        if (!drivingThrough)
                        { // 行驶通过模式不重新触发接管
                            waitingForTakeover = false;
                            manualTakeover = true;
                            startManualTakeover(); // 启动手动接管
                            cout << "[Busy] Manual takeover triggered after construction zone detection" << endl;
                            cout << "[Busy] manualTakeover = " << manualTakeover << ", waitingForTakeover = " << waitingForTakeover << endl;
                        }
                    }
                    else
                    {
                        cout << "[Busy] Manual takeover NOT enabled in current lap config" << endl;
                        slowing = true; // 手动接管未启用时减速通过
                        timeout = 0;
                    }
                }
                break;
            }
        }
    }
    if (countRec > 0)
    {
        countSes++;
        if (countSes > 6)
            countRec = 0;
    }

    if (slowing)
    {
        timeout++;
        enable = true; // 场景检测使能标志
        if (timeout > 10)
            slowing = false;
    }

    // 手动接管模式下不执行自动控制
    if (manualTakeover)
    {
        return;
    }

    // 处理停靠区停车逻辑（只在非手动接管模式下）
    if (!manualTakeover)
    {
        // 只有启用了停车功能才检测停靠区
        if (params->config.currentLapConfig &&
            params->config.currentLapConfig->busyStopEnable &&
            params->config.currentLapConfig->busyStopPoint > 0)
        {
            handleParkingSpot();
        }
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmBusy::show(Mat &img)
{
    if (!enable || params->mode != FsmMode::BUSY)
        return;

    putText(img, "[1] Busy", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

/**
 * @brief 处理停靠区停车逻辑
 */
void FsmBusy::handleParkingSpot()
{
    // 如果未启用停车功能，直接返回
    if (!params->config.currentLapConfig->busyStopEnable)
    {
        return;
    }

    switch (parkingState)
    {
    case PARKING_IDLE:
        // 不处理
        break;

    case PARKING_DELAY_WAIT:
        // 等待3秒（假设30fps，即90帧）
        if (frameCount - parkingDelayStart >= 90)
        {
            parkingState = PARKING_ACTIVE;
            parkingStartTime = frameCount;
            parkingStationCount = 0;
            printf("[Busy] Delay timeout, starting parking at station spot %d\n", parkingTargetSpot);
        }
        break;

    case PARKING_ACTIVE:
        // 参考station.cpp的停车逻辑
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_STATION)
            {
                // 检测到停靠区，开始停车计时
                if (!params->stationStarted)
                {
                    params->stationStarted = true;
                    pressTimer = 1;
                    printf("[Busy] Station spot %d detected, starting stop sequence\n", parkingTargetSpot);
                }

                // 延时约0.3秒后执行停车
                if (pressTimer > 0)
                {
                    pressTimer++;
                    if (pressTimer > 9)
                    { // 0.3秒（9帧）
                        // 执行停车
                        params->ctrl.stop = true;
                        params->ctrl.speed = 0;
                        params->ctrl.servo = PWMSERVOMID;
                        stopCounter++;
                        printf("[Busy] Station stop %d/30\n", stopCounter);

                        // 停车约1秒（30帧）
                        if (stopCounter > 30)
                        {
                            printf("[Busy] Parking completed at station spot %d\n", parkingTargetSpot);
                            endParkingSequence();
                        }
                    }
                }
                break;
            }
        }
        break;

    case PARKING_COMPLETE:
        // 停车完成，等待一段时间后继续行驶
        if (parkingStationMaskTime > 0)
        {
            parkingStationMaskTime--;
        }
        else
        {
            // 重置状态，进入行驶通过模式
            parkingState = PARKING_IDLE;
            drivingThrough = true;
            immediateParking = false;
            delayedParking = false;
            // 重置停车相关变量
            params->stationStarted = false;
            pressTimer = 0;
            stopCounter = 0;
            printf("[Busy] Parking complete, continuing through construction zone\n");
        }
        break;
    }
}

/**
 * @brief 检测停靠区标志
 */
bool FsmBusy::isStationDetected()
{
    for (int i = 0; i < params->results.size(); i++)
    {
        if (params->results[i].type == LABEL_STATION)
        {
            // 检查停靠区标志的特征
            if (params->results[i].width > 60 && params->results[i].width < 150 &&
                params->results[i].height > 30 && params->results[i].height < 120)
            {
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief 结束停车序列
 */
void FsmBusy::endParkingSequence()
{
    parkingState = PARKING_COMPLETE;
    parkingStationMaskTime = 30; // 屏蔽1秒，避免重复检测
    parkingStationCount = 0;
    printf("[Busy] Parking sequence completed at station spot %d\n", parkingTargetSpot);
}

/**
 * @brief 缩减优化车道线（双车道→单车道）
 *
 * @param left
 */
/**
 * @brief 启动手动接管
 */
void FsmBusy::startManualTakeover()
{
    manualTakeover = true;
    waitingForTakeover = false;
    printf("[Busy] Starting manual takeover\n");
}

/**
 * @brief 结束手动接管
 */
void FsmBusy::endManualTakeover()
{
    manualTakeover = false;
    waitingForTakeover = false; // 改为false，不进入BUSY_WAIT
    enable = true;              // 保持在施工区模式
    countRec = 0;
    countSes = 0;
    busyStopCount = 0;
    busyStopMaskTime = 0;
    params->busyZone = true; // 标记施工区状态

    // 重置停车相关变量
    stationStarted = false;
    pressTimer = 0;
    stopCounter = 0;

    // 使用station的skip逻辑控制停靠点，busy只负责行驶通过模式
    drivingThrough = true;
    if (params->config.currentLapConfig &&
        params->config.currentLapConfig->busyStopEnable &&
        params->config.currentLapConfig->busyStopPoint > 0)
    {
        printf("[Busy] Station skip controlled, driving through (stopPoint=%d)\n",
               params->config.currentLapConfig->busyStopPoint);
    }
    else
    {
        printf("[Busy] No parking, driving through construction zone\n");
    }

    // 重置退出相关变量
    exiting = false;
    exitTimeout = 0;
    countRes = 0;
    stationExitCooldown = 0;
    printf("[Busy] Manual takeover ended\n");
}

void FsmBusy::resetLap()
{
    enable = false;
    manualTakeover = false;
    waitingForTakeover = true;
    countRec = 0;
    countSes = 0;
    timeout = 0;
    busyStopCount = 0;
    busyStopMaskTime = 0;
    recoveryFrames = 0;
    drivingThrough = false;
    exiting = false;
    exitTimeout = 0;
    countRes = 0;
    stationExitCooldown = 0;

    // 停靠区停车状态复位
    parkingState = PARKING_IDLE;
    immediateParking = false;
    delayedParking = false;
    parkingInProgress = false;
    parkingDelayStart = 0;
    parkingStartTime = 0;
    parkingTargetSpot = 0;
    parkingStationCount = 0;
    parkingStationMaskTime = 0;
    stationStarted = false;
    pressTimer = 0;
    stopCounter = 0;
    frameCount = 0;

    params->busyZone = false;
    params->stationStopCompleted = false;
}