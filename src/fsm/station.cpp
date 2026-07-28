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
 * @file station.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停靠站停车控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/station.hpp"

FsmStation::FsmStation(std::shared_ptr<Params> par)
    : FSMState(FsmMode::STATION, par)
{
}

FsmStation::~FsmStation()
{
}

FsmMode FsmStation::getMode()
{
    if (step != Step::NONE || pressTimer > 0)
        return FsmMode::STATION;
    else
        return FsmMode::NORMAL;
}

void FsmStation::run(Mat &img)
{
    // 手动接管刚结束，重置计数
    if (params->takeoverJustEnded)
    {
        params->takeoverJustEnded = false;
        stationBoxCounter = 0;
        stationBoxCounted = false;
        if (params->busyZone)
        {
            int target = params->config.currentLapConfig->busyStopPoint;
            if (target > 1)
            {
                busyEntryDelay = 60; // 跳过第一个框：等2秒再开始检测
                stationBoxCounter = target - 1;
            }
        }
        printf("[Station] Manual takeover ended, reset counters\n");
    }

    // 手动接管期间不做任何识别
    if (params->manualTakeover)
        return;

    // 当前圈未启用station
    if (!params->config.station || !params->config.currentLapConfig->station)
        return;

    countInit++;
    if (countInit > 999)
        countInit = 999;
    else if (countInit < 60) // 发车屏蔽
        return;

    switch (step)
    {
    case Step::NONE:
    {
        // 施工区进入后等4秒再开始检测（让车走过前面N个框）
        if (params->busyZone && busyEntryDelay > 0)
        {
            busyEntryDelay--;
            break;
        }

        // 施工区未启用停车或busyStopPoint为0时跳过检测
        if (params->busyZone && (!params->config.currentLapConfig->busyStopEnable ||
                                 params->config.currentLapConfig->busyStopPoint == 0))
            break;

        // 非施工区复位跳过计数
        if (!params->busyZone)
        {
            stationBoxCounter = 0;
            stationBoxCounted = false;
        }

        // 停车后冷却期内不检测
        if (cooldown > 0)
        {
            cooldown--;
            break;
        }

        if (pressTimer > 0)
        {
            pressTimer++;
            // 施工区第一个框1.3s(40帧)，第二个目标框0.6s(20帧)，左岔路1.1s(33帧)，其他0.6s(19帧)
            int pressThreshold = 19;
            if (params->busyZone && stationBoxCounter == 0)
                pressThreshold = 40;
            if (params->busyZone && params->config.currentLapConfig->busyStopPoint > 1 &&
                stationBoxCounter >= params->config.currentLapConfig->busyStopPoint - 1)
                pressThreshold = 20;
            if (params->yforkBranch == 1)
                pressThreshold = 33;
            if (pressTimer > pressThreshold)
            {
                printf("[Station] Pressed +%.1fs, stopping...\n", pressThreshold / 30.0f);
                setStep(Step::STOP);
            }
            break;
        }

        // 左岔路：引导结束见框开始计时（框过0.5后延迟N帧再停）
        if (params->yforkBranch == 1)
        {
            // 引导期间不检测
            if (params->yforkGuiding)
                break;

            bool boxSeen = false;
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_STATION)
                {
                    int boxCx = params->results[i].x + params->results[i].width / 2;
                    int boxBottom = params->results[i].y + params->results[i].height;
                    if (boxCx < COLSIMAGE / 2 && boxBottom > ROWSIMAGE * 0.3)
                    {
                        boxSeen = true;
                        break;
                    }
                }
            }

            if (boxSeen)
            {
                leftBranchDelay++;
                if (leftBranchDelay >= LEFT_BRANCH_DELAY_FRAMES)
                {
                    params->stationStarted = true;
                    pressTimer = 1;
                    leftBranchDelay = 0;
                    printf("[Station] Left branch, delay=%d done, 0.3s stop\n", LEFT_BRANCH_DELAY_FRAMES);
                }
            }
        }
        else
        {
            // yfork引导期间不检测station框，避免干扰岔路导航
            if (params->yforkGuiding)
                break;

            // 非左岔路：框到底部才检测
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_STATION)
                {
                    // 第一个框在中部检测，第二个框到底部才触发
                    int boxBottom = params->results[i].y + params->results[i].height;
                    bool isFirstBox = params->busyZone && !params->stationStopCompleted &&
                                      (params->config.currentLapConfig->busyStopPoint == 1 ||
                                       (params->config.currentLapConfig->busyStopPoint > 1 &&
                                        stationBoxCounter < params->config.currentLapConfig->busyStopPoint - 1));
                    int threshold = isFirstBox
                                        ? static_cast<int>(ROWSIMAGE * 0.5)
                                        : ROWSIMAGE - 10;
                    if (boxBottom > threshold)
                    {
                        // 施工区已停过，不再重复检测
                        if (params->busyZone && params->stationStopCompleted)
                            break;

                        // 施工区跳过前N个框
                        if (params->busyZone)
                        {
                            // 当前框已计过数，跳过本次检测
                            if (stationBoxCounted)
                                break;

                            int targetBox = params->config.currentLapConfig->busyStopPoint;
                            if (targetBox > 1 && stationBoxCounter < targetBox - 1)
                            {
                                stationBoxCounter++;
                                stationBoxCounted = true;
                                params->stationStarted = true;
                                params->stationStopCompleted = false;
                                printf("[Station] Skip box #%d (target=%d)\n", stationBoxCounter, targetBox);
                                break;
                            }
                        }

                        params->stationStarted = true;
                        pressTimer = 1;
                        printf("[Station] Pressed\n");
                        break;
                    }
                }
            }
        }
        break;
    }

    case Step::STOP:
    {
        params->ctrl.stop = true;
        stopCounter++;
        printf("[Station] Stop %d/30\n", stopCounter);
        if (stopCounter > 30) // 停车约1秒
        {
            printf("[Station] Stop end, resume\n");
            params->stationStopCompleted = true; // 通知yfork边线突变可以退出了
            setStep(Step::NONE);
            cooldown = params->busyZone ? 6 : 150; // 施工区0.2秒冷却，其他5秒
        }
        break;
    }
    }
}

void FsmStation::show(Mat &img)
{
    if (params->mode != FsmMode::STATION)
        return;

    putText(img, "[10] Station", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    switch (step)
    {
    case Step::STOP:
        putText(img, "[10] Station - STOP", Point(100, 50),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;
    }
}

void FsmStation::setStep(Step st)
{
    step = st;
    stopCounter = 0;
    pressTimer = 0;
    leftBranchDelay = 0;
    params->ctrl.stop = false;
}

void FsmStation::resetLap()
{
    setStep(Step::NONE);
    countInit = 0;
    cooldown = 0;
    stationBoxCounter = 0;
    stationBoxCounted = false;
    busyEntryDelay = 0;
    params->stationStopCompleted = false;
    params->stationStarted = false;
}
