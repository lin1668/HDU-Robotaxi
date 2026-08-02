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
#include "utils/yfork_diag.hpp"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <string>

static void busylog(const char *fmt, ...)
{
    static FILE *fp = nullptr;
    if (!fp)
        fp = fopen("./busy.log", "w");
    if (!fp)
        return;

    time_t now = time(nullptr);
    struct tm *t = localtime(&now);
    fprintf(fp, "[%02d:%02d:%02d] ", t->tm_hour, t->tm_min, t->tm_sec);

    va_list args;
    va_start(args, fmt);
    vfprintf(fp, fmt, args);
    va_end(args);
    fprintf(fp, "\n");
    fflush(fp);
}

FsmStation::FsmStation(std::shared_ptr<Params> par)
    : FSMState(FsmMode::STATION, par)
{
    busylog("BUSY LOG START");
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
        busyBoxLockFrames = 0;
        if (params->busyZone && params->config.currentLapConfig)
        {
            int target = params->config.currentLapConfig->busyStopPoint;
            if (target > 1)
            {
                busyEntryDelay = Tune::BUSY_ENTRY_DELAY_FRAMES; // 跳过第一个框：延迟一段时间再开始检测
                //stationBoxCounter = target - 1;（此段注释掉，避免无法在第二个框停车）
            }
        }
        printf("[Station] Manual takeover ended, reset counters\n");
        busylog("TAKEOVER_END busy=%d target=%d entryDelay=%d boxCounter=%d counted=%d branch=%d guiding=%d",
                params->busyZone,
                params->config.currentLapConfig ? params->config.currentLapConfig->busyStopPoint : -1,
                busyEntryDelay, stationBoxCounter, stationBoxCounted,
                params->yforkBranch, params->yforkGuiding);
    }

    // 手动接管期间不做任何识别
    if (params->manualTakeover)
    {
        static int manualLogCounter = 0;
        if (params->busyZone && manualLogCounter++ % 30 == 0)
            busylog("BLOCKED manualTakeover=1 branch=%d guiding=%d results=%d",
                    params->yforkBranch, params->yforkGuiding, (int)params->results.size());
        return;
    }

    // 当前圈未启用station
    if (!params->config.station || !params->config.currentLapConfig || !params->config.currentLapConfig->station)
    {
        static int stationDisabledLogCounter = 0;
        if (params->busyZone && stationDisabledLogCounter++ % 30 == 0)
            busylog("BLOCK stationDisabled global=%d lapConfig=%d lap=%d boxes=%d",
                    params->config.station, params->config.currentLapConfig != nullptr,
                    params->config.currentLapConfig ? params->config.currentLapConfig->station : 0,
                    (int)params->results.size());
        return;
    }

    // 停车场流程拥有停车、倒车和轨迹回放的全部控制权。Station 若在此期间
    // 继续运行，会把 ctrl.stop 写回 true，覆盖 Park::EXIT 写入的 ctrl.back。
    // 这里只复位 Station 自身状态，绝不能调用 setStep()/resetLap()，因为它们
    // 会修改 Park 当前帧写入的 ctrl.stop。
    if (params->ctrl.parking)
    {
        if (step != Step::NONE || pressTimer > 0 || params->stationStarted)
        {
            yforkDiagLog("STATION_EVENT",
                         "PAUSE reason=park_active step=%d press=%d stopHold=%d ctrlStop=%d back=%d",
                         static_cast<int>(step), pressTimer, stopCounter,
                         params->ctrl.stop, params->ctrl.back);
        }

        step = Step::NONE;
        stopCounter = 0;
        pressTimer = 0;
        cooldown = 0;
        stationBoxCounter = 0;
        stationBoxCounted = false;
        busyBoxLockFrames = 0;
        busyEntryDelay = 0;
        leftBranchDelay = 0;
        params->stationStarted = false;
        params->stationStopCompleted = false;
        return;
    }


    countInit++;
    if (countInit > 999)
        countInit = 999;
    else if (countInit < Tune::STARTUP_IGNORE_FRAMES) // 发车屏蔽
        return;

    // YFork相关停车链每帧写入同一个yfork.log。这里记录的是Station真正
    // 收到的检测结果和内部计数，能区分未识别、阈值未过、被引导屏蔽和计时阶段。
    if (params->yforkBranch != 0 || params->stationStarted || params->stationStopCompleted)
    {
        std::string stationBoxes;
        int stationCountDiag = 0;
        int maxBottomDiag = -1;
        for (size_t i = 0; i < params->results.size(); ++i)
        {
            const auto &r = params->results[i];
            if (r.type != LABEL_STATION)
                continue;
            if (!stationBoxes.empty())
                stationBoxes += ";";
            stationBoxes += "i=" + std::to_string(i) +
                            ",x=" + std::to_string(r.x) +
                            ",y=" + std::to_string(r.y) +
                            ",w=" + std::to_string(r.width) +
                            ",h=" + std::to_string(r.height) +
                            ",bottom=" + std::to_string(r.y + r.height);
            stationCountDiag++;
            maxBottomDiag = std::max(maxBottomDiag, r.y + r.height);
        }
        if (stationBoxes.empty())
            stationBoxes = "none";

        int thresholdDiag = -1;
        if (params->yforkBranch == 1)
            thresholdDiag = static_cast<int>(ROWSIMAGE * Tune::LEFT_BRANCH_TRIGGER_RATIO);
        else if (params->yforkBranch == 2)
            thresholdDiag = ROWSIMAGE - Tune::RIGHT_BRANCH_TRIGGER_BOTTOM_MARGIN;

        yforkDiagLog(
            "STATION_FRAME",
            "lap=%d mode=%d step=%d branch=%d guiding=%d busy=%d target=%d "
            "results=%zu stationBoxes=%d maxBottom=%d threshold=%d pass=%d "
            "entryDelay=%d leftDelay=%d press=%d stopHold=%d "
            "boxCounter=%d counted=%d started=%d completed=%d cooldown=%d "
            "ctrlStop=%d speed=%.3f boxes=[%s]",
            params->currentLap, static_cast<int>(params->mode), static_cast<int>(step),
            params->yforkBranch, params->yforkGuiding, params->busyZone,
            params->config.currentLapConfig ? params->config.currentLapConfig->busyStopPoint : -1,
            params->results.size(), stationCountDiag, maxBottomDiag, thresholdDiag,
            thresholdDiag >= 0 && maxBottomDiag > thresholdDiag,
            busyEntryDelay, leftBranchDelay, pressTimer, stopCounter,
            stationBoxCounter, stationBoxCounted, params->stationStarted,
            params->stationStopCompleted, cooldown, params->ctrl.stop,
            params->ctrl.speed, stationBoxes.c_str());
    }

    switch (step)
    {
    case Step::NONE:
    {
        static int busyFrameLogCounter = 0;
        int busyTarget = params->config.currentLapConfig->busyStopPoint;
        int stationCount = 0;
        int maxBottom = -1;
        for (const auto &result : params->results)
        {
            if (result.type == LABEL_STATION)
            {
                stationCount++;
                int bottom = result.y + result.height;
                if (bottom > maxBottom)
                    maxBottom = bottom;
            }
        }
        int diagThreshold = -1;
        if (params->busyZone)
        {
            if (params->yforkBranch == 1)
                diagThreshold = static_cast<int>(ROWSIMAGE * Tune::LEFT_BRANCH_TRIGGER_RATIO);
            else if (params->yforkBranch == 2)
                diagThreshold = ROWSIMAGE - Tune::RIGHT_BRANCH_TRIGGER_BOTTOM_MARGIN;
            else
            {
                bool isFirstBox = !params->stationStopCompleted &&
                                  (busyTarget == 1 || (busyTarget > 1 && stationBoxCounter < busyTarget - 1));
                diagThreshold = isFirstBox
                                    ? static_cast<int>(ROWSIMAGE * Tune::BUSY_FIRST_BOX_RATIO)
                                    : ROWSIMAGE - Tune::BUSY_TARGET_BOTTOM_MARGIN;
            }
        }
        if (params->busyZone && (stationCount > 0 || pressTimer > 0 || busyEntryDelay > 0 || cooldown > 0) &&
            busyFrameLogCounter++ % 15 == 0)
        {
            busylog("FRAME target=%d boxes=%d maxBottom=%d threshold=%d pass=%d counter=%d counted=%d entryDelay=%d cooldown=%d press=%d completed=%d branch=%d guiding=%d ctrlStop=%d speed=%.2f",
                    busyTarget, stationCount, maxBottom, diagThreshold,
                    diagThreshold >= 0 && maxBottom > diagThreshold,
                    stationBoxCounter, stationBoxCounted, busyEntryDelay, cooldown, pressTimer,
                    params->stationStopCompleted, params->yforkBranch, params->yforkGuiding,
                    params->ctrl.stop, params->ctrl.speed);
        }

        // 施工区进入后延迟检测（让车走过前面的框）
        if (params->busyZone && busyEntryDelay > 0)
        {
            if (busyEntryDelay == Tune::BUSY_ENTRY_DELAY_FRAMES || busyEntryDelay == 1)
                busylog("BLOCKED entryDelay=%d target=%d boxCounter=%d",
                        busyEntryDelay, params->config.currentLapConfig->busyStopPoint, stationBoxCounter);
            busyEntryDelay--;
            break;
        }

        // 施工区未启用停车或busyStopPoint为0时跳过检测
        if (params->busyZone && (!params->config.currentLapConfig->busyStopEnable ||
                                 params->config.currentLapConfig->busyStopPoint == 0))
        {
            static int disabledLogCounter = 0;
            if (disabledLogCounter++ % 30 == 0)
                busylog("BLOCKED busyStopEnable=%d target=%d",
                        params->config.currentLapConfig->busyStopEnable,
                        params->config.currentLapConfig->busyStopPoint);
            break;
        }

        // 非施工区复位跳过计数
        if (!params->busyZone)
        {
            stationBoxCounter = 0;
            stationBoxCounted = false;
            busyBoxLockFrames = 0;
        }

        // 停车后冷却期内不检测
        if (cooldown > 0)
        {
            if (params->busyZone && (cooldown == Tune::BUSY_COOLDOWN_FRAMES || cooldown == 1))
                busylog("BLOCK cooldown=%d target=%d counter=%d boxes=%d ctrlStop=%d speed=%.2f",
                        cooldown, busyTarget, stationBoxCounter, stationCount,
                        params->ctrl.stop, params->ctrl.speed);
            cooldown--;
            break;
        }

        // 施工区首框跳过后只固定屏蔽若干帧；不再依赖检测框完全消失，
        // 避免误检残留把目标停车框长期锁住。
        if (params->busyZone && busyBoxLockFrames > 0)
        {
            if (busyBoxLockFrames == Tune::BUSY_SKIP_LOCK_FRAMES || busyBoxLockFrames == 1)
                busylog("BLOCK skipLock=%d target=%d counter=%d boxes=%d",
                        busyBoxLockFrames, busyTarget, stationBoxCounter, stationCount);
            busyBoxLockFrames--;
            if (busyBoxLockFrames == 0)
            {
                stationBoxCounted = false;
                busylog("SKIP_LOCK_EXPIRED target=%d counter=%d; target detection enabled",
                        busyTarget, stationBoxCounter);
            }
            break;
        }

        if (pressTimer > 0)
        {
            pressTimer++;
            // 各停车场景压框等待帧数集中在 Tune 中调参
            int pressThreshold = Tune::NORMAL_PRESS_FRAMES;
            if (params->busyZone && stationBoxCounter == 0)
                pressThreshold = Tune::BUSY_FIRST_PRESS_FRAMES;
            if (params->busyZone && params->config.currentLapConfig->busyStopPoint > 1 &&
                stationBoxCounter >= params->config.currentLapConfig->busyStopPoint - 1)
                pressThreshold = Tune::BUSY_TARGET_PRESS_FRAMES;
            if (params->yforkBranch == 1)
                pressThreshold = Tune::LEFT_BRANCH_PRESS_FRAMES;
            else if (params->yforkBranch == 2)
                pressThreshold = Tune::RIGHT_BRANCH_PRESS_FRAMES;
            if (params->yforkBranch != 0 &&
                (pressTimer == 2 || pressTimer == pressThreshold ||
                 pressTimer == pressThreshold + 1))
                yforkDiagLog("STATION_EVENT",
                             "PRESS_PROGRESS path=%s timer=%d/%d stop=%d speed=%.3f",
                             params->yforkBranch == 1 ? "YFORK_LEFT" : "YFORK_RIGHT",
                             pressTimer, pressThreshold, params->ctrl.stop, params->ctrl.speed);
            if (params->busyZone && (pressTimer == 2 || pressTimer == pressThreshold || pressTimer == pressThreshold + 1))
                busylog("PRESS timer=%d threshold=%d target=%d counter=%d branch=%d ctrlStop=%d speed=%.2f",
                        pressTimer, pressThreshold, params->config.currentLapConfig->busyStopPoint,
                        stationBoxCounter, params->yforkBranch, params->ctrl.stop, params->ctrl.speed);
            if (pressTimer > pressThreshold)
            {
                if (params->busyZone)
                    busylog("STOP_TRIGGER timer=%d threshold=%d target=%d counter=%d branch=%d beforeStop=%d beforeSpeed=%.2f",
                            pressTimer, pressThreshold, params->config.currentLapConfig->busyStopPoint,
                            stationBoxCounter, params->yforkBranch, params->ctrl.stop, params->ctrl.speed);
                const char *stopPath = params->yforkBranch == 1
                                           ? "YFORK_LEFT_STATION"
                                           : (params->yforkBranch == 2 ? "YFORK_RIGHT_STATION" : "NORMAL_STATION");
                printf("[Station] STOP PATH=%s, pressed +%.1fs, stopping...\n",
                       stopPath, pressThreshold / 30.0f);
                yforkDiagLog("STATION_EVENT",
                             "STOP_TRIGGER path=%s timer=%d threshold=%d",
                             stopPath, pressTimer, pressThreshold);
                setStep(Step::STOP);
                params->ctrl.stop = true;
                params->ctrl.speed = 0.0f;
                if (params->busyZone)
                    busylog("STOP_SET ctrlStop=%d speed=%.2f step=STOP", params->ctrl.stop, params->ctrl.speed);
            }
            break;
        }

        // 左岔路：引导结束后，框底边超过画面 50% 即累计停车计时。
        if (params->yforkBranch == 1)
        {
            // 引导期间不检测
            if (params->yforkGuiding)
            {
                static int leftGuideBlockLogCounter = 0;
                if (leftGuideBlockLogCounter++ % 10 == 0)
                    yforkDiagLog("STATION_EVENT",
                                 "BLOCK reason=yforkGuiding branch=1 leftDelay=%d press=%d results=%zu",
                                 leftBranchDelay, pressTimer, params->results.size());
                if (params->busyZone)
                    busylog("BLOCK yforkGuiding branch=1 target=%d counter=%d boxes=%d",
                            params->config.currentLapConfig->busyStopPoint, stationBoxCounter, stationCount);
                break;
            }

            bool boxSeen = false;
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_STATION)
                {
                    int boxBottom = params->results[i].y + params->results[i].height;
                    if (boxBottom > ROWSIMAGE * Tune::LEFT_BRANCH_TRIGGER_RATIO)
                    {
                        boxSeen = true;
                        break;
                    }
                }
            }

            if (boxSeen)
            {
                leftBranchDelay++;
                if (leftBranchDelay == 1 || leftBranchDelay == Tune::LEFT_BRANCH_DELAY_FRAMES)
                    yforkDiagLog("STATION_EVENT",
                                 "LEFT_CONFIRM delay=%d/%d threshold=%d",
                                 leftBranchDelay, Tune::LEFT_BRANCH_DELAY_FRAMES,
                                 static_cast<int>(ROWSIMAGE * Tune::LEFT_BRANCH_TRIGGER_RATIO));
                if (leftBranchDelay >= Tune::LEFT_BRANCH_DELAY_FRAMES)
                {
                    params->stationStarted = true;
                    pressTimer = 1;
                    leftBranchDelay = 0;
                    printf("[Station] STOP PATH=YFORK_LEFT_STATION, target box confirmed, delay=%d done\n",
                           Tune::LEFT_BRANCH_DELAY_FRAMES);
                    yforkDiagLog("STATION_EVENT",
                                 "TARGET_TRIGGER path=YFORK_LEFT press=1 pressThreshold=%d",
                                 Tune::LEFT_BRANCH_PRESS_FRAMES);
                }
            }
            else if (leftBranchDelay > 0)
            {
                yforkDiagLog("STATION_EVENT",
                             "LEFT_CONFIRM_GAP keepDelay=%d/%d station box not qualified this frame",
                             leftBranchDelay, Tune::LEFT_BRANCH_DELAY_FRAMES);
            }
        }
        else
        {
            // yfork引导期间不检测station框，避免干扰岔路导航
            if (params->yforkGuiding)
            {
                static int guideBlockLogCounter = 0;
                if (guideBlockLogCounter++ % 10 == 0)
                    yforkDiagLog("STATION_EVENT",
                                 "BLOCK reason=yforkGuiding branch=%d press=%d results=%zu",
                                 params->yforkBranch, pressTimer, params->results.size());
                if (params->busyZone)
                    busylog("BLOCK yforkGuiding branch=0 target=%d counter=%d boxes=%d",
                            params->config.currentLapConfig->busyStopPoint, stationBoxCounter, stationCount);
                break;
            }

            // 普通 station：框到底部才检测
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
                    int threshold;
                    if (isFirstBox)
                        threshold = static_cast<int>(ROWSIMAGE * Tune::BUSY_FIRST_BOX_RATIO);
                    else if (params->yforkBranch == 2)
                        threshold = ROWSIMAGE - Tune::RIGHT_BRANCH_TRIGGER_BOTTOM_MARGIN;
                    else
                        threshold = ROWSIMAGE - (params->busyZone ? Tune::BUSY_TARGET_BOTTOM_MARGIN
                                                                   : Tune::NORMAL_TRIGGER_BOTTOM_MARGIN);
                    if (params->busyZone && boxBottom <= threshold && busyFrameLogCounter % 30 == 0)
                        busylog("WAIT_BOX bottom=%d threshold=%d target=%d counter=%d counted=%d completed=%d",
                                boxBottom, threshold, params->config.currentLapConfig->busyStopPoint,
                                stationBoxCounter, stationBoxCounted, params->stationStopCompleted);
                    if (params->yforkBranch == 2 && boxBottom <= threshold)
                        yforkDiagLog("STATION_EVENT",
                                     "WAIT_THRESHOLD path=YFORK_RIGHT bottom=%d needGreaterThan=%d",
                                     boxBottom, threshold);
                    if (boxBottom > threshold)
                    {
                        // 施工区第一个框需要连续满足3帧，过滤单帧误检并稍微延后停车。
                        // 施工区已停过，不再重复检测
                        if (params->busyZone && params->stationStopCompleted)
                        {
                            busylog("BLOCK completed target=%d counter=%d bottom=%d threshold=%d ctrlStop=%d speed=%.2f",
                                    busyTarget, stationBoxCounter, boxBottom, threshold,
                                    params->ctrl.stop, params->ctrl.speed);
                            break;
                        }

                        // 施工区跳过前N个框
                        if (params->busyZone)
                        {
                            int targetBox = params->config.currentLapConfig->busyStopPoint;
                            if (targetBox > 1 && stationBoxCounter < targetBox - 1)
                            {
                                // 首框已经计数：固定锁定期间不重复计数。
                                if (stationBoxCounted)
                                {
                                    busylog("BLOCKED skipLock target=%d boxCounter=%d lock=%d bottom=%d threshold=%d",
                                            targetBox, stationBoxCounter, busyBoxLockFrames, boxBottom, threshold);
                                    break;
                                }

                                stationBoxCounter++;
                                stationBoxCounted = true;
                                busyBoxLockFrames = Tune::BUSY_SKIP_LOCK_FRAMES;
                                params->stationStarted = true;
                                params->stationStopCompleted = false;
                                busylog("SKIP_BOX skipped=%d target=%d bottom=%d threshold=%d; skipLock=%d",
                                        stationBoxCounter, targetBox, boxBottom, threshold, busyBoxLockFrames);
                                printf("[Station] Skip box #%d (target=%d)\n", stationBoxCounter, targetBox);
                                break;
                            }
                        }

                        params->stationStarted = true;
                        pressTimer = 1;
                        yforkDiagLog("STATION_EVENT",
                                     "TARGET_TRIGGER path=%s bottom=%d threshold=%d press=1 branch=%d",
                                     params->yforkBranch == 2 ? "YFORK_RIGHT" : "NORMAL_OR_BUSY",
                                     boxBottom, threshold, params->yforkBranch);
                        if (params->busyZone)
                            busylog("TARGET_BOX_TRIGGER target=%d boxCounter=%d bottom=%d threshold=%d pressTimer=1 branch=%d",
                                    params->config.currentLapConfig->busyStopPoint,
                                    stationBoxCounter, boxBottom, threshold, params->yforkBranch);
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
        params->ctrl.speed = 0.0f;
        stopCounter++;
        if (params->yforkBranch != 0 &&
            (stopCounter == 1 || stopCounter == Tune::STOP_HOLD_FRAMES ||
             stopCounter == Tune::STOP_HOLD_FRAMES + 1))
            yforkDiagLog("STATION_EVENT",
                         "STOP_HOLD path=%s hold=%d/%d ctrlStop=%d speed=%.3f",
                         params->yforkBranch == 1 ? "YFORK_LEFT" : "YFORK_RIGHT",
                         stopCounter, Tune::STOP_HOLD_FRAMES,
                         params->ctrl.stop, params->ctrl.speed);
        if (params->busyZone && (stopCounter == 1 || stopCounter == Tune::STOP_HOLD_FRAMES || stopCounter == Tune::STOP_HOLD_FRAMES + 1))
            busylog("STOP_HOLD hold=%d/%d ctrlStop=%d speed=%.2f target=%d counter=%d branch=%d",
                    stopCounter, Tune::STOP_HOLD_FRAMES, params->ctrl.stop, params->ctrl.speed,
                    params->config.currentLapConfig->busyStopPoint, stationBoxCounter, params->yforkBranch);
        printf("[Station] Stop %d/%d\n", stopCounter, Tune::STOP_HOLD_FRAMES);
        if (stopCounter > Tune::STOP_HOLD_FRAMES) // 停车保持
        {
            if (params->busyZone)
                busylog("STOP_COMPLETE hold=%d target=%d boxCounter=%d branch=%d",
                        stopCounter, params->config.currentLapConfig->busyStopPoint,
                        stationBoxCounter, params->yforkBranch);
            const char *stopPath = params->yforkBranch == 1
                                       ? "YFORK_LEFT_STATION"
                                       : (params->yforkBranch == 2 ? "YFORK_RIGHT_STATION" : "NORMAL_STATION");
            printf("[Station] STOP PATH=%s, stop complete, resume\n", stopPath);
            params->stationStopCompleted = true; // 通知yfork边线突变可以退出了
            yforkDiagLog("STATION_EVENT",
                         "STOP_COMPLETE path=%s stationStopCompleted=1 hold=%d",
                         stopPath, stopCounter);
            setStep(Step::NONE);
            cooldown = params->busyZone ? Tune::BUSY_COOLDOWN_FRAMES : Tune::NORMAL_COOLDOWN_FRAMES; // 停车后冷却
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
    busyBoxLockFrames = 0;
    busyEntryDelay = 0;
    params->stationStopCompleted = false;
    params->stationStarted = false;
}
