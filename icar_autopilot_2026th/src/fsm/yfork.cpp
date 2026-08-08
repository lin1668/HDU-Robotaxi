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
 * @file yfork.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief Y型岔路口图像识别与规划
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/yfork.hpp"
#include "utils/yfork_diag.hpp"
#include <cstdio>
#include <cstdarg>
#include <ctime>
#include <string>

// TEMP fallback test switch:
// true: disable normal LABEL_FORK detection and only test STATION_BOX fallback.
// Set back to false after the left/right fallback test.
static constexpr bool TEST_DISABLE_NORMAL_YFORK_DETECT = false;
static constexpr bool ENABLE_LEFT_STATION_BOX_FALLBACK = true;   // 左分支启用虚线框/STATION_BOX兜底，FORK漏检时仍可强制左转
static constexpr bool ENABLE_RIGHT_STATION_BOX_FALLBACK = true;  // 右分支启用虚线框/STATION_BOX兜底：FORK漏检时仍可进入右停车链
static constexpr int STATION_BOX_FALLBACK_FORCE_LEFT_DELAY = 0;
static constexpr int STATION_BOX_FALLBACK_FORCE_LEFT_FRAMES = 26;
static int activeForceLeftTotalFrames = 16;

// 所有YFork事件统一写入带毫秒和行号的诊断日志。
static void ylog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    yforkDiagVLog("YFORK_EVENT", fmt, args);
    va_end(args);
}

/**
 * @brief Construct a new Fsm Yfork
 *
 * @param par
 */
FsmYfork::FsmYfork(std::shared_ptr<Params> par)
    : FSMState(FsmMode::YFORK, par)
{
    yforkDiagReset("FsmYfork constructed");
}

/**
 * @brief Destroy the Fsm Yfork
 *
 */
FsmYfork::~FsmYfork()
{
}

void FsmYfork::logFrame(const char *phase)
{
    // PRE/POST are nearly duplicates. A POST sample every 10 frames is enough
    // to diagnose steering, lane loss and detector timing.
    static unsigned int frameLogCounter = 0;
    const bool isPost = phase && phase[0] == 'P' && phase[1] == 'O' &&
                        phase[2] == 'S' && phase[3] == 'T' && phase[4] == '\0';
    if (!isPost || (++frameLogCounter % 10) != 1)
        return;

    std::string detections;
    for (size_t i = 0; i < params->results.size(); ++i)
    {
        const auto &r = params->results[i];
        if (!detections.empty())
            detections += ";";
        detections += "i=" + std::to_string(i) +
                      ",type=" + std::to_string(r.type) +
                      ",x=" + std::to_string(r.x) +
                      ",y=" + std::to_string(r.y) +
                      ",w=" + std::to_string(r.width) +
                      ",h=" + std::to_string(r.height) +
                      ",bottom=" + std::to_string(r.y + r.height);
    }
    if (detections.empty())
        detections = "none";

    const auto &left = params->track->pointsEdgeLeft;
    const auto &right = params->track->pointsEdgeRight;
    const auto &center = params->ctrl.centerEdge;
    const PointX leftFirst = left.empty() ? PointX(-1, -1) : left.front();
    const PointX leftLast = left.empty() ? PointX(-1, -1) : left.back();
    const PointX rightFirst = right.empty() ? PointX(-1, -1) : right.front();
    const PointX rightLast = right.empty() ? PointX(-1, -1) : right.back();
    const PointX centerLast = center.empty() ? PointX(-1, -1) : center.back();

    yforkDiagLog(
        "YFORK_FRAME",
        "%s lap=%d mode=%d step=%d enable=%d selectLeft=%d branch=%d guiding=%d "
        "forkSeen=%d completed=%d stationStarted=%d stationDone=%d stop=%d "
        "tip=(%d,%d) hold=(%d,%d) vloss=%d vlossTimer=%d forceLeft=%d "
        "forkForce=%d autoStop=%d parkStop=%d counter=%d timeout=%d "
        "trackL=%zu first=(%d,%d) last=(%d,%d) trackR=%zu first=(%d,%d) last=(%d,%d) "
        "centerEdge=%zu centerLast=(%d,%d) ctrlCenter=%d speed=%.3f servoPWM=%u detections=[%s]",
        phase, params->currentLap, static_cast<int>(params->mode), static_cast<int>(step),
        enable, selectLeft, params->yforkBranch, params->yforkGuiding,
        forkSeen, completed, params->stationStarted, params->stationStopCompleted,
        params->ctrl.stop, tipRow, tipCol, holdRow, holdCol, vloss, vlossTimer,
        forceLeftTimer, forkForceLeftTimer, forkAutoStopTimer, yforkParkStopCount,
        counterYfork, timeout,
        left.size(), leftFirst.x, leftFirst.y, leftLast.x, leftLast.y,
        right.size(), rightFirst.x, rightFirst.y, rightLast.x, rightLast.y,
        center.size(), centerLast.x, centerLast.y, params->ctrl.center,
        params->ctrl.speed, params->ctrl.servo, detections.c_str());
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmYfork::getMode()
{
    const bool hasLapConfig = params->config.currentLapConfig != nullptr;
    const bool lapYfork = hasLapConfig && params->config.currentLapConfig->yfork;
    if (!params->config.yfork || !hasLapConfig || !lapYfork || !enable)
    {
        static int blockLogCnt = 0;
        if ((enable || params->mode == FsmMode::YFORK || params->yforkBranch != 0) && blockLogCnt++ % 10 == 0)
        {
            ylog("[Yfork] getMode -> NORMAL: globalYfork=%d hasLapConfig=%d lapYfork=%d enable=%d mode=%d branch=%d guiding=%d",
                 params->config.yfork, hasLapConfig, lapYfork, enable,
                 static_cast<int>(params->mode), params->yforkBranch, params->yforkGuiding);
        }
        return FsmMode::NORMAL;
    }

    static int yforkModeLogCounter = 0;
    if (yforkModeLogCounter++ % 30 == 0)
        ylog("[Yfork] getMode -> YFORK: lap=%d globalYfork=%d lapYfork=%d yforkLeft=%d enable=%d step=%d branch=%d forkSeen=%d completed=%d results=%d",
             params->currentLap, params->config.yfork, lapYfork,
             params->config.currentLapConfig->yforkLeft, enable, static_cast<int>(step),
             params->yforkBranch, forkSeen, completed, (int)params->results.size());
    return FsmMode::YFORK;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmYfork::run(Mat &img)
{
    // 当前圈未配置Y型岔路时，必须清除上一圈或其它模式遗留的检测状态，
    // 防止forkSeen残留导致误进入YFORK模式。
    if (!params->config.yfork || !params->config.currentLapConfig ||
        !params->config.currentLapConfig->yfork)
    {
        static int noTaskLogCnt = 0;
        if ((enable || forkSeen || step != Step::NONE || params->yforkBranch != 0 || params->yforkGuiding) && noTaskLogCnt++ % 10 == 0)
        {
            ylog("[Yfork] run blocked: no task, reset residual state. lap=%d globalYfork=%d hasLapConfig=%d lapYfork=%d enable=%d step=%d forkSeen=%d branch=%d guiding=%d completed=%d results=%d",
                 params->currentLap, params->config.yfork, params->config.currentLapConfig != nullptr,
                 params->config.currentLapConfig ? params->config.currentLapConfig->yfork : 0,
                 enable, static_cast<int>(step), forkSeen, params->yforkBranch,
                 params->yforkGuiding, completed, (int)params->results.size());
        }
        reset();
        completed = false;
        return;
    }

    // park退出时通知yfork复位，防止残留forkSeen误触发
    if (params->ctrl.yforkReset)
    {
        ylog("[Yfork] run: yforkReset received -> reset()");
        reset();
        params->ctrl.yforkReset = false;
    }

    // 停完车后退出YFork模式，让车恢复正常巡线（仅YFork已完成时关闭）
    if (params->stationStopCompleted && completed)
    {
        enable = false;
        params->yforkBranch = 0;
        params->yforkGuiding = false;
        if (!exitRightBiasArmed)
        {
            params->ctrl.yforkExitCooldown = 0; // 停车完成后立即解除YFork限速，交给正常巡线恢复速度
            exitRightBiasArmed = true;
        }
        return;
    }

    // 非NORMAL/YFORK模式下不干扰其他FSM（如停车区内的fork地面箭头）
    if (params->mode != FsmMode::NORMAL && params->mode != FsmMode::YFORK)
        return;

    {
        static bool lastStationStarted = false;
        static bool lastStationDone = false;
        static bool lastGuiding = false;
        if (lastStationStarted != params->stationStarted ||
            lastStationDone != params->stationStopCompleted ||
            lastGuiding != params->yforkGuiding)
        {
            ylog("[Yfork] SHARED_STATE_CHANGE stationStarted %d->%d stationDone %d->%d guiding %d->%d branch=%d step=%d forceLeft=%d timeout=%d mode=%d stop=%d speed=%.3f",
                 lastStationStarted, params->stationStarted,
                 lastStationDone, params->stationStopCompleted,
                 lastGuiding, params->yforkGuiding,
                 params->yforkBranch, static_cast<int>(step), forceLeftTimer,
                 timeout, static_cast<int>(params->mode), params->ctrl.stop,
                 params->ctrl.speed);
            lastStationStarted = params->stationStarted;
            lastStationDone = params->stationStopCompleted;
            lastGuiding = params->yforkGuiding;
        }
    }

    logFrame("PRE");

    // ===== 左分支停车兜底：仅由 FORK 触发的左转启动 =====
    // 不提前 return，确保自动停车倒计时期间仍持续执行岔路引导和退出检测。
    if (forkAutoStopTimer > 0 && params->yforkBranch == 1 && !params->stationStopCompleted)
    {
        // 岔路引导未结束时不计兜底时间，避免车辆仍在强制左转阶段就提前停车。
        if (params->yforkGuiding)
        {
            // 等待引导结束后再由 Station 或自动停车兜底接管。
        }
        // Station 已经识别到目标停车框时，交由 Station 完成停车；尚未停车时才取消兜底。
        else if (params->stationStarted && yforkParkStopCount == 0)
        {
            ylog("[Yfork] STOP PATH=STATION_LEFT_BRANCH: station started, cancel FORK auto-stop fallback");
            forkAutoStopTimer = 0;
        }
        else
        {
            forkAutoStopTimer++;
            if (forkAutoStopTimer >= FORK_AUTO_STOP_DELAY)
            {
                params->ctrl.stop = true;
                yforkParkStopCount++;
                 if (yforkParkStopCount == 1)
                    ylog("[Yfork] STOP PATH=FORK_AUTO_FALLBACK: timer=%d >= %d, begin stop",
                        forkAutoStopTimer, FORK_AUTO_STOP_DELAY);
                if (yforkParkStopCount > 30)
                {
                    params->ctrl.stop = false;
                    params->stationStopCompleted = true;
                    forkAutoStopTimer = 0;
                    yforkParkStopCount = 0;
                    ylog("[Yfork] STOP PATH=FORK_AUTO_FALLBACK: stop complete, resume");
                }
            }
        }
    }

    enable = handle(img); // 处理Y型岔路口
    logFrame("POST");
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmYfork::show(Mat &img)
{
    if (params->mode != FsmMode::YFORK)
        return;

    putText(img, "[9] Yfork", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    // 绘制赛道边缘点
    for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
    {
        circle(img, Point(params->track->pointsEdgeLeft[i].y, params->track->pointsEdgeLeft[i].x), 2,
               Scalar(0, 255, 0), -1); // 绿色
    }
    for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
    {
        circle(img, Point(params->track->pointsEdgeRight[i].y, params->track->pointsEdgeRight[i].x), 2,
               Scalar(0, 255, 255), -1); // 黄色
    }
    // 绘制V尖（最初选取的岔路红点）
    if (tipRow > 0 && tipCol > 0)
        circle(img, Point(tipCol, tipRow), 4, Scalar(0, 0, 255), -1);

    switch (step)
    {
    case Step::DECIDE:
        putText(img, "[9] Yfork - DECIDE", Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;
    case Step::ENTER:
        putText(img, "[9] Yfork - ENTER " + (selectLeft ? string("LEFT") : string("RIGHT")),
                Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        if (tipRow > 0 && tipCol > 0)
            circle(img, Point(tipCol, tipRow), 4, Scalar(0, 0, 255), -1);
        break;
    case Step::EXIT:
        putText(img, "[9] Yfork - EXIT " + (selectLeft ? string("LEFT") : string("RIGHT")),
                Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;
    }
}

/**
 * @brief 重置FSM状态
 *
 */
void FsmYfork::reset(void)
{
    step = Step::NONE;
    enable = false;
    counterYfork = 0;
    timeout = 0;
    countRes = 0;
    selectLeft = false;
    forkSeen = false;
    vloss = false;
    tipRow = 0;
    tipCol = 0;
    vlossTimer = 0;
    holdRow = 0;
    holdCol = 0;
    forceLeftTimer = 0;
    activeForceLeftTotalFrames = FORCE_LEFT_FRAMES;
    forkForceLeftTimer = 0;
    forkAutoStopTimer = 0;
    yforkParkStopCount = 0;
    // forceLeftDone 不在这里清零，只在 resetLap() 清零，防止同圈重复触发
    params->stationStopCompleted = false;
    params->stationStarted = false;
    params->yforkGuiding = false;
    params->yforkStationBoxFallback = false;
    // 当前 Yfork 任务尚未完成时保留分支，供 station 使用；finish() 与 resetLap() 清零。
}

void FsmYfork::finish()
{
    // 正常完成后不能让 reset() 清掉 station 的已停靠结果；同时结束左分支
    // 标记，避免后续普通 Station 框再次按左分支规则触发停车。
    bool stationStopCompleted = params->stationStopCompleted;
    reset();
    completed = true;
    params->ctrl.yforkExitCooldown = 90; // 正常完成 YFork 后约 4.5 秒靠右，避开左转盲区障碍
    exitRightBiasArmed = true;
    params->stationStopCompleted = stationStopCompleted;
    params->yforkBranch = 0;
}

void FsmYfork::resetLap()
{
    reset();
    completed = false; // 新圈重新检测
    forceLeftDone = false; // 新一圈重新允许强制左转
    exitRightBiasArmed = false;
    params->yforkBranch = 0; // 新一圈清零
}

/**
 * @brief 当前圈没有Y型岔路任务时，清除Y岔路本身的残留状态
 *
 * 不复位station状态，避免施工区/停车场圈的停靠流程被误清除。
 */
void FsmYfork::deactivate()
{
    bool stationStopCompleted = params->stationStopCompleted;
    bool stationStarted = params->stationStarted;

    if (enable || forkSeen || step != Step::NONE || params->yforkBranch != 0 || params->yforkGuiding || completed || forceLeftDone)
    {
        ylog("[Yfork] deactivate: lap=%d no yfork task, clearing residual state. globalYfork=%d lapYfork=%d enable=%d step=%d forkSeen=%d branch=%d guiding=%d completed=%d forceLeftDone=%d results=%d",
             params->currentLap, params->config.yfork,
             params->config.currentLapConfig ? params->config.currentLapConfig->yfork : 0,
             enable, static_cast<int>(step), forkSeen, params->yforkBranch,
             params->yforkGuiding, completed, forceLeftDone, (int)params->results.size());
    }

    reset();
    completed = false;
    forceLeftDone = false;
    exitRightBiasArmed = false;
    params->yforkBranch = 0;

    params->stationStopCompleted = stationStopCompleted;
    params->stationStarted = stationStarted;
}

/**
 * @brief 处理Y型岔路口
 *
 * @param img 所传递的图像
 * @return true
 * @return false
 */
bool FsmYfork::handle(Mat &img)
{
    // 任务门禁：没有Y型岔路任务的圈次绝不允许保持或进入YFORK模式。
    if (!params->config.currentLapConfig || !params->config.currentLapConfig->yfork)
        return false;

    switch (step)
    {
    case Step::NONE:
    {
        // 每30帧输出一次AI检测到的所有标签，方便确认虚线框对应哪个ID
        static int labelLogCnt = 0;
        if (labelLogCnt++ % 30 == 1)
        {
            string labels = "";
            for (auto &r : params->results)
                labels += to_string(r.type) + " ";
            ylog("[Yfork] NONE: AI results count=%d labels=[%s]", (int)params->results.size(), labels.c_str());
        }

        // ===== 左分支强制左转：检测右侧虚线框触发 =====
        // 仅当：yfork功能启用 && 当前圈有yfork && 走左分支 && 未执行过
        // 其他情况（右分支/无yfork/已完成）完全不触发，不影响正常巡线
        bool canForceLeft = params->config.yfork &&
                            ENABLE_LEFT_STATION_BOX_FALLBACK &&
                            params->config.currentLapConfig->yfork &&
                            params->config.currentLapConfig->yforkLeft &&
                            !forceLeftDone &&
                            !completed;
        if (canForceLeft)
        {
            // 扫描所有检测结果，找右侧虚线框
            int stationCount = 0;
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_STATION)
                {
                    stationCount++;
                    int boxCx = params->results[i].x + params->results[i].width / 2;
                    int boxBottom = params->results[i].y + params->results[i].height;
                    // 检测到虚线框即触发强制左转（已去掉方向限制，forceLeftDone 保证只转一次）
                    if (params->results[i].height > 10 && params->results[i].width > 10)
                    {
                        selectLeft = true;
                        params->yforkBranch = 1;
                        params->yforkGuiding = false;
                        params->yforkStationBoxFallback = true;
                        forkSeen = true; // 检测到虚线框 = 检测到yfork
                        forkForceLeftTimer++;
                        params->ctrl.speed = params->config.velYfork; // fallback frame also slows down before hard left
                        if (forkForceLeftTimer <= STATION_BOX_FALLBACK_FORCE_LEFT_DELAY)
                        {
                            ylog("[Yfork] FORCE LEFT PATH=STATION_BOX_PRE_SLOW: timer=%d/%d box #%d cx=%d y=%d h=%d w=%d bottom=%d",
                                 forkForceLeftTimer, STATION_BOX_FALLBACK_FORCE_LEFT_DELAY, stationCount, boxCx,
                                 params->results[i].y, params->results[i].height,
                                 params->results[i].width, boxBottom);
                            return true;
                        }
                        forceLeftTimer = STATION_BOX_FALLBACK_FORCE_LEFT_FRAMES;
                        activeForceLeftTotalFrames = STATION_BOX_FALLBACK_FORCE_LEFT_FRAMES;
                        forceLeftDone = true; // 标记已执行，防止重复触发
                        params->yforkGuiding = false;
                        forkForceLeftTimer = 0;
                        step = Step::ENTER;
                        counterYfork = 0;
                        timeout = 0;
                            ylog("[Yfork] FORCE LEFT PATH=STATION_BOX: box #%d cx=%d y=%d h=%d w=%d bottom=%d, forceFrames=%d -> ENTER",
                             stationCount, boxCx, params->results[i].y,
                             params->results[i].height, params->results[i].width, boxBottom,
                             STATION_BOX_FALLBACK_FORCE_LEFT_FRAMES);
                        return true;
                    }
                    else
                    {
                        ylog("[Yfork] NONE: station box #%d cx=%d too small h=%d w=%d bottom=%d, skip",
                             stationCount, boxCx, params->results[i].height, params->results[i].width,
                             boxBottom);
                    }
                }
            }
            if (stationCount == 0)
            {
                static int noStationLog = 0;
                if (noStationLog++ % 30 == 1)
                    ylog("[Yfork] NONE: canForceLeft=true but no station box found (results=%d)", (int)params->results.size());
            }
        }
        else
        {
            // 记录不触发的原因（每100帧一次，避免刷屏）
            static int skipLog = 0;
            if (skipLog++ % 100 == 1)
            {
                ylog("[Yfork] NONE: force left DISABLED (yfork=%d lapYfork=%d yforkLeft=%d forceLeftDone=%d)",
                     params->config.yfork, params->config.currentLapConfig->yfork,
                      params->config.currentLapConfig->yforkLeft, forceLeftDone);
            }
        }

        // STATION_BOX 左分支兜底一旦启动，即使下一帧虚线框短暂丢失，也继续完成兜底触发；
        // 否则会被后面的 V 尖普通 YFork 路径抢走，导致兜底专用停车参数失效。
        if (params->yforkStationBoxFallback &&
            ENABLE_LEFT_STATION_BOX_FALLBACK &&
            params->config.currentLapConfig->yforkLeft &&
            forkSeen &&
            params->yforkBranch == 1 &&
            !forceLeftDone &&
            forkForceLeftTimer > 0)
        {
            forkForceLeftTimer++;
            params->ctrl.speed = params->config.velYfork;
            if (forkForceLeftTimer <= STATION_BOX_FALLBACK_FORCE_LEFT_DELAY)
            {
                ylog("[Yfork] FORCE LEFT PATH=STATION_BOX_ARMED_PRE_SLOW: timer=%d/%d",
                     forkForceLeftTimer, STATION_BOX_FALLBACK_FORCE_LEFT_DELAY);
                return true;
            }

            forceLeftTimer = STATION_BOX_FALLBACK_FORCE_LEFT_FRAMES;
            activeForceLeftTotalFrames = STATION_BOX_FALLBACK_FORCE_LEFT_FRAMES;
            forceLeftDone = true;
            params->yforkGuiding = false;
            forkForceLeftTimer = 0;
            step = Step::ENTER;
            counterYfork = 0;
            timeout = 0;
            ylog("[Yfork] FORCE LEFT PATH=STATION_BOX_ARMED: forceFrames=%d -> ENTER",
                 STATION_BOX_FALLBACK_FORCE_LEFT_FRAMES);
            return true;
        }

        // ===== 右分支兜底：FORK标志漏检时，虚线/停车框等效为已进入右YFork =====
        // 这次日志里只看到 LABEL_STATION，没看到 LABEL_FORK；如果不先置 branch=2，
        // Station 会被 waiting_yfork_branch 挡住，右分支停车链永远进不去。
        bool rightValidForkVisible = false;
        for (int i = 0; i < params->results.size(); i++)
        {
            if (TEST_DISABLE_NORMAL_YFORK_DETECT)
                break;
            if (params->results[i].type != LABEL_FORK)
                continue;
            int forkBottom = params->results[i].y + params->results[i].height;
            if (params->results[i].height < 150 && params->results[i].width < 150 &&
                params->results[i].height > 15 && params->results[i].width > 15 &&
                forkBottom > ROWSIMAGE * 0.20)
            {
                rightValidForkVisible = true;
                break;
            }
        }
        const bool canRightStationFallback =
            ENABLE_RIGHT_STATION_BOX_FALLBACK &&
            params->config.yfork &&
            params->config.currentLapConfig &&
            params->config.currentLapConfig->yfork &&
            !params->config.currentLapConfig->yforkLeft &&
            !completed &&
            !forkSeen &&
            !rightValidForkVisible &&
            params->yforkBranch == 0 &&
            !params->stationStarted &&
            !params->stationStopCompleted;
        if (canRightStationFallback)
        {
            int stationCount = 0;
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type != LABEL_STATION)
                    continue;
                stationCount++;
                int boxCx = params->results[i].x + params->results[i].width / 2;
                int boxBottom = params->results[i].y + params->results[i].height;
                if (params->results[i].height > 10 && params->results[i].width > 10 &&
                    boxBottom > ROWSIMAGE * 0.25)
                {
                    selectLeft = false;
                    params->yforkBranch = 2;
                    params->yforkStationBoxFallback = false;
                    forkSeen = true;
                    params->yforkGuiding = false;
                    holdRow = 0;
                    holdCol = 0;
                    vloss = true;
                    vlossTimer = 999;
                    step = Step::ENTER;
                    counterYfork = 0;
                    timeout = 0;
                    countRes = 0;
                    ylog("[Yfork] RIGHT FALLBACK=STATION_BOX: box #%d cx=%d y=%d h=%d w=%d bottom=%d -> branch=2 ENTER(no artificial guide)",
                         stationCount, boxCx, params->results[i].y,
                         params->results[i].height, params->results[i].width, boxBottom);
                    return true;
                }
            }
        }

        // ===== FORK强制左转：FORK到比例后延时触发（与虚线框互为双保险）=====
        bool canForkForce = params->config.currentLapConfig->yforkLeft && !forceLeftDone;
        if (!TEST_DISABLE_NORMAL_YFORK_DETECT && detectYfork(img))
        {
            forkSeen = true;
            params->yforkGuiding = true;
            ylog("[Yfork] NONE: fork detected, forkSeen=true");

            // 检测FORK位置，超过比例开始计时
            if (canForkForce)
            {
                bool passed = false;
                for (int i = 0; i < params->results.size(); i++)
                {
                    if (params->results[i].type == LABEL_FORK)
                    {
                        int bottom = params->results[i].y + params->results[i].height;
                        if (bottom > (int)(ROWSIMAGE * FORK_FORCE_LEFT_RATIO))
                        {
                            passed = true;
                            forkForceLeftTimer++;
                            if (forkForceLeftTimer == 1)
                                ylog("[Yfork] NONE: FORK passed %.0f%% (bottom=%d), start timer",
                                     FORK_FORCE_LEFT_RATIO * 100, bottom);
                            else if (forkForceLeftTimer % 15 == 1)
                                ylog("[Yfork] NONE: FORK timer=%d/%d", forkForceLeftTimer, FORK_FORCE_LEFT_DELAY);
                            break;
                        }
                    }
                }

                // FORK 达到阈值后在当前帧开始左转；当延迟设为 0 时不等待
                // FORK 离开画面，延迟大于 0 时则要求持续满足阈值 N 帧后触发。
                if (passed && forkForceLeftTimer > FORK_FORCE_LEFT_DELAY)
                {
                    int triggerTimer = forkForceLeftTimer;
                    selectLeft = true;
                    params->yforkBranch = 1;
                    params->yforkStationBoxFallback = false;
                    forceLeftTimer = FORCE_LEFT_FRAMES;
                    activeForceLeftTotalFrames = FORCE_LEFT_FRAMES;
                    forceLeftDone = true;
                    forkForceLeftTimer = 0;
                    forkAutoStopTimer = 1;
                    step = Step::ENTER;
                    counterYfork = 0;
                    timeout = 0;
                    ylog("[Yfork] FORCE LEFT PATH=FORK_THRESHOLD: timer=%d > delay=%d, forceFrames=%d -> ENTER",
                         triggerTimer, FORK_FORCE_LEFT_DELAY, FORCE_LEFT_FRAMES);
                    ylog("[Yfork] STOP FALLBACK=FORK_AUTO: armed, starts after guidance release, delay=%d frames",
                        FORK_AUTO_STOP_DELAY);
                    return true;
                }
                if (!passed && forkForceLeftTimer == 0)
                {
                    static int notYetLog = 0;
                    if (notYetLog++ % 30 == 1)
                        ylog("[Yfork] NONE: FORK seen but not yet %.0f%%, waiting...", FORK_FORCE_LEFT_RATIO * 100);
                }
                else if (!passed)
                {
                    forkForceLeftTimer = 0;
                }
            }
        }
        else if (forkSeen)
        {
            // fork已离开画面，开始找V尖
            if (findVTip(img))
            {
                step = Step::DECIDE;
                counterYfork = 0;
                timeout = 0;
                ylog("[Yfork] NONE: V-tip found row=%d col=%d -> DECIDE", tipRow, tipCol);
            }
            else
            {
                static int noVtipCount = 0;
                noVtipCount++;
                if (noVtipCount % 10 == 1)
                    ylog("[Yfork] NONE: forkSeen but no V-tip yet (cnt=%d)", noVtipCount);
            }
        }

        return forkSeen; // 检测到fork进入YFORK模式减速，引导在V尖找到后才开始
    }

    case Step::DECIDE:
    {
        selectLeft = params->config.currentLapConfig->yforkLeft;
        params->yforkBranch = selectLeft ? 1 : 2;
        params->yforkStationBoxFallback = params->yforkStationBoxFallback && selectLeft;
        params->yforkGuiding = !params->yforkStationBoxFallback; // 只有正常YFork引导时阻止station；STATION_BOX兜底不做左引导

        ylog("[Yfork] DECIDE: selectLeft=%d branch=%d fallback=%d -> ENTER",
             selectLeft, params->yforkBranch, params->yforkStationBoxFallback);
        step = Step::ENTER;
        counterYfork = 0;
        timeout = 0;
        return true;
    }

    case Step::ENTER:
    {
        counterYfork++;
        timeout++;

        const bool stationBoxFallbackLeft = params->yforkStationBoxFallback && selectLeft;
        const bool guidingBeforeReplan = params->yforkGuiding;
        replanTracking(selectLeft, img);
        if (guidingBeforeReplan != params->yforkGuiding)
            ylog("[Yfork] ENTER: guiding changed by replan %d->%d stationBoxFallbackLeft=%d selectLeft=%d forceLeft=%d hold=(%d,%d) vloss=%d vlossTimer=%d branch=%d",
                 guidingBeforeReplan, params->yforkGuiding, stationBoxFallbackLeft,
                 selectLeft, forceLeftTimer, holdRow, holdCol, vloss, vlossTimer,
                 params->yforkBranch);

        // ===== 强制左转：在 forceLeftTimer > 0 期间，用贝塞尔曲线强行拉左边缘 =====
        if (forceLeftTimer > 0)
        {
            forceLeftTimer--;
            // 用贝塞尔曲线覆盖左边缘，强制左转
            PointX leftStart = PointX(ROWSIMAGE - 10, 60);
            PointX leftEnd = PointX(ROWSIMAGE / 3, 1);
            PointX leftMid = PointX((leftStart.x + leftEnd.x) * 0.3f, (leftStart.y + leftEnd.y) * 0.5f);
            vector<PointX> leftPoints = {leftStart, leftMid, leftEnd};
            params->track->pointsEdgeLeft = Bezier(0.02f, leftPoints);

            const int forceFrame = activeForceLeftTotalFrames - forceLeftTimer;
            if (forceFrame == 1 || forceFrame % 4 == 0 || forceLeftTimer == 0)
                ylog("[Yfork] ENTER: FORCE LEFT frame=%d/%d remaining=%d guiding=%d stationStarted=%d stationDone=%d stop=%d stationBoxFallbackLeft=%d hold=(%d,%d) vloss=%d vlossTimer=%d leftEdge=%zu rightEdge=%zu leftGuide=(%d,%d)->(%d,%d)",
                     forceFrame, activeForceLeftTotalFrames, forceLeftTimer,
                     params->yforkGuiding, params->stationStarted,
                     params->stationStopCompleted, params->ctrl.stop,
                     stationBoxFallbackLeft, holdRow, holdCol, vloss, vlossTimer,
                     params->track->pointsEdgeLeft.size(),
                     params->track->pointsEdgeRight.size(),
                     leftStart.x, leftStart.y, leftEnd.x, leftEnd.y);

            if (forceLeftTimer == 0)
                ylog("[Yfork] ENTER: FORCE LEFT FINISHED, normal guidance resumes");
        }

        // 与上游逻辑一致：V尖消失后前5帧屏蔽station，之后一边保持引导一边搜索停车框。
        const int stationBlockFrames = 2;
        const bool guidingBeforeStationGate = params->yforkGuiding;
        const bool keepGuidingByVTip = (holdRow > 0) && (!vloss || vlossTimer < stationBlockFrames);
        const bool keepGuidingByForceLeft = forceLeftTimer > 0;
        // 强制左转还没跑完时继续屏蔽 Station，避免停车框提前接管打断左拉。
        params->yforkGuiding = !stationBoxFallbackLeft && (keepGuidingByVTip || keepGuidingByForceLeft);
        if (guidingBeforeStationGate != params->yforkGuiding)
            ylog("[Yfork] ENTER: guiding station-gate %d->%d forceLeft=%d keepByForce=%d keepByVTip=%d selectLeft=%d stationBoxFallbackLeft=%d hold=(%d,%d) vloss=%d vlossTimer=%d blockFrames=%d stationStarted=%d stationDone=%d",
                 guidingBeforeStationGate, params->yforkGuiding, forceLeftTimer,
                 keepGuidingByForceLeft, keepGuidingByVTip, selectLeft,
                 stationBoxFallbackLeft, holdRow, holdCol, vloss, vlossTimer,
                 stationBlockFrames, params->stationStarted,
                 params->stationStopCompleted);
        if (forceLeftTimer > 0 && params->yforkGuiding && !keepGuidingByVTip && !stationBoxFallbackLeft)
            ylog("[Yfork] ENTER: keep guiding because FORCE LEFT still active forceLeft=%d/%d selectLeft=%d hold=(%d,%d) vloss=%d vlossTimer=%d blockFrames=%d stationStarted=%d stationDone=%d",
                 forceLeftTimer, activeForceLeftTotalFrames, selectLeft,
                 holdRow, holdCol, vloss, vlossTimer, stationBlockFrames,
                 params->stationStarted, params->stationStopCompleted);

        // 左岔路：V尖消失后左边线突变 → 已右拐驶出岔路
        //   - 当前圈启用了station时：阻止突变退出，等先停好车
        //   - 强制左转期间不检测突变（避免贝塞尔→真实边缘跳变导致假退出）
        bool stationEnabled = params->config.currentLapConfig->station;
        bool stationBusy = stationEnabled && !params->stationStopCompleted;
        if (stationEnabled && params->stationStopCompleted && !selectLeft)
        {
            ylog("[Yfork] ENTER: RIGHT station completed -> finish immediately");
            finish();
            return true;
        }
        if (forceLeftTimer == 0 && !stationBusy && selectLeft && tipRow == 0 && params->track->pointsEdgeLeft.size() > 4)
        {
            int cur = params->track->pointsEdgeLeft.back().y;
            if (countRes > 0 && abs(cur - countRes) > 25)
            {
                ylog("[Yfork] ENTER: LEFT edge MUTATION cur=%d prev=%d diff=%d -> EXIT",
                     cur, countRes, abs(cur - countRes));
                finish();
                return true;
            }
            countRes = cur;
        }

        // 右岔路：右边缘突变 → 已左拐驶出岔路
        if (forceLeftTimer == 0 && !stationBusy && !selectLeft && tipRow == 0 && params->track->pointsEdgeRight.size() > 4)
        {
            int cur = params->track->pointsEdgeRight.back().y;
            if (countRes > 0 && abs(cur - countRes) > 25)
            {
                ylog("[Yfork] ENTER: RIGHT edge MUTATION cur=%d prev=%d diff=%d -> EXIT",
                     cur, countRes, abs(cur - countRes));
                finish();
                return true;
            }
            countRes = cur;
        }

        // 启用station时也只等待120帧，避免长期停留在YFORK减速状态。
        int exitTimeout = 150;
        if (timeout > exitTimeout)
        {
            ylog("[Yfork] ENTER: TIMEOUT timeout=%d > %d (station=%d) -> EXIT",
                 timeout, exitTimeout, stationEnabled);
            step = Step::EXIT;
            counterYfork = 0;
            timeout = 0;
        }
        return true;
    }

    case Step::EXIT:
    {
        ylog("[Yfork] EXIT -> END");
        step = Step::END;
        return true;
    }

    case Step::END:
    {
        ylog("[Yfork] END (completed)");
        finish(); // 完成一轮Y型岔路，防止停车区fork箭头误触发
        return true;
    }
    }

    return false;
}

/**
 * @brief 检测Y型岔路口
 *
 * @param img 图像
 * @return true 检测到Y型岔路口
 * @return false 未检测到
 */
bool FsmYfork::detectYfork(Mat &img)
{
    if (completed) // 已完成一轮Y型岔路，不再检测（防止停车区fork箭头误触发）
    {
        return false;
    }

    // 当前圈使能了停车场时：画面有PARK标志说明叉形箭头是车位标识，非Y型岔路
    // 当前圈未使能停车场时（如第二圈）：PARK标志不阻断Y型岔路检测
    if (params->config.park)
    {
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_PARK)
            {
                ylog("[Yfork] detect: blocked by PARK sign");
                return false;
            }
        }
    }

    for (int i = 0; i < params->results.size(); i++)
    {
        if (params->results[i].type == LABEL_FORK)
        {
            int bottom = params->results[i].y + params->results[i].height;
            if (params->results[i].height < 150 && params->results[i].width < 150 &&
                params->results[i].height > 15 && params->results[i].width > 15 &&
                bottom > ROWSIMAGE * 0.20)
            {
                ylog("[Yfork] detect: FORK ACCEPT y=%d h=%d w=%d bottom=%d",
                     params->results[i].y, params->results[i].height, params->results[i].width, bottom);
                return true;
            }
            else
            {
                ylog("[Yfork] detect: FORK REJECTED y=%d h=%d w=%d bottom=%d (need h<150&&w<150&&h>15&&w>15&&bottom>%d)",
                     params->results[i].y, params->results[i].height, params->results[i].width,
                     bottom, (int)(ROWSIMAGE * 0.35));
            }
        }
    }
    return false;
}

/**
 * @brief 在二值图中扫描V尖（岛的底部尖端）
 *
 * @param img 二值化图像
 * @return true 找到V尖
 * @return false 未找到
 */
bool FsmYfork::findVTip(const Mat &img)
{
    // V尖已确认消失，不再接受新红点
    if (vloss)
    {
        tipRow = 0;
        tipCol = 0;
        return false;
    }

    // 只用Track的岔路红点：选最远处的（row最小 = 岛尖）
    int bestRow = 0, bestCol = 0;
    int spurroadCount = (int)params->track->spurroad.size();
    for (const auto &p : params->track->spurroad)
    {
        if (p.x > ROWSIMAGE / 4 && p.x < ROWSIMAGE - 20 &&
            (bestRow == 0 || p.x < bestRow))
        {
            bestRow = p.x;
            bestCol = p.y;
        }
    }
    if (bestRow > 0)
    {
        tipRow = bestRow;
        tipCol = bestCol;

        // 红点到达图像下方 → 标记消失
        if (tipRow > ROWSIMAGE * 0.7)
        {
            vloss = true;
            ylog("[Yfork] V-tip lost (bottom): row=%d col=%d (spurroad=%d, threshold=%d)",
                 tipRow, tipCol, spurroadCount, (int)(ROWSIMAGE * 0.7));
            tipRow = 0;
            tipCol = 0;
        }
        return true;
    }

    // 红点从有到无 → 标记消失，后续不再接受新红点
    if (tipRow > 0)
    {
        vloss = true;
        ylog("[Yfork] V-tip lost (disappeared): last row=%d col=%d (spurroad now=%d)",
             tipRow, tipCol, spurroadCount);
    }

    tipRow = 0;
    tipCol = 0;
    return false;
}

/**
 * @brief 车道线重绘：保留自然检测边缘 + V尖屏障线引导进入岔路
 *
 * @param left true=左分支, false=右分支
 */
void FsmYfork::replanTracking(bool left, const Mat &img)
{
    findVTip(img); // 更新V尖位置

    int vRow = tipRow;
    int vCol = tipCol;

    // V尖可见时保存最后位置
    if (vRow > 0 && vCol > 0)
    {
        holdRow = vRow;
        holdCol = vCol;
        vlossTimer = 0;
    }

    // 与上游逻辑一致：V尖消失后左右分支都继续保持引导18帧（约0.6秒）。
    const int guideHoldFrames = 18;
    if (vRow == 0 || vCol == 0)
    {
        if (holdRow > 0 && vlossTimer < guideHoldFrames)
        {
            vlossTimer++;
            vRow = holdRow;
            vCol = holdCol;
        }
        else
        {
            if (holdRow > 0)
                ylog("[Yfork] replan: hold expired (vlossTimer=%d >= %d, left=%d), releasing guidance",
                     vlossTimer, guideHoldFrames, left);
            holdRow = 0;
            holdCol = 0;
            return;
        }
    }

    // 右分支保留自然边线，只将有效左边线轻微右移，使控制中心小幅靠右。
    // 不重建人工边线，避免极左侧误检V尖时产生过强右转。
    if (!left)
    {
        for (auto &point : params->track->pointsEdgeLeft)
            point.y = min(point.y + RIGHT_GUIDE_OFFSET, COLSIMAGE - 1);
        return;
    }

    if (left)
    {
        // 左分支：右边线 = 岛左边界 + V尖垂直屏障（保留）
        vector<PointX> island;
        for (int row = vRow; row >= ROWSIMAGE / 4; row--)
        {
            int islandCol = -1;
            for (int col = vCol; col >= 0; col--)
            {
                if (img.at<uchar>(row, col) > 128)
                {
                    islandCol = col;
                    break;
                }
            }
            if (islandCol >= 0)
            {
                if (!island.empty())
                {
                    int prev = island.back().y;
                    if (abs(islandCol - prev) > 8)
                        islandCol = prev;
                }
                island.push_back(PointX(row, islandCol));
            }
        }
        reverse(island.begin(), island.end());

        vector<PointX> barrier;
        int steps = 10;
        for (int i = 1; i <= steps; i++)
        {
            float t = (float)i / steps;
            barrier.push_back(PointX(
                vRow + (ROWSIMAGE - 10 - vRow) * t,
                vCol));
        }
        params->track->pointsEdgeRight = island;
        params->track->pointsEdgeRight.insert(params->track->pointsEdgeRight.end(),
                                              barrier.begin(), barrier.end());

        // 左边缘：如果没有强制左转在跑，才用贝塞尔引导线
        if (forceLeftTimer == 0)
        {
            PointX leftStart = PointX(ROWSIMAGE - 10, 60);
            PointX leftEnd = PointX(ROWSIMAGE / 3, 1);
            PointX leftMid = PointX((leftStart.x + leftEnd.x) * 0.3f, (leftStart.y + leftEnd.y) * 0.5f);
            vector<PointX> leftPoints = {leftStart, leftMid, leftEnd};
            params->track->pointsEdgeLeft = Bezier(0.02f, leftPoints);
        }
        // forceLeftTimer > 0 时左边缘由 handle() 的强制左转块设置，这里跳过
    }
    else
    {
        // 右分支：左边缘 = 岛右边界(尖↑) + 虚拟引导边线(尖↓→左下方)。
        // 从 V 尖向右寻找第一个白点，保证岛段取的是右支路左边界，
        // 避免通用 Track 把 V 尖左侧的边线误当成右支路左边线而使车身偏左。
        vector<PointX> island;
        for (int row = vRow; row >= ROWSIMAGE / 4; row--)
        {
            int rightEdge = -1;
            for (int col = vCol; col < COLSIMAGE; col++)
            {
                if (img.at<uchar>(row, col) > 128)
                {
                    rightEdge = col;
                    break;
                } // 第一个白点即停，避免跳到远处框上
            }
            if (rightEdge >= 0)
            {
                if (!island.empty())
                {
                    int prev = island.back().y;
                    if (abs(rightEdge - prev) > 8)
                        rightEdge = prev;
                }
                island.push_back(PointX(row, rightEdge));
            }
        }
        reverse(island.begin(), island.end());

        // 直线屏障：从V尖到底部左侧（镜像左分支）
        vector<PointX> barrier;
        int endCol = (int)(COLSIMAGE * 0.35f);
        int steps = 15;
        for (int i = 1; i <= steps; i++)
        {
            float t = (float)i / steps;
            barrier.push_back(PointX(
                vRow + (ROWSIMAGE - 10 - vRow) * t,
                vCol + (endCol - vCol) * t));
        }

        params->track->pointsEdgeLeft = island;
        params->track->pointsEdgeLeft.insert(params->track->pointsEdgeLeft.end(),
                                             barrier.begin(), barrier.end());
    }
}

void FsmYfork::drawTip(Mat &img)
{
    if (tipRow > 0 && tipCol > 0)
        circle(img, Point(tipCol, tipRow), 4, Scalar(0, 0, 255), -1);
}
