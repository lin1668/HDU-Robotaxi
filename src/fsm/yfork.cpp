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
#include <cstdio>
#include <cstdarg>
#include <ctime>

// 日志输出到文件
static void ylog(const char *fmt, ...)
{
    static FILE *fp = nullptr;
    if (!fp) fp = fopen("./yfork.log", "w");
    if (!fp) return;
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

/**
 * @brief Construct a new Fsm Yfork
 *
 * @param par
 */
FsmYfork::FsmYfork(std::shared_ptr<Params> par)
    : FSMState(FsmMode::YFORK, par)
{
}

/**
 * @brief Destroy the Fsm Yfork
 *
 */
FsmYfork::~FsmYfork()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmYfork::getMode()
{
    if (!params->config.yfork || !params->config.currentLapConfig->yfork || !enable)
        return FsmMode::NORMAL;

    return FsmMode::YFORK;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmYfork::run(Mat &img)
{
    if (!params->config.yfork) // 该模式未启用
    {
        completed = false; // 失能时复位，确保下次使能时能重新检测
        return;
    }

    // park退出时通知yfork复位，防止残留forkSeen误触发
    if (params->ctrl.yforkReset)
    {
        reset();
        params->ctrl.yforkReset = false;
    }

    // 非NORMAL/YFORK模式下不干扰其他FSM（如停车区内的fork地面箭头）
    if (params->mode != FsmMode::NORMAL && params->mode != FsmMode::YFORK)
        return;

    enable = handle(img); // 处理Y型岔路口
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

    putText(img, "[9] Yfork", Point(COLSIMAGE / 2 - 50, 15),
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
    params->stationStopCompleted = false;
    params->stationStarted = false;
    params->yforkGuiding = false;
    params->yforkBranch = 0;
}

void FsmYfork::resetLap()
{
    reset();
    completed = false; // 新圈重新检测
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
    if (!params->config.currentLapConfig->yfork)
        return forkSeen; // 检测到fork期间就启用YFORK模式（蜂鸣器+减速）

    switch (step)
    {
    case Step::NONE:
    {
        if (detectYfork(img))
        {
            forkSeen = true;
            params->yforkGuiding = true; // 检测到fork，阻止station
            printf("[Yfork] NONE: fork detected, forkSeen=true\n");
            ylog("[Yfork] NONE: fork detected, forkSeen=true");
        }
        else if (forkSeen)
        {
            // fork已离开画面，开始找V尖
            if (findVTip(img))
            {
                step = Step::DECIDE;
                counterYfork = 0;
                timeout = 0;
                printf("[Yfork] V尖找到: row=%d col=%d -> DECIDE\n", tipRow, tipCol);
                ylog("[Yfork] V尖找到: row=%d col=%d -> DECIDE", tipRow, tipCol);
            }
            else
            {
                static int noVtipCount = 0;
                noVtipCount++;
                if (noVtipCount % 10 == 1)
                {
                    printf("[Yfork] NONE: forkSeen but no V-tip yet (cnt=%d)\n", noVtipCount);
                    ylog("[Yfork] NONE: forkSeen but no V-tip yet (cnt=%d)", noVtipCount);
                }
            }
        }

        // 仅左分支：fork箭头到底部时提前启动引导（不等V尖，右分支保持原逻辑）
        if (forkSeen && step == Step::NONE && params->config.currentLapConfig->yforkLeft)
        {
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_FORK &&
                    (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.55)
                {
                    selectLeft = true;
                    params->yforkBranch = 1;
                    // 用估算V尖位置提前启动
                    holdRow = (int)(ROWSIMAGE * 0.6f);
                    holdCol = (int)(COLSIMAGE * 0.12f);
                    vlossTimer = 0;
                    params->yforkGuiding = true;
                    step = Step::ENTER;
                    counterYfork = 0;
                    timeout = 0;
                    printf("[Yfork] NONE: fork at bottom, early ENTER hold=(%d,%d)\n", holdRow, holdCol);
                    ylog("[Yfork] NONE: fork at bottom, early ENTER hold=(%d,%d)", holdRow, holdCol);
                    break;
                }
            }
        }
        return forkSeen; // 检测到fork进入YFORK模式减速，引导在V尖找到后才开始
    }

    case Step::DECIDE:
    {
        selectLeft = params->config.currentLapConfig->yforkLeft;
        params->yforkBranch = selectLeft ? 1 : 2;
        params->yforkGuiding = true; // 进入引导前阻止station

        // 提前保存V尖位置，防止ENTER第一帧replanTracking时V尖已出画面
        holdRow = tipRow;
        holdCol = tipCol;
        vlossTimer = 0;

        printf("[Yfork] DECIDE: selectLeft=%d branch=%d -> ENTER\n", selectLeft, params->yforkBranch);
        ylog("[Yfork] DECIDE: selectLeft=%d branch=%d -> ENTER", selectLeft, params->yforkBranch);
        step = Step::ENTER;
        counterYfork = 0;
        timeout = 0;
        return true;
    }

    case Step::ENTER:
    {
        counterYfork++;
        timeout++;

        replanTracking(selectLeft, img);

        // 引导期间屏蔽station检测，但V尖消失后放开让station能检测停车框
        params->yforkGuiding = (holdRow > 0) && !vloss;

        // 每秒输出一次ENTER状态
        if (timeout % 30 == 1)
        {
            printf("[Yfork] ENTER frame=%d tip=(%d,%d) vloss=%d hold=(%d,%d) vlossTimer=%d guiding=%d\n",
                   timeout, tipRow, tipCol, vloss, holdRow, holdCol, vlossTimer, params->yforkGuiding);
            ylog("[Yfork] ENTER frame=%d tip=(%d,%d) vloss=%d hold=(%d,%d) vlossTimer=%d guiding=%d",
                 timeout, tipRow, tipCol, vloss, holdRow, holdCol, vlossTimer, params->yforkGuiding);
        }

        // 左岔路：V尖消失后左边线突变 → 已右拐驶出岔路
        //   - 当前圈启用了station时：阻止突变退出，等先停好车
        bool stationEnabled = params->config.currentLapConfig->station;
        bool stationBusy = stationEnabled && !params->stationStopCompleted;
        if (!stationBusy && selectLeft && tipRow == 0 && params->track->pointsEdgeLeft.size() > 4)
        {
            int cur = params->track->pointsEdgeLeft.back().y;
            if (countRes > 0 && abs(cur - countRes) > 25)
            {
                reset();
                completed = true;
                printf("[Yfork] 驶出岔路 (left edge)\n");
                ylog("[Yfork] 驶出岔路 (left edge)");
                return true;
            }
            countRes = cur;
        }

        // 右岔路：右边缘突变 → 已左拐驶出岔路
        if (!stationBusy && !selectLeft && tipRow == 0 && params->track->pointsEdgeRight.size() > 4)
        {
            int cur = params->track->pointsEdgeRight.back().y;
            if (countRes > 0 && abs(cur - countRes) > 25)
            {
                reset();
                completed = true;
                printf("[Yfork] 驶出岔路 (right edge)\n");
                ylog("[Yfork] 驶出岔路 (right edge)");
                return true;
            }
            countRes = cur;
        }

        // 超时退出（启用了station等多等帧等停车+突变）
        int exitTimeout = stationEnabled ? 200 : 120;
        if (timeout > exitTimeout)
        {
            printf("[Yfork] ENTER timeout=%d -> EXIT\n", timeout);
            ylog("[Yfork] ENTER timeout=%d -> EXIT", timeout);
            step = Step::EXIT;
            counterYfork = 0;
            timeout = 0;
        }
        return true;
    }

    case Step::EXIT:
    {
        printf("[Yfork] EXIT -> END\n");
        ylog("[Yfork] EXIT -> END");
        step = Step::END;
        return true;
    }

    case Step::END:
    {
        printf("[Yfork] END (completed)\n");
        ylog("[Yfork] END (completed)");
        reset();
        completed = true; // 完成一轮Y型岔路，防止停车区误触发
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
        return false;

    // 当前圈使能了停车场时：画面有PARK标志说明叉形箭头是车位标识，非Y型岔路
    // 当前圈未使能停车场时（如第二圈）：PARK标志不阻断Y型岔路检测
    if (params->config.park)
    {
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_PARK)
            {
                // printf("[Yfork] detect: blocked by PARK sign\n");
                return false;
            }
        }
    }

    for (int i = 0; i < params->results.size(); i++)
    {
        if (params->results[i].type == LABEL_FORK)
        {
            if (params->results[i].height < 150 && params->results[i].width < 150 &&
                params->results[i].height > 15 && params->results[i].width > 15 &&
                (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.35)
            {
                printf("[Yfork] detect: FORK found y=%d h=%d bottom=%.0f\n",
                       params->results[i].y, params->results[i].height,
                       (float)(params->results[i].y + params->results[i].height));
                ylog("[Yfork] detect: FORK found y=%d h=%d bottom=%.0f",
                     params->results[i].y, params->results[i].height,
                     (float)(params->results[i].y + params->results[i].height));
                return true;
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
            printf("[Yfork] V-tip lost (bottom): row=%d col=%d\n", tipRow, tipCol);
            ylog("[Yfork] V-tip lost (bottom): row=%d col=%d", tipRow, tipCol);
            tipRow = 0;
            tipCol = 0;
        }
        return true;
    }

    // 红点从有到无 → 标记消失，后续不再接受新红点
    if (tipRow > 0)
    {
        vloss = true;
        printf("[Yfork] V-tip lost (disappeared): last row=%d col=%d\n", tipRow, tipCol);
        ylog("[Yfork] V-tip lost (disappeared): last row=%d col=%d", tipRow, tipCol);
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
    findVTip(img); // 更新V尖状态（vloss等），但不用实时坐标引导

    int vRow = tipRow;
    int vCol = tipCol;

    // 有DECIDE缓存就用缓存，不信任实时spurroad脏数据
    if (holdRow > 0)
    {
        // 跟踪V尖丢失帧数，用于yforkGuiding和超时退出
        if (vRow == 0 || vCol == 0)
        {
            vlossTimer++;
            if (vlossTimer > 20)
            {
                printf("[Yfork] replan: guidance hold expired, releasing\n");
                ylog("[Yfork] replan: guidance hold expired, releasing");
                holdRow = 0;
                holdCol = 0;
                return;
            }
        }
        // 始终用缓存的坐标，不被实时V尖覆盖
        vRow = holdRow;
        vCol = holdCol;
    }
    else if (vRow > 0 && vCol > 0)
    {
        // 没有缓存：首次检测到V尖，保存坐标
        holdRow = vRow;
        holdCol = vCol;
        vlossTimer = 0;
    }
    else
    {
        // 没有V尖，也没有缓存
        return;
    }

    if (left)
    {
        // 左分支：不用V尖扫岛（spurroad不可靠），直接用固定屏障 + 贝塞尔左转
        // V尖列坐标限幅，防止spurroad脏数据导致屏障太靠左或太靠右
        if (vCol < 40) vCol = 40;
        if (vCol > (int)(COLSIMAGE * 0.45f)) vCol = (int)(COLSIMAGE * 0.45f);

        // 右边缘：从V尖列到底部的斜线屏障（镜像右分支逻辑）
        params->track->pointsEdgeRight.clear();
        vector<PointX> barrier;
        int endCol = (int)(COLSIMAGE * 0.45f);
        int steps = 20;
        for (int i = 1; i <= steps; i++)
        {
            float t = (float)i / steps;
            barrier.push_back(PointX(
                vRow + (ROWSIMAGE - 10 - vRow) * t,
                vCol + (endCol - vCol) * t));
        }
        params->track->pointsEdgeRight = barrier;

        // 左边缘：前半段贝塞尔引导，后半段交还真实循线
        if (vlossTimer < 10)
        {
            PointX leftStart = PointX(ROWSIMAGE - 10, 15);
            PointX leftEnd = PointX(ROWSIMAGE / 3, 10);
            PointX leftMid = PointX((leftStart.x + leftEnd.x) * 0.3f, (leftStart.y + leftEnd.y) * 0.5f);
            vector<PointX> leftPoints = {leftStart, leftMid, leftEnd};
            params->track->pointsEdgeLeft = Bezier(0.02f, leftPoints);
        }
        // 后半段不设左边缘，让真实循线接管
    }
    else
    {
        // 右分支：左边缘 = 岛右边界(尖↑) + 直线屏障(尖↓→左下角) (镜像左分支)
        // 岛在二值图中为黑，从V尖往右找第一个白点=赛道边线就是岛右边界
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
        int endCol = (int)(COLSIMAGE * 0.3f);
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
    printf("[Yfork] replan dir=%s v=(%d,%d)\n", left ? "L" : "R", vRow, vCol);
    ylog("[Yfork] replan dir=%s v=(%d,%d)", left ? "L" : "R", vRow, vCol);
}

void FsmYfork::drawTip(Mat &img)
{
    if (tipRow > 0 && tipCol > 0)
        circle(img, Point(tipCol, tipRow), 4, Scalar(0, 0, 255), -1);
}
