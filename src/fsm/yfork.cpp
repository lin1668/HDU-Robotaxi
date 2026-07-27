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
        ylog("[Yfork] run: yforkReset received -> reset()");
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
    // forceLeftDone 不在这里清零，只在 resetLap() 清零，防止同圈重复触发
    params->stationStopCompleted = false;
    params->stationStarted = false;
    params->yforkGuiding = false;
    params->yforkBranch = 0;
}

void FsmYfork::resetLap()
{
    reset();
    completed = false; // 新圈重新检测
    forceLeftDone = false; // 新一圈重新允许强制左转
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
                            params->config.currentLapConfig->yfork &&
                            params->config.currentLapConfig->yforkLeft &&
                            !forceLeftDone;
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
                    if (params->results[i].height > 15 && params->results[i].width > 15)
                    {
                        selectLeft = true;
                        params->yforkBranch = 1;
                        forceLeftTimer = FORCE_LEFT_FRAMES;
                        forceLeftDone = true; // 标记已执行，防止重复触发
                        params->yforkGuiding = true;
                        step = Step::ENTER;
                        counterYfork = 0;
                        timeout = 0;
                        ylog("[Yfork] NONE: FORCE LEFT TRIGGER station box #%d cx=%d y=%d h=%d w=%d bottom=%d, forceFrames=%d -> ENTER (one-shot)",
                             stationCount, boxCx, params->results[i].y,
                             params->results[i].height, params->results[i].width, boxBottom, FORCE_LEFT_FRAMES);
                        return true;
                    }
                    else
                    {
                        ylog("[Yfork] NONE: station box #%d cx=%d too small h=%d w=%d, skip",
                             stationCount, boxCx, params->results[i].height, params->results[i].width);
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

        if (detectYfork(img))
        {
            forkSeen = true;
            params->yforkGuiding = true; // 检测到fork，阻止station
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
        params->yforkGuiding = true; // 进入引导前阻止station

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

            ylog("[Yfork] ENTER: FORCE LEFT frame=%d/%d (remaining=%d), Bezier left edge start=(%d,%d) end=(%d,%d)",
                 FORCE_LEFT_FRAMES - forceLeftTimer, FORCE_LEFT_FRAMES, forceLeftTimer,
                 leftStart.x, leftStart.y, leftEnd.x, leftEnd.y);

            if (forceLeftTimer == 0)
                ylog("[Yfork] ENTER: FORCE LEFT FINISHED, normal guidance resumes");
        }

        // 引导期间屏蔽station检测，但V尖消失后放开让station能检测停车框
        params->yforkGuiding = (holdRow > 0) && (!vloss || vlossTimer < 5);

        // 左岔路：V尖消失后左边线突变 → 已右拐驶出岔路
        //   - 当前圈启用了station时：阻止突变退出，等先停好车
        //   - 强制左转期间不检测突变（避免贝塞尔→真实边缘跳变导致假退出）
        bool stationEnabled = params->config.currentLapConfig->station;
        bool stationBusy = stationEnabled && !params->stationStopCompleted;
        if (forceLeftTimer == 0 && !stationBusy && selectLeft && tipRow == 0 && params->track->pointsEdgeLeft.size() > 4)
        {
            int cur = params->track->pointsEdgeLeft.back().y;
            if (countRes > 0 && abs(cur - countRes) > 25)
            {
                ylog("[Yfork] ENTER: LEFT edge MUTATION cur=%d prev=%d diff=%d -> EXIT",
                     cur, countRes, abs(cur - countRes));
                reset();
                completed = true;
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
                reset();
                completed = true;
                return true;
            }
            countRes = cur;
        }

        // 超时退出（启用了station等多等帧等停车+突变）
        int exitTimeout = stationEnabled ? 300 : 120;
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
                bottom > ROWSIMAGE * 0.2)
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

    // V尖消失后保持引导0.6秒（18帧）
    if (vRow == 0 || vCol == 0)
    {
        if (holdRow > 0 && vlossTimer < 18)
        {
            vlossTimer++;
            vRow = holdRow;
            vCol = holdCol;
        }
        else
        {
            if (holdRow > 0)
                ylog("[Yfork] replan: hold expired (vlossTimer=%d >= 18), releasing guidance", vlossTimer);
            holdRow = 0;
            holdCol = 0;
            return;
        }
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
}

void FsmYfork::drawTip(Mat &img)
{
    if (tipRow > 0 && tipCol > 0)
        circle(img, Point(tipCol, tipRow), 4, Scalar(0, 0, 255), -1);
}
