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
 * @file stop.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停车区规划与控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/stop.hpp"
#include <cstdarg>
#include <cstdio>
#include <ctime>

namespace
{
static void stopLog(const char *fmt, ...)
{
    static bool firstWrite = true;
    FILE *fp = fopen("./stop.log", firstWrite ? "w" : "a");
    firstWrite = false;
    if (!fp)
        return;

    time_t now = time(nullptr);
    tm *t = localtime(&now);
    if (t)
        fprintf(fp, "[%02d:%02d:%02d] ", t->tm_hour, t->tm_min, t->tm_sec);

    va_list args;
    va_start(args, fmt);
    vfprintf(fp, fmt, args);
    va_end(args);
    fprintf(fp, "\n");
    fclose(fp);
}
}

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmStop::FsmStop(std::shared_ptr<Params> par)
    : FSMState(FsmMode::STOP, par)
{
    stopLog("STOP LOG START");
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmStop::~FsmStop()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmStop::getMode()
{
    // 输出场景状态结果
    if (step == Step::NONE || !params->config.stop || !params->config.currentLapConfig->stop)
        return FsmMode::NORMAL;
    else
        return FsmMode::STOP;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmStop::run(Mat &img)
{
    if (!params->config.stop) // 该模式未启用
        return;

    polyRoad.clear();
    polyCar.clear();

    switch (step)
    {
    case Step::NONE: // AI未识别
    {
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_GATE) // 障碍物：道闸
            {
                const int bottom = params->results[i].y + params->results[i].height;
                stopLog("GATE_NONE lap=%d mode=%d idx=%d x=%d y=%d w=%d h=%d bottom=%d score=%.3f countRec=%u countSes=%u results=%zu",
                        params->currentLap, static_cast<int>(params->mode), i,
                        params->results[i].x, params->results[i].y,
                        params->results[i].width, params->results[i].height,
                        bottom, params->results[i].score,
                        static_cast<unsigned>(countRec),
                        static_cast<unsigned>(countSes),
                        params->results.size());
                countRec++;
                break;
            }
        }

        if (countRec > 2)
        {
            stopLog("STEP NONE->ENABLE lap=%d countRec=%u countSes=%u results=%zu",
                    params->currentLap, static_cast<unsigned>(countRec),
                    static_cast<unsigned>(countSes), params->results.size());
            setStep(Step::ENABLE); // 设置新状态
        }

        if (countRec > 0) // 识别AI标志后开始场次计数
        {
            countSes++;
            if (countSes > 4)
            {
                stopLog("RESET NONE gate_not_continuous lap=%d countRec=%u countSes=%u results=%zu",
                        params->currentLap, static_cast<unsigned>(countRec),
                        static_cast<unsigned>(countSes), params->results.size());
                setStep(Step::NONE);
            }
        }
        break;
    }

    case Step::ENABLE: // 场景使能
    {
        countSes++; // 场次计数器
        timeout++;
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_GATE) // 障碍物：道闸
            {
                countSes = 0;
                timeout = 0;
                // 停车场入口需要给入库规划预留距离；普通道闸仍保持原来的停车距离。
                const bool parkRoute = params->config.currentLapConfig && params->config.currentLapConfig->park;
                const float stopRatio = parkRoute ? 0.36f : 0.42f;
                const int bottom = params->results[i].y + params->results[i].height;
                const int threshold = (int)(ROWSIMAGE * stopRatio);
                const bool pass = bottom > threshold;
                stopLog("GATE_ENABLE lap=%d mode=%d idx=%d x=%d y=%d w=%d h=%d bottom=%d threshold=%d pass=%d score=%.3f countRec=%u countSes=%u timeout=%d parkRoute=%d results=%zu",
                        params->currentLap, static_cast<int>(params->mode), i,
                        params->results[i].x, params->results[i].y,
                        params->results[i].width, params->results[i].height,
                        bottom, threshold, pass, params->results[i].score,
                        static_cast<unsigned>(countRec),
                        static_cast<unsigned>(countSes), timeout, parkRoute,
                        params->results.size());
                if ((params->results[i].y + params->results[i].height) > ROWSIMAGE * stopRatio) // 停车距离计算
                {
                    countRec++;
                    break;
                }
                break;
            }
        }
        if (countRec > 2)
        {
            stopLog("STEP ENABLE->STOP lap=%d countRec=%u countSes=%u timeout=%d results=%zu",
                    params->currentLap, static_cast<unsigned>(countRec),
                    static_cast<unsigned>(countSes), timeout, params->results.size());
            setStep(Step::STOP); // 设置新状态
        }
        if (countSes >= 10 || timeout > 50)
        {
            stopLog("RESET ENABLE timeout lap=%d countRec=%u countSes=%u timeout=%d results=%zu",
                    params->currentLap, static_cast<unsigned>(countRec),
                    static_cast<unsigned>(countSes), timeout, params->results.size());
            setStep(Step::NONE); // 设置新状态
        }
        break;
    }

    case Step::STOP: // 停车
    {
        params->ctrl.stop = true; // 停车标志
        countSes++;               // 场次计数器
        if (countSes == 1 || countSes % 5 == 0)
            stopLog("STOP_HOLD lap=%d mode=%d countSes=%u countRec=%u timeout=%d ctrlStop=%d speed=%.2f results=%zu",
                    params->currentLap, static_cast<int>(params->mode),
                    static_cast<unsigned>(countSes),
                    static_cast<unsigned>(countRec), timeout, params->ctrl.stop,
                    params->ctrl.speed, params->results.size());
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_GATE) // 障碍物：道闸
            {
                const int bottom = params->results[i].y + params->results[i].height;
                stopLog("GATE_STOP_HOLD lap=%d mode=%d idx=%d x=%d y=%d w=%d h=%d bottom=%d score=%.3f resetHold=1 results=%zu",
                        params->currentLap, static_cast<int>(params->mode), i,
                        params->results[i].x, params->results[i].y,
                        params->results[i].width, params->results[i].height,
                        bottom, params->results[i].score, params->results.size());
                countSes = 0;
                break;
            }
        }
        if (countSes >= 30)
        {
            stopLog("STOP_RELEASE lap=%d countSes=%u countRec=%u timeout=%d ctrlStopBefore=%d results=%zu",
                    params->currentLap, static_cast<unsigned>(countSes),
                    static_cast<unsigned>(countRec), timeout,
                    params->ctrl.stop, params->results.size());
            setStep(Step::NONE); // 设置新状态
            params->ctrl.stop = false;
        }

        break;
    }
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmStop::show(Mat &img)
{
    if (params->mode != FsmMode::STOP)
        return;

    putText(img, "[7] Stop", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    switch (step)
    {
    case Step::ENABLE: // 场景使能
        putText(img, "[7] STOP - ENABLE", Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;

    case Step::STOP: // 停车
        putText(img, "[7] STOP - STOPING", Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;
    }

    if (polyRoad.size() > 0 && polyCar.size() > 0)
    {
        drawPolygon(img, polyRoad, false, true); // 绘制赛道多边形
        drawPolygon(img, polyCar, false, true);  // 绘制车辆多边形

        putText(img, "Overlap:" + doble2String(overlap, 2),
                Point(100, ROWSIMAGE - 60), FONT_HERSHEY_TRIPLEX, 0.5,
                Scalar(0, 0, 255), 0.5);
    }
}

/**
 * @brief 计算多边形重合度
 *
 * @param track
 * @param result
 */
double FsmStop::getRoadCarPloy(PredictResult result)
{
    if (params->track->pointsEdgeLeft.size() < ROWSIMAGE / 5 && params->track->pointsEdgeRight.size() < ROWSIMAGE / 5)
        return 0.0;

    // 添加赛道多边形轮廓
    PointX ppoliy;
    if (params->track->pointsEdgeLeft.size() < 5 && params->track->pointsEdgeRight.size() > 5) // 右单边
    {
        // [1]左上点
        ppoliy.x = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1].x;
        ppoliy.y = 1;
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        // [2]左下点
        ppoliy.x = params->track->pointsEdgeRight[0].x;
        ppoliy.y = 1;
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 2
    }
    else
    {
        // [1]左上点
        ppoliy = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        ppoliy = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.8];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        ppoliy = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.6];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        ppoliy = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.4];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        ppoliy = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.2];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 1
        // [2]左下点
        ppoliy = params->track->pointsEdgeLeft[0];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x)); // 2
    }

    if (params->track->pointsEdgeLeft.size() > 5 && params->track->pointsEdgeRight.size() < 5) // 左单边
    {
        //[3] 右下点
        ppoliy.x = params->track->pointsEdgeLeft[0].x;
        ppoliy.y = COLSIMAGE - 1;

        //[4] 右上点
        ppoliy.x = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1].x;
        ppoliy.y = COLSIMAGE - 1;
    }
    else
    {
        //[3] 右下点
        ppoliy = params->track->pointsEdgeRight[0];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
        ppoliy = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.2];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
        ppoliy = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.4];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
        ppoliy = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.6];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
        ppoliy = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.8];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
        //[4] 右上点
        ppoliy = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1];
        polyRoad.push_back(cv::Point(ppoliy.y, ppoliy.x));
    }

    // 添加车辆多边形轮廓
    polyCar.push_back(cv::Point(result.x, result.y));
    polyCar.push_back(cv::Point(result.x, result.y + result.height));
    polyCar.push_back(cv::Point(result.x + result.width, result.y + result.height));
    polyCar.push_back(cv::Point(result.x + result.width, result.y));

    return getOverlapArea(polyRoad, polyCar); // 计算多边形重合度
}

/**
 * @brief 计算两个多边形图形的面积重叠度
 *
 * @param polyA 多边形顶点
 * @param polyB
 * @return double [0,1]
 */
double FsmStop::getOverlapArea(const std::vector<cv::Point> &polyA, const std::vector<cv::Point> &polyB)
{
    // 创建两个多边形的掩模
    cv::Mat maskA = cv::Mat::zeros(500, 500, CV_8UC1);
    cv::Mat maskB = cv::Mat::zeros(500, 500, CV_8UC1); // 智能车

    // 填充多边形
    cv::fillConvexPoly(maskA, polyA, cv::Scalar(255));
    cv::fillConvexPoly(maskB, polyB, cv::Scalar(255));

    // 计算交集
    cv::Mat intersection;
    cv::bitwise_and(maskA, maskB, intersection);

    // 计算交集的面积
    double areaB = cv::countNonZero(maskB);
    double areaAnd = cv::countNonZero(intersection);

    // 计算重合度
    double overlap = (areaB > 0) ? (areaAnd / areaB) : 0;

    return overlap;
}

/**
 * @brief 绘制多边形
 *
 * @param img
 * @param poly 多边形点集
 * @param fill  填充
 * @param close 封闭图形
 */
void FsmStop::drawPolygon(Mat img, vector<Point> poly, bool fill, bool close)
{
    vector<vector<Point>> contours;
    contours.push_back(poly);

    if (fill)
        fillPoly(img, contours, Scalar(255, 255, 255), 8);
    else
        polylines(img, poly, close, Scalar(255, 255, 255), 2, 8);
}

/**
 * @brief 设置新状态
 *
 * @param step
 */
void FsmStop::setStep(Step st)
{
    step = st;
    countRec = 0; // AI场景识别计数器
    countSes = 0; // 场次计数器
    timeout = 0;  // 超时计数器
}

void FsmStop::resetLap()
{
    setStep(Step::NONE);
    polyRoad.clear();
    polyCar.clear();
    overlap = 0;
    scoreLap = 0.5;
}
