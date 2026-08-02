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
 * @file center.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 车辆图像控制中心计算
 * @version 0.1
 * @date 2025-07-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "ctrl/center.hpp"

using namespace cv;
using namespace std;

/**
 * @brief 控制中心计算
 *
 * @param params
 */
void Center::fitting(shared_ptr<Params> &params)
{

    sigmaCenter = 0;
    params->ctrl.center = COLSIMAGE / 2; // 控制中心
    if (!params->ctrl.fitting)           // 除停车场特殊绘制
    {
        params->ctrl.centerEdge.clear();
        vector<PointX> v_center(4); // 三阶贝塞尔曲线
        style = "STRIGHT";

        // 边缘斜率重计算（边缘修正之后）
        params->track->stdevLeft = params->track->stdevEdgeCal(params->track->pointsEdgeLeft, ROWSIMAGE);
        params->track->stdevRight = params->track->stdevEdgeCal(params->track->pointsEdgeRight, ROWSIMAGE);

        // 边缘有效行优化
        if ((params->track->stdevLeft < 80 && params->track->stdevRight > 30) || (params->track->stdevLeft > 60 && params->track->stdevRight < 50))
        {
            validRowsCal(params->track->pointsEdgeLeft, params->track->pointsEdgeRight); // 边缘有效行计算
            params->track->pointsEdgeLeft.resize(validRowsLeft);
            params->track->pointsEdgeRight.resize(validRowsRight);
        }

        if (params->track->pointsEdgeLeft.size() > 4 && params->track->pointsEdgeRight.size() > 4) // 通过双边缘有效点的差来判断赛道类型
        {
            v_center[0] = {
                (params->track->pointsEdgeLeft[0].x + params->track->pointsEdgeRight[0].x) / 2,
                (params->track->pointsEdgeLeft[0].y + params->track->pointsEdgeRight[0].y) / 2};

            v_center[1] = {
                (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() / 3].x +
                 params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() / 3].x) /
                    2,
                (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() / 3].y +
                 params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() / 3].y) /
                    2};

            v_center[2] = {
                (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.5].x +
                 params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.5].x) /
                    2,
                (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.5].y +
                 params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.5].y) /
                    2};

            v_center[3] = {
                (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.75].x +
                 params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.75].x) /
                    2,
                (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 0.75].y +
                 params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 0.75].y) /
                    2};

            params->ctrl.centerEdge = Bezier(0.03, v_center);
            style = "STRIGHT";
        }
        // 左单边
        else if ((params->track->pointsEdgeLeft.size() > 0 && params->track->pointsEdgeRight.size() <= 4) ||
                 (params->track->pointsEdgeLeft.size() > 0 && params->track->pointsEdgeRight.size() > 0 &&
                  params->track->pointsEdgeLeft[0].x - params->track->pointsEdgeRight[0].x > ROWSIMAGE / 2))
        {
            style = "RIGHT";
            params->ctrl.centerEdge = centerCompute(params->track->pointsEdgeLeft, 0);
        }
        // 右单边
        else if ((params->track->pointsEdgeRight.size() > 0 && params->track->pointsEdgeLeft.size() <= 4) ||
                 (params->track->pointsEdgeRight.size() > 0 && params->track->pointsEdgeLeft.size() > 0 &&
                  params->track->pointsEdgeRight[0].x - params->track->pointsEdgeLeft[0].x > ROWSIMAGE / 2))
        {
            style = "LEFT";
            params->ctrl.centerEdge = centerCompute(params->track->pointsEdgeRight, 1);
        }
        else if (params->track->pointsEdgeLeft.size() > 4 && params->track->pointsEdgeRight.size() == 0) // 左单边
        {
            v_center[0] = {params->track->pointsEdgeLeft[0].x, (params->track->pointsEdgeLeft[0].y + COLSIMAGE - 1) / 2};

            v_center[1] = {params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() / 3].x,
                           (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() / 3].y + COLSIMAGE - 1) / 2};

            v_center[2] = {
                params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 2 / 3].x,
                (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() * 2 / 3].y + COLSIMAGE - 1) / 2};

            v_center[3] = {params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1].x,
                           (params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1].y + COLSIMAGE - 1) / 2};

            params->ctrl.centerEdge = Bezier(0.02, v_center);
            style = "RIGHT";
        }
        else if (params->track->pointsEdgeLeft.size() == 0 &&
                 params->track->pointsEdgeRight.size() > 4) // 右单边
        {
            v_center[0] = {params->track->pointsEdgeRight[0].x, params->track->pointsEdgeRight[0].y / 2};

            v_center[1] = {params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() / 3].x,
                           params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() / 3].y / 2};

            v_center[2] = {
                params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 2 / 3].x,
                params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() * 2 / 3].y / 2};

            v_center[3] = {params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1].x,
                           params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1].y / 2};

            params->ctrl.centerEdge = Bezier(0.02, v_center);
            style = "LEFT";
        }
    }

    // 加权控制中心计算
    int controlNum = 1;
    for (auto p : params->ctrl.centerEdge)
    {
        if (p.x < ROWSIMAGE / 2)
        {
            controlNum += ROWSIMAGE / 2;
            params->ctrl.center += p.y * ROWSIMAGE / 2;
        }
        else
        {
            controlNum += (ROWSIMAGE - p.x);
            params->ctrl.center += p.y * (ROWSIMAGE - p.x);
        }
    }
    if (controlNum > 1)
    {
        params->ctrl.center = params->ctrl.center / controlNum;
    }

    // 中心外偏，让车靠外道跑，避免压内线（像素偏移量，越大越靠外）
    params->ctrl.center += 16;

    // YFork 左右分支分别调节向左压的偏移量
    static constexpr int YFORK_LEFT_CENTER_OFFSET = 16;
    // 右分支停车横向略偏左：仅在 YFork 右分支巡线时向右修正少量像素。
    static constexpr int YFORK_RIGHT_CENTER_OFFSET = 8;
    if (params->mode == FsmMode::YFORK)
    {
        if (params->yforkBranch == 1)
            params->ctrl.center -= YFORK_LEFT_CENTER_OFFSET;
        else if (params->yforkBranch == 2)
            params->ctrl.center -= YFORK_RIGHT_CENTER_OFFSET;
    }

    if (params->ctrl.center > COLSIMAGE)
        params->ctrl.center = COLSIMAGE;
    else if (params->ctrl.center < 0)
        params->ctrl.center = 0;

    // 控制率计算
    if (params->ctrl.centerEdge.size() > 20)
    {
        vector<PointX> centerV;
        int filt = params->ctrl.centerEdge.size() / 5;
        for (int i = filt; i < params->ctrl.centerEdge.size() - filt;
             i++) // 过滤中心点集前后1/5的诱导性
        {
            centerV.push_back(params->ctrl.centerEdge[i]);
        }
        sigmaCenter = sigma(centerV);
    }
    else
        sigmaCenter = 1000;

    // 车辆冲出赛道检测（手动接管/仿真模式时禁用）
    if (!params->ctrl.parking && !params->manualTakeover && !params->config.debug)
        derailmentCheck(params->track->pointsEdgeLeft, params->track->pointsEdgeRight);
}

/**
 * @brief 显示赛道线识别结果
 *
 * @param img 需要叠加显示的图像
 */
void Center::drawImage(shared_ptr<Params> &params, Mat &img)
{
    // 赛道边缘绘制
    for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
        circle(img, Point(params->track->pointsEdgeLeft[i].y, params->track->pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点

    for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
        circle(img, Point(params->track->pointsEdgeRight[i].y, params->track->pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点

    // 绘制中心点集
    for (int i = 0; i < params->ctrl.centerEdge.size(); i++)
        circle(img, Point(params->ctrl.centerEdge[i].y, params->ctrl.centerEdge[i].x), 1, Scalar(0, 0, 255), -1);

    // 绘制加权控制中心：方向
    Rect rect(params->ctrl.center, ROWSIMAGE - 20, 10, 20);
    rectangle(img, rect, Scalar(0, 0, 255), cv::FILLED);

    // 详细控制参数显示
    int dis = 20;
    string str;

    str = "Edge: " + doble2String(params->track->stdevLeft, 1) + " | " +
          doble2String(params->track->stdevRight, 1);
    putText(img, str, Point(COLSIMAGE - 100, 2 * dis), FONT_HERSHEY_PLAIN, 0.7, Scalar(0, 0, 255), 1); // 斜率：左|右

    putText(img, to_string(params->ctrl.center), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 40),
            FONT_HERSHEY_PLAIN, 1.2, Scalar(0, 0, 255), 1); // 中心

    // 绘制FSM状态
    showMode(img, params->mode);
}

/**
 * @brief 显示FSM状态
 *
 * @param img
 * @param mode
 */
void Center::showMode(Mat &img, FsmMode mode)
{
    std::string str = "", logo = "";
    switch (mode)
    {
    case FsmMode::FORK:
        str = "[Fork]";
        logo = "F";
        break;
    case FsmMode::PARK:
        str = "[Park]";
        logo = "P";
        break;
    case FsmMode::BUSY:
        str = "[Busy]";
        logo = "B";
        break;
    case FsmMode::SLOW:
        str = "[Slow]";
        logo = "S";
        break;
    case FsmMode::CURVE:
        str = "[Curve]";
        logo = "C";
        break;
    case FsmMode::FINE:
        str = "[Fine]";
        logo = "X";
        break;
    case FsmMode::STOP:
        str = "[Stop]";
        logo = "X";
        break;
    case FsmMode::CROSS:
        str = "[Cross]";
        logo = "X";
        break;
    }

    if (mode != FsmMode::NORMAL)
    {
        putText(img, str, Point(COLSIMAGE / 2 - 50, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
        putText(img, logo, Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27), FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
    }
    else
    {
        putText(img, "[Normal]", Point(COLSIMAGE / 2 - 50, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
    }
}

/**
 * @brief 搜索十字赛道突变行（左下）
 *
 * @param pointsEdgeLeft
 * @return uint16_t
 */
uint16_t Center::searchBreakLeftDown(vector<PointX> pointsEdgeLeft)
{
    uint16_t counter = 0;

    for (int i = 0; i < pointsEdgeLeft.size() - 10; i++)
    {
        if (pointsEdgeLeft[i].y >= 2)
        {
            counter++;
            if (counter > 3)
            {
                return i - 2;
            }
        }
        else
            counter = 0;
    }

    return 0;
}

/**
 * @brief 搜索十字赛道突变行（右下）
 *
 * @param pointsEdgeRight
 * @return uint16_t
 */
uint16_t Center::searchBreakRightDown(vector<PointX> pointsEdgeRight)
{
    uint16_t counter = 0;

    for (int i = 0; i < pointsEdgeRight.size() - 10; i++) // 寻找左边跳变点
    {
        if (pointsEdgeRight[i].y < COLSIMAGE - 2)
        {
            counter++;
            if (counter > 3)
            {
                return i - 2;
            }
        }
        else
            counter = 0;
    }

    return 0;
}

/**
 * @brief 赛道中心点计算：单边控制
 *
 * @param pointsEdge 赛道边缘点集
 * @param side 单边类型：左边0/右边1
 * @return vector<PointX>
 */
vector<PointX> Center::centerCompute(vector<PointX> pointsEdge, int side)
{
    int step = 4;                    // 间隔尺度
    int offsetWidth = COLSIMAGE / 2; // 首行偏移量
    int offsetHeight = 0;            // 纵向偏移量

    vector<PointX> center; // 控制中心集合

    if (side == 0) // 左边缘
    {
        uint16_t counter = 0, rowStart = 0;
        for (int i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
        {
            if (pointsEdge[i].y > 1)
            {
                counter++;
                if (counter > 2)
                {
                    rowStart = i - 2;
                    break;
                }
            }
            else
                counter = 0;
        }

        offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
        counter = 0;
        for (int i = rowStart; i < pointsEdge.size(); i += step)
        {
            int py = pointsEdge[i].y + offsetWidth;
            if (py > COLSIMAGE - 1)
            {
                counter++;
                if (counter > 2)
                    break;
            }
            else
            {
                counter = 0;
                center.emplace_back(pointsEdge[i].x - offsetHeight, py);
            }
        }
    }
    else if (side == 1) // 右边沿
    {
        uint16_t counter = 0, rowStart = 0;
        for (int i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
        {
            if (pointsEdge[i].y < COLSIMAGE - 1)
            {
                counter++;
                if (counter > 2)
                {
                    rowStart = i - 2;
                    break;
                }
            }
            else
                counter = 0;
        }

        offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
        counter = 0;
        for (int i = rowStart; i < pointsEdge.size(); i += step)
        {
            int py = pointsEdge[i].y - offsetWidth;
            if (py < 1)
            {
                counter++;
                if (counter > 2)
                    break;
            }
            else
            {
                counter = 0;
                center.emplace_back(pointsEdge[i].x - offsetHeight, py);
            }
        }
    }

    return center;
    // return Bezier(0.2,center);
}

/**
 * @brief 边缘有效行计算：左/右
 *
 * @param pointsEdgeLeft
 * @param pointsEdgeRight
 */
void Center::validRowsCal(vector<PointX> pointsEdgeLeft, vector<PointX> pointsEdgeRight)
{
    int counter = 0;
    if (pointsEdgeRight.size() > 10 && pointsEdgeLeft.size() > 10)
    {
        uint16_t rowBreakLeft = searchBreakLeftDown(pointsEdgeLeft);                                           // 右边缘上升拐点
        uint16_t rowBreakRight = searchBreakRightDown(pointsEdgeRight);                                        // 右边缘上升拐点
        if (pointsEdgeRight[pointsEdgeRight.size() - 1].y < COLSIMAGE / 2 && rowBreakRight - rowBreakLeft > 5) // 左弯道
        {
            if (pointsEdgeLeft.size() > rowBreakRight) // 左边缘有效行重新搜索
            {
                for (int i = rowBreakRight; i < pointsEdgeLeft.size(); i++)
                {
                    if (pointsEdgeLeft[i].y < 1)
                    {
                        counter++;
                        if (counter >= 3)
                        {
                            pointsEdgeLeft.resize(i - 3);
                        }
                    }
                    else
                        counter = 0;
                }
            }
        }

        else if (pointsEdgeLeft[pointsEdgeLeft.size() - 1].y > COLSIMAGE / 2 && rowBreakLeft - rowBreakRight > 5) // 右弯道
        {

            if (pointsEdgeRight.size() > rowBreakLeft) // 右边缘有效行重新搜索
            {
                for (int i = rowBreakLeft; i < pointsEdgeRight.size(); i++)
                {
                    if (pointsEdgeRight[i].y > COLSIMAGE - 2)
                    {
                        counter++;
                        if (counter >= 3)
                        {
                            pointsEdgeRight.resize(i - 3);
                        }
                    }
                    else
                        counter = 0;
                }
            }
        }
    }

    // 左边有效行
    validRowsLeft = 0;
    int count = 0;
    if (pointsEdgeLeft.size() > 10)
    {
        for (int i = pointsEdgeLeft.size() - 1; i >= 1; i--)
        {
            if (pointsEdgeLeft[i].y > 2)
            {
                count++;
                if (count > 4)
                {
                    validRowsLeft = i + 4;
                    break;
                }
            }
        }
    }

    // 右边有效行
    validRowsRight = 0;
    if (pointsEdgeRight.size() > 1)
    {
        for (int i = pointsEdgeRight.size() - 1; i >= 1; i--)
        {
            if (pointsEdgeRight[i].y <= COLSIMAGE - 2 &&
                pointsEdgeRight[i - 1].y <= COLSIMAGE - 2)
            {
                validRowsRight = i + 1;
                break;
            }
            if (pointsEdgeRight[i].y >= COLSIMAGE - 2 &&
                pointsEdgeRight[i - 1].y < COLSIMAGE - 2)
            {
                validRowsRight = i + 1;
                break;
            }
        }
    }
}

/**
 * @brief 车辆冲出赛道检测（保护车辆）
 *
 * @param pointsEdgeLeft
 * @param pointsEdgeRight
 * @return true
 * @return false
 */
void Center::derailmentCheck(vector<PointX> pointsEdgeLeft, vector<PointX> pointsEdgeRight)
{
    if (pointsEdgeLeft.size() < 30 && pointsEdgeRight.size() < 30) // 防止车辆冲出赛道
    {
        countOut++;
        timeout = 0;
        if (countOut > 20)
        {
            // 每秒最多输出一次，避免日志泛滥
            static int outlineLogThrottle = 0;
            if (outlineLogThrottle++ % 30 == 0)
                fprintf(stderr, "[FSM] ICAR Outline (count=%d)\n", countOut);
            // exit(0); // 程序退出（注释以继续运行调试）
        }
    }
    else
    {
        timeout++;
        if (timeout > 50)
        {
            countOut = 0;
            timeout = 50;
        }
    }
}
