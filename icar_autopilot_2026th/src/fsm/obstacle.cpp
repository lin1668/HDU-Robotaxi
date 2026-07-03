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
 * @file obstacle.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 全局障碍物检测（锥桶/行人）
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/obstacle.hpp"
#include "utils/tools.hpp"

FsmObstacle::FsmObstacle(std::shared_ptr<Params> par)
    : params(par)
{
}

FsmObstacle::~FsmObstacle()
{
}

void FsmObstacle::run(Mat &img)
{
    resultObs = PredictResult();

    if (params->track->pointsEdgeLeft.size() < ROWSIMAGE / 2 ||
        params->track->pointsEdgeRight.size() < ROWSIMAGE / 2)
        return;

    // 锥桶 + 行人检测
    vector<PredictResult> resultsObs;
    for (int i = 0; i < params->results.size(); i++)
    {
        if ((params->results[i].type == LABEL_CONE || params->results[i].type == LABEL_PERSON) &&
            (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.4 &&
            params->results[i].height < 100 && params->results[i].width < 90 &&
            params->results[i].height > 20 && params->results[i].width > 20)
            resultsObs.push_back(params->results[i]);
    }
    if (resultsObs.size() <= 0)
        return;

    // 选取距离最近的障碍物
    int areaMax = 0;
    int index = 0;
    for (int i = 0; i < resultsObs.size(); i++)
    {
        int area = resultsObs[i].width * resultsObs[i].height;
        if (area >= areaMax)
        {
            index = i;
            areaMax = area;
        }
    }
    resultObs = resultsObs[index];

    // 障碍物方向判定（左/右）
    int row = 0, width = COLSIMAGE;
    for (size_t i = 0; i < params->track->pointsEdgeLeft.size(); i++)
    {
        int w = abs(resultObs.y - params->track->pointsEdgeLeft[i].x);
        if (w < 2)
        {
            row = i;
            break;
        }
        if (w < width)
        {
            width = w;
            row = i;
        }
    }
    if (row > params->track->pointsEdgeRight.size() - 1)
        row = params->track->pointsEdgeRight.size() - 1;

    // 路径重规划
    int disLeft = resultsObs[index].x - params->track->pointsEdgeLeft[row].y;
    int disRight = params->track->pointsEdgeRight[row].y - (resultsObs[index].x + resultsObs[index].width);
    if (resultsObs[index].x + resultsObs[index].width > params->track->pointsEdgeLeft[row].y &&
        params->track->pointsEdgeRight[row].y > resultsObs[index].x &&
        abs(disLeft) <= abs(disRight)) //[1] 障碍物靠左
    {
        if (resultsObs[index].type == LABEL_PERSON) // 行人避障
            curtailTracking(false);                 // 缩减优化车道线（双车道→单车道）
        else
        {
            vector<PointX> points(4); // 三阶贝塞尔曲线
            points[0] = params->track->pointsEdgeLeft[row / 2];
            points[1] = {resultsObs[index].y + resultsObs[index].height, resultsObs[index].x + resultsObs[index].width * 2};
            points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2, resultsObs[index].x + resultsObs[index].width * 2};
            if (resultsObs[index].y > params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1].x)
                points[3] = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1];
            else
                points[3] = {resultsObs[index].y, resultsObs[index].x + resultsObs[index].width};

            params->track->pointsEdgeLeft.resize((size_t)row / 2); // 删除错误路线
            vector<PointX> repair = Bezier(0.01, points);          // 重新规划车道线
            for (int i = 0; i < repair.size(); i++)
                params->track->pointsEdgeLeft.push_back(repair[i]);
        }
        params->ctrl.slow = true; // 避障期间锁定限速，防止转向见解除限速标志提前加速
    }
    else if (resultsObs[index].x + resultsObs[index].width > params->track->pointsEdgeLeft[row].y &&
             params->track->pointsEdgeRight[row].y > resultsObs[index].x &&
             abs(disLeft) > abs(disRight)) //[2] 障碍物靠右
    {
        if (resultsObs[index].type == LABEL_PERSON) // 行人避障
            curtailTracking(true);                  // 缩减优化车道线（双车道→单车道）
        else
        {
            vector<PointX> points(4); // 三阶贝塞尔曲线
            points[0] = params->track->pointsEdgeRight[row / 2];
            points[1] = {resultsObs[index].y + resultsObs[index].height, resultsObs[index].x - resultsObs[index].width * 2};
            points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2, resultsObs[index].x - resultsObs[index].width * 2};
            if (resultsObs[index].y > params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1].x)
                points[3] = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1];
            else
                points[3] = {resultsObs[index].y, resultsObs[index].x};

            params->track->pointsEdgeRight.resize((size_t)row / 2); // 删除错误路线
            vector<PointX> repair = Bezier(0.01, points);           // 重新规划车道线
            for (int i = 0; i < repair.size(); i++)
                params->track->pointsEdgeRight.push_back(repair[i]);
        }
        params->ctrl.slow = true; // 避障期间锁定限速，防止转向见解除限速标志提前加速
    }

    // 车道线切除顶行1/5，避免弯道权重过大
    params->track->pointsEdgeLeft.resize(params->track->pointsEdgeLeft.size() * 0.7);
    params->track->pointsEdgeRight.resize(params->track->pointsEdgeRight.size() * 0.7);
}

void FsmObstacle::resetLap()
{
    resultObs = PredictResult();
}

void FsmObstacle::show(Mat &img)
{
    if (resultObs.x > 0 && resultObs.y > 0)
    {
        cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }
}

void FsmObstacle::curtailTracking(bool left)
{
    if (left) // 向左侧缩进
    {
        if (params->track->pointsEdgeRight.size() > params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeRight.resize(params->track->pointsEdgeLeft.size());

        for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
        {
            params->track->pointsEdgeRight[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
    else // 向右侧缩进
    {
        if (params->track->pointsEdgeRight.size() < params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeLeft.resize(params->track->pointsEdgeRight.size());

        for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
        {
            params->track->pointsEdgeLeft[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
}
