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
 * @file center.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 车辆图像控制中心计算
 * @version 0.1
 * @date 2025-07-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <cmath>
#include <numeric>
#include "utils/tools.hpp"
#include "utils/params.hpp"

#define DIS_MOVE 48       // 偏移距离，对应赛道距离的一般，在我的打表软件中默认48像素对应20cm
#define MAX_POINT_NUM 240 // 无需修改
#define DIS_SECTION 75    // 使用centerMove时每一段点集的最大长度

/**
 * @brief 控制中心处理类
 *
 */
class Center
{

public:
    uint16_t validRowsLeft = 0;  // 边缘有效行数（左）
    uint16_t validRowsRight = 0; // 边缘有效行数（右）
    double sigmaCenter = 0;      // 中心点集的方差

    void fitting(shared_ptr<Params> &params);
    void drawImage(shared_ptr<Params> &params, Mat &img);

private:
    string style = "";
    int countOut = 0;
    int timeout = 0;

    void showMode(Mat &img, FsmMode mode);
    uint16_t searchBreakLeftDown(vector<PointX> pointsEdgeLeft);
    uint16_t searchBreakRightDown(vector<PointX> pointsEdgeRight);
    vector<PointX> centerCompute(vector<PointX> pointsEdge, int side);
    void validRowsCal(vector<PointX> pointsEdgeLeft, vector<PointX> pointsEdgeRight);
    void derailmentCheck(vector<PointX> pointsEdgeLeft, vector<PointX> pointsEdgeRight);
};