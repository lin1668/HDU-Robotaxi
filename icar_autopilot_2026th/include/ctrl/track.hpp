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
 * @file track.hpp
 * @author Leo (liaotengjun@saishukeji.com)
 * @brief 车道线检测
 * @version 0.1
 * @date 2025-07-13
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <cmath>
#include <numeric>
#include "utils/tools.hpp"

using namespace cv;
using namespace std;

class Track
{
public:
    vector<PointX> pointsEdgeLeft;  // 赛道左边缘点集
    vector<PointX> pointsEdgeRight; // 赛道右边缘点集
    vector<PointX> widthBlock;      // 色块宽度=终-起（每行）
    vector<PointX> spurroad;        // 保存岔路信息
    double stdevLeft;               // 边缘斜率方差（左）
    double stdevRight;              // 边缘斜率方差（右）
    int validRowsLeft = 0;          // 边缘有效行数（左）
    int validRowsRight = 0;         // 边缘有效行数（右）
    uint16_t rowCutUp = 1;          // 图像顶部切行
    uint16_t rowCutBottom = 20;     // 图像底部切行

    void handle(Mat img);
    void handle(bool isResearch, uint16_t rowStart);
    void drawImage(Mat &img);
    double stdevEdgeCal(vector<PointX> &v_edge, int img_height);
    void fillEdgeGap(vector<PointX> &edge); // 填补边线缺口（如施工区出口）

private:
    Mat imgShare; // 赛道搜索图像
    void slopeCal(vector<PointX> &edge, int index);
    void validRowsCal(void);
    int getMiddleValue(vector<int> vec);
};
