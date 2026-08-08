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

#include "ctrl/track.hpp"

using namespace cv;
using namespace std;

/**
 * @brief 赛道线识别
 *
 * @param imageBinary 赛道识别基准图像
 */
void Track::handle(Mat img)
{
    imgShare = img;
    handle(false, 0);
}

/**
 * @brief 赛道线识别
 *
 * @param isResearch 是否重复搜索
 * @param rowStart 边缘搜索起始行
 */
void Track::handle(bool isResearch, uint16_t rowStart)
{
    bool flagStartBlock = true;                    // 搜索到色块起始行的标志（行）
    int counterSearchRows = pointsEdgeLeft.size(); // 搜索行计数
    int startBlock[30];                            // 色块起点（行）
    int endBlock[30];                              // 色块终点（行）
    int counterBlock = 0;                          // 色块计数器（行）
    PointX pointSpurroad;                          // 岔路坐标
    int counterSpurroad = 0;                       // 岔路识别标志
    bool spurroadEnable = false;

    if (rowCutUp > ROWSIMAGE / 4)
        rowCutUp = ROWSIMAGE / 4;
    if (rowCutBottom > ROWSIMAGE / 4)
        rowCutBottom = ROWSIMAGE / 4;

    if (!isResearch)
    {
        pointsEdgeLeft.clear();              // 初始化边缘结果
        pointsEdgeRight.clear();             // 初始化边缘结果
        widthBlock.clear();                  // 初始化色块数据
        spurroad.clear();                    // 岔路信息
        validRowsLeft = 0;                   // 边缘有效行数（左）
        validRowsRight = 0;                  // 边缘有效行数（右）
        flagStartBlock = true;               // 搜索到色块起始行的标志（行）
        rowStart = ROWSIMAGE - rowCutBottom; // 默认底部起始行
    }
    else
    {
        if (pointsEdgeLeft.size() > rowStart)
            pointsEdgeLeft.resize(rowStart);
        if (pointsEdgeRight.size() > rowStart)
            pointsEdgeRight.resize(rowStart);
        if (widthBlock.size() > rowStart)
        {
            widthBlock.resize(rowStart);
            if (rowStart > 1)
                rowStart = widthBlock[rowStart - 1].x - 2;
        }

        flagStartBlock = false; // 搜索到色块起始行的标志（行）
    }

    //  开始识别赛道左右边缘
    for (int row = rowStart; row > rowCutUp; row--) // 有效行：10~220
    {
        counterBlock = 0; // 色块计数器清空
                          // 搜索色（block）块信息
        if (imgShare.at<uchar>(row, 0) > 127)
        {
            startBlock[counterBlock] = 0;
        }
        for (int col = 1; col < COLSIMAGE; col++) // 搜索出每行的所有色块
        {
            if (imgShare.at<uchar>(row, col) > 127 &&
                imgShare.at<uchar>(row, col - 1) <= 127)
            {
                startBlock[counterBlock] = col;
            }
            else
            {
                if (imgShare.at<uchar>(row, col) <= 127 &&
                    imgShare.at<uchar>(row, col - 1) > 127)
                {
                    endBlock[counterBlock++] = col;
                    if (counterBlock >= end(endBlock) - begin(endBlock))
                        break;
                }
            }
        }
        if (imgShare.at<uchar>(row, COLSIMAGE - 1) > 127)
        {
            if (counterBlock < end(endBlock) - begin(endBlock) - 1)
                endBlock[counterBlock++] = COLSIMAGE - 1;
        }

        int widthBlocks = endBlock[0] - startBlock[0]; // 色块宽度临时变量
        int indexWidestBlock = 0;                      // 最宽色块的序号
        if (flagStartBlock)                            // 起始行做特殊处理
        {
            if (row < ROWSIMAGE / 3)
                return;
            if (counterBlock == 0)
            {
                continue;
            }
            for (int i = 1; i < counterBlock; i++) // 搜索最宽色块
            {
                int tmp_width = endBlock[i] - startBlock[i];
                if (tmp_width > widthBlocks)
                {
                    widthBlocks = tmp_width;
                    indexWidestBlock = i;
                }
            }

            int limitWidthBlock = (COLSIMAGE - (ROWSIMAGE - row)) * 0.65; // 首行色块宽度限制（不能太小）
            if (row < ROWSIMAGE * 0.75)
                limitWidthBlock = COLSIMAGE * 0.5; // 首行色块宽度限制（不能太小）

            if (widthBlocks > limitWidthBlock) // 满足首行宽度要求
            {
                flagStartBlock = false;
                PointX pointTmp(row, startBlock[indexWidestBlock]);
                pointsEdgeLeft.push_back(pointTmp);
                pointTmp.y = endBlock[indexWidestBlock];
                pointsEdgeRight.push_back(pointTmp);
                widthBlock.emplace_back(row, endBlock[indexWidestBlock] - startBlock[indexWidestBlock]);
                counterSearchRows++;
            }
            spurroadEnable = false;
        }
        else // 其它行色块坐标处理
        {
            if (counterBlock == 0)
            {
                continue; // 跳过缺口行，继续向上搜索
            }

            if (row == 22)
                int a = 0;
            vector<int> indexBlocks;               // 色块序号（行）
            for (int i = 0; i < counterBlock; i++) // 上下行色块的连通性判断
            {
                int g_cover = min(endBlock[i], pointsEdgeRight[pointsEdgeRight.size() - 1].y) -
                              max(startBlock[i], pointsEdgeLeft[pointsEdgeLeft.size() - 1].y);
                if (g_cover >= 0)
                {
                    indexBlocks.push_back(i);
                }
            }

            if (indexBlocks.size() == 0) // 如果没有发现联通色块，跳过缺口继续搜索
            {
                continue;
            }
            else if (indexBlocks.size() == 1) // 只存在单个色块，正常情况，提取边缘信息
            {
                if (endBlock[indexBlocks[0]] - startBlock[indexBlocks[0]] < COLSIMAGE / 20)
                {
                    break;
                }
                pointsEdgeLeft.emplace_back(row, startBlock[indexBlocks[0]]);
                pointsEdgeRight.emplace_back(row, endBlock[indexBlocks[0]]);
                slopeCal(pointsEdgeLeft, pointsEdgeLeft.size() - 1); // 边缘斜率计算
                slopeCal(pointsEdgeRight, pointsEdgeRight.size() - 1);
                widthBlock.emplace_back(row, endBlock[indexBlocks[0]] - startBlock[indexBlocks[0]]);
                spurroadEnable = false;
            }
            else if (indexBlocks.size() > 1) // 存在多个色块，则需要择优处理：选取与上一行最近的色块
            {
                int centerLast = COLSIMAGE / 2;
                if (pointsEdgeRight.size() > 0 && pointsEdgeLeft.size() > 0)
                    centerLast = (pointsEdgeRight[pointsEdgeRight.size() - 1].y + pointsEdgeLeft[pointsEdgeLeft.size() - 1].y) / 2; // 上一行色块的中心点横坐标
                int centerThis = (startBlock[indexBlocks[0]] + endBlock[indexBlocks[0]]) / 2;                                       // 当前行色块的中心点横坐标
                int differBlocks = abs(centerThis - centerLast);                                                                    // 上下行色块的中心距离
                int indexGoalBlock = 0;                                                                                             // 目标色块的编号
                int startBlockNear = startBlock[indexBlocks[0]];                                                                    // 搜索与上一行最近的色块起点
                int endBlockNear = endBlock[indexBlocks[0]];                                                                        // 搜索与上一行最近的色块终点

                for (int i = 1; i < indexBlocks.size(); i++) // 搜索与上一行最近的色块编号
                {
                    centerThis = (startBlock[indexBlocks[i]] + endBlock[indexBlocks[i]]) / 2;
                    if (abs(centerThis - centerLast) < differBlocks)
                    {
                        differBlocks = abs(centerThis - centerLast);
                        indexGoalBlock = i;
                    }
                    // 搜索与上一行最近的边缘起点和终点
                    if (abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlock[indexBlocks[i]]) <
                        abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlockNear))
                    {
                        startBlockNear = startBlock[indexBlocks[i]];
                    }
                    if (abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlock[indexBlocks[i]]) <
                        abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlockNear))
                    {
                        endBlockNear = endBlock[indexBlocks[i]];
                    }
                }

                // 检索最佳的起点与终点
                if (abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlock[indexBlocks[indexGoalBlock]]) <
                    abs(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y - startBlockNear))
                {
                    startBlockNear = startBlock[indexBlocks[indexGoalBlock]];
                }
                if (abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlock[indexBlocks[indexGoalBlock]]) <
                    abs(pointsEdgeRight[pointsEdgeRight.size() - 1].y - endBlockNear))
                {
                    endBlockNear = endBlock[indexBlocks[indexGoalBlock]];
                }

                if (endBlockNear - startBlockNear < COLSIMAGE / 10)
                {
                    continue;
                }
                PointX tmp_point(row, startBlockNear);
                pointsEdgeLeft.push_back(tmp_point);
                tmp_point.y = endBlockNear;
                pointsEdgeRight.push_back(tmp_point);
                widthBlock.emplace_back(row, endBlockNear - startBlockNear);
                slopeCal(pointsEdgeLeft, pointsEdgeLeft.size() - 1);
                slopeCal(pointsEdgeRight, pointsEdgeRight.size() - 1);
                counterSearchRows++;

                //-------------------------------<岔路信息提取>----------------------------------------
                pointSpurroad.x = row;
                pointSpurroad.y = endBlock[indexBlocks[0]];
                if (!spurroadEnable)
                {
                    spurroad.push_back(pointSpurroad);
                    spurroadEnable = true;
                }
                //------------------------------------------------------------------------------------
            }
        }
    }

    fillEdgeGap(pointsEdgeLeft, true);
    fillEdgeGap(pointsEdgeRight, false);

    stdevLeft = stdevEdgeCal(pointsEdgeLeft, ROWSIMAGE); // 计算边缘方差
    stdevRight = stdevEdgeCal(pointsEdgeRight, ROWSIMAGE);

    validRowsCal(); // 有效行计算
}

/**
 * @brief 修复局部边线缺口：补齐缺失行，并插值替换短段贴图像边界的假边线
 */
void Track::fillEdgeGap(vector<PointX> &edge, bool isLeft)
{
    if (edge.size() < 2)
        return;

    vector<PointX> filled;
    filled.reserve(edge.size() + 24);
    filled.push_back(edge.front());
    for (size_t i = 1; i < edge.size(); i++)
    {
        int rowGap = edge[i - 1].x - edge[i].x;
        if (rowGap > 1 && rowGap <= 12) // 边线由下向上排列，只补短小局部缺口
        {
            for (int offset = 1; offset < rowGap; offset++)
            {
                float t = static_cast<float>(offset) / rowGap;
                int row = edge[i - 1].x - offset;
                int col = edge[i - 1].y +
                          static_cast<int>((edge[i].y - edge[i - 1].y) * t);
                filled.emplace_back(row, col);
            }
        }
        filled.push_back(edge[i]);
    }
    edge.swap(filled);

    static constexpr size_t MAX_BORDER_GAP = 20;
    auto isBorderPoint = [isLeft](const PointX &point)
    {
        return isLeft ? point.y <= 1 : point.y >= COLSIMAGE - 2;
    };

    size_t i = 1;
    while (i + 1 < edge.size())
    {
        if (!isBorderPoint(edge[i]))
        {
            i++;
            continue;
        }

        size_t gapBegin = i;
        while (i < edge.size() && isBorderPoint(edge[i]))
            i++;
        size_t gapEnd = i;

        // 只修复夹在两段有效边线之间的短缺口，弯道长期出画时不处理。
        if (gapEnd < edge.size() && gapEnd - gapBegin <= MAX_BORDER_GAP &&
            !isBorderPoint(edge[gapBegin - 1]) && !isBorderPoint(edge[gapEnd]))
        {
            const PointX &before = edge[gapBegin - 1];
            const PointX &after = edge[gapEnd];
            int totalRows = before.x - after.x;
            if (totalRows > 0)
            {
                for (size_t j = gapBegin; j < gapEnd; j++)
                {
                    float t = static_cast<float>(before.x - edge[j].x) / totalRows;
                    edge[j].y = before.y +
                                static_cast<int>((after.y - before.y) * t);
                }
            }
        }
    }
}

/**
 * @brief 显示赛道线识别结果
 *
 * @param img 需要叠加显示的图像
 */
void Track::drawImage(Mat &img)
{
    for (int i = 0; i < pointsEdgeLeft.size(); i++)
    {
        circle(img, Point(pointsEdgeLeft[i].y, pointsEdgeLeft[i].x), 1,
               Scalar(0, 255, 0), -1); // 绿色点
    }
    for (int i = 0; i < pointsEdgeRight.size(); i++)
    {
        circle(img, Point(pointsEdgeRight[i].y, pointsEdgeRight[i].x), 1,
               Scalar(0, 255, 255), -1); // 黄色点
    }

    for (int i = 0; i < spurroad.size(); i++)
    {
        circle(img, Point(spurroad[i].y, spurroad[i].x), 3,
               Scalar(0, 0, 255), -1); // 红色点
    }

    putText(img, to_string(validRowsRight) + " " + to_string(stdevRight), Point(COLSIMAGE - 100, ROWSIMAGE - 50),
            FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 0.8, LINE_AA);
    putText(img, to_string(validRowsLeft) + " " + to_string(stdevLeft), Point(20, ROWSIMAGE - 50),
            FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 0.8, LINE_AA);
}

/**
 * @brief 边缘斜率计算
 *
 * @param v_edge
 * @param img_height
 * @return double
 */
double Track::stdevEdgeCal(vector<PointX> &v_edge, int img_height)
{
    if (v_edge.size() < img_height / 4)
    {
        return 1000;
    }
    vector<int> v_slope;
    int step = v_edge.size() / 5; // v_edge.size()/10;
    for (int i = step; i < v_edge.size(); i += step)
    {
        if (v_edge[i].x - v_edge[i - step].x)
            v_slope.push_back((v_edge[i].y - v_edge[i - step].y) * 100 / (v_edge[i].x - v_edge[i - step].x));
    }
    if (v_slope.size() > 1)
    {
        double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
        double mean = sum / v_slope.size(); // 均值
        double accum = 0.0;
        for_each(begin(v_slope), end(v_slope), [&](const double d)
                 { accum += (d - mean) * (d - mean); });

        return sqrt(accum / (v_slope.size() - 1)); // 方差
    }
    else
        return 0;
}

/**
 * @brief 边缘斜率计算
 *
 * @param edge
 * @param index
 */
void Track::slopeCal(vector<PointX> &edge, int index)
{
    if (index <= 4)
    {
        return;
    }
    float temp_slop1 = 0.0, temp_slop2 = 0.0;
    if (edge[index].x - edge[index - 2].x != 0)
    {
        temp_slop1 = (float)(edge[index].y - edge[index - 2].y) * 1.0f /
                     ((edge[index].x - edge[index - 2].x) * 1.0f);
    }
    else
    {
        temp_slop1 = edge[index].y > edge[index - 2].y ? 255 : -255;
    }
    if (edge[index].x - edge[index - 4].x != 0)
    {
        temp_slop2 = (float)(edge[index].y - edge[index - 4].y) * 1.0f /
                     ((edge[index].x - edge[index - 4].x) * 1.0f);
    }
    else
    {
        edge[index].slope = edge[index].y > edge[index - 4].y ? 255 : -255;
    }
    if (abs(temp_slop1) != 255 && abs(temp_slop2) != 255)
    {
        edge[index].slope = (temp_slop1 + temp_slop2) * 1.0 / 2;
    }
    else if (abs(temp_slop1) != 255)
    {
        edge[index].slope = temp_slop1;
    }
    else
    {
        edge[index].slope = temp_slop2;
    }
}

/**
 * @brief 边缘有效行计算：左/右
 *
 */
void Track::validRowsCal(void)
{
    // 左边有效行
    validRowsLeft = 0;
    if (pointsEdgeLeft.size() > 1)
    {
        for (int i = pointsEdgeLeft.size() - 1; i >= 1; i--)
        {
            if (pointsEdgeLeft[i].y > 2 && pointsEdgeLeft[i - 1].y >= 2)
            {
                validRowsLeft = i + 1;
                break;
            }
            if (pointsEdgeLeft[i].y < 2 && pointsEdgeLeft[i - 1].y >= 2)
            {
                validRowsLeft = i + 1;
                break;
            }
        }
    }

    // 右边有效行
    validRowsRight = 0;
    if (pointsEdgeRight.size() > 1)
    {
        for (int i = pointsEdgeRight.size() - 1; i >= 1; i--)
        {
            if (pointsEdgeRight[i].y <= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y <= COLSIMAGE - 2)
            {
                validRowsRight = i + 1;
                break;
            }
            if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y < COLSIMAGE - 2)
            {
                validRowsRight = i + 1;
                break;
            }
        }
    }
}

/**
 * @brief 冒泡法求取集合中值
 *
 * @param vec 输入集合
 * @return int 中值
 */
int Track::getMiddleValue(vector<int> vec)
{
    if (vec.size() < 1)
        return -1;
    if (vec.size() == 1)
        return vec[0];

    int len = vec.size();
    while (len > 0)
    {
        bool sort = true; // 是否进行排序操作标志
        for (int i = 0; i < len - 1; ++i)
        {
            if (vec[i] > vec[i + 1])
            {
                swap(vec[i], vec[i + 1]);
                sort = false;
            }
        }
        if (sort) // 排序完成
            break;

        --len;
    }

    return vec[(int)vec.size() / 2];
}
