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
 * @file predeal.hpp
 * @author Leo
 * @brief 图像预处理：RGB转灰度图，图像二值化
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include "utils/tools.hpp"

class Predeal
{
public:
	Predeal(int bin);
	~Predeal() {};
	cv::Mat binaryzation(cv::Mat &img);
	void correction(cv::Mat &img);
	void imgCutting(cv::Mat &img);
	int binary = -1; // 图像二值化阈值：<0 默认使用大津法

private:
	bool enable = false;  // 图像矫正使能：初始化完成
	cv::Mat cameraMatrix; // 摄像机内参矩阵
	cv::Mat distCoeffs;	  // 相机的畸变矩阵
};