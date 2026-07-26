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
 * @file show.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 绘制Debug图像
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */
/**
 * @brief UI综合图像绘制
 *
 */

#include "utils/json.hpp"

using namespace std;
using namespace cv;

class Show
{
private:
    bool enable = false;   // 显示窗口使能
    int sizeWindow = 1;    // 窗口数量
    cv::Mat imgShow;       // 窗口图像
    bool realShow = false; // 实时更新画面
public:
    int index = 0;      // 图像序号
    int indexLast = -1; // 图像序号
    int frameMax = 0;   // 视频总帧数
    bool save = false;  // 图像存储

    /**
     * @brief 显示窗口初始化
     *
     * @param size 窗口数量(1~7)
     */
    Show(const int size)
    {
        if (size <= 0 || size > 7)
            return;

        cv::namedWindow("ICAR", WINDOW_NORMAL);     // 图像名称
        cv::resizeWindow("ICAR", 480 * 2, 320 * 2); // 分辨率

        imgShow = cv::Mat::zeros(ROWSIMAGE * 2, COLSIMAGE * 2, CV_8UC3);
        enable = true;
        sizeWindow = size;
    };
    ~Show() {};

    /**
     * @brief 设置新窗口属性
     *
     * @param index 窗口序号
     * @param name 窗口名称
     * @param img 显示图像
     */
    void setNewWindow(int index, string name, Mat img)
    {
        // 数据溢出保护
        if (!enable || index <= 0 || index > sizeWindow)
            return;

        if (img.cols <= 0 || img.rows <= 0)
            return;

        Mat imgDraw = img.clone();

        if (imgDraw.type() == CV_8UC1) // 非RGB类型的图像
            cvtColor(imgDraw, imgDraw, cv::COLOR_GRAY2BGR);

        // 图像缩放
        if (imgDraw.cols != COLSIMAGE || imgDraw.rows != ROWSIMAGE)
        {
            float fx = COLSIMAGE / imgDraw.cols;
            float fy = ROWSIMAGE / imgDraw.rows;
            if (fx <= fy)
                resize(imgDraw, imgDraw, Size(COLSIMAGE, ROWSIMAGE), fx, fx);
            else
                resize(imgDraw, imgDraw, Size(COLSIMAGE, ROWSIMAGE), fy, fy);
        }

        // 限制图片标题长度
        string text = "[" + to_string(index) + "] ";
        if (name.length() > 15)
            text = text + name.substr(0, 15);
        else
            text = text + name;

        putText(imgDraw, text, Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 0), 0.5);

        if (index <= 2)
        {
            Rect placeImg = Rect(COLSIMAGE * (index - 1), 0, COLSIMAGE, ROWSIMAGE);
            imgDraw.copyTo(imgShow(placeImg));
        }

        else
        {
            Rect placeImg = Rect(COLSIMAGE * (index - 3), ROWSIMAGE, COLSIMAGE, ROWSIMAGE);
            imgDraw.copyTo(imgShow(placeImg));
        }

        if (save)
            savePicture(img); // 保存图像
    }

    /**
     * @brief 融合后的图像显示
     *
     */
    void show(void)
    {
        if (enable)
        {
            putText(imgShow, "Frame:" + to_string(index), Point(COLSIMAGE / 2 - 50, ROWSIMAGE * 2 - 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
            imshow("ICAR", imgShow);

            char key = waitKey(1);
            if (key != -1)
            {
                if (key == 32) // 空格
                    realShow = !realShow;
            }
            if (realShow)
            {
                index++;
                if (index < 0)
                    index = 0;
                if (index > frameMax)
                    index = frameMax;
            }
        }
    }
};
