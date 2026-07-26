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
 * @file imging.cpp
 * @author Leo
 * @brief 图像采集
 * @version 0.1
 * @date 2024-01-09
 * @copyright Copyright (c) 2024
 * @note 采图步骤：
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include "utils/tools.hpp"
#include "ctrl/predeal.hpp"

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
    // 打开摄像头
    VideoCapture capture("../res/samples/sample.mp4");
    // VideoCapture capture("/dev/video0", CAP_V4L2);
    if (!capture.isOpened())
    {
        cout << "can not open video device " << endl;
        return 1;
    }

    capture.set(CAP_PROP_FRAME_WIDTH, COLSCAMERA);  // 设置图像的列数
    capture.set(CAP_PROP_FRAME_HEIGHT, ROWSCAMERA); // 设置图像的行数

    double rate = capture.get(CAP_PROP_FPS);            // 读取图像的帧率
    double width = capture.get(CAP_PROP_FRAME_WIDTH);   // 读取图像的宽度
    double height = capture.get(CAP_PROP_FRAME_HEIGHT); // 读取图像的高度
    cout << "Camera Param: frame rate = " << rate << " width = " << width
         << " height = " << height << endl;

    // 读取xml中的相机标定参数
    shared_ptr<Predeal> predeal = make_shared<Predeal>(-1); // 图像预处理类

    while (1)
    {
        //[01] 图像采集
        Mat img;
        if (!capture.read(img))
        {
            cout << "no video frame" << endl;
            // 如果没有帧了，重置到视频开头
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            continue;
        }

        //[02] 图像预处理
        predeal->correction(img); // 图像矫正
        imshow("imgCor", img);
        // predeal->imgCutting(img); // 图像裁剪
        // imshow("imgCut", img);
        cv::Mat imgBin = predeal->binaryzation(img); // 图像二值化
        //[03] 透视变换
        cv::Mat imgIpm;
        ipm.homography(img, imgIpm);
        imshow("imgIpm", imgIpm);
        int key = cv::waitKey(20); // 等待用户按键（无限时长）
        if (key == 32)
        {
            savePicture(imgBin);
            savePicture(imgIpm);
        }
    }
    capture.release();
}
