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
 * @file motion.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 运动控制器
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

using namespace std;

class Motion
{

public:
    /**
     * @brief 姿态PD控制器
     *
     * @param center 智能车控制中心
     */
    void poseControl(shared_ptr<Params> &params)
    {
        float error = params->ctrl.center - COLSIMAGE / 2; // 图像控制中心转换偏差
        static int errorLast = 0;                          // 记录前一次的偏差
        if (abs(error - errorLast) > COLSIMAGE / 20)
        {
            error = error > errorLast ? errorLast + COLSIMAGE / 10
                                      : errorLast - COLSIMAGE / 10;
        }

        params->config.turnP = abs(error) * params->config.runP2 + params->config.runP1;
        int pwmDiff = (error * params->config.turnP) + (error - errorLast) * params->config.turnD;
        errorLast = error;
        params->ctrl.servo = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
        if (params->ctrl.servo > PWMSERVOMAX)
            params->ctrl.servo = PWMSERVOMAX;
        else if (params->ctrl.servo < PWMSERVOMIN)
            params->ctrl.servo = PWMSERVOMIN;
    }

    /**
     * @brief 变加速控制
     *
     * @param params
     */
    void speedControl(shared_ptr<Params> &params)
    {
        if (params->ctrl.stop) // 停车
        {
            params->ctrl.speed = 0.0;
            return;
        }
        if (params->ctrl.back) // 倒车(停车场)
        {
            params->ctrl.speed = -params->config.velPark;
            return;
        }
        if (params->mode == FsmMode::STOP) // 停车区速度
        {
            params->ctrl.speed = params->config.velStop;
            return;
        }
        else if (params->mode == FsmMode::PARK) // 停车场速度
        {
            params->ctrl.speed = params->config.velPark;
            return;
        }
        else if (params->mode == FsmMode::CROSS) // 斑马线速度
        {
            params->ctrl.speed = params->config.velCross;
            return;
        }
        else if (params->mode == FsmMode::STATION) // 停靠站速度
        {
            params->ctrl.speed = params->config.velSlow;
            return;
        }
        else if (params->mode == FsmMode::BUSY || params->busyZone) // 施工区速度（含退出手动接管后）
        {
            params->ctrl.speed = params->config.velBusy;
            return;
        }
        else if (params->mode == FsmMode::CURVE) // 连续弯道速度
        {
            params->ctrl.speed = params->config.velCurve;
            return;
        }
        else if (params->mode == FsmMode::YFORK) // Y型岔路口速度
        {
            params->ctrl.speed = params->config.velYfork;
            return;
        }
        else if (params->ctrl.slow) // 减速区速度
        {
            params->ctrl.speed = params->config.velSlow;
            return;
        }
        else if (params->ctrl.countAcc < 50)
        {
            params->ctrl.countAcc++;
            params->ctrl.speed = params->config.velCross;
            return;
        }

        int line = params->ctrl.lineArea; // 动态速度，返回点越高，速度越大，线性变化

        // 控制率
        uint8_t controlLow = 3;   // 速度控制下限
        uint8_t controlMid = 5;   // 控制率
        uint8_t controlHigh = 10; // 速度控制上限

        float upper = ROWSIMAGE * 0.35; // 84
        line = std::max(line - 20, 0);
        if (line > upper)
            line = upper;

        params->ctrl.speed = params->config.velLow +
                             std::pow(float(upper - line) / (upper), 3) * (params->config.velHigh - params->config.velLow);
        if (params->ctrl.speed > params->config.velHigh)
            params->ctrl.speed = params->config.velHigh;

        if (params->alertDecelCount > 0)
            params->ctrl.speed = std::max(0.0f, params->ctrl.speed - 0.1f);

        // 每秒输出一次行驶状态日志（~30帧/秒）
        static int statusLogCnt = 0;
        if (statusLogCnt++ % 30 == 0) {
            const char *modeStr = "NORMAL";
            switch (params->mode) {
                case FsmMode::PARK:   modeStr = "PARK";   break;
                case FsmMode::BUSY:   modeStr = "BUSY";   break;
                case FsmMode::FORK:   modeStr = "FORK";   break;
                case FsmMode::YFORK:  modeStr = "YFORK";  break;
                case FsmMode::SLOW:   modeStr = "SLOW";   break;
                case FsmMode::STOP:   modeStr = "STOP";   break;
                case FsmMode::STATION:modeStr = "STATION";break;
                case FsmMode::CROSS:  modeStr = "CROSS";  break;
                case FsmMode::CURVE:  modeStr = "CURVE";  break;
                case FsmMode::MANUAL: modeStr = "MANUAL"; break;
                default: break;
            }
            fprintf(stderr, "[STATUS] lap=%d mode=%s speed=%.2f servo=%d center=%.1f\n",
                params->currentLap, modeStr, params->ctrl.speed,
                params->ctrl.servo, params->ctrl.center);
        }
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(shared_ptr<Params> &params, Mat &img)
    {
        std::string str = "Vel: " + doble2String(params->ctrl.speed, 2);
        putText(img, str, Point(COLSIMAGE - 100, 120), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 速度
    }

    /**
     * @brief 车辆冲出赛道检测（保护车辆）
     *
     * @param track
     * @return true
     * @return false
     */
    void outlineCheck(shared_ptr<Params> &params)
    {
        // 停车场模式下车辆主动脱离赛道线，不检测
        if (params->ctrl.parking)
        {
            countRes = 0;
            outline = false;
            return;
        }

        // 出库后冷却期，给车辆时间重新捕获赛道线
        if (params->ctrl.outlineCooldown > 0)
        {
            params->ctrl.outlineCooldown--;
            countRes = 0;
            outline = false;
            return;
        }

        if (outline) // 已出现
        {
            params->ctrl.stop = true; // 停车
            countRes++;
            if (countRes > 15)
            {
                std::cout << "-----> [Stop] Game over, system exit!!! <-----" << std::endl;
                std::_Exit(0); // 直接退出，跳过静态析构避免ONNX崩溃
            }
        }
        else // 出线检测
        {
            if (params->track->pointsEdgeLeft.size() < 30 &&
                params->track->pointsEdgeRight.size() < 30) // 防止车辆冲出赛道
            {
                countRes++;
                countOut = 0;
                if (countRes > 30)
                    outline = true;
            }
            else
            {
                countRes = 0; // 边线恢复立即归零，避免跨圈/跨路段累加
                countOut = 0;
            }
        }
    }

private:
    int countRes = 0;
    int countOut = 0;
    bool outline = false; // 出线标志
};
