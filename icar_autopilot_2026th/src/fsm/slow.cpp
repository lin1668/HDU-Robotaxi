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
 * @file slow.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 慢行区识别与规划
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/slow.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmSlow::FsmSlow(std::shared_ptr<Params> par)
    : FSMState(FsmMode::SLOW, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmSlow::~FsmSlow()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmSlow::getMode()
{
    // 输出场景状态结果
    if (!params->config.slow || !params->config.currentLapConfig->slow || step == Step::NONE)
        return FsmMode::NORMAL;

    return FsmMode::SLOW;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmSlow::run(Mat &img)
{
    if (!params->config.slow || params->config.currentLapConfig == nullptr ||
        !params->config.currentLapConfig->slow) // 该模式未开启
    {
        params->ctrl.forceSlowRequest = false;
        return;
    }

    if (params->ctrl.forceSlowRequest)
    {
        params->ctrl.forceSlowRequest = false;
        if (step == Step::NONE)
            setStep(Step::ENABLE);
    }

    switch (step)
    {
    case Step::NONE: // AI标志检测

        params->ctrl.slow = false; // 清除慢行标志
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_LIMIT) // AI识别标志
            {
                if (params->results[i].height < 100 && params->results[i].width < 80) // 标志位置过滤
                {
                    countRec++;
                    break;
                }
            }
        }
        if (countRec >= 2)
            setStep(Step::ENABLE);

        if (countRec > 0) // 识别AI标志后开始场次计数
        {
            countSes++;
            if (countSes >= 5)
            {
                countRec = 0;
                countSes = 0;
            }
        }
        break;

    case Step::ENABLE: // 慢行阶段
    {
        timeout++;
        params->ctrl.slow = true; // 设置慢行标志

        bool seenUnlimit = false;
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_LIMIT) // AI识别标志
            {
                if (params->results[i].height < 100 && params->results[i].width < 80 &&
                    (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.2) // 标志位置过滤
                {
                    timeout = 0;
                }
            }
            if (params->results[i].type == LABEL_UNLIMIT) // AI识别标志
            {
                if (params->results[i].height < 100 && params->results[i].width < 80 &&
                    (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.2) // 标志位置过滤
                {
                    countRec++;
                    seenUnlimit = true;
                    break;
                }
            }
        }

        // UNLIMIT确认(countRec>=2)后，消失连续3帧则退出慢行区
        if (countRec >= 2)
        {
            if (seenUnlimit)
            {
                unlimitDelay = 0; // 还在视野内，重置丢失计数
            }
            else
            {
                unlimitDelay++; // UNLIMIT消失，累计丢失帧数
                if (unlimitDelay > 3)
                    setStep(Step::NONE);
            }
        }
        else if (countRec > 0) // 未确认前的场次计数器防抖
        {
            countSes++;
            if (countSes >= 5)
            {
                countRec = 0;
                countSes = 0;
            }
        }
        break;
    }

    default:
        break;
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmSlow::show(Mat &img)
{
    if (params->mode != FsmMode::SLOW)
        return;

    putText(img, "[6] Slow", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

/**
 * @brief 设置新状态
 *
 * @param step
 */
void FsmSlow::setStep(Step st)
{
    step = st;
    countRec = 0;     // AI场景识别计数器
    countSes = 0;     // 场次计数器
    timeout = 0;      // 超时计数器
    unlimitDelay = 0; // 解除限速延时计数器
}
