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
 * @file fsm.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 有限状态机状态基类
 * @version 0.1
 * @date 2025-03-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <string>
#include <iostream>
#include <cmath>
#include <numeric>
#include <unistd.h>
#include "utils/params.hpp"

/**
 * @brief 有限状态机状态基类
 *
 */
class FSMState
{
public:
    FSMState(FsmMode mode, std::shared_ptr<Params> par) : fsmMode(mode), params(par) {}
    virtual ~FSMState() = default;
    virtual void run(Mat &img) = 0;
    virtual void show(Mat &img) = 0;
    virtual void resetLap() {}  // 圈数变更时复位状态
    virtual FsmMode getMode() { return FsmMode::NORMAL; }
    FsmMode fsmMode;

protected:
    shared_ptr<Params> params;
};