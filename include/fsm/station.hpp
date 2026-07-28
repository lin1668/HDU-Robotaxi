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
 * @file station.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停靠站停车控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 停靠站停车控制（压到station框即停）
 *
 */
class FsmStation : public FSMState
{
public:
    FsmStation(std::shared_ptr<Params> par);
    ~FsmStation();
    void run(Mat &img);
    void show(Mat &img);
    FsmMode getMode();
    void resetLap();

private:
    /**
     * @brief 场景状态
     *
     */
    enum Step
    {
        NONE = 0, // 未检测
        STOP      // 停车
    };

    Step step = Step::NONE;
    int stopCounter = 0;
    int countInit = 0;
    int pressTimer = 0;    // 压到station后计时器（9帧=0.3秒）
    int cooldown = 0;      // 停车后冷却计数器（150帧=5秒）
    int stationBoxCounter = 0;      // 跳过计数（施工区busyStopPoint选择用）
    bool stationBoxCounted = false; // 当前框已计数标志（防止重复计数同一框）
    int busyEntryDelay = 0;         // 施工区进入后延迟检测（帧数）
    int leftBranchDelay = 0;        // 左分支框过半后延迟（帧数）

    // 可调：框过半后等多少帧才开始停车
    static constexpr int LEFT_BRANCH_DELAY_FRAMES = 15;

    void setStep(Step st);
};
