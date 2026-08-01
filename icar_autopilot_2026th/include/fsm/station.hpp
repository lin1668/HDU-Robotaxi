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

    // Station停车可调参数：只改这里的数值，不改下面各分支停车逻辑
    struct Tune
    {
        // 通用
        static constexpr int STARTUP_IGNORE_FRAMES = 15;      // 发车后屏蔽station检测帧数
        static constexpr int NORMAL_TRIGGER_BOTTOM_MARGIN = 80; // 普通station距离底部触发余量
        static constexpr int NORMAL_PRESS_FRAMES = 19;          // 普通station压框后停车等待帧数
        static constexpr int STOP_HOLD_FRAMES = 30;             // 停车保持帧数
        static constexpr int NORMAL_COOLDOWN_FRAMES = 150;    // 非施工区停车后冷却帧数

        // 施工区
        static constexpr int BUSY_ENTRY_DELAY_FRAMES = 10;    // 手动接管结束后，施工区延迟检测帧数
        static constexpr float BUSY_FIRST_BOX_RATIO = 0.45f;   // 施工区第一个框触发位置
        static constexpr int BUSY_TARGET_BOTTOM_MARGIN = 70;  // 施工区目标框距离底部触发余量，增大可提前停车
        static constexpr int BUSY_FIRST_PRESS_FRAMES = 15;    // 施工区第一个框压框后停车等待帧数
        static constexpr int BUSY_TARGET_PRESS_FRAMES = 10;   // 施工区目标框压框后停车等待帧数
        static constexpr int BUSY_COOLDOWN_FRAMES = 6;        // 施工区停车后冷却帧数

        // 左岔路
        static constexpr float LEFT_BRANCH_TRIGGER_RATIO = 0.40f;
        static constexpr int LEFT_BRANCH_DELAY_FRAMES = 16;
        static constexpr int LEFT_BRANCH_PRESS_FRAMES = 26;

        // 右岔路：沿用当前已验证的停车时机；后续只调这里即可，不影响普通路线。
        static constexpr int RIGHT_BRANCH_TRIGGER_BOTTOM_MARGIN = 14;
        static constexpr int RIGHT_BRANCH_PRESS_FRAMES = 19;

    };

    void setStep(Step st);
};
