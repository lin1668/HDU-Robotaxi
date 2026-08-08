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
 *            The具体细节咨询专业(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file yfork.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief Y型岔路口图像识别与规划
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief Y型岔路口图像识别与规划
 *
 */
class FsmYfork : public FSMState
{
public:
    FsmYfork(std::shared_ptr<Params> par);
    ~FsmYfork();
    void run(Mat &img);
    void show(Mat &img);
    FsmMode getMode();
    void drawTip(Mat &img);  // 在Ctrl图像上标V尖
    void resetLap();
    void deactivate(); // 当前圈无Y岔路任务时清除残留状态

private:
    /**
     * @brief 场景状态
     *
     */
    enum Step
    {
        NONE = 0,        // 未知类型
        DECIDE,          // 决定左右
        ENTER,           // 进入岔路口
        EXIT,            // 出岔路口
        END              // 任务结束
    };
    bool enable = false;         // 状态使能
    Step step = Step::NONE;      // Y型岔路口处理阶段
    bool selectLeft = false;     // 选择左侧路径
    bool forkSeen = false;       // 已看到fork标志（消失后开始找V尖）
    int counterYfork = 0;       // 记录一个状态的图像场数
    int timeout = 0;            // 超时计数器
    int countRes = 0;           // 退出帧计数器
    int tipRow = 0;             // V尖行位置
    int tipCol = 0;             // V尖列位置
    bool vloss = false;         // V尖已确认消失（不再接受新红点）
    bool completed = false;     // 已完成一轮Y型岔路（防止停车区fork箭头误触发）
    int holdRow = 0;            // 最后已知V尖行
    int holdCol = 0;            // 最后已知V尖列
    int vlossTimer = 0;         // V尖消失后保持引导的帧计数
    int forceLeftTimer = 0;       // 强制左转剩余帧数
    bool forceLeftDone = false;   // 强制左转已执行过（每圈只执行一次）
    bool exitRightBiasArmed = false; // 出 YFork 后靠右保护已触发；同一次 YFork 完成后不再反复续时
    int forkForceLeftTimer = 0;   // FORK达到比例后的左转延迟计时
    int forkAutoStopTimer = 0;    // FORK触发左转后到自动停车的计时
    int yforkParkStopCount = 0;   // 自动停车持续帧数

    // ===== 可调参数 =====
    static constexpr int FORCE_LEFT_FRAMES = 18;          // 强制左转持续帧数
    static constexpr float FORK_FORCE_LEFT_RATIO = 0.26f; // FORK到画面X%开始计时
    static constexpr int FORK_FORCE_LEFT_DELAY = 0;      // 达到比例后等N帧强制左转
    static constexpr int FORK_AUTO_STOP_DELAY = 80;      // FORK触发左转后等N帧自动停车
    static constexpr int RIGHT_GUIDE_OFFSET = 10;         // 右分支引导时左边线向右平移像素
    // ====================

    void reset(void);
    void finish();
    bool handle(Mat &img);
    bool detectYfork(Mat &img);
    bool findVTip(const Mat &img);  // 在二值图中扫描V尖
    void replanTracking(bool left, const Mat &img);  // 车道线重绘（V尖屏障引导）
    void logFrame(const char *phase); // 每帧完整诊断快照
};
