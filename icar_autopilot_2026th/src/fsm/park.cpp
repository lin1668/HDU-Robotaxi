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
 * @file park.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停车场控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/park.hpp"
#include "utils/yfork_diag.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmPark::FsmPark(std::shared_ptr<Params> par)
    : FSMState(FsmMode::PARK, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmPark::~FsmPark()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmPark::getMode()
{
    // std::cout << "[Park::getMode] step=" << (int)step
    //           << " config.park=" << params->config.park
    //           << " currentLapConfig->park=" << params->config.currentLapConfig->park
    //           << " parkSpot=" << params->config.currentLapConfig->parkSpot
    //           << " currentLap=" << params->currentLap << std::endl;

    if (step == Step::NONE || !params->config.park || !params->config.currentLapConfig->park)
    {
        params->ctrl.parking = false;
        return FsmMode::NORMAL;
    }
    else
    {
        params->ctrl.parking = true;
        return FsmMode::PARK;
    }
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmPark::run(Mat &img)
{
    // 出库后的慢行保险独立于Park使能状态运行，避免切圈后入口门禁提前返回。
    if (slowFallbackArmed)
    {
        bool slowEnabled = params->config.slow &&
                           params->config.currentLapConfig != nullptr &&
                           params->config.currentLapConfig->slow;
        if (!slowEnabled || params->ctrl.slow || params->mode == FsmMode::SLOW)
        {
            slowFallbackArmed = false;
            slowFallbackCount = 0;
        }
        else if (++slowFallbackCount >= SLOW_FALLBACK_FRAMES)
        {
            params->ctrl.forceSlowRequest = true;
            slowFallbackArmed = false;
            slowFallbackCount = 0;
            yforkDiagLog("PARK_EVENT", "SLOW_FALLBACK_TRIGGER frames=%d",
                         SLOW_FALLBACK_FRAMES);
        }
    }

    if (!params->config.park || params->config.currentLapConfig == nullptr ||
        !params->config.currentLapConfig->park) // 全局或当前圈未启用停车场
    {
        if (step != Step::NONE)
            reset();
        return;
    }

    if (step != Step::NONE)
        params->track->spurroad.clear(); // 停车场活动期间清除岔路红点，防止yfork误检

    stopping = false; // 停车等待标志
    speedUp++;
    if (speedUp > 1000) // 出库缓加速计时
        speedUp = 1000;

    if (params->ctrl.stop || waiting) // 禁行区不进行车库图像处理
    {
        if (params->ctrl.stop)
        {
            if (step == Step::ENABLE || step == Step::FORKIN)
            {
                stopping = true; // 停车等待标志
            }
        }
        else if (waiting) // 等待前车入库
        {
            stopping = true; // 停车等待标志
            countWait++;
            if (countWait > 140) // 50ms * 140 = 7s
            {
                waiting = false;
                countWait = 0;
            }
        }
    }

    switch (step)
    {
    case Step::NONE: // AI未识别
    {
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_PARK &&
                params->results[i].height < 100 && params->results[i].width < 90 &&
                params->results[i].x > COLSIMAGE / 2) // AI标志：停车场
                countRes++;
        }

        if (countRes >= 3)
        {
            setStep(Step::ENABLE); // 设置停车场新步骤
            std::cout << "[DEBUG] Entered PARK Step::ENABLE at countRes=" << countRes << std::endl;
        }

        if (countRes > 0) // 识别AI标志后开始场次计数
        {
            countSes++;
            if (countSes >= 5)
            {
                countSes = 0;
                countRes = 0;
            }
        }
        break;
    }

    case Step::ENABLE: // 停车场使能
    {
        countSes++;
        if (!params->ctrl.stop) // 停车等待
            timeout++;
        if (timeout > 80) // 超时退出停车状态
            reset();      // 停车场数据复位

        if (findSymbols(params->results, LABEL_PARK)) // 搜索AI标志：停车场
            countSes = 0;

        if (countSes > 30)         // 停车场Park标志丢失后延迟转入，避免过早转向
            setStep(Step::FORKIN); // 设置停车场新步骤

        // 检测入库AI标志
        PredictResult fork;
        fork.score = 0;
        fork.y = 0;
        if (!choiceDone)
        {
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_CHOICE) // AI标志：左转直行
                {
                    const int centerY = params->results[i].y + params->results[i].height / 2;
                    const bool choiceShapeOk = params->results[i].width < 90 && params->results[i].height < 90;
                    const bool choicePositionOk = centerY > ROWSIMAGE * 0.42;
                    if (choiceShapeOk && choicePositionOk && params->results[i].y > fork.y)
                    {
                        fork = params->results[i]; // 搜索最底行的岔路标志
                    }
                else
                {
                    continue;
                }

                }
            }
            if (fork.score > 0) // 检测到AI标志
            {
                countSes = 0; // 定时入库关闭 | 仅依靠AI标志转向
                countRes++;
                yforkDiagLog("PARK_EVENT", "CHOICE_ACCEPT lap=%d countRes=%d x=%d y=%d w=%d h=%d score=%.3f",
                             params->currentLap, countRes, fork.x, fork.y, fork.width, fork.height, fork.score);
                if (countRes > 1)
                {
                    choiceDone = true;
                    setStep(Step::FORKIN); // 设置停车场新步骤
                }
            }
        }
        else
        {
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_CHOICE)
                {
                    yforkDiagLog("PARK_EVENT", "CHOICE_IGNORE_DONE lap=%d x=%d y=%d w=%d h=%d score=%.3f",
                                 params->currentLap, params->results[i].x, params->results[i].y,
                                 params->results[i].width, params->results[i].height, params->results[i].score);
                }
            }
        }

        // parkSpot为0时穿过停车场不停车（不提前回退，让流程走到FORKIN）
        break;
    }

    case Step::FORKIN: // 入库岔路转向
    {
        timeout++;
        replanTracking();                             // 车道线重绘（岔路左转）
        if (findSymbols(params->results, LABEL_GATE)) // 搜索AI标志：道闸
            countRes++;
        if (countRes > 2 || timeout > 21) // 转向超时：入口左转略收一点，避免进Park时转过头
        {
            spots.reset(); // 车位信息复位

            // 自动标记非目标车位为已占用（parkSpot指定的是目标空车位）
            for (int i = 1; i <= 4; i++)
            {
                if (i != params->config.currentLapConfig->parkSpot)
                    spots.counter[i - 1] = 5; // counter > 4 即标记为已占用
            }

            countOut = 0;
            setStep(Step::TRACKIN); // 设置停车场新步骤
        }

        if (timeout > 20)
        {
            if (findSymbols(params->results, LABEL_FORK)) // 搜索AI标志：岔路箭头
                countRes++;
        }
        break;
    }

    case Step::TRACKIN: // 入库巡线中
    {
        timeout++;  // 超时计数
        countSes++; // 图像场次计数

        //[01] 道闸检测
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_GATE)
            {
                const int gateBottom = params->results[i].y + params->results[i].height;
                const bool gateShapeOk = params->results[i].width > 100 &&
                                         params->results[i].height < 130;
                if (!gateShapeOk)
                    continue;

                if (gateBottom > ROWSIMAGE * 0.33) // 停车场入口提前停车
                {
                    stopping = true; // 停车等待标志
                    break;
                }

                // 出停车场检测
                if ((params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.25 && spots.checked) // 道闸距离估算
                {
                    countOut++;
                    if (countOut > 4)
                        setStep(Step::TRACKOUT); // 设置停车场新步骤

                    break;
                }
                timeout = 0; // 超时计数
                countRes = 0;
            }
        }

        //[02] 控制中心重规划
        spots.forks = findParkStation(params->results); // 搜索停车位坐标
        PredictResult resLeft;
        resLeft.score = 0;
        for (int i = 0; i < params->results.size(); i++) // 搜索左转箭头
        {
            if (params->results[i].type == LABEL_LEFT &&
                params->results[i].width < 100 &&
                params->results[i].height < 120 &&
                params->results[i].score > resLeft.score)
                resLeft = params->results[i];
        }
        if (spots.forks.size() > 0 || resLeft.score > 0)
        {
            int targetSpot = params->config.currentLapConfig->parkSpot;
            bool rightSide = (targetSpot == 3 || targetSpot == 4);

            PredictResult direction;
            if (resLeft.score > 0)
                direction = resLeft;
            else
            {
                // 根据目标车位选择对应侧的叉（解决巡线时左偏）
                if (rightSide && spots.forks.size() >= 2)
                {
                    // 选右侧的叉
                    float cx0 = spots.forks[0].x + spots.forks[0].width / 2;
                    float cx1 = spots.forks[1].x + spots.forks[1].width / 2;
                    direction = (cx0 > cx1) ? spots.forks[0] : spots.forks[1];
                }
                else if (!rightSide && spots.forks.size() >= 2)
                {
                    // 选左侧的叉
                    float cx0 = spots.forks[0].x + spots.forks[0].width / 2;
                    float cx1 = spots.forks[1].x + spots.forks[1].width / 2;
                    direction = (cx0 < cx1) ? spots.forks[0] : spots.forks[1];
                }
                else
                {
                    direction = spots.forks[0];
                }
            }

            PointX start = PointX(ROWSIMAGE - 20, COLSIMAGE / 2); // 补线起点：车头
            if (countIn < 50)                                     // 起点矫正
            {
                if (params->track->pointsEdgeLeft.size() > ROWSIMAGE / 5 && params->track->pointsEdgeRight.size() > ROWSIMAGE / 5)
                {
                    start = {(params->track->pointsEdgeLeft[0].x + params->track->pointsEdgeRight[0].x) / 2,
                             (params->track->pointsEdgeLeft[0].y + params->track->pointsEdgeRight[0].y) / 2};
                }
                countIn++;
            }
            int sideBias = rightSide ? 20 : -30;                                            // 3/4号位右侧入库少往右推，避免先贴右边再入库
            PointX end = PointX(direction.y, direction.x + direction.width / 2 + sideBias); // 补线终点：AI标志
            PointX mid = PointX((start.x + end.x) / 2, (start.y + end.y) / 2);              // 补线中点
            vector<PointX> repairPoints = {start, mid, end};
            params->ctrl.centerEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
            params->ctrl.fitting = true;                          // 控制中心拟合标志
            timeout = 0;                                          // 超时计数
            countRes = 0;

            // 出停车场检测
            if (resLeft.score > 0 && (resLeft.y + resLeft.height / 2) > ROWSIMAGE * 0.4)
                countOut++;
            if (countOut > 3)
                setStep(Step::TRACKOUT); // 设置停车场新步骤
        }

        //[03] 空闲车位检测
        if (spots.forks.size() == 2) // 当AI图像同时检测到两个岔路箭头时判断车位是否空闲
        {
            std::cout << "[Park] Found 2 forks, checking spots..." << std::endl;
            // 已通过setParkSpotOverride指定目标车位，非目标车位由FORKIN init标记，
            // 不进行车辆检测（避免假车/他车误分类到目标车位；PredictResult默认未初始化，
            // carPark.score为随机值会导致所有车位被误判为已占用）
            for (int i = 0; i < 4; i++)
            {
                if (spots.counter[i] > 4)
                    spots.spotEnable[i] = false; // 停车位已占
            }

            // 驶入车位编号确认（用较小阈值使checked尽早完成，让spotUp/spotDown控制入库时机）
            float fork0_center = spots.forks[0].y + spots.forks[0].height / 2;
            float fork1_center = spots.forks[1].y + spots.forks[1].height / 2;
            float threshold = ROWSIMAGE * spotDown;

            std::cout << "[Park] Fork center check - fork0_center=" << fork0_center
                      << ", fork1_center=" << fork1_center
                      << ", threshold=" << threshold << std::endl;

            if (fork0_center > threshold || fork1_center > threshold)
            {
                spots.countRes++;
                std::cout << "[Park] Fork positions: fork0_y=" << fork0_center
                          << ", fork1_y=" << fork1_center << std::endl;
            }
            else
                spots.countRes = 0;
            if (spots.countRes > 1) // 开始确认驶入车位号
            {
                spots.checked = true;
                spots.countRes = 0;
                std::cout << "[Park] Spots checked = true!" << std::endl;
            }
        }

        //[04] 入库检测
        if (spots.checked && params->config.spot &&
            params->config.currentLapConfig->parkSpot > 0) // 当前圈配置了目标车位才入库
        {
            std::cout << "[Park] spotEnable[0]=" << spots.spotEnable[0]
                      << ", spotEnable[1]=" << spots.spotEnable[1]
                      << ", spotEnable[2]=" << spots.spotEnable[2]
                      << ", spotEnable[3]=" << spots.spotEnable[3] << std::endl;
            std::cout << "[Park] forks.size()=" << spots.forks.size() << std::endl;

            // 远处车位 1/4号（左前/右前）→ 第一个叉
            if (spots.spotEnable[0] || spots.spotEnable[3])
            {
                if (spots.forks.size() > 0)
                {
                    const float farEnterRatio = spotUp + (params->config.currentLapConfig->parkSpot == 4 ? rightSpotEnterDelay : 0.0f);
                    if ((spots.forks[0].y + spots.forks[0].height / 2) > ROWSIMAGE * farEnterRatio)
                    {
                        spots.times++;
                        if (spots.times > 0)
                        {
                            spots.times = 0;
                            yforkDiagLog("PARK_EVENT", "ENTER_SELECT kind=far spot=%d fork0Center=%.1f threshold=%d forks=%zu enable=%d%d%d%d",
                                         params->config.currentLapConfig->parkSpot,
                                         spots.forks[0].y + spots.forks[0].height / 2.0f,
                                          static_cast<int>(ROWSIMAGE * farEnterRatio), spots.forks.size(),
                                         spots.spotEnable[0], spots.spotEnable[1], spots.spotEnable[2], spots.spotEnable[3]);
                            setStep(Step::ENTER);
                            pointsEdgeLeftPast.clear();
                            pointsEdgeRightPast.clear();
                        }
                    }
                }
            }
            // 近处车位 2/3号（左后/右后）→ 第二个叉
            else if (spots.spotEnable[1] || spots.spotEnable[2])
            {
                if (spots.forks.size() == 2)
                {
                    const float nearEnterRatio = spotDown + (params->config.currentLapConfig->parkSpot == 3 ? rightNearEnterDelay : 0.0f);
                    if ((spots.forks[1].y + spots.forks[1].height / 2) > ROWSIMAGE * nearEnterRatio)
                    {
                        spots.times++;
                        if (spots.times > 0)
                        {
                            spots.times = 0;
                            yforkDiagLog("PARK_EVENT", "ENTER_SELECT kind=near spot=%d fork1Center=%.1f threshold=%d forks=%zu enable=%d%d%d%d",
                                         params->config.currentLapConfig->parkSpot,
                                         spots.forks[1].y + spots.forks[1].height / 2.0f,
                                          static_cast<int>(ROWSIMAGE * nearEnterRatio), spots.forks.size(),
                                         spots.spotEnable[0], spots.spotEnable[1], spots.spotEnable[2], spots.spotEnable[3]);
                            setStep(Step::ENTER);
                            pointsEdgeLeftPast.clear();
                            pointsEdgeRightPast.clear();
                        }
                    }
                }
            }
        }

        // 出库状态切换
        if (countSes > 100)
        {
            if (timeout > 20) // 超时
                countRes++;   // 出库计数器
            if (countRes > 3)
                setStep(Step::FORKOUT); // 设置停车场新步骤
        }
        break;
    }

    case Step::ENTER: // 驶入停车位
    {
        timeout++;
        int targetSpot = params->config.currentLapConfig
                             ? params->config.currentLapConfig->parkSpot
                             : 0;
        bool nearSpot = targetSpot == 2 || targetSpot == 3;
        bool rightSide = targetSpot == 3 || targetSpot == 4;
        int enterTimeoutLimit;
        if (targetSpot == 4)
            enterTimeoutLimit = 21; // 4号位略加深，避免右转入库不足
        else if (rightSide)
            enterTimeoutLimit = nearSpot ? 21 : 21;
        else
            enterTimeoutLimit = nearSpot ? 19 : 17;
        const int enterTurnLimit = (targetSpot == 4) ? 18 : (targetSpot == 3 ? 18 : 19);
        if (timeout == 1 || timeout == 18 || timeout == enterTimeoutLimit)
        {
            yforkDiagLog("PARK_EVENT",
                         "ENTER_PROGRESS timeout=%d limit=%d turnLimit=%d spot=%d near=%d edgeL=%zu edgeR=%zu center=%d speed=%.3f turnLeft=%d turnRight=%d",
                         timeout, enterTimeoutLimit, enterTurnLimit, targetSpot, nearSpot,
                         params->track->pointsEdgeLeft.size(), params->track->pointsEdgeRight.size(),
                         params->ctrl.center, params->ctrl.speed,
                         spots.spotEnable[0] || spots.spotEnable[1],
                         spots.spotEnable[2] || spots.spotEnable[3]);
        }
        if (timeout < enterTurnLimit) // 入库转向；4号位单独略延长右转，避免没完全拐进库
        {
            if (spots.spotEnable[0] || spots.spotEnable[1])      // 1/2号车位（左侧）
                replanTracking(true);                            // 入库车道线重绘（左转）
            else if (spots.spotEnable[2] || spots.spotEnable[3]) // 3/4号车位（右侧）
                replanTracking(false);                           // 入库车道线重绘（右转）
        }
        else
        {
            // Track重新捕获正常车道线
            int height = 0;
            if (params->track->pointsEdgeLeft.size() > COLSIMAGE / 2 && params->track->pointsEdgeRight.size() > COLSIMAGE / 2)
            {
                for (int i = 1; i <= 10; i++)
                {
                    height += params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - i].x;
                    height += params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - i].x;
                }
                height = height / 20;
            }

            // 2/3号位至少走满加深后的入库帧数，避免车道重识别提前把车停在外侧。
            if (height > ROWSIMAGE * 0.3 && (!nearSpot || timeout > enterTimeoutLimit)) // 入库结束
            {
                spots.countRes++;
                if (spots.countRes > 2)
                {
                    yforkDiagLog("PARK_EVENT",
                                 "STOP_TRIGGER reason=track_reacquired timeout=%d height=%d threshold=%d countRes=%d",
                                 timeout, height, static_cast<int>(ROWSIMAGE * 0.3), spots.countRes);
                    setStep(Step::PARKING); // 停车完成
                }
            }

            // 入库直行
            params->track->pointsEdgeLeft.clear();  // 清空数据
            params->track->pointsEdgeRight.clear(); // 清空数据
            // 左车道线
            PointX startPoint = PointX(ROWSIMAGE - 30, COLSIMAGE * 0.2);                                    // 入库补线起点:固定左下角
            PointX endPoint = PointX(50, COLSIMAGE * 0.35);                                                 // 入库补线终点
            PointX midPoint = PointX((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); // 入库补线中点
            vector<PointX> repairPoints = {startPoint, midPoint, endPoint};
            params->track->pointsEdgeLeft = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合

            // 右车道
            startPoint = PointX(ROWSIMAGE - 30, COLSIMAGE * 0.8);                                    // 入库补线起点:固定左下角
            endPoint = PointX(50, COLSIMAGE * 0.65);                                                 // 入库补线终点
            midPoint = PointX((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); // 入库补线中点
            repairPoints = {startPoint, midPoint, endPoint};
            params->track->pointsEdgeRight = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        }

        pointsEdgeLeftPast.push_back(params->track->pointsEdgeLeft); // 记录入库路径
        pointsEdgeRightPast.push_back(params->track->pointsEdgeRight);

        if (timeout > enterTimeoutLimit) // 到达对应车位的入库行驶帧数后停车；4号位单独略加深
        {
            yforkDiagLog("PARK_EVENT", "STOP_TRIGGER reason=enter_timeout timeout=%d limit=%d spot=%d",
                         timeout, enterTimeoutLimit, targetSpot);
            setStep(Step::PARKING); // 停车完成
        }
        break;
    }

    case Step::PARKING: // 停车
    {
        params->ctrl.stop = true; // 停车标志
        timeout++;
        if (timeout == 1)
        {
            yforkDiagLog("PARK_EVENT", "PARKING_START stop=%d speed=%.3f pastL=%zu pastR=%zu",
                         params->ctrl.stop, params->ctrl.speed,
                         pointsEdgeLeftPast.size(), pointsEdgeRightPast.size());
        }
        if (timeout > 20) // 停车计时
        {
            yforkDiagLog("PARK_EVENT", "PARKING_DONE hold=%d stop=%d pastL=%zu pastR=%zu",
                         timeout, params->ctrl.stop,
                         pointsEdgeLeftPast.size(), pointsEdgeRightPast.size());
            setStep(Step::EXIT); // 出库
        }

        break;
    }

    case Step::EXIT: // 出库
    {
        timeout++;
        params->ctrl.stop = false; // 停车标志
        params->ctrl.back = true;  // 倒车

        if (timeout == 1 || timeout % 10 == 0)
        {
            yforkDiagLog("PARK_EVENT", "EXIT_PROGRESS timeout=%d back=%d pastL=%zu pastR=%zu speed=%.3f",
                         timeout, params->ctrl.back, pointsEdgeLeftPast.size(), pointsEdgeRightPast.size(), params->ctrl.speed);
        }

        const int targetSpot = params->config.currentLapConfig
                                   ? params->config.currentLapConfig->parkSpot
                                   : 0;
        if (pointsEdgeLeftPast.size() < 1 || pointsEdgeRightPast.size() < 1)
        {
            // 路径回放完毕，继续倒车额外帧数确保完全出库
            const uint16_t exitExtendLimit = (targetSpot == 4)
                                                 ? SPOT4_EXIT_EXTEND_FRAMES
                                                 : ((targetSpot == 3)
                                                        ? RIGHT_EXIT_EXTEND_FRAMES
                                                        : (targetSpot == 2 ? SPOT2_EXIT_EXTEND_FRAMES
                                                                           : EXIT_EXTEND_FRAMES));
            if (exitExtend < exitExtendLimit)
            {
                exitExtend++;
            }
            else
            {
                yforkDiagLog("PARK_EVENT", "EXIT_DONE reason=path_empty timeout=%d back=%d pastL=%zu pastR=%zu extend=%d limit=%d spot=%d",
                             timeout, params->ctrl.back, pointsEdgeLeftPast.size(), pointsEdgeRightPast.size(),
                             static_cast<int>(exitExtend), static_cast<int>(exitExtendLimit), targetSpot);
                setStep(Step::TRACKOUT); // 停车完成
            }
        }
        else
        {
            // 出库轨迹复现
            params->track->pointsEdgeLeft = pointsEdgeLeftPast[pointsEdgeLeftPast.size() - 1];
            params->track->pointsEdgeRight = pointsEdgeRightPast[pointsEdgeRightPast.size() - 1];
            pointsEdgeLeftPast.pop_back(); // 删除最后一组路径
            pointsEdgeRightPast.pop_back();
        }

        break;
    }

    case Step::TRACKOUT: // 出库巡线
    {
        timeout++; // 超时计数
        //[01] 道闸检测
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_GATE)
            {
                const int gateBottom = params->results[i].y + params->results[i].height;
                const bool gateShapeOk = params->results[i].width > 100 &&
                                         params->results[i].height < 130;
                yforkDiagLog("PARK_GATE",
                             "TRACKOUT x=%d y=%d w=%d h=%d bottom=%d score=%.3f shapeOk=%d stopLine=%d",
                             params->results[i].x, params->results[i].y,
                             params->results[i].width, params->results[i].height,
                             gateBottom, params->results[i].score, gateShapeOk,
                             static_cast<int>(ROWSIMAGE * 0.45));
                if (!gateShapeOk)
                    continue;

                timeout = 0; // 超时计数
                countRes = 0;
                if (gateBottom > ROWSIMAGE * 0.45) // 第二道栏杆停车距离计算：阈值越大越靠近栏杆再停
                {
                    stopping = true; // 停车等待标志
                    break;
                }
            }
        }

        //[02] 控制中心重规划
        spots.forks = findParkStation(params->results); // 搜索停车位坐标
        PredictResult resLeft;
        resLeft.score = 0;
        for (int i = 0; i < params->results.size(); i++) // 搜索左转箭头
        {
            if (params->results[i].type == LABEL_LEFT &&
                params->results[i].width < 100 &&
                params->results[i].height < 120 &&
                params->results[i].score > resLeft.score)
                resLeft = params->results[i];
        }

        if (spots.forks.size() > 0 || resLeft.score > 0)
        {
            PredictResult direction;
            if (resLeft.score > 0)
                direction = resLeft;
            else
                direction = spots.forks[0];
            PointX start = PointX(ROWSIMAGE - 20, COLSIMAGE / 2);                // 补线起点：车头
            PointX end = PointX(direction.y, direction.x + direction.width / 2); // 补线终点：AI标志
            PointX mid = PointX((start.x + end.x) / 2, (start.y + end.y) / 2);   // 补线中点
            vector<PointX> repairPoints = {start, mid, end};
            params->ctrl.centerEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
            params->ctrl.fitting = true;                          // 控制中心拟合标志
            if (spots.forks.size() > 0)
                timeout = 0; // 定时入库关闭 | 仅依靠AI标志转向

            if (resLeft.score > 0 &&
                timeout > 10 &&
                (resLeft.y + resLeft.height / 2) > ROWSIMAGE * 0.25)
            {
                countRes++;
            }
        }

        // 出库状态切换：道闸消失/LEFT 满足后确认几帧，再进入出库左转；略提前一点，避免出 Park 太晚。
        if (timeout > 45)
            countRes++; // 出库计数器
        if (countRes > 4)
            setStep(Step::FORKOUT); // 设置停车场新步骤

        break;
    }

    case Step::FORKOUT: // 出库岔路转向
    {
        speedUp = 0;
        timeout++;
        replanTracking(); // 车道线重绘（岔路左转）

        // 搜索左转标志
        bool leftSign = false;
        for (int i = 0; i < params->results.size(); i++) // 搜索左转箭头
        {
            if (params->results[i].type == LABEL_LEFT &&
                params->results[i].width < 100 &&
                params->results[i].height < 120)
            {
                leftSign = true;
                break;
            }
        }
        if (leftSign) // 标志未丢失
            countRes = 0;
        else
            countRes++;

        // 出库左转稍微多跑几帧，避免刚出 Park 时左转量不够。
        if (timeout > 26 || (timeout > 24 && countRes > 2))
        {
            yforkDiagLog("PARK_EVENT",
                         "FORKOUT_DONE lap=%d timeout=%d countRes=%d cfg(yfork=%d lapYfork=%d park=%d slow=%d) mode=%d results=%zu branch=%d guiding=%d",
                         params->currentLap, timeout, countRes, params->config.yfork,
                         params->config.currentLapConfig ? params->config.currentLapConfig->yfork : 0,
                         params->config.park,
                         params->config.currentLapConfig ? params->config.currentLapConfig->slow : 0,
                         static_cast<int>(params->mode), params->results.size(),
                         params->yforkBranch, params->yforkGuiding);
            params->ctrl.countAcc = 50;        // 跳过缓加速，直接恢复速度
            params->ctrl.outlineCooldown = 90; // 出库后约4.5秒内禁用outlineCheck
            params->ctrl.yforkReset = true;    // 通知yfork复位，防止残留forkSeen误触发
            params->ctrl.yforkCooldown = 90;   // 左转完成后约3秒只巡线，屏蔽CHOICE/FORK误触发
            params->ctrl.forceSlowRequest = false;
            slowFallbackArmed = true;          // 等待正常慢行识别，超时后自动触发
            slowFallbackCount = 0;
            setStep(Step::NONE);               // 设置停车场新步骤
        }
        break;
    }
    }

    if (stopping)
    {
        params->ctrl.stop = true;
        params->ctrl.speed = 0;
    }
    else if (step != Step::PARKING)
    {
        params->ctrl.stop = false;
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmPark::show(Mat &img)
{
    if (params->mode != FsmMode::PARK)
        return;

    putText(img, "[5] Park", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    // 绘制边缘点
    for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
    {
        circle(img, Point(params->track->pointsEdgeLeft[i].y, params->track->pointsEdgeLeft[i].x), 2,
               Scalar(0, 255, 0), -1); // 绿色点
    }
    for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
    {
        circle(img, Point(params->track->pointsEdgeRight[i].y, params->track->pointsEdgeRight[i].x), 2,
               Scalar(0, 255, 255), -1); // 黄色点
    }

    // 绘制中心点集
    if (params->ctrl.fitting)
    {
        for (int i = 0; i < params->ctrl.centerEdge.size(); i++)
            circle(img, Point(params->ctrl.centerEdge[i].y, params->ctrl.centerEdge[i].x), 1, Scalar(0, 0, 255), -1);
    }

    string str = "Enable";
    switch (step)
    {
    case Step::FORKIN:
        str = "Forkin";
        break;
    case Step::TRACKIN:
    {
        str = "Trackin";
        for (int i = 0; i < spots.forks.size(); i++) // 绘制停车位箭头标志
        {
            putText(img, to_string(i + 1), Point(spots.forks[i].x + spots.forks[i].width / 2, spots.forks[i].y + spots.forks[i].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
            cv::Rect rect(spots.forks[i].x, spots.forks[i].y, spots.forks[i].width, spots.forks[i].height);
            cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
        }

        // 透视变换视角
        Mat imgIpm = Mat::zeros(Size(COLSIMAGEIPM, ROWSIMAGEIPM), CV_8UC3); // 创建全黑图像
        for (int i = 0; i < spots.carsIpm.size(); i++)
            circle(imgIpm, Point(spots.carsIpm[i].x, spots.carsIpm[i].y), 3, Scalar(0, 0, 255), -1);

        for (int i = 0; i < spots.forksIpm.size(); i++)
            circle(imgIpm, Point(spots.forksIpm[i].x, spots.forksIpm[i].y), 3, Scalar(0, 255, 0), -1);

        // 绘制所有4个车位标签（实际布局：左前=1，左后=2，右后=3，右前=4）
        // 绿色=空闲(spotEnable=true)，红色=占用(spotEnable=false)
        if (spots.forks.size() > 0)
        {
            Scalar c1 = spots.spotEnable[0] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "1", Point(spots.forks[0].x + spots.forks[0].width / 2, spots.forks[0].y + spots.forks[0].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c1, 0.5);
        }
        if (spots.forks.size() > 1)
        {
            Scalar c2 = spots.spotEnable[1] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "2", Point(spots.forks[1].x + spots.forks[1].width / 2, spots.forks[1].y + spots.forks[1].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c2, 0.5);

            Scalar c3 = spots.spotEnable[2] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "3", Point(spots.forks[1].x + spots.forks[1].width / 2 + 100, spots.forks[1].y + spots.forks[1].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c3, 0.5);

            Scalar c4 = spots.spotEnable[3] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "4", Point(COLSIMAGE - 31, spots.forks[1].y + spots.forks[1].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c4, 0.5);
        }
        savePicture("../res/samples/imgs/", imgIpm);
        savePicture("../res/samples/imgs/", img);
        break;
    }
    case Step::ENTER:
        str = "Enter";
        break;
    case Step::PARKING:
        str = "Parking";
        break;
    case Step::EXIT:
        str = "Exit";
        break;
    case Step::TRACKOUT:
        str = "TRACKOUT";
        break;
    case Step::FORKOUT:
        str = "Forkout";
        break;
    default:
        break;
    }
    putText(img, "PARK - " + str, Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

/**
 * @brief 设置指定停车位
 */
void FsmPark::setParkSpotOverride(int spotNumber)
{
    std::cout << "[Park] setParkSpotOverride: " << spotNumber << std::endl;

    // 使能指定的停车位
    for (int i = 0; i < 4; i++)
    {
        spots.spotEnable[i] = false; // 先禁用所有停车位
    }

    if (spotNumber > 0 && spotNumber <= 4)
    {
        spots.spotEnable[spotNumber - 1] = true; // 使能指定的停车位
        std::cout << "[Park] Enabled spot " << spotNumber << std::endl;
    }
}

/**
 * @brief 设置指定停车位被占用
 */
void FsmPark::setParkSpotOccupied(int spotNumber)
{
    if (spotNumber > 0 && spotNumber <= 4)
    {
        spots.spotEnable[spotNumber - 1] = false; // 设置指定停车位被占用

        // 在计数器中增加，使其被判定为已占用
        spots.counter[spotNumber - 1] = 5; // 设置足够大的计数值

        // 为其他停车位设置不占用状态
        for (int i = 0; i < 4; i++)
        {
            if (i != (spotNumber - 1))
            {
                // 清除其他停车位的占用状态
                if (spots.counter[i] > 3)
                {
                    spots.spotEnable[i] = true;
                }
            }
        }
    }
}

/**
 * @brief 停车场数据复位
 *
 */
void FsmPark::reset()
{
    countSes = 0;      // AI场景识别计数器
    timeout = 0;       // 超时计数器
    step = Step::NONE; // 停车步骤
    waiting = false;   // 停车等待使能
    countWait = 0;     // 停车等待计数器
    countOut = 0;      // 出库检测计数
    countIn = 0;       // 入库矫正计数器
    speedUp = 0;       // 出库加速延迟计数器
    countFlow = 0;     // 直行计数器
    choiceDone = false; // 允许下一次停车场流程重新识别CHOICE
    exitExtend = 0;     // 复位额外倒车计数器
}

/**
 * @brief 设置下阶段
 *
 * @param step
 */
void FsmPark::setStep(Step st)
{
    Step previous = step;
    countRes = 0; // AI场景识别计数器
    countSes = 0; // 场次计数器
    timeout = 0;  // 超时计数器
    step = st;    // 停车步骤
    countIn = 0;  // 入库矫正计数器
    countFlow = 0;
    params->ctrl.back = false; // 倒车失能
    yforkDiagLog("PARK_EVENT", "STEP old=%d new=%d spot=%d timeout_reset=1",
                 static_cast<int>(previous), static_cast<int>(step),
                 params->config.currentLapConfig ? params->config.currentLapConfig->parkSpot : -1);
}

/**
 * @brief 车道线重绘（岔路左转）
 *
 */
void FsmPark::replanTracking()
{
    params->track->pointsEdgeLeft.clear();  // 清空原来数据
    params->track->pointsEdgeRight.clear(); // 清空原来数据

    // 左车道线
    PointX startPoint = PointX(ROWSIMAGE - 10, 1);                                                // 入库补线起点:固定左下角
    PointX endPoint = PointX(ROWSIMAGE / 3, 1);                                                   // 入库补线终点
    PointX midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 入库补线中点
    vector<PointX> repairPoints = {startPoint, midPoint, endPoint};
    vector<PointX> modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
    params->track->pointsEdgeLeft = modifyEdge;

    // 右车道线
    startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE * 0.8);                                     // 入库补线起点:固定左下角
    endPoint = PointX(ROWSIMAGE / 3, 30);                                                     // 入库补线终点
    midPoint = PointX((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.35); // 入库补线中点（更靠下，左转更早）
    repairPoints = {startPoint, midPoint, endPoint};
    modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
    params->track->pointsEdgeRight = modifyEdge;
}
/**
 * @brief 停车入库车道线重绘
 *
 * @param left true：左转 | false：右转
 */
void FsmPark::replanTracking(bool left)
{
    params->track->pointsEdgeLeft.clear();  // 清空原来数据
    params->track->pointsEdgeRight.clear(); // 清空原来数据

    if (left)
    {
        // 左车道线
        PointX startPoint = PointX(ROWSIMAGE - 10, 1);                                                // 入库补线起点:固定左下角
        PointX endPoint = PointX(ROWSIMAGE / 3, 1);                                                   // 入库补线终点
        PointX midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 入库补线中点
        vector<PointX> repairPoints = {startPoint, midPoint, endPoint};
        vector<PointX> modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeLeft = modifyEdge;
        // 右车道线
        startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE * 0.8);                                  // 入库补线起点:固定左下角
        endPoint = PointX(ROWSIMAGE / 3, 1);                                                   // 入库补线终点
        midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 入库补线中点
        repairPoints = {startPoint, midPoint, endPoint};
        modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeRight = modifyEdge;
    }
    else
    {
        // 3/4号位在右侧，入库路径整体左收；EXIT倒车会回放这条路径，避免出库后太靠右影响栏杆检测。
        const int rightParkLeftBias = 0;
        // 左车道线
        PointX startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE * 0.2 - rightParkLeftBias);              // 入库补线起点
        PointX endPoint = PointX(ROWSIMAGE / 3, COLSIMAGE - 1 - rightParkLeftBias);                   // 入库补线终点
        PointX midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 入库补线中点
        vector<PointX> repairPoints = {startPoint, midPoint, endPoint};
        vector<PointX> modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeLeft = modifyEdge;
        // 右车道线
        startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE - 1 - rightParkLeftBias);                 // 入库补线起点
        endPoint = PointX(ROWSIMAGE / 3, COLSIMAGE - 1 - rightParkLeftBias);                    // 入库补线终点
        midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 入库补线中点
        repairPoints = {startPoint, midPoint, endPoint};
        modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeRight = modifyEdge;
    }
}

/**
 * @brief 搜索AI标志
 *
 * @param label
 * @return true
 * @return false
 */
bool FsmPark::findSymbols(vector<PredictResult> results, int label)
{
    for (int i = 0; i < results.size(); i++)
    {
        if (results[i].type == label) // AI标志
            return true;
    }

    return false;
}

/**
 * @brief 搜索AI标志（最顶上的一个）
 *
 * @param label
 * @param target 目标标志信息
 * @return true
 * @return false
 */
bool FsmPark::findSymbols(vector<PredictResult> results, int label, PredictResult &target)
{
    bool res = false;
    target.y = ROWSIMAGE - 1;
    for (int i = 0; i < results.size(); i++)
    {
        if (results[i].type == label) // AI标志
        {
            if (results[i].y < target.y && results[i].width < 120 && results[i].height < 120)
            {
                target = results[i];
                res = true; // 搜索到目标AI
            }
        }
    }

    return res;
}

/**
 * @brief 搜索停车位坐标
 *
 * @param results
 */
vector<PredictResult> FsmPark::findParkStation(vector<PredictResult> results)
{
    vector<PredictResult> resFork;
    std::cout << "[Park] findParkStation: Searching in " << results.size() << " results" << std::endl;

    for (int i = 0; i < results.size(); i++)
    {
        if (results[i].type == LABEL_FORK)
        {
            std::cout << "[Park] Found fork[" << i << "]: x=" << results[i].x
                      << ", y=" << results[i].y
                      << ", w=" << results[i].width
                      << ", h=" << results[i].height
                      << ", score=" << results[i].score << std::endl;

            if (results[i].width < 100 && results[i].height < 120 && results[i].y > 15)
            {
                resFork.push_back(results[i]);
                std::cout << "[Park] Fork[" << i << "] added to resFork" << std::endl;
            }
            else
            {
                std::cout << "[Park] Fork[" << i << "] filtered out (size=" << results[i].width << "x" << results[i].height
                          << ", y=" << results[i].y << ")" << std::endl;
            }
        }
    }

    std::cout << "[Park] Total forks after initial filter: " << resFork.size() << std::endl;

    vector<PredictResult> resSpot;   // size=0:无停车位，size=1:停车位1/2，size=2:停车位1/2/3/4
    vector<PredictResult> resFilter; // 滤波后的坐标

    if (resFork.size() < 1) // 未检测AI标志
        return resSpot;

    if (resFork.size() > 0)
    {
        resFilter.push_back(resFork[0]);
        std::cout << "[Park] Added first fork to filter" << std::endl;

        for (int i = 1; i < resFork.size(); i++) // Start from 1 since index 0 is already added
        {
            bool added = false;
            for (int n = 0; n < resFilter.size(); n++)
            {
                PointX centerFilter = getResultCenter(resFilter[n]);
                PointX centerResult = getResultCenter(resFork[i]);
                if (centerFilter.x > 0 && centerFilter.y > 0 &&
                    centerResult.x > 0 && centerResult.y > 0) // 数据有效
                {
                    if (abs(centerFilter.y - centerResult.y) > 30) // 两个坐标不重叠（降低阈值）
                    {
                        bool newFork = true;
                        for (int y = 0; y < resFilter.size(); y++) // 重复筛选
                        {
                            PointX centerF = getResultCenter(resFilter[y]);
                            if (abs(centerF.y - centerResult.y) < 30) // 数据重复
                            {
                                newFork = false;
                                break;
                            }
                        }
                        if (newFork)
                        {
                            resFilter.push_back(resFork[i]);
                            std::cout << "[Park] Added fork[" << i << "] to filter (new fork)" << std::endl;
                            added = true;
                        }
                        else if (resFork[i].score > resFilter[n].score) // 更新数据
                        {
                            resFilter[n] = resFork[i];
                            std::cout << "[Park] Updated fork[" << n << "] in filter (higher score)" << std::endl;
                            added = true;
                        }
                    }
                    else if (resFork[i].score > resFilter[n].score) // 更新数据
                    {
                        resFilter[n] = resFork[i];
                        std::cout << "[Park] Updated fork[" << n << "] in filter (closer but higher score)" << std::endl;
                        added = true;
                    }
                }
            }
            if (!added)
            {
                std::cout << "[Park] Fork[" << i << "] not added to filter (duplicate or invalid)" << std::endl;
            }
        }
    }

    std::cout << "[Park] Final resFilter size: " << resFilter.size() << std::endl;

    if (resFilter.size() == 1) // 一个车位
    {
        resSpot = resFilter;
        std::cout << "[Park] Returning 1 spot" << std::endl;
    }
    else if (resFilter.size() == 2) // 两个车位标识
    {
        std::cout << "[Park] Found 2 spots!" << std::endl;
        std::cout << "[Park] Spot 0: y=" << resFilter[0].y << ", center_y=" << (resFilter[0].y + resFilter[0].height / 2) << std::endl;
        std::cout << "[Park] Spot 1: y=" << resFilter[1].y << ", center_y=" << (resFilter[1].y + resFilter[1].height / 2) << std::endl;

        if ((resFilter[0].y + resFilter[0].height / 2) < (resFilter[1].y + resFilter[1].height / 2))
        {
            resSpot = resFilter;
            std::cout << "[Park] Spots in original order" << std::endl;
        }
        else // 车位号重新排序
        {
            resSpot.push_back(resFilter[1]);
            resSpot.push_back(resFilter[0]);
            std::cout << "[Park] Spots reordered" << std::endl;
        }
    }
    else
    {
        std::cout << "[Park] Unexpected resFilter size: " << resFilter.size() << std::endl;
    }

    std::cout << "[Park] Returning " << resSpot.size() << " spots" << std::endl;
    return resSpot;
}

/**
 * @brief 获取AI检测目标的质心坐标
 *
 * @param res
 * @return PointX
 */
PointX FsmPark::getResultCenter(PredictResult res)
{
    PointX center;
    center.x = res.x + res.width / 2;
    center.y = res.y + res.height / 2;

    if (center.x < 0)
        center.x = 0;
    else if (center.x > COLSIMAGE)
        center.x = COLSIMAGE;

    if (center.y < 0)
        center.y = 0;
    else if (center.y > ROWSIMAGE)
        center.y = ROWSIMAGE;

    return center;
}

/**
 * @brief 检测停车位上是否有车
 *
 * @param results
 * @return vector<PredictResult>
 */
void FsmPark::findParkCars(vector<PredictResult> results)
{
    // 数据复位
    for (int i = 0; i < spots.carPark.size(); i++)
        spots.carPark[i].score = 0;

    if (spots.forks.size() != 2) // 基于岔路标志检测车位
        return;

    if (spots.forks[1].y < ROWSIMAGE / 4) // 图像清晰度限制
        return;

    spots.forksIpm.clear();
    spots.carsIpm.clear();
    Point2d spot1Ipm = ipm.homography(Point2d(spots.forks[0].y, spots.forks[0].x));
    Point2d spot2Ipm = ipm.homography(Point2d(spots.forks[1].y, spots.forks[1].x));
    spots.forksIpm.push_back(spot1Ipm);
    spots.forksIpm.push_back(spot2Ipm);
}

void FsmPark::resetLap()
{
    reset();
    spots.reset();
    pointsEdgeLeftPast.clear();
    pointsEdgeRightPast.clear();
    stopping = false;
    fitting = false;
    countRes = 0;
}
