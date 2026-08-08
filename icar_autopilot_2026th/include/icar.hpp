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
 * @file icar.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 智能汽车控制（TOP）
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"
#include "fsm/manualControl.hpp"
#include "fsm/fork.hpp"
#include "fsm/park.hpp"
#include "fsm/stop.hpp"
#include "fsm/cross.hpp"
#include "fsm/yfork.hpp"
#include "fsm/station.hpp"
#include "fsm/slow.hpp"
#include "fsm/obstacle.hpp"
#include "com/client.hpp"
#include "utils/detection.hpp"
#include "utils/yfork_diag.hpp"
#include "utils/show.hpp"
#include "utils/loop.hpp"
#include "ctrl/predeal.hpp"
#include "ctrl/center.hpp"
#include "ctrl/motion.hpp"
#include <cstdio>
#include <ctime>


using namespace std;
using namespace cv;

class Icar
{
private:
    /**
     * @brief 状态机管理
     *
     */
    struct FsmFactory
    {
        shared_ptr<FsmBusy> busy;               // 避障控制
        shared_ptr<FsmPark> park;               // 停车场控制
        shared_ptr<FsmStop> stop;               // 停车区控制
        shared_ptr<FsmCross> cross;             // 斑马线停车控制
        shared_ptr<FsmFork> fork;               // 停车场岔路控制
        shared_ptr<FsmSlow> slow;               // 慢行区控制
        shared_ptr<FsmObstacle> obstacle;       // 全局障碍物检测（锥桶/行人）
        shared_ptr<FsmYfork> yfork;             // Y型岔路口控制
        shared_ptr<FsmStation> station;         // 停靠站控制
        shared_ptr<ManualControlThread> manual; // 手动接管控制
    };

    FsmFactory fsmFactory;                // 状态机管理
    shared_ptr<Predeal> predeal;          // 图像预处理类
    shared_ptr<Show> show;                // 初始化UI显示窗口
    shared_ptr<cv::VideoCapture> capture; // Opencv相机类
    shared_ptr<Detection> detection;      // 目标检测类
    shared_ptr<Client> client;            // TCP客户端通信类
    shared_ptr<Params> params;            // 车辆状态参数（FSM共享传递）
    shared_ptr<Loops> loops;              // 子线程循环
    shared_ptr<Center> center;            // 控制中心处理类
    shared_ptr<Motion> motion;            // 运动控制器

    int lastLap = 0; // 上一圈号（检测圈变更时复位FSM）

    // 全局共享数据链
    cv::Mat imgShare;
    std::mutex mtxImg;
    std::condition_variable cvImg;
    std::atomic<bool> readyImg{false};
    std::mutex mtxRes;
    std::atomic<bool> readyRes{false};

    /**
     * @brief 鼠标的事件回调函数
     *
     */
    static void callbackMouse(int event, int x, int y, int flags, void *userdata)
    {
        Icar *self = static_cast<Icar *>(userdata);
        if (self)
            self->handleMouse(event, x, y, flags);
    }
    void handleMouse(int event, int x, int y, int flags)
    {
        double value;
        switch (event)
        {
        case cv::EVENT_MOUSEWHEEL: // 鼠标滑球
        {
            value = cv::getMouseWheelDelta(flags); // 获取滑球滚动值
            if (value > 0)
                show->index++;
            else if (value < 0)
                show->index--;

            if (show->index < 0)
                show->index = 0;
            if (show->index > show->frameMax)
                if (show->index > show->frameMax)
                    show->index = show->frameMax;
            break;
        }
        default:
            break;
        }
    }

    /**
     * @brief AI 模型推理
     *
     */
    void runModel()
    {
        std::unique_lock<std::mutex> lock(mtxImg);
        cvImg.wait(lock, [this]
                   { return readyImg.load(); });
        cv::Mat img = imgShare.clone(); // 图像拷贝出来再释放锁
        readyImg = false;
        lock.unlock();

        // 手动接管期间跳过AI推理
        if (!params->manualTakeover)
        {
            // 启动AI推理
            detection->inference(img);
            std::lock_guard<std::mutex> lock_result(mtxRes);
            params->results = detection->results;
            readyRes = true;
        }
    }

    /**
     * @brief 有限状态机任务执行
     *
     */
    void runFsm(Mat &img)
    {
        // 圈数变更时复位所有FSM状态
        if (lastLap != params->currentLap)
        {
            fprintf(stderr, "[LAP] ====== 进入第 %d/%d 圈 ======\n", params->currentLap, params->config.totalLaps);
            lastLap = params->currentLap;
            fsmFactory.stop->resetLap();
            fsmFactory.park->resetLap();
            fsmFactory.fork->resetLap();
            fsmFactory.yfork->resetLap();
            fsmFactory.slow->resetLap();
            fsmFactory.busy->resetLap();
            fsmFactory.station->resetLap();
            fsmFactory.obstacle->resetLap();
            if (!fsmFactory.manual->isManualControl())
                fsmFactory.manual->disconnectClient(); // 清理上一圈的远程连接残留，避免isConnected误判
            params->alertCountdown = 0;                // 复位蜂鸣器报警
            params->alertDecelCount = 0;               // 复位报警减速
            params->busyAlertCountdown = 0;            // 复位施工区蜂鸣
        }

        if (params->mode == FsmMode::STATION ||
            params->mode == FsmMode::FORK ||
            params->mode == FsmMode::BUSY ||
            params->mode == FsmMode::SLOW ||
            params->mode == FsmMode::YFORK) // 状态复位
            params->mode = FsmMode::NORMAL;

        // 处理手动接管（优先执行，跳过所有FSM检测）
        if (!fsmFactory.busy->isInManualTakeover() && fsmFactory.manual->isConnected())
        {
            cout << "[Icar] Remote connected, starting manual takeover" << endl;
            fsmFactory.busy->startManualTakeover();
        }
        params->manualTakeover = fsmFactory.busy->isInManualTakeover();
        if (params->manualTakeover)
        {
            cout << "[Icar] Manual takeover active." << endl;
            fsmFactory.manual->updateVehicleState(params->ctrl.speed, params->ctrl.servo);

            // 检查是否返回自动模式
            if (fsmFactory.manual->checkForReturnKey())
            {
                fsmFactory.busy->endManualTakeover();
                fsmFactory.manual->disconnectClient(); // 断开TCP连接，恢复自动控制
                params->ctrl.stop = false;
                params->mode = FsmMode::NORMAL;   // 恢复自动模式
                params->takeoverJustEnded = true; // 通知各FSM手动接管刚结束
            }
            else if (fsmFactory.manual->isManualControl())
            {
                fsmFactory.manual->applyManualControl(&params->ctrl.speed, &params->ctrl.servo);
                params->ctrl.stop = false;
            }
            else
            {
                params->ctrl.stop = true;
                params->ctrl.speed = 0;
                params->ctrl.servo = PWMSERVOMID;
            }

            if (params->mode != params->modeLast)
            {
                client->buzzerSound(client->BUZZER_DING); // 提示音效
                params->modeLast = params->mode;
            }
            return; // 手动接管期间跳过所有FSM检测
        }

        // 根据当前圈配置设置功能使能（覆盖全局配置）
        params->config.fork = params->config.currentLapConfig->fork;
        params->config.park = params->config.currentLapConfig->park;
        params->config.busy = params->config.currentLapConfig->busy;
        params->config.slow = params->config.currentLapConfig->slow;
        params->config.stop = params->config.currentLapConfig->stop;
        params->config.cross = params->config.currentLapConfig->cross;
        const bool yforkEnabled = params->config.currentLapConfig->yfork;
        params->config.yfork = yforkEnabled;
        params->config.station = params->config.currentLapConfig->station;
        params->config.obstacle = params->config.currentLapConfig->obstacle;
        static int yforkRouteLogCnt = 0;
        if (yforkRouteLogCnt++ % 30 == 0)
        {
            yforkDiagLog("ROUTE",
                         "lap=%d mode=%d yforkEnabled=%d yforkLeft=%d fork=%d park=%d "
                         "busy=%d station=%d results=%zu branch=%d guiding=%d "
                         "stationStarted=%d stationDone=%d",
                         params->currentLap, static_cast<int>(params->mode), yforkEnabled,
                         params->config.currentLapConfig->yforkLeft, params->config.fork,
                         params->config.park, params->config.busy, params->config.station,
                         params->results.size(), params->yforkBranch, params->yforkGuiding,
                         params->stationStarted, params->stationStopCompleted);
        }




        auto traceMode = [&](const char *stage, FsmMode before) {
            if (params->mode != before || params->mode == FsmMode::YFORK || before == FsmMode::YFORK || params->currentLap == 3)
            {
                yforkDiagLog("FSM_TRACE",
                             "stage=%s lap=%d before=%d after=%d cfg(yfork=%d park=%d fork=%d station=%d slow=%d busy=%d) results=%zu branch=%d guiding=%d stationStarted=%d stationDone=%d",
                             stage, params->currentLap, static_cast<int>(before), static_cast<int>(params->mode),
                             yforkEnabled, params->config.park, params->config.fork, params->config.station,
                             params->config.slow, params->config.busy, params->results.size(),
                             params->yforkBranch, params->yforkGuiding, params->stationStarted,
                             params->stationStopCompleted);
            }
        };

        FsmMode modeBefore = params->mode;
        fsmFactory.stop->run(img); // 停车区识别与规划
        params->mode = fsmFactory.stop->getMode();
        traceMode("stop", modeBefore);

        modeBefore = params->mode;
        fsmFactory.cross->run(img); // 斑马线停车识别与规划
        params->mode = fsmFactory.cross->getMode();
        traceMode("cross", modeBefore);

        // ===== 路线隔离：仅在当前圈使能时调用对应FSM =====
        if (params->config.park)
        {
            if (params->mode == FsmMode::NORMAL || params->mode == FsmMode::PARK)
            {
                modeBefore = params->mode;
                fsmFactory.park->run(img);
                FsmMode mode = fsmFactory.park->getMode();
                if (mode != FsmMode::NORMAL)
                    params->mode = mode;
                traceMode("park", modeBefore);
            }
        }
        if (params->config.fork)
        {
            if (params->mode == FsmMode::NORMAL)
            {
                modeBefore = params->mode;
                fsmFactory.fork->run(img);
                params->mode = fsmFactory.fork->getMode();
                traceMode("fork", modeBefore);
            }
        }
        if (yforkEnabled)
        {
            if (params->mode == FsmMode::NORMAL)
            {
                modeBefore = params->mode;
                yforkDiagLog("FSM_TRACE",
                             "stage=yfork_call lap=%d before=%d cfgYfork=%d lapYfork=%d results=%zu branch=%d guiding=%d",
                             params->currentLap, static_cast<int>(modeBefore), params->config.yfork,
                             params->config.currentLapConfig ? params->config.currentLapConfig->yfork : 0,
                             params->results.size(), params->yforkBranch, params->yforkGuiding);
                fsmFactory.yfork->run(img);
                params->mode = fsmFactory.yfork->getMode();
                traceMode("yfork", modeBefore);
            }
            else if (params->currentLap == 3 || params->mode == FsmMode::YFORK)
            {
                yforkDiagLog("FSM_TRACE",
                             "stage=yfork_not_called lap=%d mode=%d cfgYfork=%d lapYfork=%d results=%zu branch=%d guiding=%d",
                             params->currentLap, static_cast<int>(params->mode), params->config.yfork,
                             params->config.currentLapConfig ? params->config.currentLapConfig->yfork : 0,
                             params->results.size(), params->yforkBranch, params->yforkGuiding);
            }
        }
        else
        {
            static int yforkDisabledLogCnt = 0;
            if (params->currentLap == 3 || yforkDisabledLogCnt++ % 30 == 0)
            {
                static bool parkRouteLogFirstWrite = true;
                FILE *fp = fopen("./park.log", parkRouteLogFirstWrite ? "w" : "a");
                parkRouteLogFirstWrite = false;
                if (fp)
                {
                    time_t now = time(nullptr);
                    struct tm *t = localtime(&now);
                    fprintf(fp, "[%02d:%02d:%02d] [YforkRoute] skip yfork: lap=%d yforkEnabled=0 mode=%d results=%zu branch=%d guiding=%d\n",
                            t->tm_hour, t->tm_min, t->tm_sec,
                            params->currentLap, static_cast<int>(params->mode),
                            params->results.size(), params->yforkBranch, params->yforkGuiding);
                    fclose(fp);
                }
            }

            // 即使上一圈曾经进入YFORK，本圈也绝不保留其引导、分支和检测状态。
            fsmFactory.yfork->deactivate();
        }

        if (params->config.slow)
        {
            fsmFactory.slow->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.slow->getMode();
        }
        if (params->config.busy)
        {
            fsmFactory.busy->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.busy->getMode();
            // 施工区鸣笛（每秒3次，间隔10帧≈333ms）
            if (params->busyAlertCountdown > 0)
            {
                if (params->busyAlertCountdown % 10 == 0)
                    client->buzzerSound(client->BUZZER_WARNNING);
                params->busyAlertCountdown--;
            }
        }
        if (params->config.station)
        {
            fsmFactory.station->run(img);
            FsmMode stationMode = fsmFactory.station->getMode();
            // 不覆盖YFORK模式，让yfork状态能正常显示
            if (stationMode != FsmMode::NORMAL && params->mode != FsmMode::YFORK)
                params->mode = stationMode;
        }

        // 全局障碍物检测（锥桶/行人），施工区/停车场/Y型岔路口期间不检测
        if (params->config.obstacle &&
            params->mode != FsmMode::BUSY &&
            params->mode != FsmMode::PARK &&
            params->mode != FsmMode::YFORK)
            fsmFactory.obstacle->run(img);

        // 蜂鸣器报警目标检测（图像下方4/5区域），遇目标持续鸣笛1s
        if (params->config.alertTarget != "none")
        {
            int alertLabel = -1;
            if (params->config.alertTarget == "cone")
                alertLabel = LABEL_CONE;
            else if (params->config.alertTarget == "person")
                alertLabel = LABEL_PERSON;
            else if (params->config.alertTarget == "busy")
                alertLabel = LABEL_BUSY;
            else if (params->config.alertTarget == "limit")
                alertLabel = LABEL_LIMIT;
            else if (params->config.alertTarget == "unlimit")
                alertLabel = LABEL_UNLIMIT;
            else if (params->config.alertTarget == "park")
                alertLabel = LABEL_PARK;

            // 施工区由FSM自身负责鸣笛，不重复触发
            if (alertLabel == LABEL_BUSY && params->config.currentLapConfig &&
                params->config.currentLapConfig->busy)
                alertLabel = -1;

            if (alertLabel >= 0)
            {
                bool targetFound = false;
                {
                    std::lock_guard<std::mutex> lock(mtxRes);
                    for (auto &r : params->results)
                    {
                        bool posOk = (alertLabel == LABEL_LIMIT || alertLabel == LABEL_PARK)
                                         ? (r.x > COLSIMAGE * 0.8 || r.x + r.width < COLSIMAGE * 0.2) // 左右两侧1/5
                                         : (r.y + r.height) > ROWSIMAGE * 0.2;                        // 其他：底部4/5区域
                        if (r.type == alertLabel && posOk)
                        {
                            targetFound = true;
                            break;
                        }
                    }
                }
                if (targetFound && params->alertCountdown <= 0)
                    params->alertCountdown = 30; // 启动最少1秒倒计时
            }
            if (params->alertCountdown > 0)
            {
                if (params->alertCountdown % 10 == 0) // 每~333ms鸣笛一次，最少3次
                    client->buzzerSound(client->BUZZER_WARNNING);
                params->alertCountdown--;
            }
        }

        {
            bool decelFound = false;
            {
                std::lock_guard<std::mutex> lock(mtxRes);
                for (auto &r : params->results)
                {
                    bool posOk = false;
                    if (r.type == LABEL_LIMIT || r.type == LABEL_PARK)
                        posOk = (r.x > COLSIMAGE * 0.8 || r.x + r.width < COLSIMAGE * 0.2); // 左右两侧1/5
                    else if (r.type == LABEL_CONE || r.type == LABEL_PERSON || r.type == LABEL_UNLIMIT)
                        posOk = (r.y + r.height) > ROWSIMAGE * 0.2; // 底部4/5区域
                    else if (r.type == LABEL_BUSY)
                    {
                        if (params->config.currentLapConfig && params->config.currentLapConfig->busy)
                            continue;
                        posOk = (r.y + r.height) > ROWSIMAGE * 0.2;
                    }
                    else
                        continue;

                    if (posOk)
                    {
                        decelFound = true;
                        break;
                    }
                }
            }
            if (decelFound)
                params->alertDecelCount = 5;
            else if (params->alertDecelCount > 0)
                params->alertDecelCount--;
        }

        if (params->mode != params->modeLast && params->alertCountdown <= 0)
        {
            client->buzzerSound(client->BUZZER_DING); // 提示音效
            params->modeLast = params->mode;
        }
    }

public:
    /**
     * @brief 参数初始化
     *
     */
    Icar()
    {
        params = make_shared<Params>();                        // 初始化参数
        center = make_shared<Center>();                        // 控制中心处理类
        motion = make_shared<Motion>();                        // 运动控制器
        predeal = make_shared<Predeal>(params->config.binary); // 图像预处理类
        detection = make_shared<Detection>(params->config.model,
                                           params->config.score); // AI模型初始化

        // 初始化TCP通信客户端
        client = make_shared<Client>();
        if (!client->start())
        {
            printf("[Error]: Socket init failed!!!\n");
            exit(-1);
        }
        // client->buzzerSound(client->BUZZER_OK); // 提示音效

        // 相机初始化
        // USB摄像头初始化
        printf("[Debug] in [08], debug=%d manual=%d\n", params->config.debug, params->manualTakeover);
        fflush(stdout);
        if (params->config.debug)
            capture = make_shared<cv::VideoCapture>(params->config.video); // 打开本地视频
        else
            capture = make_shared<cv::VideoCapture>("/dev/v4l/by-id/usb-XCX-230919-H_PC_Camera_A4-video-index0"); // 打开摄像头
        if (!capture->isOpened())
        {
            printf("[Error]: Can not open video device!!!\n");
            exit(-1);
        }
        capture->set(cv::CAP_PROP_FRAME_WIDTH, COLSCAMERA);  // 设置图像分辨率
        capture->set(cv::CAP_PROP_FRAME_HEIGHT, ROWSCAMERA); // 设置图像分辨率
        capture->set(cv::CAP_PROP_FPS, 30);                  // 设置帧率

        if (params->config.debug)
        {
            show = make_shared<Show>(4); // 调试UI初始化
            show->frameMax = capture->get(cv::CAP_PROP_FRAME_COUNT) - 1;
            cv::createTrackbar("Frame", "ICAR", &show->index, show->frameMax, [](int, void *) {}); // 创建Opencv图像滑条控件
            cv::setMouseCallback("ICAR", this->callbackMouse);                                     // 创建鼠标键盘快捷键事件
        }

        // FSM有限状态机初始化
        fsmFactory.busy = make_shared<FsmBusy>(params);         // 避障控制实例化
        fsmFactory.park = make_shared<FsmPark>(params);         // 停车场控制实例化
        fsmFactory.stop = make_shared<FsmStop>(params);         // 斑马线停车控制实例化
        fsmFactory.cross = make_shared<FsmCross>(params);       // 斑马线停车控制实例化
        fsmFactory.fork = make_shared<FsmFork>(params);         // 停车场岔路控制实例化
        fsmFactory.slow = make_shared<FsmSlow>(params);         // 慢行区控制实例化
        fsmFactory.obstacle = make_shared<FsmObstacle>(params); // 全局障碍物检测实例化
        fsmFactory.yfork = make_shared<FsmYfork>(params);       // Y型岔路口控制实例化
        fsmFactory.station = make_shared<FsmStation>(params);   // 停靠站控制实例化

        // 手动控制线程初始化
        fsmFactory.manual = make_shared<ManualControlThread>();

        // 启动AI推理子线程
        loops = make_shared<Loops>("LoopAI", 1.f / 30.f, std::bind(&Icar::runModel, this));
        loops->start(); // RL开始推理

        // 启动手动控制线程
        fsmFactory.manual->start();

        printf("[OK]: Params initial succeed!!!\n");
    };
    ~Icar()
    {
        if (fsmFactory.manual)
        {
            fsmFactory.manual->stop();
        }
    };

    /**
     * @brief 程序主循环
     *
     */
    void running()
    {
        //[01] 视频源读取
        cv::Mat img;
        if (params->config.debug) // 综合显示调试UI窗口
        {
            if (show->indexLast == show->index) // 图像帧未更新
            {
                if (client->keypress)
                {
                    client->buzzerSound(client->BUZZER_FINISH); // 祖传提示音效
                    printf("-----> System Exit!!! <-----\n");
                    exit(0); // 程序退出
                }
                show->show();        // 显示综合绘图
                client->sendHeart(); // 发送给服务器在线心跳
                usleep(10 * 1000);   // us延迟
                return;
            }

            capture->set(cv::CAP_PROP_POS_FRAMES, show->index); // 设置读取帧
            if (!capture->read(img))
                return;
            show->indexLast = show->index;
        }
        else if (!capture->read(img))
            return;

        //[02] 图像存储
        if (params->config.saveImg && !params->config.debug) // 存储原始图像
            savePicture(img);
        else if (params->config.saveImg && params->config.debug) // 存储调式图像
            show->save = true;

        //[03] 图像预处理
        cv::Mat imgBin;
        predeal->correction(img); // 图像矫正
        /*---------------子线程共享数据，避免浅拷贝-----------------*/
        std::lock_guard<std::mutex> lock(mtxImg);
        imgShare = img.clone();
        readyImg = true;
        cvImg.notify_one();
        /*-------------------------------------------------------*/
        imgBin = predeal->binaryzation(img); // 图像二值化

        //[04] 赛道识别（手动接管时跳过）
        if (!fsmFactory.busy->isInManualTakeover())
            params->track->handle(imgBin);
        if (params->config.debug)
        {
            show->setNewWindow(1, "Bin", imgBin);
            cv::Mat imgTrack = img.clone();
            params->track->drawImage(imgTrack); // 图像绘制赛道识别结果
            show->setNewWindow(2, "Track", imgTrack);
            if (params->config.saveIpm && params->config.saveImg)
            {
                cv::Mat imgIpm;
                ipm.homography(imgTrack, imgIpm);
                savePicture(imgIpm); // 保存图像
            }
        }

        //[05] 有限状态机任务执行
        params->ctrl.fitting = false;
        runFsm(imgBin);

        // 同步手动接管状态（runFsm中endManualTakeover可能改变了状态，但params->manualTakeover未更新）
        params->manualTakeover = fsmFactory.busy->isInManualTakeover();

        // 手动接管期间发送彩色图像
        if (params->manualTakeover && fsmFactory.manual->isConnected())
            fsmFactory.manual->sendImage(img);

        //[06] 控制中心计算（手动接管时跳过）
        if (!params->manualTakeover)
            center->fitting(params);

        //[07] 车辆运动控制（手动接管或远程连接时跳过，保持远程控制值）
        if (!fsmFactory.busy->isInManualTakeover() && !fsmFactory.manual->isConnected())
        {
            motion->poseControl(params);
            motion->speedControl(params);
        }

        // 记录本帧最终真正发送给车辆的控制量。前面的YFORK_FRAME/STATION_FRAME
        // 解释决策来源，这一行用于确认最终是否右打、是否减速、是否下发停车。
        if ((params->config.currentLapConfig && params->config.currentLapConfig->yfork) ||
            params->yforkBranch != 0 || params->stationStarted)
        {
            int mcServo = 90;
            if (params->ctrl.servo <= 1100)
                mcServo = 120;
            else if (params->ctrl.servo >= 1900)
                mcServo = 60;
            else
                mcServo = 90 - static_cast<int16_t>(params->ctrl.servo - 1500) * 30 / 400;

            const auto &left = params->track->pointsEdgeLeft;
            const auto &right = params->track->pointsEdgeRight;
            const PointX leftLast = left.empty() ? PointX(-1, -1) : left.back();
            const PointX rightLast = right.empty() ? PointX(-1, -1) : right.back();
            yforkDiagLog("CTRL_FINAL",
                         "lap=%d mode=%d branch=%d guiding=%d stationStarted=%d stationDone=%d "
                         "center=%d error=%d centerEdge=%zu speed=%.3f servoPWM=%u mcServo=%d "
                         "direction=%s stop=%d trackL=%zu last=(%d,%d) trackR=%zu last=(%d,%d)",
                         params->currentLap, static_cast<int>(params->mode), params->yforkBranch,
                         params->yforkGuiding, params->stationStarted,
                         params->stationStopCompleted, params->ctrl.center,
                         params->ctrl.center - COLSIMAGE / 2, params->ctrl.centerEdge.size(),
                         params->ctrl.speed, params->ctrl.servo, mcServo,
                         mcServo > 90 ? "RIGHT" : (mcServo < 90 ? "LEFT" : "STRAIGHT"),
                         params->ctrl.stop, left.size(), leftLast.x, leftLast.y,
                         right.size(), rightLast.x, rightLast.y);
        }

        //[08] 综合显示调试UI窗口
        if (params->config.debug)
        {
            detection->drawBox(img);        // 图像绘制AI结果
            center->drawImage(params, img); // 图像绘制控制路径
            motion->drawImage(params, img); // 图像绘制速度
            show->setNewWindow(3, "Ctrl", img);

            // 特殊区域图像处理结果显示
            Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
            fsmFactory.busy->show(imgRes);
            fsmFactory.park->show(imgRes);
            fsmFactory.stop->show(imgRes);
            fsmFactory.cross->show(imgRes);
            fsmFactory.fork->show(imgRes);
            fsmFactory.slow->show(imgRes);
            fsmFactory.obstacle->show(imgRes);
            fsmFactory.yfork->show(imgRes);
            show->setNewWindow(4, "FSM", imgRes);
        }
        else // 实车控制
        {

            // 无论手动/自动模式，每帧发送控制指令（MCU需要持续PWM更新）
            client->carControl(params->ctrl.speed, params->ctrl.servo);
        }
    }
};
