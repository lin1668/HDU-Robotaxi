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
 * @file running.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 智能汽车控制（TOP）: 基于UART通信控制，避免一键自启出现问题
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"
#include "fsm/fork.hpp"
#include "fsm/park.hpp"
#include "fsm/slow.hpp"
#include "fsm/stop.hpp"
#include "fsm/cross.hpp"
#include "fsm/yfork.hpp"
#include "com/uart.hpp"
#include "utils/detection.hpp"
#include "utils/show.hpp"
#include "utils/loop.hpp"
#include "ctrl/predeal.hpp"
#include "ctrl/center.hpp"
#include "ctrl/motion.hpp"
#include <fstream>
#include <sys/types.h>
#include <sys/wait.h>

using namespace std;
using namespace cv;

class Running
{
private:
    /**
     * @brief 状态机管理
     *
     */
    struct FsmFactory
    {
        shared_ptr<FsmBusy> busy;   // 避障控制
        shared_ptr<FsmPark> park;   // 停车场控制
        shared_ptr<FsmStop> stop;   // 停车区控制
        shared_ptr<FsmCross> cross; // 斑马线停车控制
        shared_ptr<FsmFork> fork;   // 停车场岔路控制
        shared_ptr<FsmSlow> slow;   // 慢行区控制
        shared_ptr<FsmYfork> yfork; // Y型岔路口控制
    };

    FsmFactory fsmFactory;                // 状态机管理
    shared_ptr<Predeal> predeal;          // 图像预处理类
    shared_ptr<Show> show;                // 初始化UI显示窗口
    shared_ptr<cv::VideoCapture> capture; // Opencv相机类
    shared_ptr<Detection> detection;      // 目标检测类
    shared_ptr<Uart> uart;                // UART通信类
    shared_ptr<Params> params;            // 车辆状态参数（FSM共享传递）
    shared_ptr<Loops> loops;              // 子线程循环
    shared_ptr<Center> center;            // 控制中心处理类
    shared_ptr<Motion> motion;            // 运动控制器

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
        Running *self = static_cast<Running *>(userdata);
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

        // 启动AI推理
        try
        {
            detection->inference(img);
        }
        catch (const std::exception &e)
        {
            std::cout << "[AI] Inference exception: " << e.what() << std::endl;
            detection->results.clear();
        }
        catch (...)
        {
            std::cout << "[AI] Unknown inference exception" << std::endl;
            detection->results.clear();
        }
        std::lock_guard<std::mutex> lock_result(mtxRes);
        params->results = detection->results;
        readyRes = true;
    }

    /**
     * @brief 有限状态机任务执行
     *
     */
    void runFsm(Mat &img)
    {
        // 手动接管标志（禁用出界exit）
        params->manualTakeover = fsmFactory.busy->isInManualTakeover();

        // 根据当前圈配置设置功能使能
        params->config.fork = params->config.currentLapConfig->fork;
        params->config.park = params->config.currentLapConfig->park;
        params->config.busy = params->config.currentLapConfig->busy;
        params->config.slow = params->config.currentLapConfig->slow;
        params->config.stop = params->config.currentLapConfig->stop;
        params->config.cross = params->config.currentLapConfig->cross;
        params->config.yfork = params->config.currentLapConfig->yfork;

        // 设置停车场停车使能和指定停车位
        if (params->config.park && params->config.currentLapConfig->parkSpot > 0)
        {
            params->config.parkSpot = params->config.currentLapConfig->parkSpot;
            // 设置指定停车位，使其他三个停车位都被占用
            if (fsmFactory.park)
            {
                fsmFactory.park->setParkSpotOverride(params->config.parkSpot);
            }
        }
        else if (fsmFactory.park)
        {
            fsmFactory.park->setParkSpotOverride(0); // 清除指定停车位
        }

        if (params->mode == FsmMode::FORK ||
            params->mode == FsmMode::BUSY ||
            params->mode == FsmMode::SLOW ||
            params->mode == FsmMode::YFORK) // 状态复位
            params->mode = FsmMode::NORMAL;

        fsmFactory.stop->run(img); // 停车区识别与规划
        params->mode = fsmFactory.stop->getMode();
        fsmFactory.cross->run(img); // 斑马线停车识别与规划
        params->mode = fsmFactory.cross->getMode();

        // ===== 路线隔离：仅在当前圈使能时调用对应FSM =====
        // 三条路线互斥（park / busy / fork+yfork），
        // 未使能路线的检测与处理被完全跳过
        if (params->config.park) // 停车场图像处理
        {
            if (params->mode == FsmMode::NORMAL || params->mode == FsmMode::PARK || params->mode == FsmMode::CROSS)
            {
                fsmFactory.park->run(img);
                FsmMode mode = fsmFactory.park->getMode();
                if (mode != FsmMode::NORMAL)
                    params->mode = mode;
            }
        }
        if (params->config.fork) // 岔路识别与规划
        {
            if (params->mode == FsmMode::NORMAL)
            {
                fsmFactory.fork->run(img);
                params->mode = fsmFactory.fork->getMode();
            }
        }
        if (params->config.slow) // 慢行区识别与规划
        {
            fsmFactory.slow->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.slow->getMode();
        }
        if (params->config.busy) // 施工区识别与规划
        {
            fsmFactory.busy->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.busy->getMode();
        }
        if (params->config.yfork) // Y型岔路口识别与规划
        {
            fsmFactory.yfork->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.yfork->getMode();
        }

        if (params->mode != params->modeLast)
        {
            uart->buzzerSound(uart->BUZZER_DING); // 提示音效
            params->modeLast = params->mode;
        }
    }

public:
    /**
     * @brief 参数初始化
     *
     */
    Running()
    {
        params = make_shared<Params>();                        // 初始化参数
        center = make_shared<Center>();                        // 控制中心处理类
        motion = make_shared<Motion>();                        // 运动控制器
        predeal = make_shared<Predeal>(params->config.binary); // 图像预处理类
        detection = make_shared<Detection>(params->config.model,
                                           params->config.score); // AI模型初始化

        // 初始化TCP通信客户端
        uart = make_shared<Uart>();
        if (uart->open("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0") < 0)
        {
            printf("[Error]: Uart init failed!!!\n");
            exit(-1);
        }
        uart->buzzerSound(uart->BUZZER_OK); // 提示音效

        // 相机初始化
        // USB摄像头初始化
        if (params->config.debug)
            capture = make_shared<cv::VideoCapture>(params->config.video); // 打开本地视频
        else
            capture = make_shared<cv::VideoCapture>("/dev/video0"); // 打开摄像头
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
        fsmFactory.busy = make_shared<FsmBusy>(params);   // 避障控制实例化
        fsmFactory.park = make_shared<FsmPark>(params);   // 停车场控制实例化
        fsmFactory.stop = make_shared<FsmStop>(params);   // 斑马线停车控制实例化
        fsmFactory.cross = make_shared<FsmCross>(params); // 斑马线停车控制实例化
        fsmFactory.fork = make_shared<FsmFork>(params);   // 停车场岔路控制实例化
        fsmFactory.slow = make_shared<FsmSlow>(params);   // 慢行区控制实例化
        fsmFactory.yfork = make_shared<FsmYfork>(params); // Y型岔路口控制实例化

        // 启动AI推理子线程
        loops = make_shared<Loops>("LoopAI", 1.f / 30.f, std::bind(&Running::runModel, this));
        loops->start(); // RL开始推理

        printf("[OK]: Params initial succeed!!!\n");
    };
    ~Running() {};

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
                if (uart->keypress)
                {
                    uart->buzzerSound(uart->BUZZER_FINISH); // 祖传提示音效
                    printf("-----> System Exit!!! <-----\n");
                    exit(0); // 程序退出
                }
                show->show();      // 显示综合绘图
                uart->sendHeart(); // 发送给服务器在线心跳
                usleep(10 * 1000); // us延迟
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

        //[04] 赛道识别
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

        //[06] 控制中心计算
        center->fitting(params);

        //[07] 车辆运动控制（手动接管或远程连接时跳过，保持远程控制值）
        if (!fsmFactory.busy->isInManualTakeover() && !fsmFactory.manual->isConnected())
        {
            motion->poseControl(params);
            motion->speedControl(params);
        }

        //[07.5] 轻量存图：saveImg开且debug关时，每3帧存一张带叠加的调试图
        static int saveCnt = 0;
        if (params->config.saveImg && !params->config.debug && (++saveCnt % 3 == 0))
        {
            detection->drawBox(img);
            center->drawImage(params, img);
            fsmFactory.yfork->show(img);  // yfork引导线（绿左/黄右/V尖）
            savePicture(img);
        }

        //[08] 综合显示调试UI窗口
        if (params->config.debug)
        {
            detection->drawBox(img);        // 图像绘制AI结果
            center->drawImage(params, img); // 图像绘制控制路径
            fsmFactory.yfork->drawTip(img); // Y型岔路V尖标记
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
            fsmFactory.yfork->show(imgRes);
            fsmFactory.station->show(imgRes);
            show->setNewWindow(4, "FSM", imgRes);
        }
        else // 实车控制
        {
            uart->carControl(params->ctrl.speed, params->ctrl.servo); // 串口通信控制车辆
        }
    }
};

int main(int argc, char const *argv[])
{
    Running icar;

    // 强制杀死TCP通信进程
    std::system("killall -9 icar boot");
    sleep(1);

    const std::chrono::milliseconds durations(1000 / 30); // 控制周期：30Fps
    while (1)
    {
        auto timeStart = std::chrono::high_resolution_clock::now();
        icar.running(); // 系统主线程

        // 计算处理耗时
        auto timeEnd = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart);
        // printf(">> FrameTime: %ldms | %.2ffps \n", elapsed.count(), 1000.0 / elapsed.count());

        // 睡眠等待
        if (elapsed < durations)
            std::this_thread::sleep_for(durations - elapsed);
    }

    return 0;
}
