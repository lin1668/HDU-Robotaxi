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
 * @file loop.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 子线程控制器
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <sstream>
#include <iomanip>

class Loops
{
public:
    Loops(const std::string &name, double period, std::function<void()> func, int bindCPU = -1)
        : _name(name), _period(period), _func(func), _bindCPU(bindCPU), _running(false) {}

    void start()
    {
        _running = true;
        log("[Loop Start] named: " + _name + ", period: " + formatPeriod() + "(ms)" + (_bindCPU != -1 ? ", run at cpu: " + std::to_string(_bindCPU) : ", cpu unspecified"));
        if (_bindCPU != -1)
        {
            _thread = std::thread(&Loops::loop, this);
            setThreadAffinity(_thread.native_handle(), _bindCPU);
        }
        else
        {
            _thread = std::thread(&Loops::loop, this);
        }
        _thread.detach();
    }

    void shutdown()
    {
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _running = false;
            _cv.notify_one();
        }
        if (_thread.joinable())
        {
            _thread.join();
        }
        log("[Loop End] named: " + _name);
    }

private:
    std::string _name;
    double _period;
    std::function<void()> _func;
    int _bindCPU;
    std::atomic<bool> _running;
    std::mutex _mutex;
    std::condition_variable _cv;
    std::thread _thread;

    void loop()
    {
        while (_running)
        {
            auto start = std::chrono::steady_clock::now();

            try
            {
                _func();
            }
            catch (const std::exception &e)
            {
                std::cout << "[Loop] Exception in " << _name << ": " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cout << "[Loop] Unknown exception in " << _name << std::endl;
            }

            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            auto sleepTime = std::chrono::milliseconds(static_cast<int>((_period * 1000) - elapsed.count()));
            if (sleepTime.count() > 0)
            {
                std::unique_lock<std::mutex> lock(_mutex);
                if (_cv.wait_for(lock, sleepTime, [this]
                                 { return !_running; }))
                {
                    break;
                }
            }
        }
    }

    std::string formatPeriod() const
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(0) << _period * 1000;
        return stream.str();
    }

    void log(const std::string &message)
    {
        static std::mutex logMutex;
        std::lock_guard<std::mutex> lock(logMutex);
        std::cout << message << std::endl;
    }

    void setThreadAffinity(std::thread::native_handle_type threadHandle, int cpuId)
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpuId, &cpuset);
        if (pthread_setaffinity_np(threadHandle, sizeof(cpu_set_t), &cpuset) != 0)
        {
            std::ostringstream oss;
            oss << "Error setting thread affinity: CPU " << cpuId << " may not be valid or accessible.";
            throw std::runtime_error(oss.str());
        }
    }
};
