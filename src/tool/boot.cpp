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
 * @file boot.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 智能车自启Boot程序
 * @version 0.1
 * @date 2024-09-03
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include "com/server.hpp"
#include <string>
#include <fstream>
#include <sys/types.h>
#include <sys/wait.h>

using namespace std;
int launchCmd(const std::string &workdir, const std::string &cmd, bool wait);

int main(int argc, char const *argv[])
{
    // 套接字服务器
    Server server;

    if (!server.start()) // 启动socket监听子线程
        return -1;

    printf("Boot is running!\n");
    while (1)
    {
        if (server.startApp)
        {
            server.countDrop++;
            if (server.countDrop > 10) // 2s
                server.startApp = false;
        }
        else
            // server.uart.sendHeart(); // 发送心跳信号

        usleep(200 * 1000); // us延迟

        if (server.uart.killAll) // 强制杀进程
        {
            server.uart.killAll = false;
            server.uart.carControl(0, 1500); // 停车
            std::system("killall -9 icar collection img2video calibration camera detection main");
            server.startApp = false;

            printf("Kill all app!\n");
        }
        else if (server.uart.keypress) // 按键
        {
            server.uart.keypress = false;
            server.transmit("Keypress");
            if (!server.startApp) // 应用未启动
            {
                printf("App icar-v1 is running!\n");
                // system("gnome-terminal export DISPLAY=:0.0 --working-directory=/root/workspace/icar_autopilot_2025th/build -- ./icar");
                launchCmd("/root/workspace/icar_autopilot_2025th/build/", "./icar", false);
                server.startApp = true;
            }
            else
            {
                std::system("pkill -f icar");
                printf("App was killed!\n");
            }
            usleep(2000 * 1000); // us延迟
        }
        else if (server.uart.exitBoot) // 退出Boot
        {
            server.uart.exitBoot = false;
            std::system("killall -9 icar collection img2video calibration camera detection main");
            server.closeServer(); // 关闭socket通信
            break;
        }
    }

    printf("Boot was closed!\n");
    server.closeServer(); // 关闭socket通信

    return 0;
}

/**
 * @brief 启动命令
 *
 * @param workdir 工作目录
 * @param cmd 命令
 * @param wait 是否等待命令执行完毕
 * @return int 命令执行状态
 */
int launchCmd(const std::string &workdir, const std::string &cmd, bool wait)
{
    pid_t pid = fork();

    if (pid == 0)
    {
        // 子进程
        if (!workdir.empty())
        {
            if (chdir(workdir.c_str()) != 0)
            {
                perror("chdir failed");
                _exit(1);
            }
        }
        execl("/bin/sh", "sh", "-c", cmd.c_str(), (char *)NULL);
        perror("execl failed");
        _exit(1);
    }
    else if (pid > 0)
    {
        // 父进程
        if (wait)
        {
            int status;
            if (waitpid(pid, &status, 0) == -1)
            {
                perror("waitpid failed");
                return -1;
            }
            if (WIFEXITED(status))
            {
                return WEXITSTATUS(status); // 返回退出码
            }
            else if (WIFSIGNALED(status))
            {
                std::cerr << "Process killed by signal " << WTERMSIG(status) << std::endl;
                return -1;
            }
            return -1;
        }
        else
        {
            return pid; // 后台运行，返回子进程pid
        }
    }
    else
    {
        perror("fork failed");
        return -1;
    }
}

// gnome-terminal export DISPLAY=:0.0 --working-directory=/root/workspace/icar_autopilot_2025th/build -- ./boot