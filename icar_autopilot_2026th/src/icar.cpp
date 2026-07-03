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

#include "icar.hpp"

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
    Icar icar;
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