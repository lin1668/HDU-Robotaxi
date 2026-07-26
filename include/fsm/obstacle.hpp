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
 * @file obstacle.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 全局障碍物检测（锥桶/行人）
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "utils/params.hpp"

/**
 * @brief 全局障碍物检测（锥桶/行人）
 *
 */
class FsmObstacle
{
public:
  FsmObstacle(std::shared_ptr<Params> par);
  ~FsmObstacle();
  void run(Mat &img);
  void show(Mat &img);
  void resetLap();

  PredictResult resultObs;  // 当前避障目标（用于显示）

private:
  std::shared_ptr<Params> params;
  void curtailTracking(bool left);  // 缩减优化车道线（双车道→单车道）
};
