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
 * @file cross.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 斑马线停车控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 斑马线停车控制
 *
 */
class FsmCross : public FSMState
{
public:
  FsmCross(std::shared_ptr<Params> par);
  ~FsmCross();
  void run(Mat &img);
  void show(Mat &img);
  FsmMode getMode();
  bool checkCrossPass();

private:
  /**
   * @brief 场景状态
   *
   */
  enum Step
  {
    NONE = 0, // AI未识别
    ENABLE,   // 场景使能
    STOP      // 停车
  };

  Step step = Step::NONE; // 场景状态
  uint16_t countRec = 0;  // AI场景识别计数器
  uint16_t countSes = 0;  // 场次计数器
  int timeout = 0;        // 超时计数器
  int countCross = 0;     // 斑马线屏蔽计数器
  int countInit = 0;      // 起点屏蔽计数器
  int crossLostCount = 0;      // checkCrossPass用：复位crossPassed（>10帧）
  int crossLostStepCount = 0;  // ENABLE步用：触发STOP（>=3帧）
  int crossCount = 0;          // 累计检测到cross次数

  void setStep(Step st);
};
