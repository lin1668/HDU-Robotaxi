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
 * @file stop.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停车区规划与控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 停车区规划与控制
 *
 */
class FsmStop : public FSMState
{
public:
  FsmStop(std::shared_ptr<Params> par);
  ~FsmStop();
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
    NONE = 0, // AI未识别
    ENABLE,   // 场景使能
    STOP      // 停车
  };
  Step step = Step::NONE;     // 场景状态
  uint16_t countRec = 0;      // AI场景识别计数器
  uint16_t countSes = 0;      // 场次计数器
  int timeout = 0;            // 超时计数器
  vector<cv::Point> polyRoad; // 赛道多边形点集
  vector<cv::Point> polyCar;  // 智能车多边形点集
  double overlap = 0;         // 重合度
  float scoreLap = 0.5;       // 智能车在赛道上的置信度

  double getRoadCarPloy(PredictResult result);
  double getOverlapArea(const std::vector<cv::Point> &polyA, const std::vector<cv::Point> &polyB);
  void drawPolygon(Mat img, vector<Point> poly, bool fill, bool close = true);
  void setStep(Step st);
};
