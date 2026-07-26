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
 * @file fork.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停车场岔路图像识别与规划
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 停车场岔路图像识别与规划
 *
 */
class FsmFork : public FSMState
{
public:
  FsmFork(std::shared_ptr<Params> par);
  ~FsmFork();
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
    NONE = 0, // 未知类型
    ENTER,    // 入环
    EXIT,     // 出环
    END       // 环任务结束
  };
  bool enable = false;    // 状态使能
  Step step = Step::NONE; // 环岛处理阶段
  bool repairing = false; // 判断是否正在补线
  int counterFork = 0;    // 记录一个状态的图像场数，如果卡某个状态控制可通过此标志调整出状态时间
  PointX lastForkL;       // 左下有效连接点（位置更低才有效，否则就是找错了）
  PointX lastForkR;       // 右下有效连接点
  uint16_t countRec = 0;  // AI场景识别计数器
  uint16_t countSes = 0;  // 场次计数器
  uint16_t countExit = 0; // 程序退出计数器

  void reset(void);
  bool handle(Mat &img, int type = 0);
  PointX searchFilletLeftUp(vector<PointX> pointsEdgeLeft, PointX leftFilletDown);
  PointX searchFilletLeftDown(vector<PointX> pointsEdgeLeft, vector<PointX> pointsEdgeRight, int rowsEnd = -1, bool straightSideJudge = true);
  PointX searchFilletRightDown(vector<PointX> pointsEdgeRight, vector<PointX> pointsEdgeLeft, int rowsEnd = -1, bool straightSideJudge = true);
  int countFormerStickpointL(vector<PointX> pointsEdgeLeft, int StartLine = 0, int Endline = ROWSIMAGE * 2 / 3);
  int countFormerStickpointR(vector<PointX> pointsEdgeRight, int StartLine = 0, int Endline = ROWSIMAGE * 2 / 3);
};
