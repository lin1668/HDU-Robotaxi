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
 * @file park.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停车场控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 * 入库：（parkInsideStep）
 *          1：车库识别（初始化）
 *          2：岔路转弯模式设定
 *          3：道闸前停车识别
 *          4：识别入库标志与当前停车情况
 *          5：开始转弯入库
 *          6：入库结束
 * 出库：（parkOutsideStep）
 *          1：车库识别（初始化）
 *          2：倒车出库
 *          3：识别道闸停车
 *          4：出道闸进入岔路转弯模式
 *          5：退出转弯模式并退出出库阶段
 */

#include "fsm/fsm.hpp"

/**
 * @brief 停车场控制
 *
 */
class FsmPark : public FSMState
{
public:
  FsmPark(std::shared_ptr<Params> par);
  ~FsmPark();
  void run(Mat &img);
  void show(Mat &img);
  FsmMode getMode();
  void resetLap();

  bool stopping = false; // 停车等待标志
  bool fitting = false;  // 控制中心拟合标志
  uint16_t speedUp = 0;  // 出库加速延迟计数器
  float spotUp = 0.51;    // 上停车位距离像素百分比[0,1]
  float spotDown = 0.37;  // 下停车位距离像素百分比[0,1]
  float rightSpotEnterDelay = -0.03; // 4号右侧车位入库触发额外延后比例；越大越晚开始右转入库
  float rightNearEnterDelay = -0.06; // 3号右侧近处车位入库触发额外延后比例；越大越晚开始右转入库

private:
  /**
   * @brief 车位信息
   *
   */
  class Spots
  {
  private:
    PredictResult carNone;

  public:
    vector<PredictResult> forks; // 停车位箭头信息（size=0:无停车位，size=1:停车位1/2，size=2:停车位1/2/3/4）
    vector<Point2d> forksIpm;    // 停车标志IPM坐标
    vector<Point2d> carsIpm;     // 车辆IPM坐标
    int counter[4] = {0};        // 车位检测计数器
    bool spotEnable[4] = {true}; // 停车位使能标志
    int countRes = 0;            // 车位检测计数器
    bool checked = false;        // 已确定停车位编号标志
    int times = 0;               // 临时计数器

    vector<PredictResult> carPark = {carNone, carNone, carNone, carNone}; // 1/2/3/4号停车位检测车辆信息

    void reset()
    {
      for (int i = 0; i < 4; i++)
      {
        counter[i] = 0;
        spotEnable[i] = true;
      }
      forks.clear();
      countRes = 0; // 车位检测计数器
      checked = false;
      times = 0;
    }
  };

  /**
   * @brief 停车步骤
   *
   */
  enum Step
  {
    NONE = 0, // 未知状态
    ENABLE,   // 停车场使能
    FORKIN,   // 入库岔路转向
    TRACKIN,  // 入库巡线
    ENTER,    // 入库
    PARKING,  // 停车
    EXIT,     // 出库
    TRACKOUT, // 出库巡线
    FORKOUT,  // 出库岔路转向

  };
  Spots spots;                                                    // 车位信息
  Step step = Step::NONE;                                         // 停车步骤
  uint16_t countFlow = 0;                                         // 直行计数器
  uint16_t countRes = 0;                                          // AI场景识别计数器
  uint16_t countSes = 0;                                          // 场次计数器
  uint16_t timeout = 0;                                           // 超时计数器
  int countOut = 0;                                               // 出库检测计数
  bool waiting = false;                                           // 停车等待使能
  int countWait = 0;                                              // 停车等待计数器
  int countIn = 0;                                                // 入库矫正计数器
  bool slowFallbackArmed = false;                                 // 出库后慢行保险已启动
  uint16_t slowFallbackCount = 0;                                 // 等待正常进入慢行区的帧数
  bool choiceDone = false;                                        // 本次停车场流程已执行CHOICE入库
  uint16_t exitExtend = 0;                                        // EXIT路径耗尽后额外倒车帧数
  static constexpr uint16_t EXIT_EXTEND_FRAMES = 4;              // 1号位路径耗尽后额外倒车
  static constexpr uint16_t SPOT2_EXIT_EXTEND_FRAMES = 4;        // 2号位缩短额外倒车，减少出库等待
  static constexpr uint16_t RIGHT_EXIT_EXTEND_FRAMES = 3;        // 3号右侧车位路径耗尽后额外倒车
  static constexpr uint16_t SPOT4_EXIT_EXTEND_FRAMES = 6;        // 4号位单独多倒一点，帮助车身出库后回正
  static constexpr uint16_t SLOW_FALLBACK_FRAMES = 20;            // 约3秒未进入慢行则强制触发
  vector<vector<PointX>> pointsEdgeLeftPast, pointsEdgeRightPast; // 记录赛道入库车道线

  void setStep(Step st);
  void reset();
  void replanTracking();
  void replanTracking(bool left);
  bool findSymbols(vector<PredictResult> results, int label);
  bool findSymbols(vector<PredictResult> results, int label, PredictResult &target);
  vector<PredictResult> findParkStation(vector<PredictResult> results);
  PointX getResultCenter(PredictResult res);
  void findParkCars(vector<PredictResult> results);
  void setParkSpotOverride(int spotNumber); // 设置指定停车位
  void setParkSpotOccupied(int spotNumber); // 设置指定停车位被占用
};
