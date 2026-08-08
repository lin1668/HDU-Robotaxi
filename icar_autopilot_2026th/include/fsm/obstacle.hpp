#pragma once
/**
 ********************************************************************************************************
 * @file obstacle.hpp
 * @brief Global obstacle detection and avoidance for cones / pedestrians.
 ********************************************************************************************************
 */

#include "utils/params.hpp"

class FsmObstacle
{
public:
  FsmObstacle(std::shared_ptr<Params> par);
  ~FsmObstacle();
  void run(Mat &img);
  void show(Mat &img);
  void resetLap();

  PredictResult resultObs; // Current obstacle target for display.

private:
  std::shared_ptr<Params> params;
  int obstacleHoldFrames = 0;       // Frames to keep avoidance after the obstacle box disappears.
  int obstacleHoldSide = 0;         // 1=left object, keep shifting right; 2=right object, keep shifting left.
  PredictResult obstacleHoldResult; // Last valid obstacle used during hold.
  void curtailTracking(bool left);  // Shrink lane line from double-lane to single-lane avoidance.
};