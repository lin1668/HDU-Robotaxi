/**
 ********************************************************************************************************
 * @file obstacle.cpp
 * @brief Global obstacle detection and avoidance for cones / pedestrians.
 ********************************************************************************************************
 */

#include "fsm/obstacle.hpp"
#include "utils/tools.hpp"
#include <cstdarg>
#include <cstdio>
#include <ctime>

namespace
{
// Cone avoidance lateral clearance = detection box width * multiplier.
constexpr int CONE_AVOID_WIDTH_MULTIPLIER = 3;
// Keep avoidance for a few frames after the obstacle box disappears to avoid early recentering.
constexpr int OBSTACLE_AVOID_HOLD_FRAMES = 5;
}

static void obstacleLog(const char *fmt, ...)
{
    static bool firstWrite = true;
    FILE *fp = fopen("./obstacle.log", firstWrite ? "w" : "a");
    firstWrite = false;
    if (!fp)
        return;

    time_t now = time(nullptr);
    tm *t = localtime(&now);
    if (t)
        fprintf(fp, "[%02d:%02d:%02d] ", t->tm_hour, t->tm_min, t->tm_sec);

    va_list args;
    va_start(args, fmt);
    vfprintf(fp, fmt, args);
    va_end(args);
    fprintf(fp, "\n");
    fclose(fp);
}

FsmObstacle::FsmObstacle(std::shared_ptr<Params> par)
    : params(par)
{
}

FsmObstacle::~FsmObstacle()
{
}

void FsmObstacle::run(Mat &img)
{
    resultObs = PredictResult();
    params->ctrl.obstacleSeen = false;

    if (params->track->pointsEdgeLeft.size() < ROWSIMAGE / 2 ||
        params->track->pointsEdgeRight.size() < ROWSIMAGE / 2)
    {
        obstacleHoldFrames = 0;
        obstacleHoldSide = 0;
        return;
    }

    // Detect cones and pedestrians that are close enough and have reasonable box size.
    vector<PredictResult> resultsObs;
    for (int i = 0; i < params->results.size(); i++)
    {
        if ((params->results[i].type == LABEL_CONE || params->results[i].type == LABEL_PERSON) &&
            (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.4 &&
            params->results[i].height < 100 && params->results[i].width < 90 &&
            params->results[i].height > 20 && params->results[i].width > 20)
            resultsObs.push_back(params->results[i]);
    }

    bool usingObstacleHold = false;
    int index = 0;
    if (resultsObs.size() <= 0)
    {
        if (obstacleHoldFrames <= 0 || obstacleHoldSide == 0 ||
            obstacleHoldResult.width <= 0 || obstacleHoldResult.height <= 0)
            return;

        resultsObs.push_back(obstacleHoldResult);
        usingObstacleHold = true;
        --obstacleHoldFrames;
        obstacleLog("OBSTACLE_HOLD side=%s remaining=%d type=%d x=%d y=%d w=%d h=%d",
                    obstacleHoldSide == 1 ? "LEFT_OBJECT" : "RIGHT_OBJECT",
                    obstacleHoldFrames, obstacleHoldResult.type,
                    obstacleHoldResult.x, obstacleHoldResult.y,
                    obstacleHoldResult.width, obstacleHoldResult.height);
    }
    else
    {
        // Pick the nearest/largest obstacle candidate.
        int areaMax = 0;
        for (int i = 0; i < resultsObs.size(); i++)
        {
            int area = resultsObs[i].width * resultsObs[i].height;
            if (area >= areaMax)
            {
                index = i;
                areaMax = area;
            }
        }
    }

    resultObs = resultsObs[index];
    params->ctrl.obstacleSeen = true;

    // Find the track row nearest to the obstacle vertical position.
    int row = 0, width = COLSIMAGE;
    for (size_t i = 0; i < params->track->pointsEdgeLeft.size(); i++)
    {
        int w = abs(resultObs.y - params->track->pointsEdgeLeft[i].x);
        if (w < 2)
        {
            row = i;
            break;
        }
        if (w < width)
        {
            width = w;
            row = i;
        }
    }
    if (row > params->track->pointsEdgeRight.size() - 1)
        row = params->track->pointsEdgeRight.size() - 1;

    int disLeft = resultsObs[index].x - params->track->pointsEdgeLeft[row].y;
    int disRight = params->track->pointsEdgeRight[row].y - (resultsObs[index].x + resultsObs[index].width);
    const bool forceLeftObject = usingObstacleHold && obstacleHoldSide == 1;
    const bool forceRightObject = usingObstacleHold && obstacleHoldSide == 2;
    bool avoidApplied = false;

    static int obstacleDetectLogCounter = 0;
    if (obstacleDetectLogCounter++ % 5 == 0)
        obstacleLog("OBSTACLE_DETECT lap=%d mode=%d type=%d x=%d y=%d w=%d h=%d row=%d disLeft=%d disRight=%d trackL=%zu trackR=%zu slow=%d hold=%d",
                    params->currentLap, static_cast<int>(params->mode),
                    resultsObs[index].type, resultsObs[index].x, resultsObs[index].y,
                    resultsObs[index].width, resultsObs[index].height, row,
                    disLeft, disRight,
                    params->track->pointsEdgeLeft.size(),
                    params->track->pointsEdgeRight.size(), params->ctrl.slow,
                    usingObstacleHold ? obstacleHoldFrames : 0);

    if (forceLeftObject ||
        (resultsObs[index].x + resultsObs[index].width > params->track->pointsEdgeLeft[row].y &&
         params->track->pointsEdgeRight[row].y > resultsObs[index].x &&
         abs(disLeft) <= abs(disRight))) // Obstacle is closer to left side: shift right.
    {
        avoidApplied = true;
        if (!usingObstacleHold)
        {
            obstacleHoldFrames = OBSTACLE_AVOID_HOLD_FRAMES;
            obstacleHoldSide = 1;
            obstacleHoldResult = resultsObs[index];
        }

        obstacleLog("OBSTACLE_AVOID side=LEFT_OBJECT action=shift_right type=%d disLeft=%d disRight=%d slowLock=1 hold=%d",
                    resultsObs[index].type, disLeft, disRight, obstacleHoldFrames);
        if (resultsObs[index].type == LABEL_PERSON)
            curtailTracking(false);
        else
        {
            vector<PointX> points(4);
            points[0] = params->track->pointsEdgeLeft[row / 2];
            points[1] = {(int)(resultsObs[index].y + resultsObs[index].height * 1.5),
                         resultsObs[index].x + resultsObs[index].width * CONE_AVOID_WIDTH_MULTIPLIER};
            points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2,
                         resultsObs[index].x + resultsObs[index].width * CONE_AVOID_WIDTH_MULTIPLIER};
            if (resultsObs[index].y > params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1].x)
                points[3] = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1];
            else
                points[3] = {resultsObs[index].y, resultsObs[index].x + resultsObs[index].width};

            params->track->pointsEdgeLeft.resize((size_t)row / 2);
            vector<PointX> repair = Bezier(0.01, points);
            for (int i = 0; i < repair.size(); i++)
                params->track->pointsEdgeLeft.push_back(repair[i]);
        }
        params->ctrl.slow = true;
    }
    else if (forceRightObject ||
             (resultsObs[index].x + resultsObs[index].width > params->track->pointsEdgeLeft[row].y &&
              params->track->pointsEdgeRight[row].y > resultsObs[index].x &&
              abs(disLeft) > abs(disRight))) // Obstacle is closer to right side: shift left.
    {
        avoidApplied = true;
        if (!usingObstacleHold)
        {
            obstacleHoldFrames = OBSTACLE_AVOID_HOLD_FRAMES;
            obstacleHoldSide = 2;
            obstacleHoldResult = resultsObs[index];
        }

        obstacleLog("OBSTACLE_AVOID side=RIGHT_OBJECT action=shift_left type=%d disLeft=%d disRight=%d slowLock=1 hold=%d",
                    resultsObs[index].type, disLeft, disRight, obstacleHoldFrames);
        if (resultsObs[index].type == LABEL_PERSON)
            curtailTracking(true);
        else
        {
            vector<PointX> points(4);
            points[0] = params->track->pointsEdgeRight[row / 2];
            points[1] = {resultsObs[index].y + resultsObs[index].height,
                         resultsObs[index].x - resultsObs[index].width * CONE_AVOID_WIDTH_MULTIPLIER};
            points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2,
                         resultsObs[index].x - resultsObs[index].width * CONE_AVOID_WIDTH_MULTIPLIER};
            if (resultsObs[index].y > params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1].x)
                points[3] = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1];
            else
                points[3] = {resultsObs[index].y, resultsObs[index].x};

            params->track->pointsEdgeRight.resize((size_t)row / 2);
            vector<PointX> repair = Bezier(0.01, points);
            for (int i = 0; i < repair.size(); i++)
                params->track->pointsEdgeRight.push_back(repair[i]);
        }
        params->ctrl.slow = true;
    }

    if (!avoidApplied && !usingObstacleHold)
    {
        obstacleHoldFrames = 0;
        obstacleHoldSide = 0;
    }

    // Cut the far top part of the track lines so curve points do not dominate the steering weight.
    params->track->pointsEdgeLeft.resize(params->track->pointsEdgeLeft.size() * 0.7);
    params->track->pointsEdgeRight.resize(params->track->pointsEdgeRight.size() * 0.7);
}

void FsmObstacle::resetLap()
{
    resultObs = PredictResult();
    obstacleHoldFrames = 0;
    obstacleHoldSide = 0;
    obstacleHoldResult = PredictResult();
}

void FsmObstacle::show(Mat &img)
{
    if (resultObs.x > 0 && resultObs.y > 0)
    {
        cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }
}

void FsmObstacle::curtailTracking(bool left)
{
    if (left) // Shift toward left side.
    {
        if (params->track->pointsEdgeRight.size() > params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeRight.resize(params->track->pointsEdgeLeft.size());

        for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
        {
            params->track->pointsEdgeRight[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
    else // Shift toward right side.
    {
        if (params->track->pointsEdgeRight.size() < params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeLeft.resize(params->track->pointsEdgeRight.size());

        for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
        {
            params->track->pointsEdgeLeft[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
}
