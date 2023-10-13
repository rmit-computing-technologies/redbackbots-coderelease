#pragma once

#include "types/AbsCoord.hpp"

class SharedStateEstimationBundle
{
  public:
    SharedStateEstimationBundle() : haveBallUpdate(false) {}

    SharedStateEstimationBundle(AbsCoord robotPos, AbsCoord ballPosRRC, AbsCoord ballVelRRC, bool haveBalUpdate):
    robotPos(robotPos),
    ballPosRRC(ballPosRRC),
    ballVelRRC(ballVelRRC),
    haveBallUpdate(haveBallUpdate)
    {
    }

    bool sanityCheck();

    AbsCoord robotPos;
    AbsCoord ballPosRRC;
    AbsCoord ballVelRRC;
    bool haveBallUpdate;
};
