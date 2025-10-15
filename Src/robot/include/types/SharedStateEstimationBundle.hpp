#pragma once

#include "types/geometry/AbsCoord.hpp"

class SharedStateEstimationBundle
{
  public:
    SharedStateEstimationBundle() : haveBallUpdate(false), haveTeamBallUpdate(false) {}

    SharedStateEstimationBundle(AbsCoord robotPos, AbsCoord ballPosRRC, AbsCoord ballVelRRC, bool haveBalUpdate, bool haveTeamBallUpdate):
    robotPos(robotPos),
    ballPosRRC(ballPosRRC),
    ballVelRRC(ballVelRRC),
    haveBallUpdate(haveBallUpdate),
    haveTeamBallUpdate(haveTeamBallUpdate)
    {
    }

    bool sanityCheck();

    AbsCoord robotPos;
    AbsCoord ballPosRRC;
    AbsCoord ballVelRRC;
    bool haveBallUpdate;
    bool haveTeamBallUpdate;
};
