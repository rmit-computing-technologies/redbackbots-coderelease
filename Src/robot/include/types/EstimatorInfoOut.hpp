#ifndef ESTIMATOR_INFO_OUT_HPP
#define ESTIMATOR_INFO_OUT_HPP

#include "types/RobotObstacle.hpp"

class EstimatorInfoOut
{
  public:
    EstimatorInfoOut(){};

    AbsCoord robotPos;
    float robotPosUncertainty;
    float robotHeadingUncertainty;
    std::vector<AbsCoord> allRobotPos;
    RRCoord ballPosRR;
    AbsCoord ballPosRRC;
    AbsCoord ballVelRRC;
    // ball pos relative to field
    AbsCoord ballPos;
    AbsCoord ballVel;
    AbsCoord teamBallPos;
    AbsCoord teamBallVel;
    float teamBallPosUncertainty;
    int ticksSinceTeamBallUpdate;
    int numOfBallSeenTicks;
    SharedStateEstimationBundle sharedStateEstimationBundle;
    std::vector<RobotObstacle> robotObstacles;
    bool haveTeamBallUpdate;
};

#endif // ESTIMATOR_INFO_OUT_HPP
