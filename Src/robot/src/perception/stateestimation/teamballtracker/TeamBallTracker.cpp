#include "perception/stateestimation/teamballtracker/TeamBallTracker.hpp"
#include "perception/stateestimation/teamballtracker/teamballkf/TeamBallKF.hpp"

TeamBallTracker::TeamBallTracker(const EstimatorInfoInit &estimatorInfoInit)
    : Estimator(estimatorInfoInit)
{
    teamBallKF = new TeamBallKF(estimatorInfoInit);
}

TeamBallTracker::~TeamBallTracker()
{
    if (teamBallKF) delete teamBallKF;
}

void TeamBallTracker::tick(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle,
    EstimatorInfoOut &estimatorInfoOut)
{
    teamBallKF->tick(estimatorInfoIn, estimatorInfoMiddle, estimatorInfoOut);
}