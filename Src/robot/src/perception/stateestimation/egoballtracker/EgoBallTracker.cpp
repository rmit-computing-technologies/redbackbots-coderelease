#include "perception/stateestimation/egoballtracker/EgoBallTracker.hpp"
#include "perception/stateestimation/egoballtracker/ballcmkf/BallCMKF.hpp"

EgoBallTracker::EgoBallTracker(const EstimatorInfoInit &estimatorInfoInit)
    : Estimator(estimatorInfoInit)
{
    ballcmkf = new BallCMKF(estimatorInfoInit);
}

EgoBallTracker::~EgoBallTracker()
{
    if (ballcmkf) delete ballcmkf;
}

void EgoBallTracker::tick(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle,
    EstimatorInfoOut &estimatorInfoOut)
{
    ballcmkf->tick(estimatorInfoIn, estimatorInfoMiddle, estimatorInfoOut);
}