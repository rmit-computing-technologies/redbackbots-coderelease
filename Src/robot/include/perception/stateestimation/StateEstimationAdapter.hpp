#pragma once

#include <chrono>

#include "blackboard/Adapter.hpp"
#include "types/geometry/AbsCoord.hpp"

#include <cstdint>

enum EstimatorsEnum
{
    LOCALISER = 0,
    EGOBALLTRACKER,
    TEAMBALLTRACKER,
    ROBOTFILTER,
    ESTIMATOR_TOTAL
};

class Estimator;
class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;
struct Odometry;

class StateEstimationAdapter : Adapter
{
  public:
    explicit StateEstimationAdapter(Blackboard *bb);
    ~StateEstimationAdapter();

    void tick();

  private:

    void initEstimators();
    void addEstimator(unsigned index, Estimator *estimator);
    void runEstimators();
    void runEstimator(unsigned index);
    Estimator *getEstimator(unsigned index);

    void createEstimatorInfoIn();
    void writeToBlackboard();
    void handleIncomingTeamBallUpdate();
    void handleOutgoingTeamBallUpdate();


    Estimator *estimators[ESTIMATOR_TOTAL];

    EstimatorInfoInit *estimatorInfoInit;
    EstimatorInfoIn *estimatorInfoIn;
    EstimatorInfoMiddle *estimatorInfoMiddle;
    EstimatorInfoOut *estimatorInfoOut;

    /* Variables used to calculate difference between frames */
    Odometry *prevOdometry;
    int64_t prevTimeStampInMicroSeconds;

    /* Functions used to calculate difference between frames */
    Odometry calculateOdometryDiff(Odometry newOdometry);
    float calculateDtInSeconds(int64_t newTimeStampInMicroSeconds);
    int numOfBallSeenTicks;
    AbsCoord lastTeamBallPos;
    bool teamBallStillUpdating;
    std::chrono::system_clock::time_point prevTeamBallUpdateTime;
};
