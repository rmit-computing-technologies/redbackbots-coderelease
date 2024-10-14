#include "perception/stateestimation/StateEstimationAdapter.hpp"

#include <boost/math/constants/constants.hpp>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/BehaviourBlackboard.hpp"
#include "blackboard/modules/GameControllerBlackboard.hpp"
#include "blackboard/modules/MotionBlackboard.hpp"
#include "blackboard/modules/ReceiverBlackboard.hpp"
#include "blackboard/modules/StateEstimationBlackboard.hpp"
#include "blackboard/modules/SynchronisationBlackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"

#include "perception/stateestimation/localiser/Localiser.hpp"
#include "perception/stateestimation/egoballtracker/EgoBallTracker.hpp"
#include "perception/stateestimation/teamballtracker/TeamBallTracker.hpp"
#include "perception/stateestimation/robotfilter/RobotFilter.hpp"

#include "types/EstimatorInfoInit.hpp"
#include "types/EstimatorInfoIn.hpp"
#include "types/EstimatorInfoMiddle.hpp"
#include "types/EstimatorInfoOut.hpp"

#include "utils/incapacitated.hpp"
#include "utils/Logger.hpp"

StateEstimationAdapter::StateEstimationAdapter(Blackboard *bb)
    : Adapter(bb),
      estimatorInfoInit(NULL),
      estimatorInfoIn(NULL),
      estimatorInfoMiddle(NULL),
      estimatorInfoOut(NULL),
      prevOdometry(NULL),
      prevTimeStampInMicroSeconds(-1)
{
    estimatorInfoInit = new EstimatorInfoInit(
        readFrom(gameController, player_number),
        readFrom(gameController, our_team).teamNumber,
        (bb->config)["stateestimation.initial_pose_type"].as<std::string>(),
        AbsCoord((bb->config)["stateestimation.specified_initial_x"].as<int>(),
            (bb->config)["stateestimation.specified_initial_y"].as<int>(),
            (bb->config)["stateestimation.specified_initial_theta"].as<int>() * boost::math::float_constants::pi / 180.0f),
        readFrom(behaviour, skill),
        readFrom(gameController, data).competitionType,
        readFrom(gameController, data).state,
        readFrom(gameController, data).gamePhase,
        readFrom(gameController, data).setPlay,
        (bb->config)["stateestimation.handle_referee_mistakes"].as<bool>());

    initEstimators();
    ticksSinceLastTeamBallUpdate = 0;
    numOfBallSeenTicks = 0;
    lastTeamBallPos = AbsCoord();
}

StateEstimationAdapter::~StateEstimationAdapter()
{
    delete estimatorInfoInit;
    delete prevOdometry;
    for (unsigned i = 0; i < ESTIMATOR_TOTAL; ++i)
    {
        delete estimators[i];
    }
}

void StateEstimationAdapter::initEstimators()
{
    addEstimator(LOCALISER, new Localiser(*estimatorInfoInit));
    addEstimator(EGOBALLTRACKER, new EgoBallTracker(*estimatorInfoInit));
    addEstimator(TEAMBALLTRACKER, new TeamBallTracker(*estimatorInfoInit));
    addEstimator(ROBOTFILTER, new RobotFilter(*estimatorInfoInit));
}

void StateEstimationAdapter::addEstimator(
    unsigned index,
    Estimator *estimator)
{
    estimators[index] = estimator;
}

void StateEstimationAdapter::runEstimators()
{
    runEstimator(LOCALISER);
    runEstimator(EGOBALLTRACKER);
    runEstimator(TEAMBALLTRACKER);
    runEstimator(ROBOTFILTER);
}

void StateEstimationAdapter::runEstimator(unsigned index)
{
    getEstimator(index)->tick(
        *estimatorInfoIn,
        *estimatorInfoMiddle,
        *estimatorInfoOut);
}

Estimator *StateEstimationAdapter::getEstimator(unsigned index)
{
    return estimators[index];
}

void StateEstimationAdapter::tick()
{
    teamBallStillUpdating = readFrom(stateEstimation, haveTeamBallUpdate);

    createEstimatorInfoIn();
    estimatorInfoMiddle = new EstimatorInfoMiddle();
    estimatorInfoOut = new EstimatorInfoOut();
    
    handleIncomingTeamBallUpdate();
    
    runEstimators();

    handleOutgoingTeamBallUpdate();
    
    writeToBlackboard();

    // Delete objects
    delete estimatorInfoIn;
    delete estimatorInfoMiddle;
    delete estimatorInfoOut;
}

void StateEstimationAdapter::createEstimatorInfoIn()
{
    // Update odometryDiff and dtInSeconds
    Odometry odometryDiff = calculateOdometryDiff(readFrom(motion, odometry));
    float dtInSeconds = calculateDtInSeconds(readFrom(vision, timestamp));

    // Read Incoming Broadcast Data
    const std::vector<bool> &havePendingIncomingSharedBundle =
        readFrom(stateEstimation, havePendingIncomingSharedBundle);
    std::vector<BroadcastData> incomingBroadcastData;
    for (unsigned i = 0; i < havePendingIncomingSharedBundle.size(); ++i)
    {
        if (havePendingIncomingSharedBundle[i])
        {
            incomingBroadcastData.push_back(readFrom(receiver, data)[i]);
        }
    }

    estimatorInfoIn = new EstimatorInfoIn(
        readFrom(vision, fieldFeatures),
        readFrom(vision, balls),
        readFrom(gameController, data).competitionType,
        readFrom(gameController, data).state,
        readFrom(gameController, data).gamePhase,
        readFrom(gameController, data).setPlay,
        readFrom(gameController, data).kickingTeam,
        readFrom(behaviour, behaviourSharedData),
        readFrom(gameController, our_team).players[estimatorInfoInit->playerNumber - 1].penalty,
        readFrom(motion, active),
        readFrom(receiver, incapacitated),
        readFrom(stateEstimation, havePendingOutgoingSharedBundle),
        havePendingIncomingSharedBundle,
        incomingBroadcastData,
        readFrom(vision, robots),
        readFrom(motion, sensors).joints.angles[Joints::HeadYaw],
        isIncapacitated(readFrom(motion, active).body.actionType),
        odometryDiff,
        dtInSeconds,
        readFrom(motion, active).body,
        readFrom(motion, sensors));
}

void StateEstimationAdapter::handleIncomingTeamBallUpdate()
{
    // Update team ball if received new packet and team ball is different.
    for (size_t i = 0; i < estimatorInfoIn->incomingBroadcastData.size(); ++i) {
        if (estimatorInfoIn->incomingBroadcastData[i].sharedStateEstimationBundle.haveTeamBallUpdate) {
            lastTeamBallPos = estimatorInfoIn->incomingBroadcastData[i].ballPosAbs;
            ticksSinceLastTeamBallUpdate = 0;
        }
    }

    // Update regardless if new update or not. Ticks have been updated so push it through
    estimatorInfoOut->ticksSinceTeamBallUpdate = ticksSinceLastTeamBallUpdate;
    estimatorInfoOut->teamBallPos = lastTeamBallPos;
    estimatorInfoOut->numOfBallSeenTicks = numOfBallSeenTicks;
}

void StateEstimationAdapter::handleOutgoingTeamBallUpdate()
{
    numOfBallSeenTicks = estimatorInfoOut->numOfBallSeenTicks;
    if (teamBallStillUpdating || estimatorInfoOut->sharedStateEstimationBundle.haveTeamBallUpdate) 
    {
        ticksSinceLastTeamBallUpdate = 0;
        lastTeamBallPos = estimatorInfoOut->ballPos;
    } 
    else {
        ticksSinceLastTeamBallUpdate++;
    }
}

void StateEstimationAdapter::writeToBlackboard()
{   
    acquireLock(serialization);
    writeTo(stateEstimation, robotPos, estimatorInfoOut->robotPos);
    writeTo(stateEstimation, robotPosUncertainty, estimatorInfoOut->robotPosUncertainty);
    writeTo(stateEstimation, robotHeadingUncertainty, estimatorInfoOut->robotHeadingUncertainty);
    writeTo(stateEstimation, allRobotPos, estimatorInfoOut->allRobotPos);
    writeTo(stateEstimation, ballPos, estimatorInfoOut->ballPos);
    writeTo(stateEstimation, ballPosRR, estimatorInfoOut->ballPosRR);
    writeTo(stateEstimation, ballPosRRC, estimatorInfoOut->ballPosRRC);
    writeTo(stateEstimation, ballVelRRC, estimatorInfoOut->ballVelRRC);
    writeTo(stateEstimation, ballVel, estimatorInfoOut->ballVel);
    writeTo(stateEstimation, teamBallPos, estimatorInfoOut->teamBallPos);
    writeTo(stateEstimation, teamBallVel, estimatorInfoOut->teamBallVel);
    writeTo(stateEstimation, teamBallPosUncertainty, estimatorInfoOut->teamBallPosUncertainty);
    writeTo(stateEstimation, lastTeamBallUpdate, estimatorInfoOut->ticksSinceTeamBallUpdate);
    writeTo(stateEstimation, havePendingOutgoingSharedBundle, true);
    writeTo(stateEstimation, havePendingIncomingSharedBundle, std::vector<bool>(5, false));
    writeTo(stateEstimation, robotObstacles, estimatorInfoOut->robotObstacles);
    if (!teamBallStillUpdating) {
        writeTo(stateEstimation, haveTeamBallUpdate, estimatorInfoOut->sharedStateEstimationBundle.haveTeamBallUpdate);
        writeTo(stateEstimation, sharedStateEstimationBundle, estimatorInfoOut->sharedStateEstimationBundle);
    }
    releaseLock(serialization);
}

Odometry StateEstimationAdapter::calculateOdometryDiff(Odometry newOdometry)
{
    Odometry odometryDiff;

    // Check if not first time
    if (prevOdometry)
    {
        odometryDiff = newOdometry - *prevOdometry;
        delete prevOdometry;
    }

    // Copy odometry
    prevOdometry = new Odometry(newOdometry);

    return odometryDiff;
}

float StateEstimationAdapter::calculateDtInSeconds(int64_t newTimeStampInMicroSeconds)
{
    float dtInSeconds = 0;

    // Check if not first time
    if (prevTimeStampInMicroSeconds > 0)
    {
        // timestamp is in micro-seconds, hence the 1e6.
        dtInSeconds = (newTimeStampInMicroSeconds - prevTimeStampInMicroSeconds) / 1000000.f;
    }

    // Copy timestamp
    prevTimeStampInMicroSeconds = newTimeStampInMicroSeconds;

    return dtInSeconds;
}
