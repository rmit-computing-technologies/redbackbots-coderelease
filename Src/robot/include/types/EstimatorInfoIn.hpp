#pragma once

#include "gamecontroller/RoboCupGameControlData.hpp"
#include "types/BehaviourSharedData.hpp"
#include "types/SensorValues.hpp"
#include "types/Odometry.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "types/BallInfo.hpp"
#include "types/ActionCommand.hpp"
#include "types/BroadcastData.hpp"
#include "types/RobotVisionInfo.hpp"
#include "types/ActionCommand.hpp"

class EstimatorInfoIn
{
  public:
    EstimatorInfoIn(
        const std::vector<FieldFeatureInfo> &fieldFeatures,
        const std::vector<BallInfo> &balls,
        const uint8_t competitionType,
        const uint8_t state,
        const uint8_t gamePhase,
        const uint8_t setPlay,
        const uint8_t kickingTeam,
        const BehaviourSharedData &behaviourSharedData,
        const uint8_t penalty,
        const ActionCommand::All &active,
        const std::vector<bool> &incapacitated,
        const bool havePendingOutgoingSharedBundle,
        const std::vector<bool> &havePendingIncomingSharedBundle,
        const std::vector<BroadcastData> &incomingBroadcastData,
        const std::vector<RobotVisionInfo> &visualRobots,
        const float headYaw,
        const bool isIncapacitated,
        const Odometry &odometryDiff,
        const float dtInSeconds,
        const ActionCommand::Body actionCommandBody,
        const SensorValues &sensorValues)
        : fieldFeatures(fieldFeatures)
        , balls(balls)
        , competitionType(competitionType)
        , state(state)
        , gamePhase(gamePhase)
        , setPlay(setPlay)
        , kickingTeam(kickingTeam)
        , behaviourSharedData(behaviourSharedData)
        , penalty(penalty)
        , active(active)
        , incapacitated(incapacitated)
        , havePendingOutgoingSharedBundle(havePendingOutgoingSharedBundle)
        , havePendingIncomingSharedBundle(havePendingIncomingSharedBundle)
        , incomingBroadcastData(incomingBroadcastData)
        , visualRobots(visualRobots)
        , headYaw(headYaw)
        , isIncapacitated(isIncapacitated)
        , odometryDiff(odometryDiff)
        , dtInSeconds(dtInSeconds)
        , actionCommandBody(actionCommandBody)
        , sensorValues(sensorValues)
    {};

    EstimatorInfoIn(){};

    std::vector<FieldFeatureInfo> fieldFeatures;
    std::vector<BallInfo> balls;
    uint8_t competitionType;
    uint8_t state;
    uint8_t gamePhase;
    uint8_t setPlay;
    uint8_t kickingTeam;
    BehaviourSharedData behaviourSharedData;
    uint8_t penalty;
    ActionCommand::All active;
    std::vector<bool> incapacitated;
    bool havePendingOutgoingSharedBundle;
    std::vector<bool> havePendingIncomingSharedBundle;
    std::vector<BroadcastData> incomingBroadcastData;
    std::vector<RobotVisionInfo> visualRobots;
    float headYaw;
    bool isIncapacitated;
    Odometry odometryDiff;
    float dtInSeconds;
    ActionCommand::Body actionCommandBody;
    SensorValues sensorValues;

};

