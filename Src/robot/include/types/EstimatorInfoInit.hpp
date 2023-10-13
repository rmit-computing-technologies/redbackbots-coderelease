#pragma once

#include "types/AbsCoord.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"

class EstimatorInfoInit
{
  public:
    EstimatorInfoInit(
        const int &playerNumber,
        const int &teamNumber,
        const std::string &initialPoseType,
        const AbsCoord &specifiedInitialPose,
        const std::string &skill,
        const uint8_t &competitionType,
        const uint8_t &state,
        const uint8_t &gamePhase,
        const uint8_t &setPlay,
        const bool &handleRefereeMistakes)
        : playerNumber(playerNumber)
        , teamNumber(teamNumber)
        , initialPoseType(initialPoseType)
        , specifiedInitialPose(specifiedInitialPose)
        , skill(skill)
        , competitionType(competitionType)
        , state(state)
        , gamePhase(gamePhase)
        , setPlay(setPlay)
        , handleRefereeMistakes(handleRefereeMistakes)
        {};

    EstimatorInfoInit(){};

    int playerNumber;
    int teamNumber;
    std::string initialPoseType;
    AbsCoord specifiedInitialPose;
    std::string skill;
    uint8_t competitionType;
    uint8_t state;
    uint8_t gamePhase;
    uint8_t setPlay;
    bool handleRefereeMistakes;
};
