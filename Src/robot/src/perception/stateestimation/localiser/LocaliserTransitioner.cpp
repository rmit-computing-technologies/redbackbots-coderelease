#include "perception/stateestimation/localiser/LocaliserTransitioner.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"
#include "types/EstimatorInfoInit.hpp"
#include "types/EstimatorInfoIn.hpp"

#define REF_PICKUP_TIMER_MIN_MSEC 1000
#define PENALISED_TIMER_MIN_MSEC 20000
#define REF_FORGOT_TO_TURN_ROBOT_AROUND_MSEC 15000
#define PENALTY_LOCALISE_DELAY_MSEC 2000

LocaliserTransitioner::LocaliserTransitioner(const EstimatorInfoInit &estimatorInfoInit)
    : estimatorInfoInit(estimatorInfoInit)
    , prevCompetitionType(estimatorInfoInit.competitionType)
    , prevGameState(estimatorInfoInit.state)
    , prevGamePhase(estimatorInfoInit.gamePhase)
    , prevPenalty(PENALTY_NONE)
    , prevPickedup(false)
    , refPickupTimer()
    , penaltyPlaceDownTimer()
    , localisedInPenalty(true)
    , isLeftTeam(true)
{
}

void LocaliserTransitioner::handleTransition(const EstimatorInfoIn &estimatorInfoIn)
{
    uint8_t newCompetitionType = estimatorInfoIn.competitionType;
    uint8_t newGamePhase = estimatorInfoIn.gamePhase;
    uint8_t newGameState = estimatorInfoIn.state;
    uint8_t newPenalty = estimatorInfoIn.penalty;
    bool newPickedup = (estimatorInfoIn.active.body.actionType == ActionCommand::Body::REF_PICKUP); // NOTE: REF_PICKUP just means off-the-ground in general.
    bool weAreKickingTeam = estimatorInfoIn.kickingTeam == estimatorInfoInit.teamNumber;
    bool newIsLeftTeam = estimatorInfoIn.leftTeam;

    // Competition Type Changes
    if (prevCompetitionType != newCompetitionType)
    {
        resetToInitialPose();
    }

    // we are right team. this if condition should only be true once during the start of the game.
    if (newIsLeftTeam != isLeftTeam)
    {
        isLeftTeam = newIsLeftTeam;
        resetToInitialPose();
    }

    if (newCompetitionType == COMPETITION_TYPE_NORMAL)
    {
        if (newGamePhase == GAME_PHASE_PENALTYSHOOT)
        {
            // Reset when we go to PLAYING
            if (prevGameState != newGameState && newGameState == STATE_PLAYING)
            {
                if (estimatorInfoIn.kickingTeam == estimatorInfoInit.teamNumber)
                    resetToPenaltyshootPhasePoseOffense();
                else
                    resetToPenaltyshootPhasePoseDefense();
            }

            // Reset positions when we get penalised / unpenalised so we can see in TCM who the specified taker is
            if (prevPenalty != newPenalty)
            {
                if (newPenalty == PENALTY_NONE)
                    resetToPenaltyshootPhasePlayerSelectedPose();
                else
                    resetToInitialPose();
            }
        }
        else
        {
            // Robot Gets Picked Up
            if (prevPickedup == false && newPickedup == true)
            {
                // Restart refPickupTimer
                refPickupTimer.restart();
            }

            // Robot Gets Placed Down
            if (prevPickedup == true && newPickedup == false)
            {
                // If we think we've been picked up for long enough,
                if (refPickupTimer.elapsed_ms() > REF_PICKUP_TIMER_MIN_MSEC)
                {
                    if (newGameState == STATE_SET && newPenalty == PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
                    {
                        // If in STATE_SET, reset to manual placement
                        // TODO: these needs updating to our new kick-off positions
                        if (weAreKickingTeam)
                            resetToManualPlacementPoseOffense();
                        else
                            resetToManualPlacementPoseDefense();
                    }
                    else if (newGameState == STATE_INITIAL || newGameState == STATE_STANDBY || newPenalty == PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
                    {
                        resetToInitialPose();
                    }
                    // if placed down while penalised
                    else if (newPenalty != PENALTY_NONE)
                    {
                        // start timer to delay localisation reset a few seconds after place down
                        // to prevent mislocalisation when the assistant ref is still in front of the robot (blocks field lines from cameras)
                        penaltyPlaceDownTimer.restart();
                        localisedInPenalty = false;
                    }
                    // if state is playing and we dont have any penalties, we're probably a substitute player.
                    else if (newGameState == STATE_PLAYING)
                    {
                        // substitute players are placed down in penalised pose
                        resetToPenalisedPose();
                    }
                    // default fall back
                    else
                    {
                        resetToInitialPose();
                    }
                }
            }

            if (localisedInPenalty == false && penaltyPlaceDownTimer.elapsed_ms() > PENALTY_LOCALISE_DELAY_MSEC)
            {
                localisedInPenalty = true;
                resetToPenalisedPose();
            }
        }
    }

    // Game State transition to STATE_INITIAL AND STATE_STANDBY
    if (prevGameState != newGameState &&
        newGameState == STATE_INITIAL)
    {
        resetToInitialPose();
    }

    // Update prev variables
    prevCompetitionType = newCompetitionType;
    prevGameState = newGameState;
    prevGamePhase = newGamePhase;
    prevPenalty = newPenalty;
    prevPickedup = newPickedup;
}

void LocaliserTransitioner::resetToInitialPose()
{
    if (estimatorInfoInit.initialPoseType == "GAME")
    {
        resetToGameInitialPose();
    }
    else if (estimatorInfoInit.initialPoseType == "SPECIFIED")
    {
        resetToSpecifiedInitialPose();
    }
    else if (estimatorInfoInit.initialPoseType == "UNPENALISED")
    {
        resetToUnpenalisedPose();
    }
    else
    {
        std::cout << "(LocaliserTransitioner) initialPoseType: '"
                  << estimatorInfoInit.initialPoseType
                  << "' not recognised. Defaulting to GAME initial pose"
                  << std::endl;

        resetToGameInitialPose();
    }
}

void LocaliserTransitioner::resetToGameInitialPose()
{
    if (this->isLeftTeam)
    {
        resetToLeftTeamInitialPose();
    }
    else
    {
        resetToRightTeamInitialPose();
    }
}
