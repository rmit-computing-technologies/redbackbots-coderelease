#ifndef LOCALISER_TRANSITIONER_HPP
#define LOCALISER_TRANSITIONER_HPP

#include <stdint.h>
#include "utils/Timer.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;

class LocaliserTransitioner
{
  public:
    LocaliserTransitioner(const EstimatorInfoInit &estimatorInfoInit);
    void handleTransition(const EstimatorInfoIn &estimatorInfoIn);
    virtual ~LocaliserTransitioner(){};

  protected:
    const EstimatorInfoInit &estimatorInfoInit;
    void resetToInitialPose();
    void resetToGameInitialPose();

  private:

    uint8_t prevCompetitionType;
    uint8_t prevGameState;
    uint8_t prevGamePhase;
    uint8_t prevPenalty;
    bool prevPickedup;

    Timer refPickupTimer;
    Timer penaltyPlaceDownTimer;
    bool localisedInPenalty;

    bool isLeftTeam;

    virtual void resetToLeftTeamInitialPose() = 0;
    virtual void resetToRightTeamInitialPose() = 0;
    virtual void resetToSpecifiedInitialPose() = 0;
    virtual void resetToUnpenalisedPose() = 0;
    virtual void resetToPenalisedPose() = 0;
    virtual void resetToManualPlacementPoseOffense() = 0;
    virtual void resetToManualPlacementPoseDefense() = 0;
    virtual void resetToPenaltyshootPhasePoseOffense() = 0;
    virtual void resetToPenaltyshootPhasePoseDefense() = 0;
    virtual void resetToPenaltyshootPhasePlayerSelectedPose() = 0;
};

#endif // LOCALISER_TRANSITIONER_HPP
