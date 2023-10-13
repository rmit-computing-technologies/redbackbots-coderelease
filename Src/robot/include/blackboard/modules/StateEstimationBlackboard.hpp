#pragma once

#include <boost/function.hpp>
#include <boost/program_options/variables_map.hpp>
#include <stdint.h>
#include <vector>

#include "types/AbsCoord.hpp"
#include "types/RRCoord.hpp"
#include "types/RobotObstacle.hpp"
#include "types/SharedStateEstimationBundle.hpp"

/**
 * Data State Estimation module will be sharing
 */
struct StateEstimationBlackboard {
    explicit StateEstimationBlackboard();
    void readOptions(const boost::program_options::variables_map& config);

    // Global robot position and uncertainty
    AbsCoord robotPos;
    float robotPosUncertainty;
    float robotHeadingUncertainty;
    std::vector<AbsCoord> allRobotPos;

    // Robot relative coords.
    RRCoord ballPosRR;

    // Cartesian robot relative ball coords.
    AbsCoord ballPosRRC;

    // Robot relative ball velocity.
    // TODO: make this in global coords and change behaviours correspondingly.
    // Ball Detection currently assumes this is in Robot Relative coordinates
    AbsCoord ballVelRRC;

    // Global ball velocity
    AbsCoord ballVel;

    // Global ball position
    AbsCoord ballPos;

    // Team ball position
    AbsCoord teamBallPos;

    // Team ball velocity
    AbsCoord teamBallVel;

    // Team ball position uncertainty
    float teamBallPosUncertainty;

    SharedStateEstimationBundle sharedStateEstimationBundle;

    bool havePendingOutgoingSharedBundle;
    std::vector<bool> havePendingIncomingSharedBundle;

    /** filtered positions of visual robots */
    std::vector<RobotObstacle> robotObstacles;

    // Whether we have had a recent team ball update
    bool hadTeamBallUpdate;

    float ballAge;
};
