#include "perception/stateestimation/robotfilter/types/RobotObservation.hpp"

#include <Eigen/Eigen>
#include "types/AbsCoord.hpp"
#include "utils/angles.hpp"
#include "perception/vision/VisionDefinitions.hpp"

using namespace Eigen;

const int RobotObservation::ESTIMATED_FRAMES_PER_SECOND = 20;


const int RobotObservation::MIN_WEIGHT = 1;
const int RobotObservation::WEIGHT_RANGE = 20;

const double RobotObservation::SECONDS_ALIVE_ON_FOV = 1;
const double RobotObservation::SECONDS_ALIVE_OFF_FOV = 10;

const int RobotObservation::MAX_LIFE_SCORE =
        RobotObservation::SECONDS_ALIVE_OFF_FOV * ESTIMATED_FRAMES_PER_SECOND;

static const double SECONDS_ALIVE_RATIO =
        RobotObservation::SECONDS_ALIVE_OFF_FOV / RobotObservation::SECONDS_ALIVE_ON_FOV;

const int RobotObservation::OFF_SCREEN_SCORE_REDUCER = 1;
const int RobotObservation::ON_SCREEN_SCORE_REDUCER = SECONDS_ALIVE_RATIO * OFF_SCREEN_SCORE_REDUCER;
const int RobotObservation::DISTANCE_SCORE_REDUCER = 4; //Max life score reduced per tick for being far away.


RobotObservation::RobotObservation(const RobotVisionInfo &visualRobot) :
    lifeScore(MAX_LIFE_SCORE) {

    rr = visualRobot.rr;
    type = visualRobot.type;
}


void RobotObservation::tick(Odometry odometryDiff, float headYaw, double distanceObservationToGroupPercentageOfMax) {
    Point cartesianCoords = rr.toCartesian();
    cartesianCoords[0] -= odometryDiff.forward;
    cartesianCoords[1] -= odometryDiff.left;

    rr = AbsCoord(cartesianCoords[0], cartesianCoords[1], 0).convertToRobotRelative();
    rr.heading() -= odometryDiff.turn;

    lifeScore -= (distanceObservationToGroupPercentageOfMax * DISTANCE_SCORE_REDUCER);
    bool onScreen = inView(headYaw);
    if (onScreen) {
        lifeScore -= ON_SCREEN_SCORE_REDUCER;
    } else {
        lifeScore -= OFF_SCREEN_SCORE_REDUCER;
    }
}

/**`
 * Calculates the weight for this given observation. It weights a recently seen
 * observations significantly more then old so it is using quadratic functions.
 */
int RobotObservation::getWeight() const {
    double lifeScoreSquared = lifeScore * lifeScore;
    double maxScoreSquared = MAX_LIFE_SCORE * MAX_LIFE_SCORE;

    return (lifeScoreSquared / maxScoreSquared) * WEIGHT_RANGE + MIN_WEIGHT;
}

bool RobotObservation::inView(float headYaw) const {
    float visibleLo = headYaw - 0.5f * IMAGE_HFOV;
    float visibleHi = headYaw + 0.5f * IMAGE_HFOV;

    return (visibleLo <= rr.heading() && rr.heading() <= visibleHi);
}

int RobotObservation::getMinWeight() {
    return MIN_WEIGHT;
}

int RobotObservation::getMaxWeight() {
    return MIN_WEIGHT + WEIGHT_RANGE;
}

unsigned int RobotObservation::numberOnScreenFramesToBeStale() {
    return ESTIMATED_FRAMES_PER_SECOND * SECONDS_ALIVE_ON_FOV;
}

unsigned int RobotObservation::numberOffScreenFramesToBeStale() {
    return ESTIMATED_FRAMES_PER_SECOND * SECONDS_ALIVE_OFF_FOV;
}

bool RobotObservation::isStale() const {
    return lifeScore <= 0;
}

Point RobotObservation::getCartesianCoordinates() const {
    return getRRCoordinates().toCartesian();
}

RRCoord RobotObservation::getRRCoordinates() const {
    return rr;
}
