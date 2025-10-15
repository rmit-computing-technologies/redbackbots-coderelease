/**
 * @file BallSpots.h
 * Declaration of a struct that represents spots that might be an indication of a ball.
 * @author <a href="mailto:jworch@informatik.uni-bremen.de">Jan-Hendrik Worch</a>
 * @author <a href="mailto:ingsie@informatik.uni-bremen.de">Ingo Sieverdingbeck</a>
 * @author RedBackBots
 */

#pragma once

#include "types/geometry/Point.hpp"

/**
 * Represents spots that might be an indication of a ball.
 */
class BallSpots {
public:
    BallSpots() {
        ballSpots.reserve(50);
    }

    void addBallSpot(int x, int y) {
        ballSpots.emplace_back(x, y);
    }

    // Ball Spots as PointF
    std::vector<Point> ballSpots;

    // true if the first ball spot is derived from the ball model
    bool firstSpotIsPredicted; 
};
