/**
 * @file PreviousBalls.hpp
 * @author RedBackBots
 */

#pragma once

#include "types/math/Eigen.hpp"

/**
 * Represents spots that might be an indication of a ball.
 */
class PreviousBalls {
public:
    unsigned int timestamp;
    std::vector<Vector2i> balls;

    inline void reset() {
        timestamp = 0;
        balls.clear();
    }
};