#pragma once

#include "Candidates.hpp"
#include <vector>

/**
 * @struct PlaneSpots
 * This struct contains all the PlaneSpots
 */
class PlaneSpots {
    public:
        std::vector<const Spot*> spots;
};