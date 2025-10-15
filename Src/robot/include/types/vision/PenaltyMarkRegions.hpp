/**
 * Defines penalty mark regions.
 * @author RedbackBots
 */

#pragma once

#include "types/math/Boundary.hpp"

struct PenaltyMarkRegions
{
    PenaltyMarkRegions() = default;

    std::vector<Boundaryi> regions;
};