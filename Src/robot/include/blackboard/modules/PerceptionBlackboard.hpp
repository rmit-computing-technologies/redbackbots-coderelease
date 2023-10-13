#pragma once

#include <stdint.h>

/**
 * Shared data from the perception module.
 */
struct PerceptionBlackboard {
    explicit PerceptionBlackboard();
    
    uint32_t kinematics;
    uint32_t stateEstimation;
    uint32_t vision;
    uint32_t behaviour;
    uint32_t total;
};
