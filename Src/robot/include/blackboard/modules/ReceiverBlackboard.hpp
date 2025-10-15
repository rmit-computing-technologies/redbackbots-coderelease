#pragma once

#include <boost/program_options/variables_map.hpp>
#include <time.h>
#include <vector>

#include "types/BroadcastData.hpp"
#include "types/SPLStandardMessage.hpp"
#include "utils/defs/RobotDefinitions.hpp"

/**
 * Shared data from the team data receiver module.
 */
struct ReceiverBlackboard {
    explicit ReceiverBlackboard();
    void readOptions(const boost::program_options::variables_map& config);

    // one for each robot on the team
    SPLStandardMessage message[ROBOTS_PER_TEAM];
    BroadcastData data[ROBOTS_PER_TEAM];
    int team;
    time_t lastReceived[ROBOTS_PER_TEAM];
    std::vector<bool> incapacitated;
};