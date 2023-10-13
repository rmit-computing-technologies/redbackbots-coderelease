#pragma once

#include <boost/program_options/variables_map.hpp>
#include <stdint.h>

#include "gamecontroller/RoboCupGameControlData.hpp"

/**
 * Shared data from the Game Controller 2018.
 */
struct GameControllerBlackboard {
    explicit GameControllerBlackboard();
    void readOptions(const boost::program_options::variables_map& config);
    
    bool connect;
    bool connected;
    RoboCupGameControlData data;
    TeamInfo our_team;
    int player_number;
    uint8_t gameState;
    char* lastGameControllerIPAddress;
    bool whistleDetected;
};
