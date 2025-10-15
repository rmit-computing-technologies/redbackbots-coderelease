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
    bool active;
    bool seenRefGesture;
    RoboCupGameControlData data;
    TeamInfo our_team;
    TeamInfo opponent_team;
    bool leftTeam; // Are we the left team in terms of GC pov
    int player_number;
    uint8_t gameState;
    char* lastGameControllerIPAddress;
};
