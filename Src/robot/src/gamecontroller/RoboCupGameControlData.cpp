#include "gamecontroller/RoboCupGameControlData.hpp"

const char *gameControllerGamePhaseNames[] = {
   " ",  // Don't care about saying normal
   "shootout ",
   "overtime ",
   "timeout "
   // Note extra space at end so we don't merge words
};

const char *gameControllerStateNames[] = {
   "initial",     // 0
   "standby",     // 5
   "ready",       // 1
   "set",         // 2
   "playing",     // 3
   "finished",    // 4
};

const char *gameControllerPenaltyNames[] = {
   "none",                       // 0
   "illegal ball contact",       // 1
   "player pushing",             // 2
   "motion in set",              // 3
   "inactive player",            // 4
   "illegal position",           // 5
   "leaving the field",          // 6
   "request for pickup",         // 7
   "local game stuck",           // 8
   "illegal position in set",    // 9
   "illegal player stance",      // 10
   "illegal motion in initial ", // 11
   "invalid",                    // 12
   "invalid",                    // 13
   "substitute penalty",         // 14
   "manual penalty",             // 15
};
