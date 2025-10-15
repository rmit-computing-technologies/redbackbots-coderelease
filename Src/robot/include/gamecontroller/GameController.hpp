/**
 * GameController.hpp
 * Description: A thread to recieve game state information from the Game
 * Controller sever and implements the Button Interface. Adapted from 2009 code.
 */

#pragma once

#include <vector>
#include "gamecontroller/RoboCupGameControlData.hpp"
#include "types/ButtonPresses.hpp"
#include "blackboard/Adapter.hpp"
#include "utils/defs/RobotDefinitions.hpp"
#include <array>

class GameController : Adapter {
    public:
        // Constructor
        GameController(Blackboard *bb);
        // Destructor
        ~GameController();
        // Called on each cycle
        void tick();
    private:
        RoboCupGameControlData data;
        TeamInfo *our_team;
        TeamInfo *opponent_team;
        bool connected;
        bool active;
        int sock;

        /**
         * Connect to the GameController
         */
        void initialiseConnection();

        /**
         * Update the state using the Button Interface
         */
        void buttonUpdate();

        /**
         * Update the state using the GameController Interface
         */
        void wirelessUpdate();

        /**
         * Update the robot's state to do things like leave the WiFi and
         * self-terminate on `redbackbots` start.
         */
        void handleFinishedPacket();

        /* Game configs */
        bool usePassing;
        int numRequiredTouches;
        int previousSetPlay = SET_PLAY_NONE; // Used to check if we have returned to the game without timing out

        /**
         * Parse data from the GameController
         * @param update Pointer to newly recieved GC data
         */
        void parseData(RoboCupGameControlData *update);

        // parseData helper functions

        bool isValidData(RoboCupGameControlData *gameData);

        bool checkHeader(char* header);

        bool isThisGame(RoboCupGameControlData *gameData);

        bool gameDataEqual(void* gameData, void* previous);

        void rawSwapTeams(RoboCupGameControlData *gameData);

        /**
         * Internal state for speech updates
         */
        uint8_t lastState;
        uint16_t myLastPenalty;

        /* Player & team number re-checked from config each cycle */
        int myPlayerNumber;
        int teamNumber;

        /* Touched ball state */
        std::array<bool, ROBOTS_PER_TEAM> touchedBall;

        int ballSeenFrames; // Works like python Globals ball_lost_frames

        /* Structure containing mask of buttons that have been pushed */
        ButtonPresses buttons;

        /**
         * Structure containing the timestamps of when STATE_FINISHED
         * packets were received
         */
        std::vector<time_t> finishedTimes;

        // Call this every time data or teamNumber changes
        void setOurTeam();
};
