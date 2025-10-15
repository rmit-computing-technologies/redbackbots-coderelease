/**
 * GameController.hpp
 * Description: A thread to recieve game state information from the Game
 * Controller sever and implements the Button Interface. Adapted from 2009 code.
 */

#include "gamecontroller/GameController.hpp"

#include <arpa/inet.h>
#include <ctime>
#include <dirent.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <fstream>
#include <chrono>
#include <stdexcept>
#include <vector>

#include "utils/Logger.hpp"
#include "utils/speech.hpp"
#include "utils/defs/RobotDefinitions.hpp"
#include "motion/LoLAData.hpp"

/* Blackboards */
#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/GameControllerBlackboard.hpp"
#include "blackboard/modules/MotionBlackboard.hpp"
#include "blackboard/modules/ReceiverBlackboard.hpp"
#include "blackboard/modules/StateEstimationBlackboard.hpp"
#include "blackboard/modules/EventTransmitterBlackboard.hpp"
#include "blackboard/modules/EventReceiverBlackboard.hpp"
#include "blackboard/modules/WhistleBlackboard.hpp"

#define POLL_TIMEOUT 200

using namespace std;

/**
 * Average number of broadcast GameController packets
 * expected to be received per second.
 */
const uint8_t PACKETS_PER_SECOND = 2;

/**
 * Minimum number of seconds to wait before leaving WiFi.
 */
const uint8_t LEAVE_WIFI_SECONDS = 20;

/**
 * To handle spotty WiFi scenarios, allow up to this much network jitter,
 * specifically allow this proportion of packets dropped before leaving the
 * WiFi, as the second required guard to leave the WiFi and kill `redbackbots`.
 *
 * If set too high, robots will wait longer before actually leaving the WiFi,
 * if set too low, robots will depend more only on the time that the
 * first and last packets were received.
 *
 * Should be set to a value between 0.0 and 1.0
 */
const double JITTER_BUFFER = 0.5;

// Used for goal scoring - stay in ready for STAY_IN_READY_SECS
Timer goalTimer = Timer();
const float STAY_IN_READY_SECS = 16.0; // GC delay is 15 seconds (2025 Rules)
const float MIN_STAY_IN_READY = 5.0; // If this amount of time has passed then can exit ready early
bool goalWhistleHeard = false;

// Used for passing infastructure
const int NUMBER_OF_TOUCHES_REQUIRED = 2; // The number of touches required before we are allowed to score a goal
const float TOUCHED_BALL_TIME_TO_SEND = 5.0f; // The maximum time to wait before sending the event
const float REF_DETECT_TIME_TO_RECEIVE = 60.0f; // The maximum time to wait before discarding the event
const double ROBOT_HEADING_THRESHOLD = 0.75; // In radians - The angle threshold for the robot to be considered facing the ball
const int RESEND_POSITION_MOVEMENT_RANGE = 300;

// Defining the region that we deem the ball close enough to be "touched" by us
const int MINIMUM_DISTANCE_THRESHOLD = 110; // needed to avoid the default of 100 when we can't see the ball
const int MAXIMUM_DISTANCE_THRESHOLD = 180;

// Ball touch detection - SetPlay handling
Timer setPlayTimeout = Timer();
const float SETPLAY_TIMEOUT = 30.0f;

// Event Constants
const float DEFAULT_MIN_TIME_TO_SEND = 3.0f;

// COMP 2025 Hack Fixes
Timer refDetectTimeout = Timer();
const float REF_DETECT_TIMEOUT = 30.0f;

// Global var to update 'active'
bool GCActive = false;

const uint8_t min_packets =
    JITTER_BUFFER * LEAVE_WIFI_SECONDS * PACKETS_PER_SECOND;

GameController::GameController(Blackboard *bb)
    : Adapter(bb), our_team(NULL), opponent_team(NULL), connected(false), active(false) {
    lastState = STATE_INITIAL;
    myLastPenalty = PENALTY_NONE;
    if (readFrom(gameController, connect)) {
        initialiseConnection();
    }
    this->touchedBall.fill(false);

    // Read in configs
    usePassing = (bb->config)["game.use_passing"].as<bool>();
    numRequiredTouches = (bb->config)["game.required_passes"].as<int>();
}

GameController::~GameController() {
    close(sock);
}

void GameController::tick() {
    /* Notes:
     * gameState represents what we think the game state should be
     * It is the same as data.state unless we hear a whistle
     * If we hear a whistle, set gameState to playing to tell our teammates
     * If the team agrees, then override the official data.state with playing
     */

    // -- Standard GameController Packet Update -- //
    uint8_t previousGameState = readFrom(gameController, gameState);
    data = readFrom(gameController, data);
    uint8_t gameState = data.state;
    teamNumber = readFrom(gameController, our_team).teamNumber;
    myPlayerNumber = readFrom(gameController, player_number);
    setOurTeam();

    if (!connected && readFrom(gameController, connect)) {
        initialiseConnection();
    }
    if (connected) wirelessUpdate();
    buttons = readFrom(motion, buttons);
    buttonUpdate();
    writeTo(motion, buttons, buttons);

    // COMP 2025 Hack Fixes
    if (gameState == STATE_STANDBY && previousGameState != STATE_STANDBY)
    {
        refDetectTimeout.restart();
    }

    // If we previously changed our gameState then don't let it get overriden by the game controller
	if ((gameState == STATE_SET && previousGameState == STATE_PLAYING)              // Kickoff whistle
            || (gameState == STATE_PLAYING && previousGameState == STATE_READY)     // Goal whistle
            || (gameState == STATE_STANDBY && previousGameState == STATE_READY))    // Referee detection
    {
        gameState = previousGameState;
    }


    // -- Referee Detection Management -- //
    if (gameState == STATE_STANDBY) {
        // Go from STANDBY --> READY when we see referee gesture
        RefereeGesture::Gesture seenGesture = readFrom(vision, refereeGesture).gesture;
        bool refereeReadyDetected = (seenGesture == RefereeGesture::Gesture::standbyToReady || 
                                     seenGesture == RefereeGesture::Gesture::goalKickBlue || // These gesture denote one arm up and the other down
                                     seenGesture == RefereeGesture::Gesture::goalKickRed);
        AbsCoord myPos = readFrom(stateEstimation, robotPos);

        if (refereeReadyDetected) {
            // If we have detected ref, then send the ready event
            EventTransmitter.raiseEvent("REF_DETECTED", true, 0.0f);
            gameState = STATE_READY;
            llog(INFO) << "STANDBY --> READY" << std::endl;
        }
        else if (refDetectTimeout.elapsed() >= REF_DETECT_TIMEOUT)
        {
            // Add timeout for ref detect
            gameState = STATE_READY;
        }

        // Check for sent events if we haven't seen signal yet
        if (gameState != STATE_READY) {
            for (int playerNum = 1; playerNum <= ROBOTS_PER_TEAM; ++playerNum) {   
                float timeSinceTouchedEvent = EventReceiver.getEventTimeSinceReceived(playerNum, "REF_DETECTED");
                if (timeSinceTouchedEvent < REF_DETECT_TIME_TO_RECEIVE) {
                    bool detectionRecieved = EventReceiver.getEventData(playerNum, "REF_DETECTED")->getUnpackedValue(false);
                    if (detectionRecieved) {
                        gameState = STATE_READY;
                        llog(INFO) << "STANDBY --> READY" << std::endl;
                        writeTo(gameController, seenRefGesture, true);
                        break;
                    }
                }
            }
        }
    }

    if (gameState == STATE_SET) {
        writeTo(gameController, seenRefGesture, false);
    }
    
    // -- Check for Passing Requests -- //
    // AbsCoord myPos = readFrom(stateEstimation, robotPos);
    // AbsCoord myPrevPos = readFrom(stateEstimation, prevRobotPos);

    // float timeSinceMyLastUpdate = EventReceiver.getEventTimeSinceReceived(myPlayerNumber, "POSITION_UPDATE");

    // for (int playerNum = 2; playerNum <= ROBOTS_PER_TEAM; ++playerNum) // Do not include goalie
    // { 
    //     float timeSincePassingRequestEvent = EventReceiver.getEventTimeSinceReceived(playerNum, "PASSING_REQUEST");
    //     int defaultVal = 0;
    //     int requestedPlayerNumbers = EventReceiver.getEventData(playerNum, "PASSING_REQUEST")->getUnpackedValue(defaultVal);

    //     // If the player has been asked to send its position or we want all player's positions (requestedPlayerNumbers = 0)
    //     // If we have never sent an update, send one
    //     // Otherwise enough time needs to have passed AND my x or y need to have NOT moved much
    //     if ((requestedPlayerNumbers == 0 || requestedPlayerNumbers == myPlayerNumber) &&
    //         (    
    //             timeSinceMyLastUpdate == -1 || 
    //                 (
    //                     timeSinceMyLastUpdate > timeSincePassingRequestEvent && 
    //                     (
    //                         abs(myPrevPos.x() - myPos.x()) < RESEND_POSITION_MOVEMENT_RANGE
    //                         || abs(myPrevPos.y() - myPos.y()) < RESEND_POSITION_MOVEMENT_RANGE
    //                     )
    //                 )
    //         ))
    //     {
    //         // If we are not the player that sent the event then send a reply
    //         EventTransmitter.raiseEvent("POSITION_UPDATE", myPos, DEFAULT_MIN_TIME_TO_SEND);
    //         break;
    //     }
    // }
    // // Update Prev pos
    // writeTo(stateEstimation, prevRobotPos, myPrevPos);


    // -- Load Received Team Positions into Blackboard -- //
    // for (int playerNum = 1; playerNum <= ROBOTS_PER_TEAM; ++playerNum)
    // { 
    //     AbsCoord defaultVal = AbsCoord();
    //     AbsCoord receivedData = EventReceiver.getEventData(playerNum, "POSITION_UPDATE")->getUnpackedValue(defaultVal);

    //     // If the robot is penalised or inactive then make its position the default value
    //     if (our_team->players[playerNum].penalty != PENALTY_NONE) 
    //     {
    //         receivedData = defaultVal;
    //     }

    //     // If the received data is not the default value
    //     if ( !(receivedData == defaultVal) )
    //     {
    //         // Write the received data to blackboard
    //         blackboard->receiver->data[playerNum].sharedStateEstimationBundle.robotPos = receivedData;
    //     }
    // }


    // -- Ball Contact Detection/Management -- //
    if (usePassing) // If we are not in fallback mode
    {
        // Evaluate whether or not enough robots have touched the ball
        if (data.kickingTeam == our_team->teamNumber
                && (data.setPlay == SET_PLAY_GOAL_KICK
                || data.setPlay == SET_PLAY_PUSHING_FREE_KICK 
                || data.setPlay == SET_PLAY_CORNER_KICK 
                || data.setPlay == SET_PLAY_GOAL_KICK)
            )
        {
            previousSetPlay = data.setPlay;
            setPlayTimeout.restart();
        }
        // If we have returned to game without timing out then assume that the kick was failed (redo indirect kick)
        else if (data.state == STATE_SET || 
            (data.setPlay == SET_PLAY_NONE && previousSetPlay != SET_PLAY_NONE && setPlayTimeout.elapsed() >= SETPLAY_TIMEOUT))
        {
            previousSetPlay = SET_PLAY_NONE;
            writeTo(stateEstimation, hasTouchedBall, false);
            this->touchedBall.fill(false);
        }
        else
        {
            // Read in the necessary values from blackboard
            RRCoord relBallPos = readFrom(stateEstimation, ballPosRR);
            std::vector<BallInfo> seenBall = readFrom(vision, balls);

            // Check how many frames we have seen the ball for
            if (seenBall.size() > 0) 
            {
                ballSeenFrames += 1;
            }
            else 
            {
                ballSeenFrames = 0;
            }
            // If the ball is infront of us, we can currently see the ball and we are close enough to the ball
            if (abs(relBallPos.heading()) <= ROBOT_HEADING_THRESHOLD && ballSeenFrames >= 3
                && (MINIMUM_DISTANCE_THRESHOLD <= relBallPos.distance() && relBallPos.distance() <= MAXIMUM_DISTANCE_THRESHOLD))
            {
                // If we have touched the ball then send the HAS_TOUCHED_BALL event
                if (!this->touchedBall[myPlayerNumber - 1])
                {
                    writeTo(stateEstimation, hasTouchedBall, true);
                    EventTransmitter.raiseEvent("HAS_TOUCHED_BALL", true, TOUCHED_BALL_TIME_TO_SEND);
                }
            }
            // If the above conditions have been met then inform the team that we have made contact with the ball
            for (int playerNum = 1; playerNum <= ROBOTS_PER_TEAM; ++playerNum) 
            {   
                float timeSinceTouchEvent = EventReceiver.getEventTimeSinceReceived(playerNum, "HAS_TOUCHED_BALL");
                if (0 <= timeSinceTouchEvent && timeSinceTouchEvent < TOUCHED_BALL_TIME_TO_SEND) 
                {
                    this->touchedBall[playerNum - 1] = EventReceiver.getEventData(playerNum, "HAS_TOUCHED_BALL")->getUnpackedValue(false);;
                }
            }
        }

        // Check if we have enough touches for an indirect kick
        int total_touched_ball = std::count(this->touchedBall.begin(), this->touchedBall.end(), true);
        bool canScore = false;

        // If we are in a penalty kick we can always score
        if (data.setPlay == SET_PLAY_PENALTY_KICK)
        {
            canScore = true;
        }
        // Handle being in the penalty kick setPlay (Always direct)
        else if (data.setPlay == SET_PLAY_PENALTY_KICK && data.kickingTeam == our_team->teamNumber)
        {
            canScore = true;
        }
        // If we have enough team ball touches to go for a goal
        else if (total_touched_ball >= numRequiredTouches)
        {
            canScore = true;
        }
        // If we have 1 touch less than the required, check if the current robot has touched the ball
        else if (total_touched_ball == numRequiredTouches - 1)
        {
            canScore = !this->touchedBall[myPlayerNumber - 1];
        }

        writeTo(stateEstimation, canScore, canScore);
    }


    // -- Whistle Detection -- //
    bool whistleDetected = false;
    if (!readFrom(whistle, whistleThreadCrashed)) {
        whistleDetected = (readFrom(whistle, whistleDetectionState) == isDetected);
    } else {
        whistleDetected = false;
        goalWhistleHeard = false;
    }

    if (whistleDetected) 
    {
        // If the confidence is set to 0 then don't check with team, just go if bot hears a whistle
        if (data.state == STATE_SET && previousGameState != STATE_PLAYING)
        {
            gameState = STATE_PLAYING;
            llog(INFO) << "SET --> PLAYING" << std::endl;
        }
        // else if (data.state == STATE_PLAYING && previousGameState != STATE_READY)
        // {
        //     goalTimer.restart();
        //     goalWhistleHeard = true;
        //     gameState = STATE_READY;
        //     llog(INFO) << "PLAYING --> READY" << std::endl;
        // }
    } 

    // // Handling for the robot being kept in the ready state
    // if (data.state == STATE_PLAYING)
    // {
    //     /* 
    //      * This is only needed while gc is in playing
    //      * Should also prevent the issue where the bots go into ready instead of the finished state
    //      */

    //     // Keeps the robot in ready
    //     if (goalWhistleHeard && goalTimer.elapsed() <= STAY_IN_READY_SECS) {
    //         llog(DEBUG) << "Goal Scored - Staying in ready" << std::endl;
    //         llog(DEBUG) << "Time remaining in Ready before return to Playing: " << double(STAY_IN_READY_SECS - goalTimer.elapsed()) << std::endl;
    //         gameState = STATE_READY;

    //         // If can still see ball after the minimum time has passed then go back to playing
    //         for (int playerNum = 1; playerNum <= ROBOTS_PER_TEAM; playerNum++)
    //         {
    //             // Uses the most recent ball score update to check if a bot has seen the ball
    //             float timeSinceLastBallUpdate = EventReceiver.getEventTimeSinceReceived(playerNum, "BALL_SCORE_UPDATE");

    //             if (timeSinceLastBallUpdate < (goalTimer.elapsed() - MIN_STAY_IN_READY)) {
    //                 goalWhistleHeard = false;
    //                 gameState = STATE_PLAYING;
    //             }
    //         }
    //     }
    //     // Timeout: If the GC doesn't go into Ready within time limit then return to Playing
    //     else if (goalWhistleHeard && goalTimer.elapsed() > STAY_IN_READY_SECS) {
    //         llog(INFO) << "Exiting Goal Scored Ready -> Timeout" << std::endl;
    //         gameState = STATE_PLAYING;
    //         goalWhistleHeard = false;
    //     }
    // }

    writeTo(gameController, data, data);
    writeTo(gameController, our_team, *our_team);
    writeTo(gameController, opponent_team, *opponent_team);
    writeTo(gameController, gameState, gameState);

    // llog(INFO) << "Message Budget: " << our_team->messageBudget << std::endl;
}

void GameController::initialiseConnection() {
    llog(INFO) << "GameController: Connecting on port "
               << GAMECONTROLLER_DATA_PORT << endl;
    stringstream s;
    s << GAMECONTROLLER_DATA_PORT;

    struct addrinfo myInfo, *results;
    memset(&myInfo, 0, sizeof myInfo);
    myInfo.ai_family = AF_UNSPEC;
    myInfo.ai_socktype = SOCK_DGRAM;
    myInfo.ai_flags = AI_PASSIVE;  // use my IP

    if (getaddrinfo(NULL, s.str().c_str(), &myInfo, &results) == -1) {
        llog(ERROR) << "GameController: Invalid Address Information" << endl;
        return;
    }

    // loop through all the results and bind to the first we can
    struct addrinfo *p;
    for (p = results; p != NULL; p = p->ai_next) {
        if ((sock = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
            llog(INFO) << "GameController: Cannot use Socket, trying next"
                       << endl;
            continue;
        }

        // set the socket to reuse ports (so we can have multiple instances of
        // redbackbots listening to the same UDP port ;-) )
        int enable = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) != 0)
        {
             llog(ERROR) << "Could not set socket options: " << errno << endl;
             continue;
        }

        if (bind(sock, p->ai_addr, p->ai_addrlen) == -1) {
            close(sock);
            llog(INFO) << "GameController: Cannot Bind, trying next" << endl;
            continue;
        }
        break;
    }

    if (p == NULL) {
        llog(ERROR) << "GameController: Failed to bind socket" << endl;
        return;
    }

    // We don't want memory leaks...
    freeaddrinfo(results);

    llog(INFO) << "GameController: Connected on port - " << s.str() << endl;
    connected = true;
    writeTo(gameController, connected, connected);
}

void GameController::buttonUpdate() {
    static int pressedTime=0;
    bool chestPressed=LoLAData::sensors.sensors[Sensors::ChestBoard_Button];
    if (chestPressed) {
        pressedTime++;
        if(pressedTime>=2){
            llog(INFO) << "button pushed once, switching state" << endl;
            switch (our_team->players[myPlayerNumber - 1].penalty) {
            case PENALTY_NONE:
                our_team->players[myPlayerNumber - 1].penalty =
                    PENALTY_MANUAL;
                  SAY((string("Penalised for ") +
                        gameControllerPenaltyNames[our_team->players[myPlayerNumber - 1].penalty]).c_str());
                break;
            default:
                our_team->players[myPlayerNumber - 1].penalty =
                    PENALTY_NONE;
                  SAY("Playing");
            }
        }
    }else{
        pressedTime=0;
    }
}

void GameController::wirelessUpdate() {
    // Setup receiving client
    int bytesRecieved;
    struct sockaddr_storage clientAddress;
    socklen_t addr_len = sizeof(clientAddress);

    // Setup buffer to write to
    int dataSize = sizeof(RoboCupGameControlData);
    unsigned char buffer[dataSize + 1];

    // Setup for polling
    struct pollfd ufds[1];
    ufds[0].fd = sock;
    ufds[0].events = POLLIN;                // For incoming packets

    // Congested WiFi: Try to grab several packets in one tick
    // to clear buffers and lower effective latency
    for (int i = 0; i < 5; i++) {
        int rv = poll(ufds, 1, POLL_TIMEOUT);  // Wait up to POLL_TIMEOUT ms

        // Check to see if we've received a packet
        if (rv > 0) {
            bytesRecieved = recvfrom(sock, buffer, dataSize, 0,
                                     (struct sockaddr *)&clientAddress,
                                     &addr_len);
            // Should use inet_ntop, but we don't need IPv6 support so meh
            writeTo(gameController, lastGameControllerIPAddress, inet_ntoa(
                ((struct sockaddr_in *)&clientAddress)->sin_addr
            ));
            if (bytesRecieved > 0) {
                llog(DEBUG) << "GameController: Received " << bytesRecieved
                             << "/"<<sizeof(RoboCupGameControlData)<<" bytes" << endl;
                for (const auto &item: buffer){
                    llog(DEBUG) <<std::hex<<(0xFF&item);
                }
                llog(DEBUG) << endl;
                parseData((RoboCupGameControlData*)buffer);
                handleFinishedPacket();
            }
            // Set active GC to true
            if (!GCActive) {
                GCActive = true;
                writeTo(gameController, active, GCActive);
            }
        }
    }
}

void GameController::handleFinishedPacket() {
    if (data.state != STATE_FINISHED ||
            data.gamePhase != GAME_PHASE_NORMAL) {
        // If not FINISHED, or in a timeout or penalty shootout game state
        finishedTimes.clear();
        return;
    }
    time_t now = time(0);  // get time now
    finishedTimes.push_back(now);
    double diff = difftime(finishedTimes.front(), finishedTimes.back());
    if (abs(diff) > LEAVE_WIFI_SECONDS &&
        finishedTimes.size() > min_packets
    ) {
       SAY("THANKS AND SEE YOU LATER");
    }
}

void GameController::parseData(RoboCupGameControlData *update) {
    if (isValidData(update)) {
        bool manualPenalty = our_team->players[myPlayerNumber - 1].penalty == PENALTY_MANUAL;

        // Update the data
        if (!gameDataEqual(update, &data)) {
            memcpy(&data, update, sizeof(RoboCupGameControlData));
            setOurTeam();
        }

        if (manualPenalty){
            our_team->players[myPlayerNumber - 1].penalty = PENALTY_MANUAL;
        }

        llog(TRACE) << "GameController: Valid data" << endl;
        if (data.state != lastState) {
            char comboState[100];
            strcpy(comboState, gameControllerGamePhaseNames[update->gamePhase]);
            strcat(comboState, gameControllerStateNames[data.state]);
            SAY(comboState);
            lastState = data.state;
        }

        unsigned char myPenalty = our_team->players[myPlayerNumber - 1].penalty;

        if (myPenalty != myLastPenalty) {
            if (myPenalty == PENALTY_NONE) {
               SAY("Unpenalised");
            } else {
                // TODO this crashes (at use of map). When we implement penalising a robot, we should reintroduce this log
                SAY((string("Penalised for ") +
                    gameControllerPenaltyNames[myPenalty]).c_str());
            }
            myLastPenalty = myPenalty;
        }
    } else {
        llog(ERROR) << "GameController: Invalid data" << endl;
    }
}

void GameController::setOurTeam() {// make our_team point to the my actual team, based on teamNumber
    bool leftTeam = false;
   if (data.teams[0].teamNumber == teamNumber) {
        our_team = &(data.teams[0]);
        opponent_team = &(data.teams[1]);
        leftTeam = true;
   } else if (data.teams[1].teamNumber == teamNumber) {
      our_team = &(data.teams[1]);
      opponent_team = &(data.teams[0]);
   } else {
        llog(ERROR) << "We are team " << teamNumber << " but GC is sending " <<
                  data.teams[0].teamNumber << " and " << data.teams[1].teamNumber << "\n";
   }
   writeTo(gameController, leftTeam, leftTeam);
}

bool GameController::isValidData(RoboCupGameControlData *gameData) {
    // check the right structure header has come in
    if (!(checkHeader(gameData->header))) {
        llog(ERROR) << "GameController: DATA HEADER MISMATCH! "
                    << "Expected: " << GAMECONTROLLER_STRUCT_HEADER
                    << " received: " << gameData->header << endl;
        return false;
    }

    // check for partial packets
    if (sizeof(*gameData) != sizeof(RoboCupGameControlData)) {
        llog(ERROR) << "GameController: RECEIVED PARTIAL PACKET! "
                    << "Expected: " << sizeof(RoboCupGameControlData)
                    << " received: " << sizeof(*gameData) << endl;
        return false;
    }

    // check the right version of the structure is being used
    if (gameData->version != GAMECONTROLLER_STRUCT_VERSION) {
        llog(ERROR) << "GameController: DATA VERSION MISMATCH! "
                    << "Expected: " << GAMECONTROLLER_STRUCT_VERSION
                    << " received: " << gameData->version << endl;
        return false;
    }

    // check whether this packet belongs to this game at all
    if (!isThisGame(gameData)) {
        llog(ERROR) << "GameController: DATA NOT FOR THIS GAME!" << endl;
        return false;
    }

    // Data is valid ^_^
    return true;
}

bool GameController::checkHeader(char* header) {
    for (int i = 0; i < 4; i++) {
        if (header[i] != GAMECONTROLLER_STRUCT_HEADER[i]) return false;
    }
    return true;
}

bool GameController::isThisGame(RoboCupGameControlData* gameData) {
   return gameData->teams[0].teamNumber == teamNumber ||
          gameData->teams[1].teamNumber == teamNumber;
}

bool GameController::gameDataEqual(void* gameData, void* previous) {
    if (!memcmp(previous, gameData, sizeof(RoboCupGameControlData))) {
        return true;
    }
    return false;
}
