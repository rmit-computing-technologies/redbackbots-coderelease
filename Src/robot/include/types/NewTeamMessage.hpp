#pragma once

#include <stdint.h>
#include "gamecontroller/RoboCupGameControlData.hpp"
#include "types/AbsCoord.hpp"
#include "types/ActionCommand.hpp"
#include "types/BroadcastData.hpp"


#define MAX_MESSAGE_SIZE      128

/*
 * Important remarks about units:
 *
 * For each parameter, the respective comments describe its unit.
 * The following units are used:
 *
 * - Distances:  Millimeters (mm)
 * - Angles:     Radian
 * - Time:       Seconds (s)
 */
struct NewTeamMessage
{
  uint8_t playerNum;     // [MANDATORY FIELD] 1-20
  uint8_t teamNum;       // [MANDATORY FIELD] the number of the team (as provided by the organizers)
  uint8_t fallen;        // [MANDATORY FIELD] 1 means that the robot is incapacitated, 0 means that the robot can play

  // [MANDATORY FIELD]
  // position and orientation of robot
  // coordinates in millimeters
  // 0,0 is in center of field
  // +ve x-axis points towards the goal we are attempting to score on
  // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
  // angle in radians, 0 along the +x axis, increasing counter clockwise
  float pose[3];         // x,y,theta

  // ball information
  float ballAge;         // seconds since this robot last saw the ball. -1.f if we haven't seen it

  // position of ball relative to the robot
  // coordinates in millimeters
  // 0,0 is in center of the robot
  // +ve x-axis points forward from the robot
  // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
  float ball[2];
  


  //BroadcastData (const int &playerNum,      DONE
                // const AbsCoord &RobotPos,  DONE
                // const AbsCoord &ballPosAbs,
    // position of ball relative to field
  float absBallPos[3];

                // const RRCoord &ballPosRR,
  float ballPosRR[3];
                // const SharedStateEstimationBundle &sharedStateEstimationBundle,
                      // AbsCoord robotPos; DONE
                      // AbsCoord ballPosRRC;
  float ballPosRRC[3];
                      // AbsCoord ballVelRRC;
  float ballVelRRC[3];
                      // bool haveBallUpdate;
  uint8_t haveBallUpdate;

                // const BehaviourSharedData &behaviourSharedData,

                        // int secondsSinceLastKick;
  int secondsSinceLastKick;

                        // // The encoded enum value for the robots role
                        // int role;
  uint8_t role=-1;

                        // // Whether the robot is playing the ball or not
                        // bool playingBall;
  uint8_t playingBall;

                        // // Whether the robot needs assistance
                        // bool needAssistance;

  uint8_t needAssistance;

                        // // Whether the robot is assisting
                        // bool isAssisting;

  uint8_t isAssisting;

                        // bool isKickedOff;
  uint8_t isKickedOff;

                        // float walkingToX; // x of where a robot is walking to
  float walkingTo[3];
                        // float walkingToY; // y of where a robot is walking to
                        // float walkingToH; // theta of where a robot is walking to

                        // // Notification to other robots that robot is about to kick the ball.
                        // bool kickNotification;

  uint8_t kickNotification;
                // const ActionCommand::Body::ActionType &acB,
  ActionCommand::Body::ActionType acb;
                // const float &uptime,
  float upTime;
                // const uint8_t &gameState);
  uint8_t gameState;


#ifdef __cplusplus
  // constructor
  NewTeamMessage() :
    playerNum(-1),
    teamNum(-1),
    fallen(-1),
    pose{0},
    ballAge(-1.f),
    ball{0},
    absBallPos{0},
    ballPosRR{0},
    ballPosRRC{0},
    ballVelRRC{0},
    haveBallUpdate(-1),
    secondsSinceLastKick(-1),
    role(-1),
    playingBall(-1),
    needAssistance(-1),
    isAssisting(-1),
    isKickedOff(-1),
    walkingTo{0},
    kickNotification(-1),
    acb(ActionCommand::Body::DEAD),
    upTime(-1),
    gameState(STATE_INITIAL)
  {
    #ifdef VALGRIND
      // initialize the allocated bytes because we might send them
      bzero(data + numOfDataBytes, sizeof(data) - numOfDataBytes);
    #endif
  }

  NewTeamMessage(const int &playerNum,
                 const int &teamNum,
                 const int &fallen,
                 const AbsCoord &robotPos,
                 const int &ballAge,
                 const AbsCoord &ballPosition,
                 const BroadcastData &broadcast)
     : playerNum(playerNum),
       teamNum(teamNum),
       fallen(fallen),
       ballAge(ballAge),
       gameState(gameState) {

      pose[0] = robotPos.x();
      pose[1] = robotPos.y();
      pose[2] = robotPos.theta();

      AbsCoord absCoordBallPosRRC = ballPosition.convertToRobotRelativeCartesian(robotPos);
      ball[0] = absCoordBallPosRRC.x();
      ball[1] = absCoordBallPosRRC.y();

      absBallPos[0] = ballPosition.x();
      absBallPos[1] = ballPosition.y();
      absBallPos[2] = ballPosition.theta();

      ballPosRR[0] = broadcast.ballPosRR.distance();
      ballPosRR[1] = broadcast.ballPosRR.heading();
      ballPosRR[2] = broadcast.ballPosRR.orientation();

      ballPosRRC[0] = absCoordBallPosRRC.x();
      ballPosRRC[1] = absCoordBallPosRRC.y();
      ballPosRRC[2] = absCoordBallPosRRC.theta();

      ballVelRRC[0] = broadcast.sharedStateEstimationBundle.ballVelRRC.x();
      ballVelRRC[1] = broadcast.sharedStateEstimationBundle.ballVelRRC.y();
      ballVelRRC[2] = broadcast.sharedStateEstimationBundle.ballVelRRC.theta();

      if (broadcast.sharedStateEstimationBundle.haveBallUpdate){
        haveBallUpdate = 1;
      }
      else{
        haveBallUpdate = 0;
      }

      secondsSinceLastKick = broadcast.behaviourSharedData.secondsSinceLastKick;
      role = broadcast.behaviourSharedData.role;
      playingBall = broadcast.behaviourSharedData.playingBall;
      needAssistance = broadcast.behaviourSharedData.needAssistance;
      isAssisting = broadcast.behaviourSharedData.isAssisting;
      isKickedOff = broadcast.behaviourSharedData.isKickedOff;

      walkingTo[0] = broadcast.behaviourSharedData.walkingToX;
      walkingTo[1] = broadcast.behaviourSharedData.walkingToY;
      walkingTo[2] = broadcast.behaviourSharedData.walkingToH;

      kickNotification = broadcast.behaviourSharedData.kickNotification;


      acb = broadcast.acB;
      upTime = broadcast.uptime;
      gameState = broadcast.gameState;

      #ifdef VALGRIND
         // initialize the rest of the allocated bytes because we might send them
         bzero(data + numOfDataBytes, sizeof(data) - numOfDataBytes);
      #endif

  int testsize = sizeof(ActionCommand::Body::ActionType);
  int testsize2 = sizeof(float[3]);
  int totalsize = sizeof(NewTeamMessage);
  }


#endif
};
