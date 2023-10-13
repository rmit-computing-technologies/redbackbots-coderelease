#pragma once

#include <stdint.h>
#include "types/BroadcastData.hpp"

#define SPL_STANDARD_MESSAGE_STRUCT_HEADER  "SPL "
#define SPL_STANDARD_MESSAGE_STRUCT_VERSION 9

#define SPL_STANDARD_MESSAGE_DATA_SIZE      92

/**
 * Important remarks about units:
 *
 * For each parameter, the respective comments describe its unit.
 * The following units are used:
 *
 * - Distances:  Millimeters (mm)
 * - Angles:     Radian
 * - Time:       Seconds (s)
 * @deprecated This is replaced by NewTeamMessage. Pending deletion
 */
struct SPLStandardMessage
{
  char header[4];        // "SPL "
  uint8_t version;       // has to be set to SPL_STANDARD_MESSAGE_STRUCT_VERSION
  uint8_t playerNum;     // [MANDATORY FIELD] 1-20
  uint8_t teamNum;       // [MANDATORY FIELD] the number of the team (as provided by the organizers)
  uint8_t fallen;        // [MANDATORY FIELD] 1 means that the robot is fallen, 0 means that the robot can play

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

  // number of bytes that is actually used by the data array
  uint8_t numOfDataBytes;

  // buffer for arbitrary data, teams do not need to send more than specified in numOfDataBytes
  uint8_t data[SPL_STANDARD_MESSAGE_DATA_SIZE];

#ifdef __cplusplus
  // constructor
  SPLStandardMessage() :
    version(SPL_STANDARD_MESSAGE_STRUCT_VERSION),
    playerNum(-1),
    teamNum(-1),
    fallen(-1),
    ballAge(-1.f),
    numOfDataBytes(0)
  {
    const char* init = SPL_STANDARD_MESSAGE_STRUCT_HEADER;
    for(unsigned int i = 0; i < sizeof(header); ++i)
      header[i] = init[i];
    pose[0] = 0.f;
    pose[1] = 0.f;
    pose[2] = 0.f;
    ball[0] = 0.f;
    ball[1] = 0.f;
    #ifdef VALGRIND
      // initialize the allocated bytes because we might send them
      bzero(data + numOfDataBytes, sizeof(data) - numOfDataBytes);
    #endif
  }

  SPLStandardMessage(const int &playerNum,
                     const int &teamNum,
                     const int &fallen,
                     const AbsCoord &robotPos,
                     const int &ballAge,
                     const AbsCoord &ballPosition,
                     const BroadcastData &broadcast)
     : playerNum(playerNum),
       teamNum(teamNum),
       fallen(fallen),
       ballAge(ballAge) {

      const char* init = SPL_STANDARD_MESSAGE_STRUCT_HEADER;
      for(unsigned int i = 0; i < sizeof(header); ++i)
         header[i] = init[i];
      version = SPL_STANDARD_MESSAGE_STRUCT_VERSION;
      
      pose[0] = robotPos.x();
      pose[1] = robotPos.y();
      pose[2] = robotPos.theta();

      AbsCoord ballPosRRC = ballPosition.convertToRobotRelativeCartesian(robotPos);
      ball[0] = ballPosRRC.x();
      ball[1] = ballPosRRC.y();

      // Everything else we need goes into the "data" section
      numOfDataBytes = sizeof(broadcast);
      assert(numOfDataBytes <= SPL_STANDARD_MESSAGE_DATA_SIZE);   // May this line of code save someone from the segfault hell I experienced (well actually you're probably building with NDEBUG... I'll add a cout below)
      if (numOfDataBytes > SPL_STANDARD_MESSAGE_DATA_SIZE)
      {
        std::cout << "BroadcastData is too big! SPL Standard Message data size is " << SPL_STANDARD_MESSAGE_DATA_SIZE << " and BroadcastData is " << numOfDataBytes << std::endl;
      }
      memcpy(data, &broadcast, numOfDataBytes);
      #ifdef VALGRIND
         // initialize the rest of the allocated bytes because we might send them
         bzero(data + numOfDataBytes, sizeof(data) - numOfDataBytes);
      #endif
  }


#endif
};
