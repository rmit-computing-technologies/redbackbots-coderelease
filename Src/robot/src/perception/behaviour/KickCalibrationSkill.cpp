#include "perception/behaviour/KickCalibrationSkill.hpp"

#include "types/BallInfo.hpp"
#include "types/AbsCoord.hpp"

#define X_TARGET 200
#define Y_TARGET 60

#define DISTANCE_THRESHOLD 80
#define BALL_LOST_COUNT_THRESHOLD 50

KickCalibrationSkill::KickCalibrationSkill(): ballLostCount(0)
{
}

BehaviourRequest KickCalibrationSkill::execute(const AbsCoord &ballRel, const std::vector<BallInfo> &detectedBalls,
                                               ActionCommand::Body::Foot foot)
{
   BehaviourRequest request;
   request.actions.head.pitch = 1;

   // Default to crouching
   request.actions.body = ActionCommand::Body::WALK;
   request.actions.body.forward = 1;
   request.actions.body.left = 0;
   request.actions.body.turn = 0;
   request.actions.body.bend = 1;

   if (detectedBalls.empty()) {
      ballLostCount++;
   } else {
      ballLostCount = 0;
   }

   float adjustedY = Y_TARGET * (foot == ActionCommand::Body::LEFT ?  1 : -1);

   if (ballLostCount < BALL_LOST_COUNT_THRESHOLD) {
      float distance = ballRel.convertToRobotRelative(AbsCoord(X_TARGET, adjustedY, 0)).distance();
      if (distance <= DISTANCE_THRESHOLD) {
         request.actions.body = ActionCommand::Body::KICK;
         request.actions.body.foot = foot;
      }
   }
   return request;
}
