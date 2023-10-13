#ifndef KICK_CALIBRATION_SKILL_HPP
#define KICK_CALIBRATION_SKILL_HPP


#include "types/BehaviourRequest.hpp"

class AbsCoord;
class BallInfo;

class KickCalibrationSkill {
   public:
      KickCalibrationSkill();
      BehaviourRequest execute(const AbsCoord &ballRel,
            const std::vector<BallInfo> &detectedBalls,
            ActionCommand::Body::Foot foot);
   private:
      long ballLostCount;
};


#endif //KICK_CALIBRATION_SKILL_HPP
