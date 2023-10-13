#pragma once

#include "motion/touch/Touch.hpp"

class LoLATouch : Touch {
   public:
      explicit LoLATouch(int team, int player_number);

      ~LoLATouch() override;

      void loadCache() override;

      SensorValues getSensors(Kinematics &kinematics) override;

      bool getStanding() override;

      bool getSitUnstiff() override;

      ButtonPresses getButtons() override;
};
