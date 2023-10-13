#pragma once

#include "Touch.hpp"

class NullTouch : Touch {
   public:
      SensorValues getSensors(Kinematics &kinematics);
      bool getStanding();
      bool getSitUnstiff();
      ButtonPresses getButtons();
   private:
      SensorValues nullSensors;
};

