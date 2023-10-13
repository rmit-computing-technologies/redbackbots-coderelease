#ifndef MOTION_DEBUG_INFO_HPP
#define MOTION_DEBUG_INFO_HPP

#include "types/FeetPosition.hpp"

class MotionDebugInfo
{
  public:

    FeetPosition feetPosition;
    float x;
    float y;

    MotionDebugInfo() {};
};

#endif // MOTION_DEBUG_INFO_HPP
