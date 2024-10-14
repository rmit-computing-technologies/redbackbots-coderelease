//
// Created by jayen on 25/01/19.
//

#include "types/BehaviourSharedData.hpp"

#include <cmath>
#include <iostream>

#include "utils/PositioningDefs.hpp"

BehaviourSharedData::BehaviourSharedData() {
   #ifdef VALGRIND
      // there are holes in the struct causing valgrind to throw warnings when we transmit this struct as raw bytes
      bzero(this, sizeof(BehaviourSharedData));
   #endif
   secondsSinceLastKick = -1;
   role = POSITIONING_NONE;
   playingBall = false;
   playingBallScore = -1.f;
   needAssistance = false;
   isAssisting = false;
   isKickedOff = false;
   walkingToX = 0.f;
   walkingToY = 0.f;
   walkingToH = 0.f;
   kickNotification = false;
}

bool BehaviourSharedData::sanityCheck() {
   // Check for secondsSinceLastKick nan
   if (std::isnan(secondsSinceLastKick)){
      std::cout << "received nan for secondsSinceLastKick" << std::endl;
      return false;
   }

   return true;
}
