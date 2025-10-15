/**
 * @file RefereeKeypointsOverlay.hpp
 * 
 * Draw the Referee Keypoints
 * 
 * @author RedbackBots
*/

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"
#include "types/vision/RefereeGesture.hpp"

#include <string>
#include <array>

class RefereeKeypointsOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultSpotWidth  = 3;

   RefereeKeypointsOverlay(CameraInfo::Camera whichCamera);
   virtual ~RefereeKeypointsOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;

   const std::array<std::string, RefereeGesture::Gesture::count> gestureToStringMap = {
      "none",
      "kickInBlue",
      "kickInRed",
      "goalKickBlue",
      "goalKickRed",
      "cornerKickBlue",
      "cornerKickRed",
      "goalBlue",
      "goalRed",
      "pushingFreeKickBlue",
      "pushingFreeKickRed",
      "fullTime",
      "substitution",
      "standbyToReady",
   };

   std::string gestureToString(RefereeGesture::Gesture refereeGesture) {
      return gestureToStringMap[(int) refereeGesture];
   }
};