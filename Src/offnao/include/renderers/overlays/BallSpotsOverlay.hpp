/**
 * @file HorizonOverlay.hpp
 * 
 * Draw the horizon
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class BallSpotsOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultSpotPointsWidth  = 3;

   BallSpotsOverlay(CameraInfo::Camera whichCamera);
   virtual ~BallSpotsOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};
