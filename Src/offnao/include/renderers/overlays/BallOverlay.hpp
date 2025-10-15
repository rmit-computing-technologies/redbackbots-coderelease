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

class BallOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultBallWidth  = 5;

   BallOverlay(CameraInfo::Camera whichCamera);
   virtual ~BallOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};
