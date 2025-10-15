/**
 * @file RobotOverlay.hpp
 * 
 * Draw the robot bounding boxes
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class RobotOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultWidth  = 3;

   RobotOverlay(CameraInfo::Camera whichCamera);
   virtual ~RobotOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};