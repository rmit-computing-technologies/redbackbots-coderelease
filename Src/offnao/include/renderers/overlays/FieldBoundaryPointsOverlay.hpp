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

class FieldBoundaryPointsOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultFieldPointsWidth  = 3;

   FieldBoundaryPointsOverlay(CameraInfo::Camera whichCamera);
   virtual ~FieldBoundaryPointsOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};
