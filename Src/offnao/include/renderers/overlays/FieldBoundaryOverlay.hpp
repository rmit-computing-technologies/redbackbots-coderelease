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

class FieldBoundaryOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultFieldBoundaryWidth  = 5;

   FieldBoundaryOverlay(CameraInfo::Camera whichCamera);
   virtual ~FieldBoundaryOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};
