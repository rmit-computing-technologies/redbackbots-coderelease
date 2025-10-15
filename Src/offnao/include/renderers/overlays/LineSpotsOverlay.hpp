/**
 * @file LineSpotsOverlay.hpp
 * 
 * Draw the line spots
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class LineSpotsOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultLineSpotsWidth  = 3;

   LineSpotsOverlay(CameraInfo::Camera whichCamera, int maxX, int maxY);
   virtual ~LineSpotsOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
   unsigned int maxX;
   unsigned int maxY;
};