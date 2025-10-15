/**
 * @file IsWhiteSpotsOverlay.hpp
 * 
 * Draw the spots being compared to by the isWhite functions
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class IsWhiteSpotsOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultLineSpotsWidth  = 1;


   IsWhiteSpotsOverlay(CameraInfo::Camera whichCamera);
   virtual ~IsWhiteSpotsOverlay() {};

   void drawCross(QPoint point, int size);

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};