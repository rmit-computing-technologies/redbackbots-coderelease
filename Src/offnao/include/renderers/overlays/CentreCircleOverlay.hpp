/**
 * @file CentreCircleOverlay.hpp
 * 
 * Draw the centre circle
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class CentreCircleOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultWidth  = 6;

   CentreCircleOverlay(CameraInfo::Camera whichCamera, int maxX, int maxY);
   virtual ~CentreCircleOverlay() {};

   void drawCross(QPoint, int size);

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
   unsigned int maxX;
   unsigned int maxY;
 };