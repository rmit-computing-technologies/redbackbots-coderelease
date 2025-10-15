/**
 * @file PlaneSpotsOverlay.hpp
 * 
 * Draw the line spots
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class PlaneSpotsOverlay : public OverlayPainter {
public:
   PlaneSpotsOverlay(CameraInfo::Camera whichCamera, int maxX, int maxY);
   virtual ~PlaneSpotsOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);
   void drawCross(QPoint point, int size);

private:
   CameraInfo::Camera whichCamera;
   unsigned int maxX;
   unsigned int maxY;
};