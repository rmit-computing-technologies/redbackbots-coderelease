/**
 * @file GridOverlay.hpp
 * 
 * Draw a grid over the image
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"
#include "types/camera/CameraInfo.hpp"


class GridOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int width        = 2;

   GridOverlay(CameraInfo::Camera whichCamera);
   virtual ~GridOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};
