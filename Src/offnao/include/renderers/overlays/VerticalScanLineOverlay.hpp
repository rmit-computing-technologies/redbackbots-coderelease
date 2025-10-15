/**
 * @file VerticalScanLineOverlay.hpp
 * 
 * Draw the scan lines
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class VerticalScanLineOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultScanLineWidth  = 5;

   VerticalScanLineOverlay(CameraInfo::Camera whichCamera);
   virtual ~VerticalScanLineOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};