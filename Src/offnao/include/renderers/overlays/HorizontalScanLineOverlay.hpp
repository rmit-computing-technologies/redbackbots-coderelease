/**
 * @file HorizontalScanLineOverlay.hpp
 * 
 * Draw the scan lines
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class HorizontalScanLineOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultScanLineWidth  = 5;

   HorizontalScanLineOverlay(CameraInfo::Camera whichCamera);
   virtual ~HorizontalScanLineOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};