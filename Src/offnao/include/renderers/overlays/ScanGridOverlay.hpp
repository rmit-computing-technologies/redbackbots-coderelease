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

class ScanGridOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultScanGridWidth  = 5;

   ScanGridOverlay(CameraInfo::Camera whichCamera);
   virtual ~ScanGridOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};
