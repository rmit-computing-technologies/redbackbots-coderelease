/**
 * @file PenaltyMarkRegionsOverlay.hpp
 * 
 * Draw the potential penalty mark regions
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class PenaltyMarkRegionsOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultLineWidth  = 3;

   PenaltyMarkRegionsOverlay(CameraInfo::Camera whichCamera);
   virtual ~PenaltyMarkRegionsOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};