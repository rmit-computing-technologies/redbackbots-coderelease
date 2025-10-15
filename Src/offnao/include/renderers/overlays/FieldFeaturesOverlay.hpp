/**
 * @file FieldFeaturesOverlay.hpp
 * 
 * Draw the field features
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class FieldFeaturesOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultLineWidth  = 3;

   FieldFeaturesOverlay(CameraInfo::Camera whichCamera);
   virtual ~FieldFeaturesOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};
