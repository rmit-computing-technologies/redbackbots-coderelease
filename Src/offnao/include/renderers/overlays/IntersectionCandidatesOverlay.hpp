/**
 * @file IntersectionCandidatesOverlay.hpp
 * 
 * Draw the horizon
 * 
 * @author RedbackBots
*/

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class IntersectionCandidatesOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultWidth  = 3;

   IntersectionCandidatesOverlay(CameraInfo::Camera whichCamera);
   virtual ~IntersectionCandidatesOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
};
