/**
 * @file CircleCandidatesOverlay.hpp
 * 
 * Draw the Circle Candidates
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class CircleCandidatesOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultLineSpotsWidth  = 3;

   CircleCandidatesOverlay(CameraInfo::Camera whichCamera, bool groupedCandidates);
   virtual ~CircleCandidatesOverlay() {};

   void drawCross(QPoint point, int size);

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
   bool groupedCandidates;
};