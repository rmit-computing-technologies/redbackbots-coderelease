/**
 * @file LineSpotsCandidatesOverlay.hpp
 * 
 * Draw the line spots
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"

#include "types/camera/CameraInfo.hpp"

class LineSpotsCandidatesOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultLineSpotsWidth  = 3;

   enum class Type {
      Final,
      Before,
      After
   };


   LineSpotsCandidatesOverlay(CameraInfo::Camera whichCamera, int maxX, int maxY, Type type, bool groupedCandidates);
   virtual ~LineSpotsCandidatesOverlay() {};

   void drawCross(QPoint point, int size);
   void drawPlus(QPoint point, int size);

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

private:
   CameraInfo::Camera whichCamera;
   unsigned int maxX;
   unsigned int maxY;
   QColor horizontalColour;
   QColor verticalColour;
   Type type;
   int lineThickness;
   bool groupedCandidates;
};