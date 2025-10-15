/**
 * @file HorizonOverlay.hpp
 * 
 * Draw the horizon
 * 
 * @author RedbackBots
 */

#pragma once

#include "renderers/OverlayPainter.hpp"


class HorizonOverlay : public OverlayPainter {
public:
   /* Defaults */
   static constexpr int defaultHorizonWidth        = 5;

   HorizonOverlay() = default;
   HorizonOverlay(QPaintDevice * device) : OverlayPainter(device) {};
   virtual ~HorizonOverlay() {};

protected:
   virtual void _drawOverlay(Blackboard* blackboard);

   /* Draw Horizon as computed by motion */
   void drawHorizon(std::pair<int, int>& horizon, int width = defaultHorizonWidth);
};
