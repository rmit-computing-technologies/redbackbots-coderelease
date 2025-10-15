/**
 * @file OverlayPainter.hpp
 * 
 * Base class of overlays that can be drawn.
 * This base version can be used to draw common items
 * 
 * This also implements a virtual drawOverlay() method
 * which sub-classes use to draw their features
 * 
 * @author RedbackBots
 */

#pragma once

#include <QPainter>
#include "types/geometry/Point.hpp"

class BallInfo;
class BBox;
class FieldBoundary;
class FieldFeatureInfo;
class GoalInfo;
class RegionI;
class RobotVisionInfo;

// Forward declare blackboard
class Blackboard;

class OverlayPainter : public QPainter {
public:
   /* Defaults */
   static constexpr int defaultPointSize           = 5;

   OverlayPainter();
   OverlayPainter(QPaintDevice * device);
   virtual ~OverlayPainter();

   /* 
    * Common Drawing Methods, allows setting properties 
    * compared to QPainter versions
   */
   void drawLinePath(const QPainterPath &path, QColor colour);
   void drawPolygon(QPolygon poly, QBrush fill);
   void drawRectangle(Point &a, Point &b);

   /* Custom set pen taking width and colour values */
   void setPen_(QColor colour, int width);

   /* 
    * Draw this overlay using data from the provided blackboard.
    * Override this method for sub-classes
    */
   void drawOverlay(Blackboard* blackboard);

   /* Enabled status of the overlay */
   bool getEnabled();
   void setEnabled(bool enabled);

   /*
    * DEPRECATED METHODS - KEPT AS VISION PORT IS IN PROGRESS
    */

   /* FORMER drawing methods. Saved ATM for reference */
   void drawRobotBox(const BBox &region, const QBrush &colour);

   // Draws a region's bounding box on the image.
   void drawRegionOverlay (const RegionI &region);
   void drawRegionBox(const RegionI &region, const QBrush &colour);

   void drawRobotOverlay(const RobotVisionInfo &robot);
   void drawFieldFeatureOverlay(const FieldFeatureInfo &fieldFeature);
   void drawFieldFeatureOverlay(const FieldFeatureInfo &fieldFeature,
                                 QColor colour);

protected:
   static QPoint p2q(const Point& p) {
      return QPoint(p.x(), p.y()); 
   }
   static QPoint v2i2q(const Vector2i& vec) { 
      return QPoint(vec.x(), vec.y()); 
   }
   static QPoint v2f2q(const Vector2f& vec) { 
      return QPoint(vec.x(), vec.y()); 
   }
   static QPoint xy2q(const int x, const int y) {
      return QPoint(x, y); 
   }

   /* Internal draw. Subclasses override this method */
   virtual void _drawOverlay(Blackboard* blackboard);

   // Track if this overlay is enabled
   bool enabled = true;
};
