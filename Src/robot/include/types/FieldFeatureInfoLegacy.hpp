#pragma once

#include "types/Point.hpp"
#include "types/RRCoord.hpp"

struct FieldLinePointInfo {
   Point p, rrp;

   FieldLinePointInfo () {
   }
   FieldLinePointInfo (Point p, Point rrp)
      : p (p), rrp (rrp) {
   }
};

/* Line structure, given in terms of parameters
 * t1, t2 and t3, that satisfy the equation:
 * t1x + t2y + t3 = 0, each of quich are integral.
 **/
struct LineInfo {
   Point p1, p2;
   int t1, t2, t3;
   RRCoord rr;

   LineInfo () {
      p1 = Point(0,0);
      p2 = Point(0,0);
      t1 = 0;
      t2 = 0;
      t3 = 0;
      rr = RRCoord();
   }
   LineInfo (Point p1, Point p2, RRCoord rr = RRCoord(0,0))
      : p1(p1), p2(p2), rr(rr) {
      t1 = p1.y() - p2.y();
      t2 = p2.x() - p1.x();
      t3 = p1.x() * p2.y() - p2.x() * p1.y();
   }
};

struct CornerInfo
{
   // The main corner (kink) point.
   Point p;

   // The points that form the ends of the corner.
   // e1
   // |
   // |
   // p --- e2
   Point e1;
   Point e2;

   CornerInfo () {}
   CornerInfo (Point p, Point e1, Point e2) : p (p), e1 (e1), e2 (e2) {}
};

struct TJunctionInfo
{
   Point p;

   TJunctionInfo() {}
   TJunctionInfo(Point p) : p (p) {}
};

struct GoalBoxCornerInfo
{
   Point p;
   /* Left is from the orientation of the STRIKER */
   bool left_corner;

   GoalBoxCornerInfo() {}
   GoalBoxCornerInfo(Point p, bool left_corner) : p (p), left_corner (left_corner) {}
};

struct PenaltySpotInfo
{
   Point p;
   int w, h;

   PenaltySpotInfo() {}
   PenaltySpotInfo(Point p) : p (p) {}
   PenaltySpotInfo(Point p, int w_, int h_) : p (p), w(w_), h(h_) {}
};

struct XJunctionInfo
{
   Point p;

   XJunctionInfo() {}
   XJunctionInfo(Point p) : p (p) {}
};

struct CentreCircleInfo
{
};

struct ParallelLinesInfo
{
   LineInfo l1, l2;

   ParallelLinesInfo() {}
   ParallelLinesInfo(LineInfo l1, LineInfo l2) : l1 (l1), l2 (l2) {}
};
