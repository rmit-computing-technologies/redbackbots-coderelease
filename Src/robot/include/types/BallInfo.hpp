#pragma once

#include "types/Point.hpp"
#include "types/RRCoord.hpp"
#include "types/XYZ_Coord.hpp"

#include <iostream>
#include <cmath>


struct BallInfo {
   BallInfo () {}
   BallInfo (RRCoord rr, int radius, Point imageCoords) :
      rr(rr),
      radius(radius),
      imageCoords(imageCoords){}

   virtual ~BallInfo () {}

   RRCoord rr;
   int radius;
   Point imageCoords;
   bool topCamera;

   bool operator== (const BallInfo &other) const
   {
      return rr           == other.rr
          && radius       == other.radius
          && imageCoords  == other.imageCoords
          && topCamera    == other.topCamera;
   }
};

inline std::ostream &operator<<(std::ostream &os, const BallInfo &info) {
    os << "rr: " << info.rr << std::endl;
    os << "imageCoord: " << info.imageCoords << std::endl;
    os << "radius: " << info.radius << std::endl;
    return os;
}


struct BallHint
{

   enum Type
   {
      bLeft           = 0x00,
      bRight          = 0x01,
      bHidden         = 0x02,
      bNone           = 0x03
   };

   /* Names of corresponding enums from above */
   static const char *const TypeName[];

   BallHint () :
      type(bNone) {
   }

   BallHint (Type type) :
      type(type) {
   }

   virtual ~BallHint () {
   }

   Type type;
};
