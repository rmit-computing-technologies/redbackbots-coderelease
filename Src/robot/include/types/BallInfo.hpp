#pragma once

#include "types/geometry/Point.hpp"
#include "types/geometry/RRCoord.hpp"
#include "types/XYZ_Coord.hpp"

#include <iostream>
#include <cmath>


class BallInfo {
public:
   BallInfo () {}
   BallInfo (RRCoord rr, int radius, Point imageCoords) :
      rr(rr),
      radius(radius),
      imageCoords(imageCoords){}

   virtual ~BallInfo () {}

   void clear(){}
   
   void reset();

   void verify() const;

   // New
   enum Status {
      notSeen, /**< The ball was not seen. */
      seen,    /**< The ball was seen. */
      guessed, /**< Would be ok for a moving ball. */
   };

   /** Indicates, if the ball was seen in the current image. */
   Status status = notSeen;  

   /** The measurement covariance of positionOnField */
   // Matrix2f covarianceOnField;      

   /** Ball position relative to the robot. */
   RRCoord rr; // BHuman - positionOnField

   /** The radius of the ball in the current image */
   int radius; // BHuman - radiusOnField        

   /** The position of the ball in the current image */
   Point imageCoords; // BHuman - positionInImage

   /** If ball was detected on topCamera or bottom camera */
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
