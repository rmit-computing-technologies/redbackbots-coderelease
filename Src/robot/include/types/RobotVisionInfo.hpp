#pragma once

#include "types/geometry/Point.hpp"
#include "types/geometry/RRCoord.hpp"
#include "types/BBox.hpp"

struct RobotVisionInfo {

   enum Type
   {
      rUnknown       = 0x00,
      rOwnTeam       = 0x01,
      rEnemyTeam     = 0x02,
   };

   enum Cameras
   {
       TOP_CAMERA,
       BOT_CAMERA,
       BOTH_CAMERAS,
       OLD_DETECTION
   };

   /* Names of corresponding enums from above */
   static const char *const TypeName[];

   RobotVisionInfo () :
   rr(),
   type(rUnknown),
   cameras(BOTH_CAMERAS),
   topImageCoords(),
   botImageCoords()
   {}

   RobotVisionInfo (RRCoord rr, Type type, BBox imageCoords) :
         rr(rr),
         type(type),
         cameras(OLD_DETECTION), // OLD_DETECTION?? Is this a leftover file?
         imageCoords(imageCoords),
         topImageCoords(),
         botImageCoords() {}

   RobotVisionInfo (RRCoord rr, Type type, BBox topImageCoords, BBox botImageCoords) :
       rr(rr),
       type(type),
       cameras(BOTH_CAMERAS),
       topImageCoords(topImageCoords),
       botImageCoords(botImageCoords){}

   RobotVisionInfo (RRCoord rr, Type type, BBox imageCoords, Cameras cameras) :
      rr(rr),
      type(type),
      cameras(cameras),
      topImageCoords(),
      botImageCoords() {
      if (cameras == TOP_CAMERA) {
          topImageCoords = imageCoords;
      } else if (cameras == BOT_CAMERA) {
          botImageCoords = imageCoords;
      } else if (cameras == BOTH_CAMERAS) {
          //should use function above for BOTH
      }

   }

   virtual ~RobotVisionInfo () {}

   RRCoord rr;
   Type type;
   Cameras cameras;

   BBox imageCoords; //BACKWARD COMPAT ONLY

   BBox topImageCoords;
   BBox botImageCoords;


   bool operator== (const RobotVisionInfo &other) const {
      return rr == other.rr;
   }
};
