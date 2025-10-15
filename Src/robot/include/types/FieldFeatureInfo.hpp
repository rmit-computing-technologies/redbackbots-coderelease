#pragma once

#include <ostream>

#include "types/geometry/Point.hpp"
#include "types/geometry/RRCoord.hpp"
#include "utils/math/basic_maths.hpp"
#include "FieldFeatureInfoLegacy.hpp"

// TODO TW: Combined or correlate this with
//          the types/field/FieldFeatureLocations.hpp
//          I suspect that these were developed separately and how have duplicate items
//          Investigate as part of vision port and field feature detection

struct FieldFeatureInfo {

   enum Type
   {
      fNone           = 0x00,
      fLine           = 0x01,
      fCorner         = 0x02,
      fTJunction      = 0x03,
      fPenaltySpot    = 0x04,
      fCentreCircle   = 0x05,
      fFieldLinePoint = 0x06,
      fXJunction      = 0x07,
      fParallelLines  = 0x08,
      fGoalBoxCorner  = 0x09,
      NUMBER_OF_TYPES
   };

   /* Names of corresponding enums from above */
   static const char *const TypeName[NUMBER_OF_TYPES];

   FieldFeatureInfo (RRCoord rr, Type type) :
      rr(rr),
      type(type) {

      p1.setZero();
      p2.setZero();

      field1.setZero();
      field2.setZero();
   }

   FieldFeatureInfo () {
   }

   FieldFeatureInfo(const FieldFeatureInfo &other) {
      this->rr = other.rr;
      this->type = other.type;
      this->p1 = other.p1;
      this->p2 = other.p2;
      this->field1 = other.field1;
      this->field2 = other.field2;
      this->topCamera = other.topCamera;
   }

   virtual ~FieldFeatureInfo () {
   }

   RRCoord rr;
   Type type;
   /**
    * start and end points of a line, so we can see lines in offnao
    */
   Point p1; // First Point in image coordinates
   Point p2; // Last Point in image coordinates

   PointF field1; // First Point in field coordinates
   PointF field2; // Last Point in field coordinates
   
   float fieldLinesWidth;
   bool topCamera;
};

inline std::ostream &operator<<(std::ostream &os, const FieldFeatureInfo &info)
{
    os << " type: " << FieldFeatureInfo::TypeName[info.type] << "\n rr: \n"
       << info.rr;
    return os;
}

