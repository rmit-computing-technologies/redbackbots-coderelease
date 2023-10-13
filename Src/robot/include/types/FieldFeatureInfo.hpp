#pragma once

#include <ostream>

#include "types/Point.hpp"
#include "types/RRCoord.hpp"
#include "utils/basic_maths.hpp"
#include "FieldFeatureInfoLegacy.hpp"

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
   }

   FieldFeatureInfo () {
   }

   FieldFeatureInfo(const FieldFeatureInfo &other) {
      this->rr = other.rr;
      this->type = other.type;
      this->p1 = other.p1;
      this->p2 = other.p2;
   }

   virtual ~FieldFeatureInfo () {
   }

   RRCoord rr;
   Type type;
   /**
    * start and end points of a line, so we can see lines in offnao
    */
   Point p1;
   Point p2;
};

inline std::ostream &operator<<(std::ostream &os, const FieldFeatureInfo &info)
{
    os << " type: " << FieldFeatureInfo::TypeName[info.type] << "\n rr: \n"
       << info.rr;
    return os;
}

