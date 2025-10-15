#pragma once

#include "types/math/Eigen.hpp"
#include <cmath>
#include <ostream>

#include "types/geometry/Point.hpp"
#include "types/geometry/RRCoord.hpp"
#include "utils/math/angles.hpp"
#include "utils/math/basic_maths.hpp"
#include "utils/defs/FieldDefinitions.hpp"

struct AbsCoord {
   AbsCoord(float x, float y, float theta) : vec(x, y, theta) {
      var.setZero();
      var(0, 0) = SQUARE(FULL_FIELD_LENGTH);
      var(1, 1) = SQUARE(FULL_FIELD_WIDTH);
      var(2, 2) = SQUARE(M_PI);
      weight = 1.0;
   }

   AbsCoord() : vec(0, 0, 0) {
      var.setZero();
      var(0, 0) = SQUARE(FULL_FIELD_LENGTH);
      var(1, 1) = SQUARE(FULL_FIELD_WIDTH);
      var(2, 2) = SQUARE(M_PI);
      weight = 1.0;
   }

   Eigen::Vector3f vec;
   Eigen::Matrix<float, 3, 3> var;
   float weight;

   const float x() const {
      return vec[0];
   }

   float &x() {
      return vec[0];
   }

   const float y() const {
      return vec[1];
   }

   float &y() {
      return vec[1];
   }

   const float theta() const {
      return vec[2];
   }

   float &theta() {
      return vec[2];
   }

   float getVar(int m, int n) const {
      return var(m,n);
   }

   bool operator== (const AbsCoord &other) const {
      return vec == other.vec;
   }

   /**
    * This assumes that the AbsCoord is already in robot relative coordinates, this will simply
    * convert it to polar coords.
    */
   RRCoord convertToRobotRelative(void) const {
      return RRCoord(sqrtf(vec.x()*vec.x() + vec.y()*vec.y()), atan2f(vec.y(), vec.x()), 0.0f);
   }

   RRCoord convertToRobotRelative(const AbsCoord &robotPose) const {
      float xdiff = x() - robotPose.x();
      float ydiff = y() - robotPose.y();
      float distance = sqrtf(xdiff*xdiff + ydiff*ydiff);

      float angle = atan2(ydiff, xdiff);
      float rrHeading = normaliseTheta(angle - robotPose.theta());

      return RRCoord(distance, rrHeading, 0.0f);
   }

   AbsCoord convertToRobotRelativeCartesian(const AbsCoord &robotPose) const {
      RRCoord rrCoord = convertToRobotRelative(robotPose);
      return AbsCoord(
            rrCoord.distance() * cos(rrCoord.heading()),
            rrCoord.distance() * sin(rrCoord.heading()),
            0.0f);
   }

   Vector2f convertToVector(void) const {
      Vector2f vCoord = Vector2f(vec.x(), vec.y());
      return vCoord;
   }
};

// Overload the << operator for AbsCoord
inline std::ostream& operator<<(std::ostream& os, const AbsCoord& coord) {
   os << "AbsCoord(x: " << coord.x()
      << ", y: " << coord.y()
      << ", theta: " << coord.theta() << ")";
   return os;
}