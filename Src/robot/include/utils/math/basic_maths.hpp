#pragma once

#include "types/geometry/Point.hpp"

#include <algorithm>
#include <cmath>

/*
 * This now wraps the C++11 implementation if possible.
 * The C++ cmath version is strictly faster at the bit or compiler instruction level.
 */
#ifndef MAX
   template <class T>
   inline static T MAX(const T &x, const T &y) {
      return std::max(x, y);
   }
   inline static float MAX(const float x, const int y) {
      return (x > y ? x : y);
   }
   inline static float MAX(const int x, const float y) {
      return (x > y ? x : y);
   }
#endif

/*
 * This now wraps the C++11 implementation if possible.
 * The C++ cmath version is strictly faster at the bit or compiler instruction level.
 */
#ifndef MIN
   template <class T>
   inline static T MIN(const T &x, const T &y) {
      return std::min(x, y);
   }
   inline static float MIN(const float x, const int y) {
      return (x > y ? y : x);
   }
   inline static float MIN(const int x, const float y) {
      return (x > y ? y : x);
   }
#endif

/*
 * This is now just a wrapper on the C++11 implementation.
 * The C++ cmath version is strictly faster as the operation 
 * is done at the bit-level
 */
// TODO (TW): Just remove this and replace each instance inline in the code
template <class T>
inline static T ABS(const T &x) {
   return std::abs(x);
}

template <class T>
inline static T SQUARE(const T &x) {
   return x * x;
}

template <class T>
inline static T DISTANCE_SQR(const T &xA,
                             const T &yA,
                             const T &xB,
                             const T &yB) {
   return SQUARE(xA - xB) + SQUARE(yA - yB);
}

// This is slow. The sqrt probably sucks.
inline static float DISTANCE(const float xA,
                             const float yA,
                             const float xB,
                             const float yB) {
   return sqrt(DISTANCE_SQR(xA, yA, xB, yB));
}

template <class T>
inline static int SIGN(const T &x) {
   return (x < 0 ? -1 : (x > 0) ? 1 : 0);
}

template <typename T>
inline static T crop(const T &x, const T &minimum, const T &maximum)
{
   if (x < minimum) {
      return minimum;
   } else if (x > maximum) {
      return maximum;
   } else {
      return x;
   }
}

//TODO: duplicate code in angles.hpp
inline static float normaliseTheta(float theta) {
   double r = fmod(theta - M_PI, 2.0*M_PI);
   if (r > 0) {
      return r - M_PI;
   } else {
      return r + M_PI;
   }
}

inline double pointSegmentDist(Point point, Point lineStart, Point lineEnd) {
   Point v;
   v.x() = lineEnd.x() - lineStart.x();
   v.y() = lineEnd.y() - lineStart.y();

   Point w;
   w.x() = point.x() - lineStart.x();
   w.y() = point.y() - lineStart.y();

   double c1 = w.x()*v.x() + w.y()*v.y();
   if (c1 <= 0.0) {
      return sqrt(w.x()*w.x() + w.y()*w.y());
   }

   double c2 = v.x()*v.x() + v.y()*v.y();
   if (c2 <= c1) {
      double dx = point.x() - lineEnd.x();
      double dy = point.y() - lineEnd.y();

      return sqrt(dx*dx + dy*dy);
   }

   double b = c1 / c2;
   Point Pb;
   Pb.x() = lineStart.x() + v.x()*b;
   Pb.y() = lineStart.y() + v.y()*b;

   double dx = point.x() - Pb.x();
   double dy = point.y() - Pb.y();
   return sqrt(dx*dx + dy*dy);
}

/**
 * Calculates the logit of a floating point value.
 * The logit function is defined as logit(p) = log(p/(1-p)).
 * From BHuman 
 * @param a The value
 * @return The logit of \c a.
 */
inline static float LOGIT(float a) {
   if(a < 0.5f)
     return std::log(a / (1.f - a));
   else
     return -std::log(1.f / a - 1.f);
 }