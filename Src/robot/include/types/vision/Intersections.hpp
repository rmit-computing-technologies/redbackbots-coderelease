#pragma once

#include "types/math/Eigen.hpp"
#include <vector>

/**
 * @struct Intersections
 * This struct contains all the intersections between lines fitted through linespots.
 */
class Intersections {
public:
  struct Intersection {
    
    enum IntersectionType
    {
      L,
      T,
      X
    };

    Intersection() = default;
    Intersection(const IntersectionType& t, const Vector2f& p, const Vector2i& image, const Vector2f& d1, const Vector2f& d2, unsigned l1, unsigned l2)
    {
      type = t;
      pos = p;
      img = image;
      dir1 = d1;
      dir2 = d2;
      line1Index = l1;
      line2Index = l2;
    }

    IntersectionType type;
    Vector2f pos; /**< The field coordinates of the intersection, relative to the robot */
    Vector2i img; /**< The image coordinates of the intersection */
    Vector2f dir1; /**< The first direction of the lines intersected (in field coordinates, relative to the robot) */
    Vector2f dir2; /**< The second direction of the lines intersected (in field coordinates, realative to the robot) */
    unsigned line1Index; /**< The first line of the intersection*/
    unsigned line2Index; /**< The second line of the intersection*/
  };

  std::vector<Intersection> intersections;
};
