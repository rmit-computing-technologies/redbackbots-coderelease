/**
 * @file Line.hpp
 * Declares the Line type
 *
 * @author <A href=mailto:juengel@informatik.hu-berlin.de>Matthias Jüngel</A>
 * @author RedbackBots
 */

#pragma once

#include "types/math/Eigen.hpp"

/** Defines a line by two vectors*/
class Line {
public:
    Line() = default;

    Line(const Vector2f& base, const Vector2f& direction) :
        base(base), direction(direction)
    {};
    Line(const Vector2i& base, const Vector2f& direction) :
        base(base.cast<float>()), direction(direction)
    {};
    Line(const Vector2i& base, const Vector2i& direction) :
        base(base.cast<float>()), direction(direction.cast<float>())
    {};
    // Line(const Pose2f& base, float length = 1.f) :
    //     base(base.translation), direction(Pose2f(base.rotation) * Vector2f(length, 0))
    // {}
    Line(float baseX, float baseY, float directionX, float directionY) :
        base(baseX, baseY), direction(directionX, directionY)
    {}

    void normalizeDirection(){
        direction.normalize();
    };

    Vector2f base;
    Vector2f direction;
};
