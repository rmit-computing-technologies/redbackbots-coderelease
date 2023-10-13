#pragma once

#include <math.h>
#include <string.h>
#include <Eigen/Eigen>

#include "types/Point.hpp"

struct RRCoord {
    /**
     * RRCoord
     * (Robot Relative Coord)
     *
     * @param heading       heading from robot to object
     * @param distance      distance from robot to object
     * @param orientation   angle between robot front and object front
     */
    RRCoord(float distance, float heading = 0, float orientation = 0)
        : vec(distance, heading, orientation)
    {
        var.setZero();
    }

    Point toCartesian() const
    {
        return Point(distance()*cosf(heading()), distance()*sinf(heading()));
    }

    RRCoord()
    {
        vec.setZero();
        var.setZero();
    }

    RRCoord(const RRCoord &other)
    {
        this->vec = other.vec;
        this->var = other.var;
    }

    RRCoord& operator=(const RRCoord &other)
    {
        this->vec = other.vec;
        this->var = other.var;

        return *this;
    }

    Eigen::Vector3f vec;
    Eigen::Matrix<float, 3, 3> var;

    /* Distance to object */
    const float distance() const
    {
        return vec[0];
    }

    float &distance()
    {
        return vec[0];
    }

    /* Distance between coords, squared */
    const float distanceSquared(const RRCoord& other) const
    {
        // The magnitudes of the vectors.
        const float mag0 = vec[0];
        const float mag1 = other.vec[0];

        // The angles of the vectors.
        const float angle0 = vec[1];
        const float angle1 = other.vec[1];

        // The squares of the magnitudes of the vectors.
        const float square0 = mag0 * mag0;
        const float square1 = mag1 * mag1;

        // Return the squared distance between the vectors.
        return(square0 + square1 - 2.0 * mag0 * mag1 * cos(angle0 - angle1));
    }

    /* Distance between coords */
    const float distance(const RRCoord& other) const
    {
        return(sqrt(distanceSquared(other)));
    }

    /* Heading to object */
    const float heading() const
    {
        return vec[1];
    }

    float &heading() {
        return vec[1];
    }

    /* Angle between robot's front and object's front */
    const float orientation() const
    {
        return vec[2];
    }

    float &orientation()
    {
        return vec[2];
    }

    bool operator== (const RRCoord &other) const
    {
        return vec == other.vec;
    }
};

inline std::ostream& operator<<(std::ostream& os, const RRCoord& coord)
{
    os << "(" << coord.distance() << ", " << coord.heading() << ", " << coord.orientation() << ")";
    return os;
}

typedef enum
{
    DIST,
    HEADING,
    ORIENTATION,
} RRCoordEnum;
