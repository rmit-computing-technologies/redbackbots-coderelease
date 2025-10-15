/*
 * RobotObservation.hpp
 *
 *  Created on: 02/07/2014
 *      Author: jashmore
 */

#pragma once

#include "RobotVisionInfo.hpp"
#include "geometry/AbsCoord.hpp"
#include "geometry/RRCoord.hpp"

struct RobotObstacle {
    RRCoord rr;
    RobotVisionInfo::Type type;
    AbsCoord rrc;
    AbsCoord pos;
    //Used to determine if a heading would overlap the obstacle
    double tangentHeadingLeft;
    double tangentHeadingRight;

    //Used as a path if we want to go around the obstacle
    RRCoord evadeVectorLeft;
    RRCoord evadeVectorRight;

    virtual ~RobotObstacle () {}

    bool operator== (const RobotObstacle &other) const {
       return rr == other.rr;
    }
};
