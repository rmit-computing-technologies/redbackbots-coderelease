/**
 * @file RobotObstaclesField.hpp
 *
 * This file declares a representation that lists the obstacles that were detected in
 * the current image in robot-relative field coordinates. Only obstacles the lower
 * end of which were visible are represented.
 *
 * @author Andre Mühlenbrock
 * @author Tim Laue
 * @author Thomas Röfer
 * @author RedbackBots
 */

#pragma once

#include "types/math/Eigen.hpp"
#include "utils/debug/Assert.hpp"

class RobotObstaclesField{
public:
    enum Type {
        unknown,            /**< No jersey detected. */
        ownPlayer,          /**< Jersey of a field player of the own team found. */
        opponentPlayer,     /**< Jersey of a field player of the opponent team found. */
        ownGoalkeeper,      /**< Jersey of the own team's goalkeeper found. */
        opponentGoalkeeper, /**< Jersey of the opponent team's goalkeeper found. */
    };

    struct Obstacle {
        Vector2f center = Vector2f::Zero();      /**< Obstacle's center in robot-relative coordinates (in mm) */
        Vector2f left = Vector2f::Zero();        /**< Obstacle's left edge in robot-relative coordinates (in mm) */
        Vector2f right = Vector2f::Zero();       /**< Obstacle's right edge in robot-relative coordinates (in mm) */
        bool fallen = false;                     /**< Is the obstacle a player lying on the field? */
        Type type = Type::unknown;               /**< The type of the obstacle */
        float confidence = 0.f;                  /**< The type's confidence */
    };

    void verify() const {
        for([[maybe_unused]] const Obstacle& obstacle : obstacles)
            {
                ASSERT(std::isfinite(obstacle.center.x()));
                ASSERT(std::isfinite(obstacle.center.y()));
                ASSERT(std::isfinite(obstacle.left.x()));
                ASSERT(std::isfinite(obstacle.left.y()));
                ASSERT(std::isfinite(obstacle.right.x()));
                ASSERT(std::isfinite(obstacle.right.y()));
            }
    }

    std::vector<Obstacle> obstacles; /**< The obstacles with detected lower ends found in the current image. */
};