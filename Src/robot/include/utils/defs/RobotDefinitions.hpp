#pragma once

#include "types/math/Eigen.hpp"
#include "types/math/Angle.hpp"
#include <array>

/** 
 * @file RobotDefinitions.hpp 
 * 
 * Provides definitions about robots
 * @author Cord Niehaus
 * @author Thomas Röfer
 * @author RedbackBots
 */

// Number of robots permitted per-team (Challenger Shield)
constexpr int ROBOTS_PER_TEAM = 6;

constexpr float Y_HIP_OFFSET = 50;
constexpr float UPPER_LEG_LENGTH = 100;
constexpr float LOWER_LEG_LENGTH = 102.9;
constexpr float FOOT_HEIGHT = 45.19;
constexpr float FOOT_LENGTH = 110;
constexpr float SOLE_TO_FRONT_EDGE_LENGTH = 110;
constexpr float SOLE_TO_INNER_EDGE_LENGTH = 38.77;
constexpr float SOLE_TO_OUTER_EDGE_LENGTH = 50.02;
constexpr float SOLE_TO_BACK_EDGE_LENGTH =  50.;
static Vector2f BUMPER_INNER_EDGE = {88, 38};
static Vector2f BUMPER_OUTER_EDGE = {97, 36};
constexpr float HIP_TO_NECK_LENGTH = 211.5;

constexpr float X_OFFSET_NECK_TO_LOWER_CAMERA = 50.71;
constexpr float Z_OFFSET_NECK_TO_LOWER_CAMERA = 17.74;
static Angle TILT_NECK_TO_LOWER_CAMERA = 39.7_deg;

constexpr float X_OFFSET_NECK_TO_UPPER_CAMERA = 58.71;
constexpr float OFFSET_NECK_TO_UPPER_CAMERA = 63.64;
static Angle TILT_NECK_TO_UPPER_CAMERA = 1.2_deg;

static Vector3f ARM_OFFSET = {0, 98, 185};
constexpr float Y_OFFSET_ELBOW_TO_SHOULDER = 15;
constexpr float UPPER_ARM_LENGTH = 105;
constexpr float LOWER_ARM_LENGTH = 130; // estimated
constexpr float X_OFFSET_ELBOW_TO_WRIST = 55.95;
static Vector3f HAND_OFFSET = {57.75, 0, 12.31};
constexpr float HAND_RADIUS = 32.5;
constexpr float ARM_RADIUS = 25; // estimated

static Vector3f IMU_OFFSET = {-8, 6.06, 112};
static Vector2f LEFT_FSR_POSITIONS[] = {
    {70.25, 29.9},
    {70.25, -23.1},
    {-30.25, 29.9},
    {-29.65, -19.1}
};
static Vector2f RIGHT_FSR_POSITIONS[] = {
    {70.25, 23.1},
    {70.25, -29.9},
    {-29.65, 19.1},
    {-30.25, -29.9}
};

