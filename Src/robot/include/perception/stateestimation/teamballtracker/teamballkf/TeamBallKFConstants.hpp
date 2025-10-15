#pragma once

#include "types/math/Eigen.hpp"

#define NUM_DIM_TEAM_BALL_POS 2
#define NUM_DIM_TEAM_BALL_VEL 2

#define NUM_DIM_TEAM_BALL_TOTAL NUM_DIM_TEAM_BALL_POS + NUM_DIM_TEAM_BALL_VEL

#define TEAM_BALL_X_DIM 0
#define TEAM_BALL_Y_DIM 1
#define TEAM_BALL_U_DIM 2
#define TEAM_BALL_V_DIM 3

#define BALL_BUFFER_RADIUS 250 
#define TICKS_PER_SECOND 30

#define TEAM_BALL_ACCELERATION -318.9f // mm/s^2

typedef Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, 1> TeamBallStateVector;
typedef Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL> TeamBallCovarianceMatrix;
