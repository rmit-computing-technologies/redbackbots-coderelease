#pragma once

/**
 * @file StateEstimationDefinitions.hpp
 * 
 * This is used to export state estimation limits of pose and obstacle counts to blackboard
 * to instantiate blackboard storage.
 * 
 * Note this is not used in the state estimation logic
 * TODO: (TW) ensure limits are checked!
*/

#define MAX_POSE_HYPOTHESES 8
#define MAX_ROBOT_OBSTACLES 7
