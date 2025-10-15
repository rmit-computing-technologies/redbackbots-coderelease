#pragma once

#include <cstdint>

/**
 * the type of what this is actually receiving.
 * DO NOT CHANGE THIS WITHOUT FINISHING ALL THE TODOS IN OFFNAO TRANSMITTER
 * historically used only by the off-nao transmitter.  TODO(jayen) refactor to
 * have separate types for command masks and data masks.
 */
typedef uint64_t OffNaoMask_t;

// Masks to send to OffNao
// if the first bit is set in the 64bit mask, the message is intended for the Nao robot
constexpr OffNaoMask_t BLACKBOARD_MASK      = 0x0000000000000001ull;
constexpr OffNaoMask_t CAMERA_IMAGE_MASK    = 0x0000000000000002ull;
constexpr OffNaoMask_t JPEG_IMAGE_MASK      = 0x0000000000000004ull;
constexpr OffNaoMask_t PARTICLE_FILTER_MASK = 0x0000000000000008ull;
constexpr OffNaoMask_t ROBOT_FILTER_MASK    = 0x0000000000000010ull;
constexpr OffNaoMask_t BB_DEBUG_MASK        = 0x0000000000000020ull;

constexpr OffNaoMask_t WHITEBOARD_MASK      = 0x0000000000000040ull;
constexpr OffNaoMask_t USE_BATCHED_MASK     = 0x0000000000000080ull;

constexpr OffNaoMask_t INITIAL_MASK         = BLACKBOARD_MASK | CAMERA_IMAGE_MASK;
constexpr OffNaoMask_t ALL_MASKS            = INITIAL_MASK | JPEG_IMAGE_MASK | PARTICLE_FILTER_MASK | ROBOT_FILTER_MASK | BB_DEBUG_MASK;

// Masks to send to robot (likely from offnao)
// If the below bit is 1, the message is intended for the computer running offnao
constexpr OffNaoMask_t TO_NAO_MASKS         = 0x0010000000000000ull;
constexpr OffNaoMask_t CAMERA_SETTINGS_MASK = 0x0000000000000001ull;
constexpr OffNaoMask_t STRING_MASK          = 0x0000000000000010ull;

