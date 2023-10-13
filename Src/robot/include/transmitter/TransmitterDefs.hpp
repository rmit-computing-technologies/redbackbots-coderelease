#pragma once

/**
 * the type of what this is actually receiving.
 * DO NOT CHANGE THIS WITHOUT FINISHING ALL THE TODOS IN OFFNAO TRANSMITTER
 * historically used only by the off-nao transmitter.  TODO(jayen) refactor to
 * have separate types for command masks and data masks.
 */
typedef uint64_t OffNaoMask_t;

constexpr OffNaoMask_t BLACKBOARD_MASK      = 0x0000000000000001ull;
constexpr OffNaoMask_t SALIENCY_MASK        = 0x0000000000000002ull;
constexpr OffNaoMask_t RAW_IMAGE_MASK       = 0x0000000000000004ull;
constexpr OffNaoMask_t PARTICLE_FILTER_MASK = 0x0000000000000008ull;
constexpr OffNaoMask_t ROBOT_FILTER_MASK    = 0x0000000000000010ull;
constexpr OffNaoMask_t LANDMARKS_MASK       = 0x0000000000000020ull;

constexpr OffNaoMask_t WHITEBOARD_MASK      = 0x0000000000000040ull;
constexpr OffNaoMask_t USE_BATCHED_MASK     = 0x0000000000000080ull;

constexpr OffNaoMask_t INITIAL_MASK         = BLACKBOARD_MASK | SALIENCY_MASK;
constexpr OffNaoMask_t ALL_MASKS            = INITIAL_MASK | RAW_IMAGE_MASK | PARTICLE_FILTER_MASK | ROBOT_FILTER_MASK | LANDMARKS_MASK;

// if the first bit is set in the 64bit mask, the message is intended for the Nao robot
// If the first bit is 0, the message is intended for the computer running offnao
constexpr OffNaoMask_t TO_NAO_MASKS         = 0x8000000000000000ull;
constexpr OffNaoMask_t COMMAND_MASK         = 0x8000000000000000ull;

