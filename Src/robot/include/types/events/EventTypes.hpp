#pragma once

/**
 *
 * Convenience file to list all event types and import them where required
 * NOTE: This is where you should add new event types as they are created.
 *
 */

#include <array>
#include <memory>
#include <tuple>
#include <cstddef>

#include "utils/EventDefinitions.hpp"

// Include all event type headers
#include "types/events/TeamBallUpdateEventData.hpp"
#include "types/events/BallScoreUpdateEventData.hpp"
#include "types/events/WhistleHeardEventData.hpp"
#include "types/events/HasTouchedBallEventData.hpp"
#include "types/events/PassingRequestEventData.hpp"
#include "types/events/PositionUpdateEventData.hpp"
#include "types/events/RefDetectedEventData.hpp"
// NOTE: Add new event type includes here

// Define all event types in one place
#define EVENT_TYPES \
    X(TeamBallUpdateEventData)  \
    X(BallScoreUpdateEventData) \
    X(WhistleHeardEventData)    \
    X(HasTouchedBallEventData)  \
    X(PassingRequestEventData)  \
    X(PositionUpdateEventData)  \
    X(RefDetectedEventData)     \
    /* NOTE: Add new event types here */

/** No need to modify or add event types to anywhere below here */

// Forward declaration of event types
#define X(eventType) class eventType;
EVENT_TYPES
#undef X

// Calculate EVENTS_ARRAY_SIZE
constexpr std::size_t countDefinedEvents() {
    std::size_t count = 0;
    #define X(eventType) ++count;
    EVENT_TYPES
    #undef X
    return count;
}

constexpr std::size_t DEFINED_EVENTS_COUNT = countDefinedEvents();

// Compile-time assertion to ensure EVENTS_ARRAY_SIZE is sufficient
static_assert(EVENTS_ARRAY_SIZE >= DEFINED_EVENTS_COUNT, 
              "Number of event types exceeds EVENT_HASH_BIT_SIZE. Consider increasing the size of EVENT_HASH_TYPE.");


/**
 * @brief Creates and initializes an array of EventDataPtr.
 *
 * This function initializes an std::array of EventDataPtr with a fixed size defined by EVENTS_ARRAY_SIZE.
 * It iterates through each event type specified in EVENT_TYPES, instantiates a new event of that type,
 * and stores the pointer in the array. The function ensures that no more events are added than the
 * predefined array size.
 *
 * @return An std::array containing initialized EventDataPtr instances.
 */
inline std::array<EventDataPtr, EVENTS_ARRAY_SIZE> createEventsArray() {
    std::array<EventDataPtr, EVENTS_ARRAY_SIZE> events = { nullptr };
    std::size_t index = 0;
    #define X(eventType) \
        if (index < EVENTS_ARRAY_SIZE) { \
            events[index++] = new eventType(); \
        }
    EVENT_TYPES
    #undef X
    return events;
}