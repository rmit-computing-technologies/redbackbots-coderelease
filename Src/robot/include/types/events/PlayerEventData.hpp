#pragma once

#include <array>
#include <chrono>
#include <tuple>
#include <memory>
#include <utility>

#include "types/events/EventTypes.hpp"
#include "utils/EventDefinitions.hpp"

using std::array;
using std::chrono::system_clock;
using std::tuple_size;
using std::get;

/**
 * @struct PlayerEventData
 * @brief A struct for managing a player's event data.
 *
 * The PlayerEventData struct is designed to store a player's event data,
 * including their event objects, receive times, and validity status.
 *
 */
struct PlayerEventData {
    int playerNumber;
    array<EventDataPtr, EVENTS_ARRAY_SIZE> events;
    array<system_clock::time_point, EVENTS_ARRAY_SIZE> receiveTimes;
    array<bool, EVENTS_ARRAY_SIZE> hasReceiveTime;

    PlayerEventData(int number = 0)
        : playerNumber(number),
          events(createEventsArray())
    {
        hasReceiveTime.fill(false);
        receiveTimes.fill(system_clock::time_point());
    }

    bool isValid() const {
        return playerNumber > 0;
    }
};
