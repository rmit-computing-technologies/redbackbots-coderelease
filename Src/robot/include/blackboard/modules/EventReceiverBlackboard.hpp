#pragma once

#include <algorithm>
#include <array>
#include <boost/program_options/variables_map.hpp>
#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "types/events/EventTypes.hpp"
#include "types/events/PlayerEventData.hpp"
#include "utils/EventSerializer.hpp"
#include "utils/defs/FieldDefinitions.hpp"
#include "utils/EventDefinitions.hpp"

#include "types/events/AbstractEventData.hpp"
#include "types/events/EventData.hpp"

#define EventReceiver EventReceiverBlackboard::getInstance()

/**
 * @struct EventReceiverBlackboard
 * @brief A struct template for managing and transmitting events.
 *
 * The EventReceiverBlackboard struct template is designed to store a list of
 * all the event types and allows for the transmission of events and event data.
 *
 */
struct EventReceiverBlackboard {
  // Define EventsArraySize
  static const std::size_t EventsArraySize = EVENT_HASH_BIT_SIZE;
  static const int MaxPlayers = ROBOTS_PER_TEAM;

  // The array of players' event data
  std::array<PlayerEventData, MaxPlayers> players;

  // Constructor
  explicit EventReceiverBlackboard()
  {
    llog(DEBUG) << "Initializing EventReceiverBlackboard" << std::endl;

    for (int i = 0; i < MaxPlayers; ++i) {
      llog(DEBUG) << "Creating PlayerEventData for player: " << (i + 1) << std::endl;
      players[i] = PlayerEventData(i + 1);
    }
    llog(DEBUG) << "EventReceiverBlackboard initialized" << std::endl;
  }

  /**
   * @brief Reads configuration options.
   *
   * This function reads configuration options from a given variables map.
   *
   * @param config A boost::program_options::variables_map containing the
   * configuration options.
   */
  void readOptions(const boost::program_options::variables_map &config);

  /**
   * @brief Gets the singleton instance of EventReceiverBlackboard.
   *
   * @return EventReceiverBlackboard& The singleton instance.
   */
  static EventReceiverBlackboard &getInstance() {
    static EventReceiverBlackboard instance;
    return instance;
  }

  // Delete copy constructor and assignment operator to enforce singleton
  // pattern
  EventReceiverBlackboard(const EventReceiverBlackboard &) = delete;
  EventReceiverBlackboard &operator=(const EventReceiverBlackboard &) = delete;

  /**
   * @brief Deconstructs the EventReceiverBlackboard singleton.
   * 
   * Destroys all event objects and data stored in the players array.
   */
  ~EventReceiverBlackboard() {
    llog(DEBUG) << "Destroying EventReceiverBlackboard" << std::endl;
    for (PlayerEventData &player : players) {
        llog(DEBUG) << "Destroying Events" << std::endl;
        for (EventDataPtr &event : player.events) {
            delete event;
            event = nullptr;
        }
        llog(DEBUG) << "PlayerEventData destroyed" << std::endl;
    }
    llog(DEBUG) << "EventReceiverBlackboard destroyed" << std::endl;
  }

  /**
   * @brief Decodes an incoming packet and updates player event data.
   *
   * @param buffer The buffer containing the packet data.
   */
  void decodeIncomingPacket(std::vector<uint8_t> &buffer) {
    EventSerializer::deserializeEvents(buffer, players);
  }

  /**
   * @brief Gets the latest events for a specific player.
   *
   * @param playerNum The player number.
   * @return std::vector<EventDataPtr> The latest events for the player.
   */
  std::vector<EventDataPtr> getLatestEvent(int playerNum) {
    std::vector<EventDataPtr> latestEvents;
    if (playerNum < 1 || playerNum > MaxPlayers) {
      llog(ERROR) << "Invalid player number: " << playerNum << std::endl;
      return latestEvents;
    }
    int index = playerNum - 1;
    std::chrono::system_clock::time_point latestTime;
    bool haveLatestTime = false;

    // Find the latest receive time
    for (std::size_t i = 0; i < EventsArraySize; ++i) {
      if (players[index].events[i] && players[index].hasReceiveTime[i]) {
        if (!haveLatestTime || players[index].receiveTimes[i] > latestTime) {
          latestTime = players[index].receiveTimes[i];
          haveLatestTime = true;
        }
      }
    }
    // Collect all events with that latest time
    if (haveLatestTime) {
      for (std::size_t i = 0; i < EventsArraySize; ++i) {
        if (players[index].events[i] && players[index].hasReceiveTime[i] &&
            players[index].receiveTimes[i] == latestTime) {
          latestEvents.push_back(players[index].events[i]);
        }
      }
    }
    return latestEvents;
  }

  /**
   * @brief Returns which player most recently sent a given event type.
   *
   * @param eventName The name of the event.
   * @return int The player number who most recently sent the event.
   */
  int getLatestPlayer(const std::string &eventName) {
    int latestPlayer = -1;
    std::chrono::system_clock::time_point latestTime;

    for (int i = 0; i < MaxPlayers; ++i) {
      for (std::size_t j = 0; j < EventsArraySize; ++j) {
        auto &event = players[i].events[j];
        if (event && event->eventName == eventName &&
            players[i].hasReceiveTime[j]) {
          if (latestPlayer < 0 || players[i].receiveTimes[j] > latestTime) {
            latestTime = players[i].receiveTimes[j];
            latestPlayer = i;
          }
        }
      }
    }
    return (latestPlayer >= 0) ? (latestPlayer + 1)
                               : -1; // return -1 if no player found
  }

  /**
   * @brief Gets the event data for a specific player and event name.
   *
   * @param playerNum The player number.
   * @param eventName The event name.
   * @return EventDataPtr The event data pointer.
   */
  EventDataPtr getEventData(int playerNum,
                                  const std::string &eventName) const {
    if (playerNum < 1 || playerNum > MaxPlayers) {
      llog(ERROR) << "Invalid player number: " << playerNum << std::endl;
      return nullptr;
    }
    int index = playerNum - 1;
    auto it =
        std::find_if(players[index].events.begin(), players[index].events.end(),
                [&eventName](EventDataPtr event) {
                  return event && event->eventName == eventName;
                });
    if (it != players[index].events.end()) {
      return *it;
    }
    llog(WARNING) << "Event not found: " << eventName
                  << " for player: " << playerNum << std::endl;
    return nullptr;
  }

  /**
   * @brief Gets the event data for a specific player.
   *
   * @param playerNum The player number.
   * @return const PlayerEventData& The player event data.
   */
  const PlayerEventData& getPlayerEvents(int playerNum) const {
    if (playerNum < 1 || playerNum > MaxPlayers) {
        llog(ERROR) << "Invalid player number: " << playerNum << std::endl;
        throw std::out_of_range("Invalid player number");
    }
    return players[playerNum - 1];
  }

  /**
   * @brief Retrieves a Python object from a specific player's event slot by
   * event name.
   *
   * @param playerNum The target player number.
   * @param eventName The event's name.
   * @return A PyObject pointer or nullptr if not found.
   */
  PyObject *getPyEventData(int playerNum, const std::string &eventName) {
    llog(DEBUG) << "Getting event Python object by name: " << eventName
               << " for player: " << playerNum << std::endl;
    auto eventData = getEventData(playerNum, eventName);
    if (eventData) {
      return eventData->getPythonData();
    }
    llog(WARNING) << "Event not found: " << eventName
                  << " for player: " << playerNum << std::endl;
    return nullptr;
  }

  /**
   * @brief Gets an array of Python objects for an event type across all
   * players.
   *
   * @param eventName The event type to retrieve.
   * @return A vector of PyObject pointers in player order.
   */
  std::vector<PyObject *> getPyEventData(const std::string &eventName) {
    llog(DEBUG) << "Getting event Python objects by type: " << eventName
               << std::endl;
    auto allEvents = getEventData(eventName);
    std::vector<PyObject *> result;
    result.reserve(allEvents.size());

    for (auto &evtPtr : allEvents) {
      if (evtPtr) {
        result.push_back(evtPtr->getPythonData());
      } else {
        result.push_back(nullptr);
      }
    }
    return result;
  }

  /**
   * @brief Fetches the scheduled sending time for a player's event.
   *
   * @param playerNum The target player number.
   * @param eventName The event's name.
   * @return float Seconds until the event is sent, or -1 if not set.
   */
  float getEventTimeSinceReceived(int playerNum, const std::string &eventName) {
    // Return how many seconds since this event was received
    if (playerNum < 1 || playerNum > MaxPlayers) {
      return -1;
    }
    int index = playerNum - 1;
    auto it =
        std::find_if(players[index].events.begin(), players[index].events.end(),
                [&eventName](EventDataPtr event) {
                  return event && event->eventName == eventName;
                });
    if (it != players[index].events.end()) {
      std::size_t eventIndex = std::distance(players[index].events.begin(), it);
      if (players[index].hasReceiveTime[eventIndex]) {
        auto now = std::chrono::system_clock::now();
        auto diff = now - players[index].receiveTimes[eventIndex];
        return std::chrono::duration_cast<std::chrono::duration<float>>(diff)
            .count();
      } else {
        // llog(DEBUG) << "Event: " << eventName << " never received"
        //            << " for player: " << playerNum << std::endl;
        return -1;
      }
    }
    llog(WARNING) << "Event not found: " << eventName
                  << " for player: " << playerNum << std::endl;
    return -1;
  }

  /**
   * @brief Gets the time since the event was received for a specific event name across all players.
   *
   * @param eventName The event name.
   * @return std::array<float, MaxPlayers> An array of times since the event was received for each player.
   */
  std::array<float, MaxPlayers> getEventTimesSinceReceived(const std::string &eventName) {
    std::array<float, MaxPlayers> times;
    for (int i = 0; i < MaxPlayers; ++i) {
      times[i] = getEventTimeSinceReceived(i + 1, eventName);
    }
    return times;
  }

  /**
   * @brief Gets the event data for a specific event name across all players.
   *
   * @param eventName The event name.
   * @return std::vector<EventDataPtr> The event data pointers.
   */
  std::vector<EventDataPtr> getEventData(const std::string &eventName) {
    std::vector<EventDataPtr> result;
    for (int i = 0; i < MaxPlayers; ++i) {
      auto it = std::find_if(players[i].events.begin(), players[i].events.end(),
                             [&eventName](EventDataPtr event) {
                               return event && event->eventName == eventName;
                             });
      if (it != players[i].events.end()) {
        result.push_back(*it);
      } else {
        result.push_back(nullptr);
      }
    }
    return result;
  }

  /**
   * @brief Returns the player number of the most recent packet received 
   *        (the player with the events that has the most recent receive time).
   * @return int The player number of the most recent packet received.
   */
  int getLatestPlayer() {
    int mostRecentPlayer = -1;
    std::chrono::system_clock::time_point mostRecentTime;
    for (int i = 0; i < MaxPlayers; ++i) {
      for (std::size_t j = 0; j < EventsArraySize; ++j) {
        if (players[i].events[j] && players[i].hasReceiveTime[j]) {
          if (mostRecentPlayer < 0 ||
              players[i].receiveTimes[j] > mostRecentTime) {
            mostRecentTime = players[i].receiveTimes[j];
            mostRecentPlayer = i;
          }
        }
      }
    }
    return (mostRecentPlayer >= 0) ? (mostRecentPlayer + 1) : -1;
  }

  /**
   * @brief Returns the most recent event data from the most recent packet received.
   */
  std::vector<EventDataPtr> getLatestEvents() {
    return getLatestEvent(getLatestPlayer());
  }

  /**
   * @brief Gets the receive time of a specific event for a player.
   *
   * @param playerNum The player number.
   * @param eventName The event name.
   * @return std::chrono::system_clock::time_point The receive time of the event.
   */
  std::chrono::system_clock::time_point getEventReceiveTime(int playerNum, const std::string &eventName) const {
    if (playerNum < 1 || playerNum > MaxPlayers) {
        llog(ERROR) << "Invalid player number: " << playerNum << std::endl;
        return std::chrono::system_clock::time_point::min();
          }
    int index = playerNum - 1;
    auto it = std::find_if(players[index].events.begin(), players[index].events.end(),
                          [&eventName](EventDataPtr event) {
                              return event && event->eventName == eventName;
                          });
    
    if (it != players[index].events.end()) {
        std::size_t eventIndex = std::distance(players[index].events.begin(), it);
        if (players[index].hasReceiveTime[eventIndex]) {
            return players[index].receiveTimes[eventIndex];
        }
    }
  }
};
