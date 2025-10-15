#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include <cstring>
#include "types/events/AbstractEventData.hpp"
#include "types/events/PlayerEventData.hpp"
#include "utils/EventDefinitions.hpp"
#include "utils/defs/FieldDefinitions.hpp"
#include "utils/defs/RobotDefinitions.hpp"

using std::chrono::system_clock;

// Forward declaration to allow friendship
class EventSerializer;

/**
 * @brief Event Serializer for multiple event types
 */
class EventSerializer {
public:
  /**
   * @brief Serialize events to a buffer
   * @param header EventMessageHeader struct to serialize
   * @param events Array of AbstractEventData pointers
   * @return Serialized buffer
   */
  static std::vector<uint8_t>
  serializeEvents(EventMessageHeader header,
                  const std::array<EventDataPtr, MAX_EVENTS> &events) {
    llog(INFO) << "************************ Serializing Packet ************************" << std::endl;

    // Initialize buffer with header
    std::vector<uint8_t> buffer;
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t *>(&header),
                  reinterpret_cast<uint8_t *>(&header) + sizeof(header));

    llog(INFO) << "Serialized Header:" << std::endl
                << "PlayerNum: " << static_cast<int>(header.playerNum) << std::endl
                << "EventHash: " << std::bitset<EVENT_HASH_BIT_SIZE>(header.eventHash) << std::endl;

    // Serialize event data
    size_t bitOffset = sizeof(header) * CHAR_BIT;
    for (size_t i = 0; i < events.size(); ++i) {
      if (header.eventHash & (1u << i) && events[i]) {
        llog(INFO) << "===============================================================" << std::endl;
        llog(INFO) << "Serialized Event: " << events[i]->eventName << std::endl;
        llog(INFO) << "From bit offset: " << bitOffset << std::endl;
        events[i]->serializeToBuffer(buffer, bitOffset);
        // try {
        //   llog(INFO) << "Data: " << events[i]->getPackedData() << std::endl;
        // } catch (...) {
        //   llog(INFO) << "Data: No valid cout conversion" << std::endl;
        // }
        llog(INFO) << "To bit offset: " << bitOffset << std::endl;
        llog(INFO) << "===============================================================" << std::endl;
      }
    }

    // Ensure buffer does not exceed the message size limit
    if (buffer.size() > MAX_MESSAGE_SIZE) {
      buffer.resize(MAX_MESSAGE_SIZE);
    }

    return buffer;
  }

  /**
   * @brief Deserialize events from a buffer
   * @param buffer Serialized buffer
   * @param events Array of AbstractEventData pointers to populate
   * @param header Reference to an EventMessageHeader struct
   */
  static void deserializeEvents(const std::vector<uint8_t> &buffer,
                                std::array<EventDataPtr, MAX_EVENTS> &events,
                                EventMessageHeader &header) {
    if (!deserializeHeader(buffer, header)) {
      return;
    }
    deserializeEventData(buffer, events, header);
  }

  /**
   * @brief Deserialize header from a buffer
   * @param buffer Serialized buffer
   * @param header Reference to an EventMessageHeader struct
   * @return True if successful, false otherwise
   */
  static bool deserializeHeader(const std::vector<uint8_t> &buffer,
                                EventMessageHeader &header) {
    if (buffer.size() < sizeof(EventMessageHeader)) {
      llog(ERROR) << "deserializeHeader failed: insufficient buffer size." << std::endl;
      return false;
    }
    std::memcpy(&header, buffer.data(), sizeof(header));
    llog(INFO) << "Deserialized Header:" << std::endl
                << "PlayerNum: " << static_cast<int>(header.playerNum) << std::endl
                << "EventHash: " << std::bitset<EVENT_HASH_BIT_SIZE>(header.eventHash) << std::endl;
    return true;
  }

  /**
   * @brief Deserialize event data from a buffer
   * @param buffer Serialized buffer
   * @param events Array of AbstractEventData pointers to populate
   * @param header Reference to an EventMessageHeader struct
   */
  static void deserializeEventData(const std::vector<uint8_t> &buffer,
                                   std::array<EventDataPtr, MAX_EVENTS> &events,
                                   const EventMessageHeader &header) {
    size_t bitOffset = sizeof(header) * CHAR_BIT;
    for (size_t i = 0; i < events.size(); ++i) {
      if (header.eventHash & (1u << i) && events[i]) {
        events[i]->deserializeFromBuffer(buffer, bitOffset);
      }
    }
  }

  /**
   * @brief Deserialize events for players from a buffer
   * @param buffer Serialized buffer
   * @param players Array of PlayerEventData to populate
   */
  static void deserializeEvents(
      const std::vector<uint8_t> &buffer,
      std::array<PlayerEventData, ROBOTS_PER_TEAM> &players) {
      llog(INFO) << "************************ Deserializing Packet ************************" << std::endl;

      EventMessageHeader header;
      if (!deserializeHeader(buffer, header)) {
          llog(ERROR) << "Failed to deserialize header." << std::endl;
          return;
      }
      int playerIndex = header.playerNum - 1;
      if (playerIndex < 0 ||
          playerIndex >= static_cast<int>(players.size())) {
          llog(ERROR) << "Invalid player index: " << playerIndex << std::endl;
          return;
      }
      size_t bitOffset = sizeof(header) * CHAR_BIT;
      auto &playerData = players[playerIndex];
      auto now = system_clock::now();
      for (size_t i = 0; i < playerData.events.size(); ++i) {
          if (header.eventHash & (1u << i) && playerData.events[i]) {
            // Deserialize event data from buffer with built-in deserialization method from each event
            playerData.events[i]->deserializeFromBuffer(buffer, bitOffset);
            // Set time received and flag
            playerData.receiveTimes[i] = now;
            playerData.hasReceiveTime[i] = true;

            llog(INFO) << "===============================================================" << std::endl;
            llog(INFO) << "Deserialized Event: " << playerData.events[i]->eventName << std::endl;
            // try {
            //   llog(INFO) << "Data: " << playerData.events[i]->getUnpackedData() << std::endl;
            // } catch (...) {
            //   llog(INFO) << "Data: No valid cout conversion" << std::endl;
            // }
            llog(INFO) << "Bit offset: " << bitOffset << std::endl;
            llog(INFO) << "===============================================================" << std::endl;
          }
      }
  }
};