// EventDefinitions.hpp
#pragma once

#include <climits>
#include <boost/variant.hpp>
#include <string>
#include <array>
#include <ostream>
#include <memory>
#include <chrono>
#include <ctime>

#include "types/geometry/AbsCoord.hpp"
#include "utils/CriticalityConfig.hpp"
#include "types/events/EventDataTypes.hpp"
#include "utils/Logger.hpp"

#define DEFAULT_CRITICALITY Criticality::MEDIUM

using std::array;

// Define EVENT_HASH_TYPE and EVENT_HASH_BIT_SIZE
using EVENT_HASH_TYPE = uint32_t;

// Forward declaration of AbstractEventData
class AbstractEventData;

// Define a pointer to AbstractEventData for convenience.
// Used for all references to event data objects.
using EventDataPtr = AbstractEventData*;

/**
 * @brief The size of the event hash IN BITS.
 *
 * This constant defines the size of the event hash in bits, based on the type
 * used for event hashing. This is responsible for setting the number of unique
 * event types we can support by the size of the hash type, as each bit in this
 * hash is responsible for a unique event type.
 */
constexpr std::size_t EVENT_HASH_BIT_SIZE = sizeof(EVENT_HASH_TYPE) * CHAR_BIT;

// The maximum number of events that can be stored in the event hash
static constexpr std::size_t MAX_EVENTS = EVENT_HASH_BIT_SIZE;

// Array size for storing events to be the same as the event hash size in the packet
static const std::size_t EVENTS_ARRAY_SIZE = EVENT_HASH_BIT_SIZE;

// The number of packets that should be left in the message budget at the end of the game
constexpr int RESERVE_PACKETS = 50;

// The ideal packet usage proportion
constexpr float IDEAL_PACKET_USAGE_PROPORTION = 0.75;

// The ideal packet usage count
constexpr int IDEAL_PACKET_USAGE_COUNT = static_cast<int>((PACKET_LIMIT - RESERVE_PACKETS) * IDEAL_PACKET_USAGE_PROPORTION);

// The ideal packet rate (packets/second)
constexpr float IDEAL_PACKET_RATE = static_cast<float>(IDEAL_PACKET_USAGE_COUNT) / GAME_TIME;

// The maximum increase in packet send time when scaling
constexpr float MAX_SEND_TIME_SCALE = 5.0f;

// The minimum increase in packet send time when scaling
constexpr float MIN_SEND_TIME_SCALE = 0.0f;

/**
 * @brief Calculates the rate at which packets are sent based on the game time.
 *
 * @param packetsSent The number of packets sent.
 * @param secondsRemainingInHalf The number of seconds remaining in the current half.
 * @param firstHalf Indicates whether it is the first half of the game.
 * @return The calculated packet rate as a floating-point value. (packets/second)
 */
inline float calculatePacketRate(int packetsSent, uint16_t secondsRemainingInHalf, bool firstHalf) {
    // Convert the unsigned secondsRemainingInHalf to signed by direct casting
    // This is to handle overtime scenarios where the seconds remaining in the half can be negative
    int16_t signedSecondsRemaining = static_cast<int16_t>(secondsRemainingInHalf);
    int secondsRemaining = firstHalf ? signedSecondsRemaining + (GAME_TIME / 2) : signedSecondsRemaining;
    int secondsElapsed = GAME_TIME - secondsRemaining;

    llog(DEBUG) << "Seconds remaining: " << secondsRemaining << ", seconds elapsed: " << secondsElapsed << std::endl;

    if (secondsElapsed <= 0) {
        secondsElapsed = 1;
    }

    float packetRate = static_cast<float>(packetsSent) / secondsElapsed;

    llog(DEBUG) << "Packets sent: " << packetsSent << ", seconds elapsed: " << secondsElapsed << std::endl;
    llog(DEBUG) << "Packet rate: " << packetRate << " packets/second" << std::endl;

    return packetRate;
}

/**
 * @brief Calculates the projected packet usage based on the current packet rate.
 *
 * This function computes the projected packet usage as a proportion of the ideal packet rate.
 * It logs the packet rate and the ideal packet rate for debugging purposes.
 *
 * @param packetRate The current packet rate, representing the number of packets sent per second.
 * @return The projected packet usage as a float, representing the proportion of the ideal packet rate.
 */
inline float calculateProjectedPacketUsage(float packetRate) {
    llog(DEBUG) << "Calculating projected packet usage" << std::endl;
    llog(DEBUG) << "Packet rate: " << packetRate << ", ideal packet rate: " << IDEAL_PACKET_RATE << std::endl;
    llog(DEBUG) << "Projected packet usage: " << packetRate / IDEAL_PACKET_RATE << std::endl;

    return packetRate / IDEAL_PACKET_RATE;
}

// Packed struct to hold the event message header
#pragma pack(push, 1)
struct EventMessageHeader {
    uint8_t playerNum;
    uint32_t eventHash;
};
#pragma pack(pop)

// Calculate the maximum packet event data size
constexpr std::size_t MAX_PACKET_EVENT_DATA_SIZE =
    (MAX_MESSAGE_SIZE - sizeof(EventMessageHeader));
