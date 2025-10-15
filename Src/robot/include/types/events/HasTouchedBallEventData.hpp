#pragma once

#include "EventData.hpp"

#define EVENT_NAME "HAS_TOUCHED_BALL"
#define EVENT_CRITICALITY Criticality::MEDIUM

#define PACKED_DATA_TYPE bool
#define UNPACKED_DATA_TYPE bool

using HasTouchedBallEventDataTemplate =
    EventData<PACKED_DATA_TYPE, UNPACKED_DATA_TYPE>;

/**
 * @class HasTouchedBallEventData
 * @brief Represents the event that is sent when a bot has touched a ball while in the playing state
 *
 * Passing behaviour helper event
 */
class HasTouchedBallEventData : public HasTouchedBallEventDataTemplate {
public:
    /**
     * @brief Constructs a new HasTouchedBallEventData object.
     *
     * Initializes the HasTouchedBallEventData with the specified event
     * criticality and event name.
     */
    HasTouchedBallEventData()
        : HasTouchedBallEventDataTemplate(EVENT_CRITICALITY, EVENT_NAME) {
        llog(DEBUG) << "HasTouchedBallEventData constructor called." << std::endl;
    }

    /**
     * @brief Determines whether the event data should be sent.
     *
     * @param timeToSend The scheduled time for sending the event.
     * @param projectedPacketUsage The rate at which packets are being sent.
     * @return true If the event should be sent.
     * @return false Otherwise.
     */
    bool shouldSend(float timeToSend, float projectedPacketUsage);

    /**
     * @brief Packs the event data into the packed format.
     *
     * Adjusts the internal packed_data based on unpacked_data.
     */
    void packData();

    /**
     * @brief Unpacks the event data from the packed format.
     *
     * Adjusts the internal unpacked_data based on packed_data.
     */
    void unpackData();

    using PackedDataType = PACKED_DATA_TYPE; /**< Alias for the packed data type. */
    using UnpackedDataType = UNPACKED_DATA_TYPE; /**< Alias for the unpacked data type. */
};

inline void HasTouchedBallEventData::packData() {
        // Packs the unpacked ball score data into the packed format.
        // Converts unpacked_data into packed_data for transmission.
        llog(DEBUG) << "Packing HasTouchedBallEventData." << std::endl;
        packed_data = unpacked_data;
        llog(DEBUG) << "Packed HasTouchedBallEventData." << std::endl;
    }

inline void HasTouchedBallEventData::unpackData() {
        llog(DEBUG) << "Unpacking HasTouchedBallEventData." << std::endl;
        unpacked_data = packed_data;
        llog(DEBUG) << "Unpacked HasTouchedBallEventData." << std::endl;
    }

inline bool HasTouchedBallEventData::shouldSend(float timeToSend, float projectedPacketUsage) {
        llog(DEBUG) << "shouldSend() called for HasTouchedBallEventData." << std::endl;
        bool result = unpacked_data;
        return result;
    }

