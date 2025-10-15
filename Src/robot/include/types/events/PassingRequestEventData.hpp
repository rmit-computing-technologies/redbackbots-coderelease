#pragma once

#include "EventData.hpp"

#define EVENT_NAME "PASSING_REQUEST"
#define EVENT_CRITICALITY Criticality::HIGH

#define PACKED_DATA_TYPE uint8_t
#define UNPACKED_DATA_TYPE int

using PassingRequestEventDataTemplate =
    EventData<PACKED_DATA_TYPE, UNPACKED_DATA_TYPE>;

/**
 * @class PassingRequestEventData
 * @brief Represents event data for passing data requests
 *
 * Manages the packing and unpacking of ball score related data for transmission.
 */
class PassingRequestEventData : public PassingRequestEventDataTemplate {
public:
    /**
     * @brief Constructs a new PassingRequestEventData object.
     *
     * Initializes the PassingRequestEventData with the specified event
     * criticality and event name.
     */
    PassingRequestEventData()
        : PassingRequestEventDataTemplate(EVENT_CRITICALITY, EVENT_NAME) {
        llog(DEBUG) << "PassingRequestEventData constructor called." << std::endl;
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

inline void PassingRequestEventData::packData() {
        llog(DEBUG) << "Packing PassingRequestEventData." << std::endl;
        packed_data = static_cast<PackedDataType>(unpacked_data);
        llog(DEBUG) << "Packed PassingRequestEventData." << std::endl;
    }

inline void PassingRequestEventData::unpackData() {
        llog(DEBUG) << "Unpacking PassingRequestEventData." << std::endl;
        unpacked_data = static_cast<UnpackedDataType>(packed_data);
        llog(DEBUG) << "Unpacked PassingRequestEventData." << std::endl;
    }

inline bool PassingRequestEventData::shouldSend(float timeToSend, float projectedPacketUsage) {
        llog(DEBUG) << "shouldSend() called for PassingRequestEventData." << std::endl;
        return unpacked_data >= 0;
    }