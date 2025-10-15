#pragma once

#include "EventData.hpp" 

#define EVENT_NAME "REF_DETECTED"
#define EVENT_CRITICALITY Criticality::HIGH

// TODO: Make this send a NoneType instead of a bool
#define PACKED_DATA_TYPE bool
#define UNPACKED_DATA_TYPE bool

using RefDetectedEventDataTemplate =
    EventData<PACKED_DATA_TYPE, UNPACKED_DATA_TYPE>;

/**
 * @class RefDetectedEventData
 * @brief Represents the event that is sent when ref gives read signal during standby
 *
 * Adopted from whistleheard event
 * Manages the packing and unpacking of ready trigger related data for transmission.
 */
class RefDetectedEventData : public RefDetectedEventDataTemplate {
public:
    /**
     * @brief Constructs a new RefDetectedEventData object.
     *
     * Initializes the RefDetectedEventData with the specified event
     * criticality and event name.
     */
    RefDetectedEventData()
        : RefDetectedEventDataTemplate(EVENT_CRITICALITY, EVENT_NAME) {
        llog(DEBUG) << "RefDetectedEventData constructor called." << std::endl;
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

inline void RefDetectedEventData::packData() {
        // Packs the unpacked ball score data into the packed format.
        // Converts unpacked_data into packed_data for transmission.
        llog(DEBUG) << "Packing RefDetectedEventData." << std::endl;
        packed_data = static_cast<PackedDataType>(unpacked_data);
        llog(DEBUG) << "Packed: " << packed_data << std::endl;
        llog(DEBUG) << "Packed RefDetectedEventData." << std::endl;
    }

inline void RefDetectedEventData::unpackData() {
        llog(DEBUG) << "Unpacking RefDetectedEventData." << std::endl;
        unpacked_data = static_cast<UnpackedDataType>(packed_data);
        llog(DEBUG) << "Unpacked: " << unpacked_data << std::endl;
        llog(DEBUG) << "Unpacked RefDetectedEventData." << std::endl;
    }

inline bool RefDetectedEventData::shouldSend(float timeToSend, float projectedPacketUsage) {
        llog(DEBUG) << "shouldSend() called for RefDetectedEventData." << std::endl;
        return true;
    }
