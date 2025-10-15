#pragma once

#include "EventData.hpp"

#define EVENT_NAME "WHISTLE_HEARD"
#define EVENT_CRITICALITY Criticality::HIGH

// TODO: Make packed data type 2-bit unsigned integer
#define PACKED_DATA_TYPE uint8_t
#define UNPACKED_DATA_TYPE int

using WhistleHeardEventDataTemplate =
    EventData<PACKED_DATA_TYPE, UNPACKED_DATA_TYPE>;

/**
 * @class WhistleHeardEventData
 * @brief Represents the event that is sent when a whistle is heard
 *
 * Manages the packing and unpacking of whistle related data for transmission.
 */
class WhistleHeardEventData : public WhistleHeardEventDataTemplate {
public:
    /**
     * @brief Constructs a new WhistleHeardEventData object.
     *
     * Initializes the WhistleHeardEventData with the specified event
     * criticality and event name.
     */
    WhistleHeardEventData()
        : WhistleHeardEventDataTemplate(EVENT_CRITICALITY, EVENT_NAME) {
        llog(DEBUG) << "WhistleHeardEventData constructor called." << std::endl;
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

inline void WhistleHeardEventData::packData() {
        // Packs the unpacked ball score data into the packed format.
        // Converts unpacked_data into packed_data for transmission.
        llog(DEBUG) << "Packing WhistleHeardEventData." << std::endl;
        packed_data = static_cast<PackedDataType>(unpacked_data);
        llog(DEBUG) << "Packed: " << packed_data << std::endl;
        llog(DEBUG) << "Packed WhistleHeardEventData." << std::endl;
    }

inline void WhistleHeardEventData::unpackData() {
        llog(DEBUG) << "Unpacking WhistleHeardEventData." << std::endl;
        unpacked_data = static_cast<UnpackedDataType>(packed_data);
        llog(DEBUG) << "Unpacked: " << unpacked_data << std::endl;
        llog(DEBUG) << "Unpacked WhistleHeardEventData." << std::endl;
    }

inline bool WhistleHeardEventData::shouldSend(float timeToSend, float projectedPacketUsage) {
        llog(DEBUG) << "shouldSend() called for WhistleHeardEventData." << std::endl;
        bool result = (unpacked_data >= 0);
        llog(DEBUG) << "shouldSend() result: " << (result ? "true" : "false") << std::endl;
        return result;
    }

