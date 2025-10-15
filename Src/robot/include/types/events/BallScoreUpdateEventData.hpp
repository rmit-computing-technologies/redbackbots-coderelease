#pragma once

#include "EventData.hpp"

#define EVENT_NAME "BALL_SCORE_UPDATE"
#define EVENT_CRITICALITY Criticality::HIGH

#define PACKED_DATA_TYPE int
#define UNPACKED_DATA_TYPE float

using BallScoreUpdateEventDataTemplate =
    EventData<PACKED_DATA_TYPE, UNPACKED_DATA_TYPE>;

/**
 * @class BallScoreUpdateEventData
 * @brief Represents event data for ball score updates.
 *
 * Manages the packing and unpacking of ball score related data for transmission.
 */
class BallScoreUpdateEventData : public BallScoreUpdateEventDataTemplate {
public:
    /**
     * @brief Constructs a new BallScoreUpdateEventData object.
     *
     * Initializes the BallScoreUpdateEventData with the specified event
     * criticality and event name.
     */
    BallScoreUpdateEventData()
        : BallScoreUpdateEventDataTemplate(EVENT_CRITICALITY, EVENT_NAME) {
        llog(DEBUG) << "BallScoreUpdateEventData constructor called." << std::endl;
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

inline void BallScoreUpdateEventData::packData() {
        // Packs the unpacked ball score data into the packed format.
        // Converts unpacked_data into packed_data for transmission.
        llog(DEBUG) << "Packing BallScoreUpdateEventData." << std::endl;
        packed_data = static_cast<PackedDataType>(unpacked_data);
        llog(DEBUG) << "Packed: " << packed_data << std::endl;
        llog(DEBUG) << "Packed BallScoreUpdateEventData." << std::endl;
    }

inline void BallScoreUpdateEventData::unpackData() {
        llog(DEBUG) << "Unpacking BallScoreUpdateEventData." << std::endl;
        unpacked_data = static_cast<UnpackedDataType>(packed_data);
        llog(DEBUG) << "Unpacked: " << unpacked_data << std::endl;
        llog(DEBUG) << "Unpacked BallScoreUpdateEventData." << std::endl;
    }

inline bool BallScoreUpdateEventData::shouldSend(float timeToSend, float projectedPacketUsage) {
        llog(DEBUG) << "shouldSend() called for BallScoreUpdateEventData." << std::endl;
        // (Optional) This condition is so that the event is sent only if the ball score is greater than 0.
        // This is not required, and will vary based on the use case.
        // Can be left empty if not needed.
        bool result = (unpacked_data > 0);
        llog(DEBUG) << "shouldSend() result: " << (result ? "true" : "false") << std::endl;
        return result;
    }
