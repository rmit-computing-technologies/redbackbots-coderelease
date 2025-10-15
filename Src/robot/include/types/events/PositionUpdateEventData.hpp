#pragma once

#include "EventData.hpp"
#include "types/geometry/AbsCoord.hpp"

#define EVENT_NAME "POSITION_UPDATE"
#define EVENT_CRITICALITY Criticality::HIGH

#define PACKED_DATA_TYPE std::array<short, 3>
#define UNPACKED_DATA_TYPE AbsCoord

#define USE_LOGS 0

using PositionUpdateEventDataTemplate =
    EventData<PACKED_DATA_TYPE, UNPACKED_DATA_TYPE>;

/**
 * @class PositionUpdateEventData
 * @brief A class representing the event data for a robot to update its position.
 *
 * This class inherits from EventDataTemplate and is used to handle the event
 * data related to robot position update in the system.
 */
class PositionUpdateEventData : public PositionUpdateEventDataTemplate {
public:
  /**
   * @brief Constructs a new PositionUpdateEventData object.
   *
   * Initializes the PositionUpdateEventData with the specified event
   * criticality and event name.
   */
  PositionUpdateEventData()
      : PositionUpdateEventDataTemplate(EVENT_CRITICALITY, EVENT_NAME) {
    llog(INFO) << "PositionUpdateEventData constructor called." << std::endl;
  }

  /**
   * @brief Packs the event data into a buffer.
   *
   * Converts internal data structures into a format suitable for transmission.
   */
  void packData() override;

  /**
   * @brief Unpacks the event data from a buffer.
   *
   * Converts received data into internal data structures for processing.
   */
  void unpackData() override;

  using PackedDataType = PACKED_DATA_TYPE; /**< Alias for the packed data type. */
  using UnpackedDataType = UNPACKED_DATA_TYPE; /**< Alias for the unpacked data type. */

#ifdef __cplusplus
  /**
   * @brief Overrides the print method to stream PositionUpdateEventData specifics.
   *
   * @param os The output stream.
   */
  void print(std::ostream& os) const override {
      os << "PositionUpdateEventData - Packed: (" << packed_data[0] << ", " << packed_data[1] << ", " << packed_data[2]
         << ") , Unpacked: (" << unpacked_data.x() << ", " << unpacked_data.y() << ", " << unpacked_data.theta() << ")";
  }
#endif
};

// Implement the packData and unpackData methods
inline void PositionUpdateEventData::packData() {
    // llog(DEBUG) << "Packing PositionUpdateEventData." << std::endl;
    packed_data[0] = static_cast<short>(unpacked_data.x());
    packed_data[1] = static_cast<short>(unpacked_data.y());
    packed_data[2] = static_cast<short>(unpacked_data.theta());
    // llog(DEBUG) << "Packed: " 
    //             << packed_data[0] << ", " 
    //             << packed_data[1] << ", " 
    //             << packed_data[2] << std::endl;
    llog(DEBUG) << "Packed PositionUpdateEventData." << std::endl;
}

inline void PositionUpdateEventData::unpackData() {
    // llog(DEBUG) << "Unpacking PositionUpdateEventData." << std::endl;
    unpacked_data.x() = static_cast<float>(packed_data[0]);
    unpacked_data.y() = static_cast<float>(packed_data[1]);
    unpacked_data.theta() = static_cast<float>(packed_data[2]);
    // llog(DEBUG) << "Unpacked: " << unpacked_data << std::endl;
    llog(DEBUG) << "Unpacked PositionUpdateEventData." << std::endl;
}
