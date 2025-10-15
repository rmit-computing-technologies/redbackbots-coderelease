#pragma once

#include "EventData.hpp"
#include "types/geometry/AbsCoord.hpp"

#define EVENT_NAME "TEAM_BALL_UPDATE"
#define EVENT_CRITICALITY Criticality::HIGH

#define PACKED_DATA_TYPE std::array<short, 2>
#define UNPACKED_DATA_TYPE AbsCoord

using TeamBallUpdateEventDataTemplate =
    EventData<PACKED_DATA_TYPE, UNPACKED_DATA_TYPE>;

/**
 * @class TeamBallUpdateEventData
 * @brief A class representing the event data for a team ball update.
 *
 * This class inherits from EventDataTemplate and is used to handle the event
 * data related to a team ball update in the system.
 */
class TeamBallUpdateEventData : public TeamBallUpdateEventDataTemplate {
public:
  /**
   * @brief Constructs a new TeamBallUpdateEventData object.
   *
   * Initializes the TeamBallUpdateEventData with the specified event
   * criticality and event name.
   */
  TeamBallUpdateEventData()
      : TeamBallUpdateEventDataTemplate(EVENT_CRITICALITY, EVENT_NAME) {
    llog(INFO) << "TeamBallUpdateEventData constructor called." << std::endl;
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
   * @brief Overrides the print method to stream TeamBallUpdateEventData specifics.
   *
   * @param os The output stream.
   */
  void print(std::ostream& os) const override {
      os << "TeamBallUpdateEventData - Packed: (" << packed_data[0] << ", " << packed_data[1]
         << ") , Unpacked: (" << unpacked_data.x() << ", " << unpacked_data.y() << ")";
  }
#endif
};

// Implement the packData and unpackData methods
inline void TeamBallUpdateEventData::packData() {
    llog(INFO) << "Packing TeamBallUpdateEventData." << std::endl;
    packed_data[0] = static_cast<short>(unpacked_data.x());
    packed_data[1] = static_cast<short>(unpacked_data.y());
    llog(INFO) << "Packed: " << packed_data[0] << ", " << packed_data[1] << std::endl;
    llog(INFO) << "Packed TeamBallUpdateEventData." << std::endl;
}

inline void TeamBallUpdateEventData::unpackData() {
    llog(INFO) << "Unpacking TeamBallUpdateEventData." << std::endl;
    unpacked_data.x() = static_cast<float>(packed_data[0]);
    unpacked_data.y() = static_cast<float>(packed_data[1]);
    llog(INFO) << "Unpacked: " << unpacked_data << std::endl;
    llog(INFO) << "Unpacked TeamBallUpdateEventData." << std::endl;
}
