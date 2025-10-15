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
#include <boost/optional.hpp>

#include "types/events/EventTypes.hpp"
#include "utils/EventDefinitions.hpp"

#define EventTransmitter EventTransmitterBlackboard::getInstance()

/**
 * @struct EventTransmitterBlackboard
 * @brief A struct template for managing and transmitting events.
 *
 * The EventTransmitterBlackboard struct template is designed to store a list of
 * all the event types and allows for the transmission of events and event data.
 *
 */
struct EventTransmitterBlackboard {
  // Define EVENTS_ARRAY_SIZE
  static const std::size_t EVENTS_ARRAY_SIZE = EVENT_HASH_BIT_SIZE;

  // The array of pointers to event objects
  std::array<EventDataPtr, EVENTS_ARRAY_SIZE> events;

  // Time tracking arrays, etc.
  std::array<std::chrono::system_clock::time_point, EVENTS_ARRAY_SIZE> eventRaisedTimes;
  std::array<float, EVENTS_ARRAY_SIZE> eventTimeToSend;
  std::array<bool, EVENTS_ARRAY_SIZE> eventRaisedSet;

  std::array<bool, EVENTS_ARRAY_SIZE> eventSent;
  std::array<std::chrono::system_clock::time_point, EVENTS_ARRAY_SIZE> eventSentTimes;

  /**
   * @brief Constructs the EventTransmitterBlackboard singleton.
   * 
   * Initializes various internal arrays and logs the event transmitter blackboard creation.
   */
  explicit EventTransmitterBlackboard()
      : events(createEventsArray()) // Initialize events array using the factory function
  {
    llog(DEBUG) << "Initializing EventTransmitterBlackboard" << std::endl;
    eventRaisedSet.fill(false);
    eventTimeToSend.fill(0.0f);
    eventRaisedTimes.fill(std::chrono::system_clock::time_point());
    eventSent.fill(false);
    eventSentTimes.fill(std::chrono::system_clock::time_point());
    llog(DEBUG) << "EventTransmitterBlackboard initialized" << std::endl;
  }

  /**
   * @brief Deconstructs the EventTransmitterBlackboard singleton.
   * 
   * Destroys all event objects stored in the events array.
   */
  ~EventTransmitterBlackboard() {
    llog(DEBUG) << "Destroying EventTransmitterBlackboard" << std::endl;
    for (EventDataPtr &event : events) {
      delete event;
      event = nullptr;
    }
    llog(DEBUG) << "EventTransmitterBlackboard destroyed" << std::endl;
  }

  /**
  * @brief Reads configuration options from a variables map.
  * 
  * Currently, this function does not handle any specific options.
  * @param config A boost::program_options::variables_map containing the configuration options.
  */
  void readOptions(const boost::program_options::variables_map &config); 

  /**
   * @brief Raises an event by name.
   * 
   * Looks up an event by name, optionally sets its data if provided, and updates the send time.
   * @tparam T The type of data to store in the event.
   * @param name The name of the event.
   * @param value Optional data value to store. If not provided, the event data is not updated.
   * @param seconds_until_send Optional time in seconds until the event is sent. Defaults to 0.0.
   * @throws std::runtime_error If there is a type mismatch when setting event data.
   */
  template <typename T>
  void raiseEvent(const std::string &name, const T &value = boost::none, boost::optional<float> seconds_until_send = boost::none) {
    llog(DEBUG) << "Raising event: " << name << std::endl;
    auto it = std::find_if(events.begin(), events.end(), [&name](EventDataPtr event) {
      return event && event->eventName == name;
    });
    if (it != events.end()) {
      std::size_t index = std::distance(events.begin(), it);
      llog(DEBUG) << "Event found at index: " << index << std::endl;
      
      // If a value is provided, and not a boost::none, set the event data
      if (!std::is_same<T, boost::none_t>::value) {
        UnpackedDataVariant dataVariant = value;
        if (!(*it)->setUnpackedData(dataVariant)) {
          llog(ERROR) << "Type mismatch when setting event data for event: " << name << std::endl;
          throw std::runtime_error("Type mismatch when setting event data.");
        }
      }
      float timeToSend = seconds_until_send ? *seconds_until_send : 0.0f;
      updateEventTimeIfSooner(index, timeToSend);
    } else {
      llog(WARNING) << "Event not found: " << name << std::endl;
    }
  }

  /**
   * @brief Determines if any raised event warrants sending a packet.
   * 
   * Checks the size limit, criticalities, and timing of events to decide if sending is needed.
   * @param projectedPacketUsage The predicted future usage of the packet.
   * @return True if a packet should be sent, false otherwise.
   */
  bool shouldSendPacket(float projectedPacketUsage) {
    llog(DEBUG) << "Checking if packet should be sent" << std::endl;
    std::size_t cumulativeSizeBits = 0;
    for (std::size_t i = 0; i < EVENTS_ARRAY_SIZE; ++i) {
      if (events[i] && isEventRaised(i)) {  // TODO: add use of local can send to each event
        // Check if the event is due to be sent
        if (isEventDueToSend(i, projectedPacketUsage)) {
          llog(INFO) << "Event at index " << i << " is due to be sent" << std::endl;
          llog(DEBUG) << "Event at index " << i << " has bit size "
                      << events[i]->bitSize << "." << std::endl;
          llog(INFO) << "Event at index " << i << " has name "
                      << events[i]->eventName << "." << std::endl;
          llog(DEBUG) << "Event at index " << i << " has size of type "
                      << events[i]->getPackedDataSize() * CHAR_BIT << "."
                      << std::endl;
          llog(DEBUG) << "Event at index " << i << " has criticality "
                      << events[i]->criticality << "." << std::endl;
          return true;
        }
        // Add the size of the event to the cumulative size in bits
        cumulativeSizeBits += events[i]->bitSize;
      }
    }
    // Check if the cumulative size in bits reaches the packet size limit
    bool shouldSend =
        cumulativeSizeBits >= (MAX_PACKET_EVENT_DATA_SIZE * CHAR_BIT);
    if (shouldSend) {

      llog(INFO) << "Cumulative size of triggered events in bits: " << cumulativeSizeBits << std::endl;
    }
    return shouldSend;
  }

  /**
   * @brief Checks if an event at a given index is due to be sent.
   * 
   * Uses timing and usage thresholds to determine if it's time to send the event.
   * @param index The index in the event array.
   * @param projectedPacketUsage The predicted future usage of the packet.
   * @return True if the event can and should be sent, false otherwise.
   */
  bool isEventDueToSend(std::size_t index, float projectedPacketUsage) const {
    if (index >= EVENTS_ARRAY_SIZE) {
      llog(ERROR) << "isEventDueToSend called with out-of-bounds index: "
                  << index << std::endl;
      return false;
    }

    if (eventRaisedSet[index]) {
      auto now = std::chrono::system_clock::now();
      float timeToSend = 0;
      timeToSend = scaledTimeToSend(index, eventTimeToSend[index], projectedPacketUsage);

      auto scheduledTime = eventRaisedTimes[index]
          + std::chrono::milliseconds(static_cast<int>(timeToSend * 1000));
      bool timeDue = now >= scheduledTime;

      llog(DEBUG) << "Event at index " << index << " is due to send by time: " << timeDue << std::endl;

      bool canTriggerPacket = false;
      {
        Criticality level = events[index]->criticality;
        // Locate the CriticalityUsageThreshold for this event's level
        auto it = std::find_if(
            CriticalityDetails.begin(),
            CriticalityDetails.end(),
            [level](const CriticalityUsageThreshold &threshold){
                return threshold.level == level;
            }
        );
        if (it != CriticalityDetails.end()) {
            // Only allow if below the "do not send" threshold
            if (projectedPacketUsage <= it->usageProjectionImpact) {
                canTriggerPacket = true;
            } else {
                llog(INFO) << "Projected packet usage exceeds threshold for event at index " << index << std::endl;
                canTriggerPacket = false;
            }
        }
      }

      if (canTriggerPacket) {
        llog(INFO) << "Event at index " << index << " can trigger a packet." << std::endl;
      }

      return timeDue && canTriggerPacket;
    }
    return false;
  }

  /**
   * @brief Scales the time to send an event based on projected usage.
   * 
   * Adjusts the event's scheduled send time to be earlier or later to avoid packet overload.
   * @param index The index of the event in the array.
   * @param timeToSend The current time-to-send in seconds.
   * @param projectedPacketUsage The predicted future usage of the packet.
   * @return The scaled time-to-send in seconds.
   */
  float scaledTimeToSend(std::size_t index, float timeToSend, float projectedPacketUsage) const {
    // Check if valid index
    if (index >= EVENTS_ARRAY_SIZE) {
      llog(ERROR) << "shouldScaleTimeToSend called with out-of-bounds index: "
                  << index << std::endl;
      return timeToSend;
    }

    float scaleFactor = 1.0f;
    if (eventRaisedSet[index]) {
      Criticality level = events[index]->criticality;
      // Locate the CriticalityUsageThreshold for this event's level
      auto criticalityConfig = std::find_if(
          CriticalityDetails.begin(),
          CriticalityDetails.end(),
          [level](const CriticalityUsageThreshold &threshold){
              return threshold.level == level;
          }
      );

      if (criticalityConfig != CriticalityDetails.end()) {
        float minUsageProjection = criticalityConfig->minUsageProjection;
        float maxUsageProjection = criticalityConfig->maxUsageProjection;

        llog(DEBUG) << "Projected packet usage: " << projectedPacketUsage << std::endl;
        llog(DEBUG) << "Criticality level: " << level << std::endl;
        llog(DEBUG) << "Min usage projection: " << minUsageProjection << std::endl;
        llog(DEBUG) << "Max usage projection: " << maxUsageProjection << std::endl;

        if (projectedPacketUsage <= minUsageProjection)
        {
          llog(DEBUG) << "Scaling time to send for event at index " << index << std::endl;
          llog(DEBUG) << "Sending earlier" << std::endl;
          // Should send earlier
          // Scale factor is the ratio of the projected packet usage to the minUsageProjection or the projectedPacketUsage, whichever is smaller
          // For example, if we want to scale below projected usage of 2.0, and the current usage is 1.0, we should scale to 0.5
          // Although most common case is to scale to projectedPacketUsage (As only critical will scale downw if projected usage is over 1.0)
          // projectedPacketUsage / minUsageProjection is always less than 1.0 as here projectedPacketUsage <= minUsageProjection
          scaleFactor = std::min(projectedPacketUsage / minUsageProjection, projectedPacketUsage);
        }
        else if (projectedPacketUsage >= maxUsageProjection)
        {
          // Should send later
          llog(DEBUG) << "Scaling time to send for event at index " << index << std::endl;
          llog(DEBUG) << "Sending later" << std::endl;
          // Scale factor is the ratio of the projected packet usage to the maxUsageProjection or the projectedPacketUsage, whichever is larger
          // For example, if we want to scale above projected usage of 1.5, and the current usage is 3.0, we should scale to 3.0 as 1.5/3.0 = 0.5 < projected of 3.0
          // However if we want to scale above projected usage of 0.5 and the current usage is 1.0, we should scale to 2.0 as 1.0/0.5 = 2 < projected of 0.5
          // projectedPacketUsage / maxUsageProjection is always greater than 1.0 as here projectedPacketUsage >= maxUsageProjection
          scaleFactor = std::max(projectedPacketUsage / maxUsageProjection, projectedPacketUsage);
        }
      } else {
        llog(ERROR) << "No matching CriticalityUsageThreshold found. Using scaleFactor=1.0" << std::endl;
      }
    }
    // Limit the scale factor to the range [MIN_SEND_TIME_SCALE, MAX_SEND_TIME_SCALE] to prevent extreme scaling
    scaleFactor = std::min(std::max(scaleFactor, MIN_SEND_TIME_SCALE), MAX_SEND_TIME_SCALE);

    // Just for logging
    if (scaleFactor == 1.0f) {
      llog(DEBUG) << "Not scaling time to send for event at index " << index << std::endl;
    } else {
      llog(DEBUG) << "Scaling time to send for event at index " << index << " by factor " << scaleFactor << std::endl;
      llog(DEBUG) << "Scaled from " << timeToSend << "s to " << timeToSend * scaleFactor << "s" << std::endl;
    }
    return timeToSend * scaleFactor;
  }

  /**
   * @brief Provides global access to the EventTransmitterBlackboard instance (singleton).
   * @return A reference to the singleton instance of EventTransmitterBlackboard.
   */
  static EventTransmitterBlackboard &getInstance() {
    static EventTransmitterBlackboard instance;
    return instance;
  }

  // Delete copy constructor and assignment operator to enforce singleton
  // pattern
  EventTransmitterBlackboard(const EventTransmitterBlackboard &) = delete;
  EventTransmitterBlackboard& operator=(const EventTransmitterBlackboard &) = delete;

  /**
   * @brief Sets event data with a Python object.
   * 
   * Finds the specified event, optionally updates its Python data, and updates send time.
   * @param eventName The event's name.
   * @param pyObject The Python object holding the event's data (if not None).
   * @param secondsUntilSend The time in seconds until the event is sent.
   */
  void raiseEvent(const std::string &eventName, PyObject *pyObject, float secondsUntilSend) {
    llog(DEBUG) << "Setting event data by name: " << eventName << std::endl;
    // Find the event by name
    auto it =
        std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
          return event && event->eventName == eventName;
        });
    if (it != events.end()) {
      std::size_t index = std::distance(events.begin(), it);
      llog(DEBUG) << "Event found at index: " << index << std::endl;

      // Skip trySetPythonData if None
      if (pyObject && pyObject != Py_None) {
        (*it)->trySetPythonData(pyObject);
      } else {
        llog(DEBUG) << "PyObject is None, skipping data update." << std::endl;
      }
      updateEventTimeIfSooner(index, secondsUntilSend);
    } else {
      llog(WARNING) << "Event not found: " << eventName << std::endl;
    }
  }

  /**
   * @brief Retrieves a Python object from an event by name.
   * 
   * @param eventName The event's name.
   * @return The associated PyObject pointer or nullptr if no event is found.
   */
  PyObject* getPyEventData(const std::string &eventName) {
    llog(DEBUG) << "Getting event data by name: " << eventName << std::endl;
    // Find the event by name
    auto it =
        std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
          return event && event->eventName == eventName;
        });
    if (it != events.end()) {
      llog(DEBUG) << "Event found" << std::endl;
      return (*it)->getPythonData();
    } else {
      llog(WARNING) << "Event not found: " << eventName << std::endl;
      return nullptr;
    }
  }

    /**
   * @brief Sets the unpacked data of an event by name.
   * 
   * Finds the specified event and updates its unpacked data.
   * @tparam T The type of data to store in the event.
   * @param eventName The event's name.
   * @param data The data to set.
   * @throws std::runtime_error If there is a type mismatch when setting event data.
   */
  template <typename T>
  void setEventData(const std::string &eventName, const T &data) {
    llog(DEBUG) << "Setting event data by name: " << eventName << std::endl;
    auto it = std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
      return event && event->eventName == eventName;
    });
    if (it != events.end()) {
      std::size_t index = std::distance(events.begin(), it);
      llog(DEBUG) << "Event found at index: " << index << std::endl;
      UnpackedDataVariant dataVariant = data;
      if (!(*it)->setUnpackedData(dataVariant)) {
        llog(ERROR) << "Type mismatch when setting event data for event: " << eventName << std::endl;
        throw std::runtime_error("Type mismatch when setting event data.");
      }
    } else {
      llog(WARNING) << "Event not found: " << eventName << std::endl;
    }
  }

  /**
   * @brief Gets the unpacked data of an event by name.
   * 
   * Finds the specified event and retrieves its unpacked data.
   * @param eventName The event's name.
   * @return The unpacked data of the event.
   * @throws std::runtime_error If the event is not found.
   */
  UnpackedDataVariant getEventData(const std::string &eventName) const {
    llog(DEBUG) << "Getting event data by name: " << eventName << std::endl;
    auto it = std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
      return event && event->eventName == eventName;
    });
    if (it != events.end()) {
      llog(DEBUG) << "Event found" << std::endl;
      return (*it)->getUnpackedData();
    } else {
      llog(WARNING) << "Event not found: " << eventName << std::endl;
      throw std::runtime_error("Event not found.");
    }
  }

  /**
   * @brief Sets the unpacked data of an event by name using a Python object.
   * 
   * Finds the specified event and updates its unpacked data with the provided Python object.
   * @param eventName The event's name.
   * @param pyObject The Python object holding the event's data.
   */
  void setEventData(const std::string &eventName, PyObject *pyObject) {
    llog(DEBUG) << "Setting event data by name: " << eventName << std::endl;
    auto it = std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
      return event && event->eventName == eventName;
    });
    if (it != events.end()) {
      std::size_t index = std::distance(events.begin(), it);
      llog(DEBUG) << "Event found at index: " << index << std::endl;
      if (pyObject && pyObject != Py_None) {
        (*it)->trySetPythonData(pyObject);
      } else {
        llog(DEBUG) << "PyObject is None, skipping data update." << std::endl;
      }
    } else {
      llog(WARNING) << "Event not found: " << eventName << std::endl;
    }
  }

  /**
   * @brief Gets the unpacked data of an event by name as a Python object.
   * 
   * Finds the specified event and retrieves its unpacked data as a Python object.
   * @param eventName The event's name.
   * @return The Python object holding the event's data.
   * @throws std::runtime_error If the event is not found.
   */
  PyObject* getEventDataPy(const std::string &eventName) const {
    llog(DEBUG) << "Getting event data by name: " << eventName << std::endl;
    auto it = std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
      return event && event->eventName == eventName;
    });
    if (it != events.end()) {
      llog(DEBUG) << "Event found" << std::endl;
      return (*it)->getPythonData();
    } else {
      llog(WARNING) << "Event not found: " << eventName << std::endl;
      throw std::runtime_error("Event not found.");
    }
  }

  /**
   * @brief Checks how long until an event is sent based on event scheduling.
   * 
   * @param eventName The name of the event.
   * @return The remaining seconds until the event is sent, or -1 if not found.
   */
  float getEventTimeToSend(const std::string &eventName) {
    llog(DEBUG) << "Getting event time to send by name: " << eventName << std::endl;
    // Find the event by name
    auto it =
        std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
          return event && event->eventName == eventName;
        });
    if (it != events.end()) {
      std::size_t index = std::distance(events.begin(), it);
      llog(DEBUG) << "Event found at index: " << index << std::endl;
      // Calculate the remaining time to send
      auto now = std::chrono::system_clock::now();
      auto remainingTime = eventRaisedTimes[index] + std::chrono::milliseconds(static_cast<int>(eventTimeToSend[index] * 1000)) - now;
      float remainingSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(remainingTime).count() / 1000.0f;
      return remainingSeconds;
    } else {
      llog(WARNING) << "Event not found: " << eventName << std::endl;
      return -1.0f;
    }
  }

  /**
   * @brief Updates or sets the send time for an event by name.
   * 
   * If found, the event is scheduled for sending sooner if the new time is earlier.
   * @param eventName The event's name.
   * @param secondsUntilSend The new time in seconds until the event is sent.
   */
  void setEventTime(const std::string &eventName, float secondsUntilSend) {
    auto it = std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
        return event && event->eventName == eventName;
    });
    if (it != events.end()) {
        std::size_t index = std::distance(events.begin(), it);
        updateEventTimeIfSooner(index, secondsUntilSend);
    }
  }

  /**
   * @brief Retrieves all valid event names from the blackboard.
   * 
   * @return A vector of valid event names.
   */
  std::vector<std::string> getValidEventNames() const {
    std::vector<std::string> names;
    for (auto* event : events) {
      if (event) {
        names.push_back(event->eventName);
      }
    }
    return names;
  }

  /**
   * @brief Retrieves the absolute time at which an event is scheduled to be sent.
   * 
   * @param eventName The event's name.
   * @return The absolute send time in seconds since epoch, or -1 if not found or not raised.
   */
  float getEventSendTime(const std::string &eventName) const {
    auto it = std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
      return event && event->eventName == eventName;
    });
    if (it != events.end()) {
      std::size_t index = std::distance(events.begin(), it);
      if (eventRaisedSet[index]) {
        auto scheduledTime = eventRaisedTimes[index]
            + std::chrono::milliseconds(static_cast<int>(eventTimeToSend[index] * 1000));
        auto diff = scheduledTime.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::duration<float>>(diff).count();
      }
    }
    return -1.0f;
  }

  /**
   * @brief Retrieves the absolute time at which an event is scheduled to be sent by index.
   * 
   * @param index The index of the event in the array.
   * @return The absolute send time in seconds since epoch, or -1 if not raised or out of range.
   */
  float getEventSendTime(std::size_t index) const {
    if (index < EVENTS_ARRAY_SIZE && eventRaisedSet[index]) {
      auto scheduledTime = eventRaisedTimes[index]
          + std::chrono::milliseconds(static_cast<int>(eventTimeToSend[index] * 1000));
      auto diff = scheduledTime.time_since_epoch();
      return std::chrono::duration_cast<std::chrono::duration<float>>(diff).count();
    }
    return -1.0f;
  }

  /**
   * @brief Updates an event's send time without altering its data.
   * 
   * Finds the event by name and updates its time to be sent sooner if requested.
   * @param eventName The event's name.
   * @param secondsUntilSend The new time in seconds to schedule the event.
   */
  void updateEventSendTime(const std::string &eventName, float secondsUntilSend) {
    llog(DEBUG) << "Updating send time for event: " << eventName << std::endl;
    // Find the event by name
    auto it = std::find_if(events.begin(), events.end(),
                           [&eventName](EventDataPtr event) {
                               return event && event->eventName == eventName;
                           });
    if (it != events.end()) {
        std::size_t index = std::distance(events.begin(), it);
        llog(DEBUG) << "Event found at index: " << index << std::endl;
        updateEventTimeIfSooner(index, secondsUntilSend);
    } else {
        llog(WARNING) << "Event not found: " << eventName << std::endl;
    }
  }

  /**
   * @brief Clears all raised events.
   * 
   * Resets their raised flags, times, and marks them as sent.
   */
  void clearRaisedEvents() {
    llog(DEBUG) << "Clearing raised events" << std::endl;
    for (std::size_t i = 0; i < EVENTS_ARRAY_SIZE; ++i) {
      if (eventRaisedSet[i]) {
        this->clearEvent(i);
      }
    }
  }

  /**
   * @brief Clears specific events by their indexes.
   * 
   * @param indexes An array of indexes referring to events that should be cleared.
   */
  void clearEvents(const std::array<std::size_t, EVENTS_ARRAY_SIZE> &indexes) {
    llog(DEBUG) << "Clearing specific events" << std::endl;
    for (std::size_t index : indexes) {
        this->clearEvent(index);
    }
  }

  /**
   * @brief Clears an event at a given index.
   * 
   * Resets the time and raised state, marking the event as sent.
   * @param index The index of the event in the array.
   */
  void clearEvent(std::size_t index) {
    llog(DEBUG) << "Clearing event at index: " << index << std::endl;
    if (index < EVENTS_ARRAY_SIZE) {
      eventRaisedSet[index] = false;
      eventRaisedTimes[index] = std::chrono::system_clock::time_point();
      eventTimeToSend[index] = 0.0f;
      eventSent[index] = true;
      eventSentTimes[index] = std::chrono::system_clock::now();
      llog(DEBUG) << "Cleared event at index: " << index << std::endl;
    } else {
      llog(WARNING) << "Index out of bounds: " << index << std::endl;
    }
  }

  /**
   * @brief Checks if an event with a given name is raised.
   * 
   * @param eventName The event's name.
   * @return True if the event is raised, false otherwise.
   */
  bool isEventRaised(const std::string &eventName) const {
    // llog(DEBUG) << "Checking if event is raised: " << eventName << std::endl;
    auto it = std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
      return event && event->eventName == eventName;
    });
    if (it != events.end()) {
      std::size_t index = std::distance(events.begin(), it);
      bool raised = eventRaisedSet[index];
      // llog(DEBUG) << "Event at index " << index << " has been raised: " << raised << std::endl;
      return raised;
    } else {
      llog(WARNING) << "Event not found: " << eventName << std::endl;
      return false;
    }
  }

  /**
   * @brief Checks if an event at a given index is raised.
   * 
   * @param index The index of the event in the array.
   * @return True if the event is raised, false otherwise.
   */
  bool isEventRaised(std::size_t index) const {
    // llog(DEBUG) << "Checking if event is raised at index: " << index << std::endl;
    if (index >= EVENTS_ARRAY_SIZE) {
      llog(ERROR) << "isEventRaised called with out-of-bounds index: "
                  << index << std::endl;
      return false;
    }
    bool raised = eventRaisedSet[index];

    // Just for logging
    if (raised) {
      llog(DEBUG) << "Event at index " << index << " has been raised" << std::endl;
    }

    return raised;
  }

  /**
   * @brief Retrieves the time an event was raised, in seconds since epoch.
   * 
   * @param eventName The event's name.
   * @return The time the event was raised, or -1 if not found or not raised.
   */
  float getEventRaisedTime(const std::string &eventName) const {
    auto it = std::find_if(events.begin(), events.end(), [&eventName](EventDataPtr event) {
        return event && event->eventName == eventName;
    });
    if (it != events.end()) {
        std::size_t index = std::distance(events.begin(), it);
        if (eventRaisedSet[index]) {
            auto diff = eventRaisedTimes[index].time_since_epoch();
            return std::chrono::duration_cast<std::chrono::duration<float>>(diff).count();
        }
    }
    return -1.0f;
  }

  /**
   * @brief Selects which events should be placed into a packet based on criticality and size.
   * 
   * Sorts raised events by criticality and age, then caps by available packet size.
   * @return A vector of indexes of events that should be included in a packet.
   */
  std::vector<std::size_t> selectEventsForPacket() {
    std::vector<std::size_t> triggeredIndexes; // TODO: Add events[index]->shouldSend(getEventTimeToSend(index), projectedPacketUsage); here
    for (std::size_t i = 0; i < EVENTS_ARRAY_SIZE; ++i) {
        if (events[i] && eventRaisedSet[i]) {
            triggeredIndexes.push_back(i);
        }
    }
    // Sort by descending criticality, then oldest first
    std::sort(triggeredIndexes.begin(), triggeredIndexes.end(),
        [&](std::size_t a, std::size_t b){
            if (events[a]->criticality != events[b]->criticality) {
                return events[a]->criticality > events[b]->criticality;
            }
            return eventRaisedTimes[a] < eventRaisedTimes[b];
        }
    );

    // Cap total bit size to fit in a packet
    std::size_t totalBits = 0;
    std::vector<std::size_t> selected;
    for (auto idx : triggeredIndexes) {
        std::size_t nextSize = totalBits + events[idx]->bitSize;

        // If the next event fits in the packet, add it
        // Search through all events as some have a bit size of 0
        if ((nextSize <= (MAX_PACKET_EVENT_DATA_SIZE * CHAR_BIT))) {
            selected.push_back(idx);
            totalBits = nextSize;
        }
    }
    return selected;
  }

  /**
   * @brief Clears a list of events by indexes.
   * 
   * @param indexes A vector of indexes referring to events to be cleared.
   */
  void clearEvents(const std::vector<std::size_t> &indexes) {
    for (auto idx : indexes) {
        if (idx < EVENTS_ARRAY_SIZE && eventRaisedSet[idx]) {
            clearEvent(idx);
        }
    }
  }

  /**
   * @brief Retrieves a filtered subset of events by index.
   * 
   * Non-selected events are returned as null pointers in the array.
   * @param selectedIndices The indices of events to include.
   * @return An array of event pointers, populated only for selected indices.
   */
  std::array<EventDataPtr, EVENTS_ARRAY_SIZE> 
  getFilteredEvents(const std::vector<std::size_t> &selectedIndices) const {
      std::array<EventDataPtr, EVENTS_ARRAY_SIZE> filtered{};
      filtered.fill(nullptr);
      for (auto i : selectedIndices) {
          if (i < EVENTS_ARRAY_SIZE && events[i]) {
              filtered[i] = events[i];
          }
      }
      return filtered;
  }

private:
  /**
   * @brief Updates the send time of an event if the specified new time is sooner.
   * 
   * This is a private helper function that checks the current scheduled time and replaces it if earlier.
   * @param index The index of the event.
   * @param seconds_until_send The new time in seconds until the event should be sent.
   */
  inline void updateEventTimeIfSooner(std::size_t index, float seconds_until_send) {
    auto now = std::chrono::system_clock::now();
    auto new_send_time = now + std::chrono::milliseconds(static_cast<int>(seconds_until_send * 1000));
    if (!eventRaisedSet[index] ||
        new_send_time < (eventRaisedTimes[index]
            + std::chrono::milliseconds(static_cast<int>(eventTimeToSend[index] * 1000)))) {
        eventRaisedTimes[index] = now;
        eventTimeToSend[index] = seconds_until_send;
        eventRaisedSet[index] = true;
    }
  }
};
