#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <typeinfo>

#ifdef __cplusplus
#include <boost/python/object.hpp>
#include <ostream>
#include "types/events/EventDataTypes.hpp"
#endif

#include "utils/EventDefinitions.hpp"

/**
 * @class AbstractEventData
 * @brief Abstract base class for all event data types.
 *
 * Defines the interface for event data serialization, deserialization, and data handling.
 */
class AbstractEventData {
public:
  // Event name for identification
  std::string eventName;

  // Size of the event data in bits
  unsigned short int bitSize;

  // Criticality of the event
  Criticality criticality;

  /**
   * @brief Constructs a new AbstractEventData object.
   *
   * Initializes member variables to default values.
   */
  AbstractEventData()
      : eventName(""), bitSize(0), criticality(DEFAULT_CRITICALITY) {}

  /**
   * @brief Virtual destructor for AbstractEventData.
   */
  virtual ~AbstractEventData() = default;

  /**
   * @brief Packs the unpacked data into the packed format.
   *
   * Must be implemented by derived classes.
   */
  virtual void packData() = 0;

  /**
   * @brief Unpacks the packed data into the unpacked format.
   *
   * Must be implemented by derived classes.
   */
  virtual void unpackData() = 0;

  /**
   * @brief Serializes the packed data into a byte buffer.
   *
   * @param buffer The buffer to serialize data into.
   * @param bitOffset The current bit offset in the buffer.
   */
  virtual void serializeToBuffer(std::vector<uint8_t> &buffer,
                                 size_t &bitOffset) = 0;

  /**
   * @brief Deserializes data from a byte buffer into packed format.
   *
   * @param buffer The source buffer containing serialized data.
   * @param bitOffset The current bit offset in the buffer.
   */
  virtual void deserializeFromBuffer(const std::vector<uint8_t> &buffer,
                                     size_t &bitOffset) = 0;

  /**
   * @brief Determines whether the event should be sent based on timing and packet rate.
   *
   * @param timeToSend The scheduled time for sending the event.
   * @param projectedPacketUsage The rate at which packets are being sent.
   * @return true If the event should be sent.
   * @return false Otherwise.
   */
  virtual bool shouldSend(float time_to_send, float packet_rate) {
    llog(DEBUG) << "shouldSend() called in AbstractEventData." << std::endl;
    return true;
  }

  /**
   * @brief Gets the packed data.
   *
   * @return PackedDataVariant The packed data.
   */
  virtual PackedDataVariant getPackedData() const = 0;

  /**
   * @brief Gets the unpacked data.
   *
   * @return UnpackedDataVariant The unpacked data.
   */
  virtual UnpackedDataVariant getUnpackedData() const = 0;

  /**
   * @brief Sets the packed data.
   *
   * @param data The packed data to set.
   * @return true If the data is set successfully.
   * @return false Otherwise.
   */
  virtual bool setPackedData(const PackedDataVariant &data) = 0;

  /**
   * @brief Sets the unpacked data.
   *
   * @param data The unpacked data to set.
   * @return true If the data is set successfully.
   * @return false Otherwise.
   */
  virtual bool setUnpackedData(const UnpackedDataVariant &data) = 0;

  /**
   * @brief Retrieves the type information of the packed data.
   *
   * @return const std::type_info& Type information.
   */
  virtual const std::type_info& getPackedDataType() const = 0;

  /**
   * @brief Retrieves the type information of the unpacked data.
   *
   * @return const std::type_info& Type information.
   */
  virtual const std::type_info& getUnpackedDataType() const = 0;

  /**
   * @brief Gets the size of the packed data in bytes.
   *
   * @return std::size_t Size of packed data.
   */
  virtual std::size_t getPackedDataSize() const = 0;

  /**
   * @brief Optionally, add a virtual function to get bit size.
   *
   * @return unsigned short int Size of the event data in bits.
   */
  virtual unsigned short int getBitSize() const { return bitSize; }

#ifdef __cplusplus
  /**
   * @brief Converts a Python object to unpacked data.
   *
   * @param obj The Python object to convert.
   * @return true If conversion is successful.
   * @return false Otherwise.
   */
  virtual bool fromPythonObject(const boost::python::object &obj) = 0;

  /**
   * @brief Converts unpacked data to a Python object.
   *
   * @return boost::python::object The resulting Python object.
   */
  virtual boost::python::object toPythonObject() const = 0;

  /**
   * @brief Sets unpacked data from a Python object.
   *
   * @param obj The Python object containing unpacked data.
   */
  virtual void setUnpackedDataAsPython(const boost::python::object &obj) = 0;

  /**
   * @brief Retrieves unpacked data as a Python object.
   *
   * @return boost::python::object The resulting Python object.
   */
  virtual boost::python::object getUnpackedDataAsPython() const = 0;

  /**
   * @brief Tries to set Python data.
   *
   * @param pyObj The Python object to set unpacked data from.
   * @return true If the data is set successfully.
   * @return false Otherwise.
   */
  virtual bool trySetPythonData(PyObject* pyObj) = 0;

  /**
   * @brief Gets Python instance of unpacked data from the event.
   *
   * @return PyObject* The Python data.
   */
  virtual PyObject* getPythonData() = 0;

  /**
   * @brief Pure virtual method to print event data to an output stream.
   *
   * Must be implemented by derived classes to define how their data is streamed.
   *
   * @param os The output stream to print to.
   */
  virtual void print(std::ostream& os) const = 0;

  /**
   * @brief Generic getter for your unpacked data in a type-safe way.
   *
   * @param defaultVal The default value to return if unpacked data is not available.
   * @return T The unpacked data.
   */
  template <typename T>
  T getUnpackedValue(const T &defaultVal = T()) const {
      return getValueOr<T>(getUnpackedData(), defaultVal);
  }

  /**
   * @brief Generic getter for your packed data in a type-safe way.
   *
   * @param defaultVal The default value to return if packed data is not available.
   * @return T The packed data.
   */
  template <typename T>
  T getPackedValue(const T &defaultVal = T()) const {
      return getValueOr<T>(getPackedData(), defaultVal);
  }
#endif

protected:
  /**
   * @brief Helper method to safely increment Python object reference.
   *
   * @param obj The Python object.
   */
  void incref(PyObject* obj) {
    if (obj) Py_INCREF(obj);
  }

  /**
   * @brief Helper method to safely decrement Python object reference.
   *
   * @param obj The Python object.
   */
  void decref(PyObject* obj) {
    if (obj) Py_DECREF(obj);
  }
};

#ifdef __cplusplus
/**
 * @brief Overloads the << operator to stream AbstractEventData objects.
 *
 * Calls the virtual print method of the AbstractEventData instance.
 *
 * @param os The output stream.
 * @param event The AbstractEventData object to stream.
 * @return std::ostream& The output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const AbstractEventData& event) {
    event.print(os);
    return os;
}
#endif
