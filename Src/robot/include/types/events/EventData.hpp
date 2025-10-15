#pragma once

#include <cstring>
#include <limits.h>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "AbstractEventData.hpp"
#include "utils/EventDefinitions.hpp"

#ifdef __cplusplus
#include <boost/python/extract.hpp>
#endif

/**
 * @class Event Data
 * @brief Class to hold the data for an event to be sent or data received from
 * an event
 *
 * This class is the base class for all event types that will override the
 * packData and unpackData functions as well as the shouldSend function if
 * custom logic is required for packing, unpacking, or determining if the event
 * should be sent, i.e if we don't care about the number of packets we have left
 * or if we should only send with high priority if we are running low on packets
 */
template <typename PackedDataType, typename UnpackedDataType>
class EventData : public AbstractEventData {
public:
  // Constructor with criticality and event name
  EventData(Criticality criticality = DEFAULT_CRITICALITY,
            std::string eventName = "")
      : AbstractEventData() {
    this->criticality = criticality;
    this->eventName = eventName;
    this->packed_data = PackedDataType();
    this->unpacked_data = UnpackedDataType();
    // Initialize bitSize
    if constexpr (std::is_same<PackedDataType, bool>::value) {
      this->bitSize = 1;
    } else if constexpr (std::is_same<PackedDataType, void>::value) {
      this->bitSize = 0;
    } else {
      this->bitSize = sizeof(PackedDataType) * CHAR_BIT;
      llog(DEBUG) << "Bit size set to " << this->bitSize << std::endl;
      llog(DEBUG) << "Bit size type: " << typeid(this->bitSize).name()
                 << std::endl;
    }
  }
  // Destructor
  virtual ~EventData() = default;

  /**
   * @brief Override getPackedData from AbstractEventData
   * @return PackedDataVariant containing the packed data
   */
  virtual PackedDataVariant getPackedData() const override {
    return packed_data; // Will ensure PackedDataVariant can hold PackedDataType
  }

  /**
   * @brief Override getUnpackedData from AbstractEventData
   * @return UnpackedDataVariant containing the unpacked data
   */
  virtual UnpackedDataVariant getUnpackedData() const override {
    return unpacked_data; // Will ensure UnpackedDataVariant can hold UnpackedDataType
  }

  /**
   * @brief Implement getPackedDataType
   * @return const std::type_info& of the packed data type
   */
  const std::type_info &getPackedDataType() const override {
    return typeid(PackedDataType);
  }

  /**
   * @brief Implement getUnpackedDataType
   * @return const std::type_info& of the unpacked data type
   */
  const std::type_info &getUnpackedDataType() const override {
    return typeid(UnpackedDataType);
  }

  /**
   * @brief Implement getPackedDataSize
   * @return std::size_t size of the packed data
   */
  std::size_t getPackedDataSize() const override {
    return sizeof(PackedDataType);
  }

  /**
   * @brief Pure virtual function to pack the data to be sent
   */
  virtual void packData() override = 0;

  /**
   * @brief Pure virtual function to unpack the data received
   */
  virtual void unpackData() override = 0;

  /**
   * @brief Function to overload the "should be sent" evaluation in the transmitter if
   * custom logic is required for evaluating if the event should be sent
   * @param timeToSend Time to send the event in seconds
   * @param projectedPacketUsage Projected packet usage (projected proportion of ideal 
   *                              packet usage expected)
   * @return true if the event should be sent, false otherwise
   */
  virtual bool shouldSend(float timeToSend, float projectedPacketUsage) override {
    llog(INFO) << "shouldSend() called in EventData." << std::endl;
    return true;
  }

  /**
   * @brief Override serializeToBuffer
   * @param buffer Buffer to serialize data into
   * @param bitOffset Bit offset in the buffer
   */
  void serializeToBuffer(std::vector<uint8_t> &buffer,
                         size_t &bitOffset) override {
    if constexpr (!std::is_same<PackedDataType, void>::value) {
      packData();
      serializeData(packed_data, buffer, bitOffset);
    }
    // If PackedDataType is void, do nothing (no data to serialize)
  }

  /**
   * @brief Override deserializeFromBuffer
   * @param buffer Buffer to deserialize data from
   * @param bitOffset Bit offset in the buffer
   */
  void deserializeFromBuffer(const std::vector<uint8_t> &buffer,
                             size_t &bitOffset) override {
    if constexpr (!std::is_same<PackedDataType, void>::value) {
      deserializeData(buffer, bitOffset, packed_data);
      unpackData();
    }
    // If PackedDataType is void, do nothing (no data to deserialize)
  }

#ifdef __cplusplus
  /**
   * @brief Provide default Python conversion: handle void types and try to extract
   * 'UnpackedDataType'
   * @param obj Python object to convert from
   * @return true if conversion is successful, false otherwise
   */
  bool fromPythonObject(const boost::python::object &obj) override {
    if constexpr (!std::is_same<UnpackedDataType, void>::value) {
      boost::python::extract<UnpackedDataType> conv(obj);
      if (conv.check()) {
        unpacked_data = conv();
        packData(); // pack immediately
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Convert to Python object
   * @return boost::python::object containing the converted data
   */
  boost::python::object toPythonObject() const override {
    if constexpr (!std::is_same<UnpackedDataType, void>::value) {
      return boost::python::object(unpacked_data);
    }
    return boost::python::object(); // Return None for void data
  }

  /**
   * @brief Set unpacked data from Python object
   * @param obj Python object to set from
   */
  void setUnpackedDataAsPython(const boost::python::object &obj) override {
    if constexpr (!std::is_same<UnpackedDataType, void>::value) {
      fromPythonObject(obj);
    }
    // If UnpackedDataType is void, do nothing (no data to set)
  }

  /**
   * @brief Get unpacked data as Python object
   * @return boost::python::object containing the unpacked data
   */
  boost::python::object getUnpackedDataAsPython() const override {
    if constexpr (!std::is_same<UnpackedDataType, void>::value) {
      return toPythonObject();
    }
    return boost::python::object(); // Return None for void data
  }

  /**
   * @brief Try to set Python data
   * @param pyObj Python object to set from
   * @return true if successful, false otherwise
   */
  bool trySetPythonData(PyObject *pyObj) override {
    if constexpr (!std::is_same<UnpackedDataType, void>::value) {
      // Skip data update if None
      if (!pyObj || pyObj == Py_None) {
        llog(INFO) << "Python object is None, skipping data update."
                   << std::endl;
        return true;
      }

      boost::python::object obj(
          boost::python::handle<>(boost::python::borrowed(pyObj)));
      boost::python::extract<UnpackedDataType> converter(obj);

      if (converter.check()) {
        unpacked_data = converter();
        packData(); // Ensure packed data is updated
        return true;
      }
      llog(ERROR) << "Failed to convert Python object to UnpackedDataType."
                  << std::endl;
      return false;
    }
    return true; // For void, no data to set so return true
  }

  /**
   * @brief Get Python data
   * @return PyObject* containing the Python data
   */
  PyObject *getPythonData() override {
    if constexpr (!std::is_same<UnpackedDataType, void>::value) {
      try {
        if (this->bitSize == 0) {
          // Return None for empty data
          Py_RETURN_NONE;
        }
        boost::python::object obj(unpacked_data);
        PyObject *result = obj.ptr();
        incref(result); // Increment reference for caller return
        return result;
      } catch (...) {
        Py_RETURN_NONE;
      }
    }
    Py_RETURN_NONE; // For void, return None (no data to return)
  }

  /**
   * @brief Overrides the print method to stream packed and unpacked data.
   *
   * @param os The output stream.
   */
  virtual void print(std::ostream& os) const override {
      os << "Packed Data: " << packed_data << ", Unpacked Data: " << unpacked_data;
  }
#endif

  /**
   * @brief Set packed data
   * @param data PackedDataVariant containing the data to set
   * @return true if successful, false otherwise
   */
  virtual bool setPackedData(const PackedDataVariant &data) override {
    // Attempt to assign the data to packed_data
    llog(INFO) << "Setting packed data." << std::endl;
    if (auto ptr = boost::get<PackedDataType>(&data)) {
      packed_data = *ptr;
      // Use builtin print function to log packed data
      llog(INFO) << "Packed data: " << packed_data << std::endl;
      return true;
    }
    return false;
  }

  /**
   * @brief Set unpacked data
   * @param data UnpackedDataVariant containing the data to set
   * @return true if successful, false otherwise
   */
  virtual bool setUnpackedData(const UnpackedDataVariant &data) override {
    // Attempt to assign the data to unpacked_data
    if (auto ptr = boost::get<UnpackedDataType>(&data)) {
      unpacked_data = *ptr;
      // Use builtin print function to log unpacked data
      llog(INFO) << "Unpacked data: " << unpacked_data << std::endl;
      return true;
    }
    return false;
  }

protected:
  PackedDataType packed_data;     // Holds the processed data to be buffered and
                                  // sent in the packet
  UnpackedDataType unpacked_data; // Holds the data to be processed/serialized
                                  // to be sent/received

  /**
   * @brief Default serialization helper for common types
   * @param data Data to serialize
   * @param buffer Buffer to serialize data into
   * @param bitOffset Bit offset in the buffer
   */
  template <typename T>
  typename std::enable_if<std::is_integral<T>::value ||
                              std::is_floating_point<T>::value,
                          void>::type
  serializeData(const T &data, std::vector<uint8_t> &buffer,
                size_t &bitOffset) const {
    llog(INFO) << "Default primative serializeData() called." << std::endl;

    // Validate buffer size
    if (buffer.size() < bitOffset / CHAR_BIT + sizeof(T)) {
      llog(INFO) << "Buffer size insufficient, resizing." << std::endl;
      buffer.resize(bitOffset / CHAR_BIT + sizeof(T), 0);
    }

    // Serialize data
    std::memcpy(&buffer[bitOffset / CHAR_BIT], &data, sizeof(T));
    bitOffset += sizeof(T) * CHAR_BIT;

    llog(INFO) << "Serialized data (in bits): " << std::bitset<sizeof(T) * CHAR_BIT>(data) << std::endl;

    llog(INFO) << "Default primative serializeData() completed." << std::endl;
  }

  /**
   * @brief Default deserialization helper for common types
   * @param buffer Buffer to deserialize data from
   * @param bitOffset Bit offset in the buffer
   * @param data Data to deserialize into
   */
  template <typename T>
  typename std::enable_if<std::is_integral<T>::value ||
                              std::is_floating_point<T>::value,
                          void>::type
  deserializeData(const std::vector<uint8_t> &buffer, size_t &bitOffset,
                  T &data) {
    llog(INFO) << "Default primative deserializeData() called." << std::endl;

    // Check buffer bounds
    if (bitOffset / CHAR_BIT + sizeof(T) > buffer.size()) {
      llog(ERROR) << "deserializeData() failed: Buffer underflow." << std::endl;
      throw std::runtime_error("Buffer underflow during deserialization.");
    }

    // Deserialize data
    std::memcpy(&data, &buffer[bitOffset / CHAR_BIT], sizeof(T));
    bitOffset += sizeof(T) * CHAR_BIT;

    llog(INFO) << "Deserialized data (in bits): " << std::bitset<sizeof(T) * CHAR_BIT>(data) << std::endl;

    llog(INFO) << "Default primative deserializeData() completed successfully." << std::endl;
  }

  /**
   * @brief Overload for std::array serialization
   * @param data Data to serialize
   * @param buffer Buffer to serialize data into
   * @param bitOffset Bit offset in the buffer
   */
  template <typename T, size_t N>
  void serializeData(const std::array<T, N> &data, std::vector<uint8_t> &buffer,
                     size_t &bitOffset) const {
    for (const auto &elem : data) {
      serializeData(elem, buffer, bitOffset);
    }
  }

  /**
   * @brief Overload for std::array deserialization
   * @param buffer Buffer to deserialize data from
   * @param bitOffset Bit offset in the buffer
   * @param data Data to deserialize into
   */
  template <typename T, size_t N>
  void deserializeData(const std::vector<uint8_t> &buffer, size_t &bitOffset,
                       std::array<T, N> &data) {
    for (auto &elem : data) {
      deserializeData(buffer, bitOffset, elem);
    }
  }

  /**
   * @brief Overload for std::string serialization
   * @param data Data to serialize
   * @param buffer Buffer to serialize data into
   * @param bitOffset Bit offset in the buffer
   */
  void serializeData(const std::string &data, std::vector<uint8_t> &buffer,
                     size_t &bitOffset) const {
    uint32_t length = static_cast<uint32_t>(data.size());
    serializeData(length, buffer, bitOffset);
    for (char c : data) {
      serializeData(c, buffer, bitOffset);
    }
  }

  /**
   * @brief Overload for std::string deserialization
   * @param buffer Buffer to deserialize data from
   * @param bitOffset Bit offset in the buffer
   * @param data Data to deserialize into
   */
  void deserializeData(const std::vector<uint8_t> &buffer, size_t &bitOffset,
                       std::string &data) {
    uint32_t length = 0;
    deserializeData(buffer, bitOffset, length);
    data.resize(length);
    for (uint32_t i = 0; i < length; ++i) {
      char c = 0;
      deserializeData(buffer, bitOffset, c);
      data[i] = c;
    }
  }
};
