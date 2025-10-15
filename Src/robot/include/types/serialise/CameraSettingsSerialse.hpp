/**
 * @file CameraSettingsSerialse.hpp
 * 
 * For serialisation, this wraps a camera settings with the
 * associated camera (top/bot).
 * This class is protobuf serialisable to directly convert
 * this from the types into the output stream.
 * 
 * Primarily used for sending new camera settings from Offnao
 * to the robot in Camera Tab.
 * 
 * @author RedbackBots
 * 
 */

#pragma once

#include <types/camera/CameraInfo.hpp>
#include <types/camera/CameraSettings.hpp>
#include <communication/serialisation/ProtobufSerialisable.hpp>

class CameraSettingsSerialse : public ProtobufSerialisable {
public:
    CameraInfo::Camera whichCamera;
    CameraSettings settings;

    CameraSettingsSerialse() = default;
    virtual ~CameraSettingsSerialse() {};

    // Serialise implementations
    virtual void serialise(std::ostream& os) const;
    virtual void deserialise(std::istream& is);
};
