/**
 * @file SerialiseImage.hpp
 * 
 * Friend class for serialising Image types
 * 
 * @author RedbackBOts
 * 
 */
#pragma once

#include "types/Image.hpp"

// Generated file from Protobuf
#include "Blackboard.pb.h"

class SerialiseImage {
public:
    template <typename T>
    static void serialise(const Image<T> &cpp, offnao::Vision_CameraImage &pb);

    template <typename T>
    static void deserialise(Image<T> &cpp, const offnao::Vision_CameraImage &pb);
};