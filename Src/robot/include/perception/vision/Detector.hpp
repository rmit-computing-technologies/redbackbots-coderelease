#pragma once

#include "perception/vision/VisionInfoIn.hpp"
#include "perception/vision/VisionInfoOut.hpp"
#include <utils/Timer.hpp>

#include <string>

// Forward Declare vision info middle
class VisionInfoMiddle;

class Detector {
public:
    Detector(std::string name) :name(name) {};
    virtual ~Detector() {};

    /**
     * detect abstract function intended for implementation in a subclass
     */
    virtual void detect(const VisionInfoIn* info_in, 
                        VisionInfoMiddle* info_middle,
                        VisionInfoOut* info_out) {
        timer.restart();
        detect_(info_in, info_middle, info_out);
    }

    /**
     * Reset the middle info constructs populated by this detector.
     * ONLY the elements populated by this detector are reset
     */
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle) = 0;

    /**
     * Reset the vision out constructs populated by this detector.
     * ONLY the elements populated by this detector are reset
     */
    virtual void resetVisionOut(VisionInfoOut* info_out) = 0;

    // Timer for this detector
    Timer timer;

    // Human readable string of this detector, useful for logging
    std::string name;

protected:
    /**
     * detect abstract function intended for implementation in a subclass
     */
    virtual void detect_(const VisionInfoIn* info_in, 
                        VisionInfoMiddle* info_middle,
                        VisionInfoOut* info_out) = 0;    
};
