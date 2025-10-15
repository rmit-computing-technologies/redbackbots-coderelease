/**
 * @file RelativeFieldColorsProvider.h
 *
 * This file declares a module that approximates the current image's average
 * luminance and saturation for discerning field and white.
 *
 * @author Lukas Malte Monnerjahn
 */

#pragma once

#include "blackboard/Blackboard.hpp"
#include "perception/vision/Detector.hpp"

#include "perception/vision/configuration/RelativeFieldColorsParameters.hpp"

#include "types/Image.hpp"
#include "types/vision/ECImage.hpp"
#include "types/vision/ScanGrid.hpp"
#include "types/vision/RelativeFieldColors.hpp"

class RelativeFieldColorsMiddleInfo : public Detector {
public:
    RelativeFieldColorsMiddleInfo(Blackboard* blackboard);
    virtual ~RelativeFieldColorsMiddleInfo();

    // Resets middle info
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
    virtual void detect_(const VisionInfoIn* info_in, 
                        VisionInfoMiddle* info_middle,
                        VisionInfoOut* info_out);

private:
    /**
    * Run detection for the given camera
    **/   
    void detect_(CameraInfo::Camera whichCamera,
                const VisionInfoIn* info_in, 
                VisionInfoMiddle* info_middle,
                VisionInfoOut* info_out);

    void configure(Blackboard* blackboard);

    /**
    * Approximates an average value over the image.
    * @param image The image.
    * @return The approximated average.
    */
    [[nodiscard]] float approximateAverage(const Image<PixelTypes::GrayscaledPixel>& image, ScanGrid& scanGrid) const;
    
};