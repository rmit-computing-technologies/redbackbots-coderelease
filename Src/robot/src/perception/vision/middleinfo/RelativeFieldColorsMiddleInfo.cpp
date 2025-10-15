/**
* @file RelativeFieldColorsProvider.cpp
*
* This file implements a module that approximates the current image's average
* luminance and saturation for discerning field and white.
*
* @author Lukas Malte Monnerjahn
* @author RedBackBots
*/

#include "perception/vision/middleinfo/RelativeFieldColorsMiddleInfo.hpp"
#include "perception/vision/VisionInfoMiddle.hpp"

#include <list>

RelativeFieldColorsMiddleInfo::RelativeFieldColorsMiddleInfo(Blackboard* blackboard):
  Detector("RelativeFieldColorsMiddleInfo")
{
  configure(blackboard);

  llog(INFO) << NDEBUG_LOGSYMB << "RelativeFieldColorsMiddleInfo loaded" << std::endl;
}


RelativeFieldColorsMiddleInfo::~RelativeFieldColorsMiddleInfo() {

}

void RelativeFieldColorsMiddleInfo::configure(Blackboard* blackboard) {

}

void RelativeFieldColorsMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
  // TODO - reset the middle info for RelativeFieldColorsMiddleInfo
}

void RelativeFieldColorsMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {

}


void RelativeFieldColorsMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    // Detect with top camera
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // Detect with bottom camera
    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void RelativeFieldColorsMiddleInfo::detect_(CameraInfo::Camera whichCamera, 
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
  
    ECImage& ecImage = info_middle->ecImage[whichCamera];
    ScanGrid& scanGrid = info_middle->scanGrid[whichCamera];
    
    RelativeFieldColors& relativeFieldColors = info_middle->relativeFieldColors[whichCamera];
    RelativeFieldColorsParameters relativeFieldColorsParameters; // TODO: Ask Timothy Wiley if this is correct

    relativeFieldColors.rfcParameters = relativeFieldColorsParameters;
    relativeFieldColors.averageLuminanceF = approximateAverage(ecImage.grayscaled, scanGrid);
    relativeFieldColors.averageSaturationF = approximateAverage(ecImage.saturated, scanGrid);
    relativeFieldColors.averageLuminance = static_cast<PixelTypes::GrayscaledPixel>(relativeFieldColors.averageLuminanceF);
    relativeFieldColors.averageSaturation = static_cast<PixelTypes::GrayscaledPixel>(relativeFieldColors.averageSaturationF);
}

float RelativeFieldColorsMiddleInfo::approximateAverage(const Image<PixelTypes::GrayscaledPixel>& image, ScanGrid& scanGrid) const
{
    std::list<unsigned int> yList;
    if(scanGrid.lowResHorizontalLines.empty())
    {
        yList.emplace_back(image.height * 3 / 8);
        yList.emplace_back(image.height / 4);
        yList.emplace_back(image.height / 8);
    }
    else
    {
        yList.emplace_back(scanGrid.lowResHorizontalLines[scanGrid.lowResHorizontalLines.size() / 4 * 3].y);
        yList.emplace_back(scanGrid.lowResHorizontalLines[scanGrid.lowResHorizontalLines.size() / 2].y);
        yList.emplace_back(scanGrid.lowResHorizontalLines[scanGrid.lowResHorizontalLines.size() / 4].y);
    }
    int sum = 0;
    int step = 8;
    unsigned int samples = image.width / step;
    for(const auto y : yList) {
        for(unsigned int x = 0; x < samples; ++x) {
            sum += image[y][step * x + 2];
        }
    }
    return static_cast<float>(sum) / (static_cast<float>(yList.size()) * static_cast<float>(samples));
}
