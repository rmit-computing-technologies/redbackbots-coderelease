/**
 * @file PenaltyMarkRegionsMiddleInfo.cpp
 *
 * This file implements a module that determines candidate regions for
 * penalty marks in the image. It provides regions in which should be
 * searched for the center as well as the regions that must be provided
 * in the CNS image for the search to work.
 *
 * @author Thomas Röfer
 * @author RedbackBots
 */

#include "perception/vision/middleinfo/PenaltyMarkRegionsMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "utils/math/IISC.hpp"
#include "types/PixelTypes.hpp"
#include "utils/debug/Assert.hpp"

PenaltyMarkRegionsMiddleInfo::PenaltyMarkRegionsMiddleInfo(Blackboard* blackboard):
  Detector("PenaltyMarkRegionsMiddleInfo")
{
    configure(blackboard);

    llog(INFO) << NDEBUG_LOGSYMB << "PenaltyMarkRegionsMiddleInfo loaded" << std::endl;
}

PenaltyMarkRegionsMiddleInfo::~PenaltyMarkRegionsMiddleInfo() {

}

void PenaltyMarkRegionsMiddleInfo::configure(Blackboard* blackboard) {

}

void PenaltyMarkRegionsMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    info_middle->penaltyMarkRegions[CameraInfo::Camera::top].regions.clear();
    // info_middle->penaltyMarkRegions[CameraInfo::Camera::bot].regions.clear();
}

void PenaltyMarkRegionsMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void PenaltyMarkRegionsMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "tick PenaltyMarkRegions" << std::endl;

    // Run top camera
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // (MO) Penalty mark regions only for top camera, BOPPerceptor runs on lower camera
    // detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void PenaltyMarkRegionsMiddleInfo::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {

    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    ScanGrid& scanGrid = info_middle->scanGrid[whichCamera];
    ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped = info_middle->colorScanLineRegionsVerticalClipped[whichCamera];

    PenaltyMarkRegions& penaltyMarkRegions = info_middle->penaltyMarkRegions[whichCamera];
    regions.reserve(1000);

    if(!scanGrid.isValid())
        return;

    // if(Transformation::robotWithCameraRotationToImage(Vector2f(maxDistanceOnField, 0), theCameraMatrix, theCameraInfo, pointInImage) 
    // && theColorScanLineRegionsVerticalClipped.scanLines.size() > theColorScanLineRegionsVerticalClipped.lowResStart
    // + theColorScanLineRegionsVerticalClipped.lowResStep)
    Vector2f pointInImage = info_in->kinematicPose.headRelRobotToImageXY(Vector2f(maxDistanceOnField, 0), cameraInfo).cast<float>();
    if (!(pointInImage.x() < 0) && colorScanLineRegionsVerticalClipped.scanLines.size() > colorScanLineRegionsVerticalClipped.lowResStart + colorScanLineRegionsVerticalClipped.lowResStep)
    {
        int xStep = colorScanLineRegionsVerticalClipped.scanLines[colorScanLineRegionsVerticalClipped.lowResStart + colorScanLineRegionsVerticalClipped.lowResStep].x
                    - colorScanLineRegionsVerticalClipped.scanLines[colorScanLineRegionsVerticalClipped.lowResStart].x;

        unsigned short upperBound = static_cast<unsigned short>(std::max(0.f, pointInImage.y()));
        if(upperBound < scanGrid.fullResY[0])
        {
            initTables(upperBound, cameraInfo, scanGrid);
            if(initRegions(upperBound, colorScanLineRegionsVerticalClipped))
            {
                //llog(DEBUG) << "-init regions-" << std::endl;
                unionFind(xStep);
                analyseRegions(upperBound, xStep, penaltyMarkRegions.regions, info_in, cameraInfo, scanGrid);
            }
        }
    }
    // llog(DEBUG) << "Num regions upper: " << penaltyMarkRegions.regions.size()  << std::endl;
    // for(const Boundaryi region : penaltyMarkRegions.regions) {
    //   llog(DEBUG) << "    - xmin: " << region.x.min << std::endl;
    //   llog(DEBUG) << "    - xmax: " << region.x.max << std::endl;
    //   llog(DEBUG) << "    - ymin: " << region.y.min << std::endl;
    //   llog(DEBUG) << "    - ymax: " << region.y.max << std::endl;
    // }
}

void PenaltyMarkRegionsMiddleInfo::initTables(unsigned short upperBound, const CameraInfo& cameraInfo, ScanGrid& scanGrid)
{
  extendedLower.clear();
  extendedLower.resize(cameraInfo.height + 1);

  int y = scanGrid.fullResY[0] + 1;
  int yGrid;
  for(size_t i = 1; i < scanGrid.fullResY.size() && (yGrid = scanGrid.fullResY[i]) >= upperBound; ++i)
  {
    int extension = (scanGrid.fullResY[i - 1] - yGrid) * regionExtensionFactor;
    while(y >= yGrid)
    {
      extendedLower[y] = static_cast<unsigned short>(y + extension);
      --y;
    }
  }
}

bool PenaltyMarkRegionsMiddleInfo::initRegions(unsigned short upperBound, ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped)
{
  regions.clear();
  for(size_t i = colorScanLineRegionsVerticalClipped.lowResStart; i < colorScanLineRegionsVerticalClipped.scanLines.size(); i += colorScanLineRegionsVerticalClipped.lowResStep)
  {
    auto& scanLine = colorScanLineRegionsVerticalClipped.scanLines[i];
    bool first = true;
    for(auto& region : scanLine.regions)
    {
      if(region.color != ScanLineRegion::field && region.range.lower > upperBound)
      {
        if(regions.empty() || regions.back().left != scanLine.x || regions.back().upper != region.range.lower)
        {
          if(regions.size() == regions.capacity()) 
          {
            return false;
          }
          regions.emplace_back(std::max(region.range.upper, upperBound), region.range.lower, scanLine.x, region.color == ScanLineRegion::white);
        }
        else
        {
          Region& r = regions.back();
          unsigned short upper = std::max(region.range.upper, upperBound);
          r.upper = upper;
          r.pixels += region.range.lower - upper;
          if(region.color == ScanLineRegion::white) 
          {
            r.whitePixels += region.range.lower - upper;
          }
        }
        if(first || region.range.upper <= upperBound) {
          regions.back().pixels = 0x8000; // force ignoring any region containing this one.
        }
      }
      first = false;
    }
  }
  return true;
}

void PenaltyMarkRegionsMiddleInfo::unionFind(int xStep)
{
  auto i = regions.begin();
  auto j = regions.begin();
  while(i != regions.end())
  {
    if(j->left + xStep == i->left && extendedLower[j->lower] > i->upper && j->upper < extendedLower[i->lower]) {
      i->getRoot()->parent = j->getRoot();
    }

    if(j->left + xStep < i->left || (j->left + xStep == i->left && j->upper > i->upper)) {
      ++j;
    }
    else {
      ++i;
    }
  }
}

void PenaltyMarkRegionsMiddleInfo::analyseRegions(unsigned short upperBound, int xStep, std::vector<Boundaryi>& searchRegions, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ScanGrid& scanGrid)
{
  std::vector<Region*> mergedRegions;
  mergedRegions.reserve(100);
  for(Region& region : regions)
    if(region.parent == &region) {
      mergedRegions.emplace_back(&region);
    }
    else {
      Region* root = region.getRoot();
      root->upper = std::min(root->upper, region.upper);
      root->lower = std::max(root->lower, region.lower);
      root->left = std::min(root->left, region.left);
      root->right = std::max(root->right, region.right);
      root->pixels += region.pixels;
      root->whitePixels += region.whitePixels;
    }

  upperBound = extendedLower[upperBound];
  unsigned short lowerBound = static_cast<unsigned short>(scanGrid.fullResY[0] + 1);

  struct Candidate
  {
    Boundaryi region;
    Vector2i center;
    int xExtent;
    int yExtent;
  } candidate;

  std::vector<Candidate> candidates;
  for(Region* region : mergedRegions)
    if(region->upper >= upperBound && region->lower < lowerBound)
    {
      Vector2f center(static_cast<float>(region->right + region->left) * .5f, static_cast<float>(region->lower + region->upper) * .5f);
      float expectedWidth;
      float expectedHeight;
      if(IISC::calculateImagePenaltyMeasurementsByCenter(center, expectedWidth, expectedHeight, info_in, cameraInfo))
      {
        float measuredWidth = static_cast<float>(region->right - region->left);
        float measuredHeight = static_cast<float>(region->lower - region->upper);
        if(measuredWidth <= expectedWidth * (1.f + sizeToleranceRatio)
           && measuredWidth + 2 * (xStep - 1) >= expectedWidth * (1.f - sizeToleranceRatio)
           && measuredHeight <= expectedHeight * (1.f + sizeToleranceRatio)
           && measuredHeight >= expectedHeight * (1.f - sizeToleranceRatio)
           && region->left > scanGrid.verticalLines[scanGrid.lowResStart].x
           && region->right < scanGrid.verticalLines[scanGrid.verticalLines.size() - 1 - scanGrid.lowResStart].x
           && region->pixels * minWhiteRatio < region->whitePixels)
        {
          candidate.center = center.cast<int>();
          candidate.region = Boundaryi(Rangei((candidate.center.x() - xStep + 1) / blockSizeX * blockSizeX,
                                              (candidate.center.x() + xStep) / blockSizeX * blockSizeX),
                                       Rangei(candidate.center.y() / blockSizeY * blockSizeY,
                                              (candidate.center.y() + blockSizeY) / blockSizeY * blockSizeY));
          candidate.xExtent = (static_cast<int>(expectedWidth * (1.f + sizeToleranceRatio) / 2.f) + blockSizeX - 1) / blockSizeX * blockSizeX;
          candidate.yExtent = (static_cast<int>(expectedHeight * (1.f + sizeToleranceRatio) / 2.f) + blockSizeY - 1) / blockSizeY * blockSizeY;
          Boundaryi cnsRegion(Rangei(std::max(0, candidate.region.x.min - candidate.xExtent),
                                     std::min(cameraInfo.width, candidate.region.x.max + candidate.xExtent)),
                              Rangei(std::max(0, candidate.region.y.min - candidate.yExtent),
                                     std::min(cameraInfo.height, candidate.region.y.max + candidate.yExtent)));
          candidate.region = Boundaryi(Rangei(cnsRegion.x.min + candidate.xExtent, cnsRegion.x.max - candidate.xExtent),
                                       Rangei(cnsRegion.y.min + candidate.yExtent, cnsRegion.y.max - candidate.yExtent));
          if(candidate.region.x.min < candidate.region.x.max && candidate.region.y.min < candidate.region.y.max) {
            candidates.emplace_back(candidate);
          }
        }
      }
    }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {return a.center.y() > b.center.y();});
  for(size_t i = 0; i < candidates.size() && i < maxNumberOfRegions; ++i)
  {
    const Candidate& candidate = candidates[i];
    searchRegions.emplace_back(candidate.region);
  }
}
