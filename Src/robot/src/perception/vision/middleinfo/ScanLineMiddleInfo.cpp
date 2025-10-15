/**
 * @file ScanLineMiddleInfo.cpp
 *
 * This file implements a module that segments the image horizontally
 * and vertically by detecting edges and applying heuristics to classify
 * the regions between them.
 *
 * @author Lukas Malte Monnerjahn
 * @author Arne Hasselbring
 * @author RedBackBots
 */

#include "perception/vision/middleinfo/ScanLineMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "utils/debug/Assert.hpp"

#include <functional>
#include <list>
#include <vector>

ScanLineMiddleInfo::ScanLineMiddleInfo(Blackboard* blackboard):
  Detector("ScanLineMiddleInfo")
{
    configure(blackboard);

    llog(INFO) << NDEBUG_LOGSYMB << "ScanLineMiddleInfo loaded" << std::endl;
}

ScanLineMiddleInfo::~ScanLineMiddleInfo() {

}

void ScanLineMiddleInfo::configure(Blackboard* blackboard) {

}

void ScanLineMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    info_middle->colorScanLineRegionsHorizontal[CameraInfo::Camera::top].scanLines.clear();
    info_middle->colorScanLineRegionsVerticalClipped[CameraInfo::Camera::top].scanLines.clear();
    info_middle->colorScanLineRegionsHorizontal[CameraInfo::Camera::bot].scanLines.clear();
    info_middle->colorScanLineRegionsVerticalClipped[CameraInfo::Camera::bot].scanLines.clear();
}

void ScanLineMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void ScanLineMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    // Top camera must run first, as the bottom camera is only computed if the top camera boundary can't be projected
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // TODO (TW): decide if lower field boundary is actually needed
    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void ScanLineMiddleInfo::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    
    // llog(DEBUG) << NDEBUG_LOGSYMB << "# colorScanLineRegionsHorizontal" << std::endl;
    ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal = info_middle->colorScanLineRegionsHorizontal[whichCamera];
    updateRegionsHorizontal(colorScanLineRegionsHorizontal, whichCamera, info_in, info_middle, info_out);
	  // llog(DEBUG) << NDEBUG_LOGSYMB << "	-- Number of scan lines: " << colorScanLineRegionsHorizontal.scanLines.size() << std::endl;

    // llog(DEBUG) << NDEBUG_LOGSYMB << "# colorScanLineRegionsVerticalClipped" << std::endl;
    ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped = info_middle->colorScanLineRegionsVerticalClipped[whichCamera];
    updateRegionsVerticalClipped(colorScanLineRegionsVerticalClipped, whichCamera, info_in, info_middle, info_out);
	  // llog(DEBUG) << NDEBUG_LOGSYMB << "	-- Number of scan lines: " << colorScanLineRegionsVerticalClipped.scanLines.size() << std::endl;
}

void ScanLineMiddleInfo::updateRegionsHorizontal(ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal,
  CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {

  const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
  const CameraImage* cameraImage = info_in->image[whichCamera];
  ScanGrid& scanGrid = info_middle->scanGrid[whichCamera];
  ECImage& ecImage = info_middle->ecImage[whichCamera];
  RelativeFieldColors& relativeFieldColors = info_middle->relativeFieldColors[whichCamera];
  FieldBoundary& fieldBoundary = info_out->fieldBoundary[whichCamera];

  if(!scanGrid.isValid() || !fieldBoundary.isValid) {
      // llog(DEBUG) << NDEBUG_LOGSYMB << " -scangrid is empty or fieldBoundary is invalid" << std::endl;
      return;
  }

  // 0. Preprocess
  ScanLineMiddleInfo::approximateBaseLuminance(relativeFieldColors);
  ScanLineMiddleInfo::approximateBaseSaturation(relativeFieldColors);

  // 1. Define scan-lines
  std::vector<unsigned short> yPerScanLine;  // heights of the scan-lines in the image
  std::vector<std::vector<InternalRegion>> regionsPerScanLine;

  // find height in image up to which additional smoothing (5x5 filter) will be applied.
  // compute by projecting on-field distance into image
  Point2i pointInImage;  // helper variable for projection
  int middle = 0;
  if (whichCamera == CameraInfo::Camera::top) {
    pointInImage = info_in->kinematicPose.headRelRobotToImageXY(additionalSmoothingPoint, cameraInfo);
    if (pointInImage.x() < 0) {
      middle = cameraInfo.height - 1;
    } 
    else {
      middle = static_cast<const int>(pointInImage.y());
    }
  }

  for(const ScanGrid::ScanGridHorizontalLine& horizontalLine : scanGrid.lowResHorizontalLines) {
    yPerScanLine.emplace_back(horizontalLine.y);
    regionsPerScanLine.emplace_back();

    // 2. Detect edges and create temporary regions in between including a representative YHS triple.
    if(cameraInfo.camera == CameraInfo::Camera::bot || middle < horizontalLine.y) {
      scanHorizontalAdditionalSmoothing(horizontalLine.y, regionsPerScanLine.back(), horizontalLine.left, horizontalLine.right, ecImage, scanGrid);
    }
    else {
      scanHorizontal(horizontalLine.y, regionsPerScanLine.back(), horizontalLine.left, horizontalLine.right, ecImage, scanGrid);
    }
  }

  // 3. Classify field regions.
  uniteHorizontalFieldRegions(yPerScanLine, regionsPerScanLine);
  classifyFieldRegions(yPerScanLine, regionsPerScanLine, true, cameraInfo, cameraImage, scanGrid, relativeFieldColors.rfcParameters);

  // 4. Classify white regions.
  classifyWhiteRegionsWithThreshold(regionsPerScanLine, relativeFieldColors.rfcParameters);

  // 5. Fill small gaps between field and white regions
  stitchUpHoles(regionsPerScanLine, true);

  // 6. Convert to external representation.
  emplaceInScanLineRegionsHorizontal(colorScanLineRegionsHorizontal, yPerScanLine, regionsPerScanLine);
}

void ScanLineMiddleInfo::updateRegionsVerticalClipped(ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped,
  CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {

  const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
  const CameraImage* cameraImage = info_in->image[whichCamera];
  ScanGrid& scanGrid = info_middle->scanGrid[whichCamera];
  ECImage& ecImage = info_middle->ecImage[whichCamera];
  RelativeFieldColors& relativeFieldColors = info_middle->relativeFieldColors[whichCamera];
  FieldBoundary& fieldBoundary = info_out->fieldBoundary[whichCamera];

  if(scanGrid.verticalLines.empty() || scanGrid.lowResHorizontalLines.empty() || !fieldBoundary.isValid) {
    // llog(DEBUG) << NDEBUG_LOGSYMB << " --- scangrid is empty or fieldBoundary is invalid" << std::endl;
    return;
  }

  colorScanLineRegionsVerticalClipped.lowResStart = scanGrid.lowResStart;
  colorScanLineRegionsVerticalClipped.lowResStep = scanGrid.lowResStep;

  // 0. Preprocess
  approximateBaseLuminance(relativeFieldColors);
  approximateBaseSaturation(relativeFieldColors);

  // 1. Define scan-lines and limit scan-line ranges by field boundary (body contour is already excluded by ScanGrid)
  const std::size_t numOfScanLines = scanGrid.verticalLines.size();
  std::vector<unsigned short> xPerScanLine(numOfScanLines);
  std::vector<std::vector<InternalRegion>> regionsPerScanLine(numOfScanLines);

  // find height in image up to which additional smoothing (5x5 filter) will be applied.
  // compute by projecting on-field distance into image
  Point2i pointInImage;  // helper variable for projection
  int middle = 0;
  if (whichCamera == CameraInfo::Camera::top) {
    pointInImage = info_in->kinematicPose.headRelRobotToImageXY(additionalSmoothingPoint, cameraInfo);
    if (pointInImage.x() < 0) {
      middle = cameraInfo.height - 1;
    } 
    else {
      middle = static_cast<const int>(pointInImage.y());
    }
  }

  for(std::size_t i = 0; i < scanGrid.verticalLines.size(); ++i) {
    xPerScanLine[i] = static_cast<unsigned short>(scanGrid.verticalLines[i].x);
    const int top = scanGrid.verticalLines[i].yMin + 1;

    // 2. Detect edges and create temporary regions in between including a representative YHS triple.
    scanVertical(scanGrid.verticalLines[i], middle, top, regionsPerScanLine[i], scanGrid, ecImage);
  }

  // 3. Classify field regions.
  uniteVerticalFieldRegions(xPerScanLine, regionsPerScanLine);
  classifyFieldRegions(xPerScanLine, regionsPerScanLine, false, cameraInfo, cameraImage, scanGrid, relativeFieldColors.rfcParameters);

  // 4. Classify white regions.
  classifyWhiteRegionsWithThreshold(regionsPerScanLine, relativeFieldColors.rfcParameters);

  // 5. Fill small gaps between field and white regions
  stitchUpHoles(regionsPerScanLine, false);

  // 6. Convert to external representation.
  emplaceInScanLineRegionsVertical(colorScanLineRegionsVerticalClipped, xPerScanLine, regionsPerScanLine);
}

void ScanLineMiddleInfo::scanHorizontal(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX, 
  ECImage& ecImage, ScanGrid& scanGrid) const {
  if(y < 1 || y >= ecImage.grayscaled.height - 1) {
    return;
  }

  // initialize variables
  const int scanY = static_cast<int>(y);
  const unsigned int scanStart = leftmostX < ecImage.grayscaled.width - 2 ? leftmostX : ecImage.grayscaled.width - 3;
  bool nextRegionWhite = false;

  ScanRun<3> scanRun(true, static_cast<int>(y), scanStart, rightmostX);
  scanRun.leftScanEdgePosition = leftmostX;

  // define filter:
  // vertical 1D gauss/sobel smoothing: filter-matrix [[1], [2], [1]]
  scanRun.gaussV = [](const PixelTypes::GrayscaledPixel* line, const unsigned int imageWidth) {
    return static_cast<int>(line[-static_cast<int>(imageWidth)] + 2 * line[0] + line[imageWidth]);
  };

  // horizontal 1D Gauss/sobel smoothing: filter-matrix [[1, 2, 1]]
  // results in 2D-Gaussian-filter when applied on values returned by the other 1D-Gaussian-filter
  scanRun.gaussSecond = [](std::array<int, 3>& gaussBuffer, int x) {
    return gaussBuffer[(x - 1) % 3] + 2 * gaussBuffer[x % 3] + gaussBuffer[(x + 1) % 3];
  };

  // linear gradient, results in sobel filter when applied on values returned by the 1D-Gaussian-filter
  scanRun.gradient = [](std::array<int, 3>& gaussBuffer, int x) {
    return gaussBuffer[(x - 1) % 3] - gaussBuffer[(x + 1) % 3];
  };

  // 1D 3x3 gauss filter: 1, 2, 1 -> sum: 4, sum of 2D gauss filter values is 16
  const int thresholdAdaption = 16;
  int threshold = thresholdAdaption * static_cast<int>(edgeThreshold);

  // Initialize the buffer of smoothed values.
  const PixelTypes::GrayscaledPixel* luminance = &ecImage.grayscaled[scanY][scanStart];
  scanRun.leftGaussBuffer[0] = scanRun.gaussV(luminance, ecImage.grayscaled.width);
  scanRun.leftGaussBuffer[1] = scanRun.gaussV(++luminance, ecImage.grayscaled.width);
  scanRun.leftGaussBuffer[2] = scanRun.gaussV(++luminance, ecImage.grayscaled.width);

  // grid scan stuff
  unsigned int gridX = scanStart + 1;
  int gridValue = scanRun.gaussSecond(scanRun.leftGaussBuffer, 1);
  int nextGridValue;
  size_t gridLineIndex = scanGrid.lowResStart;
  unsigned int nextGridX = scanGrid.verticalLines[gridLineIndex].x;
  while(nextGridX <= gridX && nextGridX < rightmostX && ++gridLineIndex < scanGrid.verticalLines.size()) {
    nextGridX = scanGrid.verticalLines[gridLineIndex].x;
  }
  if(nextGridX <= gridX && nextGridX < rightmostX) {
    nextGridX = rightmostX; // skip the loop, make only a single region
  }

  // 'gridLineIndex <= scanGrid.lines.size()' so that the image's right edge becomes the last grid point
  while(gridLineIndex <= scanGrid.verticalLines.size() && nextGridX < rightmostX) {
    bool regionAdded = false;
    scanRun.rightGaussBuffer[0] = scanRun.gaussV(&ecImage.grayscaled[scanY][nextGridX - 1], ecImage.grayscaled.width);
    scanRun.rightGaussBuffer[1] = scanRun.gaussV(&ecImage.grayscaled[scanY][nextGridX], ecImage.grayscaled.width);
    scanRun.rightGaussBuffer[2] = scanRun.gaussV(&ecImage.grayscaled[scanY][nextGridX + 1], ecImage.grayscaled.width);

    nextGridValue = scanRun.gaussSecond(scanRun.rightGaussBuffer, 1);
    if(gridValue - nextGridValue >= threshold) { // bright to dark transition
      findEdgeInSubLineHorizontal(regions, scanRun, gridX, nextGridX, true, ecImage);
      regionAdded = true;
      if(prelabelAsWhite && nextRegionWhite && prelabelWhiteCheck(regions.back()) &&
          prelabelWhiteNeighborCheck(regions.back(), regions[regions.size() - 2])) {
        regions.back().color = ScanLineRegion::white;
      }
      nextRegionWhite = false;
    }
    else if(gridValue - nextGridValue <= -threshold) { // dark to bright transition
      findEdgeInSubLineHorizontal(regions, scanRun, gridX, nextGridX, false, ecImage);
      regionAdded = true;
      nextRegionWhite = true;
    }
    if(prelabelAsWhite && regionAdded && regions.size() >= 2 && regions[regions.size() - 2].color == ScanLineRegion::white) {
      if(!prelabelWhiteNeighborCheck(regions[regions.size() - 2], regions.back())) {
        regions[regions.size() - 2].color = ScanLineRegion::unset;
      }
    }
    // prepare next loop
    ++gridLineIndex;
    gridX = nextGridX;
    if(gridLineIndex < scanGrid.verticalLines.size())
      nextGridX = scanGrid.verticalLines[gridLineIndex].x;
    else
      nextGridX = ecImage.grayscaled.width - 2;
    gridValue = nextGridValue;
    scanRun.leftGaussBuffer = scanRun.rightGaussBuffer;
  }
  // add last region
  ASSERT(scanRun.leftScanEdgePosition < rightmostX);
  regions.emplace_back(scanRun.leftScanEdgePosition, rightmostX,
      getHorizontalRepresentativeValue(ecImage.grayscaled, scanRun.leftScanEdgePosition, rightmostX, y),
      getHorizontalRepresentativeHueValue(ecImage.hued, scanRun.leftScanEdgePosition, rightmostX, y),
      getHorizontalRepresentativeValue(ecImage.saturated, scanRun.leftScanEdgePosition, rightmostX, y));
}

void ScanLineMiddleInfo::scanHorizontalAdditionalSmoothing(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX,
    ECImage& ecImage, ScanGrid& scanGrid) const {
  if(y < 2 || y >= ecImage.grayscaled.height - 2) {
    return;
  }
  // initialize variables
  const int scanY = static_cast<int>(y);
  const unsigned int scanStart = leftmostX < ecImage.grayscaled.width - 4 ? leftmostX : ecImage.grayscaled.width - 5;
  const unsigned int scanStop = rightmostX >= 2 ? rightmostX - 2 : 0;

  ScanRun<5> scanRun(true, static_cast<int>(y), scanStart, rightmostX);
  scanRun.leftScanEdgePosition = leftmostX;

  // define filter
  // vertical 1D gauss/sobel smoothing: filter-matrix [[1], [2], [4], [2], [1]]
  scanRun.gaussV = [](const PixelTypes::GrayscaledPixel* line, const unsigned int imageWidth) { // 5x5 gauss smoothing vertical
    return static_cast<int>(line[-2 * static_cast<int>(imageWidth)] + 2 * line[-static_cast<int>(imageWidth)] + 4 * line[0] +
                            2 * line[imageWidth] + line[2 * imageWidth]);
  };

  // horizontal 1D Gauss/sobel smoothing: filter-matrix [[1, 2, 4, 2, 1]]
  // results in 2D-Gaussian-filter when applied on values returned by the other 1D-Gaussian-filter
  scanRun.gaussSecond = [](std::array<int, 5>& gaussBuffer, int x) { // 5x5 gauss smoothing horizontal
    return gaussBuffer[(x - 2) % 5] + 2 * gaussBuffer[(x - 1) % 5] + 4 * gaussBuffer[x % 5] + 2 * gaussBuffer[(x + 1) % 5] + gaussBuffer[(x + 2) % 5];
  };

  // sobel-like gradient, results in sobel-like filter when applied on values returned by the 1D-Gaussian-filter
  scanRun.gradient = [](std::array<int, 5>& gaussBuffer, int x) {
    return gaussBuffer[(x - 2) % 5] + 2 * gaussBuffer[(x - 1) % 5] - 2 * gaussBuffer[(x + 1) % 5] - gaussBuffer[(x + 2) % 5];
  };

  // 1D 5x5 gauss filter: 1, 2, 4, 2, 1 -> sum: 10, sum of 2D gauss filter values is 100
  const int thresholdAdaption = 100;
  int threshold = thresholdAdaption * static_cast<int>(edgeThreshold);

  // Initialize the buffer of smoothed values.
  const PixelTypes::GrayscaledPixel* luminance = &ecImage.grayscaled[scanY][scanStart];
  scanRun.leftGaussBuffer[0] = scanRun.gaussV(luminance, ecImage.grayscaled.width);
  scanRun.leftGaussBuffer[1] = scanRun.gaussV(++luminance, ecImage.grayscaled.width);
  scanRun.leftGaussBuffer[2] = scanRun.gaussV(++luminance, ecImage.grayscaled.width);
  scanRun.leftGaussBuffer[3] = scanRun.gaussV(++luminance, ecImage.grayscaled.width);
  scanRun.leftGaussBuffer[4] = scanRun.gaussV(++luminance, ecImage.grayscaled.width);

  // setup grid scan
  unsigned int gridX = scanStart + 2;
  int gridValue = scanRun.gaussSecond(scanRun.leftGaussBuffer, 2);
  int nextGridValue;
  size_t gridLineIndex = scanGrid.lowResStart;
  unsigned int nextGridX;
  do {
    nextGridX = scanGrid.verticalLines[gridLineIndex].x;
  }
  while(nextGridX <= gridX && nextGridX <= scanStop && ++gridLineIndex < scanGrid.verticalLines.size());
  if(nextGridX <= gridX) {
    nextGridX = rightmostX; // skip the loop, make only a single region
  }

  while(gridLineIndex <= scanGrid.verticalLines.size() && nextGridX <= scanStop) {
    scanRun.rightGaussBuffer[0] = scanRun.gaussV(&ecImage.grayscaled[scanY][nextGridX - 2], ecImage.grayscaled.width);
    scanRun.rightGaussBuffer[1] = scanRun.gaussV(&ecImage.grayscaled[scanY][nextGridX - 1], ecImage.grayscaled.width);
    scanRun.rightGaussBuffer[2] = scanRun.gaussV(&ecImage.grayscaled[scanY][nextGridX], ecImage.grayscaled.width);
    scanRun.rightGaussBuffer[3] = scanRun.gaussV(&ecImage.grayscaled[scanY][nextGridX + 1], ecImage.grayscaled.width);
    scanRun.rightGaussBuffer[4] = scanRun.gaussV(&ecImage.grayscaled[scanY][nextGridX + 2], ecImage.grayscaled.width);

    nextGridValue = scanRun.gaussSecond(scanRun.rightGaussBuffer, 2);
    if(gridValue - nextGridValue >= threshold) {
      findEdgeInSubLineHorizontal(regions, scanRun, gridX, nextGridX, true, ecImage);
    }
    else if(gridValue - nextGridValue <= -threshold) {
      findEdgeInSubLineHorizontal(regions, scanRun, gridX, nextGridX, false, ecImage);
    }
    // prepare next loop
    gridLineIndex += scanGrid.lowResStep;
    gridX = nextGridX;
    if(gridLineIndex < scanGrid.verticalLines.size()) {
      nextGridX = scanGrid.verticalLines[gridLineIndex].x;
    }
    else {
      nextGridX = ecImage.grayscaled.width - 3;
    }
    gridValue = nextGridValue;
    scanRun.leftGaussBuffer = scanRun.rightGaussBuffer;
  }
  // add last region
  ASSERT(scanRun.leftScanEdgePosition < rightmostX);
  regions.emplace_back(scanRun.leftScanEdgePosition, rightmostX,
      getHorizontalRepresentativeValue(ecImage.grayscaled, scanRun.leftScanEdgePosition, rightmostX, y),
      getHorizontalRepresentativeHueValue(ecImage.hued, scanRun.leftScanEdgePosition, rightmostX, y),
      getHorizontalRepresentativeValue(ecImage.saturated, scanRun.leftScanEdgePosition, rightmostX, y));
}

void ScanLineMiddleInfo::scanVertical(const ScanGrid::ScanGridLine& line, int middle, int top, std::vector<InternalRegion>& regions,
    ScanGrid& scanGrid, ECImage& ecImage) const {
  if(line.x < 1 || static_cast<unsigned int>(line.x + 1) >= ecImage.grayscaled.width || line.yMax <= std::max(2, top)) {
    return;
  }
  // initialize variables shared between 5x5 and 3x3
  unsigned int lowerY = line.yMax;
  bool nextRegionWhite = false;
  int threshold = static_cast<int>(100.f * edgeThreshold); // "100 *" because it relates to the smoothed values of the 5x5 filter.
  // y-coordinate ranges
  ASSERT(top >= 0); // 3x3 filter stop exclusive
  const int lowestY = std::min<int>(line.yMax - 2, static_cast<int>(ecImage.grayscaled.height) - 3); // 5x5 filter start
  const int middleY = std::min(std::max<int>(middle + 1, top + 1), lowestY + 1); // 5x5 filter stop exclusive, 3x3 filter start
  const PixelTypes::GrayscaledPixel* luminance = &ecImage.grayscaled[lowestY][line.x];
  // grid variables
  int gridY = std::max(lowestY, middleY);
  int gridValue = 0;
  int nextGridValue;
  size_t gridYIndex = line.yMaxIndex;
  while(scanGrid.fullResY[gridYIndex] + verticalGridScanMinStep > gridY && gridYIndex + 1 < scanGrid.fullResY.size()) {
    ++gridYIndex;
  }
  int nextGridY = scanGrid.fullResY[gridYIndex];
  if(nextGridY >= gridY) {
    nextGridY = 2;
  }

  // start with 5x5 filter
  if(lowestY > middleY) {
    ScanRun<5> scanRun(false, static_cast<int>(line.x), lowestY, middleY);
    scanRun.lowerScanEdgePosition = lowerY;

    scanRun.gaussH = [](const PixelTypes::GrayscaledPixel* line) { // 5x5 gauss horizontal
      return static_cast<int>(line[-2] + 2 * line[-1] + 4 * line[0] + 2 * line[1] + line[2]);
    };
    scanRun.gaussSecond = [](std::array<int, 5>& gaussBuffer, int y) { // 5x5 gauss vertical
      return gaussBuffer[(y - 2) % 5] + 2 * gaussBuffer[(y - 1) % 5] + 4 * gaussBuffer[y % 5] + 2 * gaussBuffer[(y + 1) % 5] + gaussBuffer[(y + 2) % 5];
    };
    scanRun.gradient = [](std::array<int, 5>& gaussBuffer, int y) { // sobel-like gradient
      return gaussBuffer[(y - 2) % 5] + 2 * gaussBuffer[(y - 1) % 5] - 2 * gaussBuffer[(y + 1) % 5] - gaussBuffer[(y + 2) % 5];
    };
    // Initialize the buffer of smoothed values.
    scanRun.lowerGaussBuffer = {}; // buffer for the lower grid point and for sobel scans
    scanRun.upperGaussBuffer = {}; // buffer for the upper grid point, centered around grid point -> gridX is at index 2
    scanRun.lowerGaussBuffer[0] = scanRun.gaussH(luminance + 2 * ecImage.grayscaled.width);
    scanRun.lowerGaussBuffer[1] = scanRun.gaussH(luminance + ecImage.grayscaled.width);
    scanRun.lowerGaussBuffer[2] = scanRun.gaussH(luminance);
    scanRun.lowerGaussBuffer[3] = scanRun.gaussH(luminance -= ecImage.grayscaled.width);
    scanRun.lowerGaussBuffer[4] = scanRun.gaussH(luminance -= ecImage.grayscaled.width);
    gridValue = scanRun.gaussSecond(scanRun.lowerGaussBuffer, 2);

    while(nextGridY > middleY && gridYIndex <= scanGrid.lowResHorizontalLines.size()) {
      scanRun.upperGaussBuffer[0] = scanRun.gaussH(&ecImage.grayscaled[nextGridY + 2][line.x]);
      scanRun.upperGaussBuffer[1] = scanRun.gaussH(&ecImage.grayscaled[nextGridY + 1][line.x]);
      scanRun.upperGaussBuffer[2] = scanRun.gaussH(&ecImage.grayscaled[nextGridY][line.x]);
      scanRun.upperGaussBuffer[3] = scanRun.gaussH(&ecImage.grayscaled[nextGridY - 1][line.x]);
      scanRun.upperGaussBuffer[4] = scanRun.gaussH(&ecImage.grayscaled[nextGridY - 2][line.x]);
      nextGridValue = scanRun.gaussSecond(scanRun.upperGaussBuffer, 2);
      if(gridValue - nextGridValue >= threshold) {
        findEdgeInSubLineVertical(regions, scanRun, gridY, nextGridY, true, ecImage);
      }
      else if(gridValue - nextGridValue <= -threshold) {
        findEdgeInSubLineVertical(regions, scanRun, gridY, nextGridY, false, ecImage);
      }
      // prepare next loop
      gridY = nextGridY;
      while(gridY < nextGridY + verticalGridScanMinStep) {
        ++gridYIndex;
        if(gridYIndex < scanGrid.fullResY.size()) {
          nextGridY = scanGrid.fullResY[gridYIndex];
        }
        else {
          break;
        }
      }
      gridValue = nextGridValue;
      scanRun.lowerGaussBuffer = scanRun.upperGaussBuffer;
    }
    lowerY = scanRun.lowerScanEdgePosition;
  }
  // switch to 3x3 filter
  if(middleY > top) {
    auto gauss = [](const PixelTypes::GrayscaledPixel* line) { // 3x3 gauss horizontal
      return static_cast<int>(line[-1] + 2 * line[0] + line[1]);
    };
    auto gaussSecond = [](std::array<int, 3>& gaussBuffer, int y) { // 3x3 gauss vertical
      return gaussBuffer[(y - 1) % 3] + 2 * gaussBuffer[y % 3] + gaussBuffer[(y + 1) % 3];
    };
    auto gradient = [](std::array<int, 3>& gaussBuffer, int y) {
      return gaussBuffer[(y - 1) % 3] - gaussBuffer[(y + 1) % 3];
    };
    // Initialize the buffer of smoothed values.
    std::array<int, 3> gaussBuffer {};
    luminance = &ecImage.grayscaled[gridY][line.x];
    gaussBuffer[0] = gauss(luminance + ecImage.grayscaled.width);
    gaussBuffer[1] = gauss(luminance);
    int gaussBufferIndex = 2;
    int prevSobelMin = std::numeric_limits<int>::max();
    int prevSobelMax = std::numeric_limits<int>::min();
    unsigned int edgeYMin = ecImage.grayscaled.height;
    unsigned int edgeYMax = ecImage.grayscaled.height;
    // adjust values for switch from 5x5 to 3x3
    gridValue = gridValue * 4 / 25; // smoothing factor is reduced from 100 to 16
    threshold = static_cast<int>(16.f * edgeThreshold); // "16 *" because it relates to the smoothed values.
    for(int y = gridY; y > top; --y) {
      // This line is one above the current y.
      gaussBuffer[gaussBufferIndex % 3] = gauss(luminance -= ecImage.grayscaled.width);
      ++gaussBufferIndex;
      // This gradient is centered around the current y.
      int sobelL = gradient(gaussBuffer, gaussBufferIndex - 2);
      if(sobelL > prevSobelMax) {
        prevSobelMax = sobelL;
        edgeYMax = y;
      }
      if(sobelL < prevSobelMin) {
        prevSobelMin = sobelL;
        edgeYMin = y;
      }
      if(y == gridY) {
        gridValue = gaussSecond(gaussBuffer, gaussBufferIndex - 2);
      }
      else if(y == nextGridY) {
        nextGridValue = gaussSecond(gaussBuffer, gaussBufferIndex - 2);
        if(gridValue - nextGridValue >= threshold) {
          ASSERT(lowerY > edgeYMax);
          regions.emplace_back(edgeYMax, lowerY, getVerticalRepresentativeValue(ecImage.grayscaled, line.x, edgeYMax, lowerY),
              getVerticalRepresentativeHueValue(ecImage.hued, line.x, edgeYMax, lowerY),
              getVerticalRepresentativeValue(ecImage.saturated, line.x, edgeYMax, lowerY));
          lowerY = edgeYMax;
          if(prelabelAsWhite && nextRegionWhite && prelabelWhiteCheck(regions.back()) &&
              prelabelWhiteNeighborCheck(regions.back(), regions[regions.size() - 2]))
            regions.back().color = ScanLineRegion::white;
          nextRegionWhite = false;
        }
        else if(gridValue - nextGridValue <= -threshold) {
          ASSERT(lowerY > edgeYMin);
          regions.emplace_back(edgeYMin, lowerY, getVerticalRepresentativeValue(ecImage.grayscaled, line.x, edgeYMin, lowerY),
              getVerticalRepresentativeHueValue(ecImage.hued, line.x, edgeYMin, lowerY),
              getVerticalRepresentativeValue(ecImage.saturated, line.x, edgeYMin, lowerY));
          lowerY = edgeYMin;
          nextRegionWhite = true;
        }
        if(prelabelAsWhite && regions.size() >= 2 && regions[regions.size() - 2].color == ScanLineRegion::white) {
          if(!prelabelWhiteNeighborCheck(regions[regions.size() - 2], regions.back())) {
            regions[regions.size() - 2].color = ScanLineRegion::unset;
          }
        }
        gridY = nextGridY;
        while(gridY < nextGridY + verticalGridScanMinStep) {
          ++gridYIndex;
          if(gridYIndex < scanGrid.fullResY.size()) {
            nextGridY = scanGrid.fullResY[gridYIndex];
          }
          else {
            break;
          }
        }
        gridValue = nextGridValue;
        prevSobelMax = std::numeric_limits<int>::min();
        prevSobelMin = std::numeric_limits<int>::max();
        edgeYMax = ecImage.grayscaled.height;
        edgeYMin = ecImage.grayscaled.height;
      }
    }
  }
  // add last region
  ASSERT(lowerY > static_cast<unsigned int>(top));
  regions.emplace_back(top, lowerY, getVerticalRepresentativeValue(ecImage.grayscaled, line.x, top, lowerY),
      getVerticalRepresentativeHueValue(ecImage.hued, line.x, top, lowerY),
      getVerticalRepresentativeValue(ecImage.saturated, line.x, top, lowerY));
}

template <int filterSize>
void ScanLineMiddleInfo::findEdgeInSubLineHorizontal(std::vector<InternalRegion>& regions,
    ScanRun<filterSize>& scanRun, unsigned int startPos, unsigned int stopPos, bool maxEdge, ECImage& ecImage) const {
  unsigned int edgeXMax = startPos;
  int sobelMax = scanRun.gradient(scanRun.leftGaussBuffer, (filterSize - 1) / 2);
  const PixelTypes::GrayscaledPixel* luminance = &ecImage.grayscaled[scanRun.scanLinePosition][startPos + 1];
  int gaussBufferIndex = 0;
  for(unsigned int x = startPos + 1; x < stopPos; ++x, ++luminance, ++gaussBufferIndex)
  {
    scanRun.leftGaussBuffer[gaussBufferIndex % filterSize] = scanRun.gaussV(luminance, ecImage.grayscaled.width);
    int sobelL = scanRun.gradient(scanRun.leftGaussBuffer, gaussBufferIndex + (filterSize + 1) / 2);
    if((maxEdge && sobelL > sobelMax) || (!maxEdge && sobelL < sobelMax))
    {
      edgeXMax = x;
      sobelMax = sobelL;
    }
  }
  // save region
  ASSERT(scanRun.leftScanEdgePosition < edgeXMax);
  regions.emplace_back(scanRun.leftScanEdgePosition, edgeXMax, getHorizontalRepresentativeValue(ecImage.grayscaled, scanRun.leftScanEdgePosition, edgeXMax, scanRun.scanLinePosition),
      getHorizontalRepresentativeHueValue(ecImage.hued, scanRun.leftScanEdgePosition, edgeXMax, scanRun.scanLinePosition),
      getHorizontalRepresentativeValue(ecImage.saturated, scanRun.leftScanEdgePosition, edgeXMax, scanRun.scanLinePosition));
  scanRun.leftScanEdgePosition = edgeXMax;
}

template <int filterSize>
void ScanLineMiddleInfo::findEdgeInSubLineVertical(std::vector<InternalRegion>& regions,
    ScanRun<filterSize>& scanRun, unsigned int startPos, unsigned int stopPos, bool maxEdge, ECImage& ecImage) const {
  unsigned int edgeYMax = startPos;
  int sobelMax = scanRun.gradient(scanRun.lowerGaussBuffer, (filterSize - 1) / 2);
  const PixelTypes::GrayscaledPixel* luminance = &ecImage.grayscaled[startPos - 1][scanRun.scanLinePosition];
  int gaussBufferIndex = 0;
  for(int y = static_cast<int>(startPos) - 1; y > static_cast<int>(stopPos); --y, ++gaussBufferIndex, luminance -= ecImage.grayscaled.width)
  {
    scanRun.lowerGaussBuffer[gaussBufferIndex % filterSize] = scanRun.gaussH(luminance);
    int sobelL = scanRun.gradient(scanRun.lowerGaussBuffer, gaussBufferIndex + (filterSize + 1) / 2);
    if((maxEdge && sobelL > sobelMax) || (!maxEdge && sobelL < sobelMax))
    {
      edgeYMax = y;
      sobelMax = sobelL;
    }
  }
  // save region
  ASSERT(scanRun.lowerScanEdgePosition > edgeYMax);
  regions.emplace_back(edgeYMax, scanRun.lowerScanEdgePosition, getVerticalRepresentativeValue(ecImage.grayscaled, scanRun.scanLinePosition, edgeYMax, scanRun.lowerScanEdgePosition),
      getVerticalRepresentativeHueValue(ecImage.hued, scanRun.scanLinePosition, edgeYMax, scanRun.lowerScanEdgePosition),
      getVerticalRepresentativeValue(ecImage.saturated, scanRun.scanLinePosition, edgeYMax, scanRun.lowerScanEdgePosition));
  scanRun.lowerScanEdgePosition = edgeYMax;
}

void ScanLineMiddleInfo::uniteHorizontalFieldRegions(const std::vector<unsigned short>& y, std::vector<std::vector<InternalRegion>>& regions) const {
  ASSERT(y.size() == regions.size());
  for(std::size_t lineIndex = 0; lineIndex < y.size(); ++lineIndex) {
    std::size_t nextLineIndex = lineIndex + 1;
    std::vector<InternalRegion>::iterator nextLineRegion;
    if(nextLineIndex < regions.size()) {
      nextLineRegion = regions[nextLineIndex].begin();
    }

    for(std::size_t regionIndex = 0; regionIndex < regions[lineIndex].size(); ++regionIndex) {
      InternalRegion& region = regions[lineIndex][regionIndex];
      // try union with next region in the same line
      if(regionIndex + 1 < regions[lineIndex].size()) {
        if(areSimilar(region, regions[lineIndex][regionIndex + 1])) {
          region.unite(regions[lineIndex][regionIndex + 1]);
        }
      }
      // try union with adjacent regions in next line
      if(nextLineIndex < regions.size() && !regions[nextLineIndex].empty()) {
        bool entered = false;
        // regions in the horizontal scan lines are sorted from lower to higher pixel number
        while(nextLineRegion != regions[nextLineIndex].end() && nextLineRegion->range.from < region.range.to) {
          if(areSimilar(region, *nextLineRegion)) {
            region.unite(*nextLineRegion);
          }
          ++nextLineRegion;
          entered = true;
        }
        // The last region on the next line adjacent to the current region on this line is also adjacent to the next region on this line
        if(entered) {
          --nextLineRegion;
        }
      }
    }
  }
}

void ScanLineMiddleInfo::uniteVerticalFieldRegions(const std::vector<unsigned short>& x, std::vector<std::vector<InternalRegion>>& regions) const {
  ASSERT(x.size() == regions.size());
  for(std::size_t lineIndex = 0; lineIndex < regions.size(); ++lineIndex) {
    // build list of regions in the neighborhood of the line. Can span multiple scan lines due to ScanGrid layout
    if(regions[lineIndex].empty()) {
      continue;
    }
    unsigned short lineRangeFrom = regions[lineIndex][regions[lineIndex].size() - 1].range.from;
    const unsigned short lineRangeTo = regions[lineIndex][0].range.to;
    ASSERT(lineRangeFrom < lineRangeTo);
    std::list<std::pair<InternalRegion*, unsigned short>> nextLinesRegions;
    for(std::size_t nextLineIndex = lineIndex + 1; nextLineIndex < regions.size() && nextLineIndex <= lineIndex + 4; ++nextLineIndex) {
      if(lineRangeFrom >= lineRangeTo) {
        break;
      }
      for(long nextLineRegionIndex = static_cast<long>(regions[nextLineIndex].size()) - 1; nextLineRegionIndex >= 0; --nextLineRegionIndex) {
        if(regions[nextLineIndex][nextLineRegionIndex].range.to <= lineRangeFrom) {
          continue;
        }
        else if(regions[nextLineIndex][nextLineRegionIndex].range.from >= lineRangeTo) {
          break;
        }
        else {
          nextLinesRegions.emplace_back(&regions[nextLineIndex][nextLineRegionIndex], x[nextLineIndex]);
          lineRangeFrom = regions[nextLineIndex][nextLineRegionIndex].range.to;
        }
      }
    }
    std::list<std::pair<InternalRegion*, unsigned short>>::iterator nextLineRegion;
    if(!nextLinesRegions.empty()) {
      nextLineRegion = nextLinesRegions.begin();
    }

    for(long regionIndex = static_cast<long>(regions[lineIndex].size()) - 1; regionIndex >= 0 ; --regionIndex) {
      InternalRegion& region = regions[lineIndex][regionIndex];
      // try union with next region in the same line
      if(regionIndex > 0) {
        if(areSimilar(region, regions[lineIndex][regionIndex - 1])) {
          region.unite(regions[lineIndex][regionIndex - 1]);
        }
      }
      // try union with adjacent regions in next lines
      if(!nextLinesRegions.empty()) {
        // regions in the vertical scan lines are sorted from higher to lower pixel number
        bool loopEntered = false;
        while(nextLineRegion != nextLinesRegions.end() && nextLineRegion->first->range.from < region.range.to) {
          if(areSimilar(region, *(nextLineRegion->first))) {
            region.unite(*(nextLineRegion->first));
          }
          loopEntered = true;
          ++nextLineRegion;
        }
        if(loopEntered) {
          --nextLineRegion;
        }
      }
    }
  }
}

bool ScanLineMiddleInfo::areSimilar(InternalRegion& a, InternalRegion& b) const {
  const InternalRegion* const aParent = a.findSet();
  const InternalRegion* const bParent = b.findSet();
  // test for white because of white prelabeling -> prelabeled regions should not be changed
  if(std::abs(static_cast<short>(aParent->y) - static_cast<short>(bParent->y)) < luminanceSimilarityThreshold &&
      std::abs(static_cast<short>(aParent->h) - static_cast<short>(bParent->h)) < hueSimilarityThreshold &&
      std::abs(static_cast<short>(aParent->s) - static_cast<short>(bParent->s)) < saturationSimilarityThreshold &&
      aParent->color != ScanLineRegion::white && bParent->color != ScanLineRegion::white) {
    return true;
  }
  return false;
}

void ScanLineMiddleInfo::classifyFieldRegions(const std::vector<unsigned short>& xy, std::vector<std::vector<InternalRegion>>& regions, bool horizontal,
    const CameraInfo& cameraInfo, const CameraImage* cameraImage, ScanGrid& scanGrid, RelativeFieldColorsParameters& relativeFieldColorsParameters) {
  unsigned char minHue = 255;
  unsigned char maxHue = 0;
  unsigned char minSaturation = initialMinSaturation;
  // luminance is only important for images with low saturated field
  unsigned char maxLuminance = baseLuminance;
  int dataPoints = 0;
  for(auto& line : regions) {
    for(auto& region : line) {
      InternalRegion* representative = region.findSet();
      // ASSERTs that all Regions start with color == ScanLineRegion::unset !
      if(representative->color == ScanLineRegion::unset) {
        if(regionIsField(representative, cameraInfo, relativeFieldColorsParameters)) {
          representative->color = ScanLineRegion::field;
          ++dataPoints;
          if(representative->h < minHue) {
            minHue = representative->h;
          }
          if(representative->h > maxHue) {
            maxHue = representative->h;
          }
          if(representative->s < minSaturation) {
            minSaturation = representative->s;
          }
          if(representative->y > maxLuminance) {
            maxLuminance = representative->y;
          }
        }
        else {
          representative->color = ScanLineRegion::none;
        }
      }
      region.color = representative->color;
    }
  }
  if(dataPoints > 0) {
    // expand min/max range
    // the values from the representative regions are averages, but we need sensible min/max-values
    // higher min/max range expansion if we have few data points
    const unsigned char hueRangeExpansion = 8 + static_cast<unsigned char>(2.f * static_cast<float>(hueSimilarityThreshold) / static_cast<float>(dataPoints + 1));
    const unsigned char satRangeExpansion = 5 + static_cast<unsigned char>(2.f * static_cast<float>(saturationSimilarityThreshold) / static_cast<float>(dataPoints + 1));
    const unsigned char lumRangeExpansion = 5 + static_cast<unsigned char>(0.5f * static_cast<float>(baseLuminance) + 1.5f * static_cast<float>(luminanceSimilarityThreshold) / static_cast<float>(dataPoints + 1));
    estimatedFieldColor.minHue = hueRangeExpansion >= minHue ? 0 : minHue - hueRangeExpansion;
    estimatedFieldColor.maxHue = hueRangeExpansion >= std::numeric_limits<unsigned char>::max() - maxHue ? std::numeric_limits<unsigned char>::max() : maxHue + hueRangeExpansion;
    estimatedFieldColor.minSaturation = satRangeExpansion >= minSaturation ? 0 : minSaturation - satRangeExpansion;
    estimatedFieldColor.maxLuminance = lumRangeExpansion >= std::numeric_limits<unsigned char>::max() - maxLuminance ? std::numeric_limits<unsigned char>::max() : maxLuminance + lumRangeExpansion;
    estimatedFieldColor.lastSet = cameraImage->timestamp;
  }
  // label smaller field regions
  if(isEstimatedFieldColorValid(cameraImage)) {
    if(horizontal) {
      classifyFieldHorizontal(xy, regions, cameraInfo, scanGrid);
    }
    else {
      classifyFieldVertical(regions, cameraInfo, scanGrid);
    }
  }
}

void ScanLineMiddleInfo::classifyFieldHorizontal(const std::vector<unsigned short>& y, std::vector<std::vector<InternalRegion>>& regions,
    const CameraInfo& cameraInfo, ScanGrid& scanGrid) const {
  ASSERT(y.size() == regions.size());
  for(size_t lineIndex = 0; lineIndex < regions.size(); ++lineIndex) {
    auto& line = regions[lineIndex];
    for(auto& region : line) {
      classifyFieldSingleRegion(region,  y[lineIndex], true, cameraInfo, scanGrid);
    }
  }
}

void ScanLineMiddleInfo::classifyFieldVertical(std::vector<std::vector<InternalRegion>>& regions,
    const CameraInfo& cameraInfo, ScanGrid& scanGrid) const {
  for(auto& line : regions) {
    for(auto& region : line) {
      classifyFieldSingleRegion(region, 0, false, cameraInfo, scanGrid);
    }
  }
}

void ScanLineMiddleInfo::classifyFieldSingleRegion(ScanLineMiddleInfo::InternalRegion& region, float regionHeight, bool horizontal, const CameraInfo& cameraInfo, ScanGrid& scanGrid) const
{
  ASSERT(region.color != ScanLineRegion::unset);
  if(region.color == ScanLineRegion::none) {
    if(region.findSet()->color == ScanLineRegion::field) {
      region.color = ScanLineRegion::field;
      return;
    }

    if(!horizontal) {
      regionHeight = static_cast<float>(region.range.from + region.range.to) / 2.f;
    }

    unsigned char minSaturation = fieldClassificationSaturationThreshold(region, regionHeight, cameraInfo, scanGrid);
    char hueRangeExpansion = 0;
    if(cameraInfo.camera == CameraInfo::Camera::top) {
        hueRangeExpansion = static_cast<char>(0.5f * static_cast<float>(hueSimilarityThreshold) - (regionHeight - static_cast<float>(scanGrid.fieldLimit)) /
        static_cast<float>(cameraInfo.height -
        scanGrid.fieldLimit) * 0.5f *
        static_cast<float>(hueSimilarityThreshold));
    }

    if(region.h >= estimatedFieldColor.minHue - hueRangeExpansion &&
        region.h <= estimatedFieldColor.maxHue + hueRangeExpansion &&
        region.s >= minSaturation && region.y <= estimatedFieldColor.maxLuminance) {
      region.color = ScanLineRegion::field;
    }
    else {
      region.color = ScanLineRegion::none;
    }
  }
}

unsigned char ScanLineMiddleInfo::fieldClassificationSaturationThreshold(const InternalRegion& region, float regionHeight, const CameraInfo& cameraInfo, ScanGrid& scanGrid) const
{
  // saturation of very dark field pixels is often 0
  if(region.y > 2) {
    if(cameraInfo.camera == CameraInfo::Camera::top) {
      // reduce saturation threshold for further away regions
      float scalingFactor = ((regionHeight - static_cast<float>(scanGrid.fieldLimit)) / static_cast<float>(cameraInfo.height - scanGrid.fieldLimit) + 1.f) / 1.8f;
      return static_cast<unsigned char>(static_cast<float>(estimatedFieldColor.minSaturation) * scalingFactor);
    }
    return estimatedFieldColor.minSaturation;
  }
  return 0;
}

bool ScanLineMiddleInfo::regionIsField(const InternalRegion* const region,
    const CameraInfo& cameraInfo, RelativeFieldColorsParameters& relativeFieldColorsParameters) const {
  if((cameraInfo.camera == CameraInfo::Camera::bot && region->regionSize > lowerMinRegionSize) ||
      (cameraInfo.camera == CameraInfo::Camera::top && region->regionSize > upperMinRegionSize)) {
    if(region->y <= relativeFieldColorsParameters.maxFieldLuminance &&
        region->s >= relativeFieldColorsParameters.minFieldSaturation &&
        relativeFieldColorsParameters.fieldHue.isInside(region->h)) {
      return true;
    }
  }
  return false;
}

void ScanLineMiddleInfo::classifyWhiteRegionsWithThreshold(std::vector<std::vector<InternalRegion>>& regions,
  RelativeFieldColorsParameters& relativeFieldColorsParameters) const {
  unsigned char minWhiteLuminance = static_cast<unsigned char>(
                                      std::min(std::min(static_cast<int>(estimatedFieldColor.maxLuminance), static_cast<int>(relativeFieldColorsParameters.maxFieldLuminance)),
                                               static_cast<int>(baseLuminance) + luminanceSimilarityThreshold));
  unsigned char maxWhiteSaturation = static_cast<unsigned char>(
                                       std::min(static_cast<int>(relativeFieldColorsParameters.maxWhiteSaturation),
                                                std::max(static_cast<int>(static_cast<float>(relativeFieldColorsParameters.minFieldSaturation) + (static_cast<float>(estimatedFieldColor.minSaturation) -
                                                         static_cast<float>(relativeFieldColorsParameters.minFieldSaturation)) * 0.7f),
                                                         static_cast<int>(static_cast<float>(baseSaturation) * 0.5f) - saturationSimilarityThreshold)));
  for(auto& line : regions) {
    for(auto& region : line) {
      if(region.color != ScanLineRegion::field && region.y > minWhiteLuminance && region.s < maxWhiteSaturation) {
        region.color = ScanLineRegion::white;
      }
    }
  }
}

bool ScanLineMiddleInfo::prelabelWhiteCheck(const InternalRegion& checkedRegion) const {
  return checkedRegion.regionSize <= maxPrelabelRegionSize &&
         checkedRegion.y >= baseLuminance && checkedRegion.y >= baseLuminance + luminanceSimilarityThreshold / 2 &&
         checkedRegion.s <= baseSaturation - saturationSimilarityThreshold / 2;
}

bool ScanLineMiddleInfo::prelabelWhiteNeighborCheck(const InternalRegion& checkedRegion, const InternalRegion& neighborRegion) const {
  int neighborRegionSize = neighborRegion.range.to - neighborRegion.range.from;
  float thresholdModifier = neighborRegionSize >= 10 ? 1 : static_cast<float>(neighborRegionSize) / 10.f;
  return static_cast<int>(checkedRegion.y) > static_cast<int>(neighborRegion.y) &&
         static_cast<int>(checkedRegion.s) < static_cast<int>(neighborRegion.s) &&
         static_cast<int>(checkedRegion.y) + static_cast<int>(neighborRegion.s) >=
         static_cast<int>(neighborRegion.y) + static_cast<int>(checkedRegion.s) +
         static_cast<int>(thresholdModifier * static_cast<float>(luminanceSimilarityThreshold + saturationSimilarityThreshold));
}

void ScanLineMiddleInfo::stitchUpHoles(std::vector<std::vector<InternalRegion>>& regions, bool horizontal) const {
  ScanLineRegion::Color lastColor = ScanLineRegion::none;
  ScanLineRegion::Color currentColor = ScanLineRegion::none;
  size_t lastColorIndex;
  int currentRegionSize;
  for(auto& line : regions) {
    if(line.size() < 3) {
      continue;
    }
    lastColor = line[0].color;
    currentColor = line[1].color;
    lastColorIndex = 0;
    currentRegionSize = line[1].range.to - line[1].range.from;
    for(size_t regionIndex = 2; regionIndex < line.size(); ++regionIndex) {
      if(currentColor == ScanLineRegion::none && currentRegionSize <= maxRegionSizeForStitching) {
        if(line[regionIndex].color == ScanLineRegion::none && regionIndex + 1 < line.size() &&
           currentRegionSize + line[regionIndex].range.to - line[regionIndex].range.from <= maxRegionSizeForStitching) {
          // Successive "none" regions that can be changed together
          currentRegionSize += line[regionIndex].range.to - line[regionIndex].range.from;
          continue;
        }
        if(lastColor == ScanLineRegion::field && line[regionIndex].color == ScanLineRegion::field) {
          for(size_t changeIndex = lastColorIndex + 1; changeIndex < regionIndex; ++changeIndex) {
            line[changeIndex].color = ScanLineRegion::field;
          }
        }
        else if((lastColor == ScanLineRegion::field && line[regionIndex].color == ScanLineRegion::white) ||
                (lastColor == ScanLineRegion::white && line[regionIndex].color == ScanLineRegion::field)) {
          if(horizontal) {
            unsigned short stitchPos = (line[lastColorIndex].range.to + line[regionIndex].range.from + 1) / 2;
            line[lastColorIndex].range.to = stitchPos;
            line[regionIndex].range.from = stitchPos;
          }
          else {
            // vertical
            unsigned short stitchPos = (line[lastColorIndex].range.from + line[regionIndex].range.to) / 2;
            line[lastColorIndex].range.from = stitchPos;
            line[regionIndex].range.to = stitchPos;
          }
          line.erase(line.cbegin() + static_cast<long>(lastColorIndex) + 1, line.cbegin() + static_cast<long>(regionIndex));
          regionIndex = lastColorIndex + 1;
          currentColor = lastColor;
        }
      }
      lastColor = currentColor;
      currentColor = line[regionIndex].color;
      lastColorIndex = regionIndex - 1;
      currentRegionSize = line[regionIndex].range.to - line[regionIndex].range.from;
    }
  }
}

void ScanLineMiddleInfo::emplaceInScanLineRegionsHorizontal(ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal,
    const std::vector<unsigned short>& yPerScanLine, const std::vector<std::vector<InternalRegion>>& regionsPerScanLine) {
  ASSERT(yPerScanLine.size() == regionsPerScanLine.size());
  const std::size_t numOfScanLines = yPerScanLine.size();
  colorScanLineRegionsHorizontal.scanLines.reserve(numOfScanLines);
  for(std::size_t i = 0; i < numOfScanLines; ++i) {
    colorScanLineRegionsHorizontal.scanLines.emplace_back(yPerScanLine[i]);
    const std::vector<InternalRegion>& regions = regionsPerScanLine[i];
    std::vector<ScanLineRegion>& newRegions = colorScanLineRegionsHorizontal.scanLines[i].regions;
    for(std::size_t j = 0; j < regions.size(); ++j) {
      // Merge adjacent regions of the same color.
      if(!j || newRegions.back().color != regions[j].color) {
        newRegions.emplace_back(regions[j].range.from, regions[j].range.to, regions[j].color);
      }
      else {
        newRegions.back().range.to = regions[j].range.to;
      }
    }
  }
}

void ScanLineMiddleInfo::emplaceInScanLineRegionsVertical(ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped,
    const std::vector<unsigned short>& xPerScanLine, const std::vector<std::vector<InternalRegion>>& regionsPerScanLine) {
  ASSERT(xPerScanLine.size() == regionsPerScanLine.size());
  std::size_t numOfScanLines = xPerScanLine.size();
  colorScanLineRegionsVerticalClipped.scanLines.reserve(numOfScanLines);
  for(std::size_t i = 0; i < numOfScanLines; ++i) {
    colorScanLineRegionsVerticalClipped.scanLines.emplace_back(xPerScanLine[i]);
    const std::vector<InternalRegion>& regions = regionsPerScanLine[i];
    std::vector<ScanLineRegion>& newRegions = colorScanLineRegionsVerticalClipped.scanLines[i].regions;
    for(std::size_t j = 0; j < regions.size(); ++j) {
      // Merge adjacent regions of the same color.
      if(!j || newRegions.back().color != regions[j].color) {
        newRegions.emplace_back(regions[j].range.from, regions[j].range.to, regions[j].color);
      }
      else {
        newRegions.back().range.from = regions[j].range.from;
      }
    }
  }
}

bool ScanLineMiddleInfo::isEstimatedFieldColorValid(const CameraImage* cameraImage) const {
  return static_cast<int>(cameraImage->timestamp - estimatedFieldColor.lastSet) <= estimatedFieldColorInvalidationTime;
}

PixelTypes::GrayscaledPixel ScanLineMiddleInfo::getHorizontalRepresentativeValue(const Image<PixelTypes::GrayscaledPixel>& image,
    const unsigned int from, const unsigned int to, const unsigned int y) {
  ASSERT(to > from);
  const unsigned int length = to - from;
  unsigned int sum = 0;
  if(length <= 6) {
    // take every pixel from very small regions
    for(unsigned int x = from; x < to; ++x) {
      sum += image[y][x];
    }
    return static_cast<unsigned char>(sum / length);
  }
  if(length <= 19) {
    // take every second pixel from small to middle-sized regions
    for(unsigned int x = from + 1; x < to - 1; x += 2) { // don't start or stop in the edge of the region
      sum += image[y][x];
    }
    return static_cast<unsigned char>(sum / ((length - 1) / 2));
  }
  else {
    // take every fourth pixel from bigger regions
    for(unsigned int x = from + 2; x < to - 1; x += 4) { // don't start or stop in the edge of the region
      sum += image[y][x];
    }
    // this relies on (length / 4) being floored; expanding to 4*sum/length will lead to false results
    return static_cast<unsigned char>(sum / (length / 4));
  }
}

PixelTypes::HuePixel ScanLineMiddleInfo::getHorizontalRepresentativeHueValue(const Image<PixelTypes::HuePixel>& image,
    unsigned int from, unsigned int to, unsigned int y) {
  ASSERT(to > from);
  const unsigned int length = to - from;
  float hueValue = 0;
  int currentStep = 1;
  if(length <= 6) {
    // take every pixel from very small regions
    for(unsigned int x = from; x < to; ++x, ++currentStep) {
      hueValue = ScanLineMiddleInfo::hueAverage(hueValue, image[y][x], currentStep);
    }
  }
  if(length <= 19) {
    // take every second pixel from small to middle-sized regions
    for(unsigned int x = from + 1; x < to - 1; x += 2, ++currentStep) { // don't start or stop in the edge of the region
      hueValue = ScanLineMiddleInfo::hueAverage(hueValue, image[y][x], currentStep);
    }
  }
  else {
    // take every fourth pixel from bigger regions
    for(unsigned int x = from + 2; x < to - 1; x += 4, ++currentStep) { // don't start or stop in the edge of the region
      hueValue = ScanLineMiddleInfo::hueAverage(hueValue, image[y][x], currentStep);
    }
  }
  return static_cast<unsigned char>(hueValue);
}

PixelTypes::GrayscaledPixel ScanLineMiddleInfo::getVerticalRepresentativeValue(const Image<PixelTypes::GrayscaledPixel>& image,
    const unsigned int x, const unsigned int from, const unsigned int to) {
  ASSERT(to > from);
  const int length = static_cast<int>(to - from);
  unsigned int sum = 0;
  if(length <= 6) {
    // take every pixel from very small regions
    for(unsigned int y = from; y < to; ++y) {
      sum += image[y][x];
    }
    return static_cast<unsigned char>(sum / length);
  }
  if(length <= 19) {
    // take every second pixel from small to middle-sized regions
    for(unsigned int y = from + 1; y < to - 1; y += 2) { // don't start or stop in the edge of the region
      sum += image[y][x];
    }
    return static_cast<unsigned char>(sum / ((length - 1) / 2));
  }
  else {
    // take every forth pixel from sufficiently large regions
    for(unsigned int y = from + 2; y < to - 1; y += 4) { // don't start or stop in the edge of the region
      sum += image[y][x];
    }
    // this relies on (length / 4) being floored, so please don't change to 4*sum/length as that will lead to false results
    return static_cast<unsigned char>(sum / (length / 4));
  }
}

PixelTypes::HuePixel ScanLineMiddleInfo::getVerticalRepresentativeHueValue(const Image<PixelTypes::HuePixel>& image,
    const unsigned int x, const unsigned int from, const unsigned int to) {
  ASSERT(to > from);
  const int length = static_cast<int>(to - from);
  float hueValue = 0;
  int currentStep = 1;
  if(length <= 6) {
    // take every pixel from very small regions
    for(unsigned int y = from; y < to; ++y, ++currentStep) {
      hueValue = ScanLineMiddleInfo::hueAverage(hueValue, image[y][x], currentStep);
    }
  }
  if(length <= 19) {
    // take every second pixel from small to middle-sized regions
    for(unsigned int y = from + 1; y < to - 1; y += 2, ++currentStep) { // don't start or stop in the edge of the region
      hueValue = ScanLineMiddleInfo::hueAverage(hueValue, image[y][x], currentStep);
    }
  }
  else {
    // take every fourth pixel from sufficiently large regions
    for(unsigned int y = from + 2; y < to - 1; y += 4, ++currentStep) { // don't start or stop in the edge of the region
      hueValue = ScanLineMiddleInfo::hueAverage(hueValue, image[y][x], currentStep);
    }
  }
  return static_cast<unsigned char>(hueValue);
}

void ScanLineMiddleInfo::approximateBaseLuminance(RelativeFieldColors& relativeFieldColors) {
  baseLuminance = static_cast<PixelTypes::GrayscaledPixel>(baseLuminanceReduction * relativeFieldColors.averageLuminanceF);
}

void ScanLineMiddleInfo::approximateBaseSaturation(RelativeFieldColors& relativeFieldColors) {
  baseSaturation = relativeFieldColors.averageSaturation;
}

float ScanLineMiddleInfo::hueAverage(float hueValue, const float hueAddition, const int dataPoints) {
  const float hueDiff = hueAddition - hueValue;
  hueValue += hueDiff / static_cast<float>(dataPoints);
  if(hueValue < 0.f)
    hueValue += 256.f;
  if(hueValue >= 256.f)
    hueValue -= 256.f;
  return hueValue;
}