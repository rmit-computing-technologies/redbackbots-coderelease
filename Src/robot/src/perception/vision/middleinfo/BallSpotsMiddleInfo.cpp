/**
 * @file BallSpotsProvider.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "perception/vision/middleinfo/BallSpotsMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "utils/math/IISC.hpp"
#include "utils/math/basic_maths.hpp"

#include <cmath>
#include <vector>

BallSpotsMiddleInfo::BallSpotsMiddleInfo(Blackboard* blackboard): 
	Detector("BallSpotsMiddleInfo")
{
	configure(blackboard);

	// NOTE (MP): BHuman had some code here that utilised predictions from other predictions made within last 100ms
	// I am not sure if we want to do this or even what mechanism we would use to do this.

	llog(INFO) << NDEBUG_LOGSYMB << "BallSpotsMiddleInfo loaded" << std::endl;
}

BallSpotsMiddleInfo::~BallSpotsMiddleInfo() {
}

void BallSpotsMiddleInfo::configure(Blackboard* blackboard) {

}

void BallSpotsMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
	info_middle->ballSpots[CameraInfo::Camera::top].ballSpots.clear();
	info_middle->ballSpots[CameraInfo::Camera::top].firstSpotIsPredicted = false;
}

void BallSpotsMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {}

void BallSpotsMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);
}

void BallSpotsMiddleInfo::detect_(CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
	const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
	const CameraImage* cameraImage = info_in->image[whichCamera];
	PreviousBalls& previousBalls = info_middle->previousBalls[whichCamera];

	BallSpots& ballSpots = info_middle->ballSpots[whichCamera];

	// @TODO: add initial prediction based on previous ball model to canidate spots. (See: bhuman update function in BallSpotsProvider.cpp, line: 15-38)
	// Add a prediction based on the previous ball model to the candidates
	if ((cameraImage->timestamp - previousBalls.timestamp) < 100)
	{
		for(const Vector2i& ball : previousBalls.balls)
		{
			ballSpots.firstSpotIsPredicted = true;
			ballSpots.addBallSpot(ball.x(), ball.y());
		}
	}
	
	searchScanLines(ballSpots, whichCamera, info_in, info_middle, cameraInfo);
}

void BallSpotsMiddleInfo::searchScanLines(BallSpots& ballSpots, CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, const CameraInfo& cameraInfo) const {
	ColorScanLineRegionsVertical& colorScanLineRegionsVerticalClipped = info_middle->colorScanLineRegionsVerticalClipped[whichCamera];
	RelativeFieldColors& relativeFieldColors = info_middle->relativeFieldColors[whichCamera];
	ECImage& ecImage = info_middle->ecImage[whichCamera];

	const unsigned step = colorScanLineRegionsVerticalClipped.lowResStep > 1 ? colorScanLineRegionsVerticalClipped.lowResStep / 2 : 1;
  	const unsigned start = colorScanLineRegionsVerticalClipped.lowResStart >= step ? colorScanLineRegionsVerticalClipped.lowResStart - step : colorScanLineRegionsVerticalClipped.lowResStart;
  	Geometry::Circle circle;

	// llog(INFO) << NDEBUG_LOGSYMB << "Step: " << step << " Start: " << start << std::endl;
	// llog(INFO) << NDEBUG_LOGSYMB << "Step: " << step << " Start: " << start << std::endl;
	// llog(INFO) << NDEBUG_LOGSYMB << "Scan Lines Size: " << colorScanLineRegionsVerticalClipped.scanLines.size() << std::endl;
  	for(unsigned scanLineIndex = start; scanLineIndex < colorScanLineRegionsVerticalClipped.scanLines.size(); scanLineIndex += step) {
    	int lowestYOfCurrentArea = 0;
    	int currentLengthNeeded = 0;

    	for(const ScanLineRegion& region : colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].regions) {
      		if (region.color != ScanLineRegion::field) {
				// llog(INFO) << NDEBUG_LOGSYMB << "Current color scan region vertical clip region is field" << std::endl;
        		if(currentLengthNeeded == 0) {
					// llog(INFO) << NDEBUG_LOGSYMB << "Current length needed is 0" << std::endl;
          			lowestYOfCurrentArea = region.range.lower;
          			currentLengthNeeded = static_cast<int>(getNeededLengthFor(colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x, region.range.lower, circle, info_in, cameraInfo));
					// llog(INFO) << NDEBUG_LOGSYMB << "Current length needed now = " << currentLengthNeeded << std::endl;
					// llog(INFO) << NDEBUG_LOGSYMB << "Current lowestYOfCurrentArea needed now = " << lowestYOfCurrentArea << std::endl;
          			if(!currentLengthNeeded) {
						// llog(INFO) << NDEBUG_LOGSYMB << "Enter !currentLengthNeeded" << std::endl;
						break;
					}
            	}
      		} else {
				// llog(INFO) << NDEBUG_LOGSYMB << "Current color scan region vertical clip region is NOT field" << std::endl;
				// llog(INFO) << NDEBUG_LOGSYMB << "Current length needed now = " << currentLengthNeeded << std::endl;
				// llog(INFO) << NDEBUG_LOGSYMB << "Current lowestYOfCurrentArea needed now = " << lowestYOfCurrentArea << std::endl;
        		if (currentLengthNeeded != 0) {
					if (lowestYOfCurrentArea - region.range.lower > currentLengthNeeded) {
						ballSpots.ballSpots.emplace_back(circle.center.cast<int>());
					} else if (lowestYOfCurrentArea == colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].regions.front().range.lower &&
					lowestYOfCurrentArea - region.range.lower > currentLengthNeeded / 2) {
						ballSpots.ballSpots.emplace_back(colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x, (lowestYOfCurrentArea + region.range.lower) / 2);
					} else {

						goto noSpot;
					}
					
          			if (!(isLastDuplicative(ballSpots, static_cast<int>(circle.radius * minAllowedDistanceRadiusRelation))
               			|| isSpotClearlyInsideARobot(ballSpots.ballSpots.back(), circle.radius))) {
            			
						unsigned char luminanceRef = 0;
            			unsigned char saturationRef = 0;
            			int luminanceAverage = 0;

						for (int i = region.range.lower; i < lowestYOfCurrentArea; i++) {
							const unsigned char luminance = ecImage.grayscaled[i][colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x];
							const unsigned char saturation = ecImage.saturated[i][colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x];
							luminanceAverage += luminance;
							if(luminance > luminanceRef) {
								luminanceRef = luminance;
							}
							if(saturation > saturationRef) {
								saturationRef = saturation;
							}
						}

            			luminanceRef = static_cast<unsigned char>((static_cast<int>(luminanceAverage / (lowestYOfCurrentArea - region.range.lower)) + (lessStrictChecks ? 0.f : luminanceRef)) / 2);
            			saturationRef = static_cast<unsigned char>((static_cast<int>(luminanceAverage / (lowestYOfCurrentArea - region.range.lower)) + (lessStrictChecks ? 0.f : saturationRef)) / 2);
						if (!correctWithScanLeftAndRight(ballSpots.ballSpots.back(), circle, luminanceRef, saturationRef, ecImage, relativeFieldColors)
               				|| (currentLengthNeeded < minRadiusOfWantedRegion && !checkGreenAround(ballSpots.ballSpots.back(), circle.radius, luminanceRef, saturationRef, ecImage, relativeFieldColors))
               				|| isSpotClearlyInsideARobot(ballSpots.ballSpots.back(), circle.radius))
						{
							ballSpots.ballSpots.pop_back();
            			}
          			}
          			else {
            			ballSpots.ballSpots.pop_back();
          			}
        		}

      			noSpot:
        		lowestYOfCurrentArea = 0;
        		currentLengthNeeded = 0;
      		}
    	}

		// allowScanLineTopSpotFitting is False, this will never be run.
		// That means if the scan line region is a field, then no spot will be added
    	if (allowScanLineTopSpotFitting  
       		&& currentLengthNeeded > SQUARE(minRadiusOfWantedRegion)
       		&& lowestYOfCurrentArea - colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].regions.back().range.upper > std::sqrt(currentLengthNeeded)
       		&& !isSpotClearlyInsideARobot(circle.center.cast<int>(), circle.radius))
    	{
      		ballSpots.ballSpots.emplace_back(circle.center.cast<int>());

      		if(isLastDuplicative(ballSpots, static_cast<int>(circle.radius * minAllowedDistanceRadiusRelation))) {
				ballSpots.ballSpots.pop_back();
			}
    	}
  	}

	// if (ballSpots.ballSpots.size()  > 0) {
		// llog(INFO) << NDEBUG_LOGSYMB << "camera: " << whichCamera << std::endl;
	// llog(INFO) << NDEBUG_LOGSYMB << "Number of BallSpots: " << ballSpots.ballSpots.size() << std::endl;
	// }
}

bool BallSpotsMiddleInfo::correctWithScanLeftAndRight(Vector2i& initialPoint, const Geometry::Circle& circle, unsigned char luminanceRef, unsigned char saturationRef, ECImage& ecImage, RelativeFieldColors& relativeFieldColors) const
{	
  	const int maxScanLength = static_cast<int>(circle.radius * scanLengthRadiusFactor);

  	int leftMaximum(0), rightMaximum(ecImage.grayscaled.width);

	// @TODO: Implement BodyContour
  	// theBodyContour.clipLeft(leftMaximum, initialPoint.y());
  	// theBodyContour.clipRight(rightMaximum, initialPoint.y());

  	const int maxLeftScanLength = std::min(maxScanLength, initialPoint.x() - leftMaximum);
  	const int maxRightScanLength = std::min(maxScanLength, rightMaximum - initialPoint.x());

  	unsigned foundGoodPixel = 0;
  	int leftScanLength = 0;
  	scanBallSpotOneDirection(initialPoint, leftScanLength, maxLeftScanLength, foundGoodPixel,
  	[](const Vector2i& spot, const int currentLength) {return int(spot.x() - currentLength); },
  	[](const Vector2i& spot, const int) {return int(spot.y()); },
  	luminanceRef, saturationRef, ecImage, relativeFieldColors);

  	int rightScanLength = 0;
  	scanBallSpotOneDirection(initialPoint, rightScanLength, maxRightScanLength, foundGoodPixel,
  	[](const Vector2i& spot, const int currentLength) {return int(spot.x() + currentLength); },
  	[](const Vector2i& spot, const int) {return int(spot.y()); },
  	luminanceRef, saturationRef, ecImage, relativeFieldColors);

  	initialPoint.x() -= (leftScanLength - rightScanLength) / 2;
  	const float noise = 1.f - static_cast<float>(foundGoodPixel) / static_cast<float>(leftScanLength + rightScanLength);
  	const float foundDiameterPercentage = static_cast<float>(leftScanLength + rightScanLength) / static_cast<float>(2 * circle.radius);

  	return noise < noiseThreshold && foundDiameterPercentage > minFoundDiameterPercentage;
}

void BallSpotsMiddleInfo::scanBallSpotOneDirection(const Vector2i& spot, int& currentLength, const int& maxLength,
                                                       unsigned& goodPixelCounter,
                                                       int(*getX)(const Vector2i& spot, const int currentLength),
                                                       int(*getY)(const Vector2i& spot, const int currentLength),
                                                       unsigned char luminanceRef, unsigned char saturationRef, 
													   ECImage& ecImage, RelativeFieldColors& relativeFieldColors) const
{
  	unsigned currentSkipped = 0;

  	while (checkPixel(ecImage.grayscaled[getY(spot, currentLength)][getX(spot, currentLength)], ecImage.saturated[getY(spot, currentLength)][getX(spot, currentLength)], goodPixelCounter, currentSkipped, luminanceRef, saturationRef, relativeFieldColors)
        && ++currentLength <= maxLength);
	currentLength -= currentSkipped;
}

bool BallSpotsMiddleInfo::checkPixel(unsigned char pixelLuminance, unsigned char pixelSaturation, unsigned& goodPixelCounter, unsigned& currentSkipped, unsigned char luminanceRef, unsigned char saturationRef, RelativeFieldColors& relativeFieldColors) const
{
  	if (!relativeFieldColors.isFieldNearWhite(pixelLuminance, pixelSaturation, luminanceRef, saturationRef)) {
    	currentSkipped = 0;
    	++goodPixelCounter;
  	}
  	else {
		++currentSkipped;
	}
    	
  	return currentSkipped < maxNumberOfSkippablePixel;
}

bool BallSpotsMiddleInfo::isLastDuplicative(const BallSpots& ballSpots, const int minAllowedDistance) const
{
  	if (ballSpots.ballSpots.size() < 2) {
		return false;
  	}
    
  	const int squaredAllowedDistance = SQUARE(minAllowedDistance);
  	const Vector2i& spotToCheck = ballSpots.ballSpots.back();
  	for (auto ptr = ballSpots.ballSpots.begin(); ptr < ballSpots.ballSpots.end() - 1; ptr++) {
		if ((*ptr - spotToCheck).squaredNorm() < squaredAllowedDistance) {
			return true;
		}
	}

  	return false;
}

bool BallSpotsMiddleInfo::checkGreenAround(const Vector2i& spot, const float radius, unsigned char luminanceRef, unsigned char saturationRef, ECImage& ecImage, RelativeFieldColors& relativeFieldColors) const
{

  	int useRadius = additionalRadiusForGreenCheck + static_cast<int>(radius);
	if (useRadius >= spot.x() - 1 || useRadius >= spot.y() - 1 ||
    	spot.x() + 1 + useRadius >= static_cast<int>(ecImage.grayscaled.width) ||
    	spot.y() + 1 + useRadius >= static_cast<int>(ecImage.grayscaled.height)) {
		return false;
	}
    
  	int count(0);
  	const int lastX = spot.x() + useRadius - 1;

	const int y1 = spot.y() - useRadius;
	const int y12 = spot.y() - useRadius - 1;
	const int y2 = spot.y() + useRadius;
	const int y22 = spot.y() + useRadius + 1;

  	for (int x = spot.x() - useRadius + 1; x <= lastX; x++) {
		if (relativeFieldColors.isFieldNearWhite(ecImage.grayscaled[y1][x], ecImage.saturated[y1][x], luminanceRef, saturationRef)) {
			count++;
		}
	}

  	for (int x = spot.x() - useRadius + 1; x <= lastX; x++) {
    	if (relativeFieldColors.isFieldNearWhite(ecImage.grayscaled[y12][x], ecImage.saturated[y12][x], luminanceRef, saturationRef)) {
			count++;
		}
      		
  	}

  	for(int x = spot.x() - useRadius + 1; x <= lastX; x++) {
    	if(relativeFieldColors.isFieldNearWhite(ecImage.grayscaled[y2][x], ecImage.saturated[y2][x], luminanceRef, saturationRef)) {
			count++;
		}
  	}

  	for (int x = spot.x() - useRadius + 1; x <= lastX; x++) {
    	if (relativeFieldColors.isFieldNearWhite(ecImage.grayscaled[y22][x], ecImage.saturated[y22][x], luminanceRef, saturationRef)) {
			count++;
		}
  	}

	const int lastY = spot.y() + useRadius - 1;

	const int x1 = spot.x() - useRadius;
	const int x12 = spot.x() - useRadius;
	const int x2 = spot.x() + useRadius;
	const int x22 = spot.x() + useRadius;

	for (int y = spot.y() - useRadius + 1; y <= lastY; y++) {
		if (relativeFieldColors.isFieldNearWhite(ecImage.grayscaled[y][x1], ecImage.saturated[y][x1], luminanceRef, saturationRef)) {
			count++;
		}
	}

  	for(int y = spot.y() - useRadius + 1; y <= lastY; y++) {
    	if (relativeFieldColors.isFieldNearWhite(ecImage.grayscaled[y][x12], ecImage.saturated[y][x12], luminanceRef, saturationRef)) {
			count++;
		}
  	}

  	for (int y = spot.y() - useRadius + 1; y <= lastY; y++) {
    	if (relativeFieldColors.isFieldNearWhite(ecImage.grayscaled[y][x2], ecImage.saturated[y][x2], luminanceRef, saturationRef)) {
			count++;
		}
	}

  	for (int y = spot.y() - useRadius + 1; y <= lastY; y++) {
    	if (relativeFieldColors.isFieldNearWhite(ecImage.grayscaled[y][x22], ecImage.saturated[y][x22], luminanceRef, saturationRef)) {
			count++;
		}
    }

	const int allPixel = 8 * (2 * useRadius - 1);
	const float percentGreen = static_cast<float>(count) / static_cast<int>(allPixel);

	return percentGreen > greenPercent;
}

bool BallSpotsMiddleInfo::isSpotClearlyInsideARobot(const Vector2i& spot, const float estimatedRadius) const {
	// @TODO (bk): seems to be not fully implemented
	
	// for(const ObstaclesImagePercept::Obstacle& obstacle : theObstaclesImagePercept.obstacles)
	//   if(spot.x() > obstacle.left && spot.x() < obstacle.right
	//      && spot.y() < obstacle.bottom - estimatedRadius - IISC::getImageLineDiameterByLowestPoint(spot.cast<float>(), theCameraInfo, theCameraMatrix, theFieldDimensions))
	//     return true;

  return false;
}

float BallSpotsMiddleInfo::getNeededLengthFor(const int x, const int y, Geometry::Circle& circle, const VisionInfoIn* info_in, const CameraInfo& cameraInfo) const {
	const Vector2f startPoint = Vector2i(x, y).cast<float>();
  	if (!IISC::calcPossibleVisibleBallByLowestPoint(startPoint, circle, info_in, cameraInfo, greenEdge)) {
		return 0;
	} else {
		return circle.radius + (startPoint - circle.center).norm() * ballSpotDistUsage;
	}
}