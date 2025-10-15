/**
    * @file BallSpotsProvider.h
    * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
    * @author RedBackBots
*/

#pragma once

#include "perception/vision/Detector.hpp"
#include "blackboard/Blackboard.hpp"

#include "types/math/Geometry.hpp"
#include "types/math/Angle.hpp"

#include "types/FieldFeatureInfo.hpp"
#include "types/BallInfo.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/camera/CameraImage.hpp"

#include "types/vision/BallSpots.hpp"
#include "types/vision/PreviousBalls.hpp"
#include "types/vision/ECImage.hpp"
#include "types/vision/RelativeFieldColors.hpp"
#include "types/vision/ColorScanLineRegions.hpp"

#include <CompiledNN.h>

/**
 * @class BallSpotsMiddleInfo
 * A module that provides spots that might be inside balls
 */
class BallSpotsMiddleInfo : public Detector{
public:

  BallSpotsMiddleInfo(Blackboard* blackboard);
  virtual ~BallSpotsMiddleInfo();

  // Resets middle info
  virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
  virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
  virtual void detect_(const VisionInfoIn* info_in, 
                      VisionInfoMiddle* info_middle,
                      VisionInfoOut* info_out);

private:

    void configure(Blackboard* blackboard);
    /**
        * The main method of this module.
        * @param ballSpots The percept that is filled by this module.
    */

    /**
        * Run detection for the given camera
    **/
    void detect_(CameraInfo::Camera whichCamera,
                 const VisionInfoIn* info_in, 
                 VisionInfoMiddle* info_middle,
                 VisionInfoOut* info_out);

    /**
        * The method searches with the help of ColorScanLineRegionsVerticalClipped
        * for initial ball spot and adds them to the list (ballSpot.spots) if no
        * check fails.
        *
        * @param ballSpots The percept that is filled by this module.
    */
    // void searchScanLines(BallSpots& ballSpots, ECImage& theECImage, ColorScanLineRegionsVertical& theColorScanLineRegionsVerticalClipped) const;
    void searchScanLines(BallSpots& ballSpots, CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, const CameraInfo& cameraInfo) const;

    /**
        * The method scans in y-direction to adjust the initial spot guess.
        *
        * @param initialPoint The initial ball spot guess
        * @param circle The guessed ball in image
        * @param luminanceRef The luminance of a white reference Pixel
        * @param saturationRef The saturation of a white reference Pixel
        * @return Is the scanned width not entirely smaller then the calculated one?
        *         And are the pixels right colored in the majority?
    */
    bool correctWithScanLeftAndRight(Vector2i& initialPoint, const Geometry::Circle& circle, unsigned char luminanceRef, unsigned char saturationRef, ECImage& theECImage, RelativeFieldColors& relativeFieldColors) const;

    /**
        * The method performs a scan in one direction.
        *
        * @param spot The current ball spot to work with
        * @param currentLength A variable that counts the scanned pixels
        * @param maxLength The maximum scan length
        * @param goodPixelCounter A variable that counts the accepted pixels
        * @param getX(Vector2i,int) A pointer of a function that calculates the x-Value of the next element
        * @param getY(Vector2i,int) A pointer of a function that calculates the y-Value of the next element
        * @param luminanceRef The luminance of a white reference Pixel
        * @param saturationRef The saturation of a white reference Pixel
    */
    void scanBallSpotOneDirection(const Vector2i& spot, int& currentLength, const int& maxLength,
                                unsigned& goodPixelCounter,
                                int(*getX)(const Vector2i& spot, const int currentLength),
                                int(*getY)(const Vector2i& spot, const int currentLength),
                                unsigned char luminanceRef, unsigned char saturationRef,
                                ECImage& theECImage, RelativeFieldColors& relativeFieldColors) const;

    /**
        * The method checks a pixel.
        *
        * @param pixel The color pixel
        * @param goodPixelCounter A variable that counts the accepted pixels
        * @param currentSkipped A variable that counts the consecutive skipped pixels
        * @param luminanceRef The luminance of a white reference Pixel
        * @param saturationRef The saturation of a white reference Pixel
        * @return Is the consecutive skipped pixel (or green pixel) count to high?
    */
    bool checkPixel(unsigned char pixelLuminance, unsigned char pixelSaturation,
                  unsigned& goodPixelCounter, unsigned& currentSkipped, unsigned char luminanceRef, unsigned char saturationRef,
                  RelativeFieldColors& relativeFieldColors) const;

    /**
        * The method checks if the last spot is duplicative.
        *
        * @param ballSpots The representation with all previously found ball spots
        * @param minAllowedDistance The minimum allowed distance (in pixel) to an other spot
        * @return Is the spot duplicative?
    */
    bool isLastDuplicative(const BallSpots& ballSpots, const int minAllowedDistance) const;

    /**
        * The method checks if the spot is clearly inside a visually detected robot.
        * A spot is not clearly inside a robot if it is somewhere inside the feet
        *  or not above them
        *
        * @param spot The spot to check
        * @param estimatedRadius The calculate radius (in pixel) for a ball at this image position
        * @return Is the spot clearly inside a robot?
    */
    bool isSpotClearlyInsideARobot(const Vector2i& spot, const float estimatedRadius) const;

    /**
        * The method checks if the spot surrounded by green pixels
        *
        * @param spot The spot to check
        * @param radius The calculate radius (in pixel) for a ball at this image position
        * @return Is the spot surrounded by enough green pixel?
    */
    bool checkGreenAround(const Vector2i& spot, const float radius, unsigned char luminanceRef, unsigned char saturationRef, ECImage& ecImage, RelativeFieldColors& relativeFieldColors) const;

    /**
        * The method calculates how a ball would look like if the ball visibility starts at
        * (x,y) and which part of it is probably recognizable.
        *
        * @param x The x-coordinate to calculate with
        * @param y The y-coordinate to calculate with
        * @param circle The circle which describes the possible ball
        * @param info_in
        * @param cameraInfo
        * @return The amount of pixel that must be recognizable above this y-coordinate
        *           if it is a ball.
    */
    float getNeededLengthFor(const int x, const int y, Geometry::Circle& circle, const VisionInfoIn* info_in, const CameraInfo& cameraInfo) const;

    static constexpr float minRadiusOfWantedRegion = 3.0; // if a regions radius is smaller than this, the green check must be successful
    static constexpr Angle greenEdge  = 80_deg; // The greenEdge for IISC::calcPossibleVisibleBallByLowestPoint(..)
    static constexpr float ballSpotDistUsage  = 0.3; // How much of the lower visible part of the ball has to be found
    static constexpr float scanLengthRadiusFactor = 1.2; // The factor to determine the maximum scan length
    static constexpr unsigned maxNumberOfSkippablePixel = 3; // The maximum number of pixels that are allowed to be skipped while scanBallSpotOneDirection(..)
    static constexpr float minAllowedDistanceRadiusRelation = 1.3; // The factor to determine the minimum distance between two ball spots
    static constexpr float minFoundDiameterPercentage = 0.5; // The minimum ratio of the measured width compared to the calculated width
    static constexpr float noiseThreshold = 0.3; // The maximum ratio of non-good pixels
    static constexpr int additionalRadiusForGreenCheck = 2; // The distance between the ball spot and the green scan areas
    static constexpr float greenPercent = 0.9; // The minimum ratio of detected green compared to all considered pixels in the green check
    static constexpr bool allowScanLineTopSpotFitting = false; // Is it allowed to find a spot on top of a scanLine?
    static constexpr bool lessStrictChecks = false; // Allow more Ballspots?
};
