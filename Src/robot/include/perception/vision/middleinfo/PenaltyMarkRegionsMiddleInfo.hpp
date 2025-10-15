/**
 * @file PenaltyMarkRegionsMiddleInfo.h
 *
 * This file declares a module that determines candidate regions for
 * penalty marks in the image. It provides regions in which should be
 * searched for the center as well as the regions that must be provided
 * in the CNS image for the search to work.
 *
 * @author Thomas Röfer
 * @author RedbackBots
 */
#pragma once

#include "perception/vision/Detector.hpp"

#include "blackboard/Blackboard.hpp"
#include "utils/defs/FieldDefinitions.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/vision/ScanGrid.hpp"
#include "types/vision/ColorScanLineRegions.hpp"
#include "types/vision/PenaltyMarkRegions.hpp"
#include "types/math/Boundary.hpp"

class PenaltyMarkRegionsMiddleInfo : public Detector {
public:
    PenaltyMarkRegionsMiddleInfo(Blackboard* blackboard);
    virtual ~PenaltyMarkRegionsMiddleInfo();

    // Resets
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
    virtual void detect_(const VisionInfoIn* info_in, 
                        VisionInfoMiddle* info_middle,
                        VisionInfoOut* info_out);

private:

    /** A pixel region as part of a union find structure. */
    struct Region
    {
        Region* parent = this; /**< Parent region. If pointing to itself, this is the root. */
        unsigned short upper; /**< Upper border. Inclusive. */
        unsigned short lower; /**< Lower border. Exclusive. */
        unsigned short left; /**< Left border. Inclusive. */
        unsigned short right; /**< Right border. Inclusive. */
        unsigned short pixels; /**< The overall number of pixels in this region. */
        unsigned short whitePixels; /**< The overall number of white pixels in this region. */

        /**
         * Constructor.
         * @param upper The upper border of the region (inclusive).
         * @param lower The lower border of the region (exclusive).
         * @param x The pixel column of this region.
         * @param isWhite It this a white region?
         */
        Region(unsigned short upper, unsigned short lower, unsigned short x, bool isWhite)
        : upper(upper),
            lower(lower),
            left(x),
            right(x + 1),
            pixels(lower - upper),
            whitePixels(isWhite ? pixels : 0) {}

        /**
         * Find the root of the current region (with path compression).
         * @return The root of the current region.
         */
        Region* getRoot()
        {
        Region* r = this;
        while(r != r->parent)
        {
            Region* r2 = r;
            r = r->parent;
            r2->parent = r->parent;
        }
        return r;
        }
    };

    /**
     * Initializes the extendedLower table.
     * @param upperBound The smallest y coordinate that can be expected.
     * @param cameraInfo CameraInfo passed from info_in.
     * @param scanGrid ScanGrid passed from info_middle.
     */
    void initTables(unsigned short upperBound, const CameraInfo& cameraInfo, ScanGrid& scanGrid);

    /**
     * Initializes the scan lines containing the non-green regions.
     * @param upperBound The smallest y coordinate that can be expected.
     * @param colorScanLineRegionsVerticalClipped ColorScanLineRegionsVerticalClipped passed from info_middle.
     * @return Could the regions be initialized? This method returns false
     *         if there would be too many regions. In that case, the image
     *         is probably very noisy.
     */
    bool initRegions(unsigned short upperBound, ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped);

    /**
     * Merges all neighboring regions.
     * @param xStep The distance between neighboring low-res scan lines.
     */
    void unionFind(int xStep);

    /**
     * Analyses the merged regions finding candidates for penalty marks.
     * This method also updates the cnsRegion member.
     * @param upperBound The smallest y coordinate that can be expected.
     * @param xStep The distance between neighboring low-res scan lines.
     * @param searchRegions The regions that should be searched for the center of a penalty mark.
     * @param info_in
     * @param cameraInfo CameraInfo passed from info_in.
     * @param scanGrid ScanGrid passed from info_middle.
     */
    void analyseRegions(unsigned short upperBound, int xStep, std::vector<Boundaryi>& searchRegions, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ScanGrid& scanGrid);

    void configure(Blackboard* blackboard);

    /**
     * Run detection for the given camera
     */
    void detect_(CameraInfo::Camera whichCamera,
                 const VisionInfoIn* info_in, 
                 VisionInfoMiddle* info_middle,
                 VisionInfoOut* info_out);

    // The regions of non-green pixels (left to right, bottom to bottom).
    std::vector<Region> regions;

    // A table that maps y coordinates to y coordinates with region extension.
    std::vector<unsigned short> extendedLower;

    static constexpr float maxDistanceOnField = 3000.0f; // The maximum distance in which penalty marks are detected.
    static constexpr int regionExtensionFactor = 3; // Region heights are extended by this value times the expected line width to better merge diagonal lines.
    static constexpr float sizeToleranceRatio = 0.5f; // Acceptable deviation of the measured size from the expected one.
    static constexpr float minWhiteRatio = 0.9f; // Ratio of pixels that must be white in a candidate region.
    static constexpr int blockSizeX = 16; // Must be the same value as in the CNSRegionProvider.
    static constexpr int blockSizeY = 16; // Must be the same value as in the CNSRegionProvider.
    static constexpr unsigned maxNumberOfRegions = 3; // The maximum number of regions created.
};