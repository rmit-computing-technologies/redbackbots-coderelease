
/**
 * @file ScanGridMiddleInfo.hpp
 *
 * Runs as a detector to provide middle information.
 * Provides the description of a grid for scanning the image. 
 * The grid resolution adapts to the camera perspective.
 * 
 * @author Thomas Röfer 
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"

#include "blackboard/Blackboard.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/vision/ScanGrid.hpp"
#include "types/field/FieldBoundary.hpp"


class ScanGridMiddleInfo : public Detector {
public:
    ScanGridMiddleInfo(Blackboard* blackboard);
    virtual ~ScanGridMiddleInfo();

    // Resets
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
    virtual void detect_(const VisionInfoIn* info_in, 
                        VisionInfoMiddle* info_middle,
                        VisionInfoOut* info_out);

private:
    struct ImageCornersOnField {
        Vector2f leftOnField;
        Vector2f rightOnField;
        bool valid;
    };

    enum VerticalBoundary {
        UPPER,
        LOWER
    };

    /**
     * Compute the furthest point away that could be part of the field given an unknown own position.
     * @return
     */
    int calcFieldLimit(const VisionInfoIn* info_in, const CameraInfo& cameraInfo) const;

    /**
     * Determine vertical sampling points of the grid.
     * @param scanGrid
     * @param lowerImageCornersOnField
     */
    void setFullResY(ScanGrid& scanGrid, ImageCornersOnField& lowerImageCornersOnField, const VisionInfoIn* info_in, const CameraInfo& cameraInfo) const;

    /**
     * Determine vertical sampling points of the grid.
     * @param scanGrid
     */
    void setLowResHorizontalLines(ScanGrid& scanGrid, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const;

    /**
     *
     * @param boundary Whether to calc upper or lower image border position on field
     * @return
     */
    ImageCornersOnField calcImageCornersOnField(VerticalBoundary boundary, const VisionInfoIn* info_in, const CameraInfo& cameraInfo) const;

    /**
     *
     * @param scanGrid
     * @param lowerImageCornersOnField
     */
    void setVerticalLines(ScanGrid& scanGrid, ImageCornersOnField& lowerImageCornersOnField, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const;

    /**
     *
     * @param scanGrid
     * @param y
     */
    void addLowResHorizontalLine(ScanGrid& scanGrid, int y, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const;

    /**
     * Determine the horizontal scanLine left edge, excluding both areas outside the field and inside the robots body.
     * @param usedY The height of the scan line in the image.
     * @return The x-coordinate of where to start the scan
     */
    int horizontalLeftScanStop(int usedY, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const;

    /**
     * Determine the horizontal scanLine right edge, excluding both areas outside the field and inside the robots body.
     * @param usedY The height of the scan line in the image.
     * @return The x-coordinate of where to stop the scan
     */
    int horizontalRightScanStop(int usedY, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const;

    /**
     * Determines the horizontal left scan start, excluding areas outside the field boundary
     * @param x x-coordinate from where to start searching for a start point.
     * @param y The height of the scan line in the image.
     * @return The x-coordinate of where to start the scan
     */
    int horizontalFieldBoundaryLeftScanStop(int x, int y, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const;

    /**
     * Determines the horizontal right scan stop, excluding areas outside the field boundary
     * @param x-coordinate up to which to search for a stop point.
     * @param y The height of the scan line in the image.
     * @return The x-coordinate of where to stop the scan
     */
    int horizontalFieldBoundaryRightScanStop(int x, int y, FieldBoundary& fieldBoundary) const;

    /**
     * The function determines how far an object is away depending on its real size and the size in the image.
     * @param cameraInfo Information about the camera (opening angles, resolution, etc.).
     * @param sizeInReality The real size of the object.
     * @param sizeInPixels The size in the image.
     * @return The distance between camera and object.
     */
    float getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels) const;

    void configure(Blackboard* blackboard);

    /**
     * Run detection for the given camera
     */
    void detect_(CameraInfo::Camera whichCamera,
                 const VisionInfoIn* info_in, 
                 VisionInfoMiddle* info_middle,
                 VisionInfoOut* info_out);
    
    static constexpr int minVerticalStepSize = 12; /**< The minimum pixel distance between two neighboring vertical scan lines. */
    static constexpr int minHorizontalLowResStepSize = 8; /**< The minimum pixel distance between two neighboring horizontal scan lines. */
    static constexpr int minNumOfLowResScanLines = 25; /**< The minimum number of scan lines for low resolution. */
    static constexpr float lineWidthRatio = 0.9f; /**< The ratio of field line width that is sampled when scanning the image. */
    static constexpr float ballWidthRatio = 0.8f; /**< The ratio of ball width that is sampled when scanning the image. */
};