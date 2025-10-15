
/**
 * @file FieldBoundaryDetector.hpp
 *
 * This file declares a module that calculates the field boundary
 * using a deep neural network (and subsequent line fitting).
 *
 * @author Arne Hasselbring
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"

#include "blackboard/Blackboard.hpp"
#include "types/geometry/Line.hpp"
#include "types/math/Eigen.hpp"

#include <CompiledNN.h>

/**
 * Detects the Field Boundary
*/
class FieldBoundaryDetector : public Detector {
public:
    FieldBoundaryDetector(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime);
    virtual ~FieldBoundaryDetector();

    // Resets
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
    virtual void detect_(const VisionInfoIn* info_in, 
                        VisionInfoMiddle* info_middle,
                        VisionInfoOut* info_out);

private:
    // A boundary spot both in image and field coordinates.
    struct Spot {
        // The spot in image coordinates.
        Vector2i inImage;

        // The spot in robot-relative field coordinates.
        Vector2f onField;

        // Uncertainty of the network for this spot 0 if the network do not provide it
        float uncertanty;

        Spot() = default;
        Spot(const Vector2i& inImage, const Vector2f& onField, const float u);
    };

    void configure(Blackboard* blackboard);

    /**
     * Run detection for the given camera
     */
    void detect_(CameraInfo::Camera whichCamera,
                 const VisionInfoIn* info_in, 
                 VisionInfoMiddle* info_middle,
                 VisionInfoOut* info_out);

    /**
     * Predict the spots using the provided network and camera info, etc.
    */
    void predictSpots(NeuralNetwork::CompiledNN& network, Vector2i& patchSize,
                      const CameraImage* cameraImage, const CameraInfo& cameraInfo,
                      const VisionInfoIn* info_in,
                      std::vector<Spot>& spots);

    /**
     * Checks if the calculated boundary spots is odd/not good.
     * @param spots The spots that are validated.
     * @return True if the boundary spots seem odd.
     */
    bool boundaryIsOdd(const std::vector<Spot>& spots) const;

    /**
     * Predict where the previous field boundary will be in the current image.
     */
    void projectPrevious(FieldBoundary& fieldBoundary, FieldBoundary& otherFieldBoundary,
                         const CameraInfo& cameraInfo,
                         const VisionInfoIn* info_in);

    /**
     * Validate the spots and, if valid, store the spots in the field boundary
     * 
     * @param fieldBoundary 
     * @param spots 
     */
    void validatePrediction(FieldBoundary& fieldBoundary, std::vector<Spot>& spots);

    /**
     * 
     * Return a weighted, squared, and saturated error between boundary spots and a
     * boundary line.
     * @param error The vertical pixel offset between spot and line. Positive if the
     *              spot is above the line.
     */
    // int effectiveError(int error) const {
    //     return std::min(sqr(error), maxSquaredError) * (error < 0 ? 1 : spotAbovePenaltyFactor);
    // }

    // The model of the neural network, separately for upper and lower camera
    std::unique_ptr<NeuralNetwork::Model> modelUpper;
    std::unique_ptr<NeuralNetwork::Model> modelLower;

    // The compiled neural network.
    NeuralNetwork::CompiledNN networkUpper;
    NeuralNetwork::CompiledNN networkLower;

    // The width and height of the neural network input image.    
    Vector2i patchSizeUpper;
    Vector2i patchSizeLower;

    /** Settings - make non-constexpr if configuration is required */
    // Boundary spots closer than this distance will be ignored (in mm). 
    static constexpr float minDistance = 100.f;
    static constexpr float minDistanceSqr = minDistance * minDistance;

    // The minimum number of valid spots to calculate a field boundary. 
    static constexpr unsigned minNumberOfSpots = 10;

    // threshold to determine weather the boundary is smooth enough 
    static constexpr float threshold = 2.f;

    //how many points must not be at the top of the image in relation to the total number of points 
    static constexpr float nonTopPoints = 0.1;

    // to which pixel points are considered as at the top
    static constexpr int top = 4;

    // maximum average uncertainty of the non top spots to be not considered as odd 
    static constexpr float uncertaintyLimit = 6;

    // how much the points are allowed to be below the lower end on average
    static constexpr int maxPointsUnderBorder = 2;
};