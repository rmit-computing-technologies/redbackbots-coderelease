/**
 * @file IntersectionsMiddleInfo.hpp
 *
 *
 * @author Kevin Dehmlow
 * @author Roman Sablotny
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"
#include "blackboard/Blackboard.hpp"

#include "types/vision/IntersectionCandidates.hpp"
#include "types/vision/Intersections.hpp"
#include "utils/ml/PatchUtilities.hpp"
#include "types/camera/CameraInfo.hpp"
#include "utils/defs/FieldDefinitions.hpp"

#include <CompiledNN.h>

class IntersectionsMiddleInfo : public Detector {
public:
	IntersectionsMiddleInfo(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime);
  	virtual ~IntersectionsMiddleInfo();

	// Resets middle info
	virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
	virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
  	virtual void detect_(const VisionInfoIn* info_in, 
                      	VisionInfoMiddle* info_middle,
                      	VisionInfoOut* info_out);

private:
    void configure(Blackboard* blackboard);

	void detect_(CameraInfo::Camera whichCamera,
                const VisionInfoIn* info_in, 
                VisionInfoMiddle* info_middle,
                VisionInfoOut* info_out);

    /**
     * Validates the intersection type and emplaces the newly found intersection into the IntersectionsPercept.
     * @param Intersections the intersection the intersection is added to.
     * @param IntersectionCandidates::IntersectionCandidate the classified intersection to add.
     */
    void addIntersection(Intersections& intersections, IntersectionCandidates::IntersectionCandidate& intersectionCandidate);

    /** Classifies the intersection candidate with a neural net and returns the predicted type.
     * @param intersectionCandidate the intersection to be classified.
     * @return False if the neural net predicted the given candidate not to be an intersection. True otherwise.
     */
    bool classifyIntersection(IntersectionCandidates::IntersectionCandidate& intersectionCandidate);

    /** enforces that horizontal is +90° of vertical */
    void enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const;

    std::unique_ptr<NeuralNetwork::Model> model;
    NeuralNetwork::CompiledNN network;

    float threshold = 0.8; /**< threshold value for the confidence value of the neural net. If 0, neural net is not used. */
};