/**
 * @file IntersectionsMiddleInfo.cpp
 *
 *
 * @author Kevin Dehmlow
 * @author Roman Sablotny
 * @author RedbackBots
 */

#include "perception/vision/middleinfo/IntersectionsMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "types/math/Geometry.hpp"
#include "utils/debug/Assert.hpp"
#include "utils/Logger.hpp"
#include "utils/math/basic_maths.hpp"

IntersectionsMiddleInfo::IntersectionsMiddleInfo(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime):
  Detector("IntersectionsMiddleInfo"),
  network(jitRuntime),
  model(nullptr)
{
    configure(blackboard);

    std::string modelDir = blackboard->config["vision.ml.modeldir"].as<std::string>();
    std::string modelFile = modelDir + "/Intersections/distanceUpdatedModel.h5";
    llog(INFO) << NDEBUG_LOGSYMB << "Intersections Loading Model from file " << modelFile << std::endl;

    // Initialize model for the neural net
    model = std::make_unique<NeuralNetwork::Model>(modelFile);
    network.compile(*model);

    llog(INFO) << NDEBUG_LOGSYMB << "IntersectionsMiddleInfo loaded" << std::endl;
}

IntersectionsMiddleInfo::~IntersectionsMiddleInfo() {
    model = nullptr;
}

void IntersectionsMiddleInfo::configure(Blackboard* blackboard) {

}

void IntersectionsMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    info_middle->intersections[CameraInfo::Camera::top].intersections.clear();
    info_middle->intersections[CameraInfo::Camera::bot].intersections.clear();
}

void IntersectionsMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void IntersectionsMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "tick Intersections" << std::endl;

    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void IntersectionsMiddleInfo::detect_(CameraInfo::Camera whichCamera,
 const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {

  const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
  IntersectionCandidates& intersectionCandidates = info_middle->intersectionCandidates[whichCamera];
  
  Intersections& intersections = info_middle->intersections[whichCamera];

  // check whether network has been successfully compiled
  if(!network.valid()) {
    return;
  }

  for(IntersectionCandidates::IntersectionCandidate intersectionCandidate : intersectionCandidates.intersections)
  {
    if(!classifyIntersection(intersectionCandidate)) {
      continue;
    }

    // change attributes dependent on intersection type
    switch(intersectionCandidate.type) {
      case Intersections::Intersection::L:
        intersectionCandidate.dir1 = intersectionCandidate.line1FurtherEnd - intersectionCandidate.pos;
        intersectionCandidate.dir2 = intersectionCandidate.line2FurtherEnd - intersectionCandidate.pos;
        break;
      case Intersections::Intersection::X:
        break;
      case Intersections::Intersection::T:
        const bool lineIsEnd = (intersectionCandidate.line1CloserEnd - intersectionCandidate.pos).squaredNorm() < SQUARE(4 * FIELD_LINE_WIDTH);
        const bool line2IsEnd = (intersectionCandidate.line2CloserEnd - intersectionCandidate.pos).squaredNorm() < SQUARE(4 * FIELD_LINE_WIDTH);

        // which line ends in intersection
        if(lineIsEnd && !line2IsEnd) { //line is vertical
          Vector2f vertical = intersectionCandidate.line1FurtherEnd - intersectionCandidate.pos;
          Vector2f horizontal = intersectionCandidate.line2FurtherEnd - intersectionCandidate.pos;
          enforceTIntersectionDirections(vertical, horizontal);
          intersectionCandidate.dir1 = vertical;
          intersectionCandidate.dir2 = horizontal;
        }
        else { //line is horizontal
          Vector2f vertical = intersectionCandidate.line2FurtherEnd - intersectionCandidate.pos;
          Vector2f horizontal = intersectionCandidate.line1FurtherEnd - intersectionCandidate.pos;
          enforceTIntersectionDirections(vertical, horizontal);
          intersectionCandidate.dir1 = vertical;
          intersectionCandidate.dir2 = horizontal;

          const unsigned new2Index = intersectionCandidate.line1Index;
          intersectionCandidate.line1Index = intersectionCandidate.line2Index;
          intersectionCandidate.line2Index = new2Index;
        }
    }
    addIntersection(intersections, intersectionCandidate);
  }
}

void IntersectionsMiddleInfo::addIntersection(Intersections& intersections, IntersectionCandidates::IntersectionCandidate& intersectionCandidate)
{
  intersections.intersections.emplace_back(intersectionCandidate.type, intersectionCandidate.pos, intersectionCandidate.img, intersectionCandidate.dir1, intersectionCandidate.dir2, intersectionCandidate.line1Index, intersectionCandidate.line2Index);
}

bool IntersectionsMiddleInfo::classifyIntersection(IntersectionCandidates::IntersectionCandidate& intersectionCandidate)
{
  const unsigned patchSize = intersectionCandidate.imagePatch.height;
  PatchUtilities::extractPatch(Vector2i(patchSize/2, patchSize/2), Vector2i(patchSize, patchSize), Vector2i(patchSize, patchSize), intersectionCandidate.imagePatch, network.input(0).data());
  *(network.input(1).data()) = intersectionCandidate.distance;
  ASSERT(network.input(1).rank() == 1);

  network.apply();
  float l = network.output(0)[0];
  float none = network.output(0)[1];
  float t = network.output(0)[2];
  float x = network.output(0)[3];

  if(none >= threshold + 0.1f) {
    return false;
  }
  if(l >= threshold) {
    intersectionCandidate.type = Intersections::Intersection::L;
    return true;
  }
  if(t >= threshold) {
    intersectionCandidate.type = Intersections::Intersection::T;
    return true;
  }
  // We want to be especially sure before we classify an intersection as x.
  if(x >= threshold + 0.1f) {
    intersectionCandidate.type = Intersections::Intersection::X;
    return true;
  }

  // If no prediction passes the threshold, take the original prediction by the IntersectionCandidatesMiddleInfo.
  return true;
}

void IntersectionsMiddleInfo::enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const
{
  Vector2f vertical90 = vertical;
  vertical90.rotate(M_PI_2); //vertical rotated by +90°
  if(vertical90.angleTo(horizontal) > M_PI_2) {
    horizontal.mirror();
  }
}